// SPDX-License-Identifier: BSD-2-Clause-Patent
#include <AsmMacroLib.h>

ASM_FUNC(DarwinCurrentEl)
  mrs x0, CurrentEL
  ret

// Paints one 1/16 horizontal band of the framebuffer.
// x19 = framebuffer PA (0 = none), x20 = framebuffer size in bytes.
// band selects the slice from the top; lo/hi are the halves of the BGRA word.
.macro PAINT band, lo, hi, done, loop
  cbz x19, \done\()f
  lsr x9, x20, #4
  bic x9, x9, #7
  mov x11, #\band
  madd x11, x11, x9, x19
  movz w10, #\lo
  movk w10, #\hi, lsl #16
  mov x12, x9
\loop:
  stp w10, w10, [x11], #8
  subs x12, x12, #8
  b.hi \loop\()b
  dmb sy
\done:
.endm

// x0=boot_args PA, x1=entry PA, x2=owned PA, x3=owned size, x4=frame buffer
// PA (0 for none), x5=its size in bytes, x6=bytes actually loaded.
// Bands 0/6 are entry checkpoints, 13 is the AT scan verdict (green = every
// owned page is mapped, red = an unmapped page exists), 7..12 track the cache
// clean of the loaded range only, 4/5/3 cover MMU shutdown, and bands 1/2
// belong to the kernel entry probe.
ASM_FUNC(DarwinEnter)
  msr daifset, #15
  mov x19, x4
  mov x20, x5
  // Band 0 green: reached DarwinEnter, MMU still on.
  PAINT 0, 0xFF00, 0xFF00, 10, 11
  // Band 6 red: the paint stores drained past their barrier.
  PAINT 6, 0x0000, 0xFFFF, 12, 13
  // The UEFI identity map may not cover the whole allocated window, and
  // dc civac on an unmapped page aborts silently. Probe every 4 KiB with
  // AT, which reports the fault in PAR_EL1 instead of taking it.
  add x5, x2, x3
  mov x22, x2
1:
  at s1e1r, x22
  isb
  mrs x23, par_el1
  tbnz x23, #0, 8f
  add x22, x22, #0x1000
  cmp x22, x5
  b.lo 1b
  // Band 13 green: the whole window is mapped.
  PAINT 13, 0xFF00, 0xFF00, 9, 14
  b 15f
8:
  // Band 13 red: at least one window page is unmapped.
  PAINT 13, 0x0000, 0xFFFF, 16, 17
15:
  // Clean only the loaded prefix; that is the memory the kernel reads with
  // caches off, and it lies inside the area the loader wrote with the MMU on.
  add x5, x2, x6
  mrs x6, ctr_el0
  ubfx x6, x6, #16, #4
  mov x7, #4
  lsl x7, x7, x6
  sub x8, x7, #1
  bic x4, x2, x8
  sub x14, x5, x4
  lsr x15, x14, #2
  bic x15, x15, x8
  add x16, x15, x15
  add x16, x16, x4
  add x17, x16, x15
  add x15, x15, x4
  // Band 7 purple: loop bounds ready, before any cache maintenance.
  PAINT 7, 0x0080, 0xFF80, 18, 19
  dc civac, x4
  add x4, x4, x7
  // Band 8 white: one line cleaned.
  PAINT 8, 0xFFFF, 0xFFFF, 20, 21
2:
  dc civac, x4
  add x4, x4, x7
  cmp x4, x15
  b.ne 3f
  // Band 9 gray: 25% cleaned.
  PAINT 9, 0x8080, 0xFF80, 22, 23
3:
  cmp x4, x16
  b.ne 4f
  // Band 10 teal: 50% cleaned.
  PAINT 10, 0x8080, 0xFF00, 24, 25
4:
  cmp x4, x17
  b.ne 5f
  // Band 11 maroon: 75% cleaned.
  PAINT 11, 0x0000, 0xFF80, 26, 27
5:
  cmp x4, x5
  b.lo 2b
  dsb sy
  // Band 12 navy: clean loop and its DSB completed.
  PAINT 12, 0x0080, 0xFF00, 28, 29
  ic iallu
  dsb sy
  isb
  // Band 4 yellow: cache maintenance finished, MMU still on.
  PAINT 4, 0xFF00, 0xFFFF, 30, 31
  mrs x4, sctlr_el1
  bic x4, x4, #1
  bic x4, x4, #4
  bic x4, x4, #0x1000
  msr sctlr_el1, x4
  isb
  // Band 5 orange: SCTLR written with M=0, running with the MMU off.
  PAINT 5, 0xA500, 0xFFFF, 32, 33
  tlbi vmalle1
  dsb sy
  isb
  // Band 3 cyan: TLBI done, about to branch to the kernel entry point.
  PAINT 3, 0xFFFF, 0xFF00, 34, 35
  br x1
