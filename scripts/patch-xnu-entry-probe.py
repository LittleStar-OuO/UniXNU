#!/usr/bin/env python3
"""Put boot-progress probes into a COPY of the source XNU kernel (never the original).

There is no serial console on the device and the kernel cannot draw through its
own console this early, so progress is reported by side channels. The probe code
lives in the zero padding after the kernel entry (_start is one `b`) and
displaced instructions are re-executed there:

  stage 3 (no --reset-after)            right after the MMU is on, waits
                                        --reset-delay seconds, then PSCI
                                        SYSTEM_RESET. A reboot by itself after
                                        that time shows the CPU got through the
                                        MMU enable; the frame buffer is not
                                        mapped by then, so time is the signal.
  stage 4 (--reset-after SYMBOL)        the same wait and reset, but placed after
                                        the call to SYMBOL in --in FUNCTION (the
                                        Nth call, --occurrence). It replaces
                                        stage 3, so one run asks one question:
                                        did the kernel get past that call?
  panic screen (--panic-screen)         panic() and its common panic funnel draw
                                        their format string on the
                                        kernel's own frame buffer mapping and
                                        stops (scripts/xnu-panic-screen.c).
                                        --no-reset leaves out stages 3 and 4.
  ZALLOC progress (--zalloc-progress)   a trampoline on the call that runs each
                                        startup entry; during the ZALLOC phase it
                                        paints one green mark per entry (768 in a
                                        128 x 6 grid), with a blue helper-ok mark
                                        before the first. Where the green stops
                                        is the entry that hung. A self-test
                                        hooked on the _vm_fault_init call just
                                        before the phase maps one frame buffer
                                        window with ml_map_high_window
                                        (writeback, VREGION1), paints a white bar
                                        through it and leaves the mapping in the
                                        first startup entry's arg slot for the
                                        trampoline. The marks sit below every
                                        DarwinBoot band; no full-width color
                                        bands are painted any more.
  kalloc fixed stages (--kalloc-fixed-stages)
                                        one mark per call inside
                                        kalloc_type_view_init_fixed (yellow),
                                        kalloc_type_create_zones_fixed (cyan)
                                        and kalloc_type_create_zone_for_size
                                        (magenta), plus the tail branch into
                                        the zone creation; every mark is
                                        painted before its call.
  parse stages (--parse-stages)         window self-test plus marks inside
                                        kalloc_type_view_parse and the view
                                        copy loop: the xnu view copy
                                        (yellow -> green), the prelink and
                                        __macho_list lookups, one cell per
                                        prelinked kext (yellow scanned, cyan
                                        copy entered, green copy returned),
                                        a per-32-view progress bar, and a red
                                        mark when panic() is entered.
  fastboot on reset (--fastboot-on-reset)
                                        the self-test also maps the SoC watchdog
                                        syscon and writes the device tree's
                                        syscon-reboot-mode value for bootloader
                                        (watchdog@1c007000, offset 0x24, mask
                                        0xf, value 3). Preloader/LK turn that
                                        into fastboot on the next boot, so the
                                        reset probe returns the tablet to
                                        fastboot on its own. Needs
                                        --reset-after and --zalloc-progress.
  fastboot after parse (--fastboot-after-parse)
                                        with --parse-memory: once the late
                                        rows are painted, map the watchdog,
                                        write the same reboot-mode value and
                                        PSCI-reset after --fastboot-delay
                                        seconds. A green cell marks the write,
                                        a red one a failed map; needs
                                        --parse-stages and --parse-memory.
  entry check (--entry-check)           the entry `b` is redirected to a cave
                                        that verifies the boot window still
                                        holds the file's bytes at three known
                                        qwords (kext[0]'s __kalloc_type record
                                        and a __PLK_TEXT_EXEC canary). On a
                                        match it paints a green bar and
                                        continues into XNU; on a mismatch it
                                        paints a red bar, arms the reboot-mode
                                        syscon and PSCI-resets into fastboot
                                        after --entry-check-delay seconds.
                                        Needs --no-reset.
  entry report (--entry-report)         like --entry-check, but one 200x48
                                        status cell per condition (virtBase,
                                        physBase alignment, view name pointer,
                                        view flags, canary) plus 64-bit white/
                                        black rows of the raw and xor values,
                                        and the wait before the fastboot reset
                                        is (60 + 20*first-failing-check) *
                                        cntfrq_el0, so the time from reboot to
                                        fastboot identifies the failing check
                                        without a photo. Needs --no-reset.

Every patch site is checked against its expected original instruction first,
and the tool refuses a kernel that does not match. Offsets are specific to the
kernel built by build-xnu-4k-baseline.sh.
"""
import argparse
import os
import shutil
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

ENTRY_VA = 0xFFFFFFF00B9D0488        # LC_UNIXTHREAD pc, `b start_first_cpu`
START_FIRST_CPU = 0xFFFFFFF00B9D205C
MMU_ENABLE_VA = 0xFFFFFFF00B9D2330   # msr SCTLR_EL1, x0
AFTER_MMU_VA = 0xFFFFFFF00B9D2350    # mov x0, #0x300000
PANIC_VA = 0xFFFFFFF00BF69B04      # _panic
VINFO_VA = 0xFFFFFFF00C0A2970      # _vinfo
PANIC_CAVE_END = 0xFFFFFFF00B9D0FF0
CAVE_SIZE = 0x60
CAVES = {3: 0xFFFFFFF00B9D05C0, 4: 0xFFFFFFF00B9D0620}
TIMEBASE = 19200000
FRAMEBUFFER = 0xFCF2B000               # LK scans out 1600x2560x32 from here
FRAMEBUFFER_STRIDE = 6400
WINDOW_PA = 0xFD600000                 # mapped by ml_map_high_window; rows 1119-1323
WINDOW_LEN = 0x140000
WINDOW_OFF = WINDOW_PA - FRAMEBUFFER
WHITE_X, WHITE_Y, WHITE_W, WHITE_H = 32, 1130, 800, 16   # self-test bar
GATE_X, GATE_Y, GATE_W, GATE_H = 32, 1158, 400, 16       # helper-ok mark
GRID_X, GRID_Y = 32, 1200                                # progress grid origin
GRID_COLUMNS, GRID_ROWS = 128, 6                         # one mark per entry
CELL_W, CELL_H = 8, 8                                    # pixels per mark
CELL_PITCH_X, CELL_PITCH_Y = 10, 14                      # pixels
STAGE_X, STAGE_Y = 32, 1300                              # kalloc_init step marks
STAGE_PITCH_X = 20
ZSTAGE_X, ZSTAGE_Y = 32, 1284                            # kalloc_zone_init loop marks
ZSTAGE_PITCH_X = 24
FIXEDSTAGE_X, FIXEDSTAGE_Y = 32, 1316                    # kalloc_type_view_init_fixed marks
FIXEDSTAGE_PITCH_X = 20
PARSE_X, PARSE_Y = 32, 1300                              # kalloc_type_view_parse marks
PARSE_PITCH_X = 20
VIEWBAR_X, VIEWBAR_Y = 32, 1284                          # per-view progress bar
VIEWBAR_PITCH_X = 12
VIEWBAR_CELLS = 64
KEXT_X, KEXT_Y = 32, 1316                                # one cell per prelinked kext
KEXT_PITCH_X = 12
KEXT_CELLS = 30
PANIC_X, PANIC_Y = 560, 1316                             # panic() reached
IDENTITY_X, IDENTITY_Y = 32, 1170                        # 64-bit bitmaps, 16 px apart
IDENTITY_PITCH_X = 12
COLOR_YELLOW = 0xFFFFFF00
COLOR_CYAN = 0xFF00FFFF
COLOR_GREEN = 0xFF00FF00
COLOR_RED = 0xFFFF0000
WDT_PA = 0x1C007000                    # soc/watchdog@1c007000, syscon of reboot-mode
WDT_MODE_OFF = 0x24                    # reboot-mode offset in the syscon
WDT_MODE_MASK = 0xF                    # reboot-mode mask
WDT_MODE_BOOTLOADER = 3                # reboot-mode value for LK fastboot
# kext[0]'s first __kalloc_type_view (__PLK_DATA_CONST) and a __PLK_TEXT_EXEC
# canary whose file qwords are stable; the entry check compares them in the
# window at physical addresses derived from boot_args.
ENTRY_CHECK_PROBES = (
    (0xFFFFFFF005CB3168, 'kext[0] view name pointer'),
    (0xFFFFFFF005CB3180, 'kext[0] view flags'),
    (0xFFFFFFF005125000, '__PLK_TEXT_EXEC canary'),
)
ENTRY_CHECK_BAR_Y = 1200               # rows proven visible in earlier probes
ENTRY_CHECK_BAR_ROWS = 64
ENTRY_REPORT_Y = 1200                  # five 200x48 status cells, left to right
ENTRY_REPORT_CELL_W = 200
ENTRY_REPORT_CELL_H = 48
ENTRY_REPORT_BITS_Y = 1290             # five 64-bit rows, MSB left, black = 1, red = 0
ENTRY_REPORT_BITS_X = 32
ENTRY_REPORT_ROW_PITCH = 24
ENTRY_REPORT_DELAY_BASE = 60           # seconds; first failing check adds a step
ENTRY_REPORT_DELAY_STEP = 20

# The entry-scan cave looks for the kext[0] __kalloc_type record page in DRAM.
# Three copies of the kernel file can exist at the kernel entry: the DarwinBoot
# Image pool buffer (AllocatePool data starts POOL_HEAD bytes into its page),
# the decompressed payload FAT volume (Tiano destination, page aligned; its
# record page sits at the volume offset of the file page), and the boot window
# itself. The page starts with the first record's qword and holds the name
# pointer and flags of the probed view at fixed in-page offsets.
ENTRY_SCAN_DELAY_BASE = 20             # seconds; the result code adds a step each
ENTRY_SCAN_DELAY_STEP = 10
ENTRY_SCAN_START = 0x40000000          # DRAM base of the tb365fc memory map
ENTRY_SCAN_END = 0xC0000000            # below the display reservation; covers all low Conv regions
ENTRY_SCAN_POOL_OFFSET = 0x18          # sizeof (POOL_HEAD) without a tail: Data at +24
ENTRY_SCAN_VOLUME_PAGE_OFFSET = 0xA00  # payload10.img: record page at volume 0x1917a00
ENTRY_SCAN_BITS = (
    (0x1, 'window page holds the record page'),
    (0x2, 'a page-aligned copy exists in DRAM'),
    (0x4, 'the Image pool copy exists'),
    (0x8, 'the decompressed FAT volume copy exists'),
)

# The entry hunt replaces the entry scan. Rather than a wide DDR scan (diag64
# vanished inside 0x40000000-0xC0000000, consistent with BL31; diag68 did not
# reach its reset), it only reads the window record page and compares it
# against the loader's own readback (DarwinBoot.c writes, right after the copy
# loop, what it sees of the same page in the file buffer and in the window to
# ArgsAddress+0x800) and against what BootRamdiskDxe saw of the decompressed
# FAT image before the FAT driver read it. diag70's variable transport failed,
# so diag71 has BootRamdiskDxe copy that block to a fixed page in the boot
# payload window (0x413ff000). Five 200x48 status cells carry the result as
# green/red, six 64-bit rows read the raw readback qwords, and the reset is
# 60 s after the paints with the reboot-mode syscon re-armed just before it.
ENTRY_HUNT_SPAN = 0x7F0                # vector table + code, ending at PANIC_CAVE_END
ENTRY_HUNT_ENTRY_OFF = 0x390           # _start inside the blob
ENTRY_HUNT_HANDLER_OFF = 0x780         # exception handler inside the blob
ENTRY_HUNT_PROBE_OFF = 0x800           # loader readback block inside the args page
ENTRY_HUNT_PROBE_MAGIC = 0x45424F5250363944   # "D96PROBE", matches DarwinBoot.c
# PAYLOAD_PROBE_BLOCK fields appended to LOADER_PROBE_BLOCK after EntryFileOffset
# (0x2B0) in DarwinBoot.c.
ENTRY_HUNT_VOL_MAGIC = 0x3037505244    # "DRP70", matches BootRamdiskDxe.c
ENTRY_HUNT_VOL_MAGIC_OFF = 0x2B8
ENTRY_HUNT_VOL_DEST_OFF = 0x2C0
ENTRY_HUNT_VOL_SIZE_OFF = 0x2C8
ENTRY_HUNT_VOL_SIG_OFF = 0x2D0
ENTRY_HUNT_VOL_NAME_OFF = 0x2D8
ENTRY_HUNT_VOL_FLAGS_OFF = 0x2E0
ENTRY_HUNT_VOL_ENTRY_OFF = 0x2E8
ENTRY_HUNT_VOL_SIZE = 0x3800000        # decompressed payload FAT image size
ENTRY_HUNT_REC_FILE_OFF_OFF = 0x2A8    # RecFileOffset inside LOADER_PROBE_BLOCK
ENTRY_HUNT_REC_FILE_VALUE = 0x2B8E000  # record page offset inside kernel.macho
ENTRY_HUNT_DELAY_BASE = 60
ENTRY_HUNT_DELAY_STEP = 0              # fixed wait; the cell bitmask is not a count
ENTRY_HUNT_CELLS_Y = 1200
ENTRY_HUNT_BITS_Y = 1284
ENTRY_HUNT_ROW_PITCH = 20
ENTRY_HUNT_RED_Y = 1400                # exception block below the painted rows
ENTRY_HUNT_RED_LINES = 256             # 1.6 MB; outlives the reset latency in a video

SOURCES = {
    3: """.text
.global _start
_start:
  mov x0, #0x300000
  mrs x1, cntpct_el0
  ldr x2, ={ticks}
  add x2, x1, x2
1:
  isb
  mrs x1, cntpct_el0
  cmp x1, x2
  b.lo 1b
  movz w0, #0x0009
  movk w0, #0x8400, lsl #16
  mov x1, xzr
  mov x2, xzr
  mov x3, xzr
  smc #0
2:
  wfe
  b 2b
""" + ".ltorg\n",
    4: """.text
.global _start
_start:
  bl callee
  mrs x1, cntpct_el0
  ldr x2, ={ticks}
  add x2, x1, x2
1:
  isb
  mrs x1, cntpct_el0
  cmp x1, x2
  b.lo 1b
  movz w0, #0x0009
  movk w0, #0x8400, lsl #16
  mov x1, xzr
  mov x2, xzr
  mov x3, xzr
  smc #0
2:
  wfe
  b 2b
""" + ".ltorg\n",
}

RESET_BEFORE = """.text
.global _start
_start:
  movz w0, #0x0009
  movk w0, #0x8400, lsl #16
  mov x1, xzr
  mov x2, xzr
  mov x3, xzr
  smc #0
1:
  wfe
  b 1b
"""

ENTRY_CHECK = """.text
.global _start
_start:                         /* runs instead of the entry `b _start_first_cpu` */
  ldr x9, [x0, #{phys_off:#x}]  /* boot_args physBase */
  ldr x10, [x0, #{virt_off:#x}] /* boot_args virtBase */
  ldr x11, ={kernel_base:#x}
  cmp x10, x11
  b.ne bad
  tst x9, #0xfff
  b.ne bad
{checks}  /* the window still holds the file's bytes: mark and let XNU run */
  ldr x5, ={fb_base:#x}
  ldr x6, ={bar_off:#x}
  add x5, x5, x6
  movz w7, #0xff00
  movk w7, #0xff00, lsl #16
  ldr x8, ={pixels}
1:
  str w7, [x5], #4
  subs x8, x8, #1
  b.ne 1b
  dsb sy
  b {target}
bad:                            /* window != file: red bar, then reset to fastboot */
  ldr x5, ={fb_base:#x}
  ldr x6, ={bar_off:#x}
  add x5, x5, x6
  movz w7, #0xffff, lsl #16
  ldr x8, ={pixels}
2:
  str w7, [x5], #4
  subs x8, x8, #1
  b.ne 2b
  dsb sy
  ldr x5, ={wdt_pa:#x}
  ldr w6, [x5, #{wdt_off:#x}]
  bic w6, w6, #{wdt_mask:#x}
  orr w6, w6, #{wdt_value:#x}
  str w6, [x5, #{wdt_off:#x}]
  dsb sy
  mrs x5, cntpct_el0
  ldr x6, ={ticks}
  add x6, x5, x6
3:
  isb
  mrs x5, cntpct_el0
  cmp x5, x6
  b.lo 3b
  movz w0, #0x0009
  movk w0, #0x8400, lsl #16
  mov x1, xzr
  mov x2, xzr
  mov x3, xzr
  smc #0
4:
  wfe
  b 4b
.ltorg
"""

ENTRY_REPORT = """.text
.global _start
_start:                         /* runs instead of the entry `b _start_first_cpu` */
  ldr x9, [x0, #{phys_off:#x}]  /* boot_args physBase */
  ldr x10, [x0, #{virt_off:#x}] /* boot_args virtBase */
  mov x19, x9
  mov x20, x10
  mov x21, xzr                  /* bit k set: check k failed */
  mrs x26, cntfrq_el0           /* timer Hz, painted as the sixth row */
  ldr x11, ={kernel_base:#x}
  cmp x10, x11
  b.eq 41f
  orr x21, x21, #1
41:
  tst x9, #0xfff
  b.eq 42f
  orr x21, x21, #2
42:
{checks}
{cells}
{rows}
  dsb sy
  cbz x21, 47f
  rbit x6, x21                  /* first failing check: 60 + 20*index s */
  clz x6, x6
  mov x7, #{delay_step}
  mov x8, #{delay_base}
  madd x6, x6, x7, x8
  mrs x7, cntfrq_el0
  mul x6, x6, x7
  ldr x5, ={wdt_pa:#x}
  ldr w7, [x5, #{wdt_off:#x}]
  bic w7, w7, #{wdt_mask:#x}
  orr w7, w7, #{wdt_value:#x}
  str w7, [x5, #{wdt_off:#x}]
  dsb sy
  mrs x5, cntpct_el0
  add x6, x5, x6
45:
  isb
  mrs x5, cntpct_el0
  cmp x5, x6
  b.lo 45b
  movz w0, #0x0009
  movk w0, #0x8400, lsl #16
  mov x1, xzr
  mov x2, xzr
  mov x3, xzr
  smc #0
46:
  wfe
  b 46b
47:
  b {target}
paint_cell:                     /* x5 pixels, w7 color; clobbers x8, x9, x12 */
  ldr x12, ={cell_advance:#x}
  mov x8, #48
51:
  mov x9, #200
52:
  str w7, [x5], #4
  subs x9, x9, #1
  b.ne 52b
  add x5, x5, x12
  subs x8, x8, #1
  b.ne 51b
  ret
paint_bits:                     /* x5 first row, x25 value; 64 cells of 16x8, MSB left */
  ldr x13, ={bits_advance:#x}
  mov x8, #8
53:
  mov x9, x25
  mov x10, #64
54:
  lsr x12, x9, #63
  movz w11, #0x0000
  movk w11, #0xff00, lsl #16
  cbnz x12, 55f
  movz w11, #0xffff, lsl #16
55:
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  lsl x9, x9, #1
  subs x10, x10, #1
  b.ne 54b
  add x5, x5, x13
  subs x8, x8, #1
  b.ne 53b
  ret
.ltorg
"""

ENTRY_SCAN = """.text
.global _start
_start:                         /* runs instead of the entry `b _start_first_cpu` */
  ldr x9, [x0, #{phys_off:#x}]  /* boot_args physBase */
  ldr x10, [x0, #{virt_off:#x}] /* boot_args virtBase */
  ldr x24, ={rec_sig:#x}
  ldr x25, ={name_ptr:#x}
  ldr x26, ={flags_val:#x}
  mov x21, xzr                  /* result code bits */
  ldr x11, ={win_page_off:#x}   /* record page offset in the boot window */
  add x11, x11, x9
  ldr x12, [x11]
  cmp x12, x24
  b.ne 2f
  ldr x12, [x11, #{name_off:#x}]
  cmp x12, x25
  b.ne 2f
  ldr x12, [x11, #{flags_off:#x}]
  cmp x12, x26
  b.ne 2f
  orr x21, x21, #1
2:
  ldr x22, ={scan_start:#x}
  ldr x23, ={scan_end:#x}
scan:                           /* one page: window-, Image-, volume-shifted copies */
  ldr x12, [x22]
  cmp x12, x24
  b.ne 3f
  ldr x12, [x22, #{name_off:#x}]
  cmp x12, x25
  b.ne 3f
  ldr x12, [x22, #{flags_off:#x}]
  cmp x12, x26
  b.ne 3f
  orr x21, x21, #2
3:
  ldr x12, [x22, #{img_sig_off:#x}]
  cmp x12, x24
  b.ne 4f
  ldr x12, [x22, #{img_name_off:#x}]
  cmp x12, x25
  b.ne 4f
  ldr x12, [x22, #{img_flags_off:#x}]
  cmp x12, x26
  b.ne 4f
  orr x21, x21, #4
4:
  ldr x12, [x22, #{vol_sig_off:#x}]
  cmp x12, x24
  b.ne 5f
  ldr x12, [x22, #{vol_name_off:#x}]
  cmp x12, x25
  b.ne 5f
  ldr x12, [x22, #{vol_flags_off:#x}]
  cmp x12, x26
  b.ne 5f
  orr x21, x21, #8
5:
  add x22, x22, #0x1000
  cmp x22, x23
  b.ne scan
  mov x6, #{delay_step:#x}      /* delay = base + step * code seconds */
  mov x8, #{delay_base:#x}
  madd x6, x21, x6, x8
  mrs x7, cntfrq_el0
  mul x6, x6, x7
  ldr x5, ={wdt_pa:#x}
  ldr w7, [x5, #{wdt_off:#x}]
  bic w7, w7, #{wdt_mask:#x}
  orr w7, w7, #{wdt_value:#x}
  str w7, [x5, #{wdt_off:#x}]
  dsb sy
  mrs x5, cntpct_el0
  add x6, x5, x6
6:
  isb
  mrs x5, cntpct_el0
  cmp x5, x6
  b.lo 6b
  movz w0, #0x0009
  movk w0, #0x8400, lsl #16
  mov x1, xzr
  mov x2, xzr
  mov x3, xzr
  smc #0
7:
  wfe
  b 7b
.ltorg
"""

ENTRY_HUNT = """.text
  .org 0x000
hunt_vector:                    /* VBAR: sync from the current EL with SP0 */
  b hunt_common
  .org 0x180                    /* serror from the current EL with SP0 */
  b hunt_common
  .org 0x200                    /* sync from the current EL with SPx */
  b hunt_common
  .org 0x380                    /* serror from the current EL with SPx */
  b hunt_common
  .org 0x390
.global _start
_start:                         /* runs instead of the entry `b _start_first_cpu` */
  ldr x9, [x0, #{phys_off:#x}]  /* boot_args physBase */
  ldr x10, [x0, #{virt_off:#x}] /* boot_args virtBase */
  mov x19, x9
  mov x20, x10
  add x23, x0, #{probe_off:#x}  /* loader readback block in the args page */
  ldr x5, ={wdt_pa:#x}          /* arm fastboot before the first memory read */
  ldr w7, [x5, #{wdt_off:#x}]
  bic w7, w7, #{wdt_mask:#x}
  orr w7, w7, #{wdt_value:#x}
  str w7, [x5, #{wdt_off:#x}]
  dsb sy
  ldr x5, ={hunt_off:#x}        /* MMU is off: VBAR takes the physical vector base */
  add x5, x5, x19
  mrs x6, CurrentEL
  and x6, x6, #0xc
  cmp x6, #0x8
  b.eq 1f
  msr vbar_el1, x5
  b 2f
1:
  msr vbar_el2, x5
2:
  isb
  mov x21, xzr                  /* one bit per failed status cell */
  ldr x14, ={win_page_off:#x}
  add x14, x14, x19             /* window record page, physical */
  ldr x28, [x14]
  ldr x29, [x14, #{name_off:#x}]
  ldr x18, [x14, #{flags_off:#x}]
  ldr x15, ={entry_off:#x}
  add x15, x15, x19             /* window entry instruction, physical */
  ldr x17, [x15]                /* x30 is the `bl` link register below */
  bl hunt_paint
  dsb sy
  mov x6, #{delay_step:#x}      /* fixed wait: base + step * failed-cell bitmask */
  mov x8, #{delay_base:#x}
  madd x6, x21, x6, x8
  mrs x7, cntfrq_el0
  mul x6, x6, x7
  mrs x5, cntpct_el0
  add x6, x5, x6
8:
  isb
  mrs x5, cntpct_el0
  cmp x5, x6
  b.lo 8b
  ldr x5, ={wdt_pa:#x}          /* re-arm fastboot: the early write may go stale */
  ldr w7, [x5, #{wdt_off:#x}]
  bic w7, w7, #{wdt_mask:#x}
  orr w7, w7, #{wdt_value:#x}
  str w7, [x5, #{wdt_off:#x}]
  dsb sy
  movz w0, #0x0009
  movk w0, #0x8400, lsl #16
  mov x1, xzr
  mov x2, xzr
  mov x3, xzr
  smc #0
9:
  wfe
  b 9b
hunt_paint:
  ldr x16, ={cells_base:#x}   /* x16 survives cells and painters */
{cells}
{rows}
  dsb sy
  ret
paint_cell:                     /* x5 pixels, w7 color; clobbers x8, x9, x12 */
  ldr x12, ={cell_advance:#x}
  mov x8, #48
51:
  mov x9, #200
52:
  str w7, [x5], #4
  subs x9, x9, #1
  b.ne 52b
  add x5, x5, x12
  subs x8, x8, #1
  b.ne 51b
  ret
paint_bits:                     /* x5 first row, x25 value; 64 cells of 16x8, MSB left */
  ldr x13, ={bits_advance:#x}
  mov x8, #8
53:
  mov x9, x25
  mov x10, #64
54:
  lsr x12, x9, #63
  movz w11, #0x0000
  movk w11, #0xff00, lsl #16
  cbnz x12, 55f
  movz w11, #0xffff, lsl #16
55:
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  str w11, [x5], #4
  lsl x9, x9, #1
  subs x10, x10, #1
  b.ne 54b
  add x5, x5, x13
  subs x8, x8, #1
  b.ne 53b
  ret
  .ltorg
  .org 0x780
hunt_common:                    /* any exception: red band, then fastboot reset */
  ldr x5, ={red_bar:#x}
  movz w6, #0xffff, lsl #16
  ldr x7, ={red_words:#x}
16:
  str w6, [x5], #4
  subs x7, x7, #1
  b.ne 16b
  dsb sy
  ldr x5, ={wdt_pa:#x}
  ldr w7, [x5, #{wdt_off:#x}]
  bic w7, w7, #{wdt_mask:#x}
  orr w7, w7, #{wdt_value:#x}
  str w7, [x5, #{wdt_off:#x}]
  dsb sy
  movz w0, #0x0009
  movk w0, #0x8400, lsl #16
  mov x1, xzr
  mov x2, xzr
  mov x3, xzr
  smc #0
17:
  wfe
  b 17b
  .ltorg
"""

ENTRY_RESET = """.text
.global _start
_start:                         /* runs instead of the entry `b _start_first_cpu` */
  ldr x6, ={ticks:#x}           /* raw cntpct ticks to wait (0 = none) */
  cbz x6, 2f
  mrs x5, cntpct_el0
  add x6, x5, x6
1:
  isb
  mrs x5, cntpct_el0
  cmp x5, x6
  b.lo 1b
2:
  ldr x5, ={wdt_pa:#x}
  ldr w7, [x5, #{wdt_off:#x}]
  bic w7, w7, #{wdt_mask:#x}
  orr w7, w7, #{wdt_value:#x}
  str w7, [x5, #{wdt_off:#x}]
  dsb sy
  movz w0, #0x0009
  movk w0, #0x8400, lsl #16
  mov x1, xzr
  mov x2, xzr
  mov x3, xzr
  smc #0
3:
  wfe
  b 3b
.ltorg
"""

ZALLOC_PROGRESS = """.text
.global _start
.global _selftest
_start:                         /* runs instead of the entry `blr x8` */
  sub sp, sp, #32
  stp x0, x8, [sp]
  cmp x19, #7
  b.ne 9f
  ldr x9, ={scratch}
  ldr x10, [x9]
  lsr x11, x10, #48
  movz w12, #0xffff
  cmp w11, w12
  b.ne 9f
  ldr x11, ={zbase}
  sub x11, x23, x11
  mov x12, #24
  udiv x11, x11, x12
  cmp x11, #{last}
  b.hi 9f
  cbnz x11, 1f
  ldr x14, ={gate}
  add x0, x10, x14
  movz w1, #0x00ff
  movk w1, #0xff00, lsl #16
  mov x2, #{gate_bytes}
  mov x3, #{gate_rows}
  mov x4, #{stride}
  bl paint
  /* paint clobbers x9-x11; reload the window base and the entry index. */
  ldr x9, ={scratch}
  ldr x10, [x9]
  ldr x11, ={zbase}
  sub x11, x23, x11
  mov x12, #24
  udiv x11, x11, x12
1:
  and x12, x11, #{grid_mask}
  lsr x13, x11, #{grid_shift}
  ldr x14, ={row_step}
  mul x13, x13, x14
  mov x14, #{cell_pitch}
  madd x13, x12, x14, x13
  ldr x14, ={grid}
  add x13, x13, x14
  add x0, x10, x13
  movz w1, #0xff00
  movk w1, #0xff00, lsl #16
  mov x2, #{cell_bytes}
  mov x3, #{cell_rows}
  mov x4, #{stride}
  bl paint
9:
  ldp x0, x8, [sp]
  add sp, sp, #32
  blr x8
  ldr x30, ={return_to}
  ret

_selftest:                      /* runs instead of `bl _vm_fault_init` */
  stp x29, x30, [sp, #-16]!
  bl _vm_fault_init
{fastboot}  ldr x0, ={window_pa}
  ldr x1, ={window_len}
  bl _ml_map_high_window
  cbz x0, 8f
  ldr x9, ={scratch}
  str x0, [x9]
  ldr x14, ={white}
  add x0, x0, x14
  movn w1, #0
  mov x2, #{white_bytes}
  mov x3, #{white_rows}
  mov x4, #{stride}
  bl paint
8:
  ldp x29, x30, [sp], #16
  ldr x30, ={fault_return}
  ret

/* x0 = first pixel, w1 = colour, x2 = bytes per row, x3 = rows, x4 = stride.
 * The mapping is writeback; clean each row to the point of coherency so the
 * scanout engine sees it. */
paint:
  mov x5, x0
  mov x9, x3
3:
  mov x10, x5
  mov x7, x2
4:
  str w1, [x10], #4
  subs x7, x7, #4
  b.ne 4b
  bic x8, x5, #63
  add x11, x5, x2
  add x11, x11, #63
  bic x11, x11, #63
5:
  dc civac, x8
  add x8, x8, #64
  cmp x8, x11
  b.lo 5b
  add x5, x5, x4
  subs x9, x9, #1
  b.ne 3b
  dsb sy
  ret
.ltorg
"""

FASTBOOT_ON_RESET = f"""  /* The device tree's reboot-mode syscon: writing bootloader (3) under the
   * 0xf mask at wdt[0x24] lets preloader/LK pick fastboot after the reset, so
   * the probe's PSCI reset lands back in fastboot, not in Android. */
  ldr x0, ={WDT_PA:#x}
  ldr x1, =0x1000
  bl _ml_io_map
  cbz x0, 7f
  ldr w9, [x0, #{WDT_MODE_OFF:#x}]
  and w9, w9, #{~WDT_MODE_MASK & 0xffffffff:#x}
  orr w9, w9, #{WDT_MODE_BOOTLOADER:#x}
  str w9, [x0, #{WDT_MODE_OFF:#x}]
  dsb sy
7:
"""


def kernel_symbols(kernel):
    """Address -> name for the kernel's symbols, from llvm-nm."""
    nm = shutil.which('llvm-nm') or '/opt/homebrew/opt/llvm/bin/llvm-nm'
    table = {}
    for line in subprocess.run([nm, '-n', kernel], capture_output=True, text=True, check=True).stdout.splitlines():
        parts = line.split()
        if len(parts) == 3:
            table.setdefault(int(parts[0], 16), parts[2])
    return table


def symbol_name_addresses(kernel):
    """Name -> address, keeping aliases the address-keyed table drops."""
    nm = shutil.which('llvm-nm') or '/opt/homebrew/opt/llvm/bin/llvm-nm'
    table = {}
    for line in subprocess.run([nm, '-n', kernel], capture_output=True, text=True, check=True).stdout.splitlines():
        parts = line.split()
        if len(parts) == 3:
            table.setdefault(parts[2], int(parts[0], 16))
    return table


def function_range(symbols, name):
    """Start and end (next symbol) of a function by its symbol name."""
    starts = sorted(symbols)
    for start, symbol in symbols.items():
        if symbol == name:
            return start, next(address for address in starts if address > start)
    raise SystemExit(f'No symbol {name} in the kernel')


def find_call(data, symbols, function, name, occurrence):
    """The VA and target of the Nth `bl name` inside FUNCTION."""
    hits = []
    low, high = function_range(symbols, function)
    for va in range(low, high, 4):
        word = struct.unpack_from('<I', data, file_offset(va))[0]
        if word >> 26 == 0b100101:
            delta = word & 0x3FFFFFF
            delta -= (1 << 26) if delta & (1 << 25) else 0
            target = va + delta * 4
            if symbols.get(target) == name:
                hits.append((va, target))
    if len(hits) < occurrence:
        raise SystemExit(f'{function} has {len(hits)} call(s) to {name}, not {occurrence}')
    return hits[occurrence - 1]


def assemble_c(source, va, workdir, symbols):
    """Build a freestanding C file for VA; returns (bytes, load address of the first byte, address of panic_screen).

    The linker may start .text a little after VA (it keeps the ELF headers in front), so the
    load address is read back from the ELF and is what the caller must write the bytes to."""
    clang = shutil.which('clang') or '/opt/homebrew/opt/llvm/bin/clang'
    lld = shutil.which('ld.lld') or '/opt/homebrew/opt/llvm/bin/ld.lld'
    objcopy = shutil.which('llvm-objcopy') or '/opt/homebrew/opt/llvm/bin/llvm-objcopy'
    nm = shutil.which('llvm-nm') or '/opt/homebrew/opt/llvm/bin/llvm-nm'
    # UNIXNU_UEFI is the Mu-Silicium checkout when the unixnu driver runs this script.
    font_dir = Path(os.environ.get('UNIXNU_UEFI') or ROOT / 'uefi-sheng') / 'Silicon/Silicium/SiliciumPkg/Library/FrameBufferSerialPortLib'
    if not (font_dir / 'Font.h').is_file():
        raise SystemExit('Panic screen requires Mu-Silicium FrameBufferSerialPortLib/Font.h; '
                         'set --uefi PATH or [build] uefi in unixnu.local.toml '
                         '(UNIXNU_UEFI for direct script calls)')
    obj, elf, binary = (workdir / name for name in ('panic.o', 'panic.elf', 'panic.bin'))
    definitions = []
    for macro, name, width in (('MAX_MEM', '_max_mem', 8),
                               ('PAGE_TOTAL', '_vm_page_pages', 4),
                               ('PAGE_FREE', '_vm_page_free_count', 4)):
        address = symbols.get(name)
        if address is None or address % width or not any(
                base <= address and address + width <= base + size
                for _, base, size, _, _ in SEGMENTS):
            raise SystemExit(f'Missing or invalid diagnostic symbol {name}')
        definitions.append(f'-D{macro}={address:#x}UL')
    # Optional: the board's banner buffer for the panic message (kits/sheng/BootProgress.c); kernels without it get 0 (no copy).
    text = symbols.get('_boot_progress_panic_text')
    if text is not None:
        if not any(base <= text and text + 256 <= base + size for _, base, size, _, _ in SEGMENTS):
            raise SystemExit('Invalid _boot_progress_panic_text')
        definitions.append(f'-DPANIC_TEXT={text:#x}UL')
    subprocess.run([clang, '--target=aarch64-none-elf', '-ffreestanding', '-fno-builtin', '-fno-stack-protector',
                    '-mgeneral-regs-only', '-fno-pic', '-mcmodel=tiny', '-Oz', f'-DVINFO={VINFO_VA:#x}UL',
                    *definitions, f'-I{font_dir}', '-c', source, '-o', obj], check=True)
    script = workdir / 'panic.ld'
    script.write_text('SECTIONS { . = %#x; .text : { *(.text.a_entry) *(.text.a_common) *(.text*) *(.rodata*) } /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) *(.ARM.*) } }\n' % va)
    size = shutil.which('llvm-size') or '/opt/homebrew/opt/llvm/bin/llvm-size'
    sizes = subprocess.run([size, '--format=berkeley', obj], capture_output=True, text=True, check=True).stdout.splitlines()[-1].split()
    if int(sizes[1]) or int(sizes[2]):
        # The code runs from the kernel's read-only text: a global would fault on its first store.
        raise SystemExit(f'{source.name} has writable data (data {sizes[1]}, bss {sizes[2]}); use the stack')
    subprocess.run([lld, '-T', script, '-e', 'panic_screen', obj, '-o', elf], check=True)
    # Only .text: a plain binary dump would start at the ELF headers kept in front of it.
    subprocess.run([objcopy, '-O', 'binary', '-j', '.text', elf, binary], check=True)
    readelf = shutil.which('llvm-readelf') or '/opt/homebrew/opt/llvm/bin/llvm-readelf'
    load = None
    for line in subprocess.run([readelf, '-S', elf], capture_output=True, text=True, check=True).stdout.splitlines():
        parts = line.replace('[', ' ').replace(']', ' ').split()
        if len(parts) >= 4 and parts[1] == '.text':
            load = int(parts[3], 16)
    if load is None or load < va:
        raise SystemExit('Cannot find where the linker put .text')
    entry = None
    for line in subprocess.run([nm, elf], capture_output=True, text=True, check=True).stdout.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] == 'panic_screen':
            entry = int(parts[0], 16)
    if entry != load:
        raise SystemExit(f'panic_screen at {entry:#x} is not the first byte {load:#x}')
    return binary.read_bytes(), load, entry


def panic_common_entry(workdir, code, load):
    """Resolve the real alternate entry and verify its distinct argument ABI."""
    nm = shutil.which('llvm-nm') or '/opt/homebrew/opt/llvm/bin/llvm-nm'
    names = {parts[2]: int(parts[0], 16)
             for line in subprocess.check_output([nm, workdir / 'panic.elf'], text=True).splitlines()
             if len(parts := line.split()) == 3}
    entry = names.get('panic_screen_common')
    main = names.get('panic_screen_main')
    if entry is None or main is None or not load <= entry <= load + len(code) - 8:
        raise SystemExit('Common panic renderer entry is missing or out of bounds')
    at = entry - load
    if code[at:at + 8] != struct.pack('<II', 0xf9400021, branch(entry + 4, main)):
        raise SystemExit('Common panic entry does not dereference the Darwin va_list pointer and branch to the renderer')
    return entry


ROOT = Path(__file__).resolve().parents[1]


SEGMENTS = []   # (name, vmaddr, vmsize, fileoff, filesize) of the kernel being patched


def load_segments(data):
    """Read the Mach-O segment table; the file layout differs between kernel builds."""
    magic, _, _, _, ncmds = struct.unpack_from('<IiiII', data, 0)
    if magic != 0xFEEDFACF:
        raise SystemExit('Not a 64-bit Mach-O')
    offset = 32
    SEGMENTS.clear()
    for _ in range(ncmds):
        cmd, size = struct.unpack_from('<II', data, offset)
        if cmd == 0x19:
            name, vmaddr, vmsize, fileoff, filesize = struct.unpack_from('<16sQQQQ', data, offset + 8)
            if filesize:
                SEGMENTS.append((name.rstrip(b'\0').decode(), vmaddr, vmsize, fileoff, filesize))
        offset += size


def unixthread_pc(data):
    """LC_UNIXTHREAD entry pc (arm64 thread state: flavor, count, x0..x28, fp, lr, sp, pc)."""
    offset = 32
    for _ in range(struct.unpack_from('<I', data, 16)[0]):
        cmd, size = struct.unpack_from('<II', data, offset)
        if cmd == 0x5:
            return struct.unpack_from('<Q', data, offset + 16 + 32 * 8)[0]
        offset += size
    raise SystemExit('No LC_UNIXTHREAD in kernel')


def rebase(kernel, data):
    """The probe sites are fixed offsets from the entry; shift them for another kernel build.

    _panic and _vinfo come from the symbol table when the kernel has one (the plain source
    kernel does); the retained candidate layout is the unshifted case.
    """
    global ENTRY_VA, START_FIRST_CPU, MMU_ENABLE_VA, AFTER_MMU_VA, PANIC_VA, VINFO_VA, PANIC_CAVE_END, CAVES
    delta = unixthread_pc(data) - ENTRY_VA
    if delta:
        ENTRY_VA += delta
        START_FIRST_CPU += delta
        MMU_ENABLE_VA += delta
        AFTER_MMU_VA += delta
        PANIC_CAVE_END += delta
        CAVES = {stage: va + delta for stage, va in CAVES.items()}
        PANIC_VA += delta
        VINFO_VA += delta
    names = {name: va for va, name in kernel_symbols(kernel).items()}
    if '_start_first_cpu' in names:
        START_FIRST_CPU = names['_start_first_cpu']
        # Native bootstrap table-capacity checks can grow before SCTLR setup.
        # Locate the unique instruction sequence in this bounded entry body;
        # do not assume a constant offset from the Mach-O entry point.
        matches = []
        for va in range(START_FIRST_CPU, START_FIRST_CPU + 0x1000, 4):
            def instruction(at):
                return struct.unpack_from('<I', data, file_offset(at))[0]
            if (instruction(va) == 0xD5181000 and
                    instruction(va + 4) == 0xD5033FDF and
                    instruction(va + 0x20) == 0xD2A00600 and
                    instruction(va + 0x24) == 0xD5181040):
                matches.append(va)
        if len(matches) != 1:
            raise SystemExit('Require one verified SCTLR/ISB/CPACR entry sequence')
        MMU_ENABLE_VA = matches[0]
        AFTER_MMU_VA = MMU_ENABLE_VA + 0x20
    if '_panic' in names and '_vinfo' in names:
        PANIC_VA = names['_panic']
        # Board diagnostics seed this ABI-compatible prefix from valid console
        # mappings, then pin it to their retained GOP mapping. Ordinary source
        # kernels without the observer retain the original vinfo path.
        VINFO_VA = names.get('_boot_progress_panic_vinfo', names['_vinfo'])
    return names


def file_offset(va):
    for _, vmaddr, vmsize, fileoff, filesize in SEGMENTS:
        if vmaddr <= va < vmaddr + min(vmsize, filesize):
            return fileoff + (va - vmaddr)
    raise SystemExit(f'{va:#x} is not in a file-backed segment of this kernel')


def va_for_file(offset):
    """The virtual address of a byte offset in the file-backed segments."""
    for _, vmaddr, vmsize, fileoff, filesize in SEGMENTS:
        if fileoff <= offset < fileoff + filesize:
            return vmaddr + (offset - fileoff)
    raise SystemExit(f'File offset {offset:#x} is not in a file-backed segment')


def fb_offset(x, y, width, height, what='markers'):
    """Byte offset of a (mapped-window relative) mark from the window base."""
    offset = y * FRAMEBUFFER_STRIDE + x * 4 - WINDOW_OFF
    if offset < 0 or offset + (height - 1) * FRAMEBUFFER_STRIDE + width * 4 > WINDOW_LEN:
        raise SystemExit(f'Frame buffer {what} fall outside the mapped window')
    return offset


def branch(from_va, to_va):
    delta = (to_va - from_va) // 4
    if (to_va - from_va) % 4 or not -(1 << 25) <= delta < (1 << 25):
        raise SystemExit('Branch out of range')
    return 0x14000000 | (delta & 0x3FFFFFF)


def skip_panic_log(data, names):
    """PE_create_console() opens with check_for_panic_log(), which in this board
    config (ARM64_BOARD_CONFIG_BCM2837) maps PLATFORM_PANIC_LOG_PADDR 0x3c0fc000
    and then reads and clears 16 KiB there -- a Raspberry Pi address that on this
    SoC is not backed by memory.  Run the video console path without it: the
    block is replaced by `mov w20, #1` (the value the tail expects from it) and a
    branch past it."""
    va = names.get('_PE_create_console')
    if va is None:
        raise SystemExit('No _PE_create_console symbol')

    def word(at):
        return struct.unpack_from('<I', data, file_offset(at))[0]

    if [word(va + offset) for offset in (0x14, 0x18, 0x1c)] != [0x52980000, 0x72a781e0, 0x52880001]:
        raise SystemExit(f'_PE_create_console at {va:#x} does not start with the 0x3c0fc000 panic-log map')
    call = word(va + 0x20)
    delta = call & 0x3ffffff
    delta -= (1 << 26) if delta & (1 << 25) else 0
    if call >> 26 != 0b100101 or va + 0x20 + delta * 4 != names.get('_ml_io_map_wcomb'):
        raise SystemExit(f'_PE_create_console does not call _ml_io_map_wcomb at {va + 0x20:#x}')
    if [word(va + offset) for offset in (0x94, 0xa4)] != [0xf0ffc2c0, 0x1a941681]:
        raise SystemExit(f'_PE_create_console video console tail is not where it was at {va:#x}')
    struct.pack_into('<I', data, file_offset(va + 0x14), 0x52800034)   # mov w20, #1
    struct.pack_into('<I', data, file_offset(va + 0x18), branch(va + 0x18, va + 0x94))


def find_zalloc_site(data, symbols):
    """VA of the `blr x8` that runs each startup entry, checked against its neighbours.

    symbols is the address -> name table, as kernel_symbols() returns it."""
    low, high = function_range(symbols, '_kernel_startup_initialize_upto')
    by_name = {name: va for va, name in symbols.items()}
    log = by_name.get('_kernel_startup_log')
    if log is None:
        raise SystemExit('No _kernel_startup_log symbol')
    hits = []
    for va in range(low + 16, high, 4):
        call = struct.unpack_from('<I', data, file_offset(va - 8))[0]
        delta = call & 0x3FFFFFF
        delta -= (1 << 26) if delta & (1 << 25) else 0
        if (struct.unpack_from('<I', data, file_offset(va))[0] == 0xD63F0100
                and call >> 26 == 0b100101 and va - 8 + delta * 4 == log
                and struct.unpack_from('<I', data, file_offset(va - 12))[0] == 0xB94002E0
                and struct.unpack_from('<I', data, file_offset(va - 4))[0] == 0xA94082E8
                and struct.unpack_from('<I', data, file_offset(va + 4))[0] == 0x910062F7):
            hits.append(va)
    if len(hits) != 1:
        raise SystemExit(f'Expected one startup-entry `blr x8` in _kernel_startup_initialize_upto, found {len(hits)}')
    return hits[0]


def init_entry_set(data):
    """VA and size of the __BOOTDATA,__init_entry_set section."""
    magic, _, _, _, ncmds = struct.unpack_from('<IiiII', data, 0)
    offset = 32
    for _ in range(ncmds):
        cmd, size = struct.unpack_from('<II', data, offset)
        if cmd == 0x19:
            nsects = struct.unpack_from('<I', data, offset + 64)[0]
            section = offset + 72
            for _ in range(nsects):
                sectname, segname, addr, sect_size = struct.unpack_from('<16s16sQQ', data, section)
                if sectname.rstrip(b'\0') == b'__init_entry_set' and segname.rstrip(b'\0') == b'__BOOTDATA':
                    return addr, sect_size
                section += 80
        offset += size
    raise SystemExit('No __BOOTDATA,__init_entry_set section: not a startup-table kernel')


def zalloc_block(data):
    """VA and entry count of the STARTUP_SUB_ZALLOC block at run time.

    kernel_startup_bootstrap() qsorts the whole section by (subsystem, rank) in
    place before the first kernel_startup_initialize_upto(), so the ZALLOC
    entries end up contiguous, after every entry of a lower subsystem. The file
    keeps link order, so the run start is the section base plus 24 bytes for
    each lower-subsystem entry; that address is the same at run time."""
    base, size = init_entry_set(data)
    if size % 24:
        raise SystemExit(f'__init_entry_set is {size:#x} bytes, not a multiple of 24')
    lower = count = 0
    for index in range(size // 24):
        subsystem = struct.unpack_from('<I', data, file_offset(base + index * 24))[0]
        if subsystem < 7:
            lower += 1
        elif subsystem == 7:
            count += 1
    if not 100 <= count <= 4096 or lower + count > size // 24:
        raise SystemExit(f'Suspicious startup table: {lower} lower, {count} ZALLOC entries')
    zbase = base + lower * 24
    if struct.unpack_from('<I', data, file_offset(zbase))[0] != 7:
        raise SystemExit(f'No ZALLOC entry at the computed block start {zbase:#x}')
    return zbase, count


def zalloc_progress(data, kernel, names, workdir, fastboot=False):
    """Trampoline on the entry call plus a self-test hooked on _vm_fault_init."""
    symbols = kernel_symbols(kernel)
    site = find_zalloc_site(data, symbols)
    fault_site, _ = find_call(data, symbols, '_vm_mem_bootstrap', '_vm_fault_init', 1)
    zbase, count = zalloc_block(data)
    scratch = init_entry_set(data)[0] + 16
    needed = ('_vm_fault_init', '_ml_map_high_window')
    if fastboot:
        needed += ('_ml_io_map',)
    missing = [name for name in needed if name not in names]
    if missing:
        raise SystemExit(f'The ZALLOC progress probe needs symbols the kernel lacks: {", ".join(missing)}')
    if GRID_COLUMNS & (GRID_COLUMNS - 1) or count > GRID_COLUMNS * GRID_ROWS:
        raise SystemExit(f'{count} ZALLOC entries do not fit the {GRID_COLUMNS} x {GRID_ROWS} grid')
    if GRID_X + (GRID_COLUMNS - 1) * CELL_PITCH_X + CELL_W > 1600 or \
            GRID_Y + (GRID_ROWS - 1) * CELL_PITCH_Y + CELL_H > 2560:
        raise SystemExit('The progress grid does not fit the 1600 x 2560 screen')

    def bar(x, y, w, h):
        offset = y * FRAMEBUFFER_STRIDE + x * 4 - WINDOW_OFF
        if offset < 0 or offset + (h - 1) * FRAMEBUFFER_STRIDE + w * 4 > WINDOW_LEN:
            raise SystemExit('Frame buffer markers fall outside the mapped window')
        return offset, w * 4, h

    gate, gate_bytes, gate_rows = bar(GATE_X, GATE_Y, GATE_W, GATE_H)
    white, white_bytes, white_rows = bar(WHITE_X, WHITE_Y, WHITE_W, WHITE_H)
    grid = GRID_Y * FRAMEBUFFER_STRIDE + GRID_X * 4 - WINDOW_OFF
    cell_pitch = CELL_PITCH_X * 4
    row_step = CELL_PITCH_Y * FRAMEBUFFER_STRIDE
    end = (grid + (GRID_ROWS - 1) * row_step + (GRID_COLUMNS - 1) * cell_pitch
           + (CELL_H - 1) * FRAMEBUFFER_STRIDE + CELL_W * 4)
    if end > WINDOW_LEN:
        raise SystemExit('The progress grid falls outside the mapped window')
    cave = CAVES[4] + CAVE_SIZE
    source = ZALLOC_PROGRESS.format(
        scratch=f'{scratch:#x}', zbase=f'{zbase:#x}', last=count - 1,
        gate=f'{gate:#x}', gate_bytes=gate_bytes, gate_rows=gate_rows,
        white=f'{white:#x}', white_bytes=white_bytes, white_rows=white_rows,
        grid=f'{grid:#x}', row_step=f'{row_step:#x}', cell_pitch=cell_pitch,
        grid_mask=GRID_COLUMNS - 1, grid_shift=GRID_COLUMNS.bit_length() - 1,
        cell_bytes=CELL_W * 4, cell_rows=CELL_H, stride=FRAMEBUFFER_STRIDE,
        window_pa=f'{WINDOW_PA:#x}', window_len=f'{WINDOW_LEN:#x}',
        return_to=f'{site + 4:#x}', fault_return=f'{fault_site + 4:#x}',
        fastboot=FASTBOOT_ON_RESET if fastboot else '')
    code, load, labels = assemble(source, cave, {name: names[name] for name in needed}, workdir)
    if load != cave or '_start' not in labels or '_selftest' not in labels:
        raise SystemExit('The ZALLOC progress cave did not link where expected')
    text = (f'ZALLOC progress at {site:#x} ({count} entries from {zbase:#x}, '
            f'self-test at {fault_site:#x}, {len(code):#x} bytes)')
    if fastboot:
        text += f'; fastboot on reset (wdt[{WDT_MODE_OFF:#x}] := {WDT_MODE_BOOTLOADER})'
    return site, fault_site, cave, code, labels, text


PAINT_ASM = """/* x0 = first pixel, w1 = colour, x2 = bytes per row, x3 = rows, x4 = stride. */
paint:
  mov x5, x0
  mov x9, x3
3:
  mov x10, x5
  mov x7, x2
4:
  str w1, [x10], #4
  subs x7, x7, #4
  b.ne 4b
  bic x8, x5, #63
  add x11, x5, x2
  add x11, x11, #63
  bic x11, x11, #63
5:
  dc civac, x8
  add x8, x8, #64
  cmp x8, x11
  b.lo 5b
  add x5, x5, x4
  subs x9, x9, #1
  b.ne 3b
  dsb sy
  ret
.ltorg
"""

PAINT2_ASM = """/* x8 = first pixel, w9 = colour, x10 = bytes per row, x11 = rows, x12 = stride.
 * Like paint, but with its arguments in x8-x12 so wrapper stubs never touch the
 * wrapped call's x0-x7. Clobbers x13-x17 only. */
paint2:
  mov x13, x8
  mov x16, x11
3:
  mov x14, x13
  mov x17, x10
4:
  str w9, [x14], #4
  subs x17, x17, #4
  b.ne 4b
  bic x15, x13, #63
  add x17, x13, x10
  add x17, x17, #63
  bic x17, x17, #63
5:
  dc civac, x15
  add x15, x15, #64
  cmp x15, x17
  b.lo 5b
  add x13, x13, x12
  subs x16, x16, #1
  b.ne 3b
  dsb sy
  ret
.ltorg
"""


def mark_load(register, offset):
    """Two instructions that put a sub-32-bit absolute value into a W register."""
    return (f'  movz {register}, #{offset & 0xffff:#x}\n'
            f'  movk {register}, #{offset >> 16:#x}, lsl #16\n')

STAGE_CALLS = (('_kalloc_zone_init', 0x74), ('_kalloc_zone_init', 0x9c),
               ('_kmem_alloc_guard', 0xc8), ('_kalloc_type_view_init_fixed', 0xd4),
               ('_kalloc_type_view_init_var', 0xec))


def bl_target(word, site):
    offset = word & 0x3FFFFFF
    if offset & 0x2000000:
        offset -= 0x4000000
    return site + offset * 4


def kalloc_stages(data, names, workdir, va):
    """One yellow mark per sub-call of kalloc_init, so a stop inside it names the step.

    The stubs paint through paint2 (x8-x12) so the wrapped call still receives its
    real x0-x7 arguments -- kalloc_zone_init takes four and kmem_alloc_guard five."""
    kernel_init = names.get('_kalloc_init')
    missing = [name for name, _ in STAGE_CALLS if name not in names]
    if kernel_init is None or missing:
        raise SystemExit(f'The kalloc stage probe needs symbols this kernel lacks: {", ".join(missing) or "_kalloc_init"}')
    scratch = init_entry_set(data)[0] + 16
    marks = [STAGE_Y * FRAMEBUFFER_STRIDE + (STAGE_X + index * STAGE_PITCH_X) * 4 - WINDOW_OFF
             for index in range(len(STAGE_CALLS))]
    if marks[-1] + (CELL_H - 1) * FRAMEBUFFER_STRIDE + CELL_W * 4 > WINDOW_LEN:
        raise SystemExit('The kalloc stage marks fall outside the mapped window')
    sites = [kernel_init + offset for _, offset in STAGE_CALLS]
    for site, (name, _) in zip(sites, STAGE_CALLS):
        word = struct.unpack_from('<I', data, file_offset(site))[0]
        if word & 0xFC000000 != 0x94000000 or bl_target(word, site) != names[name]:
            raise SystemExit(f'{site:#x} is not the expected call to {name}')
    body = []
    for index, ((name, _), mark, site) in enumerate(zip(STAGE_CALLS, marks, sites)):
        label = '_start:\n_stage0' if index == 0 else f'_stage{index}'
        body.append(f"""{label}:
  ldr x10, ={scratch:#x}
  ldr x10, [x10]
  tbz x10, #48, 9f
{mark_load('w8', mark)}  add x8, x10, x8
  movz w9, #0xff00
  movk w9, #0xffff, lsl #16
  mov x10, #{CELL_W * 4}
  mov x11, #{CELL_H}
  mov x12, {FRAMEBUFFER_STRIDE}
  bl paint2
9:
  bl {name}
  ldr x30, ={site + 4:#x}
  ret
""")
    globals_ = '\n'.join(f'.global _stage{index}' for index in range(len(STAGE_CALLS)))
    source = f'.text\n.global _start\n{globals_}\n' + '\n'.join(body) + PAINT2_ASM
    code, load, labels = assemble(source, va, {name: names[name] for name, _ in STAGE_CALLS}, workdir)
    if load != va or labels.get('_start') != va or any(f'_stage{index}' not in labels for index in range(len(STAGE_CALLS))):
        raise SystemExit('The kalloc stage cave did not link where expected')
    if 'paint2' not in labels:
        raise SystemExit('The kalloc stage cave lost its paint helper')
    text = f'kalloc stages at {va:#x} ({len(code):#x} bytes)'
    return ([(site, labels[f'_stage{index}']) for index, site in enumerate(sites)],
            code, text, labels['paint2'])


ZSTAGE_CALLS = (('_scnprintf', 0x94), ('_zalloc_permanent_tag', 0xb0),
                ('_strlcpy', 0xc4), ('_zone_create_ext', 0xf4))
ZSTAGE_COLORS = (0xFFFFFF00, 0xFF00FFFF, 0xFFFF00FF, 0xFFFF0000)
ZSTAGE_ITERATIONS = 22

ZCREATE_X, ZCREATE_Y = 32, 1292                          # zone_create_ext step marks
ZCREATE_PITCH_X = 24
ZCREATE_STAGES = (
    ('_lck_spin_lock', 0x60, 0xFFFFFFFF),                # find lock taken
    ('_lck_spin_unlock', 0x428, 0xFF00FF00),             # find done, slot initialised
    ('__zalloc_permanent', 0x4f0, 0xFF00FFFF),           # percpu zone_stats
    (None, 0x748, 0xFFFF00FF),                           # extra_setup block (blr x8)
    ('_zone_setup_logging', 0x7bc, 0xFFFF0000),
    ('_hw_lck_ticket_lock', 0x910, 0xFF0000FF),          # final zone install
)


def kalloc_zone_stages(data, names, workdir, va, paint2_va, heap=None):
    """One colour-coded mark per loop iteration of kalloc_zone_init (x27 is the index).

    paint2 keeps the wrapped zalloc/zone_create_ext arguments in x0-x4 intact.
    The heap filter uses x21, where the prologue keeps the kheap_id argument,
    because the row is otherwise reused by both kalloc_heap_init calls."""
    zone_init = names.get('_kalloc_zone_init')
    missing = [name for name, _ in ZSTAGE_CALLS if name not in names]
    if zone_init is None or missing:
        raise SystemExit(f'The kalloc zone probe needs symbols this kernel lacks: {", ".join(missing) or "_kalloc_zone_init"}')
    heap_id = {'shared': 1, 'data': 2}.get(heap)
    if heap is not None and heap_id is None:
        raise SystemExit(f'Unknown kalloc heap filter: {heap}')
    scratch = init_entry_set(data)[0] + 16
    base = ZSTAGE_Y * FRAMEBUFFER_STRIDE + ZSTAGE_X * 4 - WINDOW_OFF
    end = (base + (ZSTAGE_ITERATIONS - 1) * ZSTAGE_PITCH_X * 4
           + (CELL_H - 1) * FRAMEBUFFER_STRIDE + CELL_W * 4)
    if ZSTAGE_X + (ZSTAGE_ITERATIONS - 1) * ZSTAGE_PITCH_X + CELL_W > 1600 or end > WINDOW_LEN:
        raise SystemExit('The kalloc zone marks fall outside the mapped window')
    sites = [zone_init + offset for _, offset in ZSTAGE_CALLS]
    for site, (name, _) in zip(sites, ZSTAGE_CALLS):
        word = struct.unpack_from('<I', data, file_offset(site))[0]
        if word & 0xFC000000 != 0x94000000 or bl_target(word, site) != names[name]:
            raise SystemExit(f'{site:#x} is not the expected call to {name}')
    body = []
    for index, ((name, _), site) in enumerate(zip(ZSTAGE_CALLS, sites)):
        color = ZSTAGE_COLORS[index]
        label = '_start:\n_zstage0' if index == 0 else f'_zstage{index}'
        heap_check = f'  cmp w21, #{heap_id}\n  b.ne 9f\n' if heap_id is not None else ''
        body.append(f"""{label}:
{heap_check}  ldr x10, ={scratch:#x}
  ldr x10, [x10]
  tbz x10, #48, 9f
  add x9, x27, x27, lsl #1
  lsl x9, x9, #5
{mark_load('w8', base)}  add x8, x10, x8
  add x8, x8, x9
  movz w9, #{color & 0xffff:#x}
  movk w9, #{color >> 16:#x}, lsl #16
  mov x10, #{CELL_W * 4}
  mov x11, #{CELL_H}
  mov x12, {FRAMEBUFFER_STRIDE}
  bl paint2
9:
  bl {name}
  ldr x30, ={site + 4:#x}
  ret
""")
    globals_ = '\n'.join(f'.global _zstage{index}' for index in range(len(ZSTAGE_CALLS)))
    source = f'.text\n.global _start\n{globals_}\n' + '\n'.join(body)
    symbols = {name: names[name] for name, _ in ZSTAGE_CALLS}
    symbols['paint2'] = paint2_va
    code, load, labels = assemble(source, va, symbols, workdir)
    if load != va or labels.get('_start') != va or any(f'_zstage{index}' not in labels for index in range(len(ZSTAGE_CALLS))):
        raise SystemExit('The kalloc zone cave did not link where expected')
    text = f'kalloc zone stages at {va:#x} ({len(code):#x} bytes)'
    if heap is not None:
        text += f', {heap} heap only (w21 == {heap_id})'
    return [(site, labels[f'_zstage{index}']) for index, site in enumerate(sites)], code, text


# (function, callee, instruction kind, colour); one mark each, painted before the
# call. The last branch is the tail call into the fixed zone creation, so its
# stub must jump, not return.
FIXED_STAGE_CALLS = (
    ('_kalloc_type_view_init_fixed', '_early_random', 'bl', 0xFFFFFF00),
    ('_kalloc_type_view_init_fixed', '_kalloc_type_view_parse', 'bl', 0xFFFFFF00),
    ('_kalloc_type_view_init_fixed', '_qsort', 'bl', 0xFFFFFF00),
    ('_kalloc_type_view_init_fixed', '_kalloc_type_create_iterators_fixed', 'bl', 0xFFFFFF00),
    ('_kalloc_type_view_init_fixed', '_kalloc_type_create_zones_fixed', 'b', 0xFFFFFF00),
    ('_kalloc_type_create_zones_fixed', '_kalloc_type_apply_policy', 'bl', 0xFF00FFFF),
    ('_kalloc_type_create_zones_fixed', '_kalloc_type_create_zone_for_size', 'bl', 0xFF00FFFF),
    ('_kalloc_type_create_zones_fixed', '_kalloc_type_init_sig_eq', 'bl', 0xFF00FFFF),
    ('_kalloc_type_create_zones_fixed', '_kalloc_type_distribute_zone_for_type', 'bl', 0xFF00FFFF),
    ('_kalloc_type_create_zones_fixed', '_kmem_shuffle', 'bl', 0xFF00FFFF),
    ('_kalloc_type_create_zone_for_size', '_zalloc_permanent_tag', 'bl', 0xFFFF00FF),
    ('_kalloc_type_create_zone_for_size', '_snprintf', 'bl', 0xFFFF00FF),
    ('_kalloc_type_create_zone_for_size', '_zone_create', 'bl', 0xFFFF00FF),
    ('_kalloc_type_create_zone_for_size', '_kalloc_zone_for_size', 'bl', 0xFFFF00FF),
    ('_kalloc_type_create_zone_for_size', '_zone_create_ext', 'bl', 0xFFFF00FF),
)


def kalloc_fixed_stages(data, names, workdir, va):
    """Marks on kalloc_type_view_init_fixed and the fixed zone creation below it.

    Each stub paints when its call is entered, so the last painted mark is the
    call that never returned (or never got reached). The fixed-init row keeps
    the colours by function: yellow for kalloc_type_view_init_fixed, cyan for
    kalloc_type_create_zones_fixed, magenta for kalloc_type_create_zone_for_size.
    The loop sites repaint the same cell on every iteration, so they only prove
    the first iteration reached them."""
    missing = sorted({name for _, name, _, _ in FIXED_STAGE_CALLS} - set(names))
    if missing:
        raise SystemExit('The kalloc fixed stage probe needs symbols this kernel lacks: ' + ', '.join(missing))
    scratch = init_entry_set(data)[0] + 16
    base = FIXEDSTAGE_Y * FRAMEBUFFER_STRIDE + FIXEDSTAGE_X * 4 - WINDOW_OFF
    end = (base + (len(FIXED_STAGE_CALLS) - 1) * FIXEDSTAGE_PITCH_X * 4
           + (CELL_H - 1) * FRAMEBUFFER_STRIDE + CELL_W * 4)
    if FIXEDSTAGE_X + (len(FIXED_STAGE_CALLS) - 1) * FIXEDSTAGE_PITCH_X + CELL_W > 1600 or end > WINDOW_LEN:
        raise SystemExit('The kalloc fixed stage marks fall outside the mapped window')
    sites = []
    starts = sorted(set(names.values()))

    def function_end(function):
        low = names[function]
        return next(va for va in starts if va > low)

    for function, name, kind, _ in FIXED_STAGE_CALLS:
        low, high = names[function], function_end(function)
        target = names[name]
        opcode = 0b100101 if kind == 'bl' else 0b000101
        hit = None
        for at in range(low, high, 4):
            word = struct.unpack_from('<I', data, file_offset(at))[0]
            if word >> 26 != opcode:
                continue
            delta = word & 0x3FFFFFF
            delta -= (1 << 26) if delta & (1 << 25) else 0
            if at + delta * 4 == target:
                hit = at
                break
        if hit is None:
            raise SystemExit(f'{function} has no {kind} to {name} between {low:#x} and {high:#x}')
        sites.append(hit)
    body = []
    for index, ((_, name, kind, color), site) in enumerate(zip(FIXED_STAGE_CALLS, sites)):
        label = '_start:\n_fixed0' if index == 0 else f'_fixed{index}'
        mark = base + index * FIXEDSTAGE_PITCH_X * 4
        paint = f"""  ldr x10, ={scratch:#x}
  ldr x10, [x10]
  tbz x10, #48, 9f
{mark_load('w8', mark)}  add x8, x10, x8
  movz w9, #{color & 0xffff:#x}
  movk w9, #{color >> 16:#x}, lsl #16
  mov x10, #{CELL_W * 4}
  mov x11, #{CELL_H}
  mov x12, {FRAMEBUFFER_STRIDE}
  bl paint2
9:
"""
        if kind == 'bl':
            body.append(f"""{label}:
{paint}  bl {name}
  ldr x30, ={site + 4:#x}
  ret
""")
        else:
            body.append(f"""{label}:
{paint}  b {name}
""")
    globals_ = '\n'.join(f'.global _fixed{index}' for index in range(len(FIXED_STAGE_CALLS)))
    source = f'.text\n.global _start\n{globals_}\n' + '\n'.join(body) + PAINT2_ASM
    symbols = {name: names[name] for _, name, _, _ in FIXED_STAGE_CALLS}
    code, load, labels = assemble(source, va, symbols, workdir)
    if load != va or labels.get('_start') != va or any(
            f'_fixed{index}' not in labels for index in range(len(FIXED_STAGE_CALLS))):
        raise SystemExit('The kalloc fixed stage cave did not link where expected')
    if 'paint2' not in labels:
        raise SystemExit('The kalloc fixed stage cave lost its paint helper')
    return ([(site, labels[f'_fixed{index}']) for index, site in enumerate(sites)], code,
            f'kalloc fixed stages at {va:#x} ({len(code):#x} bytes)')


PARSE_HOOK_OFFSETS = (
    # (offset in _kalloc_type_view_parse, original instruction, cave label, what)
    (0x1c4, 0x97ffff05, '_parse0', 'the KCGEN view copy call'),
    (0x1e8, 0xaa0003f9, '_parse1', 'the prelink section result'),
    (0x228, 0xf94016c8, '_parse2', 'the __macho_list result'),
    (0x370, 0x910082b3, '_kextscan', 'the kext header scan'),
    (0x494, 0x97fffe51, '_kextcopy', 'the kext view copy call'),
    (0x4e8, 0xf9402be0, '_parse3', 'the parse return'),
)
VIEWBAR_OFFSET = (0x80, 0x7100071f)     # cmp w24, #1 at the view copy loop head


def parse_stages(data, kernel, names, workdir, va, identity=False, memory=False,
                 late_fastboot=False, fastboot_delay=60):
    """Marks inside kalloc_type_view_parse and the per-view copy loop.

    A self-test (without the startup-entry grid) maps the frame buffer, then:
      parse row   x=32 yellow->green  xnu __kalloc_type copy entered/returned
                  x=52, 72 cyan       prelink section / __macho_list found
                  x=92 green          parse returned
      view row    one yellow cell per 32 copied non-data views (*cur_count >> 5)
      kext row    one cell per prelinked kext: yellow header scanned, cyan copy
                  entered, green copy returned
      x=560       red                 panic() entered
    A cell that keeps its entry colour marks the call that never returned.

    With identity, six 64-cell white-on-black bitmap rows at y=1170..1250 carry
    raw values from the first kext view copy and from the panic entry:
      y=1170  x1, the row start passed to kalloc_type_view_copy
      y=1186  [x1+0x28] kt_flags (x=32) and [x1+0x2c] kt_size (x=480)
      y=1202  [x1+0x10] kt_zv.zv_name
      y=1218  [x0]    first 8 bytes of the panic format string
      y=1234  [x0+16] next 8 bytes of the panic format string
      y=1250  x30 at panic() entry, the caller of panic()
    The rows are painted just before the wrapped call; if the reads fault, the
    panic red mark still lands and the last completed row says how far it got.

    The same rows repeat in the clean white band at x=808 so the photo carries
    readable values, together with the runtime translation of x1:
      y=1234  x1 again
      y=1248  gVirtBase
      y=1262  gPhysBase
      y=1276  kvtophys(x1)
      y=1290  [x1+0x28] kt_flags (x=808) and low word of [x1+0x10] (x=1192)
      y=1304  [x1+0x68] kt_flags (x=808) and low word of [x1+0x50] (x=1192)
      y=1316  x1 at panic() entry, the failing view's kt_name
    and transport canaries at x=808 in the dim band:
      y=1170  qword at file 0x2000000 (32 MiB, __PLK_TEXT_EXEC)
      y=1186  first qword of __PRELINK_INFO (file 0x3033000, 50.5 MiB)
      y=1202  x2 at panic() entry (the kext uuid, NULL for this path)
      y=1218  kvtophys(gVirtBase), a control that must equal gPhysBase

    With memory (needs identity), the x=808 rows after _vm_fault_init stay
    (PAR/slide/name/flags/page), and the late rows move to x=32, where the
    purple band decodes cleanly. The first two repeat the diag57 mapping
    check; the remaining ten XOR the window qword with the file's qword at
    ascending file offsets, so an all-black row means the boot window still
    holds the file bytes there:
      y=1120  x1, the first kext view VA
      y=1133  PAR_EL1 of x1
      y=1146  file 0x1fffff8  __PLK_TEXT_EXEC, just below 32 MiB
      y=1159  file 0x2000000  the 32 MiB canary
      y=1172  file 0x2000008  just past 32 MiB
      y=1185  file 0x2010000  32 MiB + 64 KiB
      y=1198  file 0x2600000  38 MiB
      y=1211  file 0x2a00000  42 MiB
      y=1224  file 0x2b8e000  __PLK_DATA_CONST record page
      y=1237  file 0x2e66000  __PLK_DATA_CONST late
      y=1250  file 0x2edfec8  __PRELINK_DATA
      y=1263  file 0x3033000  __PRELINK_INFO
    """
    needed = ('_kalloc_type_view_parse', '_kalloc_type_view_copy', '_vm_fault_init',
              '_ml_map_high_window', '_panic')
    if identity:
        needed += ('_kvtophys', '_gVirtBase', '_gPhysBase')
    if memory:
        needed += ('_vm_kernel_slide',)
    if late_fastboot or memory:
        needed += ('_ml_io_map',)
    missing = [name for name in needed if name not in names]
    if missing:
        raise SystemExit('The parse stage probe needs symbols this kernel lacks: ' + ', '.join(missing))
    parse = names['_kalloc_type_view_parse']
    copy = names['_kalloc_type_view_copy']
    symbols = kernel_symbols(kernel)
    fault_site, _ = find_call(data, symbols, '_vm_mem_bootstrap', '_vm_fault_init', 1)
    scratch = init_entry_set(data)[0] + 16

    def expect(site, word, what):
        actual = struct.unpack_from('<I', data, file_offset(site))[0]
        if actual != word:
            raise SystemExit(f'{site:#x} holds {actual:#010x}, not the expected instruction of {what}')
        return site

    for offset, word, _, what in PARSE_HOOK_OFFSETS:
        expect(parse + offset, word, what)
    for offset in (0x1c4, 0x494):
        site = parse + offset
        if bl_target(struct.unpack_from('<I', data, file_offset(site))[0], site) != copy:
            raise SystemExit(f'{site:#x} is not a call to _kalloc_type_view_copy')
    viewbar_site = expect(copy + VIEWBAR_OFFSET[0], VIEWBAR_OFFSET[1], 'the view copy loop head')
    panic_site = expect(names['_panic'], 0xd10083ff, 'panic()')
    if KEXT_PITCH_X * 4 != 48 or VIEWBAR_PITCH_X * 4 != 48:
        raise SystemExit('The cell index math assumes a 12 px pitch')

    white = fb_offset(WHITE_X, WHITE_Y, WHITE_W, WHITE_H, 'self-test bar')
    parse_marks = [fb_offset(PARSE_X + index * PARSE_PITCH_X, PARSE_Y, CELL_W, CELL_H,
                             'parse marks') for index in range(4)]
    viewbar = fb_offset(VIEWBAR_X, VIEWBAR_Y, CELL_W, CELL_H, 'view bar')
    fb_offset(VIEWBAR_X + (VIEWBAR_CELLS - 1) * VIEWBAR_PITCH_X, VIEWBAR_Y,
              CELL_W, CELL_H, 'view bar')
    kext = fb_offset(KEXT_X, KEXT_Y, CELL_W, CELL_H, 'kext cells')
    fb_offset(KEXT_X + (KEXT_CELLS - 1) * KEXT_PITCH_X, KEXT_Y, CELL_W, CELL_H,
              'kext cells')
    panic_mark = fb_offset(PANIC_X, PANIC_Y, CELL_W, CELL_H, 'panic mark')

    def paint_block(index_setup, mark, color):
        """One cell through paint2; x0-x7 and x19-x28 survive."""
        block = (
            f'  ldr x10, ={scratch:#x}\n'
            '  ldr x10, [x10]\n'
            '  tbz x10, #48, 9f\n'
            + index_setup
            + mark_load('w8', mark) + '  add x8, x10, x8\n'
        )
        if index_setup:
            block += '  add x8, x8, x9\n'
        block += (
            f'  movz w9, #{color & 0xffff:#x}\n'
            f'  movk w9, #{color >> 16:#x}, lsl #16\n'
            f'  mov x10, #{CELL_W * 4}\n'
            f'  mov x11, #{CELL_H}\n'
            f'  mov x12, {FRAMEBUFFER_STRIDE}\n'
            '  bl paint2\n'
            '9:\n'
        )
        return block

    kext_index = (
        '  sub x9, x27, #1\n'            # list_index is already advanced
        '  lsl x9, x9, #4\n'
        '  add x9, x9, x9, lsl #1\n'     # 12 px per cell
    )
    view_index = (
        '  ldr x9, [x22]\n'              # shared non-data view count
        '  lsr x9, x9, #5\n'
        f'  mov w11, #{VIEWBAR_CELLS - 1}\n'
        '  cmp x9, x11\n'
        '  csel x9, x9, x11, lo\n'
        '  lsl x9, x9, #4\n'
        '  add x9, x9, x9, lsl #1\n'
    )

    identity_kext = identity_panic = identity_helper = identity_early = ''
    if identity:
        def bit_row(x, y, cells):
            return fb_offset(x, y, (cells - 1) * IDENTITY_PITCH_X + CELL_W,
                             CELL_H, 'identity row')

        def paint_value(load, first, bits):
            """x19 := the loaded value, then one row through _paintrow."""
            return (load
                    + f'  ldr x9, ={first:#x}\n'
                    '  add x21, x22, x9\n'
                    f'  mov x20, #{bits}\n'
                    '  bl _paintrow\n')

        row_sec = bit_row(IDENTITY_X, IDENTITY_Y, 64)
        row_flags = bit_row(IDENTITY_X, IDENTITY_Y + 16, 32)
        row_size = bit_row(IDENTITY_X + 480, IDENTITY_Y + 16, 32)
        row_name = bit_row(IDENTITY_X, IDENTITY_Y + 32, 64)
        row_fmt0 = bit_row(IDENTITY_X, IDENTITY_Y + 48, 64)
        row_fmt16 = bit_row(IDENTITY_X, IDENTITY_Y + 64, 64)
        row_x30 = bit_row(IDENTITY_X, IDENTITY_Y + 80, 64)
        # Clean-band copies at x=808 (readable on the photo), plus canaries.
        clean = 808
        row_sec2 = bit_row(clean, IDENTITY_Y + 64, 64)
        row_virt = bit_row(clean, IDENTITY_Y + 78, 64)
        row_phys = bit_row(clean, IDENTITY_Y + 92, 64)
        row_kvtop = bit_row(clean, IDENTITY_Y + 106, 64)
        row_flags0 = bit_row(clean, IDENTITY_Y + 120, 32)
        row_name0 = bit_row(clean + 384, IDENTITY_Y + 120, 32)
        row_flags1 = bit_row(clean, IDENTITY_Y + 134, 32)
        row_name1 = bit_row(clean + 384, IDENTITY_Y + 134, 32)
        row_panic_name = bit_row(clean, IDENTITY_Y + 146, 64)
        row_canary32 = bit_row(clean, IDENTITY_Y, 64)
        row_canary_info = bit_row(clean, IDENTITY_Y + 16, 64)
        row_ctrl = bit_row(clean, IDENTITY_Y + 48, 64)
        row_panic_x2 = bit_row(clean, IDENTITY_Y + 32, 64)
        canary32 = va_for_file(0x2000000)
        canary_info = next(vmaddr for name, vmaddr, _, _, _ in SEGMENTS
                           if name == '__PRELINK_INFO')
        virt_base, phys_base = names['_gVirtBase'], names['_gPhysBase']
        for canary, what in ((canary32, 'file 0x2000000'), (canary_info, '__PRELINK_INFO')):
            if struct.unpack_from('<Q', data, file_offset(canary))[0] == 0:
                raise SystemExit(f'The transport canary at {what} is zero')

        identity_kext = f'''  stp x19, x20, [sp, #-96]!
  stp x21, x22, [sp, #16]
  stp x0, x1, [sp, #32]
  stp x2, x3, [sp, #48]
  stp x4, x5, [sp, #64]
  stp x6, x7, [sp, #80]
  ldr x10, ={scratch:#x}
  ldr x10, [x10]
  mov x22, x10
  tbz x22, #48, 7f
''' + ''.join((
            paint_value('  mov x19, x1\n', row_sec, 64),
            paint_value('  ldr w19, [x1, #0x28]\n  lsl x19, x19, #32\n', row_flags, 32),
            paint_value('  ldr w19, [x1, #0x2c]\n  lsl x19, x19, #32\n', row_size, 32),
            paint_value('  ldr x19, [x1, #0x10]\n', row_name, 64),
            paint_value('  mov x19, x1\n', row_sec2, 64),
            paint_value(f'  ldr x9, ={virt_base:#x}\n  ldr x19, [x9]\n', row_virt, 64),
            paint_value(f'  ldr x9, ={phys_base:#x}\n  ldr x19, [x9]\n', row_phys, 64),
            paint_value('  ldr w19, [x1, #0x28]\n  lsl x19, x19, #32\n', row_flags0, 32),
            paint_value('  ldr w19, [x1, #0x10]\n  lsl x19, x19, #32\n', row_name0, 32),
            paint_value('  ldr w19, [x1, #0x68]\n  lsl x19, x19, #32\n', row_flags1, 32),
            paint_value('  ldr w19, [x1, #0x50]\n  lsl x19, x19, #32\n', row_name1, 32),
            paint_value(f'  ldr x9, ={canary32:#x}\n  ldr x19, [x9]\n', row_canary32, 64),
            paint_value(f'  ldr x9, ={canary_info:#x}\n  ldr x19, [x9]\n', row_canary_info, 64),
            paint_value('  mov x0, x1\n  bl _kvtophys\n  mov x19, x0\n  ldr x1, [sp, #40]\n',
                        row_kvtop, 64),
            paint_value(f'  ldr x9, ={virt_base:#x}\n  ldr x0, [x9]\n  bl _kvtophys\n'
                        '  mov x19, x0\n', row_ctrl, 64),
        )) + '''7:
  ldp x6, x7, [sp, #80]
  ldp x4, x5, [sp, #64]
  ldp x2, x3, [sp, #48]
  ldp x0, x1, [sp, #32]
  ldp x21, x22, [sp, #16]
  ldp x19, x20, [sp], #96
'''
        identity_panic = f'''  ldr x10, ={scratch:#x}
  ldr x10, [x10]
  mov x22, x10
  tbz x22, #48, 7f
''' + ''.join((
            paint_value('  ldr x19, [sp, #8]\n', row_x30, 64),
            paint_value('  ldr x19, [x0]\n', row_fmt0, 64),
            paint_value('  ldr x19, [x0, #16]\n', row_fmt16, 64),
            paint_value('  mov x19, x1\n', row_panic_name, 64),
            paint_value('  mov x19, x2\n', row_panic_x2, 64),
        )) + '''7:
'''
        identity_helper = f'''
.global _paintrow
_paintrow:                      /* x19 = value, x20 = bits, x21 = first pixel */
  stp x29, x30, [sp, #-16]!
1:
  tbnz x19, #63, 2f
  mov w9, #0xff000000
  b 3f
2:
  movn w9, #0
3:
  mov x8, x21
  mov x10, #{CELL_W * 4}
  mov x11, #{CELL_H}
  mov x12, {FRAMEBUFFER_STRIDE}
  bl paint2
  add x21, x21, #{IDENTITY_PITCH_X * 4}
  lsl x19, x19, #1
  subs x20, x20, #1
  b.ne 1b
  ldp x29, x30, [sp], #16
  ret
'''
        if memory:
            # The first prelinked kext's __kalloc_type record, as the kalloc
            # parser sees it: section base + vm_kernel_slide (slide 0 here).
            plk = 0xfffffff005cb3158
            page = plk & ~0xfff
            slide = names['_vm_kernel_slide']
            record = file_offset(plk)

            def qword(at):
                return struct.unpack_from('<Q', data, at)[0]

            file_name = qword(record + 0x10)
            file_flags = qword(record + 0x28)
            file_second = qword(record + 0x68)
            if (file_name == 0 or file_flags & 0xffffffff != 0x61 or
                    file_second != 0x000000c000000061):
                raise SystemExit('The kext __kalloc_type views are not where the memory probe expects them')
            if qword(file_offset(page)) == 0:
                raise SystemExit('The __PLK_DATA_CONST page canary is zero')

            # Ascending file offsets through the coalesced kext segments. The
            # 32 MiB canary measured intact in diag57 while the 45.7 MiB record
            # was zero, so the scan brackets that boundary and every segment
            # after it. Each row is window_qword XOR file_qword: black = match.
            scan = (0x1fffff8, 0x2000000, 0x2000008, 0x2010000, 0x2600000,
                    0x2a00000, 0x2b8e000, 0x2e66000, 0x2edfec8, 0x3033000)

            def sample_row(file, row):
                expected = qword(file)
                if expected == 0:
                    raise SystemExit(f'The scan sample at file {file:#x} is zero in the file')
                address = va_for_file(file)
                return paint_value(f'  ldr x9, ={address:#x}\n  ldr x19, [x9]\n'
                                   f'  ldr x9, ={expected:#x}\n  eor x19, x19, x9\n',
                                   row, 64)

            def mem_row(y):
                return bit_row(clean, y, 64)

            # The late rows sit in the purple band at x=32 (13 px apart), where
            # the photo decodes every cell; rows at x=808 drift past cell 44.
            late_rows = [bit_row(IDENTITY_X, IDENTITY_Y - 50 + 13 * index, 64)
                         for index in range(12)]
            latch_mark = fb_offset(1540, 1120, 16, 8, 'fastboot latch marker')

            def mem_read(address, offset=0):
                return f'  ldr x9, ={address + offset:#x}\n  ldr x19, [x9]\n'

            def mem_par(address):
                return f'  ldr x9, ={address:#x}\n  at s1e1r, x9\n  isb\n  mrs x23, par_el1\n  mov x19, x23\n'

            save = '''  stp x19, x20, [sp, #-144]!
  stp x21, x22, [sp, #16]
  stp x23, x24, [sp, #32]
  stp x25, x26, [sp, #48]
  stp x0, x1, [sp, #64]
  stp x2, x3, [sp, #80]
  stp x4, x5, [sp, #96]
  stp x6, x7, [sp, #112]
'''
            restore = '''  ldp x6, x7, [sp, #112]
  ldp x4, x5, [sp, #96]
  ldp x2, x3, [sp, #80]
  ldp x0, x1, [sp, #64]
  ldp x25, x26, [sp, #48]
  ldp x23, x24, [sp, #32]
  ldp x21, x22, [sp, #16]
  ldp x19, x20, [sp], #144
'''
            guard = f'''  ldr x22, ={scratch:#x}
  ldr x22, [x22]
  tbz x22, #48, 7f
'''
            identity_early = f'''{save}{guard}''' + ''.join((
                paint_value(mem_par(plk), mem_row(IDENTITY_Y), 64),
                paint_value(mem_read(slide), mem_row(IDENTITY_Y + 14), 64),
            )) + '''  tbnz x23, #0, 7f
''' + ''.join((
                paint_value(mem_read(plk, 0x10), mem_row(IDENTITY_Y + 28), 64),
                paint_value(mem_read(plk, 0x28), mem_row(IDENTITY_Y + 42), 64),
                paint_value(mem_read(page), mem_row(IDENTITY_Y + 56), 64),
            )) + f'''7:
{restore}'''
            identity_kext = f'''{save}{guard}''' + ''.join((
                paint_value('  mov x19, x1\n', late_rows[0], 64),
                paint_value(mem_par(plk), late_rows[1], 64),
            )) + ''.join(sample_row(file, late_rows[index + 2])
                         for index, file in enumerate(scan))
            if late_fastboot:
                # After the last row: arm the reboot-mode syscon and reset into
                # LK fastboot, so the next image needs no key combination. The
                # marker is green when the syscon was written, red when the map
                # failed; the delay leaves time for the photo.
                identity_kext += f'''  ldr x0, ={WDT_PA:#x}
  ldr x1, =0x1000
  bl _ml_io_map
  cbz x0, 6f
  ldr w9, [x0, #{WDT_MODE_OFF:#x}]
  and w9, w9, #{~WDT_MODE_MASK & 0xffffffff:#x}
  orr w9, w9, #{WDT_MODE_BOOTLOADER:#x}
  str w9, [x0, #{WDT_MODE_OFF:#x}]
  dsb sy
  ldr x8, ={latch_mark:#x}
  add x8, x22, x8
  movz w9, #{COLOR_GREEN & 0xffff:#x}
  movk w9, #{COLOR_GREEN >> 16:#x}, lsl #16
  mov x10, #{16 * 4}
  mov x11, #8
  mov x12, {FRAMEBUFFER_STRIDE}
  bl paint2
  isb
  mrs x1, cntpct_el0
  ldr x2, ={TIMEBASE * fastboot_delay:#x}
  add x2, x1, x2
1:
  isb
  mrs x1, cntpct_el0
  cmp x1, x2
  b.lo 1b
  movz w0, #0x0009
  movk w0, #0x8400, lsl #16
  mov x1, xzr
  mov x2, xzr
  mov x3, xzr
  smc #0
2:
  wfe
  b 2b
6:
  ldr x8, ={latch_mark:#x}
  add x8, x22, x8
  movz w9, #{COLOR_RED & 0xffff:#x}
  movk w9, #{COLOR_RED >> 16:#x}, lsl #16
  mov x10, #{16 * 4}
  mov x11, #8
  mov x12, {FRAMEBUFFER_STRIDE}
  bl paint2
'''
            identity_kext += f'''7:
{restore}'''
            identity_panic = f'''  ldr x10, ={scratch:#x}
  ldr x10, [x10]
  mov x22, x10
  tbz x22, #48, 7f
''' + ''.join((
                paint_value('  ldr x19, [sp, #8]\n', row_x30, 64),
                paint_value('  ldr x19, [x0]\n', row_fmt0, 64),
                paint_value('  ldr x19, [x0, #16]\n', row_fmt16, 64),
            )) + '''7:
'''
        if identity_kext.count('bl _paintrow') != (12 if memory else 15) or \
                identity_panic.count('bl _paintrow') != (3 if memory else 5) or \
                (memory and identity_early.count('bl _paintrow') != 5):
            raise SystemExit('The identity probe lost a paintrow call')
    panic_stub = f'''  stp x29, x30, [sp, #-48]!
  stp x19, x20, [sp, #16]
  stp x21, x22, [sp, #32]
{paint_block('', panic_mark, COLOR_RED)}{identity_panic}  ldp x21, x22, [sp, #32]
  ldp x19, x20, [sp, #16]
  ldp x29, x30, [sp], #48
''' if identity else f'''  stp x29, x30, [sp, #-16]!
{paint_block('', panic_mark, COLOR_RED)}  ldp x29, x30, [sp], #16
'''

    source = f'''.text
.global _start
.global _parse0
.global _parse1
.global _parse2
.global _parse3
.global _viewbar
.global _kextscan
.global _kextcopy
.global _panicentry
_start:                         /* runs instead of `bl _vm_fault_init` */
  stp x29, x30, [sp, #-16]!
  bl _vm_fault_init
  ldr x0, ={WINDOW_PA:#x}
  ldr x1, ={WINDOW_LEN:#x}
  bl _ml_map_high_window
  cbz x0, 8f
  ldr x9, ={scratch:#x}
  str x0, [x9]
  ldr x14, ={white:#x}
  add x0, x0, x14
  movn w1, #0
  mov x2, #{WHITE_W * 4}
  mov x3, #{WHITE_H}
  mov x4, {FRAMEBUFFER_STRIDE}
  bl paint
{identity_early}8:
  ldp x29, x30, [sp], #16
  ldr x30, ={fault_site + 4:#x}
  ret

_parse0:                        /* xnu __kalloc_type copy: yellow in, green out */
{paint_block('', parse_marks[0], COLOR_YELLOW)}  bl _kalloc_type_view_copy
{paint_block('', parse_marks[0], COLOR_GREEN)}  ldr x30, ={parse + 0x1c8:#x}
  ret

_parse1:                        /* prelink section found */
{paint_block('', parse_marks[1], COLOR_CYAN)}  mov x25, x0
  ldr x30, ={parse + 0x1ec:#x}
  ret

_parse2:                        /* __macho_list present */
{paint_block('', parse_marks[2], COLOR_CYAN)}  ldr x8, [x22, #0x28]
  ldr x30, ={parse + 0x22c:#x}
  ret

_parse3:                        /* parse returning */
{paint_block('', parse_marks[3], COLOR_GREEN)}  ldr x0, [sp, #0x50]
  ldr x30, ={parse + 0x4ec:#x}
  ret

_viewbar:                       /* one cell per 32 copied views */
{paint_block(view_index, viewbar, COLOR_YELLOW)}  cmp w24, #1
  ldr x30, ={viewbar_site + 4:#x}
  ret

_kextscan:                      /* kext k header validated */
{paint_block(kext_index, kext, COLOR_YELLOW)}  add x19, x21, #0x20
  ldr x30, ={parse + 0x374:#x}
  ret

_kextcopy:                      /* kext k view copy: cyan in, green out */
{paint_block(kext_index, kext, COLOR_CYAN)}{identity_kext}  bl _kalloc_type_view_copy
{paint_block(kext_index, kext, COLOR_GREEN)}  ldr x30, ={parse + 0x498:#x}
  ret

_panicentry:                    /* panic() entered */
{panic_stub}  sub sp, sp, #0x20
  b _panic_after

/* x0 = first pixel, w1 = colour, x2 = bytes per row, x3 = rows, x4 = stride. */
paint:
  mov x5, x0
  mov x9, x3
3:
  mov x10, x5
  mov x7, x2
4:
  str w1, [x10], #4
  subs x7, x7, #4
  b.ne 4b
  bic x8, x5, #63
  add x11, x5, x2
  add x11, x11, #63
  bic x11, x11, #63
5:
  dc civac, x8
  add x8, x8, #64
  cmp x8, x11
  b.lo 5b
  add x5, x5, x4
  subs x9, x9, #1
  b.ne 3b
  dsb sy
  ret
{identity_helper}
{PAINT2_ASM}''' 

    cave_symbols = {
        '_vm_fault_init': names['_vm_fault_init'],
        '_ml_map_high_window': names['_ml_map_high_window'],
        '_kalloc_type_view_copy': copy,
        '_panic_after': names['_panic'] + 4,
    }
    if identity:
        cave_symbols['_kvtophys'] = names['_kvtophys']
    if late_fastboot or memory:
        cave_symbols['_ml_io_map'] = names['_ml_io_map']
    code, load, labels = assemble(source, va, cave_symbols, workdir)
    if load != va or labels.get('_start') != va:
        raise SystemExit('The parse stage cave did not link where expected')
    required = ('_parse0', '_parse1', '_parse2', '_parse3', '_viewbar',
                '_kextscan', '_kextcopy', '_panicentry')
    if identity:
        required += ('_paintrow',)
    for label in required:
        if label not in labels:
            raise SystemExit(f'The parse stage cave lost {label}')
    links = [(fault_site, labels['_start'])]
    for offset, _, label, _ in PARSE_HOOK_OFFSETS:
        links.append((parse + offset, labels[label]))
    links.append((viewbar_site, labels['_viewbar']))
    links.append((panic_site, labels['_panicentry']))
    return links, code, (f'parse stages at {va:#x} ({len(code):#x} bytes); '
                         f'self-test at {fault_site:#x}, view copy at {copy:#x}')


def zcreate_stages(data, name_addrs, workdir, va, paint2_va):
    """Colour-coded marks along zone_create_ext's main path (first kalloc zone hangs there)."""
    zcreate = name_addrs.get('_zone_create_ext')
    missing = [name for name, _, _ in ZCREATE_STAGES if name and name not in name_addrs]
    if zcreate is None or missing:
        raise SystemExit(f'The zone_create probe needs symbols this kernel lacks: {", ".join(missing) or "_zone_create_ext"}')
    scratch = init_entry_set(data)[0] + 16
    base = ZCREATE_Y * FRAMEBUFFER_STRIDE + ZCREATE_X * 4 - WINDOW_OFF
    marks = [base + index * ZCREATE_PITCH_X * 4 for index in range(len(ZCREATE_STAGES))]
    # The final lock site also reports the ticket lock's type byte and its return,
    # two extra cells after the six stage marks.
    type_mark = base + len(ZCREATE_STAGES) * ZCREATE_PITCH_X * 4
    return_mark = type_mark + ZCREATE_PITCH_X * 4
    end = (return_mark + (CELL_H - 1) * FRAMEBUFFER_STRIDE + CELL_W * 4)
    if ZCREATE_X + (len(ZCREATE_STAGES) + 1) * ZCREATE_PITCH_X + CELL_W > 1600 or end > WINDOW_LEN:
        raise SystemExit('The zone_create lock marks fall outside the mapped window')
    sites = [zcreate + offset for _, offset, _ in ZCREATE_STAGES]
    for site, (name, _, _) in zip(sites, ZCREATE_STAGES):
        word = struct.unpack_from('<I', data, file_offset(site))[0]
        if name is None:
            if word != 0xD63F0100:                       # blr x8
                raise SystemExit(f'{site:#x} is not the expected blr x8 of the extra_setup block')
        elif word & 0xFC000000 != 0x94000000 or bl_target(word, site) != name_addrs[name]:
            raise SystemExit(f'{site:#x} is not the expected call to {name}')
    body = []
    for index, ((name, _, color), mark, site) in enumerate(zip(ZCREATE_STAGES, marks, sites)):
        label = '_start:\n_zcstage0' if index == 0 else f'_zcstage{index}'
        if index == len(ZCREATE_STAGES) - 1:
            # Entered before hw_lck_ticket_lock: blue mark; if the lock's type byte
            # is the ticket type (0x44) a white mark; if the lock returns a yellow
            # mark. x19 keeps the window base across the lock call (it is
            # callee-saved there) and is restored before returning to the caller.
            body.append(f"""{label}:
  sub sp, sp, #0x10
  str x19, [sp]
  ldr x19, ={scratch:#x}
  ldr x19, [x19]
  tbz x19, #48, 9f
{mark_load('w8', mark)}  add x8, x19, x8
  movz w9, #0x00ff
  movk w9, #0xff00, lsl #16
  mov x10, #{CELL_W * 4}
  mov x11, #{CELL_H}
  mov x12, {FRAMEBUFFER_STRIDE}
  bl paint2
  ldrb w11, [x0]
  cmp w11, #0x44
  b.ne 9f
{mark_load('w8', type_mark)}  add x8, x19, x8
  movn w9, #0
  mov x10, #{CELL_W * 4}
  mov x11, #{CELL_H}
  mov x12, {FRAMEBUFFER_STRIDE}
  bl paint2
9:
  bl {name}
  cbz x19, 8f
{mark_load('w8', return_mark)}  add x8, x19, x8
  movz w9, #0xff00
  movk w9, #0xffff, lsl #16
  mov x10, #{CELL_W * 4}
  mov x11, #{CELL_H}
  mov x12, {FRAMEBUFFER_STRIDE}
  bl paint2
8:
  ldr x19, [sp]
  add sp, sp, #0x10
  ldr x30, ={site + 4:#x}
  ret
""")
            continue
        if name is None:
            # br x8 tail: keep the block invoker around paint2.
            tail = f"""  ldr x8, [sp]
  add sp, sp, #0x10
  br x8"""
            prologue = """  sub sp, sp, #0x10
  str x8, [sp]
"""
        else:
            tail = f"""  bl {name}
  ldr x30, ={site + 4:#x}
  ret"""
            prologue = ''
        body.append(f"""{label}:
{prologue}  ldr x10, ={scratch:#x}
  ldr x10, [x10]
  tbz x10, #48, 9f
{mark_load('w8', mark)}  add x8, x10, x8
  movz w9, #{color & 0xffff:#x}
  movk w9, #{color >> 16:#x}, lsl #16
  mov x10, #{CELL_W * 4}
  mov x11, #{CELL_H}
  mov x12, {FRAMEBUFFER_STRIDE}
  bl paint2
9:
{tail}
""")
    globals_ = '\n'.join(f'.global _zcstage{index}' for index in range(len(ZCREATE_STAGES)))
    source = f'.text\n.global _start\n{globals_}\n' + '\n'.join(body)
    symbols = {'paint2': paint2_va}
    symbols.update({name: name_addrs[name] for name, _, _ in ZCREATE_STAGES if name})
    code, load, labels = assemble(source, va, symbols, workdir)
    if load != va or labels.get('_start') != va or any(f'_zcstage{index}' not in labels for index in range(len(ZCREATE_STAGES))):
        raise SystemExit('The zone_create cave did not link where expected')
    text = f'zone_create stages at {va:#x} ({len(code):#x} bytes)'
    return [(site, labels[f'_zcstage{index}']) for index, site in enumerate(sites)], code, text


ZTAIL_HOOKS = (
    (0x918, 0x910202a0, 0xFF00FF00, '  add x0, x21, #0x80'),      # z_self store done
    (0x920, 0xf85a03a8, 0xFF00FFFF, '  ldur x8, [x29, #-0x60]'),  # ticket unlock returned
    (0x938, 0xaa1503e0, 0xFFFF00FF, '  mov x0, x21'),             # stack canary passed
)


def ztail_stages(data, name_addrs, workdir, va, paint2_va):
    """Four marks after the final lock: z_self store, unlock return, canary, and
    the return into kalloc_zone_init (the `add x27` after the call)."""
    zcreate = name_addrs.get('_zone_create_ext')
    zone_init = name_addrs.get('_kalloc_zone_init')
    if zcreate is None or zone_init is None:
        raise SystemExit('The zone_create tail probe needs _zone_create_ext and _kalloc_zone_init')
    scratch = init_entry_set(data)[0] + 16
    base = ZCREATE_Y * FRAMEBUFFER_STRIDE + ZCREATE_X * 4 - WINDOW_OFF
    hooks = [(zcreate + offset, expect, color, displaced, offset)
             for offset, expect, color, displaced in ZTAIL_HOOKS]
    # The last hook sits in the caller: the first instruction after the call.
    hooks.append((zone_init + 0xf8, 0x9100077b, 0xFFFF8000, '  add x27, x27, #0x1', 0xf8))
    end_mark = base + (len(ZCREATE_STAGES) + 2 + len(hooks) - 1) * ZCREATE_PITCH_X * 4
    if ZCREATE_X + (len(ZCREATE_STAGES) + 2 + len(hooks) - 1) * ZCREATE_PITCH_X + CELL_W > 1600 or \
            end_mark + (CELL_H - 1) * FRAMEBUFFER_STRIDE + CELL_W * 4 > WINDOW_LEN:
        raise SystemExit('The zone_create tail marks fall outside the mapped window')
    body = []
    for index, (site, expect, color, displaced, _) in enumerate(hooks):
        word = struct.unpack_from('<I', data, file_offset(site))[0]
        if word != expect:
            raise SystemExit(f'{site:#x} holds {word:#010x}, not the expected tail instruction')
        mark = base + (len(ZCREATE_STAGES) + 2 + index) * ZCREATE_PITCH_X * 4
        label = '_start:\n_ztail0' if index == 0 else f'_ztail{index}'
        body.append(f"""{label}:
  ldr x10, ={scratch:#x}
  ldr x10, [x10]
  tbz x10, #48, 9f
{mark_load('w8', mark)}  add x8, x10, x8
  movz w9, #{color & 0xffff:#x}
  movk w9, #{color >> 16:#x}, lsl #16
  mov x10, #{CELL_W * 4}
  mov x11, #{CELL_H}
  mov x12, {FRAMEBUFFER_STRIDE}
  bl paint2
9:
{displaced}
  ldr x30, ={site + 4:#x}
  ret
""")
    globals_ = '\n'.join(f'.global _ztail{index}' for index in range(len(hooks)))
    source = f'.text\n.global _start\n{globals_}\n' + '\n'.join(body)
    code, load, labels = assemble(source, va, {'paint2': paint2_va}, workdir)
    if load != va or labels.get('_start') != va or any(f'_ztail{index}' not in labels for index in range(len(hooks))):
        raise SystemExit('The zone_create tail cave did not link where expected')
    text = f'zone_create tail stages at {va:#x} ({len(code):#x} bytes)'
    return [(site, labels[f'_ztail{index}']) for index, (site, _, _, _, _) in enumerate(hooks)], code, text


def assemble(source, va, symbols, workdir, entry_offset=0):
    asm, obj, elf, binary = (workdir / name for name in ('cave.S', 'cave.o', 'cave.elf', 'cave.bin'))
    asm.write_text(source)
    clang = shutil.which('clang') or '/opt/homebrew/opt/llvm/bin/clang'
    lld = shutil.which('ld.lld') or '/opt/homebrew/opt/llvm/bin/ld.lld'
    objcopy = shutil.which('llvm-objcopy') or '/opt/homebrew/opt/llvm/bin/llvm-objcopy'
    nm = shutil.which('llvm-nm') or '/opt/homebrew/opt/llvm/bin/llvm-nm'
    readelf = shutil.which('llvm-readelf') or '/opt/homebrew/opt/llvm/bin/llvm-readelf'
    subprocess.run([clang, '--target=aarch64-none-elf', '-c', asm, '-o', obj], check=True)
    subprocess.run([lld, f'-Ttext={va:#x}', *[f'--defsym={name}={value:#x}' for name, value in symbols.items()], '-e', '_start', obj, '-o', elf], check=True)
    subprocess.run([objcopy, '-O', 'binary', '-j', '.text', elf, binary], check=True)
    load = None
    for line in subprocess.run([readelf, '-S', elf], capture_output=True, text=True, check=True).stdout.splitlines():
        parts = line.replace('[', ' ').replace(']', ' ').split()
        if len(parts) >= 4 and parts[1] == '.text':
            load = int(parts[3], 16)
    labels = {}
    for line in subprocess.run([nm, elf], capture_output=True, text=True, check=True).stdout.splitlines():
        parts = line.split()
        if len(parts) == 3:
            labels[parts[2]] = int(parts[0], 16)
    if load != va or labels.get('_start') != va + entry_offset:
        raise SystemExit(f'Cave .text starts at {load} / _start at {labels.get("_start")}, not {va:#x}+{entry_offset:#x}')
    return binary.read_bytes(), load, labels


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('kernel', type=Path, help='the COPY of the kernel to patch in place')
    parser.add_argument('--reset-delay', type=int, default=6, help='seconds before the reset')
    parser.add_argument('--reset-after', metavar='SYMBOL', help='place the reset after this call in arm_init instead of stage 3')
    parser.add_argument('--reset-before', action='store_true', help='reset at the call site before invoking SYMBOL (requires --reset-after)')
    parser.add_argument('--panic-screen', action='store_true', help='draw panic() messages on the screen')
    parser.add_argument('--skip-panic-log', action='store_true', help='skip check_for_panic_log() in PE_create_console (it touches the Pi panic-log PA)')
    parser.add_argument('--zalloc-progress', action='store_true', help='paint one mark per ZALLOC startup entry on the frame buffer')
    parser.add_argument('--fastboot-on-reset', action='store_true', help='write the device tree reboot-mode register for LK fastboot before the reset (requires --reset-after and --zalloc-progress)')
    parser.add_argument('--kalloc-stages', action='store_true', help='paint one mark per kalloc_init sub-step (requires --zalloc-progress)')
    parser.add_argument('--kalloc-zone-stages', action='store_true', help='paint one mark per kalloc_zone_init loop sub-call (requires --kalloc-stages)')
    parser.add_argument('--kalloc-zone-heap', choices=('any', 'shared', 'data'), default='any', help='only paint --kalloc-zone-stages for this kalloc heap (default any)')
    parser.add_argument('--kalloc-fixed-stages', action='store_true', help='paint one mark per call inside kalloc_type_view_init_fixed and the fixed zone creation (requires --zalloc-progress)')
    parser.add_argument('--parse-stages', action='store_true', help='paint kalloc_type_view_parse and view copy progress marks (installs its own frame buffer self-test)')
    parser.add_argument('--parse-identity', action='store_true', help='with --parse-stages: add bitmap rows for the first kext view pointer, flags, size, name and the panic format string')
    parser.add_argument('--parse-memory', action='store_true', help='with --parse-identity: replace its clean-band rows with AT S1E1R/PAR_EL1 and kext record contents at the early and late hooks')
    parser.add_argument('--fastboot-after-parse', action='store_true', help='with --parse-memory: after the late rows, write the reboot-mode syscon and PSCI reset into fastboot')
    parser.add_argument('--fastboot-delay', type=int, default=60, help='seconds to leave the rows on screen before --fastboot-after-parse resets (default 60)')
    parser.add_argument('--zcreate-stages', action='store_true', help='paint one mark per zone_create_ext main-path step (requires --kalloc-zone-stages)')
    parser.add_argument('--ztail-stages', action='store_true', help='paint post-lock tail marks in zone_create_ext and its return into kalloc_zone_init (requires --zcreate-stages)')
    parser.add_argument('--entry-check', action='store_true', help='verify the boot window still holds the file bytes at the kernel entry; reset into fastboot on a mismatch (requires --no-reset)')
    parser.add_argument('--entry-check-delay', type=int, default=6, help='seconds to keep the mismatch bar before the fastboot reset (default 6)')
    parser.add_argument('--entry-report', action='store_true', help='like --entry-check but paint one status cell per check and bit rows of the observed values; the reset delay encodes the first failing check (requires --no-reset)')
    parser.add_argument('--entry-scan', action='store_true', help='at the kernel entry scan DRAM for the kext[0] record page in the window, the Image pool copy and the decompressed payload volume; the reset delay encodes which copies exist (requires --no-reset)')
    parser.add_argument('--entry-hunt', action='store_true', help='at the kernel entry compare the window record page with the loader readback at args+0x800 and the fixed volume probe page, paint five status cells and six value rows, then reset into fastboot 60 s after the paints (requires --no-reset)')
    parser.add_argument('--entry-reset-now', action='store_true', help='at the kernel entry arm the reboot-mode syscon and PSCI-reset immediately, with no checks and no wait; the time from reboot to fastboot is the boot-to-entry delay (requires --no-reset)')
    parser.add_argument('--entry-reset-wait-ticks', type=int, default=0, help='with --entry-reset-now, wait this many raw cntpct_el0 ticks before the reset; the measured wait gives CNTFRQ (default 0)')
    parser.add_argument('--no-reset', action='store_true', help='no reset probe at all')
    parser.add_argument('--occurrence', type=int, default=1, help='which call to SYMBOL (1 = first)')
    parser.add_argument('--in', dest='function', default='_arm_init', help='function that makes the call (default _arm_init)')
    args = parser.parse_args()
    if args.reset_before and not args.reset_after:
        raise SystemExit('--reset-before requires --reset-after')
    if args.fastboot_on_reset and (args.no_reset or not args.reset_after or not args.zalloc_progress):
        raise SystemExit('--fastboot-on-reset needs --reset-after and --zalloc-progress (the self-test writes the mode register)')
    if args.kalloc_zone_heap != 'any' and not args.kalloc_zone_stages:
        raise SystemExit('--kalloc-zone-heap needs --kalloc-zone-stages')
    if args.parse_identity and not args.parse_stages:
        raise SystemExit('--parse-identity needs --parse-stages: the identity rows live in its cave')
    if args.parse_memory and not args.parse_identity:
        raise SystemExit('--parse-memory needs --parse-identity: it replaces its clean-band rows')
    if args.fastboot_after_parse and not (args.parse_stages and args.parse_memory):
        raise SystemExit('--fastboot-after-parse needs --parse-stages --parse-memory: it runs after their late rows')
    if args.fastboot_delay < 0:
        raise SystemExit('--fastboot-delay must not be negative')
    if args.entry_check and not args.no_reset:
        raise SystemExit('--entry-check needs --no-reset: a reset stage would hide its outcome')
    if args.entry_report and not args.no_reset:
        raise SystemExit('--entry-report needs --no-reset: a reset stage would hide its outcome')
    if args.entry_report and args.entry_check:
        raise SystemExit('--entry-report replaces --entry-check; pass only one')
    if args.entry_scan and not args.no_reset:
        raise SystemExit('--entry-scan needs --no-reset: a reset stage would hide its outcome')
    if args.entry_scan and (args.entry_check or args.entry_report):
        raise SystemExit('--entry-scan replaces the entry checks; pass only one')
    if args.entry_hunt and not args.no_reset:
        raise SystemExit('--entry-hunt needs --no-reset: it resets into fastboot itself')
    if args.entry_hunt and (args.entry_check or args.entry_report or args.entry_scan or args.entry_reset_now):
        raise SystemExit('--entry-hunt replaces the other entry probes; pass only one')
    if args.entry_reset_now and not args.no_reset:
        raise SystemExit('--entry-reset-now needs --no-reset: a reset stage would hide its outcome')
    if args.entry_reset_now and (args.entry_check or args.entry_report or args.entry_scan or args.entry_hunt):
        raise SystemExit('--entry-reset-now replaces the entry checks; pass only one')
    if args.entry_reset_wait_ticks < 0:
        raise SystemExit('--entry-reset-wait-ticks must not be negative')
    if args.entry_check_delay < 0:
        raise SystemExit('--entry-check-delay must not be negative')
    data = bytearray(args.kernel.read_bytes())
    load_segments(data)
    kernel_names = rebase(args.kernel, data)
    kernel_name_addrs = symbol_name_addresses(args.kernel)

    def word(va):
        return struct.unpack_from('<I', data, file_offset(va))[0]

    expected = {ENTRY_VA: branch(ENTRY_VA, START_FIRST_CPU), MMU_ENABLE_VA: 0xD5181000, AFTER_MMU_VA: 0xD2A00600}
    for va, value in expected.items():
        if word(va) != value:
            raise SystemExit(f'{va:#x} holds {word(va):#010x}, expected {value:#010x}: not the kernel these probes are for')
    for va in CAVES.values():
        if any(data[file_offset(va):file_offset(va) + CAVE_SIZE]):
            raise SystemExit(f'Padding at {va:#x} is not empty')

    # (site, cave stage, symbols the cave is linked with)
    sites = []
    if args.no_reset:
        description = 'no reset probe'
    elif args.reset_after:
        symbols = kernel_symbols(args.kernel)
        call_va, callee = find_call(data, symbols, args.function, args.reset_after, args.occurrence)
        sites.append((call_va, 4, {'callee': callee, 'target': call_va + 4}))
        description = f'stage 4 reset {"before" if args.reset_before else "after"} call #{args.occurrence} to {args.reset_after} in {args.function} (at {call_va:#x})'
    else:
        sites.append((AFTER_MMU_VA, 3, {'target': AFTER_MMU_VA + 4}))
        description = f'stage 3 reset {args.reset_delay} s after the MMU is on'
    if args.skip_panic_log:
        skip_panic_log(data, kernel_names)
        description += '; check_for_panic_log() skipped'

    with tempfile.TemporaryDirectory() as tmp:
        workdir = Path(tmp)
        for site, stage, symbols in sites:
            source = RESET_BEFORE if (stage == 4 and args.reset_before) else SOURCES[stage].format(ticks=TIMEBASE * args.reset_delay)
            code, load, _ = assemble(source, CAVES[stage], symbols, workdir)
            if load != CAVES[stage]:
                raise SystemExit(f'Stage {stage} code was linked at {load:#x}, not {CAVES[stage]:#x}')
            if len(code) > CAVE_SIZE:
                raise SystemExit(f'Stage {stage} code is too large')
            data[file_offset(load):file_offset(load) + len(code)] = code
        cave_end = max((CAVES[stage] + CAVE_SIZE for _, stage, _ in sites), default=CAVES[3] + CAVE_SIZE)
        zsite = zfault = zentry = zselftest = None
        if args.zalloc_progress:
            zsite, zfault, zcave, zcode, labels, text = zalloc_progress(
                data, args.kernel, kernel_names, workdir, args.fastboot_on_reset)
            zentry, zselftest = labels['_start'], labels['_selftest']
            if zcave + len(zcode) > PANIC_CAVE_END:
                raise SystemExit(f'ZALLOC progress is {len(zcode):#x} bytes at {zcave:#x}, the padding ends at {PANIC_CAVE_END:#x}')
            if any(data[file_offset(zcave):file_offset(zcave) + len(zcode)]):
                raise SystemExit(f'Padding at {zcave:#x} is not empty')
            data[file_offset(zcave):file_offset(zcave) + len(zcode)] = zcode
            description += '; ' + text
            cave_end = max(cave_end, zcave + len(zcode))
        stage_links = []
        paint2_va = None
        if args.kalloc_stages:
            if not args.zalloc_progress:
                raise SystemExit('--kalloc-stages needs --zalloc-progress: the self-test maps the window')
            stage_va = (cave_end + 15) & ~15
            stage_links, stage_code, stage_text, paint2_va = kalloc_stages(data, kernel_names, workdir, stage_va)
            if stage_va + len(stage_code) > PANIC_CAVE_END:
                raise SystemExit(f'kalloc stage cave is {len(stage_code):#x} bytes at {stage_va:#x}, the padding ends at {PANIC_CAVE_END:#x}')
            if any(data[file_offset(stage_va):file_offset(stage_va) + len(stage_code)]):
                raise SystemExit(f'Padding at {stage_va:#x} is not empty')
            data[file_offset(stage_va):file_offset(stage_va) + len(stage_code)] = stage_code
            description += '; ' + stage_text
            cave_end = max(cave_end, stage_va + len(stage_code))
        fixed_stage_links = []
        if args.kalloc_fixed_stages:
            if not args.zalloc_progress:
                raise SystemExit('--kalloc-fixed-stages needs --zalloc-progress: the self-test maps the window')
            fixed_va = (cave_end + 15) & ~15
            fixed_stage_links, fixed_code, fixed_text = kalloc_fixed_stages(
                data, kernel_names, workdir, fixed_va)
            if fixed_va + len(fixed_code) > PANIC_CAVE_END:
                raise SystemExit(f'kalloc fixed stage cave is {len(fixed_code):#x} bytes at {fixed_va:#x}, the padding ends at {PANIC_CAVE_END:#x}')
            if any(data[file_offset(fixed_va):file_offset(fixed_va) + len(fixed_code)]):
                raise SystemExit(f'Padding at {fixed_va:#x} is not empty')
            data[file_offset(fixed_va):file_offset(fixed_va) + len(fixed_code)] = fixed_code
            description += '; ' + fixed_text
            cave_end = max(cave_end, fixed_va + len(fixed_code))
        parse_stage_links = []
        if args.parse_stages:
            parse_va = (max(cave_end, CAVES[4] + CAVE_SIZE) + 15) & ~15
            parse_stage_links, parse_code, parse_text = parse_stages(
                data, args.kernel, kernel_names, workdir, parse_va,
                identity=args.parse_identity, memory=args.parse_memory,
                late_fastboot=args.fastboot_after_parse,
                fastboot_delay=args.fastboot_delay)
            if parse_va + len(parse_code) > PANIC_CAVE_END:
                raise SystemExit(f'parse stage cave is {len(parse_code):#x} bytes at {parse_va:#x}, the padding ends at {PANIC_CAVE_END:#x}')
            if any(data[file_offset(parse_va):file_offset(parse_va) + len(parse_code)]):
                raise SystemExit(f'Padding at {parse_va:#x} is not empty')
            data[file_offset(parse_va):file_offset(parse_va) + len(parse_code)] = parse_code
            description += '; ' + parse_text
            cave_end = max(cave_end, parse_va + len(parse_code))
        zone_stage_links = []
        if args.kalloc_zone_stages:
            if not args.kalloc_stages:
                raise SystemExit('--kalloc-zone-stages needs --kalloc-stages: it reuses the stage paint helper')
            zstage_va = (cave_end + 15) & ~15
            zone_stage_links, zstage_code, zstage_text = kalloc_zone_stages(
                data, kernel_names, workdir, zstage_va, paint2_va,
                None if args.kalloc_zone_heap == 'any' else args.kalloc_zone_heap)
            if zstage_va + len(zstage_code) > PANIC_CAVE_END:
                raise SystemExit(f'kalloc zone cave is {len(zstage_code):#x} bytes at {zstage_va:#x}, the padding ends at {PANIC_CAVE_END:#x}')
            if any(data[file_offset(zstage_va):file_offset(zstage_va) + len(zstage_code)]):
                raise SystemExit(f'Padding at {zstage_va:#x} is not empty')
            data[file_offset(zstage_va):file_offset(zstage_va) + len(zstage_code)] = zstage_code
            description += '; ' + zstage_text
            cave_end = max(cave_end, zstage_va + len(zstage_code))
        zcreate_links = []
        if args.zcreate_stages:
            if not args.kalloc_zone_stages:
                raise SystemExit('--zcreate-stages needs --kalloc-zone-stages: it reuses the shared paint helper')
            zcreate_va = (cave_end + 15) & ~15
            zcreate_links, zcreate_code, zcreate_text = zcreate_stages(
                data, kernel_name_addrs, workdir, zcreate_va, paint2_va)
            if zcreate_va + len(zcreate_code) > PANIC_CAVE_END:
                raise SystemExit(f'zone_create cave is {len(zcreate_code):#x} bytes at {zcreate_va:#x}, the padding ends at {PANIC_CAVE_END:#x}')
            if any(data[file_offset(zcreate_va):file_offset(zcreate_va) + len(zcreate_code)]):
                raise SystemExit(f'Padding at {zcreate_va:#x} is not empty')
            data[file_offset(zcreate_va):file_offset(zcreate_va) + len(zcreate_code)] = zcreate_code
            description += '; ' + zcreate_text
            cave_end = max(cave_end, zcreate_va + len(zcreate_code))
        ztail_links = []
        if args.ztail_stages:
            if not args.zcreate_stages:
                raise SystemExit('--ztail-stages needs --zcreate-stages: it marks the same row')
            ztail_va = (cave_end + 15) & ~15
            ztail_links, ztail_code, ztail_text = ztail_stages(
                data, kernel_name_addrs, workdir, ztail_va, paint2_va)
            if ztail_va + len(ztail_code) > PANIC_CAVE_END:
                raise SystemExit(f'zone_create tail cave is {len(ztail_code):#x} bytes at {ztail_va:#x}, the padding ends at {PANIC_CAVE_END:#x}')
            if any(data[file_offset(ztail_va):file_offset(ztail_va) + len(ztail_code)]):
                raise SystemExit(f'Padding at {ztail_va:#x} is not empty')
            data[file_offset(ztail_va):file_offset(ztail_va) + len(ztail_code)] = ztail_code
            description += '; ' + ztail_text
            cave_end = max(cave_end, ztail_va + len(ztail_code))
        entry_links = []
        if args.entry_check:
            entry_va = (cave_end + 15) & ~15
            probes = []
            for va, what in ENTRY_CHECK_PROBES:
                value = struct.unpack_from('<Q', data, file_offset(va))[0]
                if value in (0, 0xFFFFFFFFFFFFFFFF):
                    raise SystemExit(f'Entry check probe ({what}) at {va:#x} holds {value:#x} in the file')
                probes.append((va, value))
            checks = ''.join(
                f'  ldr x11, ={va:#x}\n'
                f'  sub x11, x11, x10\n'
                f'  add x11, x11, x9\n'
                f'  ldr x12, [x11]\n'
                f'  ldr x13, ={value:#x}\n'
                f'  cmp x12, x13\n'
                f'  b.ne bad\n'
                for va, value in probes)
            source = ENTRY_CHECK.format(
                phys_off=0x10, virt_off=0x8,
                kernel_base=min(vmaddr for _, vmaddr, _, _, _ in SEGMENTS),
                checks=checks,
                fb_base=FRAMEBUFFER,
                bar_off=ENTRY_CHECK_BAR_Y * FRAMEBUFFER_STRIDE,
                pixels=ENTRY_CHECK_BAR_ROWS * (FRAMEBUFFER_STRIDE // 4),
                target='target',
                wdt_pa=WDT_PA, wdt_off=WDT_MODE_OFF, wdt_mask=WDT_MODE_MASK,
                wdt_value=WDT_MODE_BOOTLOADER,
                ticks=TIMEBASE * args.entry_check_delay)
            code, load, _ = assemble(source, entry_va, {'target': START_FIRST_CPU}, workdir)
            if load != entry_va:
                raise SystemExit(f'Entry check code was linked at {load:#x}, not {entry_va:#x}')
            if entry_va + len(code) > PANIC_CAVE_END:
                raise SystemExit(f'Entry check is {len(code):#x} bytes at {entry_va:#x}, the padding ends at {PANIC_CAVE_END:#x}')
            if any(data[file_offset(entry_va):file_offset(entry_va) + len(code)]):
                raise SystemExit(f'Padding at {entry_va:#x} is not empty')
            data[file_offset(entry_va):file_offset(entry_va) + len(code)] = code
            entry_links = [(ENTRY_VA, entry_va)]
            description += (f'; entry check of {len(probes)} window qwords'
                            f' (mismatch: red bar, {args.entry_check_delay} s, fastboot reset)')
            cave_end = max(cave_end, entry_va + len(code))
        entry_report_links = []
        if args.entry_report:
            report_va = (cave_end + 15) & ~15
            probes = []
            for va, what in ENTRY_CHECK_PROBES:
                value = struct.unpack_from('<Q', data, file_offset(va))[0]
                if value in (0, 0xFFFFFFFFFFFFFFFF):
                    raise SystemExit(f'Entry report probe ({what}) at {va:#x} holds {value:#x} in the file')
                probes.append((va, value))
            checks = ''.join(
                f'  ldr x11, ={va:#x}\n'
                f'  sub x11, x11, x20\n'
                f'  add x11, x11, x19\n'
                f'  ldr x12, [x11]\n'
                f'  ldr x13, ={value:#x}\n'
                f'  eor x12, x12, x13\n'
                f'  mov {reg}, x12\n'
                f'  cbz x12, 48f\n'
                f'  orr x21, x21, #{bit:#x}\n'
                f'48:\n'
                for bit, reg, (va, value) in zip((4, 8, 16), ('x22', 'x23', 'x24'), probes))
            cells = ''
            for index, bit in enumerate((1, 2, 4, 8, 16)):
                cells += (
                    f'  ldr x5, ={FRAMEBUFFER:#x}\n'
                    f'  ldr x6, ={ENTRY_REPORT_Y * FRAMEBUFFER_STRIDE + index * ENTRY_REPORT_CELL_W * 4:#x}\n'
                    f'  add x5, x5, x6\n'
                    f'  tst x21, #{bit:#x}\n'
                    f'  b.eq 43f\n'
                    f'  movz w7, #0xffff, lsl #16\n'
                    f'  b 44f\n'
                    f'43:\n'
                    f'  movz w7, #0xff00\n'
                    f'  movk w7, #0xff00, lsl #16\n'
                    f'44:\n'
                    f'  bl paint_cell\n')
            rows = ''
            for index, reg in enumerate(('x20', 'x19', 'x22', 'x23', 'x24', 'x26')):
                y = ENTRY_REPORT_BITS_Y + index * ENTRY_REPORT_ROW_PITCH
                rows += (
                    f'  mov x25, {reg}\n'
                    f'  ldr x5, ={FRAMEBUFFER:#x}\n'
                    f'  ldr x6, ={y * FRAMEBUFFER_STRIDE + ENTRY_REPORT_BITS_X * 4:#x}\n'
                    f'  add x5, x5, x6\n'
                    f'  bl paint_bits\n')
            source = ENTRY_REPORT.format(
                phys_off=0x10, virt_off=0x8,
                kernel_base=min(vmaddr for _, vmaddr, _, _, _ in SEGMENTS),
                checks=checks, cells=cells, rows=rows,
                delay_base=ENTRY_REPORT_DELAY_BASE,
                delay_step=ENTRY_REPORT_DELAY_STEP,
                wdt_pa=WDT_PA, wdt_off=WDT_MODE_OFF, wdt_mask=WDT_MODE_MASK,
                wdt_value=WDT_MODE_BOOTLOADER, target='target',
                cell_advance=FRAMEBUFFER_STRIDE - ENTRY_REPORT_CELL_W * 4,
                bits_advance=FRAMEBUFFER_STRIDE - 4096)
            code, load, _ = assemble(source, report_va, {'target': START_FIRST_CPU}, workdir)
            if load != report_va:
                raise SystemExit(f'Entry report code was linked at {load:#x}, not {report_va:#x}')
            if report_va + len(code) > PANIC_CAVE_END:
                raise SystemExit(f'Entry report is {len(code):#x} bytes at {report_va:#x}, the padding ends at {PANIC_CAVE_END:#x}')
            if any(data[file_offset(report_va):file_offset(report_va) + len(code)]):
                raise SystemExit(f'Padding at {report_va:#x} is not empty')
            data[file_offset(report_va):file_offset(report_va) + len(code)] = code
            entry_report_links = [(ENTRY_VA, report_va)]
            description += (f'; entry report of {len(probes)} window qwords'
                            f' (cells green/red, bits from y={ENTRY_REPORT_BITS_Y},'
                            f' reset after {ENTRY_REPORT_DELAY_BASE}+{ENTRY_REPORT_DELAY_STEP}'
                            f'*first-failing-check s)')
            cave_end = max(cave_end, report_va + len(code))
        entry_scan_links = []
        if args.entry_scan:
            scan_va = (cave_end + 15) & ~15
            name_va = ENTRY_CHECK_PROBES[0][0]
            flags_va = ENTRY_CHECK_PROBES[1][0]
            page_va = name_va & ~0xFFF
            name_off = name_va - page_va
            flags_off = flags_va - page_va
            kernel_base = min(vmaddr for _, vmaddr, _, _, _ in SEGMENTS)
            rec_sig = struct.unpack_from('<Q', data, file_offset(page_va))[0]
            name_ptr = struct.unpack_from('<Q', data, file_offset(name_va))[0]
            flags_val = struct.unpack_from('<Q', data, file_offset(flags_va))[0]
            for what, value in (('record page qword', rec_sig), ('view name pointer', name_ptr),
                                ('view flags', flags_val)):
                if value in (0, 0xFFFFFFFFFFFFFFFF):
                    raise SystemExit(f'Entry scan signature ({what}) at {page_va:#x} holds {value:#x} in the file')
            source = ENTRY_SCAN.format(
                phys_off=0x10, virt_off=0x8,
                rec_sig=rec_sig, name_ptr=name_ptr, flags_val=flags_val,
                win_page_off=page_va - kernel_base,
                name_off=name_off, flags_off=flags_off,
                scan_start=ENTRY_SCAN_START, scan_end=ENTRY_SCAN_END,
                img_sig_off=ENTRY_SCAN_POOL_OFFSET,
                img_name_off=ENTRY_SCAN_POOL_OFFSET + name_off,
                img_flags_off=ENTRY_SCAN_POOL_OFFSET + flags_off,
                vol_sig_off=ENTRY_SCAN_VOLUME_PAGE_OFFSET,
                vol_name_off=ENTRY_SCAN_VOLUME_PAGE_OFFSET + name_off,
                vol_flags_off=ENTRY_SCAN_VOLUME_PAGE_OFFSET + flags_off,
                delay_base=ENTRY_SCAN_DELAY_BASE,
                delay_step=ENTRY_SCAN_DELAY_STEP,
                wdt_pa=WDT_PA, wdt_off=WDT_MODE_OFF, wdt_mask=WDT_MODE_MASK,
                wdt_value=WDT_MODE_BOOTLOADER)
            code, load, _ = assemble(source, scan_va, {}, workdir)
            if load != scan_va:
                raise SystemExit(f'Entry scan code was linked at {load:#x}, not {scan_va:#x}')
            if scan_va + len(code) > PANIC_CAVE_END:
                raise SystemExit(f'Entry scan is {len(code):#x} bytes at {scan_va:#x}, the padding ends at {PANIC_CAVE_END:#x}')
            if any(data[file_offset(scan_va):file_offset(scan_va) + len(code)]):
                raise SystemExit(f'Padding at {scan_va:#x} is not empty')
            data[file_offset(scan_va):file_offset(scan_va) + len(code)] = code
            entry_scan_links = [(ENTRY_VA, scan_va)]
            description += (f'; entry scan of {ENTRY_SCAN_START:#x}-{ENTRY_SCAN_END:#x}'
                            f' for the record page (page qword at +0x0/+0x{ENTRY_SCAN_POOL_OFFSET:x}'
                            f'/+0x{ENTRY_SCAN_VOLUME_PAGE_OFFSET:x})'
                            f', reset after {ENTRY_SCAN_DELAY_BASE}+{ENTRY_SCAN_DELAY_STEP}*code s')
            cave_end = max(cave_end, scan_va + len(code))
        entry_hunt_links = []
        if args.entry_hunt:
            hunt_vector = (PANIC_CAVE_END - ENTRY_HUNT_SPAN) & ~0x7FF
            if hunt_vector < ((cave_end + 15) & ~15):
                raise SystemExit(f'Entry hunt vectors at {hunt_vector:#x} would overlap the caves ending at {cave_end:#x}')
            name_va = ENTRY_CHECK_PROBES[0][0]
            flags_va = ENTRY_CHECK_PROBES[1][0]
            page_va = name_va & ~0xFFF
            name_off = name_va - page_va
            flags_off = flags_va - page_va
            kernel_base = min(vmaddr for _, vmaddr, _, _, _ in SEGMENTS)
            rec_sig = struct.unpack_from('<Q', data, file_offset(page_va))[0]
            name_ptr = struct.unpack_from('<Q', data, file_offset(name_va))[0]
            flags_val = struct.unpack_from('<Q', data, file_offset(flags_va))[0]
            for what, value in (('record page qword', rec_sig), ('view name pointer', name_ptr),
                                ('view flags', flags_val)):
                if value in (0, 0xFFFFFFFFFFFFFFFF):
                    raise SystemExit(f'Entry hunt signature ({what}) at {page_va:#x} holds {value:#x} in the file')
            # Status cells, left to right:
            #   0 loader readback block present and coherent with boot_args
            #   1 the fixed-page volume probe has magic "DRP70" and the
            #     decompressed FAT volume holds the expected record triple
            #   2 the loader resolved the record page in the file and the
            #     file-buffer page holds the expected triple
            #   3 the window page at entry holds the expected triple
            #   4 the entry instruction agrees with volume and file buffer
            #     and the loader's window-vs-file memcmp was clean
            # Each cell loads what it needs itself; the rows afterwards read
            # the readback block directly, so no value survives across cells.
            def hunt_cell(index, bit, tests):
                fail = 100 + index * 10
                good = fail + 1
                body = ''.join(f'  {line}\n' for line in tests)
                return (
                    f'{body}'
                    f'  movz w7, #0xff00\n'
                    f'  movk w7, #0xff00, lsl #16\n'
                    f'  b {good}f\n'
                    f'{fail}:\n'
                    f'  orr x21, x21, #{bit:#x}\n'
                    f'  movz w7, #0xffff, lsl #16\n'
                    f'{good}:\n'
                    f'  add x5, x16, #{index * ENTRY_REPORT_CELL_W * 4:#x}\n'
                    f'  bl paint_cell\n'
                )

            cells = ''
            cells += hunt_cell(0, 0x1, (
                'ldr x24, [x23, #0]',
                f'ldr x11, ={ENTRY_HUNT_PROBE_MAGIC:#x}',
                'cmp x24, x11', 'b.ne 100f',
                'ldr x24, [x23, #8]', 'cmp x24, x20', 'b.ne 100f'))
            cells += hunt_cell(1, 0x2, (
                f'ldr x25, [x23, #{ENTRY_HUNT_VOL_MAGIC_OFF:#x}]',
                f'ldr x11, ={ENTRY_HUNT_VOL_MAGIC:#x}', 'cmp x25, x11', 'b.ne 110f',
                f'ldr x2, [x23, #{ENTRY_HUNT_VOL_SIG_OFF:#x}]',
                f'ldr x3, [x23, #{ENTRY_HUNT_VOL_NAME_OFF:#x}]',
                f'ldr x4, [x23, #{ENTRY_HUNT_VOL_FLAGS_OFF:#x}]',
                f'ldr x11, ={rec_sig:#x}', 'cmp x2, x11', 'b.ne 110f',
                f'ldr x11, ={name_ptr:#x}', 'cmp x3, x11', 'b.ne 110f',
                f'ldr x11, ={flags_val:#x}', 'cmp x4, x11', 'b.ne 110f'))
            cells += hunt_cell(2, 0x4, (
                f'ldr x25, [x23, #{ENTRY_HUNT_REC_FILE_OFF_OFF:#x}]',
                f'ldr x11, ={ENTRY_HUNT_REC_FILE_VALUE:#x}', 'cmp x25, x11', 'b.ne 120f',
                'ldr x24, [x23, #0x248]',
                'ldr x26, [x23, #0x258]',
                'ldr x27, [x23, #0x268]',
                f'ldr x11, ={rec_sig:#x}', 'cmp x24, x11', 'b.ne 120f',
                f'ldr x11, ={name_ptr:#x}', 'cmp x26, x11', 'b.ne 120f',
                f'ldr x11, ={flags_val:#x}', 'cmp x27, x11', 'b.ne 120f'))
            cells += hunt_cell(3, 0x8, (
                f'ldr x11, ={rec_sig:#x}', 'cmp x28, x11', 'b.ne 130f',
                f'ldr x11, ={name_ptr:#x}', 'cmp x29, x11', 'b.ne 130f',
                f'ldr x11, ={flags_val:#x}', 'cmp x18, x11', 'b.ne 130f'))
            cells += hunt_cell(4, 0x10, (
                f'ldr x1, [x23, #{ENTRY_HUNT_VOL_ENTRY_OFF:#x}]',
                'cmp x17, x1', 'b.ne 140f',
                'ldr x22, [x23, #0x290]', 'cmp x17, x22', 'b.ne 140f',
                'ldr x22, [x23, #0x270]', 'cbnz x22, 140f'))
            # Six raw qwords straight out of the readback block, top to
            # bottom: RecFileOffset, RecFileSig, RecWinSig, VolMagic,
            # VolRecSig, VolEntry. MSB left, black = 1, red = 0; 20 px apart,
            # the last row ends above the exception block at y=1400.
            rows = (
                f'  ldr x26, ={FRAMEBUFFER:#x}\n'
                f'  ldr x27, ={ENTRY_HUNT_BITS_Y * FRAMEBUFFER_STRIDE + 32 * 4:#x}\n'
                f'  ldr x28, ={ENTRY_HUNT_ROW_PITCH * FRAMEBUFFER_STRIDE:#x}\n'
            )
            for index, offset in enumerate((
                    ENTRY_HUNT_REC_FILE_OFF_OFF,
                    0x248,                     # RecFileSig
                    0x240,                     # RecWinSig
                    ENTRY_HUNT_VOL_MAGIC_OFF,
                    ENTRY_HUNT_VOL_SIG_OFF,
                    ENTRY_HUNT_VOL_ENTRY_OFF,
            )):
                if index:
                    rows += '  add x27, x27, x28\n'
                rows += (
                    f'  ldr x25, [x23, #{offset:#x}]\n'
                    f'  add x5, x26, x27\n'
                    f'  bl paint_bits\n')
            source = ENTRY_HUNT.format(
                phys_off=0x10, virt_off=0x8,
                probe_off=ENTRY_HUNT_PROBE_OFF,
                hunt_off=hunt_vector - kernel_base,
                rec_sig=rec_sig, name_ptr=name_ptr, flags_val=flags_val,
                win_page_off=page_va - kernel_base,
                entry_off=ENTRY_VA - kernel_base,
                name_off=name_off, flags_off=flags_off,
                cells=cells, rows=rows,
                cells_base=FRAMEBUFFER + ENTRY_HUNT_CELLS_Y * FRAMEBUFFER_STRIDE,
                delay_base=ENTRY_HUNT_DELAY_BASE, delay_step=ENTRY_HUNT_DELAY_STEP,
                red_bar=FRAMEBUFFER + ENTRY_HUNT_RED_Y * FRAMEBUFFER_STRIDE,
                red_words=ENTRY_HUNT_RED_LINES * (FRAMEBUFFER_STRIDE // 4),
                wdt_pa=WDT_PA, wdt_off=WDT_MODE_OFF, wdt_mask=WDT_MODE_MASK,
                wdt_value=WDT_MODE_BOOTLOADER,
                cell_advance=FRAMEBUFFER_STRIDE - ENTRY_REPORT_CELL_W * 4,
                bits_advance=FRAMEBUFFER_STRIDE - 4096)
            code, load, labels = assemble(source, hunt_vector, {}, workdir, ENTRY_HUNT_ENTRY_OFF)
            if load != hunt_vector:
                raise SystemExit(f'Entry hunt code was linked at {load:#x}, not {hunt_vector:#x}')
            if hunt_vector + len(code) > PANIC_CAVE_END:
                raise SystemExit(f'Entry hunt is {len(code):#x} bytes at {hunt_vector:#x}, the padding ends at {PANIC_CAVE_END:#x}')
            if any(data[file_offset(hunt_vector):file_offset(hunt_vector) + len(code)]):
                raise SystemExit(f'Padding at {hunt_vector:#x} is not empty')
            data[file_offset(hunt_vector):file_offset(hunt_vector) + len(code)] = code
            entry_hunt_links = [(ENTRY_VA, hunt_vector + ENTRY_HUNT_ENTRY_OFF)]
            description += (f'; entry hunt comparing the decompressed-volume, file-buffer and window'
                            f' record pages via the loader readback at args+0x{ENTRY_HUNT_PROBE_OFF:x}'
                            f' and the fixed probe page, vectors at {hunt_vector:#x},'
                            f' reset {ENTRY_HUNT_DELAY_BASE}s after the paints with fastboot re-armed')
            cave_end = max(cave_end, hunt_vector + len(code))
        entry_reset_links = []
        if args.entry_reset_now:
            reset_va = (cave_end + 15) & ~15
            source = ENTRY_RESET.format(
                ticks=args.entry_reset_wait_ticks,
                wdt_pa=WDT_PA, wdt_off=WDT_MODE_OFF, wdt_mask=WDT_MODE_MASK,
                wdt_value=WDT_MODE_BOOTLOADER)
            code, load, _ = assemble(source, reset_va, {}, workdir)
            if load != reset_va:
                raise SystemExit(f'Entry reset code was linked at {load:#x}, not {reset_va:#x}')
            if reset_va + len(code) > PANIC_CAVE_END:
                raise SystemExit(f'Entry reset is {len(code):#x} bytes at {reset_va:#x}, the padding ends at {PANIC_CAVE_END:#x}')
            if any(data[file_offset(reset_va):file_offset(reset_va) + len(code)]):
                raise SystemExit(f'Padding at {reset_va:#x} is not empty')
            data[file_offset(reset_va):file_offset(reset_va) + len(code)] = code
            entry_reset_links = [(ENTRY_VA, reset_va)]
            description += '; entry reset (immediate fastboot reset at the kernel entry)'
            cave_end = max(cave_end, reset_va + len(code))
    for site, stage, _ in sites:
        struct.pack_into('<I', data, file_offset(site), branch(site, CAVES[stage]))
    if zsite is not None:
        struct.pack_into('<I', data, file_offset(zsite), branch(zsite, zentry))
        struct.pack_into('<I', data, file_offset(zfault), branch(zfault, zselftest))
    for site, target in stage_links:
        struct.pack_into('<I', data, file_offset(site), branch(site, target))
    for site, target in fixed_stage_links:
        struct.pack_into('<I', data, file_offset(site), branch(site, target))
    for site, target in zone_stage_links:
        struct.pack_into('<I', data, file_offset(site), branch(site, target))
    for site, target in zcreate_links:
        struct.pack_into('<I', data, file_offset(site), branch(site, target))
    for site, target in ztail_links:
        struct.pack_into('<I', data, file_offset(site), branch(site, target))
    for site, target in parse_stage_links:
        struct.pack_into('<I', data, file_offset(site), branch(site, target))
    for site, target in entry_links:
        struct.pack_into('<I', data, file_offset(site), branch(site, target))
    for site, target in entry_report_links:
        struct.pack_into('<I', data, file_offset(site), branch(site, target))
    for site, target in entry_scan_links:
        struct.pack_into('<I', data, file_offset(site), branch(site, target))
    for site, target in entry_hunt_links:
        struct.pack_into('<I', data, file_offset(site), branch(site, target))
    for site, target in entry_reset_links:
        struct.pack_into('<I', data, file_offset(site), branch(site, target))
    if args.panic_screen:
        # Right after the last probe cave in use, so the padding is shared as tightly as possible.
        PANIC_CAVE = (cave_end + 15) & ~15
        if word(PANIC_VA) in (0, 0xFFFFFFFF) or any(data[file_offset(PANIC_CAVE):file_offset(PANIC_CAVE_END)]):
            raise SystemExit('panic() or the panic screen padding is not as expected')
        with tempfile.TemporaryDirectory() as tmp:
            code, load, entry = assemble_c(ROOT / 'scripts/xnu-panic-screen.c', PANIC_CAVE, Path(tmp), kernel_names)
            common_entry = panic_common_entry(Path(tmp), code, load)
        if load + len(code) > PANIC_CAVE_END:
            raise SystemExit(f'panic screen is {len(code):#x} bytes at {load:#x}, the padding ends at {PANIC_CAVE_END:#x}')
        if any(data[file_offset(load):file_offset(load) + len(code)]):
            raise SystemExit(f'Padding at {load:#x} is not empty')
        data[file_offset(load):file_offset(load) + len(code)] = code
        struct.pack_into('<I', data, file_offset(PANIC_VA), branch(PANIC_VA, entry))
        common = kernel_names.get('_panic_trap_to_debugger')
        if common is None or word(common) in (0, 0xFFFFFFFF):
            raise SystemExit('Common panic funnel is missing; refuse a partially covered panic screen')
        struct.pack_into('<I', data, file_offset(common), branch(common, common_entry))
        description += f', panic screen ({len(code):#x} bytes at {entry:#x}; common funnel at {common:#x})'
    args.kernel.write_bytes(data)
    print(f'patched {args.kernel.name}: {description}')

if __name__ == '__main__':
    main()
