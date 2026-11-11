#!/usr/bin/env python3
"""Redirect the XNU kernel console to a 4x scaled ISO-font renderer.

diag77 proved that kprintf reaches the video console but the 8x16 glyphs are
too small for the only available camera (1280x960). This patch replaces the
first instruction of _console_write_unbuffered with a tail branch into a
renderer in the zero padding at the end of __TEXT_EXEC. The renderer draws a bottom four-line HUD: iso_font glyphs at 4x
(32x64 cells, 50x4 grid) white over a half-darkened band, keeping the UI visible
and unshifted, with a text shadow buffer that the presenter re-stamps over each
copied frame through the refresh entry.

The kernel has to be the unpatched-prelinked (or --no-reset/--skip-panic-log)
kernel; the hook site is verified by its prologue word.
"""
import argparse
import struct
import subprocess
import shutil
import tempfile
from pathlib import Path

CAVE_VA = 0xFFFFFFF0079E1E50
CAVE_REGION = (0xFFFFFFF0079E1E50, 0xFFFFFFF0079E4000)
STATE_VA = 0xFFFFFFF007C6F000
MAGIC = 0x4249475445585401
COLS = 50
ROWS = 40
HOOK_PROLOGUE = 0xD10103FF  # sub sp, sp, #0x40

RENDERER = r"""
    .equ COLS, 40
    .equ ROWS, 20
    .equ WIN_X0, 1280
    .equ WIN_Y0, 1280
    .equ WIN_W, 5120
    .equ WIN_H, 1280
    .text
    .global _start
_start:
    mov x4, x30
    and w0, w0, #0xff
    ldr x1, =vinfo
    ldr x2, [x1, #0x10]
    cbz x2, out
    ldr w3, [x1, #8]
    cmp w3, #32
    b.ne out
    ldr w6, [x1, #0xc]
    cbz x6, out
    ldr w5, [x1]
    cmp x5, #WIN_Y0+WIN_H
    b.lo out
    ldr x7, =state
    ldr x8, [x7, #16]
    ldr x1, =magic
    cmp x8, x1
    b.eq ready
    mov x14, x2
    mov x13, #WIN_Y0
    mul x13, x13, x6
    add x14, x14, x13
    add x14, x14, #WIN_X0
    mov x15, #WIN_H
clear_row:
    mov x3, x14
    mov x16, #WIN_W
clear_col:
    str xzr, [x3], #8
    sub x16, x16, #8
    cbnz x16, clear_col
    add x14, x14, x6
    sub x15, x15, #1
    cbnz x15, clear_row
    str xzr, [x7]
    str xzr, [x7, #8]
    ldr x1, =magic
    str x1, [x7, #16]
ready:
    ldr x9, [x7]
    ldr x10, [x7, #8]
    cmp w0, #13
    b.eq do_cr
    cmp w0, #10
    b.eq do_lf
    cmp x9, #COLS
    b.lo draw
    mov x9, #0
    add x10, x10, #1
    cmp x10, #ROWS
    b.lo draw
    bl scroll
    mov x10, #ROWS-1
draw:
    mov x11, #COLS
    mul x11, x10, x11
    add x11, x11, x9
    ldr x12, =text
    strb w0, [x12, x11]
    bl draw_glyph
    add x9, x9, #1
    b store
do_cr:
    mov x9, #0
    b store
do_lf:
    mov x9, #0
    add x10, x10, #1
    cmp x10, #ROWS
    b.lo store
    bl scroll
    mov x10, #ROWS-1
store:
    str x9, [x7]
    str x10, [x7, #8]
out:
    mov x30, x4
    ret
draw_glyph:
    ldr x1, =vinfo
    ldr w5, [x1]
    cmp x5, #WIN_Y0+WIN_H
    b.lo dg_out
    cbz x6, dg_out
    mov x13, #WIN_Y0
    lsl x14, x10, #6
    add x13, x13, x14
    mul x13, x13, x6
    add x13, x2, x13
    add x13, x13, #WIN_X0
    lsl x14, x9, #7
    add x12, x13, x14
    ldr x11, =iso_font
    add x11, x11, w0, uxtw #4
    mov w14, #-1
    mov x15, #0
glyph_rows:
    ldrb w0, [x11, x15]
    mov x16, #0
glyph_bits:
    lsr w3, w0, w16
    and w3, w3, #1
    cbz w3, next_bit
    lsl x17, x15, #2
    mul x17, x17, x6
    add x17, x12, x17
    lsl x3, x16, #4
    add x17, x17, x3
    stp w14, w14, [x17]
    stp w14, w14, [x17, #8]
    add x17, x17, x6
    stp w14, w14, [x17]
    stp w14, w14, [x17, #8]
    add x17, x17, x6
    stp w14, w14, [x17]
    stp w14, w14, [x17, #8]
    add x17, x17, x6
    stp w14, w14, [x17]
    stp w14, w14, [x17, #8]
next_bit:
    add x16, x16, #1
    cmp x16, #8
    b.lo glyph_bits
    add x15, x15, #1
    cmp x15, #16
    b.lo glyph_rows
dg_out:
    ret
scroll:
    ldr x1, =vinfo
    ldr w5, [x1]
    cmp x5, #WIN_Y0+WIN_H
    b.lo scroll_text
    ldr x6, [x1, #0xc]
    cbz x6, scroll_text
    mov x13, #WIN_Y0
    mul x13, x13, x6
    add x13, x2, x13
    add x13, x13, #WIN_X0
    lsl x8, x6, #6
    add x1, x13, x8
    mov x3, x13
    mov x11, #WIN_W
    sub x11, x6, x11
    mov x15, #WIN_H-64
sc_row:
    cbz x15, sc_clear
    mov x16, #WIN_W
sc_px:
    ldr x17, [x1], #8
    str x17, [x3], #8
    sub x16, x16, #8
    cbnz x16, sc_px
    add x1, x1, x11
    add x3, x3, x11
    sub x15, x15, #1
    b sc_row
sc_clear:
    mov x15, #64
sc_crow:
    cbz x15, scroll_text
    mov x16, #WIN_W
sc_cpx:
    str xzr, [x3], #8
    sub x16, x16, #8
    cbnz x16, sc_cpx
    add x3, x3, x11
    sub x15, x15, #1
    b sc_crow
scroll_text:
    ldr x11, =text
    mov x13, #(ROWS-1)*COLS
    mov x14, x11
    add x14, x14, #COLS
    mov x3, x11
st_copy:
    cbz x13, st_clear
    ldrb w12, [x14], #1
    strb w12, [x3], #1
    sub x13, x13, #1
    b st_copy
st_clear:
    mov x13, #COLS
st_fill:
    cbz x13, st_done
    strb wzr, [x3], #1
    sub x13, x13, #1
    b st_fill
st_done:
    ret
    .global refresh
refresh:
    mov x4, x30
    ldr x1, =vinfo
    ldr x2, [x1, #0x10]
    cbz x2, out2
    ldr w3, [x1, #8]
    cmp w3, #32
    b.ne out2
    ldr x6, [x1, #0xc]
    cbz x6, out2
    ldr w5, [x1]
    cmp x5, #WIN_Y0+WIN_H
    b.lo out2
    ldr x7, =state
    ldr x8, [x7, #16]
    ldr x1, =magic
    cmp x8, x1
    b.ne out2
    mov x10, #0
r_row:
    mov x9, #0
r_col:
    mov x11, #COLS
    mul x11, x10, x11
    add x11, x11, x9
    ldr x12, =text
    ldrb w0, [x12, x11]
    cbz w0, r_next
    bl draw_glyph
r_next:
    add x9, x9, #1
    cmp x9, #COLS
    b.lo r_col
    add x10, x10, #1
    cmp x10, #ROWS
    b.lo r_row
out2:
    mov x30, x4
    ret
"""


def tool(name):
    path = shutil.which(name)
    if not path:
        raise SystemExit(f'{name} not found; run this under WSL with llvm/clang installed')
    return path


def kernel_symbols(kernel):
    out = subprocess.run([tool('llvm-nm'), str(kernel)], capture_output=True, text=True, check=True).stdout
    symbols = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[1] in ('T', 't', 'D', 'd', 'B', 'b', 'S', 's', 'R', 'r'):
            symbols[parts[2]] = int(parts[0], 16)
    return symbols


def load_segments(data):
    magic, _, _, _, ncmds, _, _, _ = struct.unpack_from('<8I', data, 0)
    if magic != 0xFEEDFACF:
        raise SystemExit('not a 64-bit Mach-O')
    segments = {}
    off = 32
    for _ in range(ncmds):
        cmd, size = struct.unpack_from('<2I', data, off)
        if cmd == 0x19:
            name = data[off + 8:off + 24].rstrip(b'\x00').decode()
            vm, vsize, fileoff, filesize = struct.unpack_from('<4Q', data, off + 24)
            segments[name] = (vm, vsize, fileoff, filesize)
        off += size
    return segments


def file_offset(segments, va):
    for vm, vsize, fileoff, filesize in segments.values():
        if vm <= va < vm + vsize:
            return fileoff + (va - vm)
    raise SystemExit(f'{va:#x} is outside every segment')


def assemble(va, symbols, workdir):
    asm = workdir / 'render.S'
    obj = workdir / 'render.o'
    elf = workdir / 'render.elf'
    binary = workdir / 'render.bin'
    asm.write_text(RENDERER)
    clang = tool('clang')
    lld = tool('ld.lld')
    objcopy = tool('llvm-objcopy')
    subprocess.run([clang, '--target=aarch64-none-elf', '-c', str(asm), '-o', str(obj)], check=True)
    subprocess.run([lld, f'-Ttext={va:#x}', *[f'--defsym={name}={value:#x}' for name, value in symbols.items()],
                    '-e', '_start', str(obj), '-o', str(elf)], check=True)
    subprocess.run([objcopy, '-O', 'binary', '-j', '.text', str(elf), str(binary)], check=True)
    return binary.read_bytes()


def refresh_address(workdir):
    out = subprocess.run([tool('llvm-nm'), str(workdir / 'render.elf')], capture_output=True, text=True, check=True).stdout
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2].lstrip('_') == 'refresh':
            return int(parts[0], 16)
    raise SystemExit('the renderer refresh entry is missing from the linked ELF')


def branch(from_va, to_va):
    delta = to_va - from_va
    if delta % 4 or not -(1 << 27) <= delta < (1 << 27):
        raise SystemExit(f'branch {from_va:#x} -> {to_va:#x} out of range')
    return 0x14000000 | ((delta >> 2) & 0x3FFFFFF)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('kernel', type=Path, help='the kernel COPY to patch in place')
    parser.add_argument('--cave', type=lambda s: int(s, 0), help='__TEXT_EXEC VA for the renderer when the built-in cave is no longer empty')
    parser.add_argument('--cave-end', type=lambda s: int(s, 0), help='end VA of the zero run holding the --cave (defaults to the end of __TEXT_EXEC)')
    args = parser.parse_args()

    data = bytearray(args.kernel.read_bytes())
    segments = load_segments(data)

    symbols = kernel_symbols(args.kernel)
    needed = ('_console_write_unbuffered', '_iso_font', '_vinfo', '_native_legacy_client_status')
    missing = [name for name in needed if name not in symbols]
    if missing:
        raise SystemExit(f'symbols missing: {missing}')
    hook_va = symbols['_console_write_unbuffered']
    font_va = symbols['_iso_font']
    vinfo_va = symbols['_vinfo']

    text_exec = segments['__TEXT_EXEC']
    cave_va = CAVE_VA if args.cave is None else args.cave
    if args.cave is None:
        cave_region = CAVE_REGION
    elif args.cave_end is None:
        cave_region = (cave_va, text_exec[0] + text_exec[1])
    else:
        cave_region = (cave_va, args.cave_end)
    if not cave_va < cave_region[1] <= text_exec[0] + text_exec[1]:
        raise SystemExit(f'cave region {cave_region[0]:#x}..{cave_region[1]:#x} is empty or outside __TEXT_EXEC')
    for va, label in ((hook_va, 'hook'), (cave_va, 'cave')):
        if not text_exec[0] <= va < text_exec[0] + text_exec[1]:
            raise SystemExit(f'{label} {va:#x} is outside __TEXT_EXEC; this is not the kernel these patches are for')
    for va, label in ((font_va, 'font'), (vinfo_va, 'vinfo')):
        file_offset(segments, va)
    sized = subprocess.run([tool('llvm-nm'), '-S', '--defined-only', str(args.kernel)],
                           capture_output=True, text=True, check=True).stdout
    for line in sized.splitlines():
        parts = line.split()
        if len(parts) != 4:
            continue
        try:
            sva, ssize = int(parts[0], 16), int(parts[1], 16)
        except ValueError:
            continue
        if sva < STATE_VA + 0x1000 and sva + max(ssize, 1) > STATE_VA:
            raise SystemExit(f'state page {STATE_VA:#x} overlaps kernel symbol {parts[3]}')
    data_seg = segments['__DATA']
    if not data_seg[0] <= STATE_VA < data_seg[0] + data_seg[1]:
        raise SystemExit(f'state {STATE_VA:#x} is outside __DATA; this is not the kernel these patches are for')

    hook_off = file_offset(segments, hook_va)
    hook_word = struct.unpack_from('<I', data, hook_off)[0]
    if hook_word != HOOK_PROLOGUE:
        raise SystemExit(f'hook at {hook_va:#x} holds {hook_word:#010x}, expected {HOOK_PROLOGUE:#010x}; already patched?')

    cave_off = file_offset(segments, cave_va)
    region_off = file_offset(segments, cave_region[0])
    region_len = cave_region[1] - cave_region[0]
    region = data[region_off:region_off + region_len]
    if any(region):
        raise SystemExit('the __TEXT_EXEC cave is not zero; refusing to place code there')

    with tempfile.TemporaryDirectory() as tmp:
        code = assemble(cave_va, {'iso_font': font_va, 'vinfo': vinfo_va, 'state': STATE_VA, 'magic': MAGIC,
                                  'text': STATE_VA + 0x20, 'client': symbols['_native_legacy_client_status']}, Path(tmp))
        refresh_va = refresh_address(Path(tmp))
    if len(code) > cave_region[1] - cave_va:
        raise SystemExit(f'renderer is {len(code):#x} bytes, does not fit the cave')
    if len(code) % 4:
        raise SystemExit('renderer length is not a multiple of 4')

    data[cave_off:cave_off + len(code)] = code
    jump = branch(hook_va, cave_va)
    struct.pack_into('<I', data, hook_off, jump)
    # Keep the log on top: presenters call this after copying a frame; the stub is a bare ret.
    refresh_hook = '_native_console_overlay_refresh'
    jump2 = None
    if refresh_hook in symbols:
        refresh_hook_va = symbols[refresh_hook]
        refresh_off = file_offset(segments, refresh_hook_va)
        refresh_word = struct.unpack_from('<I', data, refresh_off)[0]
        if refresh_word != 0xD65F03C0:
            raise SystemExit(f'{refresh_hook} at {refresh_hook_va:#x} holds {refresh_word:#010x}, expected a bare ret')
        jump2 = branch(refresh_hook_va, refresh_va)
        struct.pack_into('<I', data, refresh_off, jump2)
    else:
        print('note: no native_console_overlay_refresh stub; the log is not re-stamped after presents')

    readback = bytes(data)
    if struct.unpack_from('<I', readback, hook_off)[0] != jump:
        raise SystemExit('hook readback differs')
    if jump2 is not None and struct.unpack_from('<I', readback, refresh_off)[0] != jump2:
        raise SystemExit('refresh hook readback differs')
    if readback[cave_off:cave_off + len(code)] != code:
        raise SystemExit('renderer readback differs')
    if any(readback[cave_off + len(code):region_off + region_len]):
        raise SystemExit('cave tail is no longer zero')
    if STATE_VA < data_seg[0] + data_seg[3]:
        raise SystemExit('state page is file-backed; the cursor would not start zeroed')

    args.kernel.write_bytes(readback)
    print(f'hook {hook_va:#x}: {hook_word:#010x} -> {jump:#010x} (b {cave_va:#x})')
    if jump2 is not None:
        print(f'refresh hook {refresh_hook_va:#x} -> {jump2:#010x} (b {refresh_va:#x})')
    print(f'renderer {len(code):#x} bytes at {cave_va:#x} (cave {cave_region[0]:#x}..{cave_region[1]:#x})')
    print(f'font {font_va:#x} vinfo {vinfo_va:#x} state {STATE_VA:#x} magic {MAGIC:#x}')
    print(f'40x20 window console, re-stamped after presents, {len(data)} bytes total')


if __name__ == '__main__':
    main()
