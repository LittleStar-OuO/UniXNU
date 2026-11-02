import struct
import sys

path = sys.argv[1]
wants = sys.argv[2:]
d = open(path, 'rb').read()

ncmds = struct.unpack_from('<I', d, 16)[0]
off = 32
segs = []
symtab = None
for _ in range(ncmds):
    cmd, sz = struct.unpack_from('<2I', d, off)
    if cmd == 0x19:
        name = d[off + 8:off + 24].split(b'\0')[0].decode()
        va, vs, fo, fs = struct.unpack_from('<4Q', d, off + 24)
        segs.append((name, va, vs, fo, fs))
    elif cmd == 0x2:
        symtab = struct.unpack_from('<4I', d, off + 8)
    off += sz

symoff, nsyms, stroff, strsize = symtab
syms = {}
for i in range(nsyms):
    n_strx, n_type, n_sect, n_desc, n_value = struct.unpack_from('<IBBHQ', d, symoff + 16 * i)
    if n_type & 0xe0:
        continue
    end = d.index(b'\0', stroff + n_strx)
    syms[d[stroff + n_strx:end].decode()] = n_value


def va_to_off(va):
    for name, sva, vs, fo, fs in segs:
        if sva <= va < sva + fs:
            return fo + (va - sva)
    return None


def dump(va, n):
    fo = va_to_off(va)
    print(f'@{va:#x} file {fo:#x}')
    words = struct.unpack_from('<%dI' % (n // 4), d, fo)
    for i in range(0, len(words), 4):
        print(f'  {va + i:#x}: ' + ' '.join(f'{w:08x}' for w in words[i:i + 4]))


for w in wants:
    if w.startswith('0x'):
        va, _, ln = w.partition(':')
        dump(int(va, 16), int(ln, 16) if ln else 16)
    else:
        dump(syms[w], 16)
