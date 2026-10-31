"""Android boot image: gzip(BootShim + UEFI FD) with an optional appended device tree and ramdisk.

Inputs are a Mu-Silicium checkout (BootShim sources, Resources/Configs/<device>.toml, mkbootimg.py),
the built firmware volume and a compiled device tree. [boot_shim] presence and
boot_image_kernel.append_dtb follow the Mu-Silicium device config. The result is verified by
unpacking it again; the output is deterministic.
"""
import gzip
import hashlib
import json
import shutil
import struct
import tempfile
import zlib
from pathlib import Path

from .config import load_toml
from .util import Error, log, run, sha256_file, tool

DEFAULTS = {'header_version': 0, 'base': 0, 'kernel_offset': 0x8000, 'tags_offset': 0x1e00000, 'pagesize': 4096}
MKBOOTIMG_FLAGS = ('base', 'kernel_offset', 'tags_offset', 'pagesize', 'header_version', 'ramdisk_offset', 'second_offset', 'cmdline')


def compile_shim(uefi, base, size, requires_header, scratch):
    clang, objcopy, nm = tool('clang'), tool('llvm-objcopy', 'brew install llvm'), tool('llvm-nm', 'brew install llvm')
    source = uefi / 'BootShim/BootShim.S'
    if not source.is_file():
        raise Error(f'{source} missing: --uefi must be a Mu-Silicium checkout')
    obj = scratch / 'shim.o'
    run([clang, '--target=aarch64-none-elf', '-c', f'-DFD_BASE={base}', f'-DFD_SIZE={size}',
         f'-DREQUIRES_KERNEL_HEADER={int(requires_header)}', source, '-o', obj], capture=True)
    run([objcopy, '-O', 'binary', obj, scratch / 'shim.bin'], capture=True)
    shim = (scratch / 'shim.bin').read_bytes()
    symbols = run([nm, '-n', obj], capture=True)
    offset = next((int(line.split()[0], 16) for line in symbols.splitlines() if line.endswith(' _Payload')), None)
    if offset != len(shim):
        raise Error('a literal pool or padding separates the _Payload label from the appended firmware volume')
    if requires_header and (shim[56:60] != b'ARM\x64' or struct.unpack_from('<QQ', shim, 8) != (base, len(shim) + size)):
        raise Error('the ARM64 image header does not describe the shim plus the firmware volume')
    return shim


def patch_dtb(dtb, nodes, scratch):
    """Add device-tree nodes the stock bootloader expects (board.toml [[bootimg.dtb_node]])."""
    if not nodes:
        return dtb
    fdtput, fdtget = tool('fdtput', 'brew install dtc'), tool('fdtget', 'brew install dtc')
    work = scratch / 'device.dtb'
    work.write_bytes(dtb)
    for node in nodes:
        run([fdtput, '-c', work, node['path']], capture=True)
        for prop in node.get('props', []):
            values = prop['value'] if isinstance(prop['value'], list) else [prop['value']]
            run([fdtput, '-t', prop.get('type', 's'), work, node['path'], prop['name'], *[str(v) for v in values]], capture=True)
            got = run([fdtget, '-t', prop.get('type', 's'), work, node['path'], prop['name']], capture=True).split()
            kind = prop.get('type', 's')
            if kind in ('x', 'i', 'u') and [int(v, 16 if kind == 'x' else 10) for v in got] != [int(str(v), 0) for v in values]:
                raise Error(f'dtb node {node["path"]} property {prop["name"]} did not round-trip')
    return work.read_bytes()


def settings(ctx, uefi):
    fw = ctx.board.get('firmware', {})
    config = load_toml(uefi / 'Resources/Configs' / f'{fw["config"]}.toml') if fw.get('config') else {}
    values = dict(DEFAULTS)
    values.update(config.get('boot_image', {}))
    values.update(ctx.board.get('bootimg', {}).get('image', {}))
    fd = config.get('uefi_fd', {})
    kernel = {'kernel_compression': 'gzip', 'append_dtb': False}
    kernel.update(config.get('boot_image_kernel', {}))
    return values, fd.get('base'), fd.get('size'), config.get('boot_shim'), kernel, config


def build(ctx, uefi, fd, dtb, ramdisk=None, output=None):
    log('bootimg')
    image, base, size, shim_config, kernel_config, config = settings(ctx, uefi)
    append_dtb = kernel_config.get('append_dtb', False)
    compression = kernel_config.get('kernel_compression', 'gzip')
    if compression != 'gzip':
        raise Error(f'unsupported boot_image_kernel.kernel_compression {compression!r}: only gzip is implemented')
    mkbootimg = uefi / 'Resources/Scripts/mkbootimg.py'
    if not mkbootimg.is_file():
        raise Error(f'{mkbootimg} missing: --uefi must be a Mu-Silicium checkout')
    if not fd or not Path(fd).is_file():
        raise Error(f'firmware volume not found: {fd}; run ./unixnu build firmware --uefi {uefi}')
    if append_dtb and (not dtb or not Path(dtb).is_file()):
        raise Error(f'device tree not found: {dtb} (boot_image_kernel.append_dtb is enabled)')
    firmware = Path(fd).read_bytes()
    if base is None or size is None:
        raise Error('the Mu-Silicium device config has no [uefi_fd] base and size')
    if len(firmware) != size:
        raise Error(f'firmware volume is {len(firmware):#x} bytes, the device config says {size:#x}')
    out = Path(output) if output else ctx.work / 'boot/boot.img'
    out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='unixnu-bootimg-') as scratch_name:
        scratch = Path(scratch_name)
        shim = b''
        if shim_config is not None:
            shim = compile_shim(uefi, base, size, shim_config.get('requires_kernel_header', False), scratch)
        tree = b''
        if append_dtb:
            tree = patch_dtb(Path(dtb).read_bytes(), ctx.board.get('bootimg', {}).get('dtb_node', []), scratch)
            if tree[:4] != b'\xd0\x0d\xfe\xed' or struct.unpack_from('>I', tree, 4)[0] != len(tree):
                raise Error('invalid compiled device tree')
        combined = shim + firmware
        kernel = gzip.compress(combined, mtime=0) + tree
        kernel_path = scratch / 'Image.gz-dtb'
        kernel_path.write_bytes(kernel)
        if ramdisk:
            ramdisk_path = Path(ramdisk)
            if not ramdisk_path.is_file():
                raise Error(f'ramdisk not found: {ramdisk_path}')
        else:
            ramdisk_path = scratch / 'ramdisk'
            ramdisk_path.write_bytes(b'')
        command = ['python3', mkbootimg, '--kernel', kernel_path, '--ramdisk', ramdisk_path, '-o', out]
        for key in MKBOOTIMG_FLAGS:
            if key in image:
                command += [f'--{key}', str(image[key])]
        run(command, capture=True)
        report = verify(out, combined, tree, ramdisk_path, image)
    report.update(image_sha256=sha256_file(out), image_bytes=out.stat().st_size, fd_sha256=sha256_file(fd),
                  shim_bytes=len(shim), hardware_tested=False)
    if append_dtb:
        report['dtb_sha256'] = sha256_file(dtb)
    (out.parent / 'boot.json').write_text(json.dumps(report, indent=2) + '\n')
    log(f'bootimg: {out} ({out.stat().st_size} bytes)')
    return out


def verify(path, combined, tree, ramdisk_path, image):
    """Unpack the image and compare every part with its input."""
    data = Path(path).read_bytes()
    if data[:8] != b'ANDROID!':
        raise Error('not an Android boot image')
    version = struct.unpack_from('<I', data, 40)[0]
    if version != image['header_version']:
        raise Error('boot image header does not match the requested parameters')
    if version >= 3:
        # v3/v4: fixed 4096-byte pages, header size 1580/1584, no load addresses.
        kernel_size, ramdisk_size = struct.unpack_from('<2I', data, 8)
        header_size = struct.unpack_from('<I', data, 20)[0]
        if header_size != (1580, 1584)[version - 3]:
            raise Error('boot image header size does not match the header version')
        if version == 4 and struct.unpack_from('<I', data, 1580)[0] != 4096:
            raise Error('boot image v4 signature size is not 4096 bytes')
        page = 4096
        kernel_address = tags_address = None
        expected = page * (1 + -(-kernel_size // page) + -(-ramdisk_size // page)) + (4096 if version == 4 else 0)
        if len(data) != expected:
            raise Error(f'boot image is {len(data)} bytes, expected {expected}')
    else:
        kernel_size, kernel_address, ramdisk_size, _, second_size, _, tags, page, _, _ = struct.unpack_from('<10I', data, 8)
        if second_size or page != image['pagesize']:
            raise Error('boot image header does not match the requested parameters')
        if kernel_address != image['base'] + image['kernel_offset'] or tags != image['base'] + image['tags_offset']:
            raise Error('boot image load addresses do not match the requested offsets')
        kernel_address, tags_address = hex(kernel_address), hex(tags)
    kernel = data[page:page + kernel_size]
    unpacker = zlib.decompressobj(16 + zlib.MAX_WBITS)
    if unpacker.decompress(kernel) != combined or not unpacker.eof or unpacker.unused_data != tree:
        raise Error('gzip, firmware and appended device tree do not round-trip')
    start = page * (1 + -(-kernel_size // page))
    if data[start:start + ramdisk_size] != Path(ramdisk_path).read_bytes():
        raise Error('ramdisk bytes differ')
    return {'kernel_sha256': hashlib.sha256(kernel).hexdigest(), 'ramdisk_bytes': ramdisk_size,
            'header_version': version, 'kernel_address': kernel_address, 'tags_address': tags_address}
