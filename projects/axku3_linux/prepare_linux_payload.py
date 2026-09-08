#!/usr/bin/env python3
"""Prepare Linux boot files on the host; does not build a kernel or use the board.

Inputs: generated csr.json and the checksum-pinned kernel/OpenSBI/initramfs.
Outputs: board-specific DTS/DTB, kernel/OpenSBI copies, initramfs with banner
files, ordered boot.json and input/overlay/output hashes.
The address/layout checks keep images inside RAM and prevent overlap.
"""

import argparse
import hashlib
import json
import re
import shutil
import struct
from pathlib import Path

from litex.tools.litex_json2dts_linux import generate_dts


LOAD_ADDRESSES = {
    "Image": 0x40000000,
    "rv32.dtb": 0x40EF0000,
    "rootfs.cpio": 0x41000000,
    # LiteXTerm jumps to the last JSON image after loading all regions.
    # OpenSBI must therefore remain last in insertion order.
    "opensbi.bin": 0x40F00000,
}

# Official linux_2022_03_23.zip payload from Linux-on-LiteX-VexRiscv.
EXPECTED_INPUT_SHA256 = {
    "Image": "fa269a349ac423417c9d617ee77fa366b5576e131fc45cb7457c1ba3b63a5d50",
    "opensbi.bin": "f083d87ed8c607fa5f31a5aa46253e6aada9a7c47965daf9098f5069844f969a",
    "rootfs.cpio": "6b06ecb4da84007459ea94571602a9ea31d16bbbea4e5df10d263fccebf38caa",
}


# LiteX logo/tagline from its BIOS; UberDDR4 artwork for the AXKU3 example.
BANNER_TEXT = r"""
        __   _ __      _  __
       / /  (_) /____ | |/_/
      / /__/ / __/ -_)>  <
     /____/_/\__/\__/_/|_|
   Build your hardware, easily!

 _   _ _               ____  ____  ____  _  _
| | | | |__   ___ _ __ |  _ \|  _ \|  _ \| || |
| | | | '_ \ / _ \ '__|| | | | | | | |_) | || |_
| |_| | |_) |  __/ |   | |_| | |_| |  _ <|__   _|
 \___/|_.__/ \___|_|   |____/|____/|_| \_\  |_|

       LiteX + VexRiscv + UberDDR4
              ALINX AXKU3
"""


def banner_files():
    """Root-owned files added to the pinned Buildroot filesystem."""
    return (
        ('etc/profile.d', 0o040755, b''),
        ('etc/uberddr4-banner', 0o100644, BANNER_TEXT.encode('ascii')),
        ('usr/bin/uber-banner', 0o100755,
         b'#!/bin/sh\ncat /etc/uberddr4-banner\n'),
        ('etc/profile.d/uberddr4.sh', 0o100644,
         b'# Display only when entering an interactive login shell.\n'
         b'if [ -n "$PS1" ]; then\n    /usr/bin/uber-banner\nfi\n'),
    )


def build_banner_overlay():
    """Deterministic newc archive, concatenated after the upstream trailer.

    Linux initramfs accepts multiple archives; a separate trailer keeps their
    inode/hard-link namespaces independent. Preserve all upstream bytes.
    """
    archive = bytearray()
    entries = (*banner_files(), ('TRAILER!!!', 0, b''))
    for ino, (name, mode, data) in enumerate(entries, 1):
        filename = name.encode('ascii') + b'\0'
        fields = (ino, mode, 0, 0, 2 if mode == 0o040755 else 1,
                  0, len(data), 0, 0, 0, 0, len(filename), 0)
        archive.extend(b'070701' + ''.join(f'{v:08x}' for v in fields).encode('ascii'))
        archive.extend(filename)
        archive.extend(b'\0' * (-len(archive) % 4))
        archive.extend(data)
        archive.extend(b'\0' * (-len(archive) % 4))
    archive.extend(b'\0' * (-len(archive) % 512))
    return bytes(archive)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def checked_input(image_dir, name):
    path = image_dir / name
    require(path.is_file() and path.stat().st_size > 0, f"Missing Linux input: {path}")
    digest = sha256(path)
    require(digest == EXPECTED_INPUT_SHA256[name],
            f"Unexpected {name} SHA-256: {digest}")
    return path


def check_layout(csr, file_sizes):
    """Treat each payload as a half-open byte range [start, end) in main RAM."""
    main_ram = csr["memories"]["main_ram"]
    ram_base = int(main_ram["base"])
    ram_end = ram_base + int(main_ram["size"])
    require((ram_base, ram_end) == (0x40000000, 0x80000000),
            "Linux payload requires the qualified 1 GiB UberDDR4 CPU window")

    regions = []
    for name, size in file_sizes.items():
        start = LOAD_ADDRESSES[name]
        end = start + size
        require(ram_base <= start < end <= ram_end,
                f"{name} lies outside main RAM: 0x{start:08x}-0x{end:08x}")
        regions.append((start, end, name))
    regions.sort()
    for (_, previous_end, previous_name), (start, _, name) in zip(regions, regions[1:]):
        require(previous_end <= start,
                f"Payload overlap: {previous_name} reaches 0x{previous_end:08x}, "
                f"but {name} starts at 0x{start:08x}")


def adapt_board_dts(dts, csr):
    """Remove the reset device that this minimal SoC deliberately does not implement."""
    # The standard Linux driver assumes reset at +0 and scratch at +4.
    # This board deliberately has no software reset: scratch is at +0 and
    # the read-only bus-error counter is at +4. Do not advertise that ABI.
    registers = csr['csr_registers']
    base = int(csr['csr_bases']['ctrl'])
    require('ctrl_reset' not in registers, 'Unexpected software-reset CSR')
    require(registers['ctrl_scratch']['addr'] == base and
            registers['ctrl_scratch']['type'] == 'rw' and
            registers['ctrl_bus_errors']['addr'] == base + 4 and
            registers['ctrl_bus_errors']['type'] == 'ro',
            'Unexpected reset-free SoC controller layout')
    dts, count = re.subn(r'\s*soc_ctrl0: soc_controller@[0-9a-f]+\s*\{[^{}]*\};',
                        '', dts)
    require(count == 1, 'Expected exactly one generated SoC-controller node')
    return dts


# The pinned RV32 kernel maps this page at 0xfffff000. Its copy routines
# cannot handle an exclusive end pointer wrapping to zero. Keep it out of
# the allocator, equivalent to the later upstream RISC-V last-page fix.
RV32_RESERVED_PAGE = 0x7FFFF000
RV32_PAGE_SIZE = 0x1000


def reserve_rv32_last_page(dts, csr):
    ram = csr["memories"]["main_ram"]
    require((int(ram["base"]), int(ram["size"])) == (0x40000000, 0x40000000),
            "RV32 last-page reservation requires the 1 GiB RAM window")
    node = """
            rv32-last-page@7ffff000 {
                reg = <0x7ffff000 0x1000>;
                no-map;
            };
"""
    dts, count = re.subn(r'(reserved-memory\s*\{[^{}]*ranges;)', lambda m: m[0] + node, dts)
    require(count == 1, "Expected exactly one reserved-memory node")
    return dts


def validate_rv32_reservation(tree):
    """Check the parsed tree, including after flattening, not just DTS text."""
    try:
        node = tree.get_node("/reserved-memory/rv32-last-page@7ffff000")
    except ValueError as error:
        raise RuntimeError("Missing RV32 last-page reservation; run prepare_linux_payload.ps1") from error
    require(node is not None, "Missing RV32 last-page reservation; regenerate the payload")
    reg = node.get_property("reg")
    require(reg is not None and list(reg.data) == [RV32_RESERVED_PAGE, RV32_PAGE_SIZE],
            "Wrong RV32 last-page reservation range")
    require(node.get_property("no-map") is not None,
            "RV32 last-page reservation must have no-map")


def validate_dts(dts, csr, initrd_size):
    require('litex,soc-controller' not in dts,
            'Reset-free controller must not bind the incompatible Linux driver')
    uart = int(csr["csr_bases"]["uart"])
    initrd_start = LOAD_ADDRESSES["rootfs.cpio"]
    initrd_end = initrd_start + initrd_size
    required = (
        f"earlycon=liteuart,0x{uart:x}",
        "rootwait root=/dev/ram0",
        f"linux,initrd-start = <0x{initrd_start:x}>;",
        f"linux,initrd-end   = <0x{initrd_end:x}>;",
        'riscv,isa = "rv32i2p0_ma";',
        'mmu-type = "riscv,sv32";',
        "memory@40000000",
        "serial@f0001000",
        "interrupt-controller@f0c00000",
        "clint@f0010000",
    )
    for text in required:
        require(text in dts, f"Generated DTS is missing: {text}")


def main():
    parser = argparse.ArgumentParser(
        description="Prepare a deterministic AXKU3 LiteX Linux serial-boot payload")
    parser.add_argument("--csr-json", required=True)
    parser.add_argument("--image-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    csr_path = Path(args.csr_json).resolve()
    image_dir = Path(args.image_dir).resolve()
    output_dir = Path(args.output_dir).resolve()
    require(csr_path.is_file(), f"Missing generated CSR JSON: {csr_path}")
    with csr_path.open(encoding="utf-8") as stream:
        csr = json.load(stream)

    inputs = {name: checked_input(image_dir, name)
              for name in EXPECTED_INPUT_SHA256}
    output_dir.mkdir(parents=True, exist_ok=True)
    for name, source in inputs.items():
        shutil.copyfile(source, output_dir / name)

    initramfs = output_dir / "rootfs.cpio"
    overlay = build_banner_overlay()
    with initramfs.open('ab') as stream:
        stream.write(b'\0' * (-stream.tell() % 4))
        stream.write(overlay)

    dts = generate_dts(
        csr,
        initrd=str(initramfs),
        polling=False,
        root_device="ram0",
    )
    dts = adapt_board_dts(dts, csr)
    dts = reserve_rv32_last_page(dts, csr)
    validate_dts(dts, csr, initramfs.stat().st_size)
    dts_path = output_dir / "rv32.dts"
    dts_path.write_text(dts, encoding="utf-8", newline="\n")

    # fdt 0.3.3 is a small Apache-2.0 pure-Python replacement for dtc.  It is
    # pinned by version/hash in dependencies.json, avoiding a global dtc install.
    import fdt
    device_tree = fdt.parse_dts(dts)
    validate_rv32_reservation(device_tree)
    dtb = device_tree.to_dtb(version=17)
    require(struct.unpack(">I", dtb[:4])[0] == 0xD00DFEED,
            "Generated DTB has the wrong flattened-device-tree magic")
    parsed_dtb = fdt.parse_dtb(dtb)
    validate_rv32_reservation(parsed_dtb)
    normalized = parsed_dtb.to_dts()
    for text in ("0x41000000", "serial@f0001000",
                 "interrupt-controller@f0c00000", "clint@f0010000"):
        require(text in normalized, f"DTB round-trip lost required content: {text}")
    chosen = parsed_dtb.get_node('/chosen')
    require(list(chosen.get_property('linux,initrd-end').data) ==
            [LOAD_ADDRESSES['rootfs.cpio'] + initramfs.stat().st_size],
            'DTB does not cover the complete banner initramfs')
    (output_dir / "rv32.dtb").write_bytes(dtb)

    file_sizes = {name: (output_dir / name).stat().st_size
                  for name in LOAD_ADDRESSES}
    check_layout(csr, file_sizes)
    boot = {name: f"0x{address:08x}" for name, address in LOAD_ADDRESSES.items()}
    require(next(reversed(boot)) == "opensbi.bin",
            "OpenSBI must be the final boot.json entry (LiteXTerm jump target)")
    (output_dir / "boot.json").write_text(
        json.dumps(boot, indent=4) + "\n", encoding="utf-8", newline="\n")

    manifest_files = {}
    for name in (*LOAD_ADDRESSES, "rv32.dts", "boot.json"):
        path = output_dir / name
        manifest_files[name] = {
            "bytes": path.stat().st_size,
            "sha256": sha256(path),
        }
    manifest = {
        "source_archive_sha256":
            "9ad7a043ce941024ccdcB08edcf0d627da0577a40be90579acb448c6b07fab48".lower(),
        "input_files_sha256": dict(EXPECTED_INPUT_SHA256),
        "initramfs_overlay": {
            "sha256": hashlib.sha256(overlay).hexdigest(),
            "bytes": len(overlay),
            "files": [name for name, _, _ in banner_files()],
        },
        "csr_json_sha256": sha256(csr_path),
        "fdt_version": fdt.__version__,
        "load_addresses": boot,
        "files": manifest_files,
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=4, sort_keys=True) + "\n",
        encoding="utf-8", newline="\n")

    print("AXKU3 LiteX Linux payload validation OK")
    print(f"  Output  : {output_dir}")
    print(f"  DTB     : {len(dtb)} bytes, SHA-256 {manifest_files['rv32.dtb']['sha256']}")
    print(f"  Initramfs: 0x41000000-0x{0x41000000 + file_sizes['rootfs.cpio']:08x}")
    print("  Inputs  : official 2022-03-23 Linux-on-LiteX-VexRiscv images (hash-pinned)")


if __name__ == "__main__":
    main()
