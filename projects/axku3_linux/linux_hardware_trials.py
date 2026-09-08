#!/usr/bin/env python3
"""Host-side program/boot/test runner, called by boot.ps1 (requires the board).

Each trial programs volatile FPGA configuration over JTAG, then owns the UART:
BIOS RAM test -> image upload/readback CRC -> OpenSBI/Linux -> userspace checks.
Outputs are a manifest, per-trial UART/SFL traces, and a summary of passed trials.
A failure stops the campaign; it is not silently retried or counted as a pass.
"""

import argparse
import hashlib
import json
import re
import subprocess
import sys
import time
import zlib
from pathlib import Path
from serial_trace import TracedLiteXTerm

import serial
from serial.tools import list_ports

from litex.tools.litex_term import (
    sfl_magic_ack,
    sfl_magic_req,
)

ROOT_PROMPTS = (b'~ # ', b'/ # ', b'\x1b[00m# ')


def write_console(port, data):
    # Linux 5.14 LiteUART polls a 16-byte RX FIFO (unlike the BIOS ISR).
    # Pace pasted commands; binary SFL uploads retain their fast ACK pacing.
    for offset in range(0, len(data), 8):
        chunk = data[offset:offset + 8]
        require(port.write(chunk) == len(chunk), 'Short Linux console write')
        time.sleep(0.02)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


class PersistentTranscript(bytearray):
    """Keep every received UART byte even when a hardware trial aborts."""

    def __init__(self, path):
        super().__init__()
        self.path = path
        self.path.write_bytes(b"")

    def extend(self, data):
        super().extend(data)
        with self.path.open("ab") as stream:
            stream.write(data)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def select_port(requested):
    """Prefer an explicit COM port; auto-selection refuses ambiguous USB setups."""
    ports = list(list_ports.comports())
    by_name = {port.device.upper(): port for port in ports}
    if requested.lower() != "auto":
        require(requested.upper() in by_name, f"Serial port is absent: {requested}")
        selected = by_name[requested.upper()]
    else:
        candidates = [port for port in ports if port.vid is not None]
        require(len(candidates) == 1,
                "Expected exactly one USB UART COM port; found " +
                ", ".join(f"{p.device} ({p.description})" for p in candidates))
        selected = candidates[0]
    require(not selected.hwid.upper().startswith(("PCI\\", "BTHENUM\\")),
            f"Refusing non-USB-board serial port: {selected.device} ({selected.description})")
    print(f"UART: {selected.device} -- {selected.description} -- {selected.hwid}")
    return selected.device


def read_until(port, transcript, markers, timeout, label, start=None):
    """Return a marker's end offset so later waits cannot reuse an old prompt."""
    if isinstance(markers, bytes):
        markers = (markers,)
    if start is None:
        start = len(transcript)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        window = transcript[start:]
        require(b'Kernel panic - not syncing:' not in window,
                f'Kernel panic while waiting for {label}; see saved UART transcript')
        for marker in markers:
            offset = window.find(marker)
            if offset >= 0:
                return marker, start + offset + len(marker)
        data = port.read(max(1, port.in_waiting))
        if data:
            transcript.extend(data)
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
    raise RuntimeError(f"Timed out waiting for {label}")


def value(transcript, name):
    match = re.search(rb"\b" + name.encode("ascii") + rb"=(0x[0-9a-fA-F]+)", transcript)
    require(match is not None, f"Missing Linux CSR result: {name}")
    return int(match.group(1), 16)


def program(xsdb, tcl, bitstream):
    result = subprocess.run(
        [str(xsdb), str(tcl), str(bitstream)],
        capture_output=True,
        text=True,
        timeout=90,
        check=False,
    )
    output = result.stdout + result.stderr
    print(output, end="")
    require(result.returncode == 0 and "AXKU3_PROGRAM_PASS" in output,
            f"XSDB programming failed with exit code {result.returncode}")


def upload_and_test(port_name, baudrate, xsdb, tcl, bitstream, boot_json,
                    transcript_path, software_megabytes, sfl_frame_bytes,
                    sfl_outstanding):
    """Keep one serial handle through programming, BIOS loading and Linux tests."""
    transcript = PersistentTranscript(transcript_path)
    terminal = TracedLiteXTerm(True, None, "0x40000000", str(boot_json), False,
                              trace_path=transcript_path.with_suffix('.sfl.json'),
                              stable_timeouts=True)
    # Pin the diagnostic profile. SFL errors alone do not identify whether
    # the cable, UART receiver, or software buffering is responsible.
    terminal.safe = True
    terminal.length = sfl_frame_bytes
    terminal.outstanding = sfl_outstanding
    windows = []
    for outstanding in (sfl_outstanding, min(4, sfl_outstanding), 1):
        profile = (sfl_frame_bytes, outstanding)
        if profile not in windows:
            windows.append(profile)
    safe_profile = (min(64, sfl_frame_bytes), 1)
    if safe_profile not in windows:
        windows.append(safe_profile)
    # Safe mode skips LiteX's calibration, which otherwise replaces the
    # board-qualified frame size with 251-byte bursts.  Retain adaptive
    # window fallback while keeping the CRC boundary fixed.
    terminal.upload_profiles = lambda: windows

    # Timeout property setters also invoke SetCommState on Windows. Keep
    # bounded one-second settings fixed throughout the transfer and console.
    with serial.Serial(port_name, baudrate, timeout=1.0, write_timeout=1.0) as port:
        port.reset_input_buffer()
        port.reset_output_buffer()
        program(xsdb, tcl, bitstream)

        # This BIOS goes directly to SFL without the F7 menu printed by
        # other LiteX versions. Enter SFL explicitly after its first timeout.
        _, cursor = read_until(
            port, transcript, b"\x1b[92;1mlitex\x1b[0m> ", 90, "LiteX BIOS console")
        port.write(b"mem_test 0x40000000 0x4000000\r")
        _, end = read_until(port, transcript, b"\x1b[92;1mlitex\x1b[0m> ", 180,
                            '64 MiB BIOS memory test', cursor)
        memory_test = transcript[cursor:end]
        require(b'(64.0MiB)' in memory_test and b'Memtest OK' in memory_test
                and b'Memtest KO' not in memory_test and b'Redeemed' not in memory_test,
                '64 MiB BIOS memory preflight failed')
        cursor = end
        port.write(b"serialboot\r")
        port.flush()
        read_until(port, transcript, sfl_magic_req, 10, "LiteX SFL request", cursor)
        port.write(sfl_magic_ack)
        port.flush()
        terminal.last_reply = time.monotonic()

        terminal.port = port
        terminal.port_url = port_name
        for filename, address in terminal.mem_regions.items():
            terminal.upload(filename, int(address, 16))
        terminal.save_trace()
        # SFL CRC protects frames in transit. Verify the complete DDR4 copy
        # independently through the BIOS before executing any Linux payload.
        require(terminal.abort_serialboot(), 'Could not return to BIOS for CRC verification')
        read_until(port, transcript, b"\x1b[92;1mlitex\x1b[0m> ", 10, 'BIOS CRC console')
        for filename, address in terminal.mem_regions.items():
            payload = Path(filename)
            expected_crc = 0
            with payload.open('rb') as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b''):
                    expected_crc = zlib.crc32(block, expected_crc)
            start = len(transcript)
            port.write(f"crc {address} {payload.stat().st_size}\r".encode('ascii'))
            read_until(port, transcript, b"\x1b[92;1mlitex\x1b[0m> ", 60,
                       f'DDR4 CRC for {payload.name}', start)
            observed = re.findall(rb'CRC32: ([0-9a-fA-F]{8})', transcript[start:])
            require(len(observed) == 1 and int(observed[0], 16) == expected_crc,
                    f'DDR4 payload CRC mismatch: {payload.name}')
            print(f'PAYLOAD_CRC_PASS {payload.name} {expected_crc:08x}', flush=True)
        port.write(f"boot {terminal.boot_address}\r".encode('ascii'))

        read_until(port, transcript, b"buildroot login:", 180, "Buildroot login")
        port.write(b"root\n")
        read_until(port, transcript, ROOT_PROMPTS, 30, "root shell")

        return test_userspace(port, transcript, software_megabytes)


def test_userspace(port, transcript, software_megabytes):
    """Check existing CSRs, known-zero contents and repeatability of random data.

The zero-file digest has a host-computed expected value. Matching random-file
hashes test repeatability only; they are not an independent known-data oracle.
Check zeros before overwriting that file, preserving the evidence on failure.
"""
    zero_digest = hashlib.sha256()
    for _ in range(software_megabytes):
        zero_digest.update(bytes(1024 * 1024))
    commands = (
        ("echo UBERDDR4_SWTEST_BEGIN", 10),
        ("uname -a", 10),
        ("free", 10),
        ("echo UBER_STATUS=$(/sbin/devmem 0xf1000000 32)", 10),
        ("echo UBER_TRAIN_FAIL=$(/sbin/devmem 0xf1000008 32)", 10),
        ("echo UBER_CORRECT=$(/sbin/devmem 0xf100000c 32)", 10),
        ("echo UBER_ERROR=$(/sbin/devmem 0xf1000010 32)", 10),
        ("echo UBER_BIST_STATUS=$(/sbin/devmem 0xf1000014 32)", 10),
        ("echo UBER_CONFIG=$(/sbin/devmem 0xf1000028 32)", 10),
        ("echo UBER_VERSION=$(/sbin/devmem 0xf100002c 32)", 10),
        ("echo UBER_INIT_PROGRESS=$(/sbin/devmem 0xf1000034 32)", 10),
        (f"dd if=/dev/zero of=/tmp/uberddr4_test.bin bs=1048576 "
         f"count={software_megabytes}", 180),
        ("echo UBER_ZERO_SIZE=$(wc -c < /tmp/uberddr4_test.bin)", 10),
        ("echo UBER_ZERO_HASH=$(sha256sum /tmp/uberddr4_test.bin | cut -d' ' -f1)", 60),
        (f"dd if=/dev/urandom of=/tmp/uberddr4_test.bin bs=1048576 "
         f"count={software_megabytes}", 180),
        ("echo UBER_FILE_SIZE=$(wc -c < /tmp/uberddr4_test.bin)", 10),
        ("echo UBER_HASH1=$(sha256sum /tmp/uberddr4_test.bin | cut -d' ' -f1)", 60),
        ("sync", 30),
        ("echo UBER_HASH2=$(sha256sum /tmp/uberddr4_test.bin | cut -d' ' -f1)", 60),
        ("rm -f /tmp/uberddr4_test.bin", 10),
    )
    for command, timeout in commands:
        command_start = len(transcript)
        write_console(port, (command + "\n").encode("ascii"))
        read_until(port, transcript, ROOT_PROMPTS, timeout,
                   f"shell completion: {command.split()[0]}")
        result_start = len(transcript)
        write_console(port, b"echo UBER_COMMAND_RC=$?\n")
        read_until(port, transcript, ROOT_PROMPTS, 10,
                   "command exit status", result_start)
        require(re.search(rb"UBER_COMMAND_RC=0\r?\n", transcript[result_start:]),
                f"Shell command failed: {command}")
        if command.startswith('echo UBER_ZERO_SIZE='):
            require(re.search(rb'UBER_ZERO_SIZE=\s*' +
                              str(software_megabytes * 1024 * 1024).encode() + rb'\r?\n',
                              transcript[command_start:]), 'Zero file has wrong length; file preserved')
        if command.startswith('echo UBER_ZERO_HASH='):
            require(re.search(rb'UBER_ZERO_HASH=' + zero_digest.hexdigest().encode() + rb'\r?\n',
                              transcript[command_start:]),
                    'Zero file hash mismatch; stopped before overwrite, file preserved')


    required = (
        b"OpenSBI",
        b"Linux version",
        b"32-bit RISC-V Linux running on LiteX / VexRiscv-SMP.",
        b"UBERDDR4_SWTEST_BEGIN",
    )
    for marker in required:
        require(marker in transcript, f"Transcript is missing marker: {marker!r}")
    require(b"Memtest KO" not in transcript, "BIOS reported Memtest KO")

    status = value(transcript, "UBER_STATUS")
    train_fail = value(transcript, "UBER_TRAIN_FAIL")
    correct = value(transcript, "UBER_CORRECT")
    error = value(transcript, "UBER_ERROR")
    bist_status = value(transcript, "UBER_BIST_STATUS")
    config = value(transcript, "UBER_CONFIG")
    version = value(transcript, "UBER_VERSION")
    init_progress = value(transcript, "UBER_INIT_PROGRESS")
    hashes = re.findall(rb"UBER_HASH[12]=([0-9a-fA-F]{64})", transcript)
    zero_matches = re.findall(rb"UBER_ZERO_HASH=([0-9a-fA-F]{64})", transcript)
    sizes = re.findall(rb"UBER_FILE_SIZE=\s*(\d+)\r?\n", transcript)

    require((status & 0x0FFF) == 0x00D0, f"Bad STATUS CSR: 0x{status:08x}")
    require(train_fail == 0, f"Nonzero TRAIN_FAIL CSR: 0x{train_fail:08x}")
    require(correct == 0, f"Disabled BIST has nonzero correct count: {correct}")
    require(error == 0, f"Nonzero BIST error count: {error}")
    require(bist_status == 0x40, f"Bad BIST_STATUS CSR: 0x{bist_status:08x}")
    require(config == 0x40, f"Bad CONFIG CSR: 0x{config:08x}")
    require(version == 1, f"Bad VERSION CSR: 0x{version:08x}")
    require((init_progress & 0x1C0) == 0x80,
            f"Bad INIT_PROGRESS CSR: 0x{init_progress:08x}")
    require(len(hashes) == 2 and hashes[0].lower() == hashes[1].lower(),
            "Linux userspace RAM hashes are absent or unequal")
    require(zero_matches == [zero_digest.hexdigest().encode("ascii")],
            "Known-content userspace RAM hash differs from host expectation")
    require(len(sizes) == 1 and int(sizes[0]) == software_megabytes * 1024 * 1024,
            "Random userspace RAM file has the wrong size")
    return {
        "bytes": len(transcript),
        "status": status,
        "train_fail": train_fail,
        "correct": correct,
        "error": error,
        "bist_status": bist_status,
        "config": config,
        "version": version,
        "init_progress": init_progress,
        "ram_sha256": hashes[0].decode("ascii").lower(),
    }


def main():
    parser = argparse.ArgumentParser(description="Repeat AXKU3 UberDDR4 Linux hardware boots")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--software-megabytes", type=int, default=16)
    parser.add_argument("--sfl-frame-bytes", type=int, default=251)
    parser.add_argument("--sfl-outstanding", type=int, default=8)
    parser.add_argument("--data-rate", type=int,
                        choices=(1200, 1250, 1600, 1866, 2133, 2400), required=True)
    parser.add_argument("--bitstream", required=True)
    parser.add_argument("--boot-json", required=True)
    parser.add_argument("--xsdb", required=True)
    parser.add_argument("--program-tcl", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    require(args.trials >= 1, "Trial count must be positive")
    require(args.software_megabytes >= 1, "Software memory test must be non-empty")
    require(1 <= args.sfl_frame_bytes <= 251,
            "SFL frame length must be between 1 and 251 bytes")
    require(1 <= args.sfl_outstanding <= 8,
            "SFL outstanding window must be between 1 and 8")

    bitstream = Path(args.bitstream).resolve()
    boot_json = Path(args.boot_json).resolve()
    xsdb = Path(args.xsdb).resolve()
    tcl = Path(args.program_tcl).resolve()
    output_dir = Path(args.output_dir).resolve()
    for path in (bitstream, boot_json, xsdb, tcl):
        require(path.is_file(), f"Missing hardware-test input: {path}")
    with boot_json.open(encoding="utf-8") as stream:
        boot = json.load(stream)
    require(list(boot)[-1] == "opensbi.bin" and boot["opensbi.bin"] == "0x40f00000",
            "boot.json does not jump to OpenSBI")
    port_name = select_port(args.port)
    output_dir.mkdir(parents=True, exist_ok=False)
    bit_digest = sha256(bitstream)
    payload_hashes = {name: sha256(boot_json.parent / name) for name in boot}
    # Record the exact inputs before starting. A later failure must not erase
    # prior passes, and changing a payload between trials invalidates the run.
    manifest = {
        "bitstream": str(bitstream), "bit_sha256": bit_digest,
        "boot_json": str(boot_json), "payload_sha256": payload_hashes,
        "data_rate_mtps": args.data_rate, "uart_baudrate": args.baudrate,
        "sfl_frame_bytes": args.sfl_frame_bytes,
        "sfl_outstanding": args.sfl_outstanding,
        "fixed_serial_timeouts": True,
        "runner_sha256": sha256(Path(__file__)),
        "serial_transport_sha256": sha256(Path(__file__).with_name('serial_trace.py')),
        "boot_json_sha256": sha256(boot_json),
        "requested_trials": args.trials,
        "software_megabytes": args.software_megabytes,
    }
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
    summary = output_dir / "summary.tsv"
    summary.write_text(
        "trial\tdata_rate_mtps\ttranscript_bytes\tstatus\ttrain_fail\tcorrect\terror\t"
        "bist_status\tconfig\tversion\tinit_progress\tram_sha256\tresult\n")

    rows = []
    for trial in range(1, args.trials + 1):
        print(f"\nTRIAL_{trial}_BEGIN")
        transcript_path = output_dir / f"trial_{trial:02d}_uart.bin"
        try:
            require(sha256(bitstream) == bit_digest, "Bitstream changed between trials")
            require({name: sha256(boot_json.parent / name) for name in boot} == payload_hashes,
                    "Linux payload changed between trials")
            result = upload_and_test(
                port_name, args.baudrate, xsdb, tcl, bitstream, boot_json,
                transcript_path, args.software_megabytes, args.sfl_frame_bytes,
                args.sfl_outstanding)
        except Exception as exc:
            (output_dir / f"trial_{trial:02d}_failure.json").write_text(
                json.dumps({"trial": trial, "error": str(exc)}, indent=2))
            raise
        rows.append((trial, result))
        print(f"TRIAL_{trial}_PASS: Linux + {args.software_megabytes} MiB userspace RAM test")

        with summary.open("a", encoding="utf-8", newline="\n") as stream:
            stream.write(
                f"{trial}\t{args.data_rate}\t{result['bytes']}\t0x{result['status']:08x}\t"
                f"0x{result['train_fail']:08x}\t{result['correct']}\t{result['error']}\t"
                f"0x{result['bist_status']:08x}\t0x{result['config']:08x}\t"
                f"0x{result['version']:08x}\t0x{result['init_progress']:08x}\t"
                f"{result['ram_sha256']}\tPASS\n")
    print(f"HARDWARE_TRIALS_PASS={args.trials}")
    print(f"DDR_DATA_RATE={args.data_rate}")
    print(f"BIT_SHA256={bit_digest}")
    print(f"HARDWARE_TRIALS_SUMMARY={summary}")


if __name__ == "__main__":
    main()
