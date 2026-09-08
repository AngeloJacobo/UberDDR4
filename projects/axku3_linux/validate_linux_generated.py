#!/usr/bin/env python3
"""Check generated Linux hardware before synthesis, without opening the board.

Checks source availability, pins, clocks, CPU/MMU, memory map and disabled BIST.
This verifies the generated design's structure, not its timing or RAM integrity.
"""

import argparse
import json
import re
from pathlib import Path


STEM = "axku3_vexriscv_uberddr4"
LINUX_CPU = (
    "VexRiscvLitexSmpCluster_Cc1_Iw32Is4096Iy1_"
    "Dw32Ds4096Dy1_ITs4DTs4_Ood_Wm.v"
)
DATA_RATE_CONFIGS = {
    1200: (150_000_000, 6668, 1667),
    1250: (156_250_000, 6400, 1600),
    1600: (200_000_000, 5000, 1250),
    1866: (233_333_333, 4284, 1071),
    2133: (266_666_667, 3752, 938),
    2400: (300_000_000, 3332, 833),
}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def pin_assignments(text, property_name):
    pattern = re.compile(
        rf"^set_property {property_name} ([A-Z0-9]+) \[get_ports (.+)\]$"
    )
    assignments = {}
    for line in text.splitlines():
        match = pattern.match(line)
        if match:
            pin, port = match.groups()
            assignments[port.strip("{}")] = pin
    return assignments


def validate_source_paths(tcl, gateware):
    """Resolve inputs exactly where Vivado will read them, including CPU RAM RTL."""
    sources = re.findall(r'^(?:read_verilog|add_files|read_xdc) (.+)$', tcl, re.MULTILINE)
    require(bool(sources), 'No synthesis inputs found')
    for source in sources:
        path = Path(source.strip().strip('{}'))
        if not path.is_absolute():
            path = gateware / path
        require(path.is_file() and path.stat().st_size > 0,
                f'Missing synthesis input: {path}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--reference-xdc", required=True)
    parser.add_argument("--uart-name", choices=("serial",), default="serial")
    parser.add_argument("--uart-baudrate", type=int, required=True)
    parser.add_argument("--data-rate", type=int, choices=tuple(DATA_RATE_CONFIGS), required=True)
    args = parser.parse_args()
    sys_clk_freq, controller_period, ddr4_period = DATA_RATE_CONFIGS[args.data_rate]

    output = Path(args.output_dir)
    gateware = output / "gateware"
    generated = output / "software" / "include" / "generated"
    required = {
        "bios": output / "software" / "bios" / "bios.bin",
        "verilog": gateware / f"{STEM}.v",
        "xdc": gateware / f"{STEM}.xdc",
        "tcl": gateware / f"{STEM}.tcl",
        "mem": generated / "mem.h",
        "csr_csv": output / "csr.csv",
        "csr_json": output / "csr.json",
    }
    for name, path in required.items():
        require(path.is_file() and path.stat().st_size > 0, f"Missing {name}: {path}")
    require(required["bios"].stat().st_size <= 0x10000, "BIOS exceeds integrated ROM")

    verilog = required["verilog"].read_text(encoding="utf-8")
    xdc = required["xdc"].read_text(encoding="utf-8")
    tcl = required["tcl"].read_text(encoding="utf-8")
    validate_source_paths(tcl, gateware)
    mem = required["mem"].read_text(encoding="utf-8")
    csr = json.loads(required["csr_json"].read_text(encoding="utf-8"))

    pins = pin_assignments(xdc, "LOC")
    require(len(pins) == 81 and len(set(pins.values())) == 81,
            f"Expected 81 unique physical-UART design pins, got {len(pins)}")
    reference = pin_assignments(
        Path(args.reference_xdc).read_text(encoding="utf-8"), "PACKAGE_PIN"
    )
    new_ddr = {pin for port, pin in pins.items() if port.startswith("ddr4_")}
    ref_ddr = {pin for port, pin in reference.items() if port.startswith("ddr4_")}
    require(len(new_ddr) == 71 and new_ddr == ref_ddr,
            "DDR pin set differs from the qualified AXKU3 design")
    require(pins.get("serial_rx") == "AE15" and pins.get("serial_tx") == "AD15",
            "Physical CP2102 UART pins changed")

    for name, value in (
        ("CONTROLLER_CLK_PERIOD", f"12'd{controller_period}"),
        ("DDR4_CLK_PERIOD", f"10'd{ddr4_period}"),
        ("PHY_IMPL", "1'd1"),
        ("BIST_MODE", "1'd0"),
        ("BYTE_LANES", "32'd4"),
        ("DEBUG_CSR_ENABLE", "1'd1"),
    ):
        require(re.search(rf"\.{name}\s+\({re.escape(value)}\)", verilog),
                f"Generated parameter mismatch: {name}={value}")

    constants = csr["constants"]
    memories = csr["memories"]
    csr_bases = csr["csr_bases"]
    require("config_cpu_type_vexriscv_smp" in constants,
            "VexRiscv-SMP CPU constant missing")
    require("config_cpu_variant_linux" in constants,
            "Linux CPU variant constant missing")
    require(constants.get("config_cpu_count") == 1, "CPU count is not one")
    require(constants.get("config_cpu_mmu") == "sv32", "Sv32 MMU constant missing")
    require("a" in str(constants.get("config_cpu_isa", "")).lower(),
            "Atomic ISA extension missing")
    require(constants.get("config_clock_frequency") == sys_clk_freq,
            f"Linux system clock is not {sys_clk_freq} Hz")
    require(csr_bases.get("uart") == 0xF0001000, "Linux UART CSR is not fixed at slot 2")
    require(csr_bases.get("timer0") == 0xF0001800, "Linux timer CSR is not fixed at slot 3")
    require(memories["main_ram"] == {"base": 0x40000000, "size": 0x40000000, "type": "cached"},
            "Linux main RAM map is not the 1 GiB UberDDR4 window")
    require(memories["opensbi"]["base"] == 0x40F00000,
            "OpenSBI linker region moved")

    for value in (
        "#define MAIN_RAM_BASE 0x40000000L",
        "#define MAIN_RAM_SIZE 0x40000000",
        "#define UBERDDR4_DEBUG_BASE 0xf1000000L",
        "#define UBERDDR4_DEBUG_SIZE 0x00000040",
    ):
        require(value in mem, f"Memory map mismatch: {value}")

    require(LINUX_CPU in tcl, f"Pinned pre-generated CPU source missing: {LINUX_CPU}")
    require("VexRiscv_Lite.v" not in tcl, "Bare-metal Lite CPU leaked into Linux build")
    require("serial_tx" in verilog and "serial_rx" in verilog,
            "Physical UART is absent")
    require("create_debug_core" not in tcl and "create_debug_core" not in xdc,
            "Unexpected ILA/VIO creation in Linux image")
    require("phys_opt_design -directive ExploreWithAggressiveHoldFix" in tcl,
            "Qualified post-route physical optimization directive missing")

    require("BIST_ADDR_BITS_OVERRIDE" not in verilog,
            "Debug-only BIST parameter must not be required by this example")
    print("Generated AXKU3 Linux + UberDDR4 validation OK")
    print(f"  BIOS    : {required['bios'].stat().st_size} bytes / 65536")
    print("  CPU     : 1x VexRiscv-SMP Linux, Sv32, RV32IMA")
    print(f"  Memory  : UberDDR4-{args.data_rate}, 1 GiB window, BIST disabled")
    print(f"  Console : physical UART {args.uart_baudrate} baud")


if __name__ == "__main__":
    main()
