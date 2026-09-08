#!/usr/bin/env python3
"""Pre-synthesis checks for the retained small, non-Linux bring-up target.

build.ps1 uses this only with -Linux:$false. The normal Linux build instead
uses validate_linux_generated.py. Neither validator tests physical DDR4.
"""

import argparse
import re
from pathlib import Path


STEM = "axku3_vexriscv_uberddr4"


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--reference-xdc", required=True)
    parser.add_argument(
        "--uart-name", choices=("serial", "jtag_uart"), default="jtag_uart"
    )
    args = parser.parse_args()

    output = Path(args.output_dir)
    gateware = output / "gateware"
    generated = output / "software" / "include" / "generated"
    bios = output / "software" / "bios" / "bios.bin"
    verilog = gateware / f"{STEM}.v"
    xdc = gateware / f"{STEM}.xdc"
    tcl = gateware / f"{STEM}.tcl"
    mem_h = generated / "mem.h"
    csr_csv = output / "csr.csv"
    for path in (bios, verilog, xdc, tcl, mem_h, csr_csv):
        require(path.is_file() and path.stat().st_size > 0, f"Missing artifact: {path}")
    require(bios.stat().st_size <= 0x10000, "BIOS exceeds integrated ROM")

    verilog_text = verilog.read_text(encoding="utf-8")
    xdc_text = xdc.read_text(encoding="utf-8")
    tcl_text = tcl.read_text(encoding="utf-8")
    mem_text = mem_h.read_text(encoding="utf-8")
    csr_text = csr_csv.read_text(encoding="utf-8")

    new_pins = pin_assignments(xdc_text, "LOC")
    expected_pins = 81 if args.uart_name == "serial" else 79
    require(len(new_pins) == expected_pins, f"Expected {expected_pins} pins, got {len(new_pins)}")
    require(len(set(new_pins.values())) == expected_pins, "Duplicate package pins")

    reference_text = Path(args.reference_xdc).read_text(encoding="utf-8")
    reference_pins = pin_assignments(reference_text, "PACKAGE_PIN")
    new_ddr = {pin for port, pin in new_pins.items() if port.startswith("ddr4_")}
    reference_ddr = {pin for port, pin in reference_pins.items() if port.startswith("ddr4_")}
    require(len(new_ddr) == 71, f"Expected 71 DDR pins, got {len(new_ddr)}")
    require(new_ddr == reference_ddr, "DDR pin set differs from qualified AXKU3 design")

    for value in ("ddr4_top #(", "assign sys_clk = uber_clk;"):
        require(value in verilog_text, f"Generated topology mismatch: {value}")
    for name, value in (
        ("CONTROLLER_CLK_PERIOD", "12'd3332"),
        ("DDR4_CLK_PERIOD", "10'd833"),
        ("PHY_IMPL", "1'd1"),
        ("BIST_MODE", "1'd0"),
        ("DEBUG_CSR_ENABLE", "1'd1"),
    ):
        require(
            re.search(rf"\.{name}\s+\({re.escape(value)}\)", verilog_text),
            f"Generated parameter mismatch: {name}={value}",
        )
    require(verilog_text.count("MMCME4_ADV #(") == 1, "Expected one shared MMCM")
    require("BSCANE2" in verilog_text if args.uart_name == "jtag_uart" else "serial_tx" in verilog_text,
            "Selected console is absent")
    require("ctrl_reset" not in csr_text, "Unsafe 300-to-200 MHz software reset is present")
    require("ctrl_scratch" in csr_text and "ctrl_bus_errors" in csr_text,
            "Controller diagnostics are missing")
    require("basesoc_reset_re" not in verilog_text, "Software reset pulse remains in gateware")

    for value in (
        "#define MAIN_RAM_BASE 0x40000000L",
        "#define MAIN_RAM_SIZE 0x40000000",
        "#define UBERDDR4_DEBUG_BASE 0xf1000000L",
        "#define UBERDDR4_DEBUG_SIZE 0x00000040",
    ):
        require(value in mem_text, f"Memory map mismatch: {value}")

    require("-part xcku3p-ffvb676-2-i" in tcl_text, "Vivado part mismatch")
    for source in (
        "ddr4_top.v", "ddr4_controller.v", "ddr4_prober.v", "ddr4_phy.v",
        "ddr4_phy_native.v", "ddr4_phy_native_adapter.v",
        "ddr4_phy_native_byte.v", "ddr4_phy_native_reset.v",
    ):
        require(source in tcl_text, f"Missing source in Vivado script: {source}")
    for source_path in (Path(p) for p in re.findall(r"read_verilog \{([^}]+)\}", tcl_text)):
        if "source-uberddr4\\rtl" in str(source_path) or "source-uberddr4/rtl" in str(source_path):
            require(
                'mark_debug = "true"' not in source_path.read_text(encoding="utf-8"),
                f"Application snapshot retained MARK_DEBUG probes: {source_path}",
            )
    require("VexRiscv_Lite.v" in tcl_text, "VexRiscv Lite source missing")
    require(
        "phys_opt_design -directive ExploreWithAggressiveHoldFix" in tcl_text,
        "Qualified post-route physical optimization directive missing",
    )
    require("create_clock -name jtag_tck -period 20.000" in tcl_text,
            "JTAG TCK constraint missing")
    require("set_clock_groups -asynchronous -group [get_clocks jtag_tck]" in tcl_text,
            "JTAG CDC exception missing")
    require("create_waiver -type METHODOLOGY -id TIMING-2" in tcl_text,
            "Narrow BSCANE2 waiver missing")
    require("create_debug_core" not in tcl_text and "create_debug_core" not in xdc_text,
            "Unexpected ILA/debug core in application build")

    print("Generated UberDDR4 application validation OK")
    print(f"  BIOS       : {bios.stat().st_size} bytes / 65536")
    print(f"  Board pins : {len(new_pins)} unique; DDR set matches qualified design")
    print("  Memory     : DDR4-2400 x32, 2 GiB physical / 1 GiB CPU window")
    print("  Interface  : LiteX 32-bit classic -> UberDDR4 256-bit pipelined Wishbone")
    print("  Debug CSR  : 0xf1000000-0xf100003f")
    print("  Reset      : hardware/JTAG only; controller diagnostics retained")


if __name__ == "__main__":
    main()
