#!/usr/bin/env python3
"""Decide whether the synthesized Linux SoC is feasible to place and route.

Read Vivado synthesis timing, utilization and clock-coverage reports plus the
checkpoint. Estimated pre-route slack is not final timing closure; only the
implementation validator can approve a bitstream for hardware testing.
"""

import argparse
import re
from pathlib import Path


STEM = "axku3_vexriscv_uberddr4"
SYS_CLOCK_MHZ = {
    1200: "150.000",
    1250: "156.250",
    1600: "200.000",
    1866: "233.333",
    2133: "266.667",
    2400: "300.000",
}
PHY_CLOCK_MHZ = {
    1200: "1200.000",
    1250: "1250.000",
    1600: "1600.000",
    1866: "1866.667",
    2133: "2133.333",
    2400: "2400.000",
}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read_required(path):
    require(path.is_file() and path.stat().st_size > 0,
            f"Missing synthesis artifact: {path}")
    return path.read_text(encoding="utf-8", errors="replace")


def utilization(text, resource):
    match = re.search(
        rf"^\|\s*{re.escape(resource)}\*?\s*\|\s*([0-9.]+)\s*\|.*?\|\s*([0-9.]+)\s*\|$",
        text,
        re.MULTILINE,
    )
    require(match is not None, f"Could not parse {resource} utilization")
    # A lone RAMB18E2 occupies half a Block RAM Tile, so that count is
    # reported fractionally. Keep whole counts as integers for reporting.
    count = float(match.group(1))
    return (int(count) if count.is_integer() else count), float(match.group(2))


def main():
    parser = argparse.ArgumentParser(description="Gate AXKU3 Linux synthesis feasibility")
    parser.add_argument("--gateware-dir", required=True)
    parser.add_argument("--data-rate", type=int, choices=tuple(SYS_CLOCK_MHZ), required=True)
    args = parser.parse_args()

    gateware = Path(args.gateware_dir)
    timing = read_required(gateware / f"{STEM}_timing_synth.rpt")
    utilization_report = read_required(gateware / f"{STEM}_utilization_synth.rpt")
    checks = read_required(gateware / f"{STEM}_check_timing_synth.rpt")
    checkpoint = gateware / f"{STEM}_synth.dcp"
    require(checkpoint.is_file() and checkpoint.stat().st_size > 0,
            "Missing synthesized checkpoint")

    lines = timing.splitlines()
    summary = None
    for index, line in enumerate(lines):
        if "WNS(ns)" in line and "TNS Failing Endpoints" in line and "WPWS(ns)" in line:
            values = lines[index + 2].split()
            if len(values) == 12:
                summary = values
                break
    require(summary is not None, "Could not parse synthesis timing summary")
    wns, _, setup_fail, _, whs, _, hold_fail, _, wpws, _, pulse_fail, _ = summary

    # Pre-place net-delay estimates are deliberately pessimistic and hold
    # numbers are not actionable. Reject only a large setup miss; this target
    # is close enough to proceed when WNS is at least -0.5 ns.
    require(float(wns) >= -0.500,
            f"Synthesis setup feasibility failed: WNS={wns} ns")
    require("There are 0 register/latch pins with no clock." in checks,
            "Sequential logic without a clock")
    require("There are 0 pins that are not constrained for maximum delay." in checks,
            "Genuinely unconstrained maximum-delay endpoints")
    require("There are 0 generated clocks that are not connected to a clock source." in checks,
            "Disconnected generated clock")
    require("There are 0 combinational loops in the design." in checks,
            "Combinational loop detected")
    for frequency in ("200.000", "150.000", SYS_CLOCK_MHZ[args.data_rate],
                      PHY_CLOCK_MHZ[args.data_rate]):
        require(frequency in timing, f"Expected {frequency} MHz clock is absent")

    luts, lut_percent = utilization(utilization_report, "CLB LUTs")
    registers, register_percent = utilization(utilization_report, "CLB Registers")
    bram, bram_percent = utilization(utilization_report, "Block RAM Tile")
    uram, uram_percent = utilization(utilization_report, "URAM")
    dsps, dsp_percent = utilization(utilization_report, "DSPs")
    for name, percent in (("LUT", lut_percent), ("register", register_percent),
                          ("BRAM", bram_percent), ("URAM", uram_percent),
                          ("DSP", dsp_percent)):
        require(percent < 80.0, f"{name} utilization is infeasible: {percent:.2f}%")
    # 10 RAMB36E2 plus the RAMB18E2 holding the identifier ROM, which the
    # data-rate label pushed past 64 bytes.
    require((bram, uram, dsps) == (10.5, 1, 4),
            f"Unexpected memory/DSP mapping: BRAM={bram}, URAM={uram}, DSP={dsps}")

    print("AXKU3 Linux + UberDDR4 synthesis feasibility OK")
    print(f"  Setup   : WNS {wns} ns ({setup_fail} pre-place failing endpoints)")
    print(f"  Hold/PW : WHS {whs} ns ({hold_fail}), WPWS {wpws} ns ({pulse_fail}); route-gated later")
    print(f"  Logic   : {luts} LUTs ({lut_percent:.2f}%), "
          f"{registers} registers ({register_percent:.2f}%)")
    print(f"  Memory  : {bram} BRAM, {uram} URAM; DSP: {dsps}")
    print("  Timing  : all sequential logic clocked; no genuine max-delay gaps")


if __name__ == "__main__":
    main()
