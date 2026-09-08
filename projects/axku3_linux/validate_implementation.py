#!/usr/bin/env python3
"""Gate hardware testing on routed Vivado reports and a nonempty bitstream.

Require setup/hold/pulse timing closure, complete routing and only the narrowly
reviewed DRC/methodology findings below. A changed finding fails for review;
do not relax the checks just to obtain a pass. RAM data correctness is tested
separately by linux_hardware_trials.py, not inferred from timing reports.
"""

import argparse
import hashlib
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
    require(path.is_file() and path.stat().st_size > 0, f"Missing implementation artifact: {path}")
    return path.read_text(encoding="utf-8", errors="replace")


def report_rules(text):
    """Extract rule, severity and occurrence count from Vivado's summary table."""
    pattern = re.compile(
        r"^\|\s*([A-Z0-9-]+)\s*\|\s*(Critical Warning|Error|Warning|Advisory)\s*\|.*\|\s*(\d+)\s*\|$"
    )
    rules = {}
    for line in text.splitlines():
        match = pattern.match(line)
        if match:
            rule, severity, count = match.groups()
            rules[rule] = (severity, int(count))
    return rules


def main():
    parser = argparse.ArgumentParser(description="Gate the AXKU3 UberDDR4 application implementation")
    parser.add_argument("--gateware-dir", required=True)
    parser.add_argument("--uart-name", choices=("serial", "jtag_uart"), default="jtag_uart")
    parser.add_argument("--linux", action="store_true")
    parser.add_argument("--data-rate", type=int, choices=tuple(SYS_CLOCK_MHZ), required=True)
    args = parser.parse_args()

    gateware = Path(args.gateware_dir)
    timing = read_required(gateware / f"{STEM}_timing.rpt")
    unconstrained = read_required(gateware / f"{STEM}_timing_unconstrained_path.rpt")
    route = read_required(gateware / f"{STEM}_route_status.rpt")
    drc = read_required(gateware / f"{STEM}_drc.rpt")
    methodology = read_required(gateware / f"{STEM}_timing_methodology.rpt")
    bitstream = gateware / f"{STEM}.bit"
    require(bitstream.is_file() and bitstream.stat().st_size > 0, "Missing bitstream")

    summary = None
    lines = timing.splitlines()
    for index, line in enumerate(lines):
        if "WNS(ns)" in line and "TNS Failing Endpoints" in line and "WPWS(ns)" in line:
            values = lines[index + 2].split()
            if len(values) == 12:
                summary = values
                break
    require(summary is not None, "Could not parse routed timing summary")
    wns, _, setup_fail, _, whs, _, hold_fail, _, wpws, _, pulse_fail, _ = summary
    require(float(wns) >= 0 and int(setup_fail) == 0,
            f"Setup timing failed: WNS={wns}, endpoints={setup_fail}")
    require(float(whs) >= 0 and int(hold_fail) == 0,
            f"Hold timing failed: WHS={whs}, endpoints={hold_fail}")
    require(float(wpws) >= 0 and int(pulse_fail) == 0,
            f"Pulse-width timing failed: WPWS={wpws}, endpoints={pulse_fail}")
    require("All user specified timing constraints are met." in timing,
            "Vivado did not report timing constraints met")
    require("There are 0 pins that are not constrained for maximum delay." in unconstrained,
            "Implementation has genuinely unconstrained maximum-delay endpoints")
    require("36 pins that are not constrained for maximum delay due to constant clock" in unconstrained,
            "Constant-clock endpoint classification changed")
    for frequency in ("50.000", "150.000", "200.000",
                      SYS_CLOCK_MHZ[args.data_rate], PHY_CLOCK_MHZ[args.data_rate]):
        require(frequency in timing, f"Expected {frequency} MHz clock is absent")

    route_match = re.search(r"# of nets with routing errors\.*\s*:\s*(\d+)\s*:", route)
    require(route_match is not None, "Could not parse route status")
    require(int(route_match.group(1)) == 0, "Implementation has routing errors")

    drc_rules = report_rules(drc)
    expected_drc = ({
        "DPIP-2": ("Warning", 8),
        "DPOP-4": ("Warning", 3),
    } if args.linux else {})
    # The UART-500k placement leaves the CPU MUL_HH output register in fabric.
    # Accept only this reviewed DSP pipeline recommendation, never a new cell.
    if args.linux and "DPOP-3" in drc_rules:
        require(drc_rules["DPOP-3"] == ("Warning", 1), "DPOP-3 count/severity changed")
        finding = drc.split("DPOP-3#1 Warning", 1)[-1].split("Related violations:", 1)[0]
        require("DSP VexRiscvLitexSmpCluster" in finding and
                "/cores_0_cpu_logic_cpu/memory_to_writeBack_MUL_HH_reg output" in finding and
                "PREG=0" in finding, "DPOP-3 is not the reviewed CPU MUL_HH pipeline finding")
        expected_drc["DPOP-3"] = ("Warning", 1)
    require(drc_rules == expected_drc, f"Unexpected DRC result: {drc_rules}")
    require(f"Violations found: {sum(count for _, count in expected_drc.values())}" in drc,
            "DRC summary count changed")
    if args.linux:
        require("DSP VexRiscvLitexSmpCluster" in drc,
                "Linux DSP pipeline findings are not from the pinned VexRiscv core")
        require(all(severity == "Warning" for severity, _ in drc_rules.values()),
                f"Linux DRC has a severe finding: {drc_rules}")

    methodology_rules = report_rules(methodology)
    expected_methodology = {
        # The demonstrated Linux image omits the optional startup BIST, which
        # removes one of the three previously classified async-set flags.
        "LUTAR-1": ("Warning", 2 if args.linux else 3),
        "TIMING-18": ("Warning", 1),
        "CLKC-56": ("Advisory", 1),
        "CLKC-58": ("Advisory", 2),
    }
    require(methodology_rules == expected_methodology,
            f"Unexpected methodology result: {methodology_rules}")
    if args.uart_name == "jtag_uart":
        require("Violations waived: 1" in methodology,
                "Narrow JTAG methodology waiver is absent")
    require("An input delay is missing on cpu_reset_n relative to clock(s) clk200_p" in methodology,
            "The expected TIMING-18 finding is not the asynchronous reset input")
    require("TIMING-17" not in methodology_rules, "A sequential clock domain is unconstrained")
    require("BSCK-10" not in methodology_rules and "BSCK-11" not in methodology_rules,
            "Native PHY BITSLICE clock connectivity regressed")
    require(all(severity not in {"Error", "Critical Warning"}
                for severity, _ in methodology_rules.values()),
            f"Critical methodology violation: {methodology_rules}")

    digest = hashlib.sha256(bitstream.read_bytes()).hexdigest()
    print("UberDDR4 application implementation validation OK")
    print(f"  Timing : WNS {wns} ns, WHS {whs} ns, WPWS {wpws} ns")
    print("  Route  : 0 routing errors")
    drc_summary = (", ".join(f"{rule} x{count}" for rule, (_, count) in drc_rules.items()) +
                   " (VexRiscv DSP pipeline recommendations)"
                   if args.linux else "0 violations")
    print(f"  DRC    : {drc_summary}")
    method_note = "classified; JTAG waiver present" if args.uart_name == "jtag_uart" else "classified"
    print(f"  Method : {', '.join(sorted(methodology_rules))} ({method_note})")
    print(f"  Console: {args.uart_name}")
    print(f"  BIT SHA-256: {digest}")


if __name__ == "__main__":
    main()
