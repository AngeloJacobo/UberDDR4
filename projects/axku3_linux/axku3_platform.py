#!/usr/bin/env python3
"""Physical AXKU3 board definition: FPGA part, connector pins and I/O standards.

List order maps directly to Verilog vector bits (first pin is bit zero).
The SoC requests these named resources; the companion XDC adds electrical
and timing constraints. Keep this pin mapping separate from SoC logic.
"""

import os

from litex.build.generic_platform import IOStandard, Misc, Pins, Subsignal
from litex.build.xilinx import XilinxUSPPlatform
from litex.build.xilinx.programmer import VivadoProgrammer


_io = [
    ("clk200", 0,
        Subsignal("p", Pins("K22"), IOStandard("DIFF_SSTL12")),
        Subsignal("n", Pins("K23"), IOStandard("DIFF_SSTL12")),
    ),
    ("cpu_reset_n", 0, Pins("J14"), IOStandard("LVCMOS33")),
    ("serial", 0,
        Subsignal("rx", Pins("AE15")),
        Subsignal("tx", Pins("AD15")),
        IOStandard("LVCMOS33"),
    ),
    ("user_led", 0, Pins("J12"), IOStandard("LVCMOS33")),
    ("user_led", 1, Pins("H14"), IOStandard("LVCMOS33")),
    ("user_led", 2, Pins("F13"), IOStandard("LVCMOS33")),
    ("user_led", 3, Pins("H12"), IOStandard("LVCMOS33")),
    ("fan_pwm", 0, Pins("Y16"), IOStandard("LVCMOS33")),

    # Two MT40A512M16LY-062E devices share CA and form a 32-bit data bus.
    # UberDDR4 exposes A14/A15/A16 directly rather than separate command pads.
    ("ddr4", 0,
        Subsignal("addr", Pins(
            "D26 D25 E26 C24 C26 F24 M26 B25 G26 B26 E25 H26 D23 F25",
            "K25 E23 F22"), IOStandard("SSTL12_DCI")),
        Subsignal("ba",      Pins("M25 F23"), IOStandard("SSTL12_DCI")),
        Subsignal("bg",      Pins("K26"),     IOStandard("SSTL12_DCI")),
        Subsignal("cs_n",    Pins("D24"),     IOStandard("SSTL12_DCI")),
        Subsignal("act_n",   Pins("J26"),     IOStandard("SSTL12_DCI")),
        Subsignal("cke",     Pins("L24"),     IOStandard("SSTL12_DCI")),
        Subsignal("odt",     Pins("H24"),     IOStandard("SSTL12_DCI")),
        Subsignal("reset_n", Pins("L25"),     IOStandard("LVCMOS12")),
        Subsignal("dm_n", Pins("G15 C18 H18 A22"),
            IOStandard("POD12_DCI"), Misc("PRE_EMPHASIS=RDRV_240")),
        Subsignal("dq", Pins(
            "C16 G16 D15 G17 H17 H16 D16 E15",
            "B19 C17 B20 B15 A19 A15 A20 B17",
            "G20 D19 D20 F19 G21 E18 D18 F18",
            "C23 C22 A24 B22 A25 D21 B24 E21"),
            IOStandard("POD12_DCI"),
            Misc("PRE_EMPHASIS=RDRV_240"), Misc("EQUALIZATION=EQ_LEVEL2")),
        Subsignal("dqs_p", Pins("E16 A17 F20 C21"),
            IOStandard("DIFF_POD12_DCI"),
            Misc("PRE_EMPHASIS=RDRV_240"), Misc("EQUALIZATION=EQ_LEVEL2")),
        Subsignal("dqs_n", Pins("E17 A18 E20 B21"),
            IOStandard("DIFF_POD12_DCI"),
            Misc("PRE_EMPHASIS=RDRV_240"), Misc("EQUALIZATION=EQ_LEVEL2")),
        Subsignal("ck_p", Pins("G24"), IOStandard("DIFF_SSTL12_DCI")),
        Subsignal("ck_n", Pins("G25"), IOStandard("DIFF_SSTL12_DCI")),
        Misc("SLEW=FAST"),
    ),
]


class Platform(XilinxUSPPlatform):
    """Let LiteX emit Vivado sources/constraints for this specific board."""
    default_clk_name = "clk200"
    default_clk_period = 1e9 / 200e6

    def __init__(self, toolchain="vivado"):
        XilinxUSPPlatform.__init__(
            self, "xcku3p-ffvb676-2-i", _io, toolchain=toolchain
        )
        self.add_source(os.path.join(
            os.path.dirname(__file__), "constraints", "axku3_uberddr4.xdc"
        ))

    def create_programmer(self):
        return VivadoProgrammer()

    def do_finalize(self, fragment):
        XilinxUSPPlatform.do_finalize(self, fragment)
        self.add_period_constraint(
            self.lookup_request("clk200", loose=True), self.default_clk_period
        )
