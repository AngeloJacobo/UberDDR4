#!/usr/bin/env python3
"""Wire the AXKU3 SoC: VexRiscv -> LiteX Wishbone -> existing UberDDR4 RTL.

Read _CRG for clocks/reset, _ClassicToPipelined for the bus handshake, and
BaseSoC for the memory map and Verilog instance. `uberddr4.sh build` selects Linux;
the smaller non-Linux configuration remains useful for host-side bus tests.
This file generates hardware; it is not software that runs on the RISC-V CPU.
"""

import argparse
import os

from migen import ClockDomain, ClockSignal, Constant, If, Instance, ResetSignal, Signal
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import LiteXModule
from litex.soc.cores.clock import USPMMCM
from litex.soc.cores.cpu.vexriscv_smp import VexRiscvSMP
from litex.soc.integration import builder as litex_builder
from litex.soc.integration.builder import Builder
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import SoCCore
from litex.soc.interconnect import wishbone

from axku3_platform import Platform


SYS_CLK_FREQ = int(300e6)
REF_CLK_FREQ = int(150e6)
MAIN_RAM_SIZE = 0x40000000
UBERDDR4_DEBUG_BASE = 0xF1000000
BUILD_NAME = "axku3_vexriscv_uberddr4"
WB_DATA_BITS = 256
WB_ADDR_BITS = 26

# data rate (MT/s): (controller/SoC Hz, controller period ps, DDR tCK ps)
# Linux demonstration uses 2400. Other settings are retained for bring-up;
# their presence here does not claim Linux hardware qualification at each rate.
DATA_RATE_CONFIGS = {
    1200: (150_000_000, 6668, 1667),
    1250: (156_250_000, 6400, 1600),
    1600: (200_000_000, 5000, 1250),
    1866: (233_333_333, 4284, 1071),
    2133: (266_666_667, 3752, 938),
    2400: (300_000_000, 3332, 833),
}


def _configure_windows_make_paths(output_dir):
    """Use forward-slash, relative build paths for the Windows GNU make flow."""
    if os.name != "nt":
        return
    build_root = os.path.dirname(os.path.dirname(output_dir))
    package_cwd = os.path.join(output_dir, "software", "package")
    relative_root = os.path.relpath(build_root, package_cwd).replace("\\", "/")
    normalized_root = build_root.replace("\\", "/")
    software_root = os.path.join(output_dir, "software").replace("\\", "/")

    def makefile_escape(value):
        # Every package is built from software/<package>. Use ../include,
        # not a round trip through the build root: LiteX embeds this path in
        # libc object filenames, which otherwise exceed Windows ar's limit.
        value = value.replace("\\", "/").replace(software_root + "/", "../")
        return value.replace(normalized_root, relative_root)

    litex_builder._makefile_escape = makefile_escape


class _CRG(LiteXModule):
    """Run the controller during training; release the CPU only after init_done."""
    def __init__(self, platform, init_done, sys_clk_freq=SYS_CLK_FREQ):
        self.cd_uber = ClockDomain()
        self.cd_sys = ClockDomain()
        self.cd_ref = ClockDomain(reset_less=True)

        clk200 = platform.request("clk200")
        reset_n = platform.request("cpu_reset_n")

        # One MMCM produces phase-related controller and RIU clocks. UG571
        # requires this relationship for the native PHY's RL_DLY_RNK use.
        self.pll = pll = USPMMCM(speedgrade=-2)
        self.comb += pll.reset.eq(~reset_n)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_uber, sys_clk_freq, with_reset=False)
        pll.create_clkout(self.cd_ref, REF_CLK_FREQ, with_reset=False)

        # The CPU and UberDDR4 share the controller clock, but have independent
        # reset domains. UberDDR4 trains first; only a sticky init_done releases
        # the CPU, interconnect, BIOS, and console.
        self.comb += self.cd_sys.clk.eq(self.cd_uber.clk)
        self.specials += [
            AsyncResetSynchronizer(self.cd_uber, ~pll.locked | ~reset_n),
            AsyncResetSynchronizer(self.cd_sys, ResetSignal("uber") | ~init_done),
        ]


class _ClassicToPipelined(LiteXModule):
    """Issue each LiteX classic request exactly once on UberDDR4's B4 port."""
    def __init__(self, classic):
        self.stb = Signal()
        self.stall = Signal()
        self.ack = Signal()
        accepted = Signal()

        self.comb += [
            self.stb.eq(classic.cyc & classic.stb & ~accepted),
            classic.ack.eq(self.ack),
            classic.err.eq(0),
        ]
        self.sync += [
            If(~classic.cyc,
                accepted.eq(0),
            ).Elif(self.ack,
                accepted.eq(0),
            ).Elif(self.stb & ~self.stall,
                accepted.eq(1),
            )
        ]


class BaseSoC(SoCCore):
    """Expose 1 GiB of DDR4 at 0x40000000 and existing debug CSRs at 0xf1000000."""
    def __init__(
        self,
        uberddr4_rtl_dir,
        uart_name="jtag_uart",
        data_rate=2400,
        cpu_type="vexriscv",
        cpu_variant="lite",
        uart_baudrate=115200,
        linux=False,
    ):
        if data_rate not in DATA_RATE_CONFIGS:
            raise ValueError(f"Unsupported UberDDR4 data rate: {data_rate}")
        sys_clk_freq, controller_clk_period, ddr4_clk_period = DATA_RATE_CONFIGS[data_rate]
        platform = Platform()
        init_done = Signal()
        init_failed = Signal()
        self.crg = _CRG(platform, init_done, sys_clk_freq=sys_clk_freq)
        self.data_rate = data_rate
        self.sys_clk_freq = sys_clk_freq
        self.controller_clk_period = controller_clk_period
        self.ddr4_clk_period = ddr4_clk_period

        if linux:
            if cpu_type != "vexriscv_smp" or cpu_variant != "linux":
                raise ValueError("Linux mode requires vexriscv_smp/linux")
            # Select a checked-in, pre-generated one-core configuration. The
            # Wishbone memory port is required because UberDDR4 replaces
            # LiteDRAM and is connected as the SoC's external RAM slave.
            VexRiscvSMP.cpu_count = 1
            VexRiscvSMP.icache_width = 32
            VexRiscvSMP.dcache_width = 32
            VexRiscvSMP.icache_size = 4096
            VexRiscvSMP.dcache_size = 4096
            VexRiscvSMP.icache_ways = 1
            VexRiscvSMP.dcache_ways = 1
            VexRiscvSMP.itlb_size = 4
            VexRiscvSMP.dtlb_size = 4
            VexRiscvSMP.coherent_dma = False
            VexRiscvSMP.out_of_order_decoder = True
            VexRiscvSMP.wishbone_memory = True
            VexRiscvSMP.wishbone_force_32b = False
            VexRiscvSMP.hardware_breakpoints = 0
            VexRiscvSMP.privileged_debug = False
            VexRiscvSMP.with_fpu = False
            VexRiscvSMP.with_rvc = False

        # The identifier ROM is the one build label Linux can read back over the
        # CSR bus, so record the configured rate there. Without it a running
        # system has no way to report which DATA_RATE_CONFIGS entry it was
        # built from: UberDDR4's CONFIG register carries byte lanes and BIST
        # mode only, not the clock periods.
        SoCCore.__init__(
            self,
            platform,
            sys_clk_freq,
            ident=(f"LiteX VexRiscv + UberDDR4 on ALINX AXKU3 "
                   f"DDR4-{data_rate} {sys_clk_freq // 1_000_000}MHz "
                   f"tCK{ddr4_clk_period}ps"),
            cpu_type=cpu_type,
            cpu_variant=cpu_variant,
            integrated_rom_size=0x10000,
            integrated_sram_size=0x4000,
            uart_name=uart_name,
            uart_baudrate=uart_baudrate,
            with_ctrl=False,
        )

        # Keep the useful scratch and bus-error CSRs, but omit LiteX's optional
        # one-cycle full-SoC reset field. At 300 MHz that 3.33 ns pulse cannot
        # be captured reliably by the MMCM's 200 MHz reset-delay chain. Board
        # reset and JTAG configuration remain the deterministic reset sources.
        self.add_controller(name="ctrl", with_reset=False)

        if uart_name == "jtag_uart":
            platform.toolchain.pre_placement_commands += [
                "create_clock -name jtag_tck -period 20.000 "
                "[get_nets -of_objects [get_pins BSCANE2/TCK]]",
                "set_clock_groups -asynchronous -group [get_clocks jtag_tck] "
                "-group [get_clocks -filter {{NAME != jtag_tck}}]",
                "set_max_delay 10.000 -datapath_only "
                "-from [all_registers -clock [get_clocks jtag_tck]] "
                "-to [get_pins BSCANE2/TDO]",
                "create_waiver -type METHODOLOGY -id TIMING-2 "
                "-description {{BSCANE2 TCK is the external JTAG clock root exposed only "
                "as an internal primitive pin}} -objects [get_pins BSCANE2/TCK]",
            ]

        source_names = (
            "ddr4_top.v",
            "ddr4_controller.v",
            "ddr4_prober.v",
            "ddr4_phy.v",
            os.path.join("phy", "ddr4_phy_native.v"),
            os.path.join("phy", "ddr4_phy_native_adapter.v"),
            os.path.join("phy", "ddr4_phy_native_byte.v"),
            os.path.join("phy", "ddr4_phy_native_reset.v"),
        )
        for source_name in source_names:
            source_path = os.path.join(uberddr4_rtl_dir, source_name)
            if not os.path.isfile(source_path):
                raise FileNotFoundError(source_path)
            platform.add_source(source_path)

        # LiteX converts the CPU's 32-bit words to UberDDR4's 256-bit words.
        # strip_origin makes controller address zero correspond to 0x40000000.
        main_wb = wishbone.Interface(
            data_width=WB_DATA_BITS,
            adr_width=WB_ADDR_BITS,
            addressing="word",
            bursting=True,
        )
        self.bus.add_slave(
            name="main_ram",
            slave=main_wb,
            region=SoCRegion(
                origin=self.mem_map["main_ram"], size=MAIN_RAM_SIZE, cached=True
            ),
            strip_origin=True,
        )

        debug_wb = wishbone.Interface(
            data_width=32, adr_width=4, addressing="word", bursting=False
        )
        self.bus.add_slave(
            name="uberddr4_debug",
            slave=debug_wb,
            region=SoCRegion(
                origin=UBERDDR4_DEBUG_BASE, size=0x40, cached=False
            ),
            strip_origin=True,
        )

        ddr4 = platform.request("ddr4")
        self.main_bridge = main_bridge = _ClassicToPipelined(main_wb)
        self.debug_bridge = debug_bridge = _ClassicToPipelined(debug_wb)
        # Migen prefixes: p_ = parameter; i_/o_/io_ = port direction. The second
        # prefix belongs to the actual Verilog name (e.g. i_i_wb_addr -> i_wb_addr).
        self.specials += Instance(
            "ddr4_top",
            p_CONTROLLER_CLK_PERIOD=controller_clk_period,
            p_DDR4_CLK_PERIOD=ddr4_clk_period,
            p_DEVICE_WIDTH=16,
            p_ROW_BITS=16,
            p_COL_BITS=10,
            # Preserve the width of this untyped Verilog parameter. An ordinary
            # Python 4 emits 3'd4, too narrow for the existing BYTE_LANES[3:0]
            # CSR readback. Fix the instantiation, not the controller RTL.
            p_BYTE_LANES=Constant(4, 32),
            p_DENSITY=8,
            p_PHY_IMPL=1,
            # Board-specific native BITSLICE routing, not CPU memory addresses.
            # These maps must agree with axku3_platform.py's physical DDR pins.
            p_PHY_ACMD_NIBBLE_COUNT=6,
            p_PHY_ACMD_PIN_MAP=int(
                "ffffffffffff1009130815240522022e"
                "230419281a202d1b2c03182b2921252a", 16
            ),
            p_PHY_DQ_PIN_MAP=int("253bdac42ba53dc45dbca3424ca235bd", 16),
            p_PHY_PLL_COUNT=2,
            p_PHY_ACMD_PLL_MAP=0,
            p_PHY_BYTE_PLL_MAP=0x249,
            # This example uses calibration followed by host-driven BIOS/Linux
            # memory checks. It does not enable or modify the controller BIST.
            p_BIST_MODE=0,
            p_BIST_DM_TEST=0,
            p_BIST_REREAD_DIAG=0,
            p_DEBUG_CSR_ENABLE=1,
            i_i_controller_clk=ClockSignal("uber"),
            # Native PHY generates its fast DDR clock internally; this legacy
            # component-PHY clock input is unused when PHY_IMPL=1.
            i_i_ddr4_clk=ClockSignal("uber"),
            i_i_ref_clk=ClockSignal("ref"),
            i_i_rst_n=~ResetSignal("uber"),
            i_i_wb_cyc=main_wb.cyc,
            i_i_wb_stb=main_bridge.stb,
            i_i_wb_we=main_wb.we,
            i_i_wb_addr=main_wb.adr,
            i_i_wb_data=main_wb.dat_w,
            i_i_wb_sel=main_wb.sel,
            o_o_wb_stall=main_bridge.stall,
            o_o_wb_ack=main_bridge.ack,
            o_o_wb_data=main_wb.dat_r,
            i_i_wb_dbg_cyc=debug_wb.cyc,
            i_i_wb_dbg_stb=debug_bridge.stb,
            i_i_wb_dbg_we=debug_wb.we,
            i_i_wb_dbg_addr=debug_wb.adr,
            i_i_wb_dbg_data=debug_wb.dat_w,
            i_i_wb_dbg_sel=debug_wb.sel,
            o_o_wb_dbg_stall=debug_bridge.stall,
            o_o_wb_dbg_ack=debug_bridge.ack,
            o_o_wb_dbg_data=debug_wb.dat_r,
            o_o_ddr4_ck_p=ddr4.ck_p,
            o_o_ddr4_ck_n=ddr4.ck_n,
            o_o_ddr4_reset_n=ddr4.reset_n,
            o_o_ddr4_cke=ddr4.cke,
            o_o_ddr4_cs_n=ddr4.cs_n,
            o_o_ddr4_act_n=ddr4.act_n,
            o_o_ddr4_addr=ddr4.addr,
            o_o_ddr4_ba=ddr4.ba,
            o_o_ddr4_bg=ddr4.bg,
            o_o_ddr4_odt=ddr4.odt,
            o_o_ddr4_dm_n=ddr4.dm_n,
            io_io_ddr4_dq=ddr4.dq,
            io_io_ddr4_dqs_p=ddr4.dqs_p,
            io_io_ddr4_dqs_n=ddr4.dqs_n,
            o_o_init_done=init_done,
            o_o_init_failed=init_failed,
        )

        self.uberddr4_main_wb = main_wb
        self.uberddr4_debug_wb = debug_wb
        self.uberddr4_init_done = init_done
        self.uberddr4_init_failed = init_failed

        self.comb += [
            platform.request("fan_pwm").eq(0),
            platform.request("user_led", 0).eq(init_failed),
            platform.request("user_led", 1).eq(init_done),
            platform.request("user_led", 2).eq(init_done),
            platform.request("user_led", 3).eq(init_done),
        ]


def main():
    parser = argparse.ArgumentParser(
        description="VexRiscv + UberDDR4 native PHY target for AXKU3"
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--uberddr4-rtl-dir", required=True)
    parser.add_argument("--build", action="store_true")
    parser.add_argument(
        "--uart-name", choices=("serial", "jtag_uart"), default="jtag_uart"
    )
    parser.add_argument(
        "--data-rate",
        type=int,
        choices=tuple(DATA_RATE_CONFIGS),
        default=2400,
        help="DDR data rate in MT/s; controller/SoC clock is one eighth",
    )
    parser.add_argument("--build-name", default=BUILD_NAME)
    parser.add_argument("--linux", action="store_true")
    parser.add_argument("--uart-baudrate", type=int, default=115200)
    args = parser.parse_args()

    output_dir = os.path.abspath(args.output_dir)
    _configure_windows_make_paths(output_dir)
    soc = BaseSoC(
        uberddr4_rtl_dir=os.path.abspath(args.uberddr4_rtl_dir),
        uart_name=args.uart_name,
        data_rate=args.data_rate,
        cpu_type="vexriscv_smp" if args.linux else "vexriscv",
        cpu_variant="linux" if args.linux else "lite",
        uart_baudrate=args.uart_baudrate,
        linux=args.linux,
    )
    builder = Builder(
        soc,
        output_dir=output_dir,
        csr_csv=os.path.join(output_dir, "csr.csv"),
        csr_json=os.path.join(output_dir, "csr.json") if args.linux else None,
        bios_console="lite" if args.linux else "full",
    )
    builder.build(
        run=args.build,
        build_name=args.build_name,
        vivado_post_route_phys_opt_directive="ExploreWithAggressiveHoldFix",
        vivado_report_level="signoff",
        vivado_max_threads=8,
    )


if __name__ == "__main__":
    main()
