////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_sim_top.sv
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  Top-level simulation testbench for the UberDDR4 controller.
//
//  Structure:
//    1. Clock generation  -  DDR4 CK (834ps), controller (CK/4), ref (300MHz)
//    2. Reset             -  active-low, held 100ns then released
//    3. DUT               -  ddr4_top with MICRON_SIM=1 (shortened init timers)
//    4. Micron models     -  two x8 DDR4 behavioral models (one per byte lane)
//    5. Fly-by delay      -  lane 1 CK/CMD delayed by SIM_FLY_BY_DELAY ps
//    6. Debug monitors    -  ROM phase, DFI/DDR4 command decoders, PHY training
//    7. WB test stimulus  -  phases A..Q exercising all scheduler paths
//    8. Command monitor   -  DFI-level ACT/WR/RD/PRE counter & logger
//    9. Timeout & summary -  hard 500us watchdog, final PASS/FAIL report
//
//  The fly-by delay models real PCB daisy-chain routing skew: the DDR4
//  clock, command, and address nets reach chip 1 later than chip 0. The
//  PHY's write-leveling and gate-training FSMs must compensate for this.
//  DQ/DQS are point-to-point so they stay zero-delay.
//
//  Wave dumping: VCD when `VCD_DUMP is defined. Start/end time are set
//  near the top of the simulation module.
//
// Engineer: Angelo C. Jacobo
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2025  Angelo Jacobo
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////

`timescale 1ps / 1ps

`ifdef XILINX_SIMULATOR
module short(in1, in1);
inout wire in1;
endmodule
`endif

`default_nettype none

// Compile both DFI-compatible PHYs, then select one with ddr4_top.PHY_IMPL.
// run_xsim.sh defines TB_USE_NATIVE_PHY only to select the testbench's top
// parameter; this makes the simulation use the same public selection API as
// a hardware design.
`ifndef UBERDDR4_PHY_SOURCES_PRECOMPILED
`include "../rtl/ddr4_phy.v"
`include "../rtl/phy/ddr4_phy_native_reset.v"
`include "../rtl/phy/ddr4_phy_native_byte.v"
`include "../rtl/phy/ddr4_phy_native.v"
`include "../rtl/phy/ddr4_phy_native_adapter.v"
`endif

module ddr4_sim_top;

    // This is intentionally the only PHY choice the testbench makes.  ddr4_top
    // owns the implementation-specific DFI timing compensation.
`ifdef TB_USE_NATIVE_PHY
    localparam [0:0] TB_PHY_IMPL = 1'b1;
    `define TB_PHY_HIER u_dut.gen_native_phy.u_phy
`else
    localparam [0:0] TB_PHY_IMPL = 1'b0;
    `define TB_PHY_HIER u_dut.gen_component_phy.u_phy
`endif

    // ===================================================================
    // Parameters  -  match DUT defaults but use integer-exact clock periods
    // DDR4-2400: tCK=834ps (417ps half, exact integer), 4:1 ratio
    // ===================================================================
`ifdef SIM_DDR4_CLK_PERIOD
    localparam DDR4_CLK_PERIOD = `SIM_DDR4_CLK_PERIOD;
`else
    localparam DDR4_CLK_PERIOD = 834;
`endif
    localparam CTRL_CLK_PERIOD = DDR4_CLK_PERIOD * 4;

`ifdef SIM_DEVICE_WIDTH
    localparam DEVICE_WIDTH = `SIM_DEVICE_WIDTH;
`else
    localparam DEVICE_WIDTH = 8;
`endif

`ifdef SIM_BYTE_LANES
    localparam BYTE_LANES = `SIM_BYTE_LANES;
`else
    localparam BYTE_LANES = 2;
`endif

    localparam DQ_BITS     = 8;
    localparam BG_BITS     = (DEVICE_WIDTH == 16) ? 1 : 2;
`ifdef SIM_ROW_BITS
    localparam ROW_BITS    = `SIM_ROW_BITS;
`else
    localparam ROW_BITS    = 16;
`endif
    localparam COL_BITS    = 10;
    localparam BA_BITS     = 2;

// The focused native-PHY TX diagnostic owns the Wishbone port itself, so
// disable the automatic prober BIST.  Keep this a compile-time selection so
// the normal regression configuration is completely unchanged.
`ifdef SIM_NATIVE_TX_DEBUG
    localparam TB_BIST_MODE = 0;
`elsif SIM_BIST_MODE
    localparam TB_BIST_MODE = `SIM_BIST_MODE;
`else
    localparam TB_BIST_MODE = 2;
`endif

`ifdef SIM_BIST_DM_TEST
    localparam TB_BIST_DM_TEST = `SIM_BIST_DM_TEST;
`else
    localparam TB_BIST_DM_TEST = 0;
`endif

`ifdef SIM_TB_DEPTH_BITS
    localparam TB_DEPTH_BITS = `SIM_TB_DEPTH_BITS;
`else
    localparam TB_DEPTH_BITS = 5;
`endif
    localparam TB_DEPTH = 1 << TB_DEPTH_BITS;
    localparam SERDES_RATIO   = 4;
    localparam WB_DATA_BITS   = DQ_BITS * BYTE_LANES * 2 * SERDES_RATIO;
    localparam WB_SEL_BITS    = WB_DATA_BITS / 8;
    localparam COL_LOW        = $clog2(SERDES_RATIO * 2);
    localparam WB_ADDR_BITS   = ROW_BITS + BG_BITS + BA_BITS + COL_BITS - COL_LOW;
    localparam NUM_DEVICES    = (DEVICE_WIDTH == 16) ? (BYTE_LANES / 2) :
                                (DEVICE_WIDTH == 4)  ? (BYTE_LANES * 2) :
                                                        BYTE_LANES;

    // ===================================================================
    // Regression-overridable parameters via +define+ (source stays untouched)
    //
    // FLY_BY: CK fly-by delay in ps applied to iDDR4_1 (lane 1).
    //   Models real PCB daisy-chain routing: FPGA -> chip0 -> chip1.
    //   Realistic range: 50-400ps for a 2-chip DDR4-2400 board.
    //   The PHY write-leveling FSM must discover this skew on its own
    //   and compensate with per-lane DQS tap offsets.
    //
    // TB_ADDR_MAPPING: 0 = sequential, 1 = BG-interleaved (default).
    //   Mapping 1 places BG bits at the bottom so consecutive addresses
    //   rotate across bank groups, maximizing CAS-to-CAS throughput.
    // ===================================================================
`ifdef SIM_FLY_BY_DELAY
    localparam FLY_BY = `SIM_FLY_BY_DELAY;
`else
    localparam FLY_BY = 0;
`endif


`ifdef SIM_ADDR_MAPPING
    localparam TB_ADDR_MAPPING = `SIM_ADDR_MAPPING;
`else
    localparam TB_ADDR_MAPPING = 1;
`endif

    import arch_package::*;

`ifdef SIM_DENSITY_4G
    localparam TB_DENSITY = _4G;
    localparam TB_DENSITY_GB = 4;
`else
    localparam TB_DENSITY = _8G;
    localparam TB_DENSITY_GB = 8;
`endif

    // ===================================================================
    // VCD Dump Control
    //
    // Enable with:
    //   +define+VCD_DUMP
    //
    // Optional compile-time overrides:
    //   +define+SIM_VCD_START_TIME=<time in ps>
    //   +define+SIM_VCD_END_TIME=<time in ps>
    //
    // Example: dump only 20 us through 30 us
    //   +define+VCD_DUMP +define+SIM_VCD_START_TIME=20_000_000 +define+SIM_VCD_END_TIME=30_000_000
    // ===================================================================
`ifdef VCD_DUMP
    localparam bit  TB_VCD_ENABLE = 1'b1;
`else
    localparam bit  TB_VCD_ENABLE = 1'b0;
`endif

// The focused native-PHY write diagnostic is intentionally short.  Dump its
// complete lifetime, including reset and calibration, so a waveform contains
// every event that can affect the first user write.
`ifdef SIM_VCD_START_TIME
    localparam time TB_VCD_START_TIME = `SIM_VCD_START_TIME;
`elsif SIM_NATIVE_TX_DEBUG_REDUCED_VCD
    // Fast native-PHY diagnostic trace: isolate the four directed reads and
    // the native FIFO-to-DFI return path.  Full-lifetime capture remains the
    // default for SIM_NATIVE_TX_DEBUG.
    localparam time TB_VCD_START_TIME = 'd6100000;
`elsif SIM_NATIVE_TX_DEBUG
    localparam time TB_VCD_START_TIME = 'd0;
`else
    localparam time TB_VCD_START_TIME = 'd0;
`endif

`ifdef SIM_VCD_END_TIME
    localparam time TB_VCD_END_TIME = `SIM_VCD_END_TIME;
`elsif SIM_NATIVE_TX_DEBUG_REDUCED_VCD
    localparam time TB_VCD_END_TIME = 'd6300000;
`elsif SIM_NATIVE_TX_DEBUG
    localparam time TB_VCD_END_TIME = 'd50000000;
`else
    localparam time TB_VCD_END_TIME = 'd93260389;
`endif

    // ===================================================================
    // Clock Generation
    // ddr4_clk   : 834 ps period (toggle every 417 ps)
    // controller : ddr4_clk / 4 = 3336 ps period (phase-aligned)
    // ref_clk    : component PHY = 300 MHz delay reference
    //              native PHY    = controller_clk / 2 RIU clock
    // ===================================================================
    reg ddr4_clk;
    initial ddr4_clk = 1'b0;
    always #(DDR4_CLK_PERIOD / 2) ddr4_clk = ~ddr4_clk;

    reg [1:0] clk_div;
    initial clk_div = 2'b00;
    always @(posedge ddr4_clk) clk_div <= clk_div + 1;
    wire controller_clk = clk_div[1];

    reg ref_clk;
    initial begin
        ref_clk = 1'b0;
        if (TB_PHY_IMPL) begin
            // The native RIU clock is controller_clk/2 from the same MMCM and
            // uses the same zero-degree phase shift.  Start its first rising
            // edge with a controller rising edge, then toggle once per full
            // controller period.
            #(CTRL_CLK_PERIOD / 2);
            forever #(CTRL_CLK_PERIOD) ref_clk = ~ref_clk;
        end else begin
            forever #1667 ref_clk = ~ref_clk;
        end
    end

    // ===================================================================
    // Reset  -  held low for 100 ns, released on controller_clk posedge
    // ===================================================================
    reg rst_n;
    initial begin
        rst_n = 1'b0;
        #100_000;
        @(posedge controller_clk);
        rst_n = 1'b1;
    end

    // ===================================================================
    // DDR4 wires between DUT and model wrapper
    // ===================================================================
    wire ddr4_ck_p, ddr4_ck_n;
    wire ddr4_reset_n, ddr4_cke, ddr4_cs_n, ddr4_act_n;
    wire [16:0] ddr4_addr;
    wire [BA_BITS-1:0] ddr4_ba;
    wire [BG_BITS-1:0] ddr4_bg;
    wire ddr4_odt;
    wire [BYTE_LANES-1:0] ddr4_dm_n;
    wire [DQ_BITS*BYTE_LANES-1:0] ddr4_dq;
    wire [BYTE_LANES-1:0] ddr4_dqs_p, ddr4_dqs_n;

    // ===================================================================
    // Wishbone bus signals
    //
    // Initialized to 'z at time 0, then driven to idle (0) after 1ps.
    // Starting at 'z makes it easy to spot the first real bus activity
    // in waveforms without hunting through startup noise.
    // ===================================================================
    reg                      wb_cyc;
    reg                      wb_stb;
    reg                      wb_we;
    reg [WB_ADDR_BITS-1:0]   wb_addr;
    reg [WB_DATA_BITS-1:0]   wb_data;
    reg [WB_SEL_BITS-1:0]    wb_sel;
    wire                     wb_stall;
    wire                     wb_ack;
    wire [WB_DATA_BITS-1:0]  wb_rdata;

    // Debug CSR port (separate WB B4 â€�? always accessible)
    reg        wb_dbg_cyc, wb_dbg_stb, wb_dbg_we;
    reg [3:0]  wb_dbg_addr;
    reg [31:0] wb_dbg_data;
    reg [3:0]  wb_dbg_sel;
    wire       wb_dbg_stall, wb_dbg_ack;
    wire [31:0] wb_dbg_rdata;
    wire                     init_done;
    wire                     init_failed;

    initial begin
        wb_cyc     = 'z;
        wb_stb     = 'z;
        wb_we      = 'z;
        wb_addr    = 'z;
        wb_data    = 'z;
        wb_sel     = 'z;
        wb_dbg_cyc    = 'z;
        wb_dbg_stb    = 'z;
        wb_dbg_we     = 'z;
        wb_dbg_addr   = 'z;
        wb_dbg_data   = 'z;
        wb_dbg_sel    = 'z;
        #1;
        wb_cyc     = 1'b0;
        wb_stb     = 1'b0;
        wb_we      = 1'b0;
        wb_addr    = {WB_ADDR_BITS{1'b0}};
        wb_data    = {WB_DATA_BITS{1'b0}};
        wb_sel     = {WB_SEL_BITS{1'b0}};
        wb_dbg_cyc    = 1'b0;
        wb_dbg_stb    = 1'b0;
        wb_dbg_we     = 1'b0;
        wb_dbg_addr   = 4'd0;
        wb_dbg_data   = 32'd0;
        wb_dbg_sel    = 4'hF;
    end

    // ===================================================================
    // DUT  -  ddr4_top with MICRON_SIM=1 (shortened init delays)
    // ADDR_MAPPING driven by regression-overridable param
    // ===================================================================
    ddr4_top #(
        .CONTROLLER_CLK_PERIOD (CTRL_CLK_PERIOD),
        .DDR4_CLK_PERIOD       (DDR4_CLK_PERIOD),
        .DEVICE_WIDTH          (DEVICE_WIDTH),
        .ROW_BITS              (ROW_BITS),
        .COL_BITS              (COL_BITS),
        .BYTE_LANES            (BYTE_LANES),
        .DENSITY               (TB_DENSITY_GB),
        .PHY_IMPL               (TB_PHY_IMPL),
        .MICRON_SIM            (1),
        .ADDR_MAPPING          (TB_ADDR_MAPPING),
        .BIST_MODE             (TB_BIST_MODE),
        .BIST_DM_TEST          (TB_BIST_DM_TEST),
        .DEBUG_CSR_ENABLE      (1)
    ) u_dut (
        .i_controller_clk (controller_clk),
        .i_ddr4_clk       (ddr4_clk),
        .i_ref_clk        (ref_clk),
        .i_rst_n          (rst_n),
        .i_wb_cyc          (wb_cyc),
        .i_wb_stb          (wb_stb),
        .i_wb_we           (wb_we),
        .i_wb_addr         (wb_addr),
        .i_wb_data         (wb_data),
        .i_wb_sel          (wb_sel),
        .o_wb_stall        (wb_stall),
        .o_wb_ack          (wb_ack),
        .o_wb_data         (wb_rdata),
        .i_wb_dbg_cyc            (wb_dbg_cyc),
        .i_wb_dbg_stb            (wb_dbg_stb),
        .i_wb_dbg_we             (wb_dbg_we),
        .i_wb_dbg_addr           (wb_dbg_addr),
        .i_wb_dbg_data           (wb_dbg_data),
        .i_wb_dbg_sel            (wb_dbg_sel),
        .o_wb_dbg_stall          (wb_dbg_stall),
        .o_wb_dbg_ack            (wb_dbg_ack),
        .o_wb_dbg_data           (wb_dbg_rdata),
        .o_ddr4_ck_p       (ddr4_ck_p),
        .o_ddr4_ck_n       (ddr4_ck_n),
        .o_ddr4_reset_n    (ddr4_reset_n),
        .o_ddr4_cke        (ddr4_cke),
        .o_ddr4_cs_n       (ddr4_cs_n),
        .o_ddr4_act_n      (ddr4_act_n),
        .o_ddr4_addr       (ddr4_addr),
        .o_ddr4_ba         (ddr4_ba),
        .o_ddr4_bg         (ddr4_bg),
        .o_ddr4_odt        (ddr4_odt),
        .o_ddr4_dm_n       (ddr4_dm_n),
        .io_ddr4_dq        (ddr4_dq),
        .io_ddr4_dqs_p     (ddr4_dqs_p),
        .io_ddr4_dqs_n     (ddr4_dqs_n),
        .o_init_done       (init_done),
        .o_init_failed     (init_failed)
    );

`ifdef SIM_NATIVE_TX_DEBUG_FAST_WL
    // XSim's native BITSLICE model returns an all-high WL response in this
    // configuration.  The real FSM therefore takes the defined no-edge
    // fallback after scanning every ODELAY tap, which is prohibitively slow
    // when dumping every signal from time zero.  For this diagnostic only,
    // route each lane through its WL_APPLY state.  The lanes retain their
    // pre-WL BISC DQS tap and zero DQ offset, the same safe values selected
    // by the all-high fallback.  This changes neither synthesizable RTL nor
    // the post-training TX datapath.
    initial begin : native_tx_debug_fast_wl
        wait (`TB_PHY_HIER.u_native.phy_state == 4'd8 &&
              `TB_PHY_HIER.u_native.train_lane == 0);
        force `TB_PHY_HIER.u_native.phy_state = 4'd12;
        repeat (6) @(posedge controller_clk);
        release `TB_PHY_HIER.u_native.phy_state;

        wait (`TB_PHY_HIER.u_native.phy_state == 4'd8 &&
              `TB_PHY_HIER.u_native.train_lane == 1);
        force `TB_PHY_HIER.u_native.phy_state = 4'd12;
        repeat (6) @(posedge controller_clk);
        release `TB_PHY_HIER.u_native.phy_state;
    end
`endif

    // ===================================================================
    // Internal Monitoring Wires (hierarchical refs for debug display)
    //
    // These reach into the DUT hierarchy so the TB can track calibration
    // progress and BIST status without adding extra DUT ports. Fine for
    // simulation; obviously not synthesizable.
    // ===================================================================
    wire calib_complete_int = u_dut.calib_complete;
    wire bist_busy_int      = u_dut.prober_bist_busy;
    wire bist_fail_int      = u_dut.u_prober.bist_fail_sticky;
    wire [31:0] bist_correct_int = u_dut.u_prober.correct_count;
    wire [31:0] bist_error_int   = u_dut.u_prober.error_count;

    // ===================================================================
    // Micron DDR4 Models  -  direct instantiation (no wrapper module)
    //
    // BYTE_LANES independent devices (x8 or x16), one per byte lane.
    // The Micron model ships with Vivado and is not redistributable,
    // so the user must run setup_micron_model.sh to create symlinks
    // before compiling.
    //
    // iDDR4[0] = byte lane 0 (near end, zero fly-by)
    // iDDR4[1] = byte lane 1 (far end, CK/CMD delayed by FLY_BY ps)
    //            (only present when BYTE_LANES == 2)
    //
    // Bidirectional DQ/DQS/DM wiring uses the `short` module for xsim
    // (Xilinx's own workaround  -  see MIG sim_tb_top.sv) and `tran`
    // gate primitives for all other simulators. Both approaches create
    // a transparent bidirectional connection without drive-strength issues.
    // ===================================================================
    wire model_en;
    assign model_en = 1'b1;

    genvar gl, gi, gj;

    // -----------------------------------------------------------------
    // x8: 1 DDR4_if + 1 model per byte lane (current default)
    // -----------------------------------------------------------------
    generate if (DEVICE_WIDTH == 8) begin : gen_x8
        DDR4_if #(.CONFIGURED_DQ_BITS(8)) iDDR4[BYTE_LANES-1:0]();

        for (gl = 0; gl < BYTE_LANES; gl = gl + 1) begin : gen_cmd
            if (gl == 0) begin : near
                assign iDDR4[gl].CK        = {ddr4_ck_p, ddr4_ck_n};
                assign iDDR4[gl].RESET_n   = ddr4_reset_n;
                assign iDDR4[gl].CKE       = ddr4_cke;
                assign iDDR4[gl].CS_n      = ddr4_cs_n;
                assign iDDR4[gl].ACT_n     = ddr4_act_n;
                assign iDDR4[gl].RAS_n_A16 = ddr4_addr[16];
                assign iDDR4[gl].CAS_n_A15 = ddr4_addr[15];
                assign iDDR4[gl].WE_n_A14  = ddr4_addr[14];
                assign iDDR4[gl].ADDR      = ddr4_addr[13:0];
                assign iDDR4[gl].BA        = ddr4_ba;
                assign iDDR4[gl].BG        = ddr4_bg;
                assign iDDR4[gl].ODT       = ddr4_odt;
            end else begin : far
                assign #(FLY_BY) iDDR4[gl].CK        = {ddr4_ck_p, ddr4_ck_n};
                assign #(FLY_BY) iDDR4[gl].RESET_n   = ddr4_reset_n;
                assign #(FLY_BY) iDDR4[gl].CKE       = ddr4_cke;
                assign #(FLY_BY) iDDR4[gl].CS_n      = ddr4_cs_n;
                assign #(FLY_BY) iDDR4[gl].ACT_n     = ddr4_act_n;
                assign #(FLY_BY) iDDR4[gl].RAS_n_A16 = ddr4_addr[16];
                assign #(FLY_BY) iDDR4[gl].CAS_n_A15 = ddr4_addr[15];
                assign #(FLY_BY) iDDR4[gl].WE_n_A14  = ddr4_addr[14];
                assign #(FLY_BY) iDDR4[gl].ADDR      = ddr4_addr[13:0];
                assign #(FLY_BY) iDDR4[gl].BA        = ddr4_ba;
                assign #(FLY_BY) iDDR4[gl].BG        = ddr4_bg;
                assign #(FLY_BY) iDDR4[gl].ODT       = ddr4_odt;
            end
            assign iDDR4[gl].ADDR_17 = 1'b0;
            assign iDDR4[gl].C       = '0;
            assign iDDR4[gl].TEN     = 1'b0;
            assign iDDR4[gl].PARITY  = 1'b0;
            assign iDDR4[gl].ZQ      = 1'b1;
            assign iDDR4[gl].PWR     = 1'b1;
            assign iDDR4[gl].VREF_CA = 1'b1;
            assign iDDR4[gl].VREF_DQ = 1'b1;
        end

        for (gi = 0; gi < BYTE_LANES; gi = gi + 1) begin : gen_bidi
            begin : normal
                for (gj = 0; gj < 8; gj = gj + 1) begin : gen_dq
                    `ifdef XILINX_SIMULATOR
                    short bidiDQ(iDDR4[gi].DQ[gj], ddr4_dq[gi*8 + gj]);
                    `else
                    tran  bidiDQ(iDDR4[gi].DQ[gj], ddr4_dq[gi*8 + gj]);
                    `endif
                end
                `ifdef XILINX_SIMULATOR
                short bidiDQS_t(iDDR4[gi].DQS_t, ddr4_dqs_p[gi]);
                short bidiDQS_c(iDDR4[gi].DQS_c, ddr4_dqs_n[gi]);
                short bidiDM   (iDDR4[gi].DM_n,   ddr4_dm_n[gi]);
                `else
                tran  bidiDQS_t(iDDR4[gi].DQS_t, ddr4_dqs_p[gi]);
                tran  bidiDQS_c(iDDR4[gi].DQS_c, ddr4_dqs_n[gi]);
                tran  bidiDM   (iDDR4[gi].DM_n,   ddr4_dm_n[gi]);
                `endif
            end
        end

        for (gi = 0; gi < BYTE_LANES; gi = gi + 1) begin : gen_mem
            ddr4_model #(
                .CONFIGURED_DQ_BITS (8),
                .CONFIGURED_DENSITY (TB_DENSITY),
                .CONFIGURED_RANKS   (1)
            ) u_ddr4_mem (
                .model_enable (model_en),
                .iDDR4        (iDDR4[gi])
            );
        end

    // -----------------------------------------------------------------
    // x16: 1 DDR4_if + 1 model per 2 byte lanes
    //   DQ[7:0]  -> lower lane, DQ[15:8] -> upper lane
    //   DQS_t/c[0] -> lower lane DQS, DQS_t/c[1] -> upper lane DQS
    //   DM_n[0] -> lower lane DM, DM_n[1] -> upper lane DM
    //   Fly-by applied per physical chip (not per byte lane)
    // -----------------------------------------------------------------
    end else if (DEVICE_WIDTH == 16) begin : gen_x16
        DDR4_if #(.CONFIGURED_DQ_BITS(16)) iDDR4[NUM_DEVICES-1:0]();

        for (gl = 0; gl < NUM_DEVICES; gl = gl + 1) begin : gen_cmd
            if (gl == 0) begin : near
                assign iDDR4[gl].CK        = {ddr4_ck_p, ddr4_ck_n};
                assign iDDR4[gl].RESET_n   = ddr4_reset_n;
                assign iDDR4[gl].CKE       = ddr4_cke;
                assign iDDR4[gl].CS_n      = ddr4_cs_n;
                assign iDDR4[gl].ACT_n     = ddr4_act_n;
                assign iDDR4[gl].RAS_n_A16 = ddr4_addr[16];
                assign iDDR4[gl].CAS_n_A15 = ddr4_addr[15];
                assign iDDR4[gl].WE_n_A14  = ddr4_addr[14];
                assign iDDR4[gl].ADDR      = ddr4_addr[13:0];
                assign iDDR4[gl].BA        = ddr4_ba;
                assign iDDR4[gl].BG        = {1'b0, ddr4_bg[0]};
                assign iDDR4[gl].ODT       = ddr4_odt;
            end else begin : far
                assign #(FLY_BY) iDDR4[gl].CK        = {ddr4_ck_p, ddr4_ck_n};
                assign #(FLY_BY) iDDR4[gl].RESET_n   = ddr4_reset_n;
                assign #(FLY_BY) iDDR4[gl].CKE       = ddr4_cke;
                assign #(FLY_BY) iDDR4[gl].CS_n      = ddr4_cs_n;
                assign #(FLY_BY) iDDR4[gl].ACT_n     = ddr4_act_n;
                assign #(FLY_BY) iDDR4[gl].RAS_n_A16 = ddr4_addr[16];
                assign #(FLY_BY) iDDR4[gl].CAS_n_A15 = ddr4_addr[15];
                assign #(FLY_BY) iDDR4[gl].WE_n_A14  = ddr4_addr[14];
                assign #(FLY_BY) iDDR4[gl].ADDR      = ddr4_addr[13:0];
                assign #(FLY_BY) iDDR4[gl].BA        = ddr4_ba;
                assign #(FLY_BY) iDDR4[gl].BG        = {1'b0, ddr4_bg[0]};
                assign #(FLY_BY) iDDR4[gl].ODT       = ddr4_odt;
            end
            assign iDDR4[gl].ADDR_17 = 1'b0;
            assign iDDR4[gl].C       = '0;
            assign iDDR4[gl].TEN     = 1'b0;
            assign iDDR4[gl].PARITY  = 1'b0;
            assign iDDR4[gl].ZQ      = 1'b1;
            assign iDDR4[gl].PWR     = 1'b1;
            assign iDDR4[gl].VREF_CA = 1'b1;
            assign iDDR4[gl].VREF_DQ = 1'b1;
        end

        for (gi = 0; gi < NUM_DEVICES; gi = gi + 1) begin : gen_bidi
            begin : normal
                for (gj = 0; gj < 8; gj = gj + 1) begin : gen_dq_lo
                    `ifdef XILINX_SIMULATOR
                    short bidiDQ(iDDR4[gi].DQ[gj], ddr4_dq[(gi*2)*8 + gj]);
                    `else
                    tran  bidiDQ(iDDR4[gi].DQ[gj], ddr4_dq[(gi*2)*8 + gj]);
                    `endif
                end
                for (gj = 0; gj < 8; gj = gj + 1) begin : gen_dq_hi
                    `ifdef XILINX_SIMULATOR
                    short bidiDQ(iDDR4[gi].DQ[8 + gj], ddr4_dq[(gi*2+1)*8 + gj]);
                    `else
                    tran  bidiDQ(iDDR4[gi].DQ[8 + gj], ddr4_dq[(gi*2+1)*8 + gj]);
                    `endif
                end
                `ifdef XILINX_SIMULATOR
                short bidiDQS_t0(iDDR4[gi].DQS_t[0], ddr4_dqs_p[gi*2]);
                short bidiDQS_c0(iDDR4[gi].DQS_c[0], ddr4_dqs_n[gi*2]);
                short bidiDQS_t1(iDDR4[gi].DQS_t[1], ddr4_dqs_p[gi*2+1]);
                short bidiDQS_c1(iDDR4[gi].DQS_c[1], ddr4_dqs_n[gi*2+1]);
                short bidiDM0   (iDDR4[gi].DM_n[0],   ddr4_dm_n[gi*2]);
                short bidiDM1   (iDDR4[gi].DM_n[1],   ddr4_dm_n[gi*2+1]);
                `else
                tran  bidiDQS_t0(iDDR4[gi].DQS_t[0], ddr4_dqs_p[gi*2]);
                tran  bidiDQS_c0(iDDR4[gi].DQS_c[0], ddr4_dqs_n[gi*2]);
                tran  bidiDQS_t1(iDDR4[gi].DQS_t[1], ddr4_dqs_p[gi*2+1]);
                tran  bidiDQS_c1(iDDR4[gi].DQS_c[1], ddr4_dqs_n[gi*2+1]);
                tran  bidiDM0   (iDDR4[gi].DM_n[0],   ddr4_dm_n[gi*2]);
                tran  bidiDM1   (iDDR4[gi].DM_n[1],   ddr4_dm_n[gi*2+1]);
                `endif
            end
        end

        for (gi = 0; gi < NUM_DEVICES; gi = gi + 1) begin : gen_mem
            ddr4_model #(
                .CONFIGURED_DQ_BITS (16),
                .CONFIGURED_DENSITY (TB_DENSITY),
                .CONFIGURED_RANKS   (1)
            ) u_ddr4_mem (
                .model_enable (model_en),
                .iDDR4        (iDDR4[gi])
            );
        end

    // -----------------------------------------------------------------
    // x4: 2 DDR4_if + 2 models per byte lane (paired into 8-bit lane)
    //   lo model DQ[3:0] -> lane bits [3:0], hi model DQ[3:0] -> bits [7:4]
    //   Both share the lane's DQS pair; no DM pin on x4 devices
    //   Fly-by applied per byte lane (both chips in a lane share CK/CMD)
    // -----------------------------------------------------------------
    end else begin : gen_x4
        DDR4_if #(.CONFIGURED_DQ_BITS(4)) iDDR4_lo[BYTE_LANES-1:0]();
        DDR4_if #(.CONFIGURED_DQ_BITS(4)) iDDR4_hi[BYTE_LANES-1:0]();

        for (gl = 0; gl < BYTE_LANES; gl = gl + 1) begin : gen_cmd
            if (gl == 0) begin : near
                assign iDDR4_lo[gl].CK        = {ddr4_ck_p, ddr4_ck_n};
                assign iDDR4_lo[gl].RESET_n   = ddr4_reset_n;
                assign iDDR4_lo[gl].CKE       = ddr4_cke;
                assign iDDR4_lo[gl].CS_n      = ddr4_cs_n;
                assign iDDR4_lo[gl].ACT_n     = ddr4_act_n;
                assign iDDR4_lo[gl].RAS_n_A16 = ddr4_addr[16];
                assign iDDR4_lo[gl].CAS_n_A15 = ddr4_addr[15];
                assign iDDR4_lo[gl].WE_n_A14  = ddr4_addr[14];
                assign iDDR4_lo[gl].ADDR      = ddr4_addr[13:0];
                assign iDDR4_lo[gl].BA        = ddr4_ba;
                assign iDDR4_lo[gl].BG        = ddr4_bg;
                assign iDDR4_lo[gl].ODT       = ddr4_odt;
                assign iDDR4_hi[gl].CK        = {ddr4_ck_p, ddr4_ck_n};
                assign iDDR4_hi[gl].RESET_n   = ddr4_reset_n;
                assign iDDR4_hi[gl].CKE       = ddr4_cke;
                assign iDDR4_hi[gl].CS_n      = ddr4_cs_n;
                assign iDDR4_hi[gl].ACT_n     = ddr4_act_n;
                assign iDDR4_hi[gl].RAS_n_A16 = ddr4_addr[16];
                assign iDDR4_hi[gl].CAS_n_A15 = ddr4_addr[15];
                assign iDDR4_hi[gl].WE_n_A14  = ddr4_addr[14];
                assign iDDR4_hi[gl].ADDR      = ddr4_addr[13:0];
                assign iDDR4_hi[gl].BA        = ddr4_ba;
                assign iDDR4_hi[gl].BG        = ddr4_bg;
                assign iDDR4_hi[gl].ODT       = ddr4_odt;
            end else begin : far
                assign #(FLY_BY) iDDR4_lo[gl].CK        = {ddr4_ck_p, ddr4_ck_n};
                assign #(FLY_BY) iDDR4_lo[gl].RESET_n   = ddr4_reset_n;
                assign #(FLY_BY) iDDR4_lo[gl].CKE       = ddr4_cke;
                assign #(FLY_BY) iDDR4_lo[gl].CS_n      = ddr4_cs_n;
                assign #(FLY_BY) iDDR4_lo[gl].ACT_n     = ddr4_act_n;
                assign #(FLY_BY) iDDR4_lo[gl].RAS_n_A16 = ddr4_addr[16];
                assign #(FLY_BY) iDDR4_lo[gl].CAS_n_A15 = ddr4_addr[15];
                assign #(FLY_BY) iDDR4_lo[gl].WE_n_A14  = ddr4_addr[14];
                assign #(FLY_BY) iDDR4_lo[gl].ADDR      = ddr4_addr[13:0];
                assign #(FLY_BY) iDDR4_lo[gl].BA        = ddr4_ba;
                assign #(FLY_BY) iDDR4_lo[gl].BG        = ddr4_bg;
                assign #(FLY_BY) iDDR4_lo[gl].ODT       = ddr4_odt;
                assign #(FLY_BY) iDDR4_hi[gl].CK        = {ddr4_ck_p, ddr4_ck_n};
                assign #(FLY_BY) iDDR4_hi[gl].RESET_n   = ddr4_reset_n;
                assign #(FLY_BY) iDDR4_hi[gl].CKE       = ddr4_cke;
                assign #(FLY_BY) iDDR4_hi[gl].CS_n      = ddr4_cs_n;
                assign #(FLY_BY) iDDR4_hi[gl].ACT_n     = ddr4_act_n;
                assign #(FLY_BY) iDDR4_hi[gl].RAS_n_A16 = ddr4_addr[16];
                assign #(FLY_BY) iDDR4_hi[gl].CAS_n_A15 = ddr4_addr[15];
                assign #(FLY_BY) iDDR4_hi[gl].WE_n_A14  = ddr4_addr[14];
                assign #(FLY_BY) iDDR4_hi[gl].ADDR      = ddr4_addr[13:0];
                assign #(FLY_BY) iDDR4_hi[gl].BA        = ddr4_ba;
                assign #(FLY_BY) iDDR4_hi[gl].BG        = ddr4_bg;
                assign #(FLY_BY) iDDR4_hi[gl].ODT       = ddr4_odt;
            end
            assign iDDR4_lo[gl].ADDR_17 = 1'b0;
            assign iDDR4_lo[gl].C       = '0;
            assign iDDR4_lo[gl].TEN     = 1'b0;
            assign iDDR4_lo[gl].PARITY  = 1'b0;
            assign iDDR4_lo[gl].ZQ      = 1'b1;
            assign iDDR4_lo[gl].PWR     = 1'b1;
            assign iDDR4_lo[gl].VREF_CA = 1'b1;
            assign iDDR4_lo[gl].VREF_DQ = 1'b1;
            assign iDDR4_hi[gl].ADDR_17 = 1'b0;
            assign iDDR4_hi[gl].C       = '0;
            assign iDDR4_hi[gl].TEN     = 1'b0;
            assign iDDR4_hi[gl].PARITY  = 1'b0;
            assign iDDR4_hi[gl].ZQ      = 1'b1;
            assign iDDR4_hi[gl].PWR     = 1'b1;
            assign iDDR4_hi[gl].VREF_CA = 1'b1;
            assign iDDR4_hi[gl].VREF_DQ = 1'b1;
        end

        for (gi = 0; gi < BYTE_LANES; gi = gi + 1) begin : gen_bidi
            begin : normal
                for (gj = 0; gj < 4; gj = gj + 1) begin : gen_dq_lo
                    `ifdef XILINX_SIMULATOR
                    short bidiDQ(iDDR4_lo[gi].DQ[gj], ddr4_dq[gi*8 + gj]);
                    `else
                    tran  bidiDQ(iDDR4_lo[gi].DQ[gj], ddr4_dq[gi*8 + gj]);
                    `endif
                end
                for (gj = 0; gj < 4; gj = gj + 1) begin : gen_dq_hi
                    `ifdef XILINX_SIMULATOR
                    short bidiDQ(iDDR4_hi[gi].DQ[gj], ddr4_dq[gi*8 + 4 + gj]);
                    `else
                    tran  bidiDQ(iDDR4_hi[gi].DQ[gj], ddr4_dq[gi*8 + 4 + gj]);
                    `endif
                end
                `ifdef XILINX_SIMULATOR
                short bidiDQS_tlo(iDDR4_lo[gi].DQS_t, ddr4_dqs_p[gi]);
                short bidiDQS_clo(iDDR4_lo[gi].DQS_c, ddr4_dqs_n[gi]);
                short bidiDQS_thi(iDDR4_hi[gi].DQS_t, ddr4_dqs_p[gi]);
                short bidiDQS_chi(iDDR4_hi[gi].DQS_c, ddr4_dqs_n[gi]);
                `else
                tran  bidiDQS_tlo(iDDR4_lo[gi].DQS_t, ddr4_dqs_p[gi]);
                tran  bidiDQS_clo(iDDR4_lo[gi].DQS_c, ddr4_dqs_n[gi]);
                tran  bidiDQS_thi(iDDR4_hi[gi].DQS_t, ddr4_dqs_p[gi]);
                tran  bidiDQS_chi(iDDR4_hi[gi].DQS_c, ddr4_dqs_n[gi]);
                `endif
            end
        end

        for (gi = 0; gi < BYTE_LANES; gi = gi + 1) begin : gen_mem
            ddr4_model #(
                .CONFIGURED_DQ_BITS (4),
                .CONFIGURED_DENSITY (TB_DENSITY),
                .CONFIGURED_RANKS   (1)
            ) u_ddr4_lo (
                .model_enable (model_en),
                .iDDR4        (iDDR4_lo[gi])
            );
            ddr4_model #(
                .CONFIGURED_DQ_BITS (4),
                .CONFIGURED_DENSITY (TB_DENSITY),
                .CONFIGURED_RANKS   (1)
            ) u_ddr4_hi (
                .model_enable (model_en),
                .iDDR4        (iDDR4_hi[gi])
            );
        end
    end endgenerate

    // ===================================================================
    // Human-Readable Debug Signals
    //
    // These string-type regs decode controller ROM phases and DFI/DDR4
    // commands into human-readable names. View them in the waveform
    // viewer with ASCII radix to get "MRS MR3", "ACT", "WR", etc.
    // instead of hex gibberish. There are three decoders:
    //   dbg_dfi_cmd    - DFI-side (pre-PHY, what the controller issues)
    //   dbg_ddr4_cmd   - DDR4-side (post-PHY, what hits the bus)
    //   dbg_micron_cmd - Micron interface (what the model actually sees)
    // ===================================================================
    reg [16*8-1:0] dbg_rom_phase;
    reg [8*8-1:0]  dbg_dfi_cmd;
    reg [8*8-1:0]  dbg_ddr4_cmd;
    reg [8*8-1:0]  dbg_micron_cmd;

    always @* begin
        case (u_dut.u_controller.instruction_address)
            6'd0:    dbg_rom_phase = "POWER_ON_RESET";
            6'd1:    dbg_rom_phase = "CKE_LOW";
            6'd2:    dbg_rom_phase = "tXPR_WAIT";
            6'd3:    dbg_rom_phase = "MRS MR3";
            6'd4:    dbg_rom_phase = "tMRD_WAIT";
            6'd5:    dbg_rom_phase = "MRS MR6";
            6'd6:    dbg_rom_phase = "tMRD_WAIT";
            6'd7:    dbg_rom_phase = "MRS MR5";
            6'd8:    dbg_rom_phase = "tMRD_WAIT";
            6'd9:    dbg_rom_phase = "MRS MR4";
            6'd10:   dbg_rom_phase = "tMRD_WAIT";
            6'd11:   dbg_rom_phase = "MRS MR2";
            6'd12:   dbg_rom_phase = "tMRD_WAIT";
            6'd13:   dbg_rom_phase = "MRS MR1";
            6'd14:   dbg_rom_phase = "tMRD_WAIT";
            6'd15:   dbg_rom_phase = "MRS MR0";
            6'd16:   dbg_rom_phase = "tMOD_WAIT";
            6'd17:   dbg_rom_phase = "ZQCL";
            6'd18:   dbg_rom_phase = "DLL_LOCK";
            6'd19:   dbg_rom_phase = "PRE_ALL";
            6'd20:   dbg_rom_phase = "MPR_ENABLE";
            6'd21:   dbg_rom_phase = "tMOD_WAIT";
            6'd22:   dbg_rom_phase = "READ_CAL";
            6'd23:   dbg_rom_phase = "MPR_DISABLE";
            6'd24:   dbg_rom_phase = "tMOD_WAIT";
            6'd25:   dbg_rom_phase = "WL_ENABLE";
            6'd26:   dbg_rom_phase = "tWLMRD_WAIT";
            6'd27:   dbg_rom_phase = "WRITE_CAL";
            6'd28:   dbg_rom_phase = "WL_DISABLE";
            6'd29:   dbg_rom_phase = "tMOD_WAIT";
            6'd30:   dbg_rom_phase = "PRE_ALL";
            6'd31:   dbg_rom_phase = "REFRESH";
            6'd32:   dbg_rom_phase = "INIT_DONE";
            6'd33:   dbg_rom_phase = "REF_PRE_ALL";
            6'd34:   dbg_rom_phase = "REF_REFRESH";
            6'd35:   dbg_rom_phase = "REF_IDLE";
            default: dbg_rom_phase = "???";
        endcase
    end

    always @* begin
        if (u_dut.u_controller.o_dfi_cs_n[0])
            dbg_dfi_cmd = "DES";
        else if (!u_dut.u_controller.o_dfi_act_n[0])
            dbg_dfi_cmd = "ACT";
        else begin
            case ({u_dut.u_controller.o_dfi_ras_n[0],
                   u_dut.u_controller.o_dfi_cas_n[0],
                   u_dut.u_controller.o_dfi_we_n[0]})
                3'b000:  dbg_dfi_cmd = "MRS";
                3'b001:  dbg_dfi_cmd = "REF";
                3'b010:  dbg_dfi_cmd = u_dut.u_controller.o_dfi_address[10] ? "PRE ALL" : "PRE";
                3'b100:  dbg_dfi_cmd = "WR";
                3'b101:  dbg_dfi_cmd = "RD";
                3'b110:  dbg_dfi_cmd = u_dut.u_controller.o_dfi_address[10] ? "ZQCL" : "ZQCS";
                3'b111:  dbg_dfi_cmd = "NOP";
                default: dbg_dfi_cmd = "???";
            endcase
        end
    end

    always @* begin
        if (ddr4_cs_n)
            dbg_ddr4_cmd = "DES";
        else if (!ddr4_act_n)
            dbg_ddr4_cmd = "ACT";
        else begin
            case ({ddr4_addr[16], ddr4_addr[15], ddr4_addr[14]})
                3'b000:  dbg_ddr4_cmd = "MRS";
                3'b001:  dbg_ddr4_cmd = "REF";
                3'b010:  dbg_ddr4_cmd = ddr4_addr[10] ? "PRE ALL" : "PRE";
                3'b100:  dbg_ddr4_cmd = "WR";
                3'b101:  dbg_ddr4_cmd = "RD";
                3'b110:  dbg_ddr4_cmd = ddr4_addr[10] ? "ZQCL" : "ZQCS";
                3'b111:  dbg_ddr4_cmd = "NOP";
                default: dbg_ddr4_cmd = "???";
            endcase
        end
    end

    // Micron-side command decode  -  reads the DDR4 bus signals that
    // feed directly into the Micron model (near-end chip, zero fly-by).
    wire micron_cs_n  = ddr4_cs_n;
    wire micron_act_n = ddr4_act_n;
    wire micron_ras   = ddr4_addr[16];
    wire micron_cas   = ddr4_addr[15];
    wire micron_we    = ddr4_addr[14];

    always @* begin
        if (micron_cs_n)
            dbg_micron_cmd = "DES";
        else if (!micron_act_n)
            dbg_micron_cmd = "ACT";
        else begin
            case ({micron_ras, micron_cas, micron_we})
                3'b000:  dbg_micron_cmd = "MRS";
                3'b001:  dbg_micron_cmd = "REF";
                3'b010:  dbg_micron_cmd = "PRE";
                3'b100:  dbg_micron_cmd = "WR";
                3'b101:  dbg_micron_cmd = "RD";
                3'b110:  dbg_micron_cmd = "ZQCL";
                3'b111:  dbg_micron_cmd = "NOP";
                default: dbg_micron_cmd = "???";
            endcase
        end
    end

    // ===================================================================
    // Monitoring and Test Control
    // ===================================================================
    reg reset_done_seen;
    initial reset_done_seen = 1'b0;

    initial $timeformat(-9, 3, "ns", 0);

    initial begin
        $display("[0ns] CONFIG: DDR4_CLK=%0dps DEVICE_WIDTH=x%0d BYTE_LANES=%0d BG_BITS=%0d FLY_BY=%0dps ADDR_MAPPING=%0d BIST_MODE=%0d DENSITY=%0dGb NUM_DEVICES=%0d",
            DDR4_CLK_PERIOD, DEVICE_WIDTH, BYTE_LANES, BG_BITS, FLY_BY, TB_ADDR_MAPPING, TB_BIST_MODE, TB_DENSITY_GB, NUM_DEVICES);
    end

    always @(posedge controller_clk) begin
        if (rst_n && u_dut.u_controller.reset_done && !reset_done_seen) begin
            $display("[%0t] PASS: DDR4 init sequence complete (reset_done)", $realtime);
            reset_done_seen <= 1'b1;
        end
    end

    // Init status monitor
    reg calib_complete_seen;
    reg init_failed_seen;
    initial begin calib_complete_seen = 1'b0; init_failed_seen = 1'b0; end

    always @(posedge controller_clk) begin
        if (rst_n && calib_complete_int && !calib_complete_seen) begin
            $display("[%0t] CALIB_COMPLETE asserted (internal)", $realtime);
            calib_complete_seen <= 1'b1;
        end
        if (rst_n && init_failed && !init_failed_seen) begin
            $display("[%0t] INIT_FAILED asserted", $realtime);
            init_failed_seen <= 1'b1;
        end
    end

    // PHY training progress monitor  -  tracks gate, eye, and WL phases
    reg [3:0] prev_phy_state;
    initial prev_phy_state = 4'd0;

    always @(posedge controller_clk) begin
        if (rst_n) begin
            prev_phy_state <= `TB_PHY_HIER.phy_state;

            // Gate training (no-op â€�? responds immediately)
            if (prev_phy_state == 4'd0 && `TB_PHY_HIER.phy_state == 4'd1)
`ifdef TB_USE_NATIVE_PHY
                $display("[%0t] PHY native gate training started", $realtime);
`else
                $display("[%0t] PHY gate training (no-op)", $realtime);
`endif
            if (prev_phy_state == 4'd1 && `TB_PHY_HIER.phy_state == 4'd0)
                $display("[%0t] PHY gate training done", $realtime);

            // Eye training
            if (prev_phy_state == 4'd0 && `TB_PHY_HIER.phy_state == 4'd2)
                $display("[%0t] PHY eye training started", $realtime);
            if (`TB_PHY_HIER.phy_state == 4'd7 && prev_phy_state != 4'd7)
                if (BYTE_LANES > 1)
                    $display("[%0t] PHY eye training done (lane0 center=%0d bs=%0d late=%0d, lane1 center=%0d bs=%0d late=%0d)",
                        $realtime,
                        `TB_PHY_HIER.eye_center_tap[0], `TB_PHY_HIER.bitslip_count_q[0], `TB_PHY_HIER.rd_lat_extra[0],
                        `TB_PHY_HIER.eye_center_tap[1], `TB_PHY_HIER.bitslip_count_q[1], `TB_PHY_HIER.rd_lat_extra[1]);
                else
                    $display("[%0t] PHY eye training done (lane0 center=%0d bs=%0d late=%0d)",
                        $realtime,
                        `TB_PHY_HIER.eye_center_tap[0], `TB_PHY_HIER.bitslip_count_q[0], `TB_PHY_HIER.rd_lat_extra[0]);

            // Write leveling
            if (prev_phy_state == 4'd0 && `TB_PHY_HIER.phy_state == 4'd8)
                $display("[%0t] PHY write leveling started", $realtime);
            if (`TB_PHY_HIER.phy_state == 4'd11 && prev_phy_state != 4'd11)
                if (BYTE_LANES > 1)
                    $display("[%0t] PHY write leveling done (lane0 dqs_tap=%0d, lane1 dqs_tap=%0d)",
                        $realtime,
                        `TB_PHY_HIER.wl_tap[0],
                        `TB_PHY_HIER.wl_tap[1]);
                else
                    $display("[%0t] PHY write leveling done (lane0 dqs_tap=%0d)",
                        $realtime,
                        `TB_PHY_HIER.wl_tap[0]);
        end
    end

    // Calibration result validation  -  fires once when training completes
    reg calib_results_checked;
    initial calib_results_checked = 1'b0;

    always @(posedge controller_clk) begin
        if (calib_complete_int && !calib_results_checked) begin
            calib_results_checked <= 1'b1;
            $display("[%0t] === CALIBRATION RESULTS ===", $realtime);
            $display("[%0t]   FLY_BY_DELAY = %0d ps", $realtime, FLY_BY);
            if (BYTE_LANES > 1)
                $display("[%0t]   Gate: lane0 bs=%0d, lane1 bs=%0d",
                    $realtime,
                    `TB_PHY_HIER.bitslip_count_q[0],
                    `TB_PHY_HIER.bitslip_count_q[1]);
            else
                $display("[%0t]   Gate: lane0 bs=%0d",
                    $realtime,
                    `TB_PHY_HIER.bitslip_count_q[0]);
            if (BYTE_LANES > 1)
                $display("[%0t]   Eye:  lane0 center=%0d bs=%0d late=%0d, lane1 center=%0d bs=%0d late=%0d",
                    $realtime,
                    `TB_PHY_HIER.eye_center_tap[0], `TB_PHY_HIER.bitslip_count_q[0], `TB_PHY_HIER.rd_lat_extra[0],
                    `TB_PHY_HIER.eye_center_tap[1], `TB_PHY_HIER.bitslip_count_q[1], `TB_PHY_HIER.rd_lat_extra[1]);
            else
                $display("[%0t]   Eye:  lane0 center=%0d bs=%0d late=%0d",
                    $realtime,
                    `TB_PHY_HIER.eye_center_tap[0], `TB_PHY_HIER.bitslip_count_q[0], `TB_PHY_HIER.rd_lat_extra[0]);
            if (BYTE_LANES > 1)
                $display("[%0t]   WL:   lane0 dqs_tap=%0d dq_tap=%0d, lane1 dqs_tap=%0d dq_tap=%0d",
                    $realtime,
                    `TB_PHY_HIER.wl_tap[0], `TB_PHY_HIER.wl_dq_tap[0],
                    `TB_PHY_HIER.wl_tap[1], `TB_PHY_HIER.wl_dq_tap[1]);
            else
                $display("[%0t]   WL:   lane0 dqs_tap=%0d dq_tap=%0d",
                    $realtime,
                    `TB_PHY_HIER.wl_tap[0], `TB_PHY_HIER.wl_dq_tap[0]);
            if (BYTE_LANES > 1 && FLY_BY >= 100) begin
                if (`TB_PHY_HIER.wl_tap[0] == `TB_PHY_HIER.wl_tap[1])
                    $display("[%0t]   WARNING: WL taps identical despite FLY_BY=%0dps  -  expected asymmetry",
                        $realtime, FLY_BY);
                else
                    $display("[%0t]   OK: WL taps differ (lane0=%0d, lane1=%0d)  -  fly-by compensation working",
                        $realtime, `TB_PHY_HIER.wl_tap[0], `TB_PHY_HIER.wl_tap[1]);
            end
        end
    end

    // ===================================================================
    // BIST Completion Monitor
    //
    // The built-in BIST (in ddr4_prober) runs automatically after init.
    // This block detects the busy->idle transition and reports the result.
    // ===================================================================
    reg bist_was_busy;
    reg bist_done_seen;
    initial begin bist_was_busy = 1'b0; bist_done_seen = 1'b0; end

    always @(posedge controller_clk) begin
        if (bist_busy_int)
            bist_was_busy <= 1'b1;
        if (bist_was_busy && !bist_busy_int && !bist_done_seen) begin
            bist_done_seen <= 1'b1;
            if (!bist_fail_int)
                $display("[%0t] BIST RESULT: PASS  -  correct=%0d errors=%0d",
                    $realtime, bist_correct_int, bist_error_int);
            else
                $display("[%0t] BIST RESULT: FAIL  -  correct=%0d errors=%0d",
                    $realtime, bist_correct_int, bist_error_int);
        end
    end

    // ===================================================================
    // DFI Write/Read Boundary Monitor
    //
    // Logs every DFI write and read burst during BIST. The "p2rise" field
    // extracts bits [79:64] (DFI phase-2 rising edge) which is useful for
    // debugging serializer lane-swap bugs in the PHY.
    // ===================================================================
    integer dfi_wr_seq, dfi_rd_seq;
    initial begin dfi_wr_seq = 0; dfi_rd_seq = 0; end

    always @(posedge controller_clk) begin
        if (bist_busy_int) begin
            if (|u_dut.u_controller.o_dfi_wrdata_en) begin
            `ifndef SIM_QUIET_DFI_BURST_LOG
                $display("[DBG-WR] #%0d p2rise=%04h full=%0h",
                    dfi_wr_seq,
                    u_dut.u_controller.o_dfi_wrdata[79:64],
                    u_dut.u_controller.o_dfi_wrdata);
            `endif
                dfi_wr_seq = dfi_wr_seq + 1;
            end
            if (|`TB_PHY_HIER.o_dfi_rddata_valid) begin
            `ifndef SIM_QUIET_DFI_BURST_LOG
                $display("[DBG-RD] #%0d p2rise=%04h full=%0h",
                    dfi_rd_seq,
                    `TB_PHY_HIER.o_dfi_rddata[79:64],
                    `TB_PHY_HIER.o_dfi_rddata);
            `endif
                dfi_rd_seq = dfi_rd_seq + 1;
            end
        end
    end

    // ===================================================================
    // WB Write Stimulus  -  Multi-phase test covering all scheduler paths
    //
    // The test phases below are designed to hit every interesting path in
    // the DDR4 controller's bank-state machine and command scheduler:
    //
    //   Phase A: Cold writes to idle banks       (ACT -> WR)
    //   Phase B: Bank hits, same row still open  (WR only, no ACT needed)
    //   Phase C: Row miss, different row          (PRE -> ACT -> WR)
    //   Phase D: Post-refresh, banks closed       (ACT -> WR after REF)
    //   Phase E: Same-BG different-bank writes   (tRRD_L / tCCD_L timing)
    //   Phase F: Read-back verification          (ACT -> RD, bank-hit RD)
    //   Phase G: Same-bank rapid re-access       (tRC stress)
    //   Phase H: Write-then-read same BG         (tWTR_L / tWTR_S)
    //   Phase I: Data integrity round-trip       (walking-1, all-F, etc.)
    //   Phase J: Pipelined writes (16 addrs)     (STB stays high, no gaps)
    //   Phase K: Read->Write turnaround          (RD->WR bus contention)
    //   Phase L: Byte-lane masking               (DM / wb_sel partial writes)
    //   Phase M: tFAW stress                     (5 rapid ACTs)
    //   Phase N: Pipeline saturation             (32 back-to-back writes)
    //   Phase O: Refresh-during-traffic          (data survives refresh)
    //   Phase P: Address boundary corners        (max col, max addr, etc.)
    //   Phase Q: Multi-BG interleaving           (all 16 banks exercised)
    //   CSR:     Debug register readout          (BIST counters, version)
    //   RETRIG:  BIST re-trigger via CSR write   (re-run BIST from SW)
    //
    // ADDR_MAPPING=1 (BG-interleaved) address layout:
    //   bits [1:0]   = BG
    //   bits [7:2]   = col_upper (-> col[9:4])
    //   bits [9:8]   = BA
    //   bits [25:10]  = row
    // ===================================================================

    integer ref_count;
    initial ref_count = 0;

    reg [8*8-1:0] test_phase;
    initial test_phase = "IDLE";
    reg all_tests_done;
    initial all_tests_done = 1'b0;

    // Command counters (used by monitor and throughput)
    integer act_count, wr_count, rd_count, pre_count;
    initial begin act_count = 0; wr_count = 0; rd_count = 0; pre_count = 0; end

    // Throughput measurement: free-running cycle counter + per-phase snapshots
    integer cycle_count;
    initial cycle_count = 0;
    always @(posedge controller_clk) if (reset_done_seen) cycle_count = cycle_count + 1;

    integer phase_n_start, phase_n_end;
    integer phase_o_start, phase_o_end, phase_o_wr_count;
    integer phase_q_start, phase_q_end;
    integer phase_n_wr_start, phase_o_wr_start, phase_q_wr_start;
    initial begin
        phase_n_start = 0; phase_n_end = 0;
        phase_o_start = 0; phase_o_end = 0; phase_o_wr_count = 0;
        phase_q_start = 0; phase_q_end = 0;
        phase_n_wr_start = 0; phase_o_wr_start = 0; phase_q_wr_start = 0;
    end

    // ADDR_MAPPING=1 address field positions
    localparam ROW_SHIFT = BG_BITS + (COL_BITS - COL_LOW) + BA_BITS;
    localparam BA_SHIFT  = BG_BITS + (COL_BITS - COL_LOW);
    localparam COL_SHIFT = BG_BITS;

    // Row offsets for first/middle/last row testing
    localparam [WB_ADDR_BITS-1:0] FIRST_ROW_OFF = 0,
                                    MID_ROW_OFF   = (1 << (ROW_BITS/2)) << ROW_SHIFT,
                                    LAST_ROW_OFF  = ((1 << ROW_BITS) - 4) << ROW_SHIFT;

    // Base addresses: BG selection (bits [1:0])
    localparam [WB_ADDR_BITS-1:0] ADDR_BG0 = 0, ADDR_BG1 = 1,
                                    ADDR_BG2 = 2, ADDR_BG3 = 3;

    // BA offsets (bits [10:9])
    localparam [WB_ADDR_BITS-1:0] BA0_OFF = 0,
                                    BA1_OFF = (1 << BA_SHIFT),
                                    BA2_OFF = (2 << BA_SHIFT),
                                    BA3_OFF = (3 << BA_SHIFT);

    // Column offsets (bits [8:2])
    localparam [WB_ADDR_BITS-1:0] COL1_OFF = (1 << COL_SHIFT),
                                    COL2_OFF = (2 << COL_SHIFT);

    // Row offset for row-miss testing (bits [26:11])
    localparam [WB_ADDR_BITS-1:0] ROW1_OFF = (1 << ROW_SHIFT);

    // Composite addresses (backward-compatible names)
    localparam [WB_ADDR_BITS-1:0] ROW0_BG0 = ADDR_BG0,
                                    ROW0_BG1 = ADDR_BG1,
                                    ROW0_BG2 = ADDR_BG2,
                                    ROW0_BG3 = ADDR_BG3,
                                    ROW1_BG0 = ROW1_OFF | ADDR_BG0,
                                    ROW1_BG1 = ROW1_OFF | ADDR_BG1,
                                    ROW1_BG2 = ROW1_OFF | ADDR_BG2,
                                    ROW1_BG3 = ROW1_OFF | ADDR_BG3,
                                    ROW0_BG0_BA1 = BA1_OFF | ADDR_BG0,
                                    ROW0_BG0_BA2 = BA2_OFF | ADDR_BG0,
                                    ROW0_BG0_BA3 = BA3_OFF | ADDR_BG0,
                                    ROW2_BG0 = (2 << ROW_SHIFT) | ADDR_BG0,
                                    ROW2_BG1 = (2 << ROW_SHIFT) | ADDR_BG1,
                                    ROW2_BG2 = (2 << ROW_SHIFT) | ADDR_BG2,
                                    ROW2_BG3 = (2 << ROW_SHIFT) | ADDR_BG3,
                                    ROW2_BG0_C1 = (2 << ROW_SHIFT) | COL1_OFF | ADDR_BG0,
                                    ROW2_BG1_C1 = (2 << ROW_SHIFT) | COL1_OFF | ADDR_BG1,
                                    ROW0_BG1_BA1 = BA1_OFF | ADDR_BG1,
                                    ROW0_BG1_BA2 = BA2_OFF | ADDR_BG1,
                                    ROW0_BG1_BA3 = BA3_OFF | ADDR_BG1,
                                    ROW0_BG2_BA1 = BA1_OFF | ADDR_BG2,
                                    ROW0_BG2_BA2 = BA2_OFF | ADDR_BG2,
                                    ROW0_BG2_BA3 = BA3_OFF | ADDR_BG2,
                                    ROW0_BG3_BA1 = BA1_OFF | ADDR_BG3,
                                    ROW0_BG3_BA2 = BA2_OFF | ADDR_BG3,
                                    ROW0_BG3_BA3 = BA3_OFF | ADDR_BG3;

    // -----------------------------------------------------------------
    // =================================================================
    // Pipelined Wishbone B4 Master Infrastructure
    //
    // HOW IT WORKS (big picture):
    //   1. Test phases "push" transactions (writes/reads) into a queue.
    //   2. wb_pump_all() fires them onto the bus as fast as the slave
    //      allows â€�? one per clock cycle unless STALL is asserted.
    //   3. Read responses are verified automatically as ACKs come back.
    //
    // WHY: A real WB B4 pipelined master never inserts idle cycles
    // between requests. It keeps STB=1 and only pauses when STALL=1.
    // This is how the BIST engine (ddr4_prober) works. The testbench
    // must behave the same way to properly stress the controller's
    // bank-state machine, refresh interleaving, and WR/RD scheduling.
    //
    // PROTOCOL SUMMARY (Wishbone B4 Pipelined Mode):
    //   - Master asserts CYC for the entire burst.
    //   - Master asserts STB each cycle with addr/data/we/sel.
    //   - If STALL=0 at the clock edge â†’ request ACCEPTED.
    //   - If STALL=1 at the clock edge â†’ HOLD same request, try again.
    //   - Each accepted request produces exactly one ACK later (in order).
    //   - Master deasserts STB when no more requests to send.
    //   - Master keeps CYC high until the last ACK is received.
    //
    // DATA FLOW:
    //
    //   wb_push_write/read()    wb_pump_all()          ACK checker
    //         â�?‚                      â�?‚                      â�?‚
    //         â–¼                      â�?‚                      â�?‚
    //   â�?Œâ�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�??   pop front    â�?‚     on ACK: pop      â�?‚
    //   â�?‚  txn_q   â�?‚ â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â–º bus â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â–º â�?Œâ�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�??
    //   â�?‚ (pending)â�?‚  when !STALL   â�?‚                â�?‚ inflight_q  â�?‚
    //   â�?�?â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?˜                â�?‚                â�?‚(waiting ACK)â�?‚
    //                               â�?‚                â�?�?â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?˜
    //                               â�?‚                      â�?‚
    //                               â�?‚               if read: compare
    //                               â�?‚               wb_rdata vs expected
    // =================================================================

    // Transaction descriptor â€�? one entry per WB request.
    // For writes: 'data' = write data to put on the bus.
    // For reads:  'data' = expected read-back value (used for verification).
    typedef struct {
        logic [WB_ADDR_BITS-1:0] addr;
        logic [WB_DATA_BITS-1:0] data;
        logic                     we;   // 1=write, 0=read
        logic [WB_SEL_BITS-1:0]  sel;   // byte-lane enables (all-1 = full word)
    } wb_txn_t;

    // txn_q:      Requests waiting to be sent (not yet on the bus).
    // inflight_q: Requests already accepted by the slave (STALL was 0),
    //             but we haven't received their ACK yet.
    wb_txn_t txn_q[$];
    wb_txn_t inflight_q[$];

    // Running count of read-data mismatches across all test phases.
    integer rd_err_count;
    initial rd_err_count = 0;

    // -----------------------------------------------------------------
    // Push tasks â€�? add transactions to the pending queue.
    // These are NON-BLOCKING: they just enqueue and return immediately.
    // Nothing happens on the bus until wb_pump_all() is called.
    // -----------------------------------------------------------------

    // Queue a full-word write (all byte lanes enabled).
    task wb_push_write(input [WB_ADDR_BITS-1:0] addr, input [WB_DATA_BITS-1:0] data);
        begin
            automatic wb_txn_t t;
            t.addr = addr; t.data = data; t.we = 1'b1; t.sel = {WB_SEL_BITS{1'b1}};
            txn_q.push_back(t);
        end
    endtask

    // Queue a partial write (only bytes where sel=1 are written to DRAM;
    // bytes where sel=0 keep their old value â€�? this exercises DDR4 DM_n).
    task wb_push_write_masked(input [WB_ADDR_BITS-1:0] addr, input [WB_DATA_BITS-1:0] data, input [WB_SEL_BITS-1:0] sel);
        begin
            automatic wb_txn_t t;
            t.addr = addr; t.data = data; t.we = 1'b1; t.sel = sel;
            txn_q.push_back(t);
        end
    endtask

    // Queue a read with expected data. When the ACK arrives, the
    // checker compares wb_rdata against exp_data and reports PASS/FAIL.
    task wb_push_read(input [WB_ADDR_BITS-1:0] addr, input [WB_DATA_BITS-1:0] exp_data);
        begin
            automatic wb_txn_t t;
            t.addr = addr; t.data = exp_data; t.we = 1'b0; t.sel = {WB_SEL_BITS{1'b1}};
            txn_q.push_back(t);
        end
    endtask

    // -----------------------------------------------------------------
    // wb_pump_all â€�? The core pipelined driver.
    //
    // Sends ALL queued transactions onto the WB bus at full speed,
    // then waits until every ACK has been received.
    //
    // Timing diagram (no stalls):
    //
    //   CLK   â�?€â�??  â�?Œâ�?€â�?€â�??  â�?Œâ�?€â�?€â�??  â�?Œâ�?€â�?€â�??  â�?Œâ�?€â�?€â�??  â�?Œâ�?€â�?€â�??  â�?Œâ�?€â�?€â�??
    //   CYC   â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€ (high entire time)
    //   STB   â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€â�?€ (drops when queue empty)
    //   STALL  _____________________________________ (slave not stalling)
    //   ADDR   [A0][A1][A2][A3]                      (advances each cycle)
    //   ACK    ___________[1][1][1][1]               (arrives later, in order)
    //
    // When STALL=1: addr/data/we/sel FREEZE until stall drops.
    // -----------------------------------------------------------------
    task wb_pump_all;
        begin
            if (txn_q.size() == 0) return;

            // --- Start of burst: assert CYC, present first transaction ---
            wb_cyc  = 1'b1;
            wb_stb  = 1'b1;
            wb_we   = txn_q[0].we;
            wb_addr = txn_q[0].addr;
            wb_data = txn_q[0].data;
            wb_sel  = txn_q[0].sel;

            // Loop until all requests sent AND all ACKs received.
            while (txn_q.size() > 0 || inflight_q.size() > 0) begin
                @(posedge controller_clk);

                // --- ACK processing (response side) ---
                // When the slave asserts ACK, it means the oldest in-flight
                // request has completed. For writes we just consume it.
                // For reads we compare the returned data to what we expected.
                if (wb_ack && inflight_q.size() > 0) begin
                    automatic wb_txn_t done = inflight_q.pop_front();
                    if (!done.we) begin
                        if (wb_rdata !== done.data) begin
                            $display("[%0t]   FAIL: addr=0x%0h exp=0x%0h got=0x%0h",
                                $realtime, done.addr, done.data, wb_rdata);
                            rd_err_count = rd_err_count + 1;
                        end else begin
                            $display("[%0t]   PASS: addr=0x%0h data=0x%0h", $realtime, done.addr, wb_rdata);
                        end
                    end
                end

                // --- Request acceptance (request side) ---
                // If STB=1 and STALL=0, the slave accepted our current request.
                // Move it from txn_q â†’ inflight_q, then present the next one.
                // If no more requests: drop STB (but keep CYC for remaining ACKs).
                if (wb_stb && !wb_stall) begin
                    if (txn_q.size() > 0) begin
                        inflight_q.push_back(txn_q.pop_front());
                        if (txn_q.size() > 0) begin
                            // Present next transaction immediately (no idle gap!)
                            wb_we   = txn_q[0].we;
                            wb_addr = txn_q[0].addr;
                            wb_data = txn_q[0].data;
                            wb_sel  = txn_q[0].sel;
                        end else begin
                            // All requests sent â€�? stop strobing
                            wb_stb = 1'b0;
                        end
                    end
                end
                // If STALL=1: do nothing â€�? hold current signals unchanged.
                // The slave will see the same request again next cycle.
            end

            // --- End of burst: all ACKs received, release bus ---
            wb_cyc = 1'b0;
            wb_stb = 1'b0;
            wb_we  = 1'b0;
        end
    endtask

    task wb_idle;
        begin
            wb_stb = 1'b0;
            wb_cyc = 1'b0;
            wb_we  = 1'b0;
        end
    endtask

    // Debug CSR port tasks (always accessible, 1-cycle latency).
    // The slave is zero-wait-state: accepts on EDGE A, responds on EDGE B.
    // No ACK polling needed â€�? deterministic 2-edge sequence.
    task wb_dbg_read(input [3:0] addr);
        begin
            wb_dbg_cyc  = 1'b1;
            wb_dbg_stb  = 1'b1;
            wb_dbg_we   = 1'b0;
            wb_dbg_addr = addr;
            wb_dbg_sel  = 4'hF;
            @(posedge controller_clk); // EDGE A: slave accepts request
            wb_dbg_stb = 1'b0;
            @(posedge controller_clk); // EDGE B: ACK + data valid
            // wb_dbg_rdata is valid at this point
        end
    endtask

    task wb_dbg_write(input [3:0] addr, input [31:0] wdata);
        begin
            wb_dbg_cyc  = 1'b1;
            wb_dbg_stb  = 1'b1;
            wb_dbg_we   = 1'b1;
            wb_dbg_addr = addr;
            wb_dbg_data = wdata;
            wb_dbg_sel  = 4'hF;
            @(posedge controller_clk); // EDGE A: slave accepts + executes write
            wb_dbg_stb = 1'b0;
            @(posedge controller_clk); // EDGE B: ACK asserted (write complete)
        end
    endtask

    task wb_dbg_idle;
        begin
            wb_dbg_stb = 1'b0;
            wb_dbg_cyc = 1'b0;
            wb_dbg_we  = 1'b0;
        end
    endtask

    function automatic [WB_DATA_BITS-1:0] gen_pattern_tb;
        input [7:0] addr;
        reg [31:0] seed;
        begin
            seed = {addr ^ 8'hA5, addr ^ 8'h5A, addr ^ 8'h3C, addr ^ 8'h7E};
            gen_pattern_tb = {4{seed}};
        end
    endfunction

    integer wb_write_count;
    initial wb_write_count = 0;

    initial begin
        wait (init_done || init_failed);
    `ifdef SIM_FORCE_TRAIN_FAIL
        if (init_failed) begin
            $display("[%0t] PASS: init_failed asserted as expected (forced training failure)", $realtime);
            $finish;
        end else begin
            $display("[%0t] FAIL: expected init_failed but got init_done", $realtime);
            $finish;
        end
    `elsif SIM_NATIVE_TX_DEBUG
        // =============================================================
        // Native PHY TX diagnostic
        //
        // Keep the controller in the transaction path: these requests use
        // the same Wishbone master that normal software would use, allowing
        // the controller to generate ACT/WR/RD and all JEDEC delays.  There
        // is deliberately one full-width write followed by one read only.
        // The byte pattern is lane-distinct and transition-rich, making the
        // complete BL8 DQ/DQS burst unambiguous in trace.vcd.
        // =============================================================
        begin : native_tx_debug
            localparam integer DBG_BURST_WORDS = 4;
            reg [WB_DATA_BITS-1:0] dbg_write_data [0:DBG_BURST_WORDS-1];
            reg [WB_ADDR_BITS-1:0] dbg_addr;
            reg [31:0] dbg_lfsr;
            integer dbg_byte, dbg_word;

            test_phase = "NAT_TX_DBG";
            dbg_addr = {WB_ADDR_BITS{1'b0}};
            dbg_lfsr = 32'h1D87_A5C3;
            for (dbg_word = 0; dbg_word < DBG_BURST_WORDS; dbg_word = dbg_word + 1) begin
                dbg_write_data[dbg_word] = {WB_DATA_BITS{1'b0}};
                for (dbg_byte = 0; dbg_byte < WB_SEL_BITS; dbg_byte = dbg_byte + 1) begin
                    dbg_write_data[dbg_word][dbg_byte*8 +: 8] = dbg_lfsr[7:0];
                    dbg_lfsr = {dbg_lfsr[30:0],
                                dbg_lfsr[31] ^ dbg_lfsr[21] ^
                                dbg_lfsr[1] ^ dbg_lfsr[0]};
                end
            end

            // Calibration has completed, but leave a small idle interval so
            // the diagnostic begins on a clean controller cycle boundary.
            repeat (10) @(posedge controller_clk);
            while (wb_stall) @(posedge controller_clk);

            $display("[%0t] NATIVE_TX_DEBUG: %0d contiguous writes from addr=0x%0h",
                     $realtime, DBG_BURST_WORDS, dbg_addr);
            for (dbg_word = 0; dbg_word < DBG_BURST_WORDS; dbg_word = dbg_word + 1)
                wb_push_write(dbg_addr + dbg_word, dbg_write_data[dbg_word]);
            wb_pump_all;

            $display("[%0t] NATIVE_TX_DEBUG: %0d contiguous reads from addr=0x%0h",
                     $realtime, DBG_BURST_WORDS, dbg_addr);
            for (dbg_word = 0; dbg_word < DBG_BURST_WORDS; dbg_word = dbg_word + 1) begin
                wb_push_read(dbg_addr + dbg_word, dbg_write_data[dbg_word]);
            `ifdef SIM_NATIVE_TX_DEBUG_GAPPED_READS
                wb_pump_all;
                repeat (8) @(posedge controller_clk);
            `endif
            end
        `ifndef SIM_NATIVE_TX_DEBUG_GAPPED_READS
            wb_pump_all;
        `endif

            // Preserve a few idle cycles after capture, then stop.  The VCD
            // remains active until $finish, so it covers the whole test.
            repeat (8) @(posedge controller_clk);
            if (rd_err_count == 0)
                $display("[%0t] NATIVE_TX_DEBUG: PASS", $realtime);
            else
                $display("[%0t] NATIVE_TX_DEBUG: FAIL read_errors=%0d",
                         $realtime, rd_err_count);
            $finish;
        end
    `else
        if (init_failed) begin
            $display("[%0t] FATAL: o_init_failed asserted!", $realtime);
            $finish;
        end
    `endif

    `ifdef SIM_CSR_RESET_TEST
        // =============================================================
        // CSR 0xC Reset Test â€�? exercises soft reset, auto-reset, BIST restart
        // =============================================================
        $display("[%0t] === CSR Reset Test: Phase 1 â€�? Soft Reset ===", $realtime);
        begin : csr_soft_reset_test
            reg [31:0] csr0_val, csr5_val;
            integer sr_timeout;

            // 1a. Verify init_done is high (initial calibration + BIST passed)
            if (!init_done) begin
                $display("[%0t] FAIL: init_done not asserted before soft reset test", $realtime);
                rd_err_count = rd_err_count + 1;
            end

            // 1b. Read CSR[0xC] â€�? verify auto_reset_en is 1 (fail-safe default)
            wb_dbg_read(4'hC);
            if (wb_dbg_rdata[2] !== 1'b1) begin
                $display("[%0t] FAIL: auto_reset_en not 1 at default (CSR[C]=0x%0h)", $realtime, wb_dbg_rdata);
                rd_err_count = rd_err_count + 1;
            end
            wb_dbg_idle;

            // 1c. Issue soft reset: write bit[1] of CSR 0xC
            $display("[%0t]   Writing CSR 0xC bit[1] (soft reset)", $realtime);
            wb_dbg_write(4'hC, 32'h2);
            wb_dbg_idle;
            repeat (5) @(posedge controller_clk);

            // 1d. Verify init_done drops (prober cleared status)
            if (init_done) begin
                $display("[%0t] FAIL: init_done still high after soft reset", $realtime);
                rd_err_count = rd_err_count + 1;
            end

            // 1e. Wait for controller to re-calibrate (reset_done re-asserts)
            sr_timeout = 0;
            while (!init_done && sr_timeout < 300000) begin
                @(posedge controller_clk);
                sr_timeout = sr_timeout + 1;
            end

            if (sr_timeout >= 300000) begin
                $display("[%0t] FAIL: soft reset â€�? init_done never re-asserted (timeout)", $realtime);
                rd_err_count = rd_err_count + 1;
            end else begin
                $display("[%0t]   Soft reset: init_done re-asserted after %0d cycles", $realtime, sr_timeout);
            end

            // 1f. Verify BIST passed after re-calibration
            wb_dbg_read(4'h5);
            csr5_val = wb_dbg_rdata;
            wb_dbg_idle;
            if (csr5_val[4] !== 1'b1 || csr5_val[5] !== 1'b0) begin
                $display("[%0t] FAIL: soft reset â€�? BIST did not pass after re-calib (CSR[5]=0x%0h)", $realtime, csr5_val);
                rd_err_count = rd_err_count + 1;
            end else begin
                $display("[%0t]   Soft reset: BIST re-passed after re-calibration", $realtime);
            end

            // 1g. Verify init_failed is NOT set
            if (init_failed) begin
                $display("[%0t] FAIL: init_failed asserted after successful soft reset", $realtime);
                rd_err_count = rd_err_count + 1;
            end
        end
        $display("[%0t]   Phase 1 (soft reset) complete", $realtime);

        // Phase 2: Auto-reset on BIST failure
        $display("[%0t] === CSR Reset Test: Phase 2 â€�? Auto-Reset on BIST Fail ===", $realtime);
        begin : csr_auto_reset_test
            reg [31:0] csr5_val;
            integer ar_timeout;

            // 2a. Retain enabled auto-reset and exercise the CSR write path
            $display("[%0t]   Keeping auto_reset_en enabled (CSR 0xC bit[2])", $realtime);
            wb_dbg_write(4'hC, 32'h4);
            wb_dbg_idle;
            repeat (2) @(posedge controller_clk);

            // 2b. Verify readback
            wb_dbg_read(4'hC);
            if (wb_dbg_rdata[2] !== 1'b1) begin
                $display("[%0t] FAIL: auto_reset_en not set (CSR[C]=0x%0h)", $realtime, wb_dbg_rdata);
                rd_err_count = rd_err_count + 1;
            end
            wb_dbg_idle;

            // 2c. Trigger BIST, then corrupt data only while DFI marks write
            //     data valid. This leaves the command/read paths intact.
            //     Several zeroed words guarantee a mismatch even when DRAM
            //     still contains the expected pattern from an earlier BIST.
            wb_dbg_write(4'hC, 32'h5); // bit[0]=1 (BIST start) + bit[2]=1 (keep auto_reset_en)
            wb_dbg_idle;
            wait (u_dut.u_prober.bist_state == 3'd1); // BIST_BURST_WRITE
            wait (|u_dut.dfi_wrdata_en);
            $display("[%0t]   Forcing valid DFI write data to zero", $realtime);
            force u_dut.dfi_wrdata = '0;
            repeat (8) @(posedge controller_clk);
            release u_dut.dfi_wrdata;
            $display("[%0t]   Released DFI write data corruption", $realtime);
            if (u_dut.u_prober.bist_state !== 3'd1) begin
                $display("[%0t] FAIL: write-data fault escaped the BIST write phase", $realtime);
                rd_err_count = rd_err_count + 1;
            end
            // 2d. Wait for init_done to drop (BIST finishes â†’ auto-reset fires)
            ar_timeout = 0;
            while (init_done && ar_timeout < 200000) begin
                @(posedge controller_clk);
                ar_timeout = ar_timeout + 1;
            end
            if (ar_timeout >= 200000) begin
                $display("[%0t] FAIL: auto-reset â€�? init_done never dropped", $realtime);
                rd_err_count = rd_err_count + 1;
            end else begin
                $display("[%0t]   Auto-reset triggered: init_done dropped", $realtime);
            end

            // 2g. Wait for init_done to re-assert (re-calib + BIST pass)
            ar_timeout = 0;
            while (!init_done && ar_timeout < 300000) begin
                @(posedge controller_clk);
                ar_timeout = ar_timeout + 1;
            end
            if (ar_timeout >= 300000) begin
                $display("[%0t] FAIL: auto-reset â€�? init_done never re-asserted after recovery", $realtime);
                rd_err_count = rd_err_count + 1;
            end else begin
                $display("[%0t]   Auto-reset recovery: init_done re-asserted after %0d cycles", $realtime, ar_timeout);
            end

            // 2h. Verify BIST passed
            wb_dbg_read(4'h5);
            csr5_val = wb_dbg_rdata;
            wb_dbg_idle;
            if (csr5_val[4] !== 1'b1 || csr5_val[5] !== 1'b0) begin
                $display("[%0t] FAIL: auto-reset â€�? BIST did not pass after recovery (CSR[5]=0x%0h)", $realtime, csr5_val);
                rd_err_count = rd_err_count + 1;
            end else begin
                $display("[%0t]   Auto-reset: BIST passed after recovery", $realtime);
            end

            // 2i. Verify init_failed is NOT set
            if (init_failed) begin
                $display("[%0t] FAIL: init_failed asserted after successful auto-reset recovery", $realtime);
                rd_err_count = rd_err_count + 1;
            end
        end
        $display("[%0t]   Phase 2 (auto-reset) complete", $realtime);

        // Phase 3: BIST restart without reset (verify no re-calibration)
        $display("[%0t] === CSR Reset Test: Phase 3 â€�? BIST Restart (no reset) ===", $realtime);
        begin : csr_bist_restart_test
            reg [31:0] csr5_val;
            integer br_timeout;

            // 3a. Disable auto-reset first
            wb_dbg_write(4'hC, 32'h0); // bit[2]=0
            wb_dbg_idle;

            // 3b. Trigger BIST only (bit[0])
            $display("[%0t]   Triggering BIST restart (CSR 0xC bit[0])", $realtime);
            wb_dbg_write(4'hC, 32'h1);
            wb_dbg_idle;

            // 3c. init_done should stay high (no reset occurred)
            repeat (10) @(posedge controller_clk);
            if (!init_done) begin
                $display("[%0t] FAIL: init_done dropped on BIST-only restart", $realtime);
                rd_err_count = rd_err_count + 1;
            end

            // 3d. Wait for BIST to complete
            br_timeout = 0;
            repeat (100) @(posedge controller_clk);
            wb_dbg_read(4'h5);
            while (wb_dbg_rdata[3] && br_timeout < 200000) begin
                wb_dbg_idle;
                repeat (1000) @(posedge controller_clk);
                wb_dbg_read(4'h5);
                br_timeout = br_timeout + 1000;
            end
            csr5_val = wb_dbg_rdata;
            wb_dbg_idle;

            if (br_timeout >= 200000) begin
                $display("[%0t] FAIL: BIST restart timed out", $realtime);
                rd_err_count = rd_err_count + 1;
            end else if (csr5_val[4] !== 1'b1) begin
                $display("[%0t] FAIL: BIST restart did not pass (CSR[5]=0x%0h)", $realtime, csr5_val);
                rd_err_count = rd_err_count + 1;
            end else begin
                $display("[%0t]   BIST restart: passed without re-calibration", $realtime);
            end
        end
        $display("[%0t]   Phase 3 (BIST restart) complete", $realtime);

        // Final verdict
        if (rd_err_count == 0)
            $display("[%0t] PASS: CSR reset test â€�? all phases passed", $realtime);
        else
            $display("[%0t] FAIL: CSR reset test â€�? %0d errors", $realtime, rd_err_count);

        test_phase = "DONE";
        all_tests_done = 1'b1;
    `else
        // Wait for stall to clear before starting user traffic.
        repeat (10) @(posedge controller_clk);
        while (wb_stall) @(posedge controller_clk);

        // =============================================================
        // Row-offset loop: exercises the same test patterns at three
        // different row addresses: first row (0), middle row, last row.
        // This catches bugs in row-address bit routing and edge cases
        // in the controller's bank-state tracking at address extremes.
        // =============================================================
        begin : row_loop_blk
            integer row_iter;
            reg [WB_ADDR_BITS-1:0] row_base;
            for (row_iter = 0; row_iter < 3; row_iter = row_iter + 1) begin
                case (row_iter)
                    0: row_base = FIRST_ROW_OFF;
                    1: row_base = MID_ROW_OFF;
                    2: row_base = LAST_ROW_OFF;
                endcase
                $display("[%0t] ===== ROW ITERATION %0d (row_base=0x%0h) =====",
                         $realtime, row_iter, row_base);

        // =============================================================
        // PATTERN USED IN PHASES A-H:
        //   1. Push N writes (to different bank groups / banks / rows)
        //   2. Push N reads to same addresses with expected data
        //   3. Call wb_pump_all â€�? fires ALL of them back-to-back
        //
        // The controller must handle the writeâ†’read transitions
        // internally (ACT, PRE, tWTR delays, etc.) using STALL to
        // throttle the master. We never manually wait between ops.
        // =============================================================

        // -- Phase A: Cold writes to 4 idle banks --
        // All banks are closed â†’ controller must issue ACT before each WR.
        // Tests: basic ACTâ†’WR path for 4 different bank groups.
        test_phase = "PHASE_A";
        $display("[%0t] === Phase A: Cold writes to idle banks (ACT->WR) ===", $realtime);
        wb_push_write(ROW0_BG0 + row_base, 128'h0A);
        wb_push_write(ROW0_BG1 + row_base, 128'h0B);
        wb_push_write(ROW0_BG2 + row_base, 128'h0C);
        wb_push_write(ROW0_BG3 + row_base, 128'h0D);
        wb_push_read(ROW0_BG0 + row_base, 128'h0A);
        wb_push_read(ROW0_BG1 + row_base, 128'h0B);
        wb_push_read(ROW0_BG2 + row_base, 128'h0C);
        wb_push_read(ROW0_BG3 + row_base, 128'h0D);
        wb_pump_all;

        // -- Phase B: Bank hits â€�? same bank, same row already open --
        // The row is still open from Phase A â†’ controller skips ACT,
        // issues WR directly (bank "hit"). Tests: hit path with no
        // ACT/PRE overhead â€�? maximum throughput scenario.
        test_phase = "PHASE_B";
        $display("[%0t] === Phase B: Bank hits  -  same row open (WR only) ===", $realtime);
        wb_push_write(ROW0_BG0 + row_base, 128'h1A);
        wb_push_write(ROW0_BG1 + row_base, 128'h1B);
        wb_push_write(ROW0_BG2 + row_base, 128'h1C);
        wb_push_write(ROW0_BG3 + row_base, 128'h1D);
        wb_push_read(ROW0_BG0 + row_base, 128'h1A);
        wb_push_read(ROW0_BG1 + row_base, 128'h1B);
        wb_push_read(ROW0_BG2 + row_base, 128'h1C);
        wb_push_read(ROW0_BG3 + row_base, 128'h1D);
        wb_pump_all;

        // -- Phase C: Row miss â€�? same bank, DIFFERENT row --
        // Row0 is open but we write to Row1 â†’ controller must:
        //   PRE (close Row0) â†’ ACT (open Row1) â†’ WR
        // This is the most expensive path. The controller stalls us
        // while it sequences PRE+ACT timing (tRP + tRCD).
        test_phase = "PHASE_C";
        $display("[%0t] === Phase C: Row miss  -  different row (PRE->ACT->WR) ===", $realtime);
        wb_push_write(ROW1_BG0 + row_base, 128'h2A);
        wb_push_write(ROW1_BG1 + row_base, 128'h2B);
        wb_push_write(ROW1_BG2 + row_base, 128'h2C);
        wb_push_write(ROW1_BG3 + row_base, 128'h2D);
        wb_push_read(ROW1_BG0 + row_base, 128'h2A);
        wb_push_read(ROW1_BG1 + row_base, 128'h2B);
        wb_push_read(ROW1_BG2 + row_base, 128'h2C);
        wb_push_read(ROW1_BG3 + row_base, 128'h2D);
        wb_pump_all;

        // -- Phase D: Post-refresh access --
        // We idle the bus and wait for the controller to issue a periodic
        // REF command (which closes all banks via PRE ALL). Then we write
        // again â€�? verifying that the controller correctly re-opens banks
        // after a refresh event.
        test_phase = "WAIT_REF";
        $display("[%0t] === Waiting for refresh to close all banks... ===", $realtime);
        wb_idle;
        wait (ref_count >= (row_iter + 1) * 2);
        @(posedge controller_clk);
        while (wb_stall) @(posedge controller_clk);

        test_phase = "PHASE_D";
        $display("[%0t] === Phase D: Post-refresh writes (ACT->WR) ===", $realtime);
        wb_push_write(ROW0_BG0 + row_base, 128'h3A);
        wb_push_write(ROW0_BG1 + row_base, 128'h3B);
        wb_push_write(ROW0_BG2 + row_base, 128'h3C);
        wb_push_write(ROW0_BG3 + row_base, 128'h3D);
        wb_push_read(ROW0_BG0 + row_base, 128'h3A);
        wb_push_read(ROW0_BG1 + row_base, 128'h3B);
        wb_push_read(ROW0_BG2 + row_base, 128'h3C);
        wb_push_read(ROW0_BG3 + row_base, 128'h3D);
        wb_pump_all;

        // -- Phase E: Same bank-group, different banks --
        // All 3 writes target BG0 but different bank addresses (BA0, BA1, BA2).
        // Within the SAME bank group, the controller must respect tRRD_L
        // (ACT-to-ACT) and tCCD_L (CAS-to-CAS) which are LONGER than the
        // cross-BG variants (tRRD_S / tCCD_S). Stresses the scheduler's
        // same-BG timing enforcement.
        test_phase = "PHASE_E";
        $display("[%0t] === Phase E: Same-BG different-bank writes (tRRD/tCCD_L) ===", $realtime);
        wb_push_write(ROW0_BG0 + row_base,     128'h4A);
        wb_push_write(ROW0_BG0_BA1 + row_base, 128'h4B);
        wb_push_write(ROW0_BG0_BA2 + row_base, 128'h4C);
        wb_push_read(ROW0_BG0 + row_base,     128'h4A);
        wb_push_read(ROW0_BG0_BA1 + row_base, 128'h4B);
        wb_push_read(ROW0_BG0_BA2 + row_base, 128'h4C);
        wb_pump_all;

        // -- Phase F: Mixed write+read in one burst --
        // Writes to BG0/BG1, then reads from BG0/BG1 (bank hits) and
        // BG2/BG3 (still hold Phase D data=0x3C/3D). This tests the
        // controller's ability to interleave writes and reads within a
        // single pipelined burst â€�? no idle gap between the last write
        // and the first read.
        test_phase = "PHASE_F";
        $display("[%0t] === Phase F: Read requests (ACT->RD, bank hit RD) ===", $realtime);
        wb_push_write(ROW0_BG0 + row_base, 128'h5A);
        wb_push_write(ROW0_BG1 + row_base, 128'h5B);
        wb_push_read(ROW0_BG0 + row_base, 128'h5A);
        wb_push_read(ROW0_BG1 + row_base, 128'h5B);
        wb_push_read(ROW0_BG2 + row_base, 128'h3C);  // still has Phase D data
        wb_push_read(ROW0_BG3 + row_base, 128'h3D);  // still has Phase D data
        wb_pump_all;

        // -- Phase G: Same-bank row thrashing (tRC stress) --
        // Three writes to the SAME bank (BG0/BA0) but alternating rows:
        //   Row0 â†’ Row1 â†’ Row0 (each causes PRE+ACT = row miss)
        // The minimum time between two ACTs to the same bank is tRC.
        // The controller must stall long enough to satisfy tRC.
        test_phase = "PHASE_G";
        $display("[%0t] === Phase G: Same-bank rapid re-access (tRC stress) ===", $realtime);
        wb_push_write(ROW0_BG0 + row_base, 128'h6A);  // open Row0
        wb_push_write(ROW1_BG0 + row_base, 128'h6B);  // miss: PRE Row0 â†’ ACT Row1
        wb_push_write(ROW0_BG0 + row_base, 128'h6C);  // miss: PRE Row1 â†’ ACT Row0
        wb_push_read(ROW0_BG0 + row_base, 128'h6C);   // last write wins
        wb_push_read(ROW1_BG0 + row_base, 128'h6B);
        wb_pump_all;

        // -- Phase H: Write-then-read same BG (tWTR stress) --
        // After a write, the controller must wait tWTR_L (same BG) or
        // tWTR_S (different BG) before issuing a read. We interleave
        // W-R-W-R-R-R in a single pipelined burst to stress this path.
        // The master pumps all 6 requests without gaps; the controller
        // uses STALL to enforce tWTR timing internally.
        test_phase = "PHASE_H";
        $display("[%0t] === Phase H: Write-then-read same BG (tWTR stress) ===", $realtime);
        wb_push_write(ROW0_BG0 + row_base, 128'h7A);  // WR to BG0
        wb_push_read(ROW0_BG0 + row_base, 128'h7A);   // RD from BG0 (tWTR_L applies)
        wb_push_write(ROW0_BG1 + row_base, 128'h7B);  // WR to BG1
        wb_push_read(ROW0_BG0 + row_base, 128'h7A);   // RD from BG0 (tWTR_S from BG1's WR)
        wb_push_read(ROW0_BG0 + row_base, 128'h7A);   // back-to-back RD
        wb_push_read(ROW0_BG1 + row_base, 128'h7B);   // RD from BG1
        wb_pump_all;

        // -- Phase I: Writeâ†’Read data round-trip verification --
        // Each pattern targets a different failure mode:
        //   all-F / all-0  â†’ stuck-at faults in DQ lines
        //   A5/5A          â†’ byte-lane swap (adjacent lanes swapped)
        //   DEAD_BEEF      â†’ general sanity (mixed nibbles)
        //   half-bus F/0   â†’ upper vs lower 64-bit crossbar bugs
        //   walking-1      â†’ single-bit shorts between DQ lines
        //
        // NOTE: write and read to the SAME address are queued
        // back-to-back (W0-R0-W1-R1-...). The WB B4 in-order guarantee
        // means the controller processes the write BEFORE the read,
        // so the read always returns the freshly-written value.
        test_phase = "PHASE_I";
        $display("[%0t] === Phase I: Write->Read round-trip verification ===", $realtime);
        wb_push_write(ROW2_BG0 + row_base,    128'hDEAD_BEEF_CAFE_BABE_0123_4567_89AB_CDEF);
        wb_push_read(ROW2_BG0 + row_base,     128'hDEAD_BEEF_CAFE_BABE_0123_4567_89AB_CDEF);
        wb_push_write(ROW2_BG1 + row_base,    128'hFFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF);
        wb_push_read(ROW2_BG1 + row_base,     128'hFFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF);
        wb_push_write(ROW2_BG2 + row_base,    128'h0000_0000_0000_0000_0000_0000_0000_0000);
        wb_push_read(ROW2_BG2 + row_base,     128'h0000_0000_0000_0000_0000_0000_0000_0000);
        wb_push_write(ROW2_BG3 + row_base,    128'hA5A5_A5A5_5A5A_5A5A_A5A5_A5A5_5A5A_5A5A);
        wb_push_read(ROW2_BG3 + row_base,     128'hA5A5_A5A5_5A5A_5A5A_A5A5_A5A5_5A5A_5A5A);
        wb_push_write(ROW2_BG0_C1 + row_base, 128'h0000_0000_0000_0000_FFFF_FFFF_FFFF_FFFF);
        wb_push_read(ROW2_BG0_C1 + row_base,  128'h0000_0000_0000_0000_FFFF_FFFF_FFFF_FFFF);
        wb_push_write(ROW2_BG1_C1 + row_base, 128'hFFFF_FFFF_FFFF_FFFF_0000_0000_0000_0000);
        wb_push_read(ROW2_BG1_C1 + row_base,  128'hFFFF_FFFF_FFFF_FFFF_0000_0000_0000_0000);
        wb_push_write(ROW2_BG0 + row_base,    128'h1234_5678_9ABC_DEF0_FEDC_BA98_7654_3210);
        wb_push_read(ROW2_BG0 + row_base,     128'h1234_5678_9ABC_DEF0_FEDC_BA98_7654_3210);
        wb_push_write(ROW2_BG1 + row_base,    128'h0F0F_0F0F_F0F0_F0F0_0F0F_0F0F_F0F0_F0F0);
        wb_push_read(ROW2_BG1 + row_base,     128'h0F0F_0F0F_F0F0_F0F0_0F0F_0F0F_F0F0_F0F0);
        wb_push_write(ROW2_BG2 + row_base,    128'h0000_0000_0000_0001_8000_0000_0000_0000);
        wb_push_read(ROW2_BG2 + row_base,     128'h0000_0000_0000_0001_8000_0000_0000_0000);
        wb_push_write(ROW2_BG3 + row_base,    128'hAAAA_AAAA_5555_5555_AAAA_AAAA_5555_5555);
        wb_push_read(ROW2_BG3 + row_base,     128'hAAAA_AAAA_5555_5555_AAAA_AAAA_5555_5555);
        wb_pump_all;

        if (rd_err_count == 0)
            $display("[%0t] PASS: All 10 write->read checks passed", $realtime);
        else
            $display("[%0t] FAIL: %0d of 10 write->read checks failed", $realtime, rd_err_count);

            end // for row_iter
        end // row_loop_blk

        // -- Phase J: Bulk pipelined write then bulk read --
        // Queues TB_DEPTH/2 (16) writes to consecutive addresses, then
        // 16 reads to the same addresses. All 32 transactions fire in
        // one wb_pump_all call â€�? the bus never goes idle between them.
        // gen_pattern_tb(idx) generates a deterministic 128-bit pattern
        // from the address index so we can verify each word independently.
        test_phase = "PHASE_J";
        $display("[%0t] === Phase J: Pipelined write->read (%0d addr) ===", $realtime, TB_DEPTH/2);
        begin : phase_j_blk
            integer pj_idx;
            integer pj_err;
            pj_err = 0;

            for (pj_idx = 0; pj_idx < (TB_DEPTH/2); pj_idx = pj_idx + 1) begin
                wb_push_write(27'h1000 + pj_idx, gen_pattern_tb(pj_idx[7:0]));
            end
            for (pj_idx = 0; pj_idx < (TB_DEPTH/2); pj_idx = pj_idx + 1) begin
                wb_push_read(27'h1000 + pj_idx, gen_pattern_tb(pj_idx[7:0]));
            end
            wb_pump_all;

            if (rd_err_count == pj_err)
                $display("[%0t] PASS: Phase J  -  all %0d pipelined write->read checks passed", $realtime, TB_DEPTH/2);
            else begin
                $display("[%0t] FAIL: Phase J  -  %0d of %0d checks failed", $realtime, rd_err_count - pj_err, TB_DEPTH/2);
            end
        end

        // -- Phase K: Readâ†’Write turnaround (bus contention stress) --
        // After the DQ bus is used for a read, the PHY must turn around
        // the I/O direction before driving write data. The controller
        // enforces a RDâ†’WR delay for this. We queue: WR-WR-RD-WR-WR-RD-RD
        // so the controller sees a read immediately followed by writes â€�?
        // it must stall the writes long enough for the PHY turnaround.
        test_phase = "PHASE_K";
        $display("[%0t] === Phase K: Read->Write turnaround (RD->WR delay) ===", $realtime);
        wb_push_write(ROW0_BG0, 128'hA0A0_A0A0_A0A0_A0A0_A0A0_A0A0_A0A0_A0A0);
        wb_push_write(ROW0_BG1, 128'hB0B0_B0B0_B0B0_B0B0_B0B0_B0B0_B0B0_B0B0);
        wb_push_read(ROW0_BG0, 128'hA0A0_A0A0_A0A0_A0A0_A0A0_A0A0_A0A0_A0A0);  // RD
        wb_push_write(ROW0_BG0, 128'hA1A1_A1A1_A1A1_A1A1_A1A1_A1A1_A1A1_A1A1);  // WR right after RD!
        wb_push_write(ROW0_BG1, 128'hB1B1_B1B1_B1B1_B1B1_B1B1_B1B1_B1B1_B1B1);
        wb_push_read(ROW0_BG0, 128'hA1A1_A1A1_A1A1_A1A1_A1A1_A1A1_A1A1_A1A1);  // verify new data
        wb_push_read(ROW0_BG1, 128'hB1B1_B1B1_B1B1_B1B1_B1B1_B1B1_B1B1_B1B1);
        wb_pump_all;

        // -- Phase L: Byte-lane masking (DM verification) --
        // DDR4 has a data-mask pin (DM_n) that allows per-byte write
        // protection. When wb_sel has a bit=0, the corresponding byte
        // is NOT written â€�? it retains its previous value in DRAM.
        //
        // Test approach (each sub-test):
        //   1. Write a known background value (all-A, all-C, all-F)
        //   2. Issue a MASKED write with different data + partial sel
        //   3. Read back: unmasked bytes = new data, masked bytes = old
        //
        // NOTE: We must pump the background write FIRST (separate
        // wb_pump_all) so it's committed to DRAM before the masked write
        // arrives. Otherwise the in-order pipeline would just see both
        // writes queued together and the second overwrites the first.
        //
        // Skipped for x4 devices which have no DM pin (JESD79-4D Table 28).
        test_phase = "PHASE_L";
        if (DEVICE_WIDTH == 4) begin
            $display("[%0t] === Phase L: SKIPPED (x4 has no DM pin) ===", $realtime);
        end else begin
            $display("[%0t] === Phase L: Byte-lane masking (DM verification) ===", $realtime);
        end
        if (DEVICE_WIDTH != 4) begin : phase_l_blk
            localparam [WB_ADDR_BITS-1:0] PL_ADDR0 = (5 << 10) | 0;
            localparam [WB_ADDR_BITS-1:0] PL_ADDR1 = (5 << 10) | 1;

            // Sub-test 1: write 0x55 to lower 8 bytes, upper 8 bytes keep 0xAA
            wb_push_write(PL_ADDR0, 128'hAAAA_AAAA_AAAA_AAAA_AAAA_AAAA_AAAA_AAAA);
            wb_pump_all;  // commit background
            wb_push_write_masked(PL_ADDR0, 128'h5555_5555_5555_5555_5555_5555_5555_5555, 16'h00FF);
            wb_push_read(PL_ADDR0, 128'hAAAA_AAAA_AAAA_AAAA_5555_5555_5555_5555);
            wb_pump_all;
            $display("[%0t]   Sub-test 1: sel=0x00FF (lower 8 bytes written)", $realtime);

            // Sub-test 2: write 0x33 to upper 8 bytes, lower 8 bytes keep 0xCC
            wb_push_write(PL_ADDR1, 128'hCCCC_CCCC_CCCC_CCCC_CCCC_CCCC_CCCC_CCCC);
            wb_pump_all;  // commit background
            wb_push_write_masked(PL_ADDR1, 128'h3333_3333_3333_3333_3333_3333_3333_3333, 16'hFF00);
            wb_push_read(PL_ADDR1, 128'h3333_3333_3333_3333_CCCC_CCCC_CCCC_CCCC);
            wb_pump_all;
            $display("[%0t]   Sub-test 2: sel=0xFF00 (upper 8 bytes written)", $realtime);

            // Sub-test 3: write 0x00 to ONLY byte[0], all other 15 bytes keep 0xFF
            wb_push_write(PL_ADDR0, {128{1'b1}});
            wb_pump_all;  // commit background
            wb_push_write_masked(PL_ADDR0, {128{1'b0}}, 16'h0001);
            wb_push_read(PL_ADDR0, 128'hFFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FF00);
            wb_pump_all;
            $display("[%0t]   Sub-test 3: sel=0x0001 (only byte 0 written)", $realtime);
        end

        // -- Phase M: tFAW stress (5 rapid activates) --
        // DDR4 limits the number of row-activates in a time window:
        // tFAW = "Four Activate Window" â€�? max 4 ACTs in any tFAW period.
        // We force 5 ACTs back-to-back (all banks cold after refresh)
        // to verify the controller correctly delays the 5th ACT.
        test_phase = "PHASE_M";
        $display("[%0t] === Phase M: tFAW stress (5 rapid ACTs) ===", $realtime);
        begin : phase_m_blk
            integer ref_start_m;
            wb_idle;
            ref_start_m = ref_count;
            $display("[%0t]   Waiting for refresh to close all banks...", $realtime);
            wait (ref_count > ref_start_m);  // ensure all banks are closed
            @(posedge controller_clk);
            while (wb_stall) @(posedge controller_clk);
            $display("[%0t]   5 writes to 5 cold banks:", $realtime);
            wb_push_write(ROW0_BG0,     128'hFA01);  // ACT #1
            wb_push_write(ROW0_BG1,     128'hFA02);  // ACT #2
            wb_push_write(ROW0_BG2,     128'hFA03);  // ACT #3
            wb_push_write(ROW0_BG3,     128'hFA04);  // ACT #4
            wb_push_write(ROW0_BG0_BA1, 128'hFA05);  // ACT #5 (must wait for tFAW)
            wb_push_read(ROW0_BG0,     128'hFA01);
            wb_push_read(ROW0_BG1,     128'hFA02);
            wb_push_read(ROW0_BG2,     128'hFA03);
            wb_push_read(ROW0_BG3,     128'hFA04);
            wb_push_read(ROW0_BG0_BA1, 128'hFA05);
            wb_pump_all;
        end

        // -- Phase N: Maximum pipeline depth (TB_DEPTH=32 transactions) --
        // Queues 32 writes + 32 reads = 64 total transactions in one burst.
        // This saturates the controller's internal pipeline and exercises
        // its ability to sustain back-to-back traffic at full rate.
        // Also used for throughput measurement (cycles / transaction).
        test_phase = "PHASE_N";
        $display("[%0t] === Phase N: Pipeline saturation (%0d writes) ===", $realtime, TB_DEPTH);
        begin : phase_n_blk
            integer pn_idx;
            integer pn_err_start;
            pn_err_start = rd_err_count;
            phase_n_start = cycle_count;
            phase_n_wr_start = wr_count;

            for (pn_idx = 0; pn_idx < TB_DEPTH; pn_idx = pn_idx + 1) begin
                wb_push_write(27'h2000 + pn_idx, gen_pattern_tb(pn_idx[7:0]));
            end
            for (pn_idx = 0; pn_idx < TB_DEPTH; pn_idx = pn_idx + 1) begin
                wb_push_read(27'h2000 + pn_idx, gen_pattern_tb(pn_idx[7:0]));
            end
            wb_pump_all;

            phase_n_end = cycle_count;
            if (rd_err_count == pn_err_start)
                $display("[%0t] PASS: Phase N  -  all %0d pipelined write->read checks passed", $realtime, TB_DEPTH);
            else begin
                $display("[%0t] FAIL: Phase N  -  %0d of %0d checks failed", $realtime, rd_err_count - pn_err_start, TB_DEPTH);
            end
        end

        // -- Phase O: Sustained traffic through periodic refresh --
        // Unlike other phases that use wb_pump_all (fixed queue), this
        // phase uses a CUSTOM INLINE DRIVER that keeps writing forever
        // until 2 refresh events happen. This tests that:
        //   a) The controller correctly pauses user traffic for REF
        //   b) Data written before/after/during refresh reads back OK
        //   c) The master never sees corruption from refresh interleaving
        //
        // The inline driver is hand-coded (not using wb_pump_all) because
        // we don't know in advance how many writes will be needed â€�? it
        // depends on when the refresh timer fires.
        //
        // After the write storm completes, we do 8 verification reads
        // using the standard wb_pump_all path.
        test_phase = "PHASE_O";
        $display("[%0t] === Phase O: Refresh-during-traffic ===", $realtime);
        begin : phase_o_blk
            integer ref_start_o, po_iter, po_outstanding, po_i;
            ref_start_o = ref_count;
            po_iter = 0;
            po_outstanding = 0;  // tracks how many ACKs we're still waiting for
            phase_o_start = cycle_count;
            phase_o_wr_start = wr_count;
            $display("[%0t]   Issuing continuous traffic, waiting for 2 refreshes...", $realtime);

            // Start the inline pipelined driver (same logic as wb_pump_all
            // but we generate addresses on-the-fly instead of pre-queuing).
            wb_cyc = 1'b1;
            wb_stb = 1'b1;
            wb_we  = 1'b1;
            wb_sel = {WB_SEL_BITS{1'b1}};
            wb_addr = (6 << 10) | 0;        // first address
            wb_data = gen_pattern_tb(8'd0);  // first data

            // Keep pumping writes every cycle (when !stall) until 2 REFs seen.
            // We write to 8 rotating addresses (po_iter % 8). The last value
            // written to each address is what we'll verify at the end.
            while (ref_count < ref_start_o + 2) begin
                @(posedge controller_clk);
                if (wb_ack) po_outstanding = po_outstanding - 1;
                if (!wb_stall) begin
                    po_outstanding = po_outstanding + 1;
                    po_iter = po_iter + 1;
                    // Rotate through 8 addresses across 2 bank groups
                    wb_addr = (6 << 10) | (((po_iter % 8) >> 2) << 8) | ((po_iter % 8) & 3);
                    wb_data = gen_pattern_tb(po_iter[7:0] % 8);
                end
            end
            // Done sending â€�? deassert STB, wait for remaining ACKs
            wb_stb = 1'b0;
            while (po_outstanding > 0) begin
                @(posedge controller_clk);
                if (wb_ack) po_outstanding = po_outstanding - 1;
            end
            wb_cyc = 1'b0;
            wb_we  = 1'b0;

            // Now verify: read back the 8 addresses and check against
            // the last pattern written to each (gen_pattern_tb(i)).
            $display("[%0t]   Traffic complete after %0d iterations, verifying...", $realtime, po_iter);
            for (po_i = 0; po_i < 8; po_i = po_i + 1) begin
                wb_push_read((6 << 10) | ((po_i >> 2) << 8) | (po_i & 3), gen_pattern_tb(po_i[7:0]));
            end
            wb_pump_all;
            phase_o_end = cycle_count;
            phase_o_wr_count = wr_count - phase_o_wr_start;
            $display("[%0t]   Refreshes during traffic: %0d",
                     $realtime, ref_count - ref_start_o);
        end

        // -- Phase P: Address boundary corners --
        // Writes to extreme address values to catch bit-routing bugs:
        //   - Max column upper bits + max BG
        //   - Max 10-bit address (all col+BG bits set)
        //   - Combination of row=3 / BA=3 / max col / BG=3
        test_phase = "PHASE_P";
        $display("[%0t] === Phase P: Address boundary corners ===", $realtime);
        wb_push_write((63 << 2) | 3, 128'hBEEF_0001_BEEF_0001_BEEF_0001_BEEF_0001);
        wb_push_read((63 << 2) | 3,  128'hBEEF_0001_BEEF_0001_BEEF_0001_BEEF_0001);
        wb_push_write(10'h3FF, 128'hBEEF_0002_BEEF_0002_BEEF_0002_BEEF_0002);
        wb_push_read(10'h3FF,  128'hBEEF_0002_BEEF_0002_BEEF_0002_BEEF_0002);
        wb_push_write((3 << 10) | (3 << 8) | (63 << 2) | 3,
            128'hBEEF_0003_BEEF_0003_BEEF_0003_BEEF_0003);
        wb_push_read((3 << 10) | (3 << 8) | (63 << 2) | 3,
            128'hBEEF_0003_BEEF_0003_BEEF_0003_BEEF_0003);
        wb_pump_all;
        $display("[%0t]   max col_upper + BG3", $realtime);
        $display("[%0t]   addr 1023 (max 10-bit)", $realtime);
        $display("[%0t]   row3/BA3/max_col/BG3 (addr 4095)", $realtime);

        // -- Phase Q: All 16 banks exercised in one pipelined burst --
        // DDR4 x8 has 4 bank groups Ã— 4 banks = 16 banks total.
        // We write a unique pattern to each bank (encoding BG+BA in the
        // data), then read all 16 back. All 32 transactions (16 WR + 16 RD)
        // are queued at once and fired through a single wb_pump_all.
        // This exercises the controller's full bank-state tracking and
        // measures cross-BG interleaving throughput.
        test_phase = "PHASE_Q";
        $display("[%0t] === Phase Q: Multi-BG interleaving (16 banks) ===", $realtime);
        begin : phase_q_blk
            integer pq_bg, pq_ba;
            integer pq_err_start;
            reg [WB_ADDR_BITS-1:0] pq_addr;
            reg [WB_DATA_BITS-1:0] pq_data;
            pq_err_start = rd_err_count;
            phase_q_start = cycle_count;
            phase_q_wr_start = wr_count;

            // Queue 16 writes: one per bank (BG0-3 Ã— BA0-3)
            for (pq_ba = 0; pq_ba < 4; pq_ba = pq_ba + 1) begin
                for (pq_bg = 0; pq_bg < 4; pq_bg = pq_bg + 1) begin
                    pq_addr = (7 << 10) | (pq_ba << 8) | pq_bg;
                    pq_data = {112'd0, pq_bg[3:0], pq_ba[3:0], 8'hAA};
                    wb_push_write(pq_addr, pq_data);
                end
            end
            // Queue 16 reads: same addresses, same expected data
            for (pq_ba = 0; pq_ba < 4; pq_ba = pq_ba + 1) begin
                for (pq_bg = 0; pq_bg < 4; pq_bg = pq_bg + 1) begin
                    pq_addr = (7 << 10) | (pq_ba << 8) | pq_bg;
                    pq_data = {112'd0, pq_bg[3:0], pq_ba[3:0], 8'hAA};
                    wb_push_read(pq_addr, pq_data);
                end
            end
            wb_pump_all;

            phase_q_end = cycle_count;
            if (rd_err_count == pq_err_start)
                $display("[%0t] PASS: Phase Q  -  all 16 banks verified", $realtime);
            else
                $display("[%0t] FAIL: Phase Q  -  %0d of 16 bank checks failed",
                         $realtime, rd_err_count - pq_err_start);
        end

        // =============================================================
        // PHASES R-Z: Advanced WB protocol corner cases
        //
        // These phases stress behaviors that real masters generate but
        // are NOT covered by the DDR4-timing-focused phases A-Q above.
        // A production-quality controller must handle all of these
        // without corruption, hangs, or protocol violations.
        // =============================================================

        // -- Phase R: Bursty master (STB gaps within CYC) --
        // A real CPU/DMA might assert STB for a few cycles, pause (STB=0
        // but CYC stays high), then resume. The controller must not lose
        // state or corrupt tracking when STB goes away mid-burst.
        //
        // Approach: issue writes with gaps using a custom inline driver,
        // then drain all write ACKs, then verify with wb_pump_all reads.
        test_phase = "PHASE_R";
        $display("[%0t] === Phase R: Bursty master (STB gaps within CYC) ===", $realtime);
        begin : phase_r_blk
            integer pr_err_start, pr_outstanding, pr_i;
            pr_err_start = rd_err_count;
            pr_outstanding = 0;

            wb_cyc = 1'b1;

            // Burst 1: 3 writes to addresses 0-2
            wb_stb = 1'b1; wb_we = 1'b1; wb_sel = {WB_SEL_BITS{1'b1}};
            for (pr_i = 0; pr_i < 3; pr_i = pr_i + 1) begin
                wb_addr = (8 << 10) | pr_i;
                wb_data = gen_pattern_tb(pr_i[7:0]);
                @(posedge controller_clk);
                while (wb_stall) @(posedge controller_clk);
            end

            // Gap: STB=0 for 5 cycles (CYC stays high â€�? this is the key test!)
            wb_stb = 1'b0;
            repeat (5) @(posedge controller_clk);

            // Burst 2: 3 more writes to addresses 3-5
            wb_stb = 1'b1;
            for (pr_i = 3; pr_i < 6; pr_i = pr_i + 1) begin
                wb_addr = (8 << 10) | pr_i;
                wb_data = gen_pattern_tb(pr_i[7:0]);
                @(posedge controller_clk);
                while (wb_stall) @(posedge controller_clk);
            end

            // Gap: STB=0 for 8 cycles
            wb_stb = 1'b0;
            repeat (8) @(posedge controller_clk);

            // Burst 3: 2 more writes to addresses 6-7
            wb_stb = 1'b1;
            for (pr_i = 6; pr_i < 8; pr_i = pr_i + 1) begin
                wb_addr = (8 << 10) | pr_i;
                wb_data = gen_pattern_tb(pr_i[7:0]);
                @(posedge controller_clk);
                while (wb_stall) @(posedge controller_clk);
            end
            wb_stb = 1'b0;

            // Wait for all 8 write ACKs to flush through the pipeline.
            // 40 cycles is generous (ack_pipe depth is ~20 max).
            repeat (40) @(posedge controller_clk);
            wb_cyc = 1'b0; wb_we = 1'b0;

            // Now verify all 8 writes using the proven wb_pump_all path
            for (pr_i = 0; pr_i < 8; pr_i = pr_i + 1) begin
                wb_push_read((8 << 10) | pr_i, gen_pattern_tb(pr_i[7:0]));
            end
            wb_pump_all;

            if (rd_err_count == pr_err_start)
                $display("[%0t] PASS: Phase R  -  bursty master (3+gap+3+gap+2 writes verified)", $realtime);
            else
                $display("[%0t] FAIL: Phase R  -  %0d errors", $realtime, rd_err_count - pr_err_start);
        end

        // -- Phase S: Sustained alternating W-R-W-R (64 pairs) --
        // Mimics a DMA engine doing read-modify-write. Each cycle
        // (when not stalled) alternates between write and read to the
        // SAME address â€�? the read must always return what was just written.
        // This stresses tWTR/tRTW turnaround at maximum rate.
        test_phase = "PHASE_S";
        $display("[%0t] === Phase S: Sustained alternating W-R (64 pairs) ===", $realtime);
        begin : phase_s_blk
            integer ps_err_start, ps_i;
            ps_err_start = rd_err_count;

            for (ps_i = 0; ps_i < 64; ps_i = ps_i + 1) begin
                wb_push_write((9 << 10) | (ps_i & 3), gen_pattern_tb(ps_i[7:0]));
                wb_push_read((9 << 10) | (ps_i & 3), gen_pattern_tb(ps_i[7:0]));
            end
            wb_pump_all;

            if (rd_err_count == ps_err_start)
                $display("[%0t] PASS: Phase S  -  all 64 W-R pairs verified", $realtime);
            else
                $display("[%0t] FAIL: Phase S  -  %0d errors", $realtime, rd_err_count - ps_err_start);
        end

        // -- Phase T: Sequential burst crossing row boundary --
        // Writes to consecutive addresses that span from the end of one
        // row into the start of the next row. The controller must detect
        // the row change and issue PRE+ACT automatically. A single
        // pipelined burst of 16 writes + 16 reads with no manual gaps.
        test_phase = "PHASE_T";
        $display("[%0t] === Phase T: Row-crossing sequential burst ===", $realtime);
        begin : phase_t_blk
            integer pt_err_start, pt_i;
            reg [WB_ADDR_BITS-1:0] pt_base;
            pt_err_start = rd_err_count;
            // Start 8 addresses before row boundary (row 4â†’5 transition)
            // ROW_SHIFT puts the row field at the top. Address just below row5:
            pt_base = (4 << ROW_SHIFT) | ((1 << (COL_BITS - COL_LOW)) - 8) << COL_SHIFT;

            for (pt_i = 0; pt_i < 16; pt_i = pt_i + 1) begin
                wb_push_write(pt_base + (pt_i << COL_SHIFT), gen_pattern_tb(pt_i[7:0] + 8'hC0));
            end
            for (pt_i = 0; pt_i < 16; pt_i = pt_i + 1) begin
                wb_push_read(pt_base + (pt_i << COL_SHIFT), gen_pattern_tb(pt_i[7:0] + 8'hC0));
            end
            wb_pump_all;

            if (rd_err_count == pt_err_start)
                $display("[%0t] PASS: Phase T  -  row-crossing burst verified", $realtime);
            else
                $display("[%0t] FAIL: Phase T  -  %0d errors", $realtime, rd_err_count - pt_err_start);
        end

        // -- Phase U: LFSR random address stress (128 transactions) --
        // Generates pseudo-random addresses using an LFSR, producing
        // unpredictable bank/row access patterns. This catches bugs that
        // only appear under non-deterministic traffic (e.g., bank-state
        // machine transitions that never occur with structured patterns).
        test_phase = "PHASE_U";
        $display("[%0t] === Phase U: LFSR random address stress (128 txns) ===", $realtime);
        begin : phase_u_blk
            integer pu_err_start, pu_i;
            reg [15:0] lfsr;
            reg [WB_ADDR_BITS-1:0] pu_addr;
            pu_err_start = rd_err_count;
            lfsr = 16'hACE1;  // non-zero seed

            // Write 128 random addresses
            for (pu_i = 0; pu_i < 128; pu_i = pu_i + 1) begin
                // LFSR: x^16 + x^14 + x^13 + x^11 + 1 (maximal length)
                lfsr = {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
                pu_addr = {{(WB_ADDR_BITS-16){1'b0}}, lfsr};
                wb_push_write(pu_addr, {8{lfsr, ~lfsr}});
            end
            wb_pump_all;

            // Re-seed and read back same sequence
            lfsr = 16'hACE1;
            for (pu_i = 0; pu_i < 128; pu_i = pu_i + 1) begin
                lfsr = {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
                pu_addr = {{(WB_ADDR_BITS-16){1'b0}}, lfsr};
                wb_push_read(pu_addr, {8{lfsr, ~lfsr}});
            end
            wb_pump_all;

            if (rd_err_count == pu_err_start)
                $display("[%0t] PASS: Phase U  -  all 128 random addresses verified", $realtime);
            else
                $display("[%0t] FAIL: Phase U  -  %0d errors", $realtime, rd_err_count - pu_err_start);
        end

        // -- Phase V: Same-address overwrite stress --
        // Writes the same address 16 times with different data, then reads
        // once. Only the LAST write should be visible. Tests that the
        // controller properly handles repeated bank-hits to the same column
        // without data corruption from the pipeline.
        test_phase = "PHASE_V";
        $display("[%0t] === Phase V: Same-address overwrite (16 writes, 1 read) ===", $realtime);
        begin : phase_v_blk
            integer pv_err_start, pv_i;
            reg [WB_ADDR_BITS-1:0] pv_addr;
            pv_err_start = rd_err_count;
            pv_addr = (10 << 10) | 2;  // arbitrary address

            for (pv_i = 0; pv_i < 16; pv_i = pv_i + 1) begin
                wb_push_write(pv_addr, gen_pattern_tb(pv_i[7:0] + 8'hD0));
            end
            // Only the last write's data should survive
            wb_push_read(pv_addr, gen_pattern_tb(8'hDF));  // 0xD0 + 15 = 0xDF
            wb_pump_all;

            if (rd_err_count == pv_err_start)
                $display("[%0t] PASS: Phase V  -  last write wins", $realtime);
            else
                $display("[%0t] FAIL: Phase V  -  %0d errors", $realtime, rd_err_count - pv_err_start);
        end

        // -- Phase W: Single-beat transactions --
        // Real masters sometimes do isolated single-beat accesses (one
        // write or one read, then CYC drops). Tests that the controller
        // handles the minimum-size burst correctly. Each wb_pump_all
        // below is a single-transaction burst.
        test_phase = "PHASE_W";
        $display("[%0t] === Phase W: Single-beat transactions ===", $realtime);
        begin : phase_w_blk
            integer pw_err_start;
            pw_err_start = rd_err_count;

            // Single write
            wb_push_write((11 << 10) | 0, 128'hCAFE_0001_CAFE_0001_CAFE_0001_CAFE_0001);
            wb_pump_all;

            // Single read (verify)
            wb_push_read((11 << 10) | 0, 128'hCAFE_0001_CAFE_0001_CAFE_0001_CAFE_0001);
            wb_pump_all;

            // Single write to different address
            wb_push_write((11 << 10) | 1, 128'hBEEF_0002_BEEF_0002_BEEF_0002_BEEF_0002);
            wb_pump_all;

            // Single read to different address
            wb_push_read((11 << 10) | 1, 128'hBEEF_0002_BEEF_0002_BEEF_0002_BEEF_0002);
            wb_pump_all;

            // Verify first address still intact
            wb_push_read((11 << 10) | 0, 128'hCAFE_0001_CAFE_0001_CAFE_0001_CAFE_0001);
            wb_pump_all;

            if (rd_err_count == pw_err_start)
                $display("[%0t] PASS: Phase W  -  single-beat transactions OK", $realtime);
            else
                $display("[%0t] FAIL: Phase W  -  %0d errors", $realtime, rd_err_count - pw_err_start);
        end

        // -- Phase X: Back-to-back CYC cycles (no idle between bursts) --
        // After wb_pump_all drops CYC, immediately start the next burst.
        // Tests that the slave correctly resets between bus cycles without
        // needing idle time. 4 rapid burst pairs (write burst â†’ read burst).
        test_phase = "PHASE_X";
        $display("[%0t] === Phase X: Back-to-back CYC cycles ===", $realtime);
        begin : phase_x_blk
            integer px_err_start, px_i;
            px_err_start = rd_err_count;

            for (px_i = 0; px_i < 4; px_i = px_i + 1) begin
                // Write burst (4 addrs)
                wb_push_write((12 << 10) | (px_i * 4) + 0, gen_pattern_tb(px_i[7:0]*4));
                wb_push_write((12 << 10) | (px_i * 4) + 1, gen_pattern_tb(px_i[7:0]*4+1));
                wb_push_write((12 << 10) | (px_i * 4) + 2, gen_pattern_tb(px_i[7:0]*4+2));
                wb_push_write((12 << 10) | (px_i * 4) + 3, gen_pattern_tb(px_i[7:0]*4+3));
                wb_pump_all;  // CYC drops here
                // Immediately start read burst (no idle cycles inserted)
                wb_push_read((12 << 10) | (px_i * 4) + 0, gen_pattern_tb(px_i[7:0]*4));
                wb_push_read((12 << 10) | (px_i * 4) + 1, gen_pattern_tb(px_i[7:0]*4+1));
                wb_push_read((12 << 10) | (px_i * 4) + 2, gen_pattern_tb(px_i[7:0]*4+2));
                wb_push_read((12 << 10) | (px_i * 4) + 3, gen_pattern_tb(px_i[7:0]*4+3));
                wb_pump_all;  // CYC drops, next iteration immediately re-asserts
            end

            if (rd_err_count == px_err_start)
                $display("[%0t] PASS: Phase X  -  back-to-back CYC cycles OK", $realtime);
            else
                $display("[%0t] FAIL: Phase X  -  %0d errors", $realtime, rd_err_count - px_err_start);
        end

        // -- Phase Y: Maximum outstanding under heavy stall --
        // Write to the SAME bank/row 32 times in rapid succession.
        // Since all target the same bank, the controller can issue one
        // ACT then stream 32 WRs without PRE â€�? but internal pipeline
        // depth is limited. This pushes the controller's ack_pipe to
        // its maximum occupancy and verifies no data is dropped.
        test_phase = "PHASE_Y";
        $display("[%0t] === Phase Y: Same-bank pipeline saturation (32 hits) ===", $realtime);
        begin : phase_y_blk
            integer py_err_start, py_i;
            py_err_start = rd_err_count;

            // 32 writes to same bank (BG0/BA0), same row, different columns
            for (py_i = 0; py_i < 32; py_i = py_i + 1) begin
                wb_push_write((13 << ROW_SHIFT) | (py_i << COL_SHIFT), gen_pattern_tb(py_i[7:0] + 8'hE0));
            end
            // 32 reads to verify
            for (py_i = 0; py_i < 32; py_i = py_i + 1) begin
                wb_push_read((13 << ROW_SHIFT) | (py_i << COL_SHIFT), gen_pattern_tb(py_i[7:0] + 8'hE0));
            end
            wb_pump_all;

            if (rd_err_count == py_err_start)
                $display("[%0t] PASS: Phase Y  -  same-bank 32 hits OK", $realtime);
            else
                $display("[%0t] FAIL: Phase Y  -  %0d errors", $realtime, rd_err_count - py_err_start);
        end

        // -- Phase Z: Read-after-Read repetition + Write-Read-Write-Read same addr --
        // Tests: (a) multiple reads to same address return same data,
        //        (b) write-read-write-read to same address always returns
        //        the most recent write's value.
        test_phase = "PHASE_Z";
        $display("[%0t] === Phase Z: Read-repeat + W-R-W-R same address ===", $realtime);
        begin : phase_z_blk
            integer pz_err_start;
            pz_err_start = rd_err_count;

            // Part 1: Write once, read 8 times (all must match)
            wb_push_write((14 << 10) | 0, 128'h1111_2222_3333_4444_5555_6666_7777_8888);
            wb_push_read((14 << 10) | 0, 128'h1111_2222_3333_4444_5555_6666_7777_8888);
            wb_push_read((14 << 10) | 0, 128'h1111_2222_3333_4444_5555_6666_7777_8888);
            wb_push_read((14 << 10) | 0, 128'h1111_2222_3333_4444_5555_6666_7777_8888);
            wb_push_read((14 << 10) | 0, 128'h1111_2222_3333_4444_5555_6666_7777_8888);
            wb_push_read((14 << 10) | 0, 128'h1111_2222_3333_4444_5555_6666_7777_8888);
            wb_push_read((14 << 10) | 0, 128'h1111_2222_3333_4444_5555_6666_7777_8888);
            wb_push_read((14 << 10) | 0, 128'h1111_2222_3333_4444_5555_6666_7777_8888);
            wb_push_read((14 << 10) | 0, 128'h1111_2222_3333_4444_5555_6666_7777_8888);
            wb_pump_all;

            // Part 2: W-R-W-R-W-R to same address (each read sees latest write)
            wb_push_write((14 << 10) | 1, 128'hAAAA_AAAA_AAAA_AAAA_AAAA_AAAA_AAAA_AAAA);
            wb_push_read((14 << 10) | 1, 128'hAAAA_AAAA_AAAA_AAAA_AAAA_AAAA_AAAA_AAAA);
            wb_push_write((14 << 10) | 1, 128'hBBBB_BBBB_BBBB_BBBB_BBBB_BBBB_BBBB_BBBB);
            wb_push_read((14 << 10) | 1, 128'hBBBB_BBBB_BBBB_BBBB_BBBB_BBBB_BBBB_BBBB);
            wb_push_write((14 << 10) | 1, 128'hCCCC_CCCC_CCCC_CCCC_CCCC_CCCC_CCCC_CCCC);
            wb_push_read((14 << 10) | 1, 128'hCCCC_CCCC_CCCC_CCCC_CCCC_CCCC_CCCC_CCCC);
            wb_pump_all;

            // Part 3: Verify first address wasn't corrupted by part 2
            wb_push_read((14 << 10) | 0, 128'h1111_2222_3333_4444_5555_6666_7777_8888);
            wb_pump_all;

            if (rd_err_count == pz_err_start)
                $display("[%0t] PASS: Phase Z  -  read-repeat + W-R-W-R OK", $realtime);
            else
                $display("[%0t] FAIL: Phase Z  -  %0d errors", $realtime, rd_err_count - pz_err_start);
        end

        // -- CSR Read Test: read debug registers 0x0-0xC + value verification --
        test_phase = "CSR";
        $display("[%0t] === CSR Read Test: registers 0x0-0xC ===", $realtime);
        begin : csr_read_block
            integer csr_idx;
            reg [31:0] csr_vals [0:12];
            for (csr_idx = 0; csr_idx < 13; csr_idx = csr_idx + 1) begin
                wb_dbg_read(csr_idx[3:0]);
                csr_vals[csr_idx] = wb_dbg_rdata;
                wb_dbg_idle;
                $display("[%0t]   CSR[0x%0h] = 0x%08h", $realtime, csr_idx, csr_vals[csr_idx]);
                repeat (2) @(posedge controller_clk);
            end

            begin
                reg [31:0] exp_config;
                exp_config  = {24'd0, BYTE_LANES[3:0], 2'd0, TB_BIST_MODE[1:0]};
                if (csr_vals[3] === 32'd0) begin
                    $display("[%0t]   CSR FAIL: CSR[3] correct_count=0 (expected > 0)",
                             $realtime);
                    rd_err_count = rd_err_count + 1;
                end
                if (csr_vals[4] !== 32'd0) begin
                    $display("[%0t]   CSR FAIL: CSR[4] error_count=%0d, expected 0",
                             $realtime, csr_vals[4]);
                    rd_err_count = rd_err_count + 1;
                end
                if (csr_vals[5][4] !== 1'b1 || csr_vals[5][5] !== 1'b0 || csr_vals[5][3] !== 1'b0) begin
                    $display("[%0t]   CSR FAIL: CSR[5] status=0x%0h (expected pass=1, fail=0, busy=0)",
                             $realtime, csr_vals[5]);
                    rd_err_count = rd_err_count + 1;
                end
                if (csr_vals[10] !== exp_config) begin
                    $display("[%0t]   CSR FAIL: CSR[0xA] config=0x%0h, expected 0x%0h",
                             $realtime, csr_vals[10], exp_config);
                    rd_err_count = rd_err_count + 1;
                end
            end
            if (csr_vals[11] !== 32'h0001) begin
                $display("[%0t]   CSR FAIL: CSR[0xB] version=0x%0h, expected 0x0001",
                         $realtime, csr_vals[11]);
                rd_err_count = rd_err_count + 1;
            end
        end
        $display("[%0t] CSR read test complete", $realtime);

        // -- BIST Re-trigger Test (via CSR 0xC write) --
        // Writing bit 0 of CSR 0xC kicks off a fresh BIST run.
        // We poll CSR[5].busy until it clears, then check CSR[5].pass.
        // This proves the BIST can be re-triggered from software after
        // the automatic post-init run has already completed.
        test_phase = "RETRIG";
        $display("[%0t] === BIST Re-trigger Test (via CSR 0xC) ===", $realtime);
        wb_dbg_write(4'hC, 32'd1);
        wb_dbg_idle;
        repeat (40) @(posedge controller_clk);
        begin : retrig_block
            integer retrig_timeout;
            reg [31:0] csr5_val;
            retrig_timeout = 0;
            repeat (100) @(posedge controller_clk);
            csr5_val = 32'd0;
            while (retrig_timeout < 200000) begin
                wb_dbg_read(4'h5);
                csr5_val = wb_dbg_rdata;
                wb_dbg_idle;
                if (!csr5_val[3]) break;
                repeat (1000) @(posedge controller_clk);
                retrig_timeout = retrig_timeout + 1000;
            end
            if (retrig_timeout >= 200000) begin
                $display("[%0t] FAIL: BIST re-trigger timed out", $realtime);
                rd_err_count = rd_err_count + 1;
            end else if (csr5_val[4]) begin
                $display("[%0t]   BIST re-trigger PASS (via CSR)", $realtime);
            end else begin
                $display("[%0t]   BIST re-trigger FAIL (via CSR)", $realtime);
                rd_err_count = rd_err_count + 1;
            end
        end

        // -- Post-retrigger CSR value verification --
        test_phase = "CSR2";
        $display("[%0t] === Post-retrigger CSR verification ===", $realtime);
        begin : csr_post_retrig
            reg [31:0] cv3, cv4, cv5;

            wb_dbg_read(4'h3);
            cv3 = wb_dbg_rdata;
            wb_dbg_idle;
            repeat (2) @(posedge controller_clk);

            wb_dbg_read(4'h4);
            cv4 = wb_dbg_rdata;
            wb_dbg_idle;
            repeat (2) @(posedge controller_clk);

            wb_dbg_read(4'h5);
            cv5 = wb_dbg_rdata;
            wb_dbg_idle;
            repeat (2) @(posedge controller_clk);

            $display("[%0t]   Post-retrig CSR[3]=%0d CSR[4]=%0d CSR[5]=0x%0h",
                     $realtime, cv3, cv4, cv5);
            begin
                if (cv3 === 32'd0) begin
                    $display("[%0t]   CSR2 FAIL: correct_count=0 (expected > 0)", $realtime);
                    rd_err_count = rd_err_count + 1;
                end
            end
            if (cv4 !== 32'd0) begin
                $display("[%0t]   CSR2 FAIL: error_count=%0d, expected 0", $realtime, cv4);
                rd_err_count = rd_err_count + 1;
            end
            if (cv5[4] !== 1'b1 || cv5[5] !== 1'b0) begin
                $display("[%0t]   CSR2 FAIL: status=0x%0h (expected pass=1, fail=0)", $realtime, cv5);
                rd_err_count = rd_err_count + 1;
            end
        end

        test_phase = "DONE";
        $display("[%0t] === All test phases complete ===", $realtime);
        all_tests_done = 1'b1;
    `endif
    end

    // ===================================================================
    // Command Monitor
    //
    // Decodes DFI-level commands (ACT, WR, RD, PRE) from the controller's
    // 4-phase DFI bus. Each controller_clk cycle carries 4 DFI "slots"
    // (one per DDR4 CK), so we iterate over all 4 phases and count each
    // command type. Only active during the WB test phases (not during
    // BIST or before reset_done) to keep the log readable.
    //
    // The running totals (act_count, wr_count, rd_count, pre_count) are
    // printed in the final SUMMARY line at end of simulation.
    // ===================================================================
    wire [3:0] mon_cs_n  = u_dut.u_controller.o_dfi_cs_n;
    wire [3:0] mon_act_n = u_dut.u_controller.o_dfi_act_n;
    wire [3:0] mon_ras_n = u_dut.u_controller.o_dfi_ras_n;
    wire [3:0] mon_cas_n = u_dut.u_controller.o_dfi_cas_n;
    wire [3:0] mon_we_n  = u_dut.u_controller.o_dfi_we_n;
    wire [4*BG_BITS-1:0] mon_bg   = u_dut.u_controller.o_dfi_bg;
    wire [4*BA_BITS-1:0] mon_bank = u_dut.u_controller.o_dfi_bank;

    wire [3:0] mon_is_act = ~mon_cs_n & ~mon_act_n;
    wire [3:0] mon_is_wr  = ~mon_cs_n & mon_act_n & mon_ras_n & ~mon_cas_n & ~mon_we_n;
    wire [3:0] mon_is_rd  = ~mon_cs_n & mon_act_n & mon_ras_n & ~mon_cas_n & mon_we_n;
    wire [3:0] mon_is_pre = ~mon_cs_n & mon_act_n & ~mon_ras_n & mon_cas_n & ~mon_we_n;

    integer mon_ph;



    always @(posedge controller_clk) begin
        if (reset_done_seen && !all_tests_done && !bist_busy_int) begin
            for (mon_ph = 0; mon_ph < 4; mon_ph = mon_ph + 1) begin
                if (mon_is_act[mon_ph]) begin
                    act_count = act_count + 1;
                `ifndef SIM_QUIET_COMMAND_LOG
                    $display("[%0t] [%0s] DDR4 ACT #%0d: BG=%0d BA=%0d (slot %0d)",
                             $realtime, test_phase, act_count,
                             mon_bg[mon_ph*BG_BITS +: BG_BITS],
                             mon_bank[mon_ph*BA_BITS +: BA_BITS], mon_ph);
                `endif
                end
                if (mon_is_wr[mon_ph]) begin
                    wr_count = wr_count + 1;
                `ifndef SIM_QUIET_COMMAND_LOG
                    $display("[%0t] [%0s] DDR4 WR  #%0d: BG=%0d BA=%0d (slot %0d)",
                             $realtime, test_phase, wr_count,
                             mon_bg[mon_ph*BG_BITS +: BG_BITS],
                             mon_bank[mon_ph*BA_BITS +: BA_BITS], mon_ph);
                `endif
                end
                if (mon_is_rd[mon_ph]) begin
                    rd_count = rd_count + 1;
                `ifndef SIM_QUIET_COMMAND_LOG
                    $display("[%0t] [%0s] DDR4 RD  #%0d: BG=%0d BA=%0d (slot %0d) stage2_we=%0b",
                             $realtime, test_phase, rd_count,
                             mon_bg[mon_ph*BG_BITS +: BG_BITS],
                             mon_bank[mon_ph*BA_BITS +: BA_BITS], mon_ph,
                             u_dut.u_controller.stage2_we);
                `endif
                end
                if (mon_is_pre[mon_ph]) begin
                    pre_count = pre_count + 1;
                `ifndef SIM_QUIET_COMMAND_LOG
                    $display("[%0t] [%0s] DDR4 PRE #%0d: BG=%0d BA=%0d (slot %0d)",
                             $realtime, test_phase, pre_count,
                             mon_bg[mon_ph*BG_BITS +: BG_BITS],
                             mon_bank[mon_ph*BA_BITS +: BA_BITS], mon_ph);
                `endif
                end
            end
        end
    end

    // ===================================================================
    // REF Monitor
    // ===================================================================
    realtime ref_time_prev;
    realtime ref_time_curr;
    initial ref_time_prev = 0;
    initial ref_time_curr = 0;

    always @(posedge controller_clk) begin
        if (reset_done_seen && dbg_dfi_cmd == "REF") begin
            ref_time_prev = ref_time_curr;
            ref_time_curr = $realtime;
            ref_count = ref_count + 1;
            if (ref_count >= 2) begin
                $display("[%0t] REF #%0d  -  interval = %0t (tREFI = 7800.000ns)",
                         $realtime, ref_count, ref_time_curr - ref_time_prev);
            end else begin
                $display("[%0t] REF #%0d", $realtime, ref_count);
            end
        end
    end

    // ===================================================================
    // End-of-test
    //
    // After all WB phases + CSR checks + BIST re-trigger complete, we
    // wait for a few more refresh cycles to confirm the controller stays
    // healthy in idle. Then print a summary with cumulative command
    // counts and a final PASS/FAIL verdict. The 500us watchdog below
    // catches any hang.
    // ===================================================================
    localparam NUM_REFRESH_CYCLES = 3;

    initial begin
        wait (all_tests_done);
        wait (ref_count >= NUM_REFRESH_CYCLES);
        repeat (10) @(posedge controller_clk);
        $display("");
        $display("[%0t] ===============================================", $realtime);
        $display("[%0t] SUMMARY: ACT=%0d  WR=%0d  RD=%0d  PRE=%0d  REF=%0d  RD_ERR=%0d  BIST_ERR=%0d",
                 $realtime, act_count, wr_count, rd_count, pre_count, ref_count, rd_err_count, bist_error_int);
        $display("[%0t]   Note: WR >> RD because Phase O is a sustained write-bandwidth test", $realtime);
        $display("[%0t]   (%0d burst writes through 2 refresh intervals, only 8 verify reads)", $realtime, phase_o_wr_count);
        $display("[%0t]", $realtime);
        begin : summary_bw_block
            real clk_ns, peak_bw, pn_ns_per_txn, po_ns_per_wr, pq_ns_per_txn;
            real pn_bw, po_bw, pq_bw;
            integer pn_cycles, po_cycles, pq_cycles;
            integer pn_txns, pq_txns;
            clk_ns = CTRL_CLK_PERIOD / 1000.0;
            peak_bw = (WB_DATA_BITS / 8.0) / (clk_ns / 1000.0);
            pn_cycles = phase_n_end - phase_n_start;
            po_cycles = phase_o_end - phase_o_start;
            pq_cycles = phase_q_end - phase_q_start;
            pn_txns = TB_DEPTH * 2;
            pq_txns = 32;
            pn_ns_per_txn = (pn_cycles * clk_ns) / pn_txns;
            po_ns_per_wr  = (po_cycles * clk_ns) / phase_o_wr_count;
            pq_ns_per_txn = (pq_cycles * clk_ns) / pq_txns;
            pn_bw = (pn_txns * (WB_DATA_BITS / 8.0)) / (pn_cycles * clk_ns / 1000.0);
            po_bw = (phase_o_wr_count * (WB_DATA_BITS / 8.0)) / (po_cycles * clk_ns / 1000.0);
            pq_bw = (pq_txns * (WB_DATA_BITS / 8.0)) / (pq_cycles * clk_ns / 1000.0);
            $display("[%0t] THROUGHPUT (ctrl_clk = %0.2f ns, DDR4-%0d, peak BW = %0.0f MB/s):",
                     $realtime, clk_ns, 2000000 / DDR4_CLK_PERIOD, peak_bw);
            $display("[%0t]   Phase N  %0d WR + %0d RD sequential:  %0d cycles = %0.1f ns/txn,  %0.0f MB/s (%0.1f%% eff)",
                     $realtime, TB_DEPTH, TB_DEPTH, pn_cycles, pn_ns_per_txn, pn_bw, pn_bw * 100.0 / peak_bw);
            $display("[%0t]   Phase O  %0d WR sustained (2 REFs):   %0d cycles = %0.1f ns/WR,   %0.0f MB/s (%0.1f%% eff)",
                     $realtime, phase_o_wr_count, po_cycles, po_ns_per_wr, po_bw, po_bw * 100.0 / peak_bw);
            $display("[%0t]   Phase Q  16 WR + 16 RD (all banks):   %0d cycles = %0.1f ns/txn,  %0.0f MB/s (%0.1f%% eff)",
                     $realtime, pq_cycles, pq_ns_per_txn, pq_bw, pq_bw * 100.0 / peak_bw);
        end
        $display("[%0t]", $realtime);
        if (rd_err_count == 0 && bist_error_int == 0)
            $display("[%0t] PASS: All test phases + BIST + %0d refresh cycles, zero violations",
                     $realtime, ref_count);
        else
            $display("[%0t] FAIL: rd_err=%0d bist_err=%0d",
                     $realtime, rd_err_count, bist_error_int);
        $display("[%0t] ===============================================", $realtime);

        // --- CSR Register Dump ---
        $display("[%0t] === CSR Register Dump ===", $realtime);
        begin
            reg [31:0] csr [0:13];
            integer ci;
            for (ci = 0; ci <= 13; ci = ci + 1) begin
                wb_dbg_read(ci[3:0]);
                csr[ci] = wb_dbg_rdata;
            end
            wb_dbg_idle;

            // 0x0: STATUS
            $display("[%0t]   PHY FSM State        : %0d", $realtime, csr[0][3:0]);
            $display("[%0t]   Calib State          : %0d", $realtime, csr[0][7:4]);
            $display("[%0t]   Stage1 Pending       : %0b", $realtime, csr[0][8]);
            $display("[%0t]   Stage2 Pending       : %0b", $realtime, csr[0][9]);
            $display("[%0t]   Stage2 WE            : %0b", $realtime, csr[0][11]);
            $display("[%0t]   Refresh Idle         : %0b", $realtime, csr[0][12]);
            // 0x1: BANK_STATUS
            $display("[%0t]   Bank Status          : %016b", $realtime, csr[1][15:0]);
            // 0x2: TRAIN_FAIL
            $display("[%0t]   Train Fail (gate)    : %0b", $realtime, csr[2][BYTE_LANES-1:0]);
            $display("[%0t]   Train Fail (eye)     : %0b", $realtime, csr[2][2*BYTE_LANES-1:BYTE_LANES]);
            $display("[%0t]   Train Fail (wl)      : %0b", $realtime, csr[2][3*BYTE_LANES-1:2*BYTE_LANES]);
            $display("[%0t]   Calib Retry Count    : %0d", $realtime, csr[2][3*BYTE_LANES+1:3*BYTE_LANES]);
            // 0x3/0x4: BIST counts
            $display("[%0t]   BIST Correct Count   : %0d", $realtime, csr[3]);
            $display("[%0t]   BIST Error Count     : %0d", $realtime, csr[4]);
            // 0x5: BIST_STATUS
            $display("[%0t]   BIST FSM State       : %0d", $realtime, csr[5][2:0]);
            $display("[%0t]   BIST Busy            : %0b", $realtime, csr[5][3]);
            $display("[%0t]   BIST Pass            : %0b", $realtime, csr[5][4]);
            $display("[%0t]   BIST Fail Sticky     : %0b", $realtime, csr[5][5]);
            $display("[%0t]   Init Done            : %0b", $realtime, csr[5][6]);
            $display("[%0t]   Init Failed          : %0b", $realtime, csr[5][7]);
            // 0x6: LANE0_TRAINING
            $display("[%0t]   Lane 0 IDELAY Center : %0d", $realtime, csr[6][8:0]);
            $display("[%0t]   Lane 0 WL DQS Tap    : %0d", $realtime, csr[6][17:9]);
            $display("[%0t]   Lane 0 Bitslip       : %0d", $realtime, csr[6][21:18]);
            $display("[%0t]   Lane 0 Eye Start     : %0d", $realtime, csr[6][30:22]);
            // 0x7: LANE1_TRAINING
            if (BYTE_LANES > 1) begin
                $display("[%0t]   Lane 1 IDELAY Center : %0d", $realtime, csr[7][8:0]);
                $display("[%0t]   Lane 1 WL DQS Tap    : %0d", $realtime, csr[7][17:9]);
                $display("[%0t]   Lane 1 Bitslip       : %0d", $realtime, csr[7][21:18]);
                $display("[%0t]   Lane 1 Eye Start     : %0d", $realtime, csr[7][30:22]);
            end
            // 0x8: EYE_HEALTH
            $display("[%0t]   Lane 0 Eye Width     : %0d", $realtime, csr[8][8:0]);
            $display("[%0t]   Lane 1 Eye Width     : %0d", $realtime, csr[8][17:9]);
            $display("[%0t]   Rd Lat Extra         : %02b", $realtime, csr[8][19:18]);
            $display("[%0t]   EN_VTC               : %0b", $realtime, csr[8][20]);
            // 0x9: WRITE_PATH
            $display("[%0t]   Lane 0 WL DQ Tap     : %0d", $realtime, csr[9][7:0]);
            $display("[%0t]   Lane 1 WL DQ Tap     : %0d", $realtime, csr[9][15:8]);
            $display("[%0t]   Lane 0 DQS Init Tap  : %0d", $realtime, csr[9][23:16]);
            $display("[%0t]   Lane 1 DQS Init Tap  : %0d", $realtime, csr[9][31:24]);
            // 0xA: CONFIG
            $display("[%0t]   BIST_MODE            : %0d", $realtime, csr[10][1:0]);
            $display("[%0t]   BYTE_LANES           : %0d", $realtime, csr[10][7:4]);
            // 0xB: VERSION
            $display("[%0t]   IP Version           : %0d.%0d", $realtime, csr[11][15:8], csr[11][7:0]);
            // 0xC: CONTROL
            $display("[%0t]   Auto Reset Enable    : %0b", $realtime, csr[12][2]);
            // 0xD: INIT_PROGRESS
            $display("[%0t]   ROM Instruction Addr : %0d", $realtime, csr[13][5:0]);
            $display("[%0t]   Pause Counter        : %0b", $realtime, csr[13][6]);
            $display("[%0t]   Reset Done           : %0b", $realtime, csr[13][7]);
            $display("[%0t]   Pipe Stall           : %0b", $realtime, csr[13][8]);
        end
        $display("[%0t] === End CSR Dump ===", $realtime);

        $finish;
    end

    localparam time TB_TIMEOUT_PS = 64'd10_000_000_000;
    initial begin
        #TB_TIMEOUT_PS;
        $display("[%0t] TIMEOUT: simulation did not complete within 10 ms", $realtime);
        $finish;
    end

    // ===================================================================
    // Training Failure Injection
    // When SIM_FORCE_TRAIN_FAIL is defined, force DQ[0] to constant 0
    // during gate training. The PHY expects the MPR pattern (01010101)
    // on DQ[0] but sees all-zeros, so bitslip alignment never succeeds.
    // After 3 retries the controller enters CALIB_ERROR and asserts
    // init_failed.
    // ===================================================================
`ifdef SIM_FORCE_TRAIN_FAIL
    initial begin
        force ddr4_dq[0] = 1'b0;
        wait (init_done || init_failed);
        release ddr4_dq[0];
    end
`endif

    // ===================================================================
    // Wave Dump
    // ===================================================================
    generate
        if (TB_VCD_ENABLE) begin : g_vcd_dump
            initial begin
                if (TB_VCD_END_TIME <= TB_VCD_START_TIME) begin
                    $display("[%0t] WARNING: VCD disabled because end time (%0t) is not after start time (%0t)",
                             $realtime, TB_VCD_END_TIME, TB_VCD_START_TIME);
                end else begin
                    $dumpfile("trace.vcd");
                    $dumpvars(0, ddr4_sim_top);
                    $dumpoff;

                    #(TB_VCD_START_TIME);
                    $display("[%0t] VCD dump enabled: trace.vcd", $realtime);
                    $dumpon;

                    #(TB_VCD_END_TIME - TB_VCD_START_TIME);
                    $dumpoff;
                    $display("[%0t] VCD dump disabled: trace.vcd", $realtime);
                end
            end
        end
    endgenerate

endmodule
