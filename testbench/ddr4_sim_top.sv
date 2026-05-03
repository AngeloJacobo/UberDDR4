////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_sim_top.sv
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  Top-level simulation testbench. Generates clocks, reset,
//  instantiates the DUT (ddr4_top) and Micron DDR4 behavioral model wrapper,
//  monitors the init sequence, dumps waves, and enforces a timeout.
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

module ddr4_sim_top;

    // ═══════════════════════════════════════════════════════════════════
    // Parameters — match DUT defaults but use integer-exact clock periods
    // DDR4-2400: tCK=834ps (417ps half, exact integer), 4:1 ratio
    // ═══════════════════════════════════════════════════════════════════
    localparam DDR4_CLK_PERIOD = 834;
    localparam CTRL_CLK_PERIOD = DDR4_CLK_PERIOD * 4;
    localparam DQ_BITS     = 8;
    localparam BYTE_LANES  = 2;
    localparam ROW_BITS    = 16;
    localparam COL_BITS    = 10;
    localparam BA_BITS     = 2;
    localparam BG_BITS     = 2;
    localparam SERDES_RATIO   = 4;
    localparam WB_DATA_BITS   = DQ_BITS * BYTE_LANES * 2 * SERDES_RATIO;
    localparam WB_SEL_BITS    = WB_DATA_BITS / 8;
    localparam COL_LOW        = $clog2(SERDES_RATIO * 2 * DQ_BITS * BYTE_LANES / 8);
    localparam WB_ADDR_BITS   = ROW_BITS + BG_BITS + BA_BITS + COL_BITS - COL_LOW;
    localparam EXT_ADDR_BITS  = WB_ADDR_BITS + 1;

    // ═══════════════════════════════════════════════════════════════════
    // Regression-overridable parameters via +define+ (source stays untouched)
    //
    // FLY_BY: CK fly-by delay in ps applied to iDDR4_1 (lane 1).
    //   Models real PCB daisy-chain routing: FPGA → chip0 → chip1.
    //   Realistic range: 50–400ps for a 2-chip DDR4-2400 board.
    //
    // TB_SKIP_CALIB: 0 = full PHY training, 1 = bypass (Phase 6 mode)
    // TB_ADDR_MAPPING: 0 = sequential, 1 = BG-interleaved (default)
    // ═══════════════════════════════════════════════════════════════════
`ifdef SIM_FLY_BY_DELAY
    localparam FLY_BY = `SIM_FLY_BY_DELAY;
`else
    localparam FLY_BY = 0;
`endif

`ifdef SIM_SKIP_CALIB
    localparam TB_SKIP_CALIB = `SIM_SKIP_CALIB;
`else
    localparam TB_SKIP_CALIB = 0;
`endif

`ifdef SIM_ADDR_MAPPING
    localparam TB_ADDR_MAPPING = `SIM_ADDR_MAPPING;
`else
    localparam TB_ADDR_MAPPING = 1;
`endif

    // ═══════════════════════════════════════════════════════════════════
    // Clock Generation
    // ddr4_clk   : 834 ps period (toggle every 417 ps)
    // controller : ddr4_clk / 4 = 3336 ps period (phase-aligned)
    // ref_clk    : 300 MHz = 3334 ps period (UltraScale+ IDELAYE3/ODELAYE3 min)
    // ═══════════════════════════════════════════════════════════════════
    reg ddr4_clk;
    initial ddr4_clk = 1'b0;
    always #(DDR4_CLK_PERIOD / 2) ddr4_clk = ~ddr4_clk;

    reg [1:0] clk_div;
    initial clk_div = 2'b00;
    always @(posedge ddr4_clk) clk_div <= clk_div + 1;
    wire controller_clk = clk_div[1];

    reg ref_clk;
    initial ref_clk = 1'b0;
    always #1667 ref_clk = ~ref_clk;

    // ═══════════════════════════════════════════════════════════════════
    // Reset — held low for 100 ns, released on controller_clk posedge
    // ═══════════════════════════════════════════════════════════════════
    reg rst_n;
    initial begin
        rst_n = 1'b0;
        #100_000;
        @(posedge controller_clk);
        rst_n = 1'b1;
    end

    // ═══════════════════════════════════════════════════════════════════
    // DDR4 wires between DUT and model wrapper
    // ═══════════════════════════════════════════════════════════════════
    wire ddr4_ck_p, ddr4_ck_n;
    wire ddr4_reset_n, ddr4_cke, ddr4_cs_n, ddr4_act_n;
    wire [16:0] ddr4_addr;
    wire [BA_BITS-1:0] ddr4_ba;
    wire [BG_BITS-1:0] ddr4_bg;
    wire ddr4_odt;
    wire [BYTE_LANES-1:0] ddr4_dm_n;
    wire [DQ_BITS*BYTE_LANES-1:0] ddr4_dq;
    wire [BYTE_LANES-1:0] ddr4_dqs_p, ddr4_dqs_n;

    // ═══════════════════════════════════════════════════════════════════
    // Wishbone / BIST Tie-Offs (static zero, init to 'z per convention)
    // ═══════════════════════════════════════════════════════════════════
    reg                      wb_cyc;
    reg                      wb_stb;
    reg                      wb_we;
    reg [EXT_ADDR_BITS-1:0]  wb_addr;
    reg [WB_DATA_BITS-1:0]   wb_data;
    reg [WB_SEL_BITS-1:0]    wb_sel;
    wire                     wb_stall;
    wire                     wb_ack;
    wire [WB_DATA_BITS-1:0]  wb_rdata;
    reg                      bist_start;
    wire                     calib_complete;
    wire                     calib_error;
    wire                     bist_busy;
    wire                     bist_pass;
    wire                     bist_fail;
    wire [31:0]              bist_correct;
    wire [31:0]              bist_error;

    initial begin
        wb_cyc     = 'z;
        wb_stb     = 'z;
        wb_we      = 'z;
        wb_addr    = 'z;
        wb_data    = 'z;
        wb_sel     = 'z;
        bist_start = 'z;
        #1;
        wb_cyc     = 1'b0;
        wb_stb     = 1'b0;
        wb_we      = 1'b0;
        wb_addr    = {EXT_ADDR_BITS{1'b0}};
        wb_data    = {WB_DATA_BITS{1'b0}};
        wb_sel     = {WB_SEL_BITS{1'b0}};
        bist_start = 1'b0;
    end

    // ═══════════════════════════════════════════════════════════════════
    // DUT — ddr4_top with MICRON_SIM=1 (shortened init delays)
    // SKIP_CALIB and ADDR_MAPPING driven by regression-overridable params
    // ═══════════════════════════════════════════════════════════════════
    ddr4_top #(
        .CONTROLLER_CLK_PERIOD (CTRL_CLK_PERIOD),
        .DDR4_CLK_PERIOD       (DDR4_CLK_PERIOD),
        .ROW_BITS              (ROW_BITS),
        .COL_BITS              (COL_BITS),
        .BA_BITS               (BA_BITS),
        .BG_BITS               (BG_BITS),
        .DQ_BITS               (DQ_BITS),
        .BYTE_LANES            (BYTE_LANES),
        .DENSITY               (8),
        .MICRON_SIM            (1),
        .SKIP_CALIB            (TB_SKIP_CALIB),
        .ADDR_MAPPING          (TB_ADDR_MAPPING),
        .BIST_MODE             (0),
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
        .i_bist_start      (bist_start),
        .o_bist_busy       (bist_busy),
        .o_bist_pass       (bist_pass),
        .o_bist_fail       (bist_fail),
        .o_bist_correct    (bist_correct),
        .o_bist_error      (bist_error),
        .o_calib_complete  (calib_complete),
        .o_calib_error     (calib_error)
    );

    // ═══════════════════════════════════════════════════════════════════
    // Micron DDR4 x8 Models — direct instantiation (no wrapper module)
    //
    // Bidirectional DQ/DQS/DM wiring uses the `short` module for xsim
    // (Xilinx's own workaround — see MIG sim_tb_top.sv) and `tran`
    // gate primitives for all other simulators.
    // ═══════════════════════════════════════════════════════════════════
    import arch_package::*;

    DDR4_if #(.CONFIGURED_DQ_BITS(DQ_BITS)) iDDR4_0();
    DDR4_if #(.CONFIGURED_DQ_BITS(DQ_BITS)) iDDR4_1();

    // Command/address fan-out (shared across both devices)
    assign iDDR4_0.CK        = {ddr4_ck_p, ddr4_ck_n};
    assign iDDR4_0.RESET_n   = ddr4_reset_n;
    assign iDDR4_0.CKE       = ddr4_cke;
    assign iDDR4_0.CS_n      = ddr4_cs_n;
    assign iDDR4_0.ACT_n     = ddr4_act_n;
    assign iDDR4_0.RAS_n_A16 = ddr4_addr[16];
    assign iDDR4_0.CAS_n_A15 = ddr4_addr[15];
    assign iDDR4_0.WE_n_A14  = ddr4_addr[14];
    assign iDDR4_0.ADDR      = ddr4_addr[13:0];
    assign iDDR4_0.BA        = ddr4_ba;
    assign iDDR4_0.BG        = ddr4_bg;
    assign iDDR4_0.ODT       = ddr4_odt;
    assign iDDR4_0.ADDR_17   = 1'b0;
    assign iDDR4_0.C         = '0;
    assign iDDR4_0.TEN       = 1'b0;
    assign iDDR4_0.PARITY    = 1'b0;
    assign iDDR4_0.ZQ        = 1'b1;
    assign iDDR4_0.PWR       = 1'b1;
    assign iDDR4_0.VREF_CA   = 1'b1;
    assign iDDR4_0.VREF_DQ   = 1'b1;

    // Lane 1 (far end of fly-by) — CK/CMD/ADDR delayed by FLY_BY ps.
    // Models real PCB daisy-chain: FPGA → chip0 (near) → chip1 (far).
    // DQ/DQS stay zero-delay (point-to-point, no fly-by).
    assign #(FLY_BY) iDDR4_1.CK        = {ddr4_ck_p, ddr4_ck_n};
    assign #(FLY_BY) iDDR4_1.RESET_n   = ddr4_reset_n;
    assign #(FLY_BY) iDDR4_1.CKE       = ddr4_cke;
    assign #(FLY_BY) iDDR4_1.CS_n      = ddr4_cs_n;
    assign #(FLY_BY) iDDR4_1.ACT_n     = ddr4_act_n;
    assign #(FLY_BY) iDDR4_1.RAS_n_A16 = ddr4_addr[16];
    assign #(FLY_BY) iDDR4_1.CAS_n_A15 = ddr4_addr[15];
    assign #(FLY_BY) iDDR4_1.WE_n_A14  = ddr4_addr[14];
    assign #(FLY_BY) iDDR4_1.ADDR      = ddr4_addr[13:0];
    assign #(FLY_BY) iDDR4_1.BA        = ddr4_ba;
    assign #(FLY_BY) iDDR4_1.BG        = ddr4_bg;
    assign #(FLY_BY) iDDR4_1.ODT       = ddr4_odt;
    assign iDDR4_1.ADDR_17   = 1'b0;
    assign iDDR4_1.C         = '0;
    assign iDDR4_1.TEN       = 1'b0;
    assign iDDR4_1.PARITY    = 1'b0;
    assign iDDR4_1.ZQ        = 1'b1;
    assign iDDR4_1.PWR       = 1'b1;
    assign iDDR4_1.VREF_CA   = 1'b1;
    assign iDDR4_1.VREF_DQ   = 1'b1;

    // Bidirectional DQ wiring — short (xsim) / tran (others)
    genvar gi;
    generate
        for (gi = 0; gi < DQ_BITS; gi = gi + 1) begin : bidi_dq
            `ifdef XILINX_SIMULATOR
            short bidiDQ0(iDDR4_0.DQ[gi], ddr4_dq[gi]);
            short bidiDQ1(iDDR4_1.DQ[gi], ddr4_dq[DQ_BITS + gi]);
            `else
            tran  bidiDQ0(iDDR4_0.DQ[gi], ddr4_dq[gi]);
            tran  bidiDQ1(iDDR4_1.DQ[gi], ddr4_dq[DQ_BITS + gi]);
            `endif
        end
    endgenerate

    // Bidirectional DQS/DM wiring
    `ifdef XILINX_SIMULATOR
    short bidiDQS_t0(iDDR4_0.DQS_t, ddr4_dqs_p[0]);
    short bidiDQS_c0(iDDR4_0.DQS_c, ddr4_dqs_n[0]);
    short bidiDM0   (iDDR4_0.DM_n,   ddr4_dm_n[0]);
    short bidiDQS_t1(iDDR4_1.DQS_t, ddr4_dqs_p[1]);
    short bidiDQS_c1(iDDR4_1.DQS_c, ddr4_dqs_n[1]);
    short bidiDM1   (iDDR4_1.DM_n,   ddr4_dm_n[1]);
    `else
    tran  bidiDQS_t0(iDDR4_0.DQS_t, ddr4_dqs_p[0]);
    tran  bidiDQS_c0(iDDR4_0.DQS_c, ddr4_dqs_n[0]);
    tran  bidiDM0   (iDDR4_0.DM_n,   ddr4_dm_n[0]);
    tran  bidiDQS_t1(iDDR4_1.DQS_t, ddr4_dqs_p[1]);
    tran  bidiDQS_c1(iDDR4_1.DQS_c, ddr4_dqs_n[1]);
    tran  bidiDM1   (iDDR4_1.DM_n,   ddr4_dm_n[1]);
    `endif

    // Micron DDR4 x8 model instances
    wire model_en;
    assign model_en = 1'b1;

    ddr4_model #(
        .CONFIGURED_DQ_BITS (DQ_BITS),
        .CONFIGURED_DENSITY (_8G),
        .CONFIGURED_RANKS   (1)
    ) u_ddr4_0 (
        .model_enable (model_en),
        .iDDR4        (iDDR4_0)
    );

    ddr4_model #(
        .CONFIGURED_DQ_BITS (DQ_BITS),
        .CONFIGURED_DENSITY (_8G),
        .CONFIGURED_RANKS   (1)
    ) u_ddr4_1 (
        .model_enable (model_en),
        .iDDR4        (iDDR4_1)
    );

    // ═══════════════════════════════════════════════════════════════════
    // Human-Readable Debug Signals (display with ASCII radix in viewer)
    // ═══════════════════════════════════════════════════════════════════
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

    // Micron-side command decode — reads the DDR4_if interface signals that
    // feed directly into the encrypted Micron model. Proves what the model
    // actually receives after the PHY's OSERDESE3 → OBUF chain.
    wire micron_cs_n  = iDDR4_0.CS_n;
    wire micron_act_n = iDDR4_0.ACT_n;
    wire micron_ras   = iDDR4_0.RAS_n_A16;
    wire micron_cas   = iDDR4_0.CAS_n_A15;
    wire micron_we    = iDDR4_0.WE_n_A14;

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

    // ═══════════════════════════════════════════════════════════════════
    // Monitoring and Test Control
    // ═══════════════════════════════════════════════════════════════════
    reg reset_done_seen;
    initial reset_done_seen = 1'b0;

    initial $timeformat(-9, 3, "ns", 0);

    initial begin
        $display("[0ns] CONFIG: FLY_BY=%0dps SKIP_CALIB=%0d ADDR_MAPPING=%0d",
            FLY_BY, TB_SKIP_CALIB, TB_ADDR_MAPPING);
    end

    always @(posedge controller_clk) begin
        if (rst_n && u_dut.u_controller.reset_done && !reset_done_seen) begin
            $display("[%0t] PASS: DDR4 init sequence complete (reset_done)", $realtime);
            reset_done_seen <= 1'b1;
        end
    end

    // Calibration status monitor
    reg calib_complete_seen;
    reg calib_error_seen;
    initial begin calib_complete_seen = 1'b0; calib_error_seen = 1'b0; end

    always @(posedge controller_clk) begin
        if (rst_n && calib_complete && !calib_complete_seen) begin
            $display("[%0t] CALIB_COMPLETE asserted", $realtime);
            calib_complete_seen <= 1'b1;
        end
        if (rst_n && calib_error && !calib_error_seen) begin
            $display("[%0t] CALIB_ERROR asserted — training timed out", $realtime);
            calib_error_seen <= 1'b1;
        end
    end

    // PHY training progress monitor — tracks gate, eye, and WL phases
    reg [3:0] prev_phy_state;
    initial prev_phy_state = 4'd0;

    always @(posedge controller_clk) begin
        if (rst_n) begin
            prev_phy_state <= u_dut.u_phy.phy_state;

            // Gate training
            if (prev_phy_state == 4'd0 && u_dut.u_phy.phy_state == 4'd1)
                $display("[%0t] PHY gate training started", $realtime);
            if (u_dut.u_phy.phy_state == 4'd3 && prev_phy_state != 4'd3)
                $display("[%0t] PHY gate training done (lane0 bs=%0d, lane1 bs=%0d)",
                    $realtime,
                    u_dut.u_phy.bitslip_count_q[0],
                    u_dut.u_phy.bitslip_count_q[1]);

            // Eye training
            if (prev_phy_state == 4'd0 && u_dut.u_phy.phy_state == 4'd4)
                $display("[%0t] PHY eye training started", $realtime);
            if (u_dut.u_phy.phy_state == 4'd7 && prev_phy_state != 4'd7)
                $display("[%0t] PHY eye training done (lane0 tap=%0d [%0d-%0d], lane1 tap=%0d [%0d-%0d])",
                    $realtime,
                    (u_dut.u_phy.first_pass_tap[0] + u_dut.u_phy.last_pass_tap[0]) >> 1,
                    u_dut.u_phy.first_pass_tap[0], u_dut.u_phy.last_pass_tap[0],
                    (u_dut.u_phy.first_pass_tap[1] + u_dut.u_phy.last_pass_tap[1]) >> 1,
                    u_dut.u_phy.first_pass_tap[1], u_dut.u_phy.last_pass_tap[1]);

            // Write leveling
            if (prev_phy_state == 4'd0 && u_dut.u_phy.phy_state == 4'd8)
                $display("[%0t] PHY write leveling started", $realtime);
            if (u_dut.u_phy.phy_state == 4'd11 && prev_phy_state != 4'd11)
                $display("[%0t] PHY write leveling done (lane0 dqs_tap=%0d, lane1 dqs_tap=%0d)",
                    $realtime,
                    u_dut.u_phy.wl_tap[0],
                    u_dut.u_phy.wl_tap[1]);
        end
    end

    // Calibration result validation — fires once when training completes
    reg calib_results_checked;
    initial calib_results_checked = 1'b0;

    always @(posedge controller_clk) begin
        if (calib_complete && !calib_results_checked && !TB_SKIP_CALIB) begin
            calib_results_checked <= 1'b1;
            $display("[%0t] ═══ CALIBRATION RESULTS ═══", $realtime);
            $display("[%0t]   FLY_BY_DELAY = %0d ps", $realtime, FLY_BY);
            $display("[%0t]   Gate: lane0 bs=%0d, lane1 bs=%0d",
                $realtime,
                u_dut.u_phy.bitslip_count_q[0],
                u_dut.u_phy.bitslip_count_q[1]);
            $display("[%0t]   Eye:  lane0 center=%0d [%0d-%0d], lane1 center=%0d [%0d-%0d]",
                $realtime,
                (u_dut.u_phy.first_pass_tap[0] + u_dut.u_phy.last_pass_tap[0]) >> 1,
                u_dut.u_phy.first_pass_tap[0], u_dut.u_phy.last_pass_tap[0],
                (u_dut.u_phy.first_pass_tap[1] + u_dut.u_phy.last_pass_tap[1]) >> 1,
                u_dut.u_phy.first_pass_tap[1], u_dut.u_phy.last_pass_tap[1]);
            $display("[%0t]   WL:   lane0 dqs_tap=%0d, lane1 dqs_tap=%0d",
                $realtime,
                u_dut.u_phy.wl_tap[0],
                u_dut.u_phy.wl_tap[1]);
            if (FLY_BY >= 100) begin
                if (u_dut.u_phy.wl_tap[0] == u_dut.u_phy.wl_tap[1])
                    $display("[%0t]   WARNING: WL taps identical despite FLY_BY=%0dps — expected asymmetry",
                        $realtime, FLY_BY);
                else
                    $display("[%0t]   OK: WL taps differ (lane0=%0d, lane1=%0d) — fly-by compensation working",
                        $realtime, u_dut.u_phy.wl_tap[0], u_dut.u_phy.wl_tap[1]);
            end
        end
    end

    // ═══════════════════════════════════════════════════════════════════
    // WB Write Stimulus — Multi-phase test covering all scheduler paths
    //
    // ADDR_MAPPING=1 (BG-interleaved) address layout:
    //   bits [1:0]   = BG
    //   bits [7:2]   = col_upper (→ col[9:4])
    //   bits [9:8]   = BA
    //   bits [25:10]  = row
    //
    // Phase A: Cold writes to idle banks       (ACT → WR)
    // Phase B: Bank hits, same row still open  (WR only)
    // Phase C: Row miss, different row          (PRE → ACT → WR)
    // Phase D: Post-refresh, banks closed       (ACT → WR again)
    // Phase E: Same-BG different-bank writes   (tests tRRD_L / tCCD_L)
    // ═══════════════════════════════════════════════════════════════════

    integer ref_count;
    initial ref_count = 0;

    reg [8*8-1:0] test_phase;
    initial test_phase = "IDLE";
    reg all_tests_done;
    initial all_tests_done = 1'b0;

    localparam [EXT_ADDR_BITS-1:0] ROW0_BG0 = 0,
                                    ROW0_BG1 = 1,
                                    ROW0_BG2 = 2,
                                    ROW0_BG3 = 3,
                                    ROW1_BG0 = (1 << 10) | 0,
                                    ROW1_BG1 = (1 << 10) | 1,
                                    ROW1_BG2 = (1 << 10) | 2,
                                    ROW1_BG3 = (1 << 10) | 3,
                                    ROW0_BG0_BA1 = (1 << 8) | 0,
                                    ROW0_BG0_BA2 = (2 << 8) | 0,
                                    ROW2_BG0 = (2 << 10) | 0,
                                    ROW2_BG1 = (2 << 10) | 1,
                                    ROW2_BG2 = (2 << 10) | 2,
                                    ROW2_BG3 = (2 << 10) | 3,
                                    ROW2_BG0_C1 = (2 << 10) | (1 << 2) | 0,
                                    ROW2_BG1_C1 = (2 << 10) | (1 << 2) | 1;

    task wb_write_one(input [EXT_ADDR_BITS-1:0] addr, input [WB_DATA_BITS-1:0] data);
        begin
            wb_stb = 1'b0;
            @(posedge controller_clk);
            while (wb_stall) @(posedge controller_clk);
            wb_cyc  = 1'b1;
            wb_stb  = 1'b1;
            wb_we   = 1'b1;
            wb_addr = addr;
            wb_data = data;
            wb_sel  = {WB_SEL_BITS{1'b1}};
            @(posedge controller_clk);
        end
    endtask

    task wb_read_one(input [EXT_ADDR_BITS-1:0] addr);
        begin
            wb_stb = 1'b0;
            @(posedge controller_clk);
            while (wb_stall) @(posedge controller_clk);
            wb_cyc  = 1'b1;
            wb_stb  = 1'b1;
            wb_we   = 1'b0;
            wb_addr = addr;
            wb_sel  = {WB_SEL_BITS{1'b1}};
            @(posedge controller_clk);
        end
    endtask

    task wb_idle;
        begin
            wb_stb = 1'b0;
            wb_cyc = 1'b0;
            wb_we  = 1'b0;
        end
    endtask

    task drain_pipeline;
        begin
            @(posedge controller_clk);
            wb_idle;
            repeat (39) @(posedge controller_clk);
        end
    endtask

    integer rd_err_count;
    initial rd_err_count = 0;

    task wb_write_read_check(
        input [EXT_ADDR_BITS-1:0] addr,
        input [WB_DATA_BITS-1:0]  wdata
    );
        reg [WB_DATA_BITS-1:0] captured;
        begin
            wb_write_one(addr, wdata);
            drain_pipeline;
            wb_read_one(addr);
            wb_stb = 1'b0;
            while (!wb_ack) @(posedge controller_clk);
            captured = wb_rdata;
            wb_idle;
            repeat (5) @(posedge controller_clk);
            if (captured === wdata) begin
                $display("[%0t]   PASS: addr=0x%0h data=0x%0h", $realtime, addr, captured);
            end else begin
                $display("[%0t]   FAIL: addr=0x%0h expected=0x%0h got=0x%0h",
                         $realtime, addr, wdata, captured);
                rd_err_count = rd_err_count + 1;
            end
        end
    endtask

    integer wb_write_count;
    initial wb_write_count = 0;

    initial begin
        wait (calib_complete_seen);
        @(posedge controller_clk);
        while (wb_stall) @(posedge controller_clk);

        // ── Phase A: Cold writes to 4 idle banks (ACT → WR) ──
        test_phase = "PHASE_A";
        $display("[%0t] ═══ Phase A: Cold writes to idle banks (ACT→WR) ═══", $realtime);
        wb_write_one(ROW0_BG0, 128'h0A);
        $display("[%0t]   write BG0/BA0/row0", $realtime);
        wb_write_one(ROW0_BG1, 128'h0B);
        $display("[%0t]   write BG1/BA0/row0", $realtime);
        wb_write_one(ROW0_BG2, 128'h0C);
        $display("[%0t]   write BG2/BA0/row0", $realtime);
        wb_write_one(ROW0_BG3, 128'h0D);
        $display("[%0t]   write BG3/BA0/row0", $realtime);
        drain_pipeline;

        // ── Phase B: Bank hits — same bank, same row still open (WR only) ──
        test_phase = "PHASE_B";
        $display("[%0t] ═══ Phase B: Bank hits — same row open (WR only) ═══", $realtime);
        wb_write_one(ROW0_BG0, 128'h1A);
        $display("[%0t]   write BG0/BA0/row0 (hit)", $realtime);
        wb_write_one(ROW0_BG1, 128'h1B);
        $display("[%0t]   write BG1/BA0/row0 (hit)", $realtime);
        wb_write_one(ROW0_BG2, 128'h1C);
        $display("[%0t]   write BG2/BA0/row0 (hit)", $realtime);
        wb_write_one(ROW0_BG3, 128'h1D);
        $display("[%0t]   write BG3/BA0/row0 (hit)", $realtime);
        drain_pipeline;

        // ── Phase C: Row miss — same bank, different row (PRE → ACT → WR) ──
        test_phase = "PHASE_C";
        $display("[%0t] ═══ Phase C: Row miss — different row (PRE→ACT→WR) ═══", $realtime);
        wb_write_one(ROW1_BG0, 128'h2A);
        $display("[%0t]   write BG0/BA0/row1 (miss)", $realtime);
        wb_write_one(ROW1_BG1, 128'h2B);
        $display("[%0t]   write BG1/BA0/row1 (miss)", $realtime);
        wb_write_one(ROW1_BG2, 128'h2C);
        $display("[%0t]   write BG2/BA0/row1 (miss)", $realtime);
        wb_write_one(ROW1_BG3, 128'h2D);
        $display("[%0t]   write BG3/BA0/row1 (miss)", $realtime);
        drain_pipeline;

        // ── Phase D: Wait for refresh (PRE ALL closes all banks), then re-access ──
        test_phase = "WAIT_REF";
        $display("[%0t] ═══ Waiting for refresh to close all banks... ═══", $realtime);
        wb_idle;
        wait (ref_count >= 2);
        @(posedge controller_clk);
        while (wb_stall) @(posedge controller_clk);

        test_phase = "PHASE_D";
        $display("[%0t] ═══ Phase D: Post-refresh writes (ACT→WR) ═══", $realtime);
        wb_write_one(ROW0_BG0, 128'h3A);
        $display("[%0t]   write BG0/BA0/row0 (post-refresh)", $realtime);
        wb_write_one(ROW0_BG1, 128'h3B);
        $display("[%0t]   write BG1/BA0/row0 (post-refresh)", $realtime);
        wb_write_one(ROW0_BG2, 128'h3C);
        $display("[%0t]   write BG2/BA0/row0 (post-refresh)", $realtime);
        wb_write_one(ROW0_BG3, 128'h3D);
        $display("[%0t]   write BG3/BA0/row0 (post-refresh)", $realtime);
        drain_pipeline;

        // ── Phase E: Same-BG different-bank writes (tRRD_L / tCCD_L stress) ──
        test_phase = "PHASE_E";
        $display("[%0t] ═══ Phase E: Same-BG different-bank writes (tRRD/tCCD_L) ═══", $realtime);
        wb_write_one(ROW0_BG0,     128'h4A);
        $display("[%0t]   write BG0/BA0/row0", $realtime);
        wb_write_one(ROW0_BG0_BA1, 128'h4B);
        $display("[%0t]   write BG0/BA1/row0 (same BG, diff bank)", $realtime);
        wb_write_one(ROW0_BG0_BA2, 128'h4C);
        $display("[%0t]   write BG0/BA2/row0 (same BG, diff bank)", $realtime);
        drain_pipeline;

        // ── Phase F: Read requests (exercises sched_read path) ──
        // Reads won't return data (no PHY data path yet), but the scheduler
        // issues RD commands and the Micron model validates timing.
        test_phase = "PHASE_F";
        $display("[%0t] ═══ Phase F: Read requests (ACT→RD, bank hit RD) ═══", $realtime);
        wb_write_one(ROW0_BG0, 128'h5A);
        $display("[%0t]   write BG0/BA0/row0 (open bank)", $realtime);
        wb_write_one(ROW0_BG1, 128'h5B);
        $display("[%0t]   write BG1/BA0/row0 (open bank)", $realtime);
        drain_pipeline;
        wb_read_one(ROW0_BG0);
        $display("[%0t]   read BG0/BA0/row0 (bank hit RD)", $realtime);
        wb_read_one(ROW0_BG1);
        $display("[%0t]   read BG1/BA0/row0 (bank hit RD)", $realtime);
        wb_read_one(ROW0_BG2);
        $display("[%0t]   read BG2/BA0/row0 (cold bank ACT→RD)", $realtime);
        wb_read_one(ROW0_BG3);
        $display("[%0t]   read BG3/BA0/row0 (cold bank ACT→RD)", $realtime);
        drain_pipeline;

        // ── Phase G: Same-bank rapid re-access (tRC stress: ACT→ACT same bank) ──
        test_phase = "PHASE_G";
        $display("[%0t] ═══ Phase G: Same-bank rapid re-access (tRC stress) ═══", $realtime);
        wb_write_one(ROW0_BG0, 128'h6A);
        $display("[%0t]   write BG0/BA0/row0", $realtime);
        drain_pipeline;
        wb_write_one(ROW1_BG0, 128'h6B);
        $display("[%0t]   write BG0/BA0/row1 (miss → PRE+ACT+WR same bank)", $realtime);
        drain_pipeline;
        wb_write_one(ROW0_BG0, 128'h6C);
        $display("[%0t]   write BG0/BA0/row0 (miss → PRE+ACT+WR same bank again)", $realtime);
        drain_pipeline;

        // ── Phase H: Write-then-read same BG (tWTR stress) ──
        test_phase = "PHASE_H";
        $display("[%0t] ═══ Phase H: Write-then-read same BG (tWTR stress) ═══", $realtime);
        wb_write_one(ROW0_BG0, 128'h7A);
        $display("[%0t]   write BG0/BA0/row0", $realtime);
        wb_read_one(ROW0_BG0);
        $display("[%0t]   read BG0/BA0/row0 (same BG → tWTR_L)", $realtime);
        wb_write_one(ROW0_BG1, 128'h7B);
        $display("[%0t]   write BG1/BA0/row0", $realtime);
        wb_read_one(ROW0_BG0);
        $display("[%0t]   read BG0/BA0/row0 (diff BG → tWTR_S applies to BG1)", $realtime);
        drain_pipeline;

        // ── Phase I: Write→Read data round-trip verification ──
        test_phase = "PHASE_I";
        $display("[%0t] ═══ Phase I: Write→Read round-trip verification ═══", $realtime);
        wb_write_read_check(ROW2_BG0,    128'hDEAD_BEEF_CAFE_BABE_0123_4567_89AB_CDEF);
        wb_write_read_check(ROW2_BG1,    128'hFFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF);
        wb_write_read_check(ROW2_BG2,    128'h0000_0000_0000_0000_0000_0000_0000_0000);
        wb_write_read_check(ROW2_BG3,    128'hA5A5_A5A5_5A5A_5A5A_A5A5_A5A5_5A5A_5A5A);
        wb_write_read_check(ROW2_BG0_C1, 128'h0000_0000_0000_0000_FFFF_FFFF_FFFF_FFFF);
        wb_write_read_check(ROW2_BG1_C1, 128'hFFFF_FFFF_FFFF_FFFF_0000_0000_0000_0000);
        wb_write_read_check(ROW2_BG0,    128'h1234_5678_9ABC_DEF0_FEDC_BA98_7654_3210);
        wb_write_read_check(ROW2_BG1,    128'h0F0F_0F0F_F0F0_F0F0_0F0F_0F0F_F0F0_F0F0);
        wb_write_read_check(ROW2_BG2,    128'h0000_0000_0000_0001_8000_0000_0000_0000);
        wb_write_read_check(ROW2_BG3,    128'hAAAA_AAAA_5555_5555_AAAA_AAAA_5555_5555);

        if (rd_err_count == 0)
            $display("[%0t] PASS: All 10 write→read checks passed", $realtime);
        else
            $display("[%0t] FAIL: %0d of 10 write→read checks failed", $realtime, rd_err_count);

        test_phase = "DONE";
        $display("[%0t] ═══ All test phases complete ═══", $realtime);
        all_tests_done = 1'b1;
    end

    // ═══════════════════════════════════════════════════════════════════
    // Command Monitor — uses DFI signals (all 4 phases per controller cycle)
    // DDR4 bus only shows one slot per controller_clk edge; DFI shows all 4.
    // ═══════════════════════════════════════════════════════════════════
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

    integer act_count, wr_count, rd_count, pre_count;
    integer mon_ph;
    initial begin act_count = 0; wr_count = 0; rd_count = 0; pre_count = 0; end

    always @(posedge controller_clk) begin
        if (reset_done_seen && !all_tests_done) begin
            for (mon_ph = 0; mon_ph < 4; mon_ph = mon_ph + 1) begin
                if (mon_is_act[mon_ph]) begin
                    act_count = act_count + 1;
                    $display("[%0t] [%0s] DDR4 ACT #%0d: BG=%0d BA=%0d (slot %0d)",
                             $realtime, test_phase, act_count,
                             mon_bg[mon_ph*BG_BITS +: BG_BITS],
                             mon_bank[mon_ph*BA_BITS +: BA_BITS], mon_ph);
                end
                if (mon_is_wr[mon_ph]) begin
                    wr_count = wr_count + 1;
                    $display("[%0t] [%0s] DDR4 WR  #%0d: BG=%0d BA=%0d (slot %0d)",
                             $realtime, test_phase, wr_count,
                             mon_bg[mon_ph*BG_BITS +: BG_BITS],
                             mon_bank[mon_ph*BA_BITS +: BA_BITS], mon_ph);
                end
                if (mon_is_rd[mon_ph]) begin
                    rd_count = rd_count + 1;
                    $display("[%0t] [%0s] DDR4 RD  #%0d: BG=%0d BA=%0d (slot %0d) stage2_we=%0b",
                             $realtime, test_phase, rd_count,
                             mon_bg[mon_ph*BG_BITS +: BG_BITS],
                             mon_bank[mon_ph*BA_BITS +: BA_BITS], mon_ph,
                             u_dut.u_controller.stage2_we);
                end
                if (mon_is_pre[mon_ph]) begin
                    pre_count = pre_count + 1;
                    $display("[%0t] [%0s] DDR4 PRE #%0d: BG=%0d BA=%0d (slot %0d)",
                             $realtime, test_phase, pre_count,
                             mon_bg[mon_ph*BG_BITS +: BG_BITS],
                             mon_bank[mon_ph*BA_BITS +: BA_BITS], mon_ph);
                end
            end
        end
    end

    // ═══════════════════════════════════════════════════════════════════
    // REF Monitor
    // ═══════════════════════════════════════════════════════════════════
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
                $display("[%0t] REF #%0d — interval = %0t (tREFI = 7800.000ns)",
                         $realtime, ref_count, ref_time_curr - ref_time_prev);
            end else begin
                $display("[%0t] REF #%0d", $realtime, ref_count);
            end
        end
    end

    // ═══════════════════════════════════════════════════════════════════
    // End-of-test: wait for all phases + 3 refresh cycles, then report
    // ═══════════════════════════════════════════════════════════════════
    localparam NUM_REFRESH_CYCLES = 3;

    initial begin
        wait (all_tests_done);
        wait (ref_count >= NUM_REFRESH_CYCLES);
        repeat (10) @(posedge controller_clk);
        $display("");
        $display("[%0t] ═══════════════════════════════════════════════", $realtime);
        $display("[%0t] SUMMARY: ACT=%0d  WR=%0d  RD=%0d  PRE=%0d  REF=%0d  RD_ERR=%0d",
                 $realtime, act_count, wr_count, rd_count, pre_count, ref_count, rd_err_count);
        if (rd_err_count == 0)
            $display("[%0t] PASS: All 9 test phases + %0d refresh cycles, zero violations",
                     $realtime, ref_count);
        else
            $display("[%0t] FAIL: %0d read data mismatches detected",
                     $realtime, rd_err_count);
        $display("[%0t] Simulation finished successfully", $realtime);
        $display("[%0t] ═══════════════════════════════════════════════", $realtime);
        $finish;
    end

    initial begin
        #200_000_000;
        $display("[%0t] TIMEOUT: simulation did not complete within 200 us", $realtime);
        $finish;
    end

    // ═══════════════════════════════════════════════════════════════════
    // Wave Dump — VCD for xsim, SHM for Xcelium
    // ═══════════════════════════════════════════════════════════════════
`ifdef VCD_DUMP
    initial begin
        $dumpfile("trace.vcd");
        $dumpvars(0, ddr4_sim_top);
    end
`else
    initial begin
        $shm_open("waves.shm", 1);
        $shm_probe(ddr4_sim_top, "ASCMT");
    end
`endif

endmodule
