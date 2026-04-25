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

`default_nettype none
`timescale 1ps / 1ps

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
    // Clock Generation
    // ddr4_clk   : 834 ps period (toggle every 417 ps)
    // controller : ddr4_clk / 4 = 3336 ps period (phase-aligned)
    // ref_clk    : 200 MHz = 5000 ps period
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
    always #2500 ref_clk = ~ref_clk;

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
    // DUT — ddr4_top with MICRON_SIM=1 (shortened init delays)
    // and SKIP_CALIB=1 (no PHY training in Phase 3)
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
        .SKIP_CALIB            (1),
        .ADDR_MAPPING          (1)
    ) u_dut (
        .i_controller_clk (controller_clk),
        .i_ddr4_clk       (ddr4_clk),
        .i_ref_clk        (ref_clk),
        .i_rst_n          (rst_n),
        .i_wb_cyc          (1'b0),
        .i_wb_stb          (1'b0),
        .i_wb_we           (1'b0),
        .i_wb_addr         ({EXT_ADDR_BITS{1'b0}}),
        .i_wb_data         ({WB_DATA_BITS{1'b0}}),
        .i_wb_sel          ({WB_SEL_BITS{1'b0}}),
        .o_wb_stall        (),
        .o_wb_ack          (),
        .o_wb_data         (),
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
        .i_bist_start      (1'b0),
        .o_bist_busy       (),
        .o_bist_pass       (),
        .o_bist_fail       (),
        .o_bist_correct    (),
        .o_bist_error      (),
        .o_calib_complete  (),
        .o_calib_error     ()
    );

    // ═══════════════════════════════════════════════════════════════════
    // Micron DDR4 Model Wrapper (2× x8 devices, one per byte lane)
    // ═══════════════════════════════════════════════════════════════════
    ddr4_model_wrapper #(
        .DQ_BITS    (DQ_BITS),
        .BYTE_LANES (BYTE_LANES)
    ) u_ddr4_mem (
        .i_ddr4_ck_p    (ddr4_ck_p),
        .i_ddr4_ck_n    (ddr4_ck_n),
        .i_ddr4_reset_n (ddr4_reset_n),
        .i_ddr4_cke     (ddr4_cke),
        .i_ddr4_cs_n    (ddr4_cs_n),
        .i_ddr4_act_n   (ddr4_act_n),
        .i_ddr4_addr    (ddr4_addr),
        .i_ddr4_ba      (ddr4_ba),
        .i_ddr4_bg      (ddr4_bg),
        .i_ddr4_odt     (ddr4_odt),
        .io_ddr4_dq     (ddr4_dq),
        .io_ddr4_dqs_p  (ddr4_dqs_p),
        .io_ddr4_dqs_n  (ddr4_dqs_n),
        .io_ddr4_dm_n   (ddr4_dm_n)
    );

    // ═══════════════════════════════════════════════════════════════════
    // Monitoring and Test Control
    // ═══════════════════════════════════════════════════════════════════
    reg reset_done_seen;
    initial reset_done_seen = 1'b0;

    initial $timeformat(-9, 3, "ns", 0);

    always @(posedge controller_clk) begin
        if (rst_n && u_dut.u_controller.reset_done && !reset_done_seen) begin
            $display("[%0t] PASS: DDR4 init sequence complete (reset_done)", $realtime);
            reset_done_seen <= 1'b1;
        end
    end

    initial begin
        wait (reset_done_seen);
        repeat (100) @(posedge controller_clk);
        $display("[%0t] Simulation finished successfully", $realtime);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("[%0t] TIMEOUT: init did not complete within 50 us", $realtime);
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
