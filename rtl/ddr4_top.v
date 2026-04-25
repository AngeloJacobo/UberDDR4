////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_top.v
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  Top module instantiating the controller and PHY, connected via
//  DFI 3.1 internal bus. Use this as the top module for Wishbone integration.
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

module ddr4_top #(
    parameter CONTROLLER_CLK_PERIOD = 3_333,
              DDR4_CLK_PERIOD = 833,
              ROW_BITS = 16,
              COL_BITS = 10,
              BA_BITS = 2,
              BG_BITS = 2,
              DQ_BITS = 8,
              BYTE_LANES = 2,
              DENSITY = 8,
    parameter[0:0] MICRON_SIM = 0,
                   SKIP_CALIB = 0,
    parameter[1:0] ADDR_MAPPING = 1,
    parameter[2:0] RTT_NOM = 3'b001,
                   RTT_WR = 3'b000,
                   RTT_PARK = 3'b000,
    parameter[0:0] DRIVE_IMP = 0,
    parameter[5:0] CL = 0,
    parameter[4:0] CWL_PARAM = 0,
    // Prober config (Phase 8)
    parameter[1:0] BIST_MODE = 0,
    parameter DEBUG_CSR_ENABLE = 1,
    // Derived (for port widths)
    parameter SERDES_RATIO = 4,
              NUM_BG = (1 << BG_BITS),
              NUM_BANKS = NUM_BG * (1 << BA_BITS),
              DFI_DATA_WIDTH = 2 * DQ_BITS * BYTE_LANES,
              WB_DATA_BITS = DQ_BITS * BYTE_LANES * 2 * SERDES_RATIO,
              WB_SEL_BITS = WB_DATA_BITS / 8,
              COL_LOW = $clog2(SERDES_RATIO * 2 * DQ_BITS * BYTE_LANES / 8),
              WB_ADDR_BITS = ROW_BITS + BG_BITS + BA_BITS + COL_BITS - COL_LOW,
              EXT_ADDR_BITS = WB_ADDR_BITS + DEBUG_CSR_ENABLE
) (
    input wire i_controller_clk, i_ddr4_clk, i_ref_clk,
    input wire i_rst_n,
    // Wishbone B4
    input wire i_wb_cyc, i_wb_stb, i_wb_we,
    input wire [EXT_ADDR_BITS-1:0] i_wb_addr,
    input wire [WB_DATA_BITS-1:0] i_wb_data,
    input wire [WB_SEL_BITS-1:0] i_wb_sel,
    output wire o_wb_stall, o_wb_ack,
    output wire [WB_DATA_BITS-1:0] o_wb_data,
    // DDR4 SDRAM Interface
    output wire o_ddr4_ck_p, o_ddr4_ck_n,
    output wire o_ddr4_reset_n, o_ddr4_cke, o_ddr4_cs_n, o_ddr4_act_n,
    output wire [16:0] o_ddr4_addr,
    output wire [BA_BITS-1:0] o_ddr4_ba,
    output wire [BG_BITS-1:0] o_ddr4_bg,
    output wire o_ddr4_odt,
    output wire [BYTE_LANES-1:0] o_ddr4_dm_n,
    inout wire [DQ_BITS*BYTE_LANES-1:0] io_ddr4_dq,
    inout wire [BYTE_LANES-1:0] io_ddr4_dqs_p, io_ddr4_dqs_n,
    // BIST (Phase 8)
    input wire i_bist_start,
    output wire o_bist_busy, o_bist_pass, o_bist_fail,
    output wire [31:0] o_bist_correct, o_bist_error,
    // Status
    output wire o_calib_complete, o_calib_error
);

    // DFI 3.1 Internal Bus
    wire [4*17-1:0]             dfi_address;
    wire [4*BA_BITS-1:0]        dfi_bank;
    wire [4*BG_BITS-1:0]        dfi_bg;
    wire [3:0]                  dfi_cs_n, dfi_act_n, dfi_ras_n, dfi_cas_n, dfi_we_n;
    wire [3:0]                  dfi_cke, dfi_odt, dfi_reset_n;
    wire [4*DFI_DATA_WIDTH-1:0] dfi_wrdata;
    wire [3:0]                  dfi_wrdata_en;
    wire [4*(2*BYTE_LANES)-1:0] dfi_wrdata_mask;
    wire [4*DFI_DATA_WIDTH-1:0] dfi_rddata;
    wire [3:0]                  dfi_rddata_valid;
    wire [3:0]                  dfi_rddata_en;
    wire                        dfi_init_start, dfi_init_complete;
    // DFI Training
    wire                        dfi_rdlvl_en, dfi_rdlvl_gate_en;
    wire                        dfi_wrlvl_en, dfi_wrlvl_strobe;
    wire [3:0]                  dfi_lvl_pattern;
    wire                        dfi_lvl_periodic;
    wire [BYTE_LANES-1:0]       dfi_rdlvl_resp, dfi_wrlvl_resp;
    wire                        dfi_rdlvl_req, dfi_rdlvl_gate_req, dfi_wrlvl_req;

    // Phase 8 adds address MSB decode for debug CSR + BIST priority mux
    // For now: direct passthrough
    wire [WB_ADDR_BITS-1:0] ctrl_wb_addr = i_wb_addr[WB_ADDR_BITS-1:0];

    // Stub BIST/prober outputs (Phase 8)
    assign o_bist_busy = 1'b0;
    assign o_bist_pass = 1'b0;
    assign o_bist_fail = 1'b0;
    assign o_bist_correct = 32'b0;
    assign o_bist_error = 32'b0;

    // Controller instantiation
    ddr4_controller #(
        .CONTROLLER_CLK_PERIOD(CONTROLLER_CLK_PERIOD),
        .DDR4_CLK_PERIOD(DDR4_CLK_PERIOD),
        .ROW_BITS(ROW_BITS),
        .COL_BITS(COL_BITS),
        .BA_BITS(BA_BITS),
        .BG_BITS(BG_BITS),
        .DQ_BITS(DQ_BITS),
        .BYTE_LANES(BYTE_LANES),
        .DENSITY(DENSITY),
        .MICRON_SIM(MICRON_SIM),
        .SKIP_CALIB(SKIP_CALIB),
        .ADDR_MAPPING(ADDR_MAPPING),
        .RTT_NOM(RTT_NOM),
        .RTT_WR(RTT_WR),
        .RTT_PARK(RTT_PARK),
        .DRIVE_IMP(DRIVE_IMP),
        .CL(CL),
        .CWL_PARAM(CWL_PARAM)
    ) u_controller (
        .i_controller_clk(i_controller_clk),
        .i_rst_n(i_rst_n),
        // Wishbone
        .i_wb_cyc(i_wb_cyc),
        .i_wb_stb(i_wb_stb),
        .i_wb_we(i_wb_we),
        .i_wb_addr(ctrl_wb_addr),
        .i_wb_data(i_wb_data),
        .i_wb_sel(i_wb_sel),
        .o_wb_stall(o_wb_stall),
        .o_wb_ack(o_wb_ack),
        .o_wb_data(o_wb_data),
        // DFI Control
        .o_dfi_address(dfi_address),
        .o_dfi_bank(dfi_bank),
        .o_dfi_bg(dfi_bg),
        .o_dfi_cs_n(dfi_cs_n),
        .o_dfi_act_n(dfi_act_n),
        .o_dfi_ras_n(dfi_ras_n),
        .o_dfi_cas_n(dfi_cas_n),
        .o_dfi_we_n(dfi_we_n),
        .o_dfi_cke(dfi_cke),
        .o_dfi_odt(dfi_odt),
        .o_dfi_reset_n(dfi_reset_n),
        // DFI Write
        .o_dfi_wrdata(dfi_wrdata),
        .o_dfi_wrdata_en(dfi_wrdata_en),
        .o_dfi_wrdata_mask(dfi_wrdata_mask),
        // DFI Read
        .i_dfi_rddata(dfi_rddata),
        .i_dfi_rddata_valid(dfi_rddata_valid),
        .o_dfi_rddata_en(dfi_rddata_en),
        // DFI Status
        .o_dfi_init_start(dfi_init_start),
        .i_dfi_init_complete(dfi_init_complete),
        // DFI Training
        .o_dfi_rdlvl_en(dfi_rdlvl_en),
        .o_dfi_rdlvl_gate_en(dfi_rdlvl_gate_en),
        .o_dfi_wrlvl_en(dfi_wrlvl_en),
        .o_dfi_wrlvl_strobe(dfi_wrlvl_strobe),
        .o_dfi_lvl_pattern(dfi_lvl_pattern),
        .o_dfi_lvl_periodic(dfi_lvl_periodic),
        .i_dfi_rdlvl_resp(dfi_rdlvl_resp),
        .i_dfi_wrlvl_resp(dfi_wrlvl_resp),
        .i_dfi_rdlvl_req(dfi_rdlvl_req),
        .i_dfi_rdlvl_gate_req(dfi_rdlvl_gate_req),
        .i_dfi_wrlvl_req(dfi_wrlvl_req),
        // Status
        .o_calib_complete(o_calib_complete),
        .o_calib_error(o_calib_error)
    );

    // PHY instantiation
    ddr4_phy #(
        .CONTROLLER_CLK_PERIOD(CONTROLLER_CLK_PERIOD),
        .DDR4_CLK_PERIOD(DDR4_CLK_PERIOD),
        .ROW_BITS(ROW_BITS),
        .BA_BITS(BA_BITS),
        .BG_BITS(BG_BITS),
        .DQ_BITS(DQ_BITS),
        .BYTE_LANES(BYTE_LANES)
    ) u_phy (
        .i_controller_clk(i_controller_clk),
        .i_ddr4_clk(i_ddr4_clk),
        .i_ref_clk(i_ref_clk),
        .i_rst_n(i_rst_n),
        // DFI Control
        .i_dfi_address(dfi_address),
        .i_dfi_bank(dfi_bank),
        .i_dfi_bg(dfi_bg),
        .i_dfi_cs_n(dfi_cs_n),
        .i_dfi_act_n(dfi_act_n),
        .i_dfi_ras_n(dfi_ras_n),
        .i_dfi_cas_n(dfi_cas_n),
        .i_dfi_we_n(dfi_we_n),
        .i_dfi_cke(dfi_cke),
        .i_dfi_odt(dfi_odt),
        .i_dfi_reset_n(dfi_reset_n),
        // DFI Write
        .i_dfi_wrdata(dfi_wrdata),
        .i_dfi_wrdata_en(dfi_wrdata_en),
        .i_dfi_wrdata_mask(dfi_wrdata_mask),
        // DFI Read
        .o_dfi_rddata(dfi_rddata),
        .o_dfi_rddata_valid(dfi_rddata_valid),
        .i_dfi_rddata_en(dfi_rddata_en),
        // DFI Status
        .i_dfi_init_start(dfi_init_start),
        .o_dfi_init_complete(dfi_init_complete),
        // DFI Training
        .i_dfi_rdlvl_en(dfi_rdlvl_en),
        .i_dfi_rdlvl_gate_en(dfi_rdlvl_gate_en),
        .i_dfi_wrlvl_en(dfi_wrlvl_en),
        .i_dfi_wrlvl_strobe(dfi_wrlvl_strobe),
        .i_dfi_lvl_pattern(dfi_lvl_pattern),
        .i_dfi_lvl_periodic(dfi_lvl_periodic),
        .o_dfi_rdlvl_resp(dfi_rdlvl_resp),
        .o_dfi_wrlvl_resp(dfi_wrlvl_resp),
        .o_dfi_rdlvl_req(dfi_rdlvl_req),
        .o_dfi_rdlvl_gate_req(dfi_rdlvl_gate_req),
        .o_dfi_wrlvl_req(dfi_wrlvl_req),
        // DDR4 I/O
        .o_ddr4_ck_p(o_ddr4_ck_p),
        .o_ddr4_ck_n(o_ddr4_ck_n),
        .o_ddr4_reset_n(o_ddr4_reset_n),
        .o_ddr4_cke(o_ddr4_cke),
        .o_ddr4_cs_n(o_ddr4_cs_n),
        .o_ddr4_act_n(o_ddr4_act_n),
        .o_ddr4_addr(o_ddr4_addr),
        .o_ddr4_ba(o_ddr4_ba),
        .o_ddr4_bg(o_ddr4_bg),
        .o_ddr4_odt(o_ddr4_odt),
        .o_ddr4_dm_n(o_ddr4_dm_n),
        .io_ddr4_dq(io_ddr4_dq),
        .io_ddr4_dqs_p(io_ddr4_dqs_p),
        .io_ddr4_dqs_n(io_ddr4_dqs_n),
        .o_idelayctrl_rdy()  // unused for now
    );

endmodule
