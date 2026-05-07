////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_top_axi.v
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  Top module which instantiates ddr4_top and an AXI4-to-Wishbone
//  bridge (ZipCPU axim2wbsp).  Use this as the top module when integrating
//  UberDDR4 with an AXI4 interconnect.
//
//  The AXI byte address is wider than the WB word address by AXI_LSBS
//  bits (= log2(data_width_bytes)).  The bridge strips these LSBs and
//  translates AXI bursts into pipelined WB transactions.
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

module ddr4_top_axi #(
    parameter      CONTROLLER_CLK_PERIOD = 3_336,
                   DDR4_CLK_PERIOD       = 834,
                   ROW_BITS    = 16,
                   COL_BITS    = 10,
                   BA_BITS     = 2,
                   BG_BITS     = 2,
                   DQ_BITS     = 8,
                   BYTE_LANES  = 2,
                   DENSITY     = 8,
                   AXI_ID_WIDTH = 4,
    parameter[0:0] MICRON_SIM  = 0,
    parameter      ADDR_MAPPING = 1,
    parameter[1:0] BIST_MODE  = 1,
    parameter       DEBUG_CSR_ENABLE = 1,
    // Derived parameters -- do not override
    parameter
                   SERDES_RATIO  = 4,
                   WB_DATA_BITS  = DQ_BITS * BYTE_LANES * 2 * SERDES_RATIO,
                   WB_SEL_BITS   = WB_DATA_BITS / 8,
                   COL_LOW       = $clog2(SERDES_RATIO * 2 * DQ_BITS * BYTE_LANES / 8),
                   WB_ADDR_BITS  = ROW_BITS + BG_BITS + BA_BITS + COL_BITS - COL_LOW,
                   EXT_ADDR_BITS = WB_ADDR_BITS + DEBUG_CSR_ENABLE,
                   // AXI_LSBS: number of byte-offset bits stripped by the
                   // bridge (AXI uses byte addresses, WB uses word addresses)
                   AXI_LSBS      = $clog2(WB_DATA_BITS) - 3,
                   AXI_ADDR_WIDTH = EXT_ADDR_BITS + AXI_LSBS,
                   AXI_DATA_WIDTH = WB_DATA_BITS
) (
    input wire i_controller_clk,
    input wire i_ddr4_clk,
    input wire i_ref_clk,
    input wire i_rst_n,

    // AXI4 Slave Interface -- Write Address Channel
    input  wire                       s_axi_awvalid,
    output wire                       s_axi_awready,
    input  wire [AXI_ID_WIDTH-1:0]    s_axi_awid,
    input  wire [AXI_ADDR_WIDTH-1:0]  s_axi_awaddr,
    input  wire [7:0]                 s_axi_awlen,
    input  wire [2:0]                 s_axi_awsize,
    input  wire [1:0]                 s_axi_awburst,
    input  wire [0:0]                 s_axi_awlock,
    input  wire [3:0]                 s_axi_awcache,
    input  wire [2:0]                 s_axi_awprot,
    input  wire [3:0]                 s_axi_awqos,
    // AXI4 Slave Interface -- Write Data Channel
    input  wire                       s_axi_wvalid,
    output wire                       s_axi_wready,
    input  wire [AXI_DATA_WIDTH-1:0]  s_axi_wdata,
    input  wire [AXI_DATA_WIDTH/8-1:0] s_axi_wstrb,
    input  wire                       s_axi_wlast,
    // AXI4 Slave Interface -- Write Response Channel
    output wire                       s_axi_bvalid,
    input  wire                       s_axi_bready,
    output wire [AXI_ID_WIDTH-1:0]    s_axi_bid,
    output wire [1:0]                 s_axi_bresp,
    // AXI4 Slave Interface -- Read Address Channel
    input  wire                       s_axi_arvalid,
    output wire                       s_axi_arready,
    input  wire [AXI_ID_WIDTH-1:0]    s_axi_arid,
    input  wire [AXI_ADDR_WIDTH-1:0]  s_axi_araddr,
    input  wire [7:0]                 s_axi_arlen,
    input  wire [2:0]                 s_axi_arsize,
    input  wire [1:0]                 s_axi_arburst,
    input  wire [0:0]                 s_axi_arlock,
    input  wire [3:0]                 s_axi_arcache,
    input  wire [2:0]                 s_axi_arprot,
    input  wire [3:0]                 s_axi_arqos,
    // AXI4 Slave Interface -- Read Data Channel
    output wire                       s_axi_rvalid,
    input  wire                       s_axi_rready,
    output wire [AXI_ID_WIDTH-1:0]    s_axi_rid,
    output wire [AXI_DATA_WIDTH-1:0]  s_axi_rdata,
    output wire                       s_axi_rlast,
    output wire [1:0]                 s_axi_rresp,

    // DDR4 Physical Interface
    output wire                       o_ddr4_ck_p,
    output wire                       o_ddr4_ck_n,
    output wire                       o_ddr4_reset_n,
    output wire                       o_ddr4_cke,
    output wire                       o_ddr4_cs_n,
    output wire                       o_ddr4_act_n,
    output wire [16:0]                o_ddr4_addr,
    output wire [BA_BITS-1:0]         o_ddr4_ba,
    output wire [BG_BITS-1:0]         o_ddr4_bg,
    output wire                       o_ddr4_odt,
    output wire [BYTE_LANES-1:0]      o_ddr4_dm_n,
    inout  wire [DQ_BITS*BYTE_LANES-1:0] io_ddr4_dq,
    inout  wire [BYTE_LANES-1:0]      io_ddr4_dqs_p,
    inout  wire [BYTE_LANES-1:0]      io_ddr4_dqs_n,

    // Status
    output wire o_init_done,
    output wire o_init_failed
);

    // --- Internal Wishbone bus ---
    wire                     wb_cyc;
    wire                     wb_stb;
    wire                     wb_we;
    wire [EXT_ADDR_BITS-1:0] wb_addr;
    wire [WB_DATA_BITS-1:0]  wb_wdata;
    wire [WB_SEL_BITS-1:0]   wb_sel;
    wire                     wb_stall;
    wire                     wb_ack;
    wire [WB_DATA_BITS-1:0]  wb_rdata;

    // --- DDR4 Controller (Wishbone top-level, see ddr4_top.v) ---
    ddr4_top #(
        .CONTROLLER_CLK_PERIOD (CONTROLLER_CLK_PERIOD),
        .DDR4_CLK_PERIOD       (DDR4_CLK_PERIOD),
        .ROW_BITS              (ROW_BITS),
        .COL_BITS              (COL_BITS),
        .BA_BITS               (BA_BITS),
        .BG_BITS               (BG_BITS),
        .DQ_BITS               (DQ_BITS),
        .BYTE_LANES            (BYTE_LANES),
        .DENSITY               (DENSITY),
        .MICRON_SIM            (MICRON_SIM),
        .ADDR_MAPPING          (ADDR_MAPPING),
        .BIST_MODE             (BIST_MODE),
        .DEBUG_CSR_ENABLE      (DEBUG_CSR_ENABLE)
    ) u_ddr4_top (
        .i_controller_clk (i_controller_clk),
        .i_ddr4_clk       (i_ddr4_clk),
        .i_ref_clk        (i_ref_clk),
        .i_rst_n          (i_rst_n),
        .i_wb_cyc          (wb_cyc),
        .i_wb_stb          (wb_stb),
        .i_wb_we           (wb_we),
        .i_wb_addr         (wb_addr),
        .i_wb_data         (wb_wdata),
        .i_wb_sel          (wb_sel),
        .o_wb_stall        (wb_stall),
        .o_wb_ack          (wb_ack),
        .o_wb_data         (wb_rdata),
        .o_ddr4_ck_p       (o_ddr4_ck_p),
        .o_ddr4_ck_n       (o_ddr4_ck_n),
        .o_ddr4_reset_n    (o_ddr4_reset_n),
        .o_ddr4_cke        (o_ddr4_cke),
        .o_ddr4_cs_n       (o_ddr4_cs_n),
        .o_ddr4_act_n      (o_ddr4_act_n),
        .o_ddr4_addr       (o_ddr4_addr),
        .o_ddr4_ba         (o_ddr4_ba),
        .o_ddr4_bg         (o_ddr4_bg),
        .o_ddr4_odt        (o_ddr4_odt),
        .o_ddr4_dm_n       (o_ddr4_dm_n),
        .io_ddr4_dq        (io_ddr4_dq),
        .io_ddr4_dqs_p     (io_ddr4_dqs_p),
        .io_ddr4_dqs_n     (io_ddr4_dqs_n),
        .o_init_done       (o_init_done),
        .o_init_failed     (o_init_failed)
    );

    // --- AXI4-to-Wishbone Bridge (ZipCPU axim2wbsp) ---
    // Translates full AXI4 (bursts, IDs, etc.) into pipelined WB B4
    // transactions.  LGFIFO=5 gives 32-entry deep request FIFOs.
    axim2wbsp #(
        .C_AXI_ID_WIDTH    (AXI_ID_WIDTH),
        .C_AXI_DATA_WIDTH  (AXI_DATA_WIDTH),
        .C_AXI_ADDR_WIDTH  (AXI_ADDR_WIDTH),
        .LGFIFO            (5),
        .OPT_SWAP_ENDIANNESS (0),
        .OPT_READONLY      (0),
        .OPT_WRITEONLY     (0)
    ) u_axim2wbsp (
        .S_AXI_ACLK    (i_controller_clk),
        .S_AXI_ARESETN (i_rst_n),
        .S_AXI_AWVALID (s_axi_awvalid),
        .S_AXI_AWREADY (s_axi_awready),
        .S_AXI_AWID    (s_axi_awid),
        .S_AXI_AWADDR  (s_axi_awaddr),
        .S_AXI_AWLEN   (s_axi_awlen),
        .S_AXI_AWSIZE  (s_axi_awsize),
        .S_AXI_AWBURST (s_axi_awburst),
        .S_AXI_AWLOCK  (s_axi_awlock),
        .S_AXI_AWCACHE (s_axi_awcache),
        .S_AXI_AWPROT  (s_axi_awprot),
        .S_AXI_AWQOS   (s_axi_awqos),
        .S_AXI_WVALID  (s_axi_wvalid),
        .S_AXI_WREADY  (s_axi_wready),
        .S_AXI_WDATA   (s_axi_wdata),
        .S_AXI_WSTRB   (s_axi_wstrb),
        .S_AXI_WLAST   (s_axi_wlast),
        .S_AXI_BVALID  (s_axi_bvalid),
        .S_AXI_BREADY  (s_axi_bready),
        .S_AXI_BID     (s_axi_bid),
        .S_AXI_BRESP   (s_axi_bresp),
        .S_AXI_ARVALID (s_axi_arvalid),
        .S_AXI_ARREADY (s_axi_arready),
        .S_AXI_ARID    (s_axi_arid),
        .S_AXI_ARADDR  (s_axi_araddr),
        .S_AXI_ARLEN   (s_axi_arlen),
        .S_AXI_ARSIZE  (s_axi_arsize),
        .S_AXI_ARBURST (s_axi_arburst),
        .S_AXI_ARLOCK  (s_axi_arlock),
        .S_AXI_ARCACHE (s_axi_arcache),
        .S_AXI_ARPROT  (s_axi_arprot),
        .S_AXI_ARQOS   (s_axi_arqos),
        .S_AXI_RVALID  (s_axi_rvalid),
        .S_AXI_RREADY  (s_axi_rready),
        .S_AXI_RID     (s_axi_rid),
        .S_AXI_RDATA   (s_axi_rdata),
        .S_AXI_RLAST   (s_axi_rlast),
        .S_AXI_RRESP   (s_axi_rresp),
        .o_reset       (),
        .o_wb_cyc      (wb_cyc),
        .o_wb_stb      (wb_stb),
        .o_wb_we       (wb_we),
        .o_wb_addr     (wb_addr),
        .o_wb_data     (wb_wdata),
        .o_wb_sel      (wb_sel),
        .i_wb_stall    (wb_stall),
        .i_wb_ack      (wb_ack),
        .i_wb_data     (wb_rdata),
        .i_wb_err      (1'b0)
    );

endmodule