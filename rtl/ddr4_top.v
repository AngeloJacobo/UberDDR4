////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_top.v
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  Top module instantiating the DDR4 controller, PHY, and optional
//  BIST/debug prober, connected via a DFI 3.1 internal bus.  Use this as the
//  top module for Wishbone integration.
//
//  Architecture:
//    User WB --+--> [WB Mux] --> ddr4_controller <--DFI--> ddr4_phy --> DDR4
//              |        ^
//              |        |
//              +-> ddr4_prober (BIST engine + debug CSR)
//
//  The Wishbone mux gives BIST priority when active; user transactions
//  are stalled until BIST completes.  Address MSB selects between DRAM
//  access (MSB=0) and debug CSR reads (MSB=1).
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
    // Clock periods in ps
    //   CONTROLLER_CLK_PERIOD = DDR4_CLK_PERIOD * 4 (1/4 rate controller)
    //   DDR4_CLK_PERIOD: 1250=DDR4-1600, 1071=DDR4-1866, 937=DDR4-2133, 833=DDR4-2400
    parameter CONTROLLER_CLK_PERIOD = 3_333,
              DDR4_CLK_PERIOD = 833,
    // DDR4 device data width: 4, 8, or 16
    //   4  = x4  (2 chips per byte lane, no DM, BG_BITS=2)
    //   8  = x8  (1 chip per byte lane, DM enabled, BG_BITS=2)
    //   16 = x16 (1 chip = 2 byte lanes, DM enabled, BG_BITS=1)
              DEVICE_WIDTH = 8,
              ROW_BITS = 16,
              COL_BITS = 10,
    // Number of 8-bit byte lanes (typically 2 for x8, 2 for x16, 2+ for x4)
              BYTE_LANES = 2,
    // Device density in Gb: 2, 4, 8, or 16
              DENSITY = 8,
    // Set to 1 when simulating with Micron DDR4 model (adjusts timing checks)
    parameter[0:0] MICRON_SIM = 0,
    // Address mapping:
    //   0 = sequential {row, bg, ba, col}
    //   1 = BG-interleaved {row, ba, col_hi, bg, col_lo} (recommended)
    parameter[1:0] ADDR_MAPPING = 1,
    // On-die termination (JESD79-4D MR1/MR2/MR5)
    //   RTT_NOM  (MR1 A10:A8): 000=off, 001=RZQ/4, 010=RZQ/2, 011=RZQ/6,
    //                           100=RZQ/1, 101=RZQ/5, 110=RZQ/3, 111=RZQ/7
    //   RTT_WR   (MR2 A11:A9): 000=off, 001=RZQ/2, 010=RZQ/1, 011=Hi-Z,
    //                           100=RZQ/3
    //   RTT_PARK (MR5 A8:A6):  000=off, 001=RZQ/4, 010=RZQ/2, 011=RZQ/6,
    //                           100=RZQ/1, 101=RZQ/5, 110=RZQ/3, 111=RZQ/7
    parameter[2:0] RTT_NOM = 3'b001,
                   RTT_WR = 3'b000,
                   RTT_PARK = 3'b000,
    // Output driver impedance (MR1 A2:A1): 0=RZQ/7 (34ohm), 1=RZQ/5 (48ohm)
    parameter[0:0] DRIVE_IMP = 0,
    // CAS Latency override (0=auto from DDR4_CLK_PERIOD)
    //   Auto values: DDR4-1600=10, DDR4-1866=13, DDR4-2133=15, DDR4-2400=16
    parameter[5:0] CL = 0,
    // CAS Write Latency override (0=auto from DDR4_CLK_PERIOD)
    //   Auto values: DDR4-1600=9, DDR4-1866=10, DDR4-2133=11, DDR4-2400=12
    parameter[4:0] CWL_PARAM = 0,
    // BIST / debug prober configuration
    //   BIST_MODE: 0=disabled, 1=burst sequential only, 2=full (burst+random+alternating)
    parameter[1:0] BIST_MODE = 0,
    // Debug CSR register file: 0=disabled (saves area), 1=enabled
    parameter DEBUG_CSR_ENABLE = 1,
    // Derived from DEVICE_WIDTH -- do not override
    parameter BA_BITS = 2,
              BG_BITS = (DEVICE_WIDTH == 16) ? 1 : 2,
              DQ_BITS = 8,
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
    // Status
    output wire o_init_done, o_init_failed
);

    // -----------------------------------------------------------------
    // DFI 3.1 Internal Bus
    // -----------------------------------------------------------------
    // These wires carry the full DFI 3.1 interface between the
    // controller and PHY.  4-phase command/data (SERDES_RATIO=4).
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
    wire                        dfi_rdlvl_en, dfi_rdlvl_gate_en;
    wire                        dfi_wrlvl_en, dfi_wrlvl_strobe;
    wire [3:0]                  dfi_lvl_pattern;
    wire                        dfi_lvl_periodic;
    wire [BYTE_LANES-1:0]       dfi_rdlvl_resp, dfi_wrlvl_resp;
    wire                        dfi_rdlvl_req, dfi_rdlvl_gate_req, dfi_wrlvl_req;

    // -----------------------------------------------------------------
    // Debug Status Wires (controller -> prober, PHY -> prober)
    // -----------------------------------------------------------------
    wire [3:0]              ctrl_calib_state;
    wire                    ctrl_stage1_pending;
    wire                    ctrl_stage2_pending;
    wire                    ctrl_stage2_we;
    wire                    ctrl_refresh_idle;
    wire [NUM_BANKS-1:0]    ctrl_bank_status;
    wire                    calib_complete;
    wire                    calib_error;
    wire [3:0]              phy_train_state;
    wire [9*BYTE_LANES-1:0] phy_idelay_center;
    wire [9*BYTE_LANES-1:0] phy_wl_tap;
    wire [3*BYTE_LANES-1:0] phy_bitslip;

    // -----------------------------------------------------------------
    // Prober (BIST + CSR) Wires
    // -----------------------------------------------------------------
    wire                     prober_bist_busy;
    wire                     prober_bist_pass;
    wire                     prober_bist_fail;
    wire [31:0]              prober_correct;
    wire [31:0]              prober_error;
    wire                     prober_reset_req; // V2: connect to reset path for auto-recovery
    wire                     prober_wb_cyc;
    wire                     prober_wb_stb;
    wire                     prober_wb_we;
    wire [WB_ADDR_BITS-1:0]  prober_wb_addr;
    wire [WB_DATA_BITS-1:0]  prober_wb_data;
    wire [WB_SEL_BITS-1:0]   prober_wb_sel;
    wire [31:0]              prober_csr_data;

    // -----------------------------------------------------------------
    // WB Address Decode + BIST Priority Mux
    // -----------------------------------------------------------------
    // When BIST is running, it owns the controller WB port; user
    // transactions see stall=1, ack=0.  When idle, the user port
    // passes through directly.
    wire bist_active = prober_bist_busy;

    // Address MSB decode: MSB=1 -> debug CSR, MSB=0 -> DRAM access.
    // Qualified by STB per WB B4 RULE 3.60.
    wire debug_access;
    generate if (DEBUG_CSR_ENABLE) begin : gen_dbg_decode
        assign debug_access = i_wb_cyc && i_wb_stb && i_wb_addr[WB_ADDR_BITS];
    end else begin : gen_no_dbg_decode
        assign debug_access = 1'b0;
    end endgenerate

    wire [WB_ADDR_BITS-1:0] dram_addr = i_wb_addr[WB_ADDR_BITS-1:0];
    wire dram_stb = i_wb_stb && !debug_access;

    // Mux WB to controller: BIST has priority when active
    wire                     ctrl_wb_cyc;
    wire                     ctrl_wb_stb;
    wire                     ctrl_wb_we;
    wire [WB_ADDR_BITS-1:0]  ctrl_wb_addr;
    wire [WB_DATA_BITS-1:0]  ctrl_wb_data;
    wire [WB_SEL_BITS-1:0]   ctrl_wb_sel;
    wire                     ctrl_wb_stall;
    wire                     ctrl_wb_ack;
    wire [WB_DATA_BITS-1:0]  ctrl_wb_rdata;

    assign ctrl_wb_cyc  = bist_active ? prober_wb_cyc  : i_wb_cyc;
    assign ctrl_wb_stb  = bist_active ? prober_wb_stb  : dram_stb;
    assign ctrl_wb_we   = bist_active ? prober_wb_we   : i_wb_we;
    assign ctrl_wb_addr = bist_active ? prober_wb_addr : dram_addr;
    assign ctrl_wb_data = bist_active ? prober_wb_data : i_wb_data;
    assign ctrl_wb_sel  = bist_active ? prober_wb_sel  : i_wb_sel;

    // Route stall/ack to active master, block inactive master
    wire user_wb_stall = bist_active ? 1'b1          : ctrl_wb_stall;
    wire user_wb_ack   = bist_active ? 1'b0          : ctrl_wb_ack;
    wire bist_wb_stall = bist_active ? ctrl_wb_stall : 1'b1;
    wire bist_wb_ack   = bist_active ? ctrl_wb_ack   : 1'b0;

    // Outstanding DRAM request counter -- prevents a CSR ACK from being
    // returned while DRAM read ACKs are still in-flight (would corrupt
    // the WB pipeline ordering).
    reg [3:0] dram_outstanding_q;
    wire dram_request_accepted = i_wb_cyc && dram_stb && !user_wb_stall;
    wire dram_ack_returned     = user_wb_ack;

    always @(posedge i_controller_clk) begin
        if (!i_rst_n)
            dram_outstanding_q <= 4'd0;
        else
            dram_outstanding_q <= dram_outstanding_q
                                + {3'd0, dram_request_accepted}
                                - {3'd0, dram_ack_returned};
    end

    wire csr_blocked = (dram_outstanding_q != 0);
    wire csr_ready   = debug_access && !csr_blocked;

    // Final output mux
    assign o_wb_stall = debug_access ? csr_blocked   : user_wb_stall;
    assign o_wb_ack   = csr_ready    ? 1'b1          : user_wb_ack;
    assign o_wb_data  = csr_ready    ? {{(WB_DATA_BITS-32){1'b0}}, prober_csr_data}
                                     : ctrl_wb_rdata;

    // -----------------------------------------------------------------
    // BIST Auto-Start + Init Status
    // -----------------------------------------------------------------
    // On rising edge of calib_complete (if BIST_MODE != 0), auto-start
    // fires the BIST engine once.  init_done and init_failed are sticky:
    //   - init_done:   calibration OK and (BIST passed or BIST disabled)
    //   - init_failed: calibration error OR BIST failure
    reg calib_complete_q;
    always @(posedge i_controller_clk) begin
        if (!i_rst_n)
            calib_complete_q <= 1'b0;
        else
            calib_complete_q <= calib_complete;
    end

    wire bist_auto_start = calib_complete && !calib_complete_q && (BIST_MODE != 0);
    wire bist_start      = bist_auto_start;

    wire csr_we = csr_ready && i_wb_we;

    reg init_done_q, init_failed_q;
    always @(posedge i_controller_clk) begin
        if (!i_rst_n) begin
            init_done_q   <= 1'b0;
            init_failed_q <= 1'b0;
        end else begin
            if (calib_error)
                init_failed_q <= 1'b1;
            if (!init_done_q && !init_failed_q) begin
                if (calib_complete && (BIST_MODE == 0))
                    init_done_q <= 1'b1;
                if (calib_complete && BIST_MODE != 0 && prober_bist_pass)
                    init_done_q <= 1'b1;
                if (calib_complete && BIST_MODE != 0 && prober_bist_fail)
                    init_failed_q <= 1'b1;
            end
        end
    end

    assign o_init_done   = init_done_q;
    assign o_init_failed = init_failed_q;

    // -----------------------------------------------------------------
    // Controller Instantiation
    // -----------------------------------------------------------------
    // Handles JEDEC DDR4 initialization, refresh, bank tracking, and
    // read/write scheduling.  Exposes a Wishbone B4 slave port and
    // drives the DFI 3.1 interface toward the PHY.
    ddr4_controller #(
        .CONTROLLER_CLK_PERIOD(CONTROLLER_CLK_PERIOD),
        .DDR4_CLK_PERIOD(DDR4_CLK_PERIOD),
        .DEVICE_WIDTH(DEVICE_WIDTH),
        .ROW_BITS(ROW_BITS),
        .COL_BITS(COL_BITS),
        .BYTE_LANES(BYTE_LANES),
        .DENSITY(DENSITY),
        .MICRON_SIM(MICRON_SIM),
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
        // Wishbone (muxed)
        .i_wb_cyc(ctrl_wb_cyc),
        .i_wb_stb(ctrl_wb_stb),
        .i_wb_we(ctrl_wb_we),
        .i_wb_addr(ctrl_wb_addr),
        .i_wb_data(ctrl_wb_data),
        .i_wb_sel(ctrl_wb_sel),
        .o_wb_stall(ctrl_wb_stall),
        .o_wb_ack(ctrl_wb_ack),
        .o_wb_data(ctrl_wb_rdata),
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
        .o_calib_complete(calib_complete),
        .o_calib_error(calib_error),
        .o_calib_state(ctrl_calib_state),
        .o_stage1_pending(ctrl_stage1_pending),
        .o_stage2_pending(ctrl_stage2_pending),
        .o_stage2_we(ctrl_stage2_we),
        .o_refresh_idle(ctrl_refresh_idle),
        .o_bank_status(ctrl_bank_status)
    );

    // -----------------------------------------------------------------
    // PHY Instantiation
    // -----------------------------------------------------------------
    // Xilinx UltraScale+ PHY: ISERDESE3/OSERDESE3, IDELAYE3/ODELAYE3,
    // write-leveling, read gate training.  Directly drives DDR4 I/O.
    ddr4_phy #(
        .CONTROLLER_CLK_PERIOD(CONTROLLER_CLK_PERIOD),
        .DDR4_CLK_PERIOD(DDR4_CLK_PERIOD),
        .DEVICE_WIDTH(DEVICE_WIDTH),
        .ROW_BITS(ROW_BITS),
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
        .o_idelayctrl_rdy(),
        .o_phy_state(phy_train_state),
        .o_phy_idelay_center(phy_idelay_center),
        .o_phy_wl_tap(phy_wl_tap),
        .o_phy_bitslip(phy_bitslip)
    );

    // -----------------------------------------------------------------
    // Prober Instantiation (BIST + Debug CSR)
    // -----------------------------------------------------------------
    // Combined BIST engine and debug register file.  See ddr4_prober.v
    // for the CSR map and BIST phase descriptions.
    ddr4_prober #(
        .WB_ADDR_BITS(WB_ADDR_BITS),
        .WB_DATA_BITS(WB_DATA_BITS),
        .WB_SEL_BITS(WB_SEL_BITS),
        .BYTE_LANES(BYTE_LANES),
        .NUM_BANKS(NUM_BANKS),
        .ROW_BITS(ROW_BITS),
        .MICRON_SIM(MICRON_SIM),
        .BIST_MODE(BIST_MODE),
        .DEBUG_CSR_ENABLE(DEBUG_CSR_ENABLE)
    ) u_prober (
        .i_clk(i_controller_clk),
        .i_rst_n(i_rst_n),
        .i_start(bist_start),
        .i_calib_complete(calib_complete),
        .o_bist_busy(prober_bist_busy),
        .o_bist_pass(prober_bist_pass),
        .o_bist_fail(prober_bist_fail),
        .o_correct_count(prober_correct),
        .o_error_count(prober_error),
        .o_bist_reset_req(prober_reset_req),
        .o_wb_cyc(prober_wb_cyc),
        .o_wb_stb(prober_wb_stb),
        .o_wb_we(prober_wb_we),
        .o_wb_addr(prober_wb_addr),
        .o_wb_data(prober_wb_data),
        .o_wb_sel(prober_wb_sel),
        .i_wb_stall(bist_wb_stall),
        .i_wb_ack(bist_wb_ack),
        .i_wb_data(ctrl_wb_rdata),
        .i_csr_sel(i_wb_addr[3:0]),
        .i_csr_we(csr_we),
        .i_csr_wdata(i_wb_data[31:0]),
        .o_csr_data(prober_csr_data),
        .i_calib_state(ctrl_calib_state),
        .i_stage1_pending(ctrl_stage1_pending),
        .i_stage2_pending(ctrl_stage2_pending),
        .i_stage2_we(ctrl_stage2_we),
        .i_refresh_idle(ctrl_refresh_idle),
        .i_bank_status(ctrl_bank_status),
        .i_phy_state(phy_train_state),
        .i_phy_idelay_center(phy_idelay_center),
        .i_phy_wl_tap(phy_wl_tap),
        .i_phy_bitslip(phy_bitslip)
    );

endmodule
