////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_top.v
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  Top module instantiating the DDR4 controller, PHY, and optional
//  BIST/debug prober, connected via the implemented DFI 3.1 subset. Use this as the
//  top module for Wishbone integration.
//
//  Architecture:
//    User WB ----+--> [WB Mux] --> ddr4_controller <--DFI--> selected PHY --> DDR4
//                |        ^
//                |        |
//                +-> ddr4_prober (BIST engine)
//
//    Debug WB -------> ddr4_prober (CSR register file, when enabled)
//
//  Two independent Wishbone ports:
//    - DRAM port: pipelined, used for data traffic and BIST.  BIST has
//      priority when active; user transactions stalled until complete. Drain
//      outstanding application requests before requesting runtime BIST/reset.
//    - Debug CSR port: pipelined, zero-wait-state.  Accessible at all
//      times when enabled, regardless of calibration state or BIST activity.
//      Tie unused debug inputs low; see docs/DEBUGGING.md for CONTROL caveats.
//
//  Integration, clock contracts, packing and limits: docs/INTEGRATION.md.
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
    // Simulation only: shortens init waits and BIST address counter; never enable in hardware
    parameter[0:0] MICRON_SIM = 0,
    // Address mapping:
    //   0 = WB burst-word address {row, bg, ba, col_upper}
    //   1 = WB burst-word address {row, ba, col_upper, bg}; low 3 DRAM column bits are zero
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
    //   Auto values: DDR4-1600=12, DDR4-1866=14, DDR4-2133=16, DDR4-2400=18
    parameter[5:0] CL = 0,
    // CAS Write Latency override (0=auto from DDR4_CLK_PERIOD)
    //   Auto values: DDR4-1600=9, DDR4-1866=10, DDR4-2133=11, DDR4-2400=12
    parameter[4:0] CWL = 0,
    // PHY implementation:
    //   0 = component PHY (rtl/ddr4_phy.v, Xilinx component SERDES/delay primitives)
    //   1 = native UltraScale/UltraScale+ BITSLICE PHY
    //
    // The controller-facing timing compensation is derived from this choice.
    // Keeping those values private prevents a PHY/controller mismatch caused
    // by a board-level instantiation overriding only one of them.
    parameter[0:0] PHY_IMPL = 0,
    // BIST / debug prober configuration
    //   BIST_MODE: 0=disabled, 1=partitioned counters (1/4, 1/2, 1/4), 2=full range per phase
    parameter[1:0] BIST_MODE = 1,
    //   BIST_DM_TEST: 0=full-word burst writes, 1=one write per byte position of the full WB burst (stress DM)
    //   Defaults off for x4 devices, which lack the implemented DM_n mask function
    parameter[0:0] BIST_DM_TEST = (DEVICE_WIDTH == 4) ? 0 : 1,
    // First-error diagnostic: rereads, optional native TX adaptation and retests.
    // Destructive bring-up mode; disabled by default. See docs/DEBUGGING.md.
    parameter[0:0] BIST_REREAD_DIAG = 0,
    // Debug CSR register file: 0=disabled (saves area), 1=enabled
    parameter DEBUG_CSR_ENABLE = 1,
    // Derived from DEVICE_WIDTH -- do not override
    parameter BA_BITS = 2,
              BG_BITS = (DEVICE_WIDTH == 16) ? 1 : 2,
              DQ_BITS = 8,
    // Native-PHY package topology.  These are ignored by the component PHY.
    // One ACMD byte is {physical_nibble[4:0], position[2:0]}; one DQ nibble
    // is {upper_nibble, position[2:0]}.  All ones selects the canonical
    // simulation layout.  Hardware wrappers must use their XDC pin topology.
    // Only referenced inside gen_native_phy, so the default PHY_IMPL=0
    // elaboration reports them unused; the -GPHY_IMPL=1 lint pass covers them.
    /* verilator lint_off UNUSEDPARAM */
              PHY_ACMD_NIBBLE_COUNT = 0,
    parameter [255:0] PHY_ACMD_PIN_MAP = {256{1'b1}},
    parameter [4*DQ_BITS*BYTE_LANES-1:0] PHY_DQ_PIN_MAP =
              {4*DQ_BITS*BYTE_LANES{1'b1}},
              PHY_PLL_COUNT = 1,
    parameter [95:0] PHY_ACMD_PLL_MAP = 96'd0,
    parameter [3*BYTE_LANES-1:0] PHY_BYTE_PLL_MAP =
              {3*BYTE_LANES{1'b0}},
    /* verilator lint_on UNUSEDPARAM */
    // Derived (for port widths)
    parameter SERDES_RATIO = 4,
              NUM_BG = (1 << BG_BITS),
              NUM_BANKS = NUM_BG * (1 << BA_BITS),
              DFI_DATA_WIDTH = 2 * DQ_BITS * BYTE_LANES,
              WB_DATA_BITS = DQ_BITS * BYTE_LANES * 2 * SERDES_RATIO,
              WB_SEL_BITS = WB_DATA_BITS / 8,
              COL_LOW = $clog2(SERDES_RATIO * 2),
              WB_ADDR_BITS = ROW_BITS + BG_BITS + BA_BITS + COL_BITS - COL_LOW
) (
    // Component: controller=CK/4, DDR4 clock=CK, ref=300 MHz.
    // Native: controller drives local PLLs, ref is RIU (normally controller/2);
    // i_ddr4_clk is unused. Native controller/RIU must share an MMCM and phase.
    input wire i_controller_clk, i_ddr4_clk, i_ref_clk,
    input wire i_rst_n,
    // Wishbone B4 - DRAM data path (pipelined)
    input wire i_wb_cyc, i_wb_stb, i_wb_we,
    // One address selects a full BL8 burst; there is no CSR-select address bit.
    input wire [WB_ADDR_BITS-1:0] i_wb_addr,
    input wire [WB_DATA_BITS-1:0] i_wb_data,
    input wire [WB_SEL_BITS-1:0] i_wb_sel,
    output wire o_wb_stall, o_wb_ack,
    output wire [WB_DATA_BITS-1:0] o_wb_data,
    // Wishbone B4 - Debug CSR port (pipelined, independent of DRAM path).
    // Available during calibration when enabled. Tie unused inputs to zero.
    input wire i_wb_dbg_cyc, i_wb_dbg_stb, i_wb_dbg_we,
    input wire [3:0] i_wb_dbg_addr,
    input wire [31:0] i_wb_dbg_data,
    input wire [3:0] i_wb_dbg_sel,
    output wire o_wb_dbg_stall,
    output wire o_wb_dbg_ack,
    output wire [31:0] o_wb_dbg_data,
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

    // Native PHY timing is architectural, rather than board-specific:
    // one 1:4 TX word is registered before serialization, and the first CKE
    // transition needs nine controller clocks to fill the CA serializer.
    // The component PHY has neither latency.
    // Native UltraScale BITSLICE uses one controller word to register DQ/DQS
    // and one earlier word to serialize the TBYTE preamble/ownership envelope.
    // Advertising two DFI clocks lets the controller present the write bundle
    // early enough for both native stages while preserving JEDEC CWL at pins.
    localparam [1:0] PHY_TPHY_WRLAT   = PHY_IMPL ? 2'd2 : 2'd0;
    localparam [3:0] PHY_TPHY_INIT_LAT = PHY_IMPL ? 4'd9 : 4'd0;
    // The native PHY exhaustively searches read-latency/coarse/fine tuples
    // during read leveling and coarse/fine tuples per lane during write
    // leveling. Keep the component-PHY watchdogs unchanged, but allow both
    // native searches to finish before the controller declares a timeout.
    localparam integer PHY_T_RDLVL_MAX = PHY_IMPL ? 524288 : 65536;
    localparam integer PHY_T_WRLVL_MAX = PHY_IMPL ? 524288 : 65536;

    // -----------------------------------------------------------------
    // DFI 3.1 Internal Bus
    // -----------------------------------------------------------------
    // These wires carry the full DFI 3.1 interface between the
    // controller and PHY.  SERDES_RATIO-phase command/data.
    wire [SERDES_RATIO*17-1:0]             dfi_address;
    wire [SERDES_RATIO*BA_BITS-1:0]        dfi_bank;
    wire [SERDES_RATIO*BG_BITS-1:0]        dfi_bg;
    wire [SERDES_RATIO-1:0]                dfi_cs_n, dfi_act_n, dfi_ras_n, dfi_cas_n, dfi_we_n;
    wire [SERDES_RATIO-1:0]                dfi_cke, dfi_odt, dfi_reset_n;
    wire [SERDES_RATIO*DFI_DATA_WIDTH-1:0] dfi_wrdata;
    wire [SERDES_RATIO-1:0]                dfi_wrdata_en;
    wire [SERDES_RATIO*(2*BYTE_LANES)-1:0] dfi_wrdata_mask;
    wire [SERDES_RATIO*DFI_DATA_WIDTH-1:0] dfi_rddata;
    wire [SERDES_RATIO-1:0]                dfi_rddata_valid;
    wire [SERDES_RATIO-1:0]                dfi_rddata_en;
    wire                                   dfi_init_start, dfi_init_complete;
    wire                                   dfi_rdlvl_en, dfi_rdlvl_gate_en;
    wire                                   dfi_wrlvl_en, dfi_wrlvl_strobe;
    wire [SERDES_RATIO-1:0]                dfi_lvl_pattern;
    wire                                   dfi_lvl_periodic;
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
    wire [4*BYTE_LANES-1:0] phy_bitslip;
    wire [BYTE_LANES-1:0]   phy_train_fail_gate;
    wire [BYTE_LANES-1:0]   phy_train_fail_eye;
    wire [BYTE_LANES-1:0]   phy_train_fail_wl;
    wire [9*BYTE_LANES-1:0] phy_best_width;
    wire [9*BYTE_LANES-1:0] phy_best_start;
    wire [9*BYTE_LANES-1:0] phy_wl_dq_tap;
    wire [9*BYTE_LANES-1:0] phy_dqs_initial_tap;
    wire [BYTE_LANES-1:0]   phy_rd_lat_extra;
    wire                     phy_en_vtc;
    // Driven only by the native PHY's TX-diagnostic path.
    /* verilator lint_off UNUSEDSIGNAL */
    wire                     phy_tx_diag_req;
    wire [7:0]               phy_tx_diag_dq;
    wire [8:0]               phy_tx_diag_tap;
    /* verilator lint_on UNUSEDSIGNAL */
    wire                     phy_tx_diag_ack;
    wire                     phy_tx_diag_error;
    wire [8:0]               phy_tx_diag_current_tap;
    wire [8:0]               phy_tx_diag_previous_tap;
    wire [5:0]              ctrl_instruction_address;
    wire                    ctrl_pause_counter;
    wire                    ctrl_reset_done;
    wire                    ctrl_pipe_stall;
    wire [1:0]             ctrl_calib_retry_count;

    // -----------------------------------------------------------------
    // Prober (BIST + CSR) Wires
    // -----------------------------------------------------------------
    wire                     prober_bist_busy;
    wire                     prober_bist_failed_reset_req;
    wire                     prober_soft_reset_req;
    wire                     prober_wb_cyc;
    wire                     prober_wb_stb;
    wire                     prober_wb_we;
    wire [WB_ADDR_BITS-1:0]  prober_wb_addr;
    wire [WB_DATA_BITS-1:0]  prober_wb_data;
    wire [WB_SEL_BITS-1:0]   prober_wb_sel;


    // -----------------------------------------------------------------
    // DRAM WB Mux: BIST has priority when active
    // -----------------------------------------------------------------
    // When BIST is running, it owns the controller WB port; user
    // transactions see stall=1, ack=0.  When idle, the user port
    // passes through directly.
    wire bist_active = prober_bist_busy;

    // Reset to controller + PHY:
    // external reset + CSR-triggered soft reset + BIST-failure-triggered soft-reset (if enabled in AUTO_RESET_EN)
    /* verilator lint_off SYNCASYNCNET */
    wire internal_rst_n = i_rst_n && !prober_soft_reset_req && !prober_bist_failed_reset_req;
    /* verilator lint_on SYNCASYNCNET */

    wire                     ctrl_wb_cyc;
    wire                     ctrl_wb_stb;
    wire                     ctrl_wb_we;
    wire [WB_ADDR_BITS-1:0]  ctrl_wb_addr;
    wire [WB_DATA_BITS-1:0]  ctrl_wb_data;
    wire [WB_SEL_BITS-1:0]   ctrl_wb_sel;
    wire                     ctrl_wb_stall;
    wire                     ctrl_wb_ack;
    wire [WB_DATA_BITS-1:0]  ctrl_wb_rdata;

    // Route prober or user WB signals to controller depending on BIST activity
    assign ctrl_wb_cyc  = bist_active ? prober_wb_cyc  : i_wb_cyc;
    assign ctrl_wb_stb  = bist_active ? prober_wb_stb  : i_wb_stb;
    assign ctrl_wb_we   = bist_active ? prober_wb_we   : i_wb_we;
    assign ctrl_wb_addr = bist_active ? prober_wb_addr : i_wb_addr;
    assign ctrl_wb_data = bist_active ? prober_wb_data : i_wb_data;
    assign ctrl_wb_sel  = bist_active ? prober_wb_sel  : i_wb_sel;

    // Route stall/ack to active master, block inactive master
    wire bist_wb_stall = bist_active ? ctrl_wb_stall : 1'b1;
    wire bist_wb_ack   = bist_active ? ctrl_wb_ack   : 1'b0;

    assign o_wb_stall = bist_active ? 1'b1          : ctrl_wb_stall;
    assign o_wb_ack   = bist_active ? 1'b0          : ctrl_wb_ack;
    assign o_wb_data  = ctrl_wb_rdata;



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
        .CWL(CWL),
        .TPHY_WRLAT(PHY_TPHY_WRLAT),
        .TPHY_INIT_LAT(PHY_TPHY_INIT_LAT),
        .POST_WL_READ_TRAINING(PHY_IMPL),
        .T_RDLVL_MAX(PHY_T_RDLVL_MAX),
        .T_WRLVL_MAX(PHY_T_WRLVL_MAX)
    ) u_controller (
        .i_controller_clk(i_controller_clk),
        .i_rst_n(internal_rst_n),
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
        .o_bank_status(ctrl_bank_status),
        .o_instruction_address(ctrl_instruction_address),
        .o_pause_counter(ctrl_pause_counter),
        .o_reset_done(ctrl_reset_done),
        .o_pipe_stall(ctrl_pipe_stall),
        .o_calib_retry_count(ctrl_calib_retry_count)
    );

    // -----------------------------------------------------------------
    // PHY Instantiation
    // -----------------------------------------------------------------
    // Select the PHY at elaboration.  Both implementations use the same DFI
    // interface and export identical prober status signals.
    generate
    if (!PHY_IMPL) begin : gen_component_phy
    ddr4_phy #(
        .CONTROLLER_CLK_PERIOD(CONTROLLER_CLK_PERIOD),
        .DDR4_CLK_PERIOD(DDR4_CLK_PERIOD),
        .DEVICE_WIDTH(DEVICE_WIDTH),
        .BYTE_LANES(BYTE_LANES)
    ) u_phy (
        .i_controller_clk(i_controller_clk),
        .i_ddr4_clk(i_ddr4_clk),
        .i_ref_clk(i_ref_clk),
        .i_rst_n(internal_rst_n),
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
        .o_phy_state(phy_train_state),
        .o_phy_idelay_center(phy_idelay_center),
        .o_phy_wl_tap(phy_wl_tap),
        .o_phy_bitslip(phy_bitslip),
        .o_phy_train_fail_gate(phy_train_fail_gate),
        .o_phy_train_fail_eye(phy_train_fail_eye),
        .o_phy_train_fail_wl(phy_train_fail_wl),
        .o_phy_best_width(phy_best_width),
        .o_phy_best_start(phy_best_start),
        .o_phy_wl_dq_tap(phy_wl_dq_tap),
        .o_phy_dqs_initial_tap(phy_dqs_initial_tap),
        .o_phy_rd_lat_extra(phy_rd_lat_extra),
        .o_phy_en_vtc(phy_en_vtc)
    );
    // Component mode has no native BITSLICE TX-delay diagnostic.  The prober
    // checks the support flag and never asserts a request in this branch.
    assign phy_tx_diag_ack = 1'b0;
    assign phy_tx_diag_error = 1'b0;
    assign phy_tx_diag_current_tap = 9'd0;
    assign phy_tx_diag_previous_tap = 9'd0;
    end else begin : gen_native_phy
    ddr4_phy_native_adapter #(
        .CONTROLLER_CLK_PERIOD(CONTROLLER_CLK_PERIOD),
        .DDR4_CLK_PERIOD(DDR4_CLK_PERIOD),
        .DEVICE_WIDTH(DEVICE_WIDTH),
        .BYTE_LANES(BYTE_LANES),
        .ACMD_NIBBLE_COUNT(PHY_ACMD_NIBBLE_COUNT),
        .ACMD_PIN_MAP(PHY_ACMD_PIN_MAP),
        .DQ_PIN_MAP(PHY_DQ_PIN_MAP),
        .PLL_COUNT(PHY_PLL_COUNT),
        .ACMD_PLL_MAP(PHY_ACMD_PLL_MAP),
        .BYTE_PLL_MAP(PHY_BYTE_PLL_MAP)
    ) u_phy (
        .i_controller_clk(i_controller_clk),
        .i_ddr4_clk(i_ddr4_clk),
        .i_ref_clk(i_ref_clk),
        .i_rst_n(internal_rst_n),
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
        .i_dfi_wrdata(dfi_wrdata),
        .i_dfi_wrdata_en(dfi_wrdata_en),
        .i_dfi_wrdata_mask(dfi_wrdata_mask),
        .o_dfi_rddata(dfi_rddata),
        .o_dfi_rddata_valid(dfi_rddata_valid),
        .i_dfi_rddata_en(dfi_rddata_en),
        .i_dfi_init_start(dfi_init_start),
        .o_dfi_init_complete(dfi_init_complete),
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
        .i_tx_diag_req(phy_tx_diag_req),
        .i_tx_diag_dq(phy_tx_diag_dq),
        .i_tx_diag_tap(phy_tx_diag_tap),
        .o_tx_diag_ack(phy_tx_diag_ack),
        .o_tx_diag_error(phy_tx_diag_error),
        .o_tx_diag_current_tap(phy_tx_diag_current_tap),
        .o_tx_diag_previous_tap(phy_tx_diag_previous_tap),
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
        .o_phy_state(phy_train_state),
        .o_phy_idelay_center(phy_idelay_center),
        .o_phy_wl_tap(phy_wl_tap),
        .o_phy_bitslip(phy_bitslip),
        .o_phy_train_fail_gate(phy_train_fail_gate),
        .o_phy_train_fail_eye(phy_train_fail_eye),
        .o_phy_train_fail_wl(phy_train_fail_wl),
        .o_phy_best_width(phy_best_width),
        .o_phy_best_start(phy_best_start),
        .o_phy_wl_dq_tap(phy_wl_dq_tap),
        .o_phy_dqs_initial_tap(phy_dqs_initial_tap),
        .o_phy_rd_lat_extra(phy_rd_lat_extra),
        .o_phy_en_vtc(phy_en_vtc)
    );
    end
    endgenerate

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
        .BIST_DM_TEST(BIST_DM_TEST),
        .BIST_REREAD_DIAG(BIST_REREAD_DIAG),
        .DEBUG_CSR_ENABLE(DEBUG_CSR_ENABLE)
    ) u_prober (
        .i_clk(i_controller_clk),
        .i_rst_n(i_rst_n),
        .i_calib_complete(calib_complete),
        .i_calib_error(calib_error),
        .o_init_done(o_init_done),
        .o_init_failed(o_init_failed),
        .o_bist_busy(prober_bist_busy),
        .o_bist_failed_reset_req(prober_bist_failed_reset_req),
        .o_soft_reset_req(prober_soft_reset_req),
        .o_wb_cyc(prober_wb_cyc),
        .o_wb_stb(prober_wb_stb),
        .o_wb_we(prober_wb_we),
        .o_wb_addr(prober_wb_addr),
        .o_wb_data(prober_wb_data),
        .o_wb_sel(prober_wb_sel),
        .i_wb_stall(bist_wb_stall),
        .i_wb_ack(bist_wb_ack),
        .i_wb_data(ctrl_wb_rdata),
        .i_wb_dbg_cyc(i_wb_dbg_cyc),
        .i_wb_dbg_stb(i_wb_dbg_stb),
        .i_wb_dbg_we(i_wb_dbg_we),
        .i_wb_dbg_addr(i_wb_dbg_addr),
        .i_wb_dbg_data(i_wb_dbg_data),
        .i_wb_dbg_sel(i_wb_dbg_sel),
        .o_wb_dbg_stall(o_wb_dbg_stall),
        .o_wb_dbg_ack(o_wb_dbg_ack),
        .o_wb_dbg_data(o_wb_dbg_data),
        .i_calib_state(ctrl_calib_state),
        .i_stage1_pending(ctrl_stage1_pending),
        .i_stage2_pending(ctrl_stage2_pending),
        .i_stage2_we(ctrl_stage2_we),
        .i_refresh_idle(ctrl_refresh_idle),
        .i_bank_status(ctrl_bank_status),
        .i_phy_state(phy_train_state),
        .i_phy_idelay_center(phy_idelay_center),
        .i_phy_wl_tap(phy_wl_tap),
        .i_phy_bitslip(phy_bitslip),
        .i_phy_train_fail({phy_train_fail_wl, phy_train_fail_eye, phy_train_fail_gate}),
        .i_phy_best_width(phy_best_width),
        .i_phy_best_start(phy_best_start),
        .i_phy_wl_dq_tap(phy_wl_dq_tap),
        .i_phy_dqs_initial_tap(phy_dqs_initial_tap),
        .i_phy_rd_lat_extra(phy_rd_lat_extra),
        .i_phy_en_vtc(phy_en_vtc),
        .i_instruction_address(ctrl_instruction_address),
        .i_pause_counter(ctrl_pause_counter),
        .i_reset_done(ctrl_reset_done),
        .i_pipe_stall(ctrl_pipe_stall),
        .i_calib_retry_count(ctrl_calib_retry_count)
        ,.i_phy_tx_diag_supported(PHY_IMPL)
        ,.o_phy_tx_diag_req(phy_tx_diag_req)
        ,.o_phy_tx_diag_dq(phy_tx_diag_dq)
        ,.o_phy_tx_diag_tap(phy_tx_diag_tap)
        ,.i_phy_tx_diag_ack(phy_tx_diag_ack)
        ,.i_phy_tx_diag_error(phy_tx_diag_error)
        ,.i_phy_tx_diag_current_tap(phy_tx_diag_current_tap)
        ,.i_phy_tx_diag_previous_tap(phy_tx_diag_previous_tap)
    );

endmodule
