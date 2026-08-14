////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_phy_native.v
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  Top-level native-mode DDR4 PHY for Xilinx UltraScale+ FPGAs,
//  supporting up to DDR4-2400. Uses BITSLICE_CONTROL, RXTX_BITSLICE,
//  TX_BITSLICE, and TX_BITSLICE_TRI primitives with an internal PLL
//  (PLLE4_ADV) for high-speed clocking. The DFI 3.1 interface and
//  training FSM are identical to the component-mode PHY.
//
// Architecture overview:
//  The PHY sits between the DFI 3.1 interface and the DDR4 SDRAM pins.
//  One controller clock cycle = 4 DDR4 unit intervals (8:1 DDR SERDES).
//
//  Write path:  DFI wrdata -> TX_BITSLICE (8:1 DDR) -> IOBUF -> pad
//  Read path:   pad -> IOBUF -> RXTX_BITSLICE (1:8 DDR + FIFO) -> bitslip
//               barrel shifter -> DFI rddata
//  Clock path:  TX_BITSLICE (constant 01010101 toggle) -> OBUFDS -> CK/CK#
//  Cmd/Addr:    TX_BITSLICE (SDR 4:1, doubled bits) -> OBUF -> DDR4 CA pins
//
//  Training FSM (runs after reset sequencer completes, driven by MC):
//   1. Gate training:  Sweep the native DQS-gate delay through RIU and
//                      center the widest complete-MPR capture window.
//   2. Eye training:   Sweep RX delay taps across the DQ data eye.
//   3. Write leveling: Sweep TX DQS delay to find 0->1 CK edge.
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

module ddr4_phy_native #(
    // Clock periods in ps
    //   CONTROLLER_CLK_PERIOD = DDR4_CLK_PERIOD * 4 (1/4 rate controller)
    //   DDR4_CLK_PERIOD: 1250=DDR4-1600, 1071=DDR4-1866, 937=DDR4-2133, 833=DDR4-2400
    parameter CONTROLLER_CLK_PERIOD = 3_333,
              DDR4_CLK_PERIOD = 833,
    // DDR4 device data width: 4, 8, or 16
              DEVICE_WIDTH = 8,
    // Number of 8-bit byte lanes
              BYTE_LANES = 2,
    // Derived from DEVICE_WIDTH -- do not override
    parameter BA_BITS = 2,
              BG_BITS = (DEVICE_WIDTH == 16) ? 1 : 2,
              DQ_BITS = 8,
    parameter SERDES_RATIO = 4,
              DFI_DATA_WIDTH = 2 * DQ_BITS * BYTE_LANES,
    // Native BITSLICE is available on both UltraScale and UltraScale+.
    // PHY_PROFILE and the FIFO_PACE_* parameters are retained for source
    // compatibility; physical byte/nibble placement is selected by the board
    // constraints, and reads now pace from all DQ FIFO EMPTY flags.
    parameter SIM_DEVICE = "ULTRASCALE_PLUS",
              PHY_PROFILE = "GENERIC",
              FIFO_PACE_LANE = (BYTE_LANES > 0) ? BYTE_LANES-1 : 0,
              FIFO_PACE_BIT  = DQ_BITS-1,
    // Retained for source compatibility with early native-PHY experiments.
    // The trained DQS gate now establishes the BL8 boundary directly.
              NATIVE_RX_PREAMBLE_BITS = 2
) (
    // Clocks and reset
    input wire                              i_controller_clk,
    /* verilator lint_off UNUSEDSIGNAL */
    input wire                              i_ddr4_clk,
    /* verilator lint_on UNUSEDSIGNAL */
    input wire                              i_ref_clk,
    input wire                              i_rst_n,
    // DFI 3.1 Control (SERDES_RATIO phases, packed flat)
    /* verilator lint_off UNUSEDSIGNAL */
    input wire [SERDES_RATIO*17-1:0]        i_dfi_address,
    /* verilator lint_on UNUSEDSIGNAL */
    input wire [SERDES_RATIO*BA_BITS-1:0]   i_dfi_bank,
    input wire [SERDES_RATIO*BG_BITS-1:0]   i_dfi_bg,
    input wire [SERDES_RATIO-1:0]           i_dfi_cs_n,
    input wire [SERDES_RATIO-1:0]           i_dfi_act_n,
    input wire [SERDES_RATIO-1:0]           i_dfi_ras_n,
    input wire [SERDES_RATIO-1:0]           i_dfi_cas_n,
    input wire [SERDES_RATIO-1:0]           i_dfi_we_n,
    input wire [SERDES_RATIO-1:0]           i_dfi_cke,
    input wire [SERDES_RATIO-1:0]           i_dfi_odt,
    input wire [SERDES_RATIO-1:0]           i_dfi_reset_n,
    // DFI Write Data
    input wire [SERDES_RATIO*DFI_DATA_WIDTH-1:0] i_dfi_wrdata,
    input wire [SERDES_RATIO-1:0]           i_dfi_wrdata_en,
    input wire [SERDES_RATIO*(2*BYTE_LANES)-1:0] i_dfi_wrdata_mask,
    // DFI Read Data
    output reg [SERDES_RATIO*DFI_DATA_WIDTH-1:0] o_dfi_rddata,
    output reg [SERDES_RATIO-1:0]           o_dfi_rddata_valid,
    input wire [SERDES_RATIO-1:0]           i_dfi_rddata_en,
    // DFI Status
    /* verilator lint_off UNUSEDSIGNAL */
    input wire                              i_dfi_init_start,
    /* verilator lint_on UNUSEDSIGNAL */
    output wire                             o_dfi_init_complete,
    // DFI Training (MC -> PHY)
    input wire                              i_dfi_rdlvl_en,
    input wire                              i_dfi_rdlvl_gate_en,
    input wire                              i_dfi_wrlvl_en,
    input wire                              i_dfi_wrlvl_strobe,
    /* verilator lint_off UNUSEDSIGNAL */
    input wire [SERDES_RATIO-1:0]           i_dfi_lvl_pattern,
    input wire                              i_dfi_lvl_periodic,
    /* verilator lint_on UNUSEDSIGNAL */
    // DFI Training (PHY -> MC)
    output reg [BYTE_LANES-1:0]             o_dfi_rdlvl_resp,
    output reg [BYTE_LANES-1:0]             o_dfi_wrlvl_resp,
    output wire                             o_dfi_rdlvl_req,
    output wire                             o_dfi_rdlvl_gate_req,
    output wire                             o_dfi_wrlvl_req,
    // DDR4 SDRAM I/O
    output wire                             o_ddr4_ck_p,
    output wire                             o_ddr4_ck_n,
    output wire                             o_ddr4_reset_n,
    output wire                             o_ddr4_cke,
    output wire                             o_ddr4_cs_n,
    output wire                             o_ddr4_act_n,
    output wire [16:0]                      o_ddr4_addr,
    output wire [BA_BITS-1:0]               o_ddr4_ba,
    output wire [BG_BITS-1:0]               o_ddr4_bg,
    output wire                             o_ddr4_odt,
    output wire [BYTE_LANES-1:0]            o_ddr4_dm_n,
    inout  wire [DQ_BITS*BYTE_LANES-1:0]    io_ddr4_dq,
    inout  wire [BYTE_LANES-1:0]            io_ddr4_dqs_p,
    inout  wire [BYTE_LANES-1:0]            io_ddr4_dqs_n,
    // Debug status (flat packed for synthesis)
    output wire [3:0]                       o_phy_state,
    output wire [9*BYTE_LANES-1:0]          o_phy_idelay_center,
    output wire [9*BYTE_LANES-1:0]          o_phy_wl_tap,
    output wire [4*BYTE_LANES-1:0]          o_phy_bitslip,
    output wire [BYTE_LANES-1:0]            o_phy_train_fail_gate,
    output wire [BYTE_LANES-1:0]            o_phy_train_fail_eye,
    output wire [BYTE_LANES-1:0]            o_phy_train_fail_wl,
    // Extended training debug
    output wire [9*BYTE_LANES-1:0]          o_phy_best_width,
    output wire [9*BYTE_LANES-1:0]          o_phy_best_start,
    output wire [9*BYTE_LANES-1:0]          o_phy_wl_dq_tap,
    output wire [9*BYTE_LANES-1:0]          o_phy_dqs_initial_tap,
    output wire [BYTE_LANES-1:0]            o_phy_rd_lat_extra,
    output wire                             o_phy_en_vtc
);
    // -----------------------------------------------------------------
    // PLL Configuration
    // -----------------------------------------------------------------
    // The native PLL is driven from the 4:1 DFI/controller clock, not the
    // component-PHY delay reference clock.  This preserves an exact integer
    // relationship between DFI words and the bit-slice serializer even when
    // DDR4_CLK_PERIOD is a rounded integer number of picoseconds (e.g. 937ps).
    // For the required 4:1 native DIV4 interface, VCO = 4 * controller_clk
    // and CLKOUTPHY in VCO_2X mode is the DDR transfer rate.
    localparam integer PLL_MULT = SERDES_RATIO;

    // -----------------------------------------------------------------
    // Address/Command pin count and nibble count
    // -----------------------------------------------------------------
    localparam ACMD_PINS = 17 + BA_BITS + BG_BITS + 5 + 1; // addr+ba+bg+ctrl+ck
    localparam ACMD_NIBBLES = (ACMD_PINS + 5) / 6;         // ceil(pins/6)

    // -----------------------------------------------------------------
    // PHY Training FSM state encoding (same as component-mode PHY)
    // -----------------------------------------------------------------
    localparam[3:0] PHY_IDLE          = 4'd0,
                    PHY_GATE_DONE     = 4'd1,
                    PHY_EYE_SWEEP     = 4'd2,
                    PHY_EYE_TRACK     = 4'd3,
                    PHY_EYE_DECIDE    = 4'd4,
                    PHY_EYE_VERIFY    = 4'd5,
                    PHY_EYE_VERIFY_DONE = 4'd6,
                    PHY_EYE_DONE      = 4'd7,
                    PHY_WL_SAMPLE     = 4'd8,
                    PHY_WL_ADJUST     = 4'd9,
                    PHY_WL_CHECK      = 4'd10,
                    PHY_WL_DONE       = 4'd11,
                    PHY_WL_APPLY      = 4'd12,
                    PHY_EYE_OBSERVE   = 4'd13,
                    PHY_EYE_CENTER    = 4'd14;

    // Declared with the state encoding because the native FIFO controller
    // below uses it to retain calibration's continuous-drain behavior.
    reg [3:0] phy_state;

    // MPR page 0 pattern (JESD79-4D Table 56)
    localparam [7:0] MPR_PATTERN = 8'b11110000;

    // Eye training sweep parameters
    localparam [3:0] TAP_SWEEP_STEP = 4'd4;
    localparam [3:0] WL_TAP_STEP = 4'd4;
`ifdef SIM_NATIVE_TX_DEBUG_FAST_EYE
    // Keep the directed primitive debug run short. Production calibration
    // scans the complete RL_DLY transfer function at single-tap resolution.
    localparam [4:0] GATE_TAP_STEP = 5'd16;
    localparam [8:0] GATE_SWEEP_LAST = 9'd240;
`else
    localparam [4:0] GATE_TAP_STEP = 5'd1;
    localparam [8:0] GATE_SWEEP_LAST = 9'd511;
`endif
`ifdef SIM_NATIVE_TX_DEBUG_FAST_EYE
    localparam [8:0] EYE_SWEEP_LAST = 9'd64;
`else
    localparam [8:0] EYE_SWEEP_LAST = 9'd508;
`endif
    localparam [7:0] VTC_SETTLE_CYCLES = 8'd200;

    // The native RXTX BITSLICE uses an asynchronous eight-entry receive FIFO.
    // Its output is intentionally not phase-locked to DFI rddata_en, unlike
    // the component-mode ISERDESE3 path.  Sample every possible FIFO-return
    // cycle following each MPR read; this covers the documented FIFO depth
    // without making a board- or speed-specific latency assumption.
    localparam [3:0] NATIVE_RX_OBSERVE_CYCLES = 4'd8;

    // DFI Data Layout
    localparam TOTAL_DQ      = DQ_BITS * BYTE_LANES;
    localparam DM_PER_PHASE  = 2 * BYTE_LANES;
    localparam DM_ENABLED    = (DEVICE_WIDTH != 4);

`ifndef SYNTHESIS
    // Fail early on configurations that cannot match the fixed 1:4 native
    // BITSLICE datapath. These checks do not add hardware.
    initial begin
        if (SERDES_RATIO != 4)
            $error("ddr4_phy_native requires SERDES_RATIO=4");
        if (DQ_BITS != 8)
            $error("ddr4_phy_native requires eight DQ bits per byte lane");
        if (BYTE_LANES < 1)
            $error("ddr4_phy_native requires at least one byte lane");
        if ((DEVICE_WIDTH != 4) && (DEVICE_WIDTH != 8) &&
            (DEVICE_WIDTH != 16))
            $error("ddr4_phy_native DEVICE_WIDTH must be x4, x8, or x16");
        if ((SIM_DEVICE != "ULTRASCALE") &&
            (SIM_DEVICE != "ULTRASCALE_PLUS"))
            $error("ddr4_phy_native SIM_DEVICE must select UltraScale or UltraScale+");
    end
`endif

    // -----------------------------------------------------------------
    // PLL Instance (PLLE4_ADV)
    // Generates CLKOUTPHY for bitslice serialization.
    // CLKIN = i_controller_clk.  For DDR4-1600 this is 200 MHz, yielding
    // VCO=800 MHz and CLKOUTPHY=1600 MHz.  CLKOUTPHY stays on the dedicated
    // XPHY route; it is never promoted to a fabric/global 800 MHz clock.
    // i_ref_clk remains in the public port list for DFI/top compatibility,
    // but native BITSLICE_CONTROL uses REFCLK_SRC=PLLCLK.
    // -----------------------------------------------------------------
    wire pll_clkoutphy;
    wire pll_clkfbout;
    wire pll_clkfbin;
    wire pll_locked;
    wire pll_rst;
    wire clkoutphy_en;

    /* verilator lint_off PINCONNECTEMPTY */
    // PLLE3_ADV is required by UltraScale; PLLE4_ADV is required by
    // UltraScale+.  Both expose the dedicated CLKOUTPHY path used below.
    generate
        if (SIM_DEVICE == "ULTRASCALE") begin : gen_plle3
            PLLE3_ADV #(
                .CLKFBOUT_MULT   (PLL_MULT),
                // Match the UltraScale DDR4 MIG native-XPHY PLL.  DIV4
                // BITSLICE_CONTROL requires the dedicated PLL clock in
                // quadrature with the controller word clock so PHY_RDEN and
                // serialized data phase boundaries land on complete UI.
                .CLKFBOUT_PHASE  (90.000),
                .CLKIN_PERIOD    (CONTROLLER_CLK_PERIOD / 1000.0),
                .CLKOUT0_DIVIDE  (1),
                .CLKOUT0_DUTY_CYCLE (0.500),
                .CLKOUT0_PHASE   (0.000),
                .CLKOUTPHY_MODE  ("VCO_2X"),
                .COMPENSATION    ("INTERNAL"),
                .DIVCLK_DIVIDE   (1),
                .REF_JITTER      (0.010),
                .STARTUP_WAIT    ("FALSE")
            ) u_pll (
                .CLKIN        (i_controller_clk), .CLKFBIN(pll_clkfbin),
                .CLKFBOUT     (pll_clkfbout), .CLKOUT0(), .CLKOUT0B(),
                .CLKOUT1      (), .CLKOUT1B(), .CLKOUTPHY(pll_clkoutphy),
                .LOCKED       (pll_locked), .CLKOUTPHYEN(clkoutphy_en),
                .PWRDWN       (1'b0), .RST(pll_rst), .DADDR(7'd0),
                .DCLK         (1'b0), .DEN(1'b0), .DI(16'd0), .DO(),
                .DRDY         (), .DWE(1'b0)
            );
        end else begin : gen_plle4
            PLLE4_ADV #(
                .CLKFBOUT_MULT   (PLL_MULT),
                .CLKFBOUT_PHASE  (90.000),
                .CLKIN_PERIOD    (CONTROLLER_CLK_PERIOD / 1000.0),
                .CLKOUT0_DIVIDE  (1),
                .CLKOUT0_DUTY_CYCLE (0.500),
                .CLKOUT0_PHASE   (0.000),
                .CLKOUTPHY_MODE  ("VCO_2X"),
                .COMPENSATION    ("INTERNAL"),
                .DIVCLK_DIVIDE   (1),
                .REF_JITTER      (0.010),
                .STARTUP_WAIT    ("FALSE")
            ) u_pll (
                .CLKIN        (i_controller_clk), .CLKFBIN(pll_clkfbin),
                .CLKFBOUT     (pll_clkfbout), .CLKOUT0(), .CLKOUT1(),
                .CLKOUTPHY    (pll_clkoutphy), .LOCKED(pll_locked),
                .CLKOUTPHYEN  (clkoutphy_en), .PWRDWN(1'b0), .RST(pll_rst),
                .DADDR        (7'd0), .DCLK(1'b0), .DEN(1'b0), .DI(16'd0),
                .DO           (), .DRDY(), .DWE(1'b0)
            );
        end
    endgenerate
    /* verilator lint_on PINCONNECTEMPTY */

    // Internal feedback
    assign pll_clkfbin = pll_clkfbout;

    // -----------------------------------------------------------------
    // Reset Sequencer
    // -----------------------------------------------------------------
    wire bsc_rst;
    wire bitslice_rst;
    wire rst_en_vtc;
    wire rst_phy_rden;
    wire rst_init_complete;

    // Consolidated native-calibration status.  UG571 requires every
    // BITSLICE_CONTROL in a native interface--including command/address
    // controls--to be ready before traffic or training can begin.
    wire [BYTE_LANES-1:0] byte_dly_rdy;
    wire [BYTE_LANES-1:0] byte_vtc_rdy;
    wire [ACMD_NIBBLES-1:0] acmd_dly_rdy;
    wire [ACMD_NIBBLES-1:0] acmd_vtc_rdy;
    wire all_dly_rdy = (&byte_dly_rdy) & (&acmd_dly_rdy);
    wire all_vtc_rdy = (&byte_vtc_rdy) & (&acmd_vtc_rdy);

    /* verilator lint_off PINCONNECTEMPTY */
    ddr4_phy_native_reset u_reset_seq (
        .i_clk            (i_controller_clk),
        .i_rst_n          (i_rst_n),
        .i_pll_locked     (pll_locked),
        .i_dly_rdy        (all_dly_rdy),
        .i_vtc_rdy        (all_vtc_rdy),
        .o_pll_rst        (pll_rst),
        .o_bsc_rst        (bsc_rst),
        .o_bitslice_rst   (bitslice_rst),
        .o_clkoutphy_en   (clkoutphy_en),
        .o_en_vtc         (rst_en_vtc),
        .o_tbyte_en       (),
        .o_phy_rden       (rst_phy_rden),
        .o_init_complete  (rst_init_complete),
        .o_phy_state      ()
    );
    /* verilator lint_on PINCONNECTEMPTY */

    // Fabric reset: held HIGH until reset sequencer completes
    reg sync_rst;
    always @(posedge i_controller_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            sync_rst <= 1'b1;
        else if (rst_init_complete)
            sync_rst <= 1'b0;
    end

    // -----------------------------------------------------------------
    // DFI training request outputs (PHY-initiated, not used)
    // -----------------------------------------------------------------
    assign o_dfi_rdlvl_req      = 1'b0;
    assign o_dfi_rdlvl_gate_req = 1'b0;
    assign o_dfi_wrlvl_req      = 1'b0;

    // EN_VTC split per UG571 native-mode bring-up and COUNT-mode rules:
    //  - bsc_en_vtc: BITSLICE_CONTROL.EN_VTC (LOW during reset, HIGH after DLY_RDY)
    //  - bitslice_en_vtc: HIGH during reset/BISC, then LOW for COUNT-mode
    //    RX/TX delays.  COUNT values are deliberately not VT compensated.
    reg en_vtc_q;
    wire bsc_en_vtc = rst_init_complete ? en_vtc_q : rst_en_vtc;
    reg bitslice_en_vtc_q;
    wire bitslice_en_vtc = rst_init_complete ? bitslice_en_vtc_q : 1'b1;

    // UG571 limits one RXTX_BITSLICE CNTVALUEIN update to eight taps.  Eye
    // centering and write-level fallback can move much farther than that, so
    // every physical load walks toward its target in legal increments.
    function automatic [8:0] delay_step_toward;
        input [8:0] current_tap;
        input [8:0] target_tap;
        begin
            if ({1'b0, current_tap} + 10'd8 < {1'b0, target_tap})
                delay_step_toward = current_tap + 9'd8;
            else if ({1'b0, target_tap} + 10'd8 < {1'b0, current_tap})
                delay_step_toward = current_tap - 9'd8;
            else
                delay_step_toward = target_tap;
        end
    endfunction

    // DFI init complete: asserted when reset sequencer finishes
    assign o_dfi_init_complete = rst_init_complete;

    // -----------------------------------------------------------------
    // Address/Command Output Path (TX_BITSLICE based)
    //
    // Each DDR4 CA pin gets one TX_BITSLICE (8:1 SDR, doubled for DDR clock).
    // DFI provides 4 phases per controller clock. Each phase value is
    // repeated on rise+fall edges (SDR command bus), giving 8 bits:
    //   D[7:0] = {phase3, phase3, phase2, phase2, phase1, phase1, phase0, phase0}
    //
    // Pin mapping for the packed vector acmd_data[ACMD_PINS-1:0]:
    //   [16:0]                     = Address A[16:0]
    //   [16+BA_BITS:17]            = Bank address BA
    //   [16+BA_BITS+BG_BITS:17+BA_BITS] = Bank group BG
    //   [+1] CS_n, [+1] ACT_n, [+1] CKE, [+1] ODT, [+1] RESET_n
    //   [ACMD_PINS-1]              = CK (differential clock)
    // -----------------------------------------------------------------
    wire [8*ACMD_PINS-1:0] acmd_data;   // 8-bit D per pin
    wire [ACMD_PINS-1:0]   acmd_out;    // serialized output per pin

    // Address pins A[16:0]: pack DFI phases into 8-bit OSERDES data
    generate
        genvar abit;
        for (abit = 0; abit < 17; abit = abit + 1) begin : gen_acmd_addr
            wire [3:0] addr_phases;
            if (abit < 14) begin : lo_addr
                assign addr_phases = {i_dfi_address[17*3 + abit],
                                      i_dfi_address[17*2 + abit],
                                      i_dfi_address[17*1 + abit],
                                      i_dfi_address[17*0 + abit]};
            end else if (abit == 14) begin : a14_we
                assign addr_phases = i_dfi_we_n;
            end else if (abit == 15) begin : a15_cas
                assign addr_phases = i_dfi_cas_n;
            end else begin : a16_ras
                assign addr_phases = i_dfi_ras_n;
            end
            assign acmd_data[abit*8 +: 8] = {addr_phases[3], addr_phases[3],
                                              addr_phases[2], addr_phases[2],
                                              addr_phases[1], addr_phases[1],
                                              addr_phases[0], addr_phases[0]};
        end
    endgenerate

    // Bank address BA[BA_BITS-1:0]
    generate
        genvar babit;
        for (babit = 0; babit < BA_BITS; babit = babit + 1) begin : gen_acmd_ba
            localparam integer PIN_IDX = 17 + babit;
            assign acmd_data[PIN_IDX*8 +: 8] = {
                i_dfi_bank[BA_BITS*3 + babit], i_dfi_bank[BA_BITS*3 + babit],
                i_dfi_bank[BA_BITS*2 + babit], i_dfi_bank[BA_BITS*2 + babit],
                i_dfi_bank[BA_BITS*1 + babit], i_dfi_bank[BA_BITS*1 + babit],
                i_dfi_bank[BA_BITS*0 + babit], i_dfi_bank[BA_BITS*0 + babit]};
        end
    endgenerate

    // Bank group BG[BG_BITS-1:0]
    generate
        genvar bgbit;
        for (bgbit = 0; bgbit < BG_BITS; bgbit = bgbit + 1) begin : gen_acmd_bg
            localparam integer PIN_IDX = 17 + BA_BITS + bgbit;
            assign acmd_data[PIN_IDX*8 +: 8] = {
                i_dfi_bg[BG_BITS*3 + bgbit], i_dfi_bg[BG_BITS*3 + bgbit],
                i_dfi_bg[BG_BITS*2 + bgbit], i_dfi_bg[BG_BITS*2 + bgbit],
                i_dfi_bg[BG_BITS*1 + bgbit], i_dfi_bg[BG_BITS*1 + bgbit],
                i_dfi_bg[BG_BITS*0 + bgbit], i_dfi_bg[BG_BITS*0 + bgbit]};
        end
    endgenerate

    // Control pins: CS_n, ACT_n, CKE, ODT, RESET_n
    generate
        genvar cpin;
        for (cpin = 0; cpin < 5; cpin = cpin + 1) begin : gen_acmd_ctrl
            localparam integer PIN_IDX = 17 + BA_BITS + BG_BITS + cpin;
            wire [3:0] ctrl_phases = (cpin == 0) ? i_dfi_cs_n :
                                     (cpin == 1) ? i_dfi_act_n :
                                     (cpin == 2) ? i_dfi_cke :
                                     (cpin == 3) ? i_dfi_odt :
                                                   i_dfi_reset_n;
            assign acmd_data[PIN_IDX*8 +: 8] = {ctrl_phases[3], ctrl_phases[3],
                                                 ctrl_phases[2], ctrl_phases[2],
                                                 ctrl_phases[1], ctrl_phases[1],
                                                 ctrl_phases[0], ctrl_phases[0]};
        end
    endgenerate

    // CK clock: constant 01010101 toggle
    localparam integer CK_PIN_IDX = ACMD_PINS - 1;
    assign acmd_data[CK_PIN_IDX*8 +: 8] = 8'b01_01_01_01;

    // -----------------------------------------------------------------
    // Address/Command TX_BITSLICE + BITSLICE_CONTROL (per nibble)
    // Each nibble has up to 6 TX_BITSLICEs (positions 0-5, position 6 unused).
    // No RX path needed for address/command.
    // -----------------------------------------------------------------
    /* verilator lint_off PINCONNECTEMPTY */
    /* verilator lint_off PINMISSING */
    generate
        genvar nib, pos;
        for (nib = 0; nib < ACMD_NIBBLES; nib = nib + 1) begin : gen_acmd_nibble
            // Number of pins in this nibble (last nibble may be partial)
            localparam integer PINS_THIS_NIB = (nib < ACMD_NIBBLES - 1) ? 6 :
                                              (ACMD_PINS - nib * 6);

            // BIT_CTRL buses for this nibble
            wire [39:0] rx_bctrl_out [0:6];
            wire [39:0] tx_bctrl_out [0:6];
            wire [39:0] rx_bctrl_in  [0:6];
            wire [39:0] tx_bctrl_in  [0:6];

            // Tie off all inputs by default
            genvar tiepos;
            for (tiepos = 0; tiepos < 7; tiepos = tiepos + 1) begin : gen_tie_default
                if (tiepos >= PINS_THIS_NIB || tiepos == 6) begin : tie_unused
                    assign rx_bctrl_in[tiepos] = 40'd0;
                    assign tx_bctrl_in[tiepos] = 40'd0;
                end
            end

            // BITSLICE_CONTROL for this addr/cmd nibble
            BITSLICE_CONTROL #(
                .DIV_MODE           ("DIV4"),
                .SERIAL_MODE        ("FALSE"),
                .RX_CLK_PHASE_P     ("SHIFT_0"),
                .RX_CLK_PHASE_N     ("SHIFT_0"),
                .EN_OTHER_PCLK      ("FALSE"),
                .EN_OTHER_NCLK      ("FALSE"),
                .SELF_CALIBRATE     ("ENABLE"),
                .RX_GATING          ("DISABLE"),
                .TX_GATING          ("DISABLE"),
                .SIM_DEVICE         (SIM_DEVICE)
            ) u_bsc_acmd (
                .PLL_CLK            (pll_clkoutphy),
                .REFCLK             (1'b0),
                .RIU_CLK            (i_controller_clk),
                .RST                (bsc_rst),
                .EN_VTC             (bsc_en_vtc),
                .DLY_RDY            (acmd_dly_rdy[nib]),
                .VTC_RDY            (acmd_vtc_rdy[nib]),
                // UG571: an unused inter-byte clock input is pulled High.
                // Low is reserved for an actively-driven external byte clock.
                .CLK_FROM_EXT       (1'b1),
                .PCLK_NIBBLE_IN     (1'b0),
                .NCLK_NIBBLE_IN     (1'b0),
                .PCLK_NIBBLE_OUT    (),
                .NCLK_NIBBLE_OUT    (),
                .TBYTE_IN           (4'b0000),
                .PHY_RDEN           (4'b0000),
                .PHY_RDCS0          (4'b0000),
                .PHY_RDCS1          (4'b0000),
                .PHY_WRCS0          (4'b0000),
                .PHY_WRCS1          (4'b0000),
                .RX_BIT_CTRL_OUT0   (rx_bctrl_out[0]),
                .TX_BIT_CTRL_OUT0   (tx_bctrl_out[0]),
                .RX_BIT_CTRL_IN0    (rx_bctrl_in[0]),
                .TX_BIT_CTRL_IN0    (tx_bctrl_in[0]),
                .RX_BIT_CTRL_OUT1   (rx_bctrl_out[1]),
                .TX_BIT_CTRL_OUT1   (tx_bctrl_out[1]),
                .RX_BIT_CTRL_IN1    (rx_bctrl_in[1]),
                .TX_BIT_CTRL_IN1    (tx_bctrl_in[1]),
                .RX_BIT_CTRL_OUT2   (rx_bctrl_out[2]),
                .TX_BIT_CTRL_OUT2   (tx_bctrl_out[2]),
                .RX_BIT_CTRL_IN2    (rx_bctrl_in[2]),
                .TX_BIT_CTRL_IN2    (tx_bctrl_in[2]),
                .RX_BIT_CTRL_OUT3   (rx_bctrl_out[3]),
                .TX_BIT_CTRL_OUT3   (tx_bctrl_out[3]),
                .RX_BIT_CTRL_IN3    (rx_bctrl_in[3]),
                .TX_BIT_CTRL_IN3    (tx_bctrl_in[3]),
                .RX_BIT_CTRL_OUT4   (rx_bctrl_out[4]),
                .TX_BIT_CTRL_OUT4   (tx_bctrl_out[4]),
                .RX_BIT_CTRL_IN4    (rx_bctrl_in[4]),
                .TX_BIT_CTRL_IN4    (tx_bctrl_in[4]),
                .RX_BIT_CTRL_OUT5   (rx_bctrl_out[5]),
                .TX_BIT_CTRL_OUT5   (tx_bctrl_out[5]),
                .RX_BIT_CTRL_IN5    (rx_bctrl_in[5]),
                .TX_BIT_CTRL_IN5    (tx_bctrl_in[5]),
                .RX_BIT_CTRL_OUT6   (rx_bctrl_out[6]),
                .TX_BIT_CTRL_OUT6   (tx_bctrl_out[6]),
                .RX_BIT_CTRL_IN6    (rx_bctrl_in[6]),
                .TX_BIT_CTRL_IN6    (tx_bctrl_in[6]),
                .TX_BIT_CTRL_OUT_TRI(),
                .TX_BIT_CTRL_IN_TRI (40'd0),
                .RIU_ADDR           (6'd0),
                .RIU_WR_DATA        (16'd0),
                .RIU_WR_EN          (1'b0),
                .RIU_NIBBLE_SEL     (1'b0),
                .RIU_RD_DATA        (),
                .RIU_VALID          ()
            );

            // TX_BITSLICE instances for each pin in this nibble
            for (pos = 0; pos < 6; pos = pos + 1) begin : gen_acmd_txbs
                localparam integer GLOBAL_PIN = nib * 6 + pos;
                if (GLOBAL_PIN < ACMD_PINS) begin : active_pin
                    TX_BITSLICE #(
                        .DATA_WIDTH     (8),
                        .DELAY_FORMAT   ("COUNT"),
                        .DELAY_TYPE     ("FIXED"),
                        .DELAY_VALUE    (0),
                        .INIT           (1'b0),
                        .SIM_DEVICE     (SIM_DEVICE)
                    ) u_tx_acmd (
                        .CLK            (i_controller_clk),
                        .RST            (bitslice_rst),
                        .RST_DLY        (bitslice_rst),
                        .CE             (1'b0),
                        .INC            (1'b0),
                        .EN_VTC         (bitslice_en_vtc),
                        .CNTVALUEIN     (9'd0),
                        .LOAD           (1'b0),
                        .CNTVALUEOUT    (),
                        .D              (acmd_data[GLOBAL_PIN*8 +: 8]),
                        .O              (acmd_out[GLOBAL_PIN]),
                        .T              (1'b0),
                        .TBYTE_IN       (1'b0),
                        .TX_BIT_CTRL_IN (tx_bctrl_out[pos]),
                        .TX_BIT_CTRL_OUT(tx_bctrl_in[pos]),
                        .RX_BIT_CTRL_IN (rx_bctrl_out[pos]),
                        .RX_BIT_CTRL_OUT(rx_bctrl_in[pos])
                    );
                end
            end
        end
    endgenerate
    /* verilator lint_on PINMISSING */
    /* verilator lint_on PINCONNECTEMPTY */

    // -----------------------------------------------------------------
    // Address/Command Output Buffers
    // -----------------------------------------------------------------
    // Address A[16:0]
    generate
        genvar addr_obuf;
        for (addr_obuf = 0; addr_obuf < 17; addr_obuf = addr_obuf + 1) begin : gen_obuf_addr
            OBUF u_obuf_addr (.I(acmd_out[addr_obuf]), .O(o_ddr4_addr[addr_obuf]));
        end
    endgenerate

    // Bank address BA
    generate
        genvar ba_obuf;
        for (ba_obuf = 0; ba_obuf < BA_BITS; ba_obuf = ba_obuf + 1) begin : gen_obuf_ba
            OBUF u_obuf_ba (.I(acmd_out[17 + ba_obuf]), .O(o_ddr4_ba[ba_obuf]));
        end
    endgenerate

    // Bank group BG
    generate
        genvar bg_obuf;
        for (bg_obuf = 0; bg_obuf < BG_BITS; bg_obuf = bg_obuf + 1) begin : gen_obuf_bg
            OBUF u_obuf_bg (.I(acmd_out[17 + BA_BITS + bg_obuf]), .O(o_ddr4_bg[bg_obuf]));
        end
    endgenerate

    // Control pins
    OBUF u_obuf_cs  (.I(acmd_out[17 + BA_BITS + BG_BITS + 0]), .O(o_ddr4_cs_n));
    OBUF u_obuf_act (.I(acmd_out[17 + BA_BITS + BG_BITS + 1]), .O(o_ddr4_act_n));
    OBUF u_obuf_cke (.I(acmd_out[17 + BA_BITS + BG_BITS + 2]), .O(o_ddr4_cke));
    OBUF u_obuf_odt (.I(acmd_out[17 + BA_BITS + BG_BITS + 3]), .O(o_ddr4_odt));
    OBUF u_obuf_rst (.I(acmd_out[17 + BA_BITS + BG_BITS + 4]), .O(o_ddr4_reset_n));

    // CK differential output buffer
    OBUFDS u_obufds_ck (.I(acmd_out[CK_PIN_IDX]), .O(o_ddr4_ck_p), .OB(o_ddr4_ck_n));

    // -----------------------------------------------------------------
    // Data Byte Lanes
    // -----------------------------------------------------------------
    // Training FSM signals (same names as component-mode for direct reuse)
    reg [8:0] idelay_cntvalue;
    reg [BYTE_LANES-1:0] idelay_load_lane;
    reg [8:0] odelay_dqs_cntvalue;
    reg [BYTE_LANES-1:0] odelay_dqs_load;
    reg [8:0] odelay_dq_cntvalue;
    reg [BYTE_LANES-1:0] odelay_dq_load;
    wire [8:0] odelay_dqs_cntvalueout [0:BYTE_LANES-1];

    // TX data signals
    wire [DQ_BITS*8-1:0] tx_dq_data [0:BYTE_LANES-1];
    wire [7:0] tx_dm_data [0:BYTE_LANES-1];
    reg  [7:0] dqs_pattern;

    // The native TX clock network launches a serial word after it is
    // presented to RXTX_BITSLICE. Preserve the DFI bundle at that boundary
    // while the exact native serializer phase is selected below.
    wire wrdata_en_any = |i_dfi_wrdata_en;
    reg [DFI_DATA_WIDTH*4-1:0] tx_wrdata_pipe0;
    reg [DM_PER_PHASE*4-1:0]   tx_wrmask_pipe0;
    reg [2:0]                  wrdata_en_shift;
    wire                       output_enable;

    // RX data from FIFO
    wire [DQ_BITS*8-1:0] rx_dq_data [0:BYTE_LANES-1];
    wire [DQ_BITS-1:0] fifo_empty [0:BYTE_LANES-1];
    wire [BYTE_LANES-1:0] wl_feedback_raw;
    reg calibration_fifo_pop_q;
    wire [DQ_BITS-1:0] fifo_rd_en_drive [0:BYTE_LANES-1];

    // RIU is used only by native DQS-gate training. A write is broadcast
    // while sweeping, then the selected center is written per byte lane.
    reg [5:0] native_riu_addr;
    reg [15:0] native_riu_wr_data;
    reg native_riu_wr_en;
    reg [BYTE_LANES-1:0] native_riu_sel;
    wire [15:0] native_riu_rd_data [0:BYTE_LANES-1];
    wire [BYTE_LANES-1:0] native_riu_valid;

    // BITSLICE_CONTROL RIU registers used by read-gate calibration.
    localparam [5:0] RIU_ADDR_NIBBLE_CTRL0 = 6'h00;
    localparam [5:0] RIU_ADDR_RL_DLY_RNK0  = 6'h30;
    localparam [15:0] RIU_GATE_CLEAR       = 16'h0130;
    localparam [15:0] RIU_GATE_RUN         = 16'h0030;

    // Tristate
    wire [3:0] tbyte_dq;
    wire [3:0] tbyte_dqs;

    // -----------------------------------------------------------------
    // Native receive gating enable
    //
    // BITSLICE_CONTROL must only see PHY_RDEN around an actual read DQS
    // burst.  Leaving it High also captures the controller's own writes and
    // fills the native RX FIFOs with stale words before read data arrives.
    //
    // MIG generates a four-tCK gate mask from each phase-specific READ
    // command. With additive latency disabled, its coarse placement can span
    // RL-3 through RL+4. This implementation keeps the coarse point fixed and
    // trains the lane-specific fine RL_DLY value. Until training completes,
    // start at the same nominal coarse point. This
    // keeps the primitive's internally delayed gate open through the BL8
    // postamble instead of truncating the final receive word. Adjacent reads merge into
    // a continuous gate.  A 1:4 shift register retains exact phase placement
    // across controller-clock boundaries.
    wire [SERDES_RATIO-1:0] dfi_read_command =
        (~i_dfi_cs_n) & i_dfi_act_n & i_dfi_ras_n &
        (~i_dfi_cas_n) & i_dfi_we_n;
    wire dfi_read_expected = |i_dfi_rddata_en;

    function [5:0] native_auto_cl;
        input integer clock_period_ps;
        begin
            native_auto_cl = (clock_period_ps >= 1250) ? 6'd12 :
                             (clock_period_ps >= 1071) ? 6'd14 :
                             (clock_period_ps >=  937) ? 6'd16 : 6'd18;
        end
    endfunction

    localparam integer RD_GATE_PIPE_BITS = 64;
    localparam [5:0] NATIVE_CL_NCK = native_auto_cl(DDR4_CLK_PERIOD);
    // The DFI command vector is registered at the controller/PHY boundary
    // two memory clocks later than MIG's internal rdCAS event.  Compensate
    // that fixed pipeline once here and retain the same coarse point through
    // gate training, eye training, and normal reads.
    localparam [5:0] NATIVE_GATE_MCL = NATIVE_CL_NCK - 6'd2;
    // Directed XSim tests may override the post-training coarse gate point.
    // Production builds always select NATIVE_GATE_MCL.
`ifdef SIM_NATIVE_GATE_MCL_15
    localparam [5:0] NATIVE_GATE_MCL_INITIAL = 6'd15;
`elsif SIM_NATIVE_GATE_MCL_16
    localparam [5:0] NATIVE_GATE_MCL_INITIAL = 6'd16;
`elsif SIM_NATIVE_GATE_MCL_17
    localparam [5:0] NATIVE_GATE_MCL_INITIAL = 6'd17;
`elsif SIM_NATIVE_GATE_MCL_18
    localparam [5:0] NATIVE_GATE_MCL_INITIAL = 6'd18;
`elsif SIM_NATIVE_GATE_MCL_19
    localparam [5:0] NATIVE_GATE_MCL_INITIAL = 6'd19;
`elsif SIM_NATIVE_GATE_MCL_20
    localparam [5:0] NATIVE_GATE_MCL_INITIAL = 6'd20;
`elsif SIM_NATIVE_GATE_MCL_21
    localparam [5:0] NATIVE_GATE_MCL_INITIAL = 6'd21;
`elsif SIM_NATIVE_GATE_MCL_22
    localparam [5:0] NATIVE_GATE_MCL_INITIAL = 6'd22;
`else
    localparam [5:0] NATIVE_GATE_MCL_INITIAL = NATIVE_GATE_MCL;
`endif
    reg [8:0] gate_sweep_tap;
    reg [5:0] gate_trained_mcl [0:BYTE_LANES-1];
    // MIG permits mCL to train around the nominal read latency.  This PHY
    // keeps that coarse value fixed and sweeps the per-byte RL_DLY control;
    // normal traffic then uses the same coarse value plus each selected fine
    // center.  Hold the command-side mask at the established coarse point
    // while that delay is swept; moving both controls together makes
    // GT_STATUS ambiguous.
    wire [5:0] active_read_mcl = i_dfi_rdlvl_gate_en ?
        NATIVE_GATE_MCL : gate_trained_mcl[0];
    reg [RD_GATE_PIPE_BITS-1:0] read_gate_pipe;
    reg [RD_GATE_PIPE_BITS-1:0] read_gate_pipe_next;
    integer read_gate_phase, read_gate_ui;
    integer read_gate_start;

    always @* begin
        read_gate_pipe_next = read_gate_pipe >> SERDES_RATIO;
        for (read_gate_phase = 0; read_gate_phase < SERDES_RATIO;
            read_gate_phase = read_gate_phase + 1) begin
            if (dfi_read_command[read_gate_phase]) begin
                // Match MIG rs2mask placement: a phase/slot-2 CAS shifts the
                // four-tCK gate mask two tCK later than a slot-0 CAS.
                // dfi_read_command is observed at the registered DFI/PHY
                // boundary, one controller word after the controller's
                // internal rdCAS event used by MIG's cal_rd_en block.  Remove
                // that four-tCK interface latency before applying the same
                // RL-3..RL+4 mCL placement.
                read_gate_start = active_read_mcl - 7 +
                                  ((read_gate_phase >= 2) ? 2 : 0);
                // PHY_RDEN is the four-tCK BL8 mask consumed by the native
                // gate state machine. Extending it changes the gate restart
                // cadence and merges two BL8 words into one FIFO frame.
                for (read_gate_ui = 0; read_gate_ui < SERDES_RATIO;
                     read_gate_ui = read_gate_ui + 1)
                    read_gate_pipe_next[read_gate_start + read_gate_ui] = 1'b1;
            end
        end
    end

    always @(posedge i_controller_clk) begin
        if (sync_rst)
            read_gate_pipe <= {RD_GATE_PIPE_BITS{1'b0}};
        else
            read_gate_pipe <= read_gate_pipe_next;
    end

    // PHY_RDEN is a timing mask, not a permanent receiver-enable.  MIG drives
    // one four-tCK mask for each BL8 READ so BITSLICE_CONTROL rejects the DQS
    // preamble and the FPGA's own write strobes.  The trained mCL controls the
    // coarse placement and the RIU gate delay supplies the per-byte fine phase.
`ifdef SIM_NATIVE_GATE_BYPASS
    // The acceleration bypass skips the RIU DQS-gate sweep, so keep the MPR
    // receiver open during the following eye scan. Application traffic still
    // exercises the command-timed mask exactly as hardware does.
    wire [SERDES_RATIO-1:0] phy_rden_mask = i_dfi_rdlvl_en ?
        {SERDES_RATIO{1'b1}} : read_gate_pipe[SERDES_RATIO-1:0];
`else
    wire [SERDES_RATIO-1:0] phy_rden_mask =
        read_gate_pipe[SERDES_RATIO-1:0];
`endif
    wire [SERDES_RATIO-1:0] phy_rden_ready = phy_rden_mask &
        {SERDES_RATIO{rst_phy_rden & ~output_enable}};
    // -----------------------------------------------------------------
    generate
        genvar lane;
        for (lane = 0; lane < BYTE_LANES; lane = lane + 1) begin : gen_byte_lane
            ddr4_phy_native_byte #(
                .DQ_BITS(DQ_BITS),
                .REFCLK_FREQ(1000000.0 / CONTROLLER_CLK_PERIOD),
                .SIM_DEVICE(SIM_DEVICE)
            ) u_byte (
                .i_pll_clkoutphy   (pll_clkoutphy),
                .i_div_clk         (i_controller_clk),
                .i_bsc_rst         (bsc_rst),
                .i_bitslice_rst    (bitslice_rst),
                // RXTX_BITSLICE can observe its own transmitted DQS through
                // the IOB even while the read gate is closed.  Hold only the
                // receive FIFO in reset for the complete write ownership
                // interval so echoed write words cannot precede the next
                // DRAM return.  TX serialization and trained delays are not
                // reset by this path.
                .i_rx_fifo_rst     (bitslice_rst | rx_fifo_flush |
                                    output_enable),
                .o_dly_rdy         (byte_dly_rdy[lane]),
                .o_vtc_rdy         (byte_vtc_rdy[lane]),
                .i_bsc_en_vtc      (bsc_en_vtc),
                .i_bitslice_en_vtc (bitslice_en_vtc),
                .i_tx_dq_data      (tx_dq_data[lane]),
                .i_tx_dqs_data     (dqs_pattern),
                .i_tx_dm_data      (tx_dm_data[lane]),
                .i_tbyte_dq        (tbyte_dq),
                .i_tbyte_dqs       (tbyte_dqs),
                .i_phy_rden        (phy_rden_ready),
                .o_rx_dq_data      (rx_dq_data[lane]),
                .o_fifo_empty      (fifo_empty[lane]),
                .o_wl_feedback     (wl_feedback_raw[lane]),
                .i_fifo_rd_en      (fifo_rd_en_drive[lane]),
                .i_riu_addr        (native_riu_addr),
                .i_riu_wr_data     (native_riu_wr_data),
                .i_riu_wr_en       (native_riu_wr_en),
                .i_riu_nibble_sel  (native_riu_sel[lane]),
                .o_riu_rd_data     (native_riu_rd_data[lane]),
                .o_riu_valid       (native_riu_valid[lane]),
                .i_rx_cntvaluein   (idelay_cntvalue),
                .i_rx_load         (idelay_load_lane[lane]),
                /* verilator lint_off PINCONNECTEMPTY */
                .o_rx_cntvalueout_dq0(),
                /* verilator lint_on PINCONNECTEMPTY */
                .i_tx_dq_cntvaluein(odelay_dq_cntvalue),
                .i_tx_dq_load      (odelay_dq_load[lane]),
                .i_tx_dqs_cntvaluein(odelay_dqs_cntvalue),
                .i_tx_dqs_load     (odelay_dqs_load[lane]),
                .o_tx_dqs_cntvalueout(odelay_dqs_cntvalueout[lane]),
                .io_ddr4_dq        (io_ddr4_dq[lane*DQ_BITS +: DQ_BITS]),
                .io_ddr4_dqs_p     (io_ddr4_dqs_p[lane]),
                .io_ddr4_dqs_n     (io_ddr4_dqs_n[lane]),
                .o_ddr4_dm_n       (o_ddr4_dm_n[lane])
            );
        end
    endgenerate

    // -----------------------------------------------------------------
    // FIFO Read Enable
    //
    // Every DQ FIFO advances from one common pop decision. Calibration drains
    // complete FIFO words continuously; application traffic additionally
    // requires a reserved READ command. Idle and write DQS activity therefore
    // cannot move any read pointer, and per-bit EMPTY synchronization cannot
    // tear a BL8 word across lanes.
    // -----------------------------------------------------------------
    reg [4:0] rx_fifo_flush_count;
    reg [2:0] rx_write_discard_count;
    reg       wrlvl_en_q;
    wire      rx_fifo_flush = |rx_fifo_flush_count;
    wire      rx_write_discard = output_enable |
                                     (rx_write_discard_count != 0);
    wire [TOTAL_DQ-1:0] fifo_empty_flat;
    generate
        genvar fifo_flat_lane, fifo_flat_bit;
        for (fifo_flat_lane = 0; fifo_flat_lane < BYTE_LANES; fifo_flat_lane = fifo_flat_lane + 1)
            for (fifo_flat_bit = 0; fifo_flat_bit < DQ_BITS; fifo_flat_bit = fifo_flat_bit + 1)
                assign fifo_empty_flat[fifo_flat_lane*DQ_BITS + fifo_flat_bit] =
                    fifo_empty[fifo_flat_lane][fifo_flat_bit];
    endgenerate
    wire calibration_request = i_dfi_rdlvl_gate_en || i_dfi_rdlvl_en ||
                               i_dfi_wrlvl_en;
    // Controller training enables may contain short gaps between individual
    // calibration commands. Keep the RX path in calibration mode from the
    // first gate/eye request through the falling edge of write leveling so
    // those gaps cannot be mistaken for application traffic.
    reg  calibration_session;
    wire calibration_mode = calibration_request || calibration_session;
    wire app_account_mode = !calibration_mode && !rx_fifo_flush;
    wire app_mode = app_account_mode && !rx_write_discard;
    reg [7:0] app_read_pending;
    // Every DQ bit owns an asynchronous native FIFO. Their synchronized EMPTY
    // flags can resolve on adjacent DIV_CLK cycles after the same DQS burst;
    // pacing the bus from one arbitrary bit tears a DFI word across lanes.
    // Advance none of the FIFOs until a complete word exists in all of them.
    wire fifo_pace_not_empty = ~(|fifo_empty_flat);
    wire app_fifo_pop_request = app_mode && fifo_pace_not_empty &&
                                (app_read_pending != 0);
    generate
        genvar fifo_drive_lane;
        for (fifo_drive_lane = 0; fifo_drive_lane < BYTE_LANES;
             fifo_drive_lane = fifo_drive_lane + 1) begin : gen_fifo_rd_drive
            // In application mode a READ command reserves one native FIFO
            // word in app_read_pending.  Pop that word only after every DQ
            // FIFO reports non-empty.  DFI rddata_en is a predicted return
            // time, not proof that asynchronous per-bit FIFO flags have all
            // crossed into DIV_CLK; using it directly can tear the first
            // BL8 word across byte lanes when board/package skew is present.
            assign fifo_rd_en_drive[fifo_drive_lane] = calibration_mode ?
                {DQ_BITS{calibration_fifo_pop_q}} :
                {DQ_BITS{app_fifo_pop_request}};
        end
    endgenerate
    reg       fifo_word_valid_q;
    // RXTX_BITSLICE updates Q after the active FIFO_RD_CLK edge. Capture the
    // settled word on the opposite edge so a back-to-back pop cannot advance
    // Q to burst N+1 before burst N is transferred into the DFI register.
    reg [SERDES_RATIO*DFI_DATA_WIDTH-1:0] app_rddata_half;
    reg       app_rddata_half_valid;
    // Reserve the receive FIFO entry at the READ command, not at the
    // controller's expected-data indication.  At high speed the two-entry
    // native FIFO can overflow before DFI rddata_en reaches the PHY.
    wire app_read_issued = app_account_mode & (|dfi_read_command);
    // One qualified FIFO word advances the common raw stream for every DQ
    // slice. The previous/current 16-UI window removes the fixed preamble
    // rotation, so the first qualified word is already a complete BL8 return;
    // it must not be discarded as a separate seed word.
    wire fifo_pop_fire = calibration_mode ? calibration_fifo_pop_q :
                                            app_fifo_pop_request;
    // RXTX_BITSLICE samples FIFO_RD_EN on DIV_CLK, then updates Q after that
    // edge.  Qualify Q one DIV_CLK later; consuming it on fifo_pop_fire would
    // capture the old FIFO head (or the partially assembled live word).
    wire app_fifo_pop_fire = app_fifo_pop_request;
    wire app_pop_fire = app_mode && app_rddata_half_valid;
    // Account a request when FIFO_RD_EN actually reaches the primitive.  This
    // permits one pop per controller cycle for legal back-to-back BL8 reads,
    // while still preventing an EMPTY flag's deassertion latency from causing
    // a duplicate pop after the last reserved return.
    wire [8:0] app_pending_after_fifo_pop =
        {1'b0, app_read_pending} + {8'd0, app_read_issued} -
        {8'd0, app_fifo_pop_fire};
    wire app_returns_queued = (app_pending_after_fifo_pop != 0) ||
                              app_rddata_half_valid;
    always @(posedge i_controller_clk) begin
        if (sync_rst) begin
            rx_write_discard_count <= 3'd0;
            wrlvl_en_q <= 1'b0;
            calibration_session <= 1'b0;
            app_read_pending <= 8'd0;
            fifo_word_valid_q <= 1'b0;
            calibration_fifo_pop_q <= 1'b0;
        end else begin
            // The asynchronous native FIFO presents the word selected by this
            // cycle's read enable only after the active DIV_CLK edge.
            fifo_word_valid_q <= fifo_pop_fire;
            wrlvl_en_q <= i_dfi_wrlvl_en;
            if (calibration_request)
                calibration_session <= 1'b1;
            if (wrlvl_en_q && !i_dfi_wrlvl_en) begin
                calibration_session <= 1'b0;
            end

            // TBYTE and the serialized DQS word trail the fabric write
            // enable. Keep the calibration drain active briefly after write
            // leveling so its feedback word cannot precede application data.
            if (output_enable)
                rx_write_discard_count <= 3'd3;
            else if (rx_write_discard_count != 0)
                rx_write_discard_count <= rx_write_discard_count - 1'b1;

            if (!app_account_mode) begin
                app_read_pending <= 8'd0;
            end else begin
                case ({app_read_issued, app_fifo_pop_fire})
                    2'b10: app_read_pending <= app_read_pending + 1'b1;
                    2'b01: app_read_pending <= app_read_pending - 1'b1;
                    default: app_read_pending <= app_read_pending;
                endcase
            end

            // Calibration continuously drains a word as soon as every DQ FIFO
            // contains one. Application traffic uses the same all-DQ EMPTY
            // qualification, additionally bounded by app_read_pending so idle
            // and write DQS activity cannot advance the receive FIFOs.
            calibration_fifo_pop_q <= calibration_mode &&
                                      fifo_pace_not_empty && rst_phy_rden;
        end
    end

    wire [7:0] iserdes_dq_q [0:TOTAL_DQ-1];
    generate
        genvar map_l, map_b;
        for (map_l = 0; map_l < BYTE_LANES; map_l = map_l + 1) begin : gen_rx_map
            for (map_b = 0; map_b < DQ_BITS; map_b = map_b + 1) begin : gen_rx_bit
                assign iserdes_dq_q[map_l * DQ_BITS + map_b] = rx_dq_data[map_l][map_b*8 +: 8];
            end
        end
    endgenerate

    // -----------------------------------------------------------------
    // Write Tri-State Control
    //
    // wrdata_en_any = OR of all 4 DFI phase enables.
    // A 2-stage shift register keeps the bus driven for 2 extra controller
    // clocks (8 DDR clocks) after wrdata_en drops, covering DDR4 postamble.
    // output_enable = wrdata_en_any | shift[0] | shift[1]
    //
    // TBYTE_IN carries one active-high byte-enable bit per DFI phase.
    // BITSLICE_CONTROL serialises these controls alongside the matching 8:1
    // DQ/DQS word, so they must not be replaced by fabric T.
    // -----------------------------------------------------------------
    always @(posedge i_controller_clk) begin
        if (sync_rst) begin
            tx_wrdata_pipe0 <= {(DFI_DATA_WIDTH*4){1'b0}};
            tx_wrmask_pipe0 <= {(DM_PER_PHASE*4){1'b0}};
            wrdata_en_shift <= 3'b000;
        end else begin
            tx_wrdata_pipe0 <= i_dfi_wrdata;
            tx_wrmask_pipe0 <= i_dfi_wrdata_mask;
            wrdata_en_shift <= {wrdata_en_shift[1:0], wrdata_en_any};
        end
    end

    assign output_enable = wrdata_en_any | wrdata_en_shift[0] |
                           wrdata_en_shift[1] | wrdata_en_shift[2];
    wire wl_active;

    // For the UltraScale RXTX_BITSLICE TBYTE_IN path, high enables the
    // byte transmitter and low releases the byte to the DRAM.  This is also
    // required while receiving MPR data during read calibration.
    assign tbyte_dq  = {4{output_enable}};
    assign tbyte_dqs = (wl_active) ? 4'b1111 : {4{output_enable}};

    // -----------------------------------------------------------------
    // DQS Pattern Generation
    //
    // TX_BITSLICE D[7:0] for DQS (8:1 DDR):
    //   Normal write: 01_01_01_01 → continuous toggle
    //   Write Leveling strobe: 00_00_00_01 → single rising edge
    //   Idle: 00_00_00_00
    // -----------------------------------------------------------------
    reg wl_dqs_strobe;
    always @* begin
        if (wl_active) begin
            if (wl_dqs_strobe)
                dqs_pattern = 8'b00_00_00_01;
            else
                dqs_pattern = 8'b00_00_00_00;
        end else if (wrdata_en_any || wrdata_en_shift[0]) begin
            dqs_pattern = 8'b01_01_01_01;
        end else if (wrdata_en_shift[1]) begin
            dqs_pattern = 8'b00_00_00_01;
        end else begin
            dqs_pattern = 8'b00_00_00_00;
        end
    end

    // -----------------------------------------------------------------
    // DFI Write Data Packing (per DQ bit, per byte lane)
    //
    // Maps DFI wrdata into per-lane 8-bit TX_BITSLICE data vectors.
    // TX_BITSLICE D[7:0] = {p3_fall, p3_rise, p2_fall, p2_rise,
    //                        p1_fall, p1_rise, p0_fall, p0_rise}
    // -----------------------------------------------------------------
    generate
        genvar wr_lane, wr_bit;
        for (wr_lane = 0; wr_lane < BYTE_LANES; wr_lane = wr_lane + 1) begin : gen_wr_lane
            for (wr_bit = 0; wr_bit < DQ_BITS; wr_bit = wr_bit + 1) begin : gen_wr_bit
                localparam integer DQ_IDX = wr_lane * DQ_BITS + wr_bit;
                assign tx_dq_data[wr_lane][wr_bit*8 +: 8] = {
                    tx_wrdata_pipe0[3*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 3 fall
                    tx_wrdata_pipe0[3*DFI_DATA_WIDTH + DQ_IDX],            // phase 3 rise
                    tx_wrdata_pipe0[2*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 2 fall
                    tx_wrdata_pipe0[2*DFI_DATA_WIDTH + DQ_IDX],            // phase 2 rise
                    tx_wrdata_pipe0[1*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 1 fall
                    tx_wrdata_pipe0[1*DFI_DATA_WIDTH + DQ_IDX],            // phase 1 rise
                    tx_wrdata_pipe0[0*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 0 fall
                    tx_wrdata_pipe0[0*DFI_DATA_WIDTH + DQ_IDX]             // phase 0 rise
                };
            end
        end
    endgenerate

    // DM mask packing: same structure, inverted (active-low on DRAM)
    generate
        genvar dm_lane;
        for (dm_lane = 0; dm_lane < BYTE_LANES; dm_lane = dm_lane + 1) begin : gen_dm_pack
            if (DM_ENABLED) begin : dm_active
                assign tx_dm_data[dm_lane] = {
                    ~tx_wrmask_pipe0[3*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 3 fall
                    ~tx_wrmask_pipe0[3*DM_PER_PHASE + dm_lane],              // phase 3 rise
                    ~tx_wrmask_pipe0[2*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 2 fall
                    ~tx_wrmask_pipe0[2*DM_PER_PHASE + dm_lane],              // phase 2 rise
                    ~tx_wrmask_pipe0[1*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 1 fall
                    ~tx_wrmask_pipe0[1*DM_PER_PHASE + dm_lane],              // phase 1 rise
                    ~tx_wrmask_pipe0[0*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 0 fall
                    ~tx_wrmask_pipe0[0*DM_PER_PHASE + dm_lane]               // phase 0 rise
                };
            end else begin : dm_stub
                assign tx_dm_data[dm_lane] = 8'hFF; // no mask: DM_n always high
            end
        end
    endgenerate

    // -----------------------------------------------------------------
    // Fabric Bitslip Barrel Shifter
    // RXTX_BITSLICE has no BITSLIP pin, so calibration may search adjacent
    // words for the periodic MPR pattern. Normal application data uses the
    // complete BL8 word presented at the FIFO head.
    // -----------------------------------------------------------------
    reg [3:0]  bitslip_count_q [0:BYTE_LANES-1];
    wire [7:0] aligned_dq [0:TOTAL_DQ-1];

    // PHY training FSM state registers
    reg [$clog2(BYTE_LANES > 1 ? BYTE_LANES : 2)-1:0] train_lane;
    reg [3:0] phy_timer;

    // Native DQS-gate calibration sub-FSM. The command-side mCL remains at
    // NATIVE_GATE_MCL while this FSM scans the complete per-byte RL_DLY range.
    // Keeping coarse and fine controls independent avoids ambiguous periodic
    // windows and preserves the established command-to-PHY pipeline latency.
    localparam [4:0] GATE_WRITE_ALL = 5'd0,
                     GATE_WAIT_WRITE = 5'd1,
                     GATE_WAIT_READ = 5'd2,
                     GATE_OBSERVE = 5'd3,
                     GATE_NEXT_TAP = 5'd4,
                     GATE_FINALIZE = 5'd5,
                     GATE_WRITE_LANE = 5'd6,
                     GATE_WAIT_LANE = 5'd7,
                     GATE_COMPLETE = 5'd8,
                     GATE_CLEAR = 5'd9,
                     GATE_WAIT_CLEAR = 5'd10,
                     GATE_READ_STATUS = 5'd11,
                     GATE_WAIT_STATUS = 5'd12,
                     GATE_RELEASE_CLEAR = 5'd13,
                     GATE_WAIT_RELEASE = 5'd14,
                     GATE_RESTORE_CLEAR = 5'd15,
                     GATE_RESTORE_WAIT_CLEAR = 5'd16,
                     GATE_RESTORE_RELEASE = 5'd17,
                     GATE_RESTORE_WAIT_RELEASE = 5'd18,
                     GATE_FLUSH_WAIT = 5'd19;
    reg [4:0] gate_phase;
    reg [3:0] gate_observe_count;
    reg [BYTE_LANES-1:0] gate_fresh_seen;
    reg [BYTE_LANES-1:0] gate_match_seen;
    reg [BYTE_LANES-1:0] gate_in_range;
    reg [BYTE_LANES-1:0] gate_best_valid;
    reg [8:0] gate_cur_start [0:BYTE_LANES-1];
    reg [8:0] gate_cur_width [0:BYTE_LANES-1];
    reg [8:0] gate_best_start [0:BYTE_LANES-1];
    reg [8:0] gate_best_width [0:BYTE_LANES-1];
    reg [8:0] gate_center [0:BYTE_LANES-1];
    reg [8:0] gate_restore_tap;

    wire [BYTE_LANES-1:0] train_lane_mask =
        ({{(BYTE_LANES-1){1'b0}}, 1'b1} << train_lane);
    wire [8:0] gate_target_tap = gate_best_valid[train_lane] ?
        gate_best_start[train_lane] + (gate_best_width[train_lane] >> 1) :
        9'd0;
    wire [8:0] gate_restore_next =
        delay_step_toward(gate_restore_tap, gate_target_tap);

    // Eye training registers (phase-aware range tracking)
    reg [8:0] sweep_tap;
    reg [8:0] cur_start;
    reg [8:0] cur_width;
    reg       in_range;
    reg [8:0] best_start;
    reg [8:0] best_width;
    reg       best_valid;
    reg       pattern_found_q;
    reg       eye_observe_verify;
    reg [3:0] eye_observe_count;
    reg       eye_observe_seen;
    reg [1:0] eye_verify_retries;
    reg [BYTE_LANES-1:0] rd_lat_extra;
    reg [8:0] eye_center_tap [0:BYTE_LANES-1];
    reg [8:0] eye_best_width [0:BYTE_LANES-1];
    reg [8:0] eye_best_start [0:BYTE_LANES-1];
    wire [8:0] eye_center_candidate = best_start + (best_width >> 1);

    // Write leveling registers (ODELAYE3 DQS sweep)
    reg [8:0] wl_tap        [0:BYTE_LANES-1];
    reg [8:0] wl_dq_tap     [0:BYTE_LANES-1];
    reg       wl_seen_zero  [0:BYTE_LANES-1];
    reg [8:0] dqs_initial_tap [0:BYTE_LANES-1];
    reg [7:0] vtc_settle_counter;

    // Training failure latch registers
    reg [BYTE_LANES-1:0] gate_train_fail;
    reg [BYTE_LANES-1:0] eye_train_fail;
    reg [BYTE_LANES-1:0] wl_train_fail;

    // Unlike the component ISERDES, the native RXTX_BITSLICE FIFO advances
    // only on the write-leveling DQS edge.  Its one captured word contains
    // both pre-edge and post-edge samples, and the previously trained RX
    // IDELAY changes which part of that transition appears at any Q bit.
    // DDR4 instead holds the write-leveling feedback level on DQ after the
    // strobe.  Synchronize that static IOB level into DIV_CLK and sample it
    // after PHY_WL_ADJUST's existing 15-cycle settling interval.
    reg [BYTE_LANES-1:0] wl_feedback_meta;
    reg [BYTE_LANES-1:0] wl_feedback_sync;
    always @(posedge i_controller_clk) begin
        if (sync_rst) begin
            wl_feedback_meta <= {BYTE_LANES{1'b0}};
            wl_feedback_sync <= {BYTE_LANES{1'b0}};
        end else begin
            wl_feedback_meta <= wl_feedback_raw;
            wl_feedback_sync <= wl_feedback_meta;
        end
    end
    wire wl_feedback_sample = wl_feedback_sync[train_lane];
    reg  wl_feedback_zero;
    reg  wl_feedback_one;
    always @* begin
        wl_feedback_zero = 1'b0;
        wl_feedback_one  = 1'b0;
        case (wl_feedback_sample)
            1'b0: wl_feedback_zero = 1'b1;
            1'b1: wl_feedback_one  = 1'b1;
            default: begin end // retain the tap and retry an unknown sample
        endcase
    end

    // DQS initial delay is the BISC-calibrated quarter-cycle baseline.  A
    // detected edge can be one whole clock period beyond that baseline; use
    // the calibrated four-times estimate to preserve DQS/DQ pin phase.
    wire [10:0] wl_period_estimate = {dqs_initial_tap[train_lane], 2'b00};
    wire wl_edge_has_wrap = ({2'b00, wl_tap[train_lane]} >=
                             ({2'b00, dqs_initial_tap[train_lane]} + wl_period_estimate));
    wire [8:0] wl_final_dqs_tap = wl_edge_has_wrap ?
                                  (wl_tap[train_lane] - wl_period_estimate[8:0]) :
                                  wl_tap[train_lane];

    assign wl_active = (phy_state == PHY_WL_SAMPLE) || (phy_state == PHY_WL_ADJUST)
                     || (phy_state == PHY_WL_APPLY)  || (phy_state == PHY_WL_CHECK)
                     || (phy_state == PHY_WL_DONE);

    // -----------------------------------------------------------------
    // Receive Word Alignment
    // -----------------------------------------------------------------
    // The trained gate establishes the native FIFO's BL8 boundary, so the
    // current FIFO word is already aligned. Keep the per-lane bitslip result
    // at eight for debug/interface compatibility with the component PHY.
    generate
        genvar bs_lane, bs_bit;
        for (bs_lane = 0; bs_lane < BYTE_LANES; bs_lane = bs_lane + 1) begin : gen_bs_lane
            for (bs_bit = 0; bs_bit < DQ_BITS; bs_bit = bs_bit + 1) begin : gen_bs_bit
                localparam integer BS_IDX = bs_lane * DQ_BITS + bs_bit;
                assign aligned_dq[BS_IDX] = iserdes_dq_q[BS_IDX];
            end
        end
    endgenerate

    // RXTX_BITSLICE samples FIFO_RD_EN on the rising DIV_CLK edge and only
    // updates Q after that edge.  Capture the settled BL8 word on the falling
    // edge.  This half-cycle holding register is essential for consecutive
    // reads: at the next rising edge the native FIFO may already advance to
    // burst N+1, while DFI must still receive burst N.
    integer app_half_lane, app_half_bit, app_half_phase, app_half_idx;
    always @(negedge i_controller_clk) begin
        if (sync_rst) begin
            app_rddata_half       <= {SERDES_RATIO*DFI_DATA_WIDTH{1'b0}};
            app_rddata_half_valid <= 1'b0;
        end else begin
            app_rddata_half_valid <= app_mode && app_fifo_pop_fire;
            if (app_mode && app_fifo_pop_fire) begin
                for (app_half_lane = 0; app_half_lane < BYTE_LANES;
                     app_half_lane = app_half_lane + 1) begin
                    for (app_half_bit = 0; app_half_bit < DQ_BITS;
                         app_half_bit = app_half_bit + 1) begin
                        app_half_idx = app_half_lane * DQ_BITS + app_half_bit;
                        for (app_half_phase = 0; app_half_phase < SERDES_RATIO;
                             app_half_phase = app_half_phase + 1) begin
                            app_rddata_half[
                                app_half_phase*DFI_DATA_WIDTH + app_half_idx
                            ] <= aligned_dq[app_half_idx][2*app_half_phase];
                            app_rddata_half[
                                app_half_phase*DFI_DATA_WIDTH + TOTAL_DQ +
                                app_half_idx
                            ] <= aligned_dq[app_half_idx][2*app_half_phase + 1];
                        end
                    end
                end
            end
        end
    end

    // -----------------------------------------------------------------
    // Eye Training: Fixed-boundary MPR comparison
    // -----------------------------------------------------------------
    // Gate training establishes the native FIFO's BL8 word boundary before
    // the DQ eye sweep begins.  The eye scan must therefore compare the
    // current FIFO word at that fixed boundary.  Searching the periodic MPR
    // value anywhere in {current, previous} aliases almost every delay tap
    // and can combine samples captured under two different tap settings.
    wire [DQ_BITS-1:0] lane_mpr_bit_match [0:BYTE_LANES-1];
    wire [BYTE_LANES-1:0] lane_mpr_word_match;
    generate
        genvar mpr_lane, mpr_bit;
        for (mpr_lane = 0; mpr_lane < BYTE_LANES;
             mpr_lane = mpr_lane + 1) begin : gen_mpr_lane
            for (mpr_bit = 0; mpr_bit < DQ_BITS;
                 mpr_bit = mpr_bit + 1) begin : gen_mpr_bit
                assign lane_mpr_bit_match[mpr_lane][mpr_bit] =
                    (iserdes_dq_q[mpr_lane * DQ_BITS + mpr_bit] ===
                     MPR_PATTERN);
            end
            assign lane_mpr_word_match[mpr_lane] =
                &lane_mpr_bit_match[mpr_lane];
        end
    endgenerate

    // A byte lane has one DQS gate and one shared DQ-delay control in this
    // generic native PHY.  Use DQ[0] as the lane timing reference, matching
    // the established component-PHY calibration algorithm.  Requiring all
    // eight primitive Q words to be bit-identical rejects a valid common DQS
    // eye because individual RXTX_BITSLICE FIFO phases are independent.
    wire pattern_found_comb =
        lane_mpr_bit_match[train_lane][0];
    wire [3:0] pattern_offset_comb = 4'd8;

    // Gate training proves that the current, freshly popped FIFO word is a
    // complete BL8 MPR return. Do not search neighboring FIFO words here: one
    // can belong to the preceding RL_DLY candidate and create a false gate
    // window at a tap that clips the actual burst.
    wire [BYTE_LANES-1:0] gate_pattern_found;
    assign gate_pattern_found = lane_mpr_word_match;

    // -----------------------------------------------------------------
    // Application Read Return
    //
    // Keep DFI return registers separate from calibration control. Both blocks
    // use the same controller clock, but each register has one owner.
    // -----------------------------------------------------------------
    always @(posedge i_controller_clk) begin
        if (sync_rst) begin
            o_dfi_rddata       <= {(SERDES_RATIO*DFI_DATA_WIDTH){1'b0}};
            o_dfi_rddata_valid <= {SERDES_RATIO{1'b0}};
        end else begin
            o_dfi_rddata_valid <= {SERDES_RATIO{1'b0}};
            if (app_pop_fire) begin
                o_dfi_rddata <= app_rddata_half;
                o_dfi_rddata_valid <= {SERDES_RATIO{1'b1}};
            end
        end
    end

`ifdef SIM_NATIVE_RX_DEBUG
    // Simulation diagnostics are observational only.  Keeping them outside
    // the functional FSM makes the synthesized ownership boundaries clear.
    always @(posedge i_controller_clk) begin
        if (!sync_rst) begin
            if (|phy_rden_ready) begin
                $display("[%0t] NATIVE_RX_GATE: rden=%b pipe=%h mCL=%0d cmd=%b",
                    $realtime, phy_rden_ready,
                    read_gate_pipe[23:0], active_read_mcl,
                    dfi_read_command);
            end
            if ((|dfi_read_command) || dfi_read_expected) begin
                $display("[%0t] NATIVE_RX_CMD: cmd=%b expected=%b app=%0b cal_req=%0b cal_session=%0b flush=%0d discard=%0d",
                    $realtime, dfi_read_command, i_dfi_rddata_en,
                    app_mode, calibration_request, calibration_session,
                    rx_fifo_flush_count, rx_write_discard_count);
            end
            if (app_read_issued || app_pop_fire || app_returns_queued) begin
                $display("[%0t] NATIVE_RX: due=%0b pending=%0d after=%0d word_valid=%0b pop=%0b rden0=%h empty0=%h q0=%h q1=%h",
                    $realtime, app_read_issued, app_read_pending,
                    app_pending_after_fifo_pop, fifo_word_valid_q,
                    app_pop_fire,
                    fifo_rd_en_drive[0], fifo_empty[0],
                    iserdes_dq_q[0], iserdes_dq_q[1]);
            end
        end
    end
`endif

    // -----------------------------------------------------------------
    // Calibration Control
    // -----------------------------------------------------------------
    integer dfi_pack_idx;

    always @(posedge i_controller_clk) begin
        if (sync_rst) begin
            o_dfi_rdlvl_resp   <= {BYTE_LANES{1'b0}};
            o_dfi_wrlvl_resp   <= {BYTE_LANES{1'b0}};
            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                bitslip_count_q[dfi_pack_idx] <= 4'b0;
                idelay_load_lane[dfi_pack_idx] <= 1'b0;
                eye_center_tap[dfi_pack_idx]  <= 9'b0;
                odelay_dqs_load[dfi_pack_idx]  <= 1'b0;
                odelay_dq_load[dfi_pack_idx]   <= 1'b0;
                wl_tap[dfi_pack_idx]           <= 9'b0;
                wl_dq_tap[dfi_pack_idx]        <= 9'b0;
                wl_seen_zero[dfi_pack_idx]     <= 1'b0;
                dqs_initial_tap[dfi_pack_idx]  <= 9'b0;
                eye_best_width[dfi_pack_idx]   <= 9'b0;
                eye_best_start[dfi_pack_idx]   <= 9'b0;
                gate_cur_start[dfi_pack_idx]   <= 9'b0;
                gate_cur_width[dfi_pack_idx]   <= 9'b0;
                gate_best_start[dfi_pack_idx]  <= 9'b0;
                gate_best_width[dfi_pack_idx]  <= 9'b0;
                gate_center[dfi_pack_idx]      <= 9'b0;
                gate_trained_mcl[dfi_pack_idx] <=
                    NATIVE_GATE_MCL_INITIAL;
            end
            phy_state           <= PHY_IDLE;
            train_lane          <= 0;
            phy_timer           <= 4'b0;
            idelay_cntvalue     <= 9'b0;
            sweep_tap           <= 9'b0;
            cur_start           <= 9'b0;
            cur_width           <= 9'b0;
            in_range            <= 1'b0;
            best_start          <= 9'b0;
            best_width          <= 9'b0;
            best_valid          <= 1'b0;
            pattern_found_q     <= 1'b0;
            eye_observe_verify  <= 1'b0;
            eye_observe_count   <= 4'b0;
            eye_observe_seen    <= 1'b0;
            eye_verify_retries  <= 2'b0;
            rd_lat_extra        <= {BYTE_LANES{1'b0}};
            odelay_dqs_cntvalue <= 9'b0;
            odelay_dq_cntvalue  <= 9'b0;
            wl_dqs_strobe       <= 1'b0;
            en_vtc_q            <= 1'b1;
            // The reset sequencer's mux holds RX/TX EN_VTC High through
            // BISC.  Once rst_init_complete selects this register, COUNT
            // mode requires it to remain Low.
            bitslice_en_vtc_q   <= 1'b0;
            vtc_settle_counter  <= 8'b0;
            gate_train_fail     <= {BYTE_LANES{1'b0}};
            eye_train_fail      <= {BYTE_LANES{1'b0}};
            wl_train_fail       <= {BYTE_LANES{1'b0}};
            gate_phase          <= GATE_WRITE_ALL;
            gate_sweep_tap      <= 9'd0;
            gate_restore_tap    <= 9'd0;
            gate_observe_count  <= 4'd0;
            gate_fresh_seen     <= {BYTE_LANES{1'b0}};
            gate_match_seen     <= {BYTE_LANES{1'b0}};
            gate_in_range       <= {BYTE_LANES{1'b0}};
            gate_best_valid     <= {BYTE_LANES{1'b0}};
            native_riu_addr     <= 6'd0;
            native_riu_wr_data  <= 16'd0;
            native_riu_wr_en    <= 1'b0;
            native_riu_sel      <= {BYTE_LANES{1'b0}};
            rx_fifo_flush_count <= 5'd0;
        end else begin
            // This block is the sole owner of the flush timer.  The post-WL
            // flush and the post-gate-sweep flush intentionally share it.
            // A gate-sweep assignment later in this block has priority over
            // the normal countdown, matching the original cycle sequencing.
            if (wrlvl_en_q && !i_dfi_wrlvl_en)
                rx_fifo_flush_count <= 5'd16;
            else if (rx_fifo_flush_count != 0)
                rx_fifo_flush_count <= rx_fifo_flush_count - 1'b1;

            // Default: deassert all LOAD pulses (single-cycle pulse)
            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                idelay_load_lane[dfi_pack_idx] <= 1'b0;
                odelay_dqs_load[dfi_pack_idx]  <= 1'b0;
                odelay_dq_load[dfi_pack_idx]   <= 1'b0;
            end
            wl_dqs_strobe <= 1'b0;
            native_riu_wr_en <= 1'b0;
            native_riu_sel <= {BYTE_LANES{1'b0}};

            // ---------------------------------------------------------
            // PHY Training FSM
            // ---------------------------------------------------------
            case (phy_state)
                    PHY_IDLE: begin
                        if (i_dfi_rdlvl_gate_en) begin
`ifdef SIM_NATIVE_GATE_BYPASS
                            // Explicit XSim acceleration for data-path tests.
                            // Production builds never define this symbol and
                            // always execute the complete RIU gate sweep.
                            gate_train_fail <= {BYTE_LANES{1'b0}};
                            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES;
                                 dfi_pack_idx = dfi_pack_idx + 1) begin
                                gate_center[dfi_pack_idx] <= 9'd0;
                                gate_trained_mcl[dfi_pack_idx] <=
                                    NATIVE_GATE_MCL_INITIAL;
                            end
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b1}};
                            gate_phase <= GATE_COMPLETE;
                            phy_state <= PHY_GATE_DONE;
`else
                            // Sweep the rank-0 native read-gate delay while the
                            // controller supplies repeated MPR reads.  This is
                            // the BITSLICE_CONTROL gate training mechanism used
                            // by MIG; MPR data alone cannot detect a missing
                            // edge because its period aliases the FIFO word.
                            en_vtc_q <= 1'b0;
                            bitslice_en_vtc_q <= 1'b0;
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}};
                            gate_train_fail <= {BYTE_LANES{1'b0}};
                            gate_sweep_tap <= 9'd0;
                            gate_observe_count <= 4'd0;
                            gate_fresh_seen <= {BYTE_LANES{1'b0}};
                            gate_match_seen <= {BYTE_LANES{1'b0}};
                            gate_in_range <= {BYTE_LANES{1'b0}};
                            gate_best_valid <= {BYTE_LANES{1'b0}};
                            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES;
                                 dfi_pack_idx = dfi_pack_idx + 1) begin
                                gate_cur_start[dfi_pack_idx] <= 9'd0;
                                gate_cur_width[dfi_pack_idx] <= 9'd0;
                                gate_best_start[dfi_pack_idx] <= 9'd0;
                                gate_best_width[dfi_pack_idx] <= 9'd0;
                            end
                            gate_phase <= GATE_WRITE_ALL;
                            phy_state <= PHY_GATE_DONE;
`endif
                        end else if (i_dfi_rdlvl_en) begin
                            en_vtc_q <= 1'b0;
                            bitslice_en_vtc_q <= 1'b0;
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}};
                            train_lane <= 0;
                            sweep_tap <= 9'd0;
                            idelay_cntvalue <= 9'd0;
                            eye_train_fail <= {BYTE_LANES{1'b0}};
                            in_range <= 1'b0;
                            best_valid <= 1'b0;
                            best_width <= 9'd0;
                            cur_width <= 9'd0;
                            eye_observe_verify <= 1'b0;
                            eye_observe_count <= 4'b0;
                            eye_observe_seen <= 1'b0;
                            phy_timer <= 4'd4;
                            phy_state <= PHY_EYE_SWEEP;
                        end else if (i_dfi_wrlvl_en) begin
                            en_vtc_q <= 1'b0;
                            bitslice_en_vtc_q <= 1'b0;
                            o_dfi_wrlvl_resp <= {BYTE_LANES{1'b0}};
                            train_lane <= 0;
                            wl_train_fail <= {BYTE_LANES{1'b0}};
                            odelay_dqs_cntvalue <= odelay_dqs_cntvalueout[0];
                            odelay_dq_cntvalue  <= 9'd0;
                            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                dqs_initial_tap[dfi_pack_idx] <= odelay_dqs_cntvalueout[dfi_pack_idx];
                                wl_tap[dfi_pack_idx]    <= odelay_dqs_cntvalueout[dfi_pack_idx];
                                wl_dq_tap[dfi_pack_idx] <= 9'd0;
                                wl_seen_zero[dfi_pack_idx] <= 1'b0;
                            end
                            phy_timer <= 4'd4;
                            phy_state <= PHY_WL_SAMPLE;
                        end
                    end

                    PHY_GATE_DONE: begin
                        case (gate_phase)
                            GATE_WRITE_ALL: begin
                                // RL_DLY_RNK0[8:0] is the fine read-gate delay.
                                // Broadcast the candidate to both nibbles of
                                // every byte; each byte retains its own result.
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_wr_data <= {7'd0, gate_sweep_tap};
                                native_riu_wr_en <= 1'b1;
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                phy_timer <= 4'd15;
                                gate_phase <= GATE_WAIT_WRITE;
                            end

                            GATE_WAIT_WRITE: begin
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                if (phy_timer != 0)
                                    phy_timer <= phy_timer - 1'b1;
                                else if ((&native_riu_valid) &&
                                         (native_riu_rd_data[0][8:0] == gate_sweep_tap))
                                    gate_phase <= GATE_CLEAR;
                                else begin
`ifdef SIM_NATIVE_RIU_DEBUG
                                    $display("[%0t] NATIVE_RIU_WAIT: tap=%0d valid=%b rd0=%h",
                                        $realtime, gate_sweep_tap, native_riu_valid,
                                        native_riu_rd_data[0]);
`endif
                                    phy_timer <= 4'd15;
                                end
                            end

                            GATE_CLEAR: begin
                                // Clear the edge monitor while preserving
                                // RX/TX gating and each nibble's clock source.
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_wr_data <= RIU_GATE_CLEAR;
                                native_riu_wr_en <= 1'b1;
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                phy_timer <= 4'd15;
                                gate_phase <= GATE_WAIT_CLEAR;
                            end

                            GATE_WAIT_CLEAR: begin
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                // CLR_GATE is a pulse/self-clearing control and
                                // GT_STATUS is live, so NIBBLE_CTRL0 must not be
                                // compared as a complete 16-bit value. UG571
                                // specifies RIU_VALID as the port-availability
                                // handshake; the fixed delay also covers the
                                // normal two-RIU-clock write latency.
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (&native_riu_valid) begin
                                    gate_phase <= GATE_RELEASE_CLEAR;
                                end else begin
                                    phy_timer <= 4'd15;
                                end
                            end

                            GATE_RELEASE_CLEAR: begin
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_wr_data <= RIU_GATE_RUN;
                                native_riu_wr_en <= 1'b1;
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                gate_fresh_seen <= {BYTE_LANES{1'b0}};
                                gate_match_seen <= {BYTE_LANES{1'b0}};
                                phy_timer <= 4'd15;
                                gate_phase <= GATE_WAIT_RELEASE;
                            end

                            GATE_WAIT_RELEASE: begin
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (&native_riu_valid) begin
                                    gate_phase <= GATE_WAIT_READ;
                                end else begin
                                    phy_timer <= 4'd15;
                                end
                            end

                            GATE_WAIT_READ: begin
                                if (dfi_read_expected) begin
                                    gate_observe_count <= 4'd0;
                                    gate_fresh_seen <= {BYTE_LANES{1'b0}};
                                    gate_match_seen <= {BYTE_LANES{1'b0}};
                                    gate_phase <= GATE_OBSERVE;
                                end
                            end

                            GATE_OBSERVE: begin
                                if (calibration_fifo_pop_q)
                                    gate_fresh_seen <= {BYTE_LANES{1'b1}};
                                for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                    if (fifo_word_valid_q &&
                                        gate_pattern_found[dfi_pack_idx])
                                        gate_match_seen[dfi_pack_idx] <= 1'b1;
                                end
                                if (gate_observe_count == NATIVE_RX_OBSERVE_CYCLES - 1'b1)
                                    gate_phase <= GATE_READ_STATUS;
                                else
                                    gate_observe_count <= gate_observe_count + 1'b1;
                            end

                            GATE_READ_STATUS: begin
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                // RIU_RD_DATA is returned on the cycle after
                                // RIU_ADDR/NIBBLE_SEL are sampled. RIU_VALID
                                // only reports BISC-port availability; it is
                                // not a combinational read-valid qualifier.
                                phy_timer <= 4'd2;
                                gate_phase <= GATE_WAIT_STATUS;
                            end

                            GATE_WAIT_STATUS: begin
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (&native_riu_valid) begin
                                    // GT_STATUS identifies a DQS/gate phase
                                    // relationship, but it is not a receive-
                                    // data-valid indication.  Preserve the
                                    // fresh MPR match accumulated in
                                    // GATE_OBSERVE; replacing it with bit 9
                                    // selects broad half-cycle regions that
                                    // can still clip the BL8 FIFO word.
                                    `ifndef YOSYS
                                    `ifdef SIM_QUIET_TRAINING_LOG
                                        // Keep a full 512-tap calibration visible
                                        // in long regressions without the I/O
                                        // cost of one line per candidate.
                                if (gate_sweep_tap[3:0] == 4'd0)
                                            $display("[%0t] PHY native gate sweep: tap %0d / %0d",
                                                $realtime, gate_sweep_tap,
                                                GATE_SWEEP_LAST);
                                    `else
                                        $display("[%0t] PHY gate sweep: mCL=%0d fine=%0d status0=%0b fresh=%b mpr=%b riu0=%h empty0=%h q0=%h",
                                            $realtime,
                                            active_read_mcl,
                                            gate_sweep_tap,
                                            native_riu_rd_data[0][9],
                                            gate_fresh_seen, gate_match_seen,
                                            native_riu_rd_data[0], fifo_empty[0],
                                            iserdes_dq_q[0]);
                                    `endif
                                    `endif
                                    gate_phase <= GATE_NEXT_TAP;
                                end
                            end

                            GATE_NEXT_TAP: begin
                                for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                    if (gate_fresh_seen[dfi_pack_idx] &&
                                        gate_match_seen[dfi_pack_idx]) begin
                                        if (!gate_in_range[dfi_pack_idx]) begin
                                            gate_cur_start[dfi_pack_idx] <= gate_sweep_tap;
                                            gate_cur_width[dfi_pack_idx] <= GATE_TAP_STEP;
                                            gate_in_range[dfi_pack_idx] <= 1'b1;
                                        end else begin
                                            gate_cur_width[dfi_pack_idx] <= gate_cur_width[dfi_pack_idx] +
                                                                            GATE_TAP_STEP;
                                        end
                                    end else if (gate_in_range[dfi_pack_idx]) begin
                                        // RL_DLY can span more than one DQS
                                        // period, so later taps may expose a
                                        // wider periodic copy of the same MPR
                                        // burst. Keep the first complete
                                        // window tied to the scheduled READ
                                        // mask; selecting a later copy adds a
                                        // whole-cycle receive ambiguity.
                                        if (!gate_best_valid[dfi_pack_idx]) begin
                                            gate_best_start[dfi_pack_idx] <= gate_cur_start[dfi_pack_idx];
                                            gate_best_width[dfi_pack_idx] <= gate_cur_width[dfi_pack_idx];
                                            gate_best_valid[dfi_pack_idx] <= 1'b1;
                                        end
                                        gate_in_range[dfi_pack_idx] <= 1'b0;
                                    end
                                end

                                if (gate_sweep_tap >= GATE_SWEEP_LAST) begin
                                    gate_phase <= GATE_FINALIZE;
                                end else begin
                                    gate_sweep_tap <= gate_sweep_tap + GATE_TAP_STEP;
                                    gate_phase <= GATE_WRITE_ALL;
                                end
                            end

                            GATE_FINALIZE: begin
                                // Close a range that reaches tap 511, then
                                // choose its midpoint. The following cycle
                                // observes the committed best-range registers.
                                for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                    if (gate_in_range[dfi_pack_idx] &&
                                        !gate_best_valid[dfi_pack_idx]) begin
                                        gate_best_start[dfi_pack_idx] <= gate_cur_start[dfi_pack_idx];
                                        gate_best_width[dfi_pack_idx] <= gate_cur_width[dfi_pack_idx];
                                        gate_best_valid[dfi_pack_idx] <= 1'b1;
                                    end
                                end
                                train_lane <= 0;
                                // Every byte is still programmed to the final
                                // broadcast sweep tap.  Walk each byte back to
                                // its selected centre in legal delay-update
                                // increments instead of issuing a 511-to-low
                                // discontinuity that can leave the native gate
                                // state/read FIFO unusable despite RIU readback.
                                gate_restore_tap <= gate_sweep_tap;
                                gate_phase <= GATE_WRITE_LANE;
                            end

                            GATE_WRITE_LANE: begin
                                if (gate_best_valid[train_lane]) begin
                                    gate_center[train_lane] <= gate_target_tap;
                                    gate_trained_mcl[train_lane] <= NATIVE_GATE_MCL;
                                end else begin
                                    gate_train_fail[train_lane] <= 1'b1;
                                    gate_center[train_lane] <= 9'd0;
                                    // If no fine window was found, retain the
                                    // nominal JEDEC read latency.  A failed
                                    // gate scan is reported to the controller;
                                    // moving the command mask earlier would
                                    // silently truncate the burst tail.
                                    gate_trained_mcl[train_lane] <= NATIVE_GATE_MCL;
                                end
                                // Program the selected fine delay back into the
                                // byte currently being finalized. Use the same
                                // bounded eight-tap update policy as the native
                                // delay controls; avoiding a 511-to-low jump
                                // keeps the gate state and FIFO phase stable.
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_wr_data <= {7'd0, gate_restore_next};
                                gate_restore_tap <= gate_restore_next;
                                native_riu_wr_en <= 1'b1;
                                native_riu_sel <= train_lane_mask;
                                phy_timer <= 4'd15;
                                gate_phase <= GATE_WAIT_LANE;
                            end

                            GATE_WAIT_LANE: begin
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_sel <= train_lane_mask;
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (native_riu_valid[train_lane] &&
                                             (native_riu_rd_data[train_lane][8:0] ==
                                              gate_restore_tap)) begin
                                    if (gate_restore_tap !=
                                        gate_center[train_lane]) begin
                                        gate_phase <= GATE_WRITE_LANE;
                                    end else begin
                                        // RL_DLY readback proves that the
                                        // register changed, but the native
                                        // DQS gate still retains state from
                                        // the last sweep candidate. Re-arm
                                        // this byte at its final center before
                                        // any eye-training READ is accepted.
                                        gate_phase <= GATE_RESTORE_CLEAR;
                                    end
                                end else begin
                                    phy_timer <= 4'd15;
                                end
                            end

                            GATE_RESTORE_CLEAR: begin
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_wr_data <= RIU_GATE_CLEAR;
                                native_riu_wr_en <= 1'b1;
                                native_riu_sel <= train_lane_mask;
                                phy_timer <= 4'd15;
                                gate_phase <= GATE_RESTORE_WAIT_CLEAR;
                            end

                            GATE_RESTORE_WAIT_CLEAR: begin
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_sel <= train_lane_mask;
                                if (phy_timer != 0)
                                    phy_timer <= phy_timer - 1'b1;
                                else if (native_riu_valid[train_lane])
                                    gate_phase <= GATE_RESTORE_RELEASE;
                                else
                                    phy_timer <= 4'd15;
                            end

                            GATE_RESTORE_RELEASE: begin
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_wr_data <= RIU_GATE_RUN;
                                native_riu_wr_en <= 1'b1;
                                native_riu_sel <= train_lane_mask;
                                phy_timer <= 4'd15;
                                gate_phase <= GATE_RESTORE_WAIT_RELEASE;
                            end

                            GATE_RESTORE_WAIT_RELEASE: begin
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_sel <= train_lane_mask;
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (native_riu_valid[train_lane]) begin
`ifndef YOSYS
                                    $display("[%0t] PHY gate: lane %0d start_tap=%0d width=%0d center_tap=%0d mCL=%0d fail=%0b",
                                        $realtime, train_lane,
                                        gate_best_start[train_lane],
                                        gate_best_width[train_lane],
                                        gate_center[train_lane],
                                        gate_trained_mcl[train_lane],
                                        gate_train_fail[train_lane]);
`endif
                                    if (train_lane < BYTE_LANES - 1) begin
                                        train_lane <= train_lane + 1'b1;
                                        gate_restore_tap <= gate_sweep_tap;
                                        gate_phase <= GATE_WRITE_LANE;
                                    end else begin
                                        // The complete sweep deliberately
                                        // captures words at many invalid and
                                        // periodic gate positions. Flush all
                                        // native RX FIFOs after restoring the
                                        // final per-lane centers so eye
                                        // training cannot inherit a stale
                                        // word or pointer phase from a later
                                        // sweep candidate.
                                        rx_fifo_flush_count <= 5'd16;
                                        gate_phase <= GATE_FLUSH_WAIT;
                                    end
                                end else begin
                                    phy_timer <= 4'd15;
                                end
                            end

                            GATE_FLUSH_WAIT: begin
                                if (rx_fifo_flush_count == 0) begin
                                    en_vtc_q <= 1'b1;
                                    bitslice_en_vtc_q <= 1'b0;
                                    gate_phase <= GATE_COMPLETE;
                                end
                            end

                            GATE_COMPLETE: begin
                                o_dfi_rdlvl_resp <= {BYTE_LANES{1'b1}};
                                if (!i_dfi_rdlvl_gate_en) begin
                                    o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}};
                                    phy_state <= PHY_IDLE;
                                end
                            end

                            default: gate_phase <= GATE_WRITE_ALL;
                        endcase
                        end

                    PHY_EYE_SWEEP: begin
                        if (phy_timer != 0) begin
                            if (phy_timer == 4'd3)
                                idelay_load_lane[train_lane] <= 1'b1;
                            phy_timer <= phy_timer - 1'b1;
                        end else if (|i_dfi_rddata_en) begin
                            eye_observe_verify <= 1'b0;
                            eye_observe_count <= 4'b0;
                            eye_observe_seen <= 1'b0;
                            phy_state <= PHY_EYE_OBSERVE;
                        end
                    end

                    // Observe the complete native FIFO-return window.  The
                    // first exact match is retained so an X during DQS
                    // preamble/postamble cannot overwrite a valid MPR word.
                    PHY_EYE_OBSERVE: begin
                        if (fifo_word_valid_q && pattern_found_comb &&
                            !eye_observe_seen) begin
                            eye_observe_seen <= 1'b1;
                        end

                        if (eye_observe_count == NATIVE_RX_OBSERVE_CYCLES - 1'b1) begin
`ifdef SIM_NATIVE_RIU_DEBUG
                            $display("[%0t] NATIVE_EYE_SAMPLE: lane=%0d tap=%0d verify=%0b seen=%0b now=%0b off=%0d empty=%h cur=%h",
                                $realtime, train_lane, sweep_tap,
                                eye_observe_verify, eye_observe_seen,
                                pattern_found_comb,
                                pattern_offset_comb,
                                fifo_empty[train_lane],
                                iserdes_dq_q[train_lane * DQ_BITS]);
`endif
                            if (eye_observe_verify) begin
                                if (eye_observe_seen |
                                    (fifo_word_valid_q && pattern_found_comb)) begin
                                    pattern_found_q <= 1'b1;
                                    phy_state <= PHY_EYE_VERIFY_DONE;
                                end else if (eye_verify_retries != 2'd3) begin
                                    // The FIFO-return phase is asynchronous;
                                    // retry independent MPR reads before
                                    // rejecting a centre on one empty window.
                                    eye_verify_retries <= eye_verify_retries + 1'b1;
                                    phy_state <= PHY_EYE_VERIFY;
                                end else begin
                                    pattern_found_q <= 1'b0;
                                    phy_state <= PHY_EYE_VERIFY_DONE;
                                end
                            end else begin
                                pattern_found_q <= eye_observe_seen |
                                                   (fifo_word_valid_q &&
                                                    pattern_found_comb);
                                phy_state <= PHY_EYE_TRACK;
                            end
                        end else begin
                            eye_observe_count <= eye_observe_count + 1'b1;
                        end
                    end

                    PHY_EYE_TRACK: begin
                        if (pattern_found_q) begin
                            if (!in_range) begin
                                cur_start <= sweep_tap;
                                cur_width <= 9'd0;
                                in_range <= 1'b1;
                            end else begin
                                cur_width <= cur_width + {5'd0, TAP_SWEEP_STEP};
                            end
                        end else begin
                            if (in_range) begin
                                if (!best_valid || cur_width > best_width) begin
                                    best_start <= cur_start;
                                    best_width <= cur_width;
                                    best_valid <= 1'b1;
                                end
                                in_range <= 1'b0;
                            end
                        end
                        if (sweep_tap >= EYE_SWEEP_LAST) begin
                            phy_state <= PHY_EYE_DECIDE;
                        end else begin
                            sweep_tap <= sweep_tap + {5'd0, TAP_SWEEP_STEP};
                            idelay_cntvalue <= sweep_tap + {5'd0, TAP_SWEEP_STEP};
                            phy_timer <= 4'd4;
                            phy_state <= PHY_EYE_SWEEP;
                        end
                    end

                    PHY_EYE_DECIDE: begin
                        if (in_range) begin
                            if (!best_valid || cur_width > best_width) begin
                                best_start <= cur_start;
                                best_width <= cur_width;
                                best_valid <= 1'b1;
                            end
                            in_range <= 1'b0;
                        end else if (!best_valid) begin
                            eye_train_fail[train_lane] <= 1'b1;
                            `ifndef YOSYS
                                $display("[%0t] PHY eye: lane %0d no valid range found", $realtime, train_lane);
                            `endif
                            /* verilator lint_off WIDTHEXPAND */
                            if (train_lane < BYTE_LANES - 1) begin
                            /* verilator lint_on WIDTHEXPAND */
                                train_lane <= train_lane + 1'b1;
                                sweep_tap <= 9'd0;
                                idelay_cntvalue <= 9'd0;
                                in_range <= 1'b0;
                                best_valid <= 1'b0;
                                best_width <= 9'd0;
                                cur_width <= 9'd0;
                                phy_timer <= 4'd4;
                                phy_state <= PHY_EYE_SWEEP;
                            end else begin
                                phy_state <= PHY_EYE_DONE;
                            end
                        end else begin
                            eye_center_tap[train_lane] <= eye_center_candidate;
                            // MPR's periodic pattern locates the analog eye,
                            // but offsets below eight require a preceding FIFO
                            // word that does not exist for an isolated READ.
                            // The trained native DQS gate establishes the BL8
                            // boundary, so application traffic consumes Q.
                            bitslip_count_q[train_lane] <= 4'd8;
                            rd_lat_extra[train_lane] <= 1'b0;
                            eye_best_width[train_lane] <= best_width;
                            eye_best_start[train_lane] <= best_start;
                            eye_verify_retries <= 2'b0;
                            // The sweep ends at the top of the delay range.
                            // Walk back toward the chosen center eight taps
                            // at a time; a direct 508-to-center load violates
                            // the native RXTX_BITSLICE COUNT-mode limit.
                            idelay_cntvalue <= delay_step_toward(
                                idelay_cntvalue, eye_center_candidate);
                            phy_timer <= 4'd4;
                            phy_state <= PHY_EYE_CENTER;
                            `ifndef YOSYS
                                $display("[%0t] PHY eye: lane %0d best_start=%0d width=%0d center=%0d",
                                    $realtime, train_lane, best_start, best_width,
                                    eye_center_candidate);
                            `endif
                        end
                    end

                    PHY_EYE_CENTER: begin
                        if (phy_timer != 0) begin
                            if (phy_timer == 4'd3)
                                idelay_load_lane[train_lane] <= 1'b1;
                            phy_timer <= phy_timer - 1'b1;
                        end else if (idelay_cntvalue != eye_center_tap[train_lane]) begin
                            // Five controller clocks separate LOAD pulses;
                            // CNTVALUEIN is stable before every pulse.
                            idelay_cntvalue <= delay_step_toward(
                                idelay_cntvalue, eye_center_tap[train_lane]);
                            phy_timer <= 4'd4;
                        end else begin
                            // The final center is already physically loaded.
                            // Wait for the next controller-scheduled MPR read
                            // and verify that the selected eye still matches.
                            phy_state <= PHY_EYE_VERIFY;
                        end
                    end

                    PHY_EYE_VERIFY: begin
                        if (phy_timer != 0) begin
                            if (phy_timer == 4'd3)
                                idelay_load_lane[train_lane] <= 1'b1;
                            phy_timer <= phy_timer - 1'b1;
                        end else if (|i_dfi_rddata_en) begin
                            eye_observe_verify <= 1'b1;
                            eye_observe_count <= 4'b0;
                            eye_observe_seen <= 1'b0;
                            phy_state <= PHY_EYE_OBSERVE;
                        end
                    end

                    PHY_EYE_VERIFY_DONE: begin
                        if (pattern_found_q) begin
                            // Keep one explicit state between verification
                            // and lane advance. This preserves the original
                            // response timing and externally visible state
                            // encoding while making the successful/failed
                            // verification decision unambiguous.
                            /* verilator lint_off WIDTHEXPAND */
                            if (train_lane < BYTE_LANES - 1) begin
                            /* verilator lint_on WIDTHEXPAND */
                                train_lane <= train_lane + 1'b1;
                                sweep_tap <= 9'd0;
                                idelay_cntvalue <= 9'd0;
                                in_range <= 1'b0;
                                best_valid <= 1'b0;
                                best_width <= 9'd0;
                                cur_width <= 9'd0;
                                phy_timer <= 4'd4;
                                phy_state <= PHY_EYE_SWEEP;
                            end else begin
                                phy_state <= PHY_EYE_DONE;
                                `ifndef YOSYS
                                    for (dfi_pack_idx = 0;
                                         dfi_pack_idx < BYTE_LANES;
                                         dfi_pack_idx = dfi_pack_idx + 1)
                                        $display("[%0t] PHY eye done: lane %0d center=%0d bitslip=%0d rd_lat_extra=%0d",
                                            $realtime, dfi_pack_idx,
                                            eye_center_tap[dfi_pack_idx],
                                            bitslip_count_q[dfi_pack_idx],
                                            rd_lat_extra[dfi_pack_idx]);
                                `endif
                            end
                        end else begin
                            eye_train_fail[train_lane] <= 1'b1;
                            `ifndef YOSYS
                                $display("[%0t] PHY eye: lane %0d verify FAILED at center tap",
                                    $realtime, train_lane);
                            `endif
                            /* verilator lint_off WIDTHEXPAND */
                            if (train_lane < BYTE_LANES - 1) begin
                            /* verilator lint_on WIDTHEXPAND */
                                train_lane <= train_lane + 1'b1;
                                sweep_tap <= 9'd0;
                                idelay_cntvalue <= 9'd0;
                                in_range <= 1'b0;
                                best_valid <= 1'b0;
                                best_width <= 9'd0;
                                cur_width <= 9'd0;
                                phy_timer <= 4'd4;
                                phy_state <= PHY_EYE_SWEEP;
                            end else begin
                                phy_state <= PHY_EYE_DONE;
                            end
                        end
                    end

                    PHY_EYE_DONE: begin
                        o_dfi_rdlvl_resp <= {BYTE_LANES{1'b1}};
                        if (!i_dfi_rdlvl_en) begin
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}};
                            phy_state <= PHY_IDLE;
                        end
                    end

                    PHY_WL_SAMPLE: begin
                        if (phy_timer != 0) begin
                            if (phy_timer == 4'd3) begin
                                odelay_dqs_load[train_lane] <= 1'b1;
                                odelay_dq_load[train_lane]  <= 1'b1;
                            end
                            phy_timer <= phy_timer - 1'b1;
                        end else if (i_dfi_wrlvl_strobe) begin
                            wl_dqs_strobe <= 1'b1;
                            phy_timer <= 4'd15;
                            phy_state <= PHY_WL_ADJUST;
                        end
                    end

                    PHY_WL_ADJUST: begin
                        if (phy_timer != 0) begin
                            phy_timer <= phy_timer - 1'b1;
                        end else if (!(wl_feedback_zero || wl_feedback_one)) begin
                            // The result changed within this capture.  Keep
                            // the same tap and obtain a settled WL response.
                            phy_timer <= 4'd4;
                            phy_state <= PHY_WL_SAMPLE;
                        end else begin
                            `ifndef YOSYS
                            `ifdef SIM_QUIET_TRAINING_LOG
                                if (wl_tap[train_lane][3:0] == 4'd0)
                                    $display("[%0t] PHY native WL sweep: lane %0d tap %0d",
                                        $realtime, train_lane, wl_tap[train_lane]);
                            `else
                                $display("[%0t] PHY WL sweep: lane %0d dqs_tap=%0d dq_tap=%0d low=%0b high=%0b seen_low=%0b", $realtime, train_lane, wl_tap[train_lane],
                                    wl_dq_tap[train_lane], wl_feedback_zero, wl_feedback_one, wl_seen_zero[train_lane]);
                            `endif
                            `endif
                            if (wl_feedback_zero)
                                wl_seen_zero[train_lane] <= 1'b1;

                            if (wl_seen_zero[train_lane] && wl_feedback_one) begin
                                // The required low-to-high response is found.
                                // Normalize a possible extra full tCK while
                                // preserving the DQS-to-DQ phase relationship.
                                wl_tap[train_lane] <= wl_final_dqs_tap;
                                wl_dq_tap[train_lane] <= wl_final_dqs_tap - dqs_initial_tap[train_lane];
                                odelay_dqs_cntvalue <= delay_step_toward(
                                    odelay_dqs_cntvalue, wl_final_dqs_tap);
                                odelay_dq_cntvalue <= delay_step_toward(
                                    odelay_dq_cntvalue,
                                    wl_final_dqs_tap - dqs_initial_tap[train_lane]);
                                phy_timer <= 4'd4;
                                phy_state <= PHY_WL_APPLY;
                            end else if (wl_tap[train_lane][8:2] == 7'b1111111) begin
                                // If the entire range stayed high, no edge was
                                // reachable.  Retain the pre-WL BISC baseline.
                                // If a low was seen without a later high, report
                                // an actual failure but still restore safe taps.
                                if (!wl_seen_zero[train_lane] && !wl_feedback_zero) begin
                                    wl_tap[train_lane] <= dqs_initial_tap[train_lane];
                                end else begin
                                    wl_train_fail[train_lane] <= 1'b1;
                                    wl_tap[train_lane] <= dqs_initial_tap[train_lane];
                                end
                                wl_dq_tap[train_lane] <= 9'd0;
                                odelay_dqs_cntvalue <= delay_step_toward(
                                    odelay_dqs_cntvalue, dqs_initial_tap[train_lane]);
                                odelay_dq_cntvalue <= delay_step_toward(
                                    odelay_dq_cntvalue, 9'd0);
                                phy_timer <= 4'd4;
                                phy_state <= PHY_WL_APPLY;
                            end else begin
                                wl_tap[train_lane] <= wl_tap[train_lane] + {5'b0, WL_TAP_STEP};
                                wl_dq_tap[train_lane] <= wl_dq_tap[train_lane] + {5'b0, WL_TAP_STEP};
                                odelay_dqs_cntvalue <= wl_tap[train_lane] + {5'b0, WL_TAP_STEP};
                                odelay_dq_cntvalue <= wl_dq_tap[train_lane] + {5'b0, WL_TAP_STEP};
                                phy_timer <= 4'd4;
                                phy_state <= PHY_WL_SAMPLE;
                            end
                        end
                    end

                    // CNTVALUEIN must be stable before the LOAD pulse.  This
                    // also makes restore/fallback paths physically identical
                    // to a normally detected write-leveling edge.
                    PHY_WL_APPLY: begin
                        if (phy_timer != 0) begin
                            if (phy_timer == 4'd3) begin
                                odelay_dqs_load[train_lane] <= 1'b1;
                                odelay_dq_load[train_lane]  <= 1'b1;
                            end
                            phy_timer <= phy_timer - 1'b1;
                        end else if ((odelay_dqs_cntvalue != wl_tap[train_lane]) ||
                                     (odelay_dq_cntvalue != wl_dq_tap[train_lane])) begin
                            // DQS and every DQ in the lane advance together,
                            // but each delay line independently observes the
                            // native eight-tap update limit.
                            odelay_dqs_cntvalue <= delay_step_toward(
                                odelay_dqs_cntvalue, wl_tap[train_lane]);
                            odelay_dq_cntvalue <= delay_step_toward(
                                odelay_dq_cntvalue, wl_dq_tap[train_lane]);
                            phy_timer <= 4'd4;
                        end else begin
                            phy_state <= PHY_WL_CHECK;
                        end
                    end

                    PHY_WL_CHECK: begin
                        `ifndef YOSYS
                            $display("[%0t] PHY WL: lane %0d dqs_tap=%0d dq_tap=%0d", $realtime, train_lane, wl_tap[train_lane], wl_dq_tap[train_lane]);
                        `endif
                        /* verilator lint_off WIDTHEXPAND */
                        if (train_lane < BYTE_LANES - 1) begin
                        /* verilator lint_on WIDTHEXPAND */
                            train_lane <= train_lane + 1'b1;
                            wl_tap[train_lane + 1'b1]    <= dqs_initial_tap[train_lane + 1'b1];
                            wl_dq_tap[train_lane + 1'b1] <= 9'd0;
                            wl_seen_zero[train_lane + 1'b1] <= 1'b0;
                            odelay_dqs_cntvalue <= dqs_initial_tap[train_lane + 1'b1];
                            odelay_dq_cntvalue  <= 9'd0;
                            phy_timer <= 4'd4;
                            phy_state <= PHY_WL_SAMPLE;
                        end else begin
                            en_vtc_q <= 1'b1;
                            bitslice_en_vtc_q <= 1'b0;
                            vtc_settle_counter <= VTC_SETTLE_CYCLES;
                            phy_state <= PHY_WL_DONE;
                            `ifndef YOSYS
                                for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                    $display("[%0t] PHY WL done: lane %0d dqs_tap=%0d dq_tap=%0d", $realtime, dfi_pack_idx, wl_tap[dfi_pack_idx], wl_dq_tap[dfi_pack_idx]);
                                end
                            `endif
                        end
                    end

                    PHY_WL_DONE: begin
                        if (vtc_settle_counter != 0)
                            vtc_settle_counter <= vtc_settle_counter - 1'b1;
                        else begin
                            o_dfi_wrlvl_resp <= {BYTE_LANES{1'b1}};
                            if (!i_dfi_wrlvl_en) begin
                                o_dfi_wrlvl_resp <= {BYTE_LANES{1'b0}};
                                phy_state <= PHY_IDLE;
                            end
                        end
                    end

                default: ;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // Debug Status Assigns
    // -----------------------------------------------------------------
    assign o_phy_state = phy_state;
    generate
        genvar dbg_lane;
        for (dbg_lane = 0; dbg_lane < BYTE_LANES; dbg_lane = dbg_lane + 1) begin : gen_dbg
            assign o_phy_idelay_center[dbg_lane*9 +: 9] = eye_center_tap[dbg_lane];
            assign o_phy_wl_tap[dbg_lane*9 +: 9] = wl_tap[dbg_lane];
            assign o_phy_bitslip[dbg_lane*4 +: 4] = bitslip_count_q[dbg_lane];
        end
    endgenerate

    assign o_phy_train_fail_gate = gate_train_fail;
    assign o_phy_train_fail_eye  = eye_train_fail;
    assign o_phy_train_fail_wl   = wl_train_fail;

    // Extended training debug assigns (per-lane packing)
    generate
        genvar dbg_ext_lane;
        for (dbg_ext_lane = 0; dbg_ext_lane < BYTE_LANES; dbg_ext_lane = dbg_ext_lane + 1) begin : gen_dbg_ext
            assign o_phy_best_width[dbg_ext_lane*9 +: 9]      = eye_best_width[dbg_ext_lane];
            assign o_phy_best_start[dbg_ext_lane*9 +: 9]      = eye_best_start[dbg_ext_lane];
            assign o_phy_wl_dq_tap[dbg_ext_lane*9 +: 9]       = wl_dq_tap[dbg_ext_lane];
            assign o_phy_dqs_initial_tap[dbg_ext_lane*9 +: 9]  = dqs_initial_tap[dbg_ext_lane];
        end
    endgenerate
    assign o_phy_rd_lat_extra = rd_lat_extra;
    assign o_phy_en_vtc       = en_vtc_q;

endmodule
`default_nettype wire
