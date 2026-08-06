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
//   1. Gate training:  No-op (eye training subsumes it).
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
    // A board profile supplies the physical byte/nibble assignment; it is
    // deliberately separate from this DFI-facing core.
    parameter SIM_DEVICE = "ULTRASCALE_PLUS",
              PHY_PROFILE = "GENERIC",
              FIFO_PACE_LANE = (BYTE_LANES > 0) ? BYTE_LANES-1 : 0,
              FIFO_PACE_BIT  = DQ_BITS-1,
    // Native RX FIFO framing is anchored by the DDR4 read-DQS preamble.
    // The first received 8-bit word contains two preamble samples followed
    // by BL8 data beats 0..5; the next word supplies beats 6..7.  The
    // application-data window therefore starts two serial positions in.
              NATIVE_RX_PREAMBLE_BITS = 2,
    // Number of CLKDIV cycles after DFI rddata_en before the native FIFO has
    // supplied both words required by the framing window above.
              NATIVE_RX_RETURN_DELAY = 4
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
                    PHY_EYE_LATE      = 4'd6,
                    PHY_EYE_DONE      = 4'd7,
                    PHY_WL_SAMPLE     = 4'd8,
                    PHY_WL_ADJUST     = 4'd9,
                    PHY_WL_CHECK      = 4'd10,
                    PHY_WL_DONE       = 4'd11,
                    PHY_WL_APPLY      = 4'd12,
                    PHY_EYE_OBSERVE   = 4'd13;

    // MPR page 0 pattern (JESD79-4D Table 56)
    localparam [7:0] MPR_PATTERN = 8'b11110000;

    // Eye training sweep parameters
    localparam [3:0] TAP_SWEEP_STEP = 4'd4;
    localparam [3:0] WL_TAP_STEP = 4'd4;
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
                .CLKFBOUT_PHASE  (0.000),
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
                .CLKFBOUT_PHASE  (0.000),
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

    // EN_VTC split per UG571 p.296:
    //  - bsc_en_vtc: BITSLICE_CONTROL.EN_VTC (LOW during reset, HIGH after DLY_RDY)
    //  - bitslice_en_vtc: RXTX_BITSLICE.RX/TX_EN_VTC (HIGH always, LOW only during delay adj)
    reg en_vtc_q;
    wire bsc_en_vtc = rst_init_complete ? en_vtc_q : rst_en_vtc;
    reg bitslice_en_vtc_q;
    wire bitslice_en_vtc = rst_init_complete ? bitslice_en_vtc_q : 1'b1;

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

    // RX data from FIFO
    wire [DQ_BITS*8-1:0] rx_dq_data [0:BYTE_LANES-1];
    wire [DQ_BITS-1:0] fifo_empty [0:BYTE_LANES-1];
    reg fifo_rd_en;

    // Tristate
    wire [3:0] tbyte_dq;
    wire [3:0] tbyte_dqs;

    // -----------------------------------------------------------------
    // Native receive gating enable
    //
    // PHY_RDEN is deliberately not derived from i_dfi_rddata_en. That DFI
    // signal occurs at the fabric return boundary, after the DRAM DQS
    // preamble which BITSLICE_CONTROL uses to arm its native receiver. UG571
    // requires PHY_RDEN[3:0] High once native RX bring-up is complete.
    // -----------------------------------------------------------------
    wire phy_rden_ready = rst_phy_rden;

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
                .o_dly_rdy         (byte_dly_rdy[lane]),
                .o_vtc_rdy         (byte_vtc_rdy[lane]),
                .i_bsc_en_vtc      (bsc_en_vtc),
                .i_bitslice_en_vtc (bitslice_en_vtc),
                .i_tx_dq_data      (tx_dq_data[lane]),
                .i_tx_dqs_data     (dqs_pattern),
                .i_tx_dm_data      (tx_dm_data[lane]),
                .i_tbyte_dq        (tbyte_dq),
                .i_tbyte_dqs       (tbyte_dqs),
                .i_phy_rden        ({4{phy_rden_ready}}),
                .o_rx_dq_data      (rx_dq_data[lane]),
                .o_fifo_empty      (fifo_empty[lane]),
                .i_fifo_rd_en      (fifo_rd_en),
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
    // Native RX FIFOs are written by the common byte DQS clock and must be
    // drained coherently so all DQ bits retain the same BL8 word boundary.
    // Select one profile-defined pacing FIFO (normally the physical far end
    // of the byte group), register its ~FIFO_EMPTY status, and replicate the
    // enable to every DATA slice.  This matches the common FIFO_RD_EN
    // topology used by MIG's native PHY.
    // -----------------------------------------------------------------
    wire fifo_pace_not_empty = ~fifo_empty[FIFO_PACE_LANE][FIFO_PACE_BIT];
    always @(posedge i_controller_clk)
        if (sync_rst) fifo_rd_en <= 1'b0;
        else          fifo_rd_en <= fifo_pace_not_empty & rst_phy_rden;

    // Map FIFO output to iserdes_dq_q array (same format as component mode)
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
    // Native mode uses TBYTE_IN[3:0] (4-bit parallel tristate, one per 2 UI):
    //   tbyte_dq[i]  = 1 → high-Z (tristate), 0 → driven
    //   tbyte_dqs: same, but held driven (0) during write leveling
    // -----------------------------------------------------------------
    wire wrdata_en_any = |i_dfi_wrdata_en;

    reg [1:0] wrdata_en_shift;
    always @(posedge i_controller_clk) begin
        if (sync_rst)
            wrdata_en_shift <= 2'b0;
        else
            wrdata_en_shift <= {wrdata_en_shift[0], wrdata_en_any};
    end

    wire output_enable = wrdata_en_any | wrdata_en_shift[0] | wrdata_en_shift[1];
    wire wl_active;

    assign tbyte_dq  = {4{~output_enable}};
    assign tbyte_dqs = (wl_active) ? {4{1'b0}} : {4{~output_enable}};

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
        end else if (wrdata_en_any) begin
            dqs_pattern = 8'b01_01_01_01;
        end else if (wrdata_en_shift[0]) begin
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
                    i_dfi_wrdata[3*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 3 fall
                    i_dfi_wrdata[3*DFI_DATA_WIDTH + DQ_IDX],            // phase 3 rise
                    i_dfi_wrdata[2*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 2 fall
                    i_dfi_wrdata[2*DFI_DATA_WIDTH + DQ_IDX],            // phase 2 rise
                    i_dfi_wrdata[1*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 1 fall
                    i_dfi_wrdata[1*DFI_DATA_WIDTH + DQ_IDX],            // phase 1 rise
                    i_dfi_wrdata[0*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 0 fall
                    i_dfi_wrdata[0*DFI_DATA_WIDTH + DQ_IDX]             // phase 0 rise
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
                    ~i_dfi_wrdata_mask[3*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 3 fall
                    ~i_dfi_wrdata_mask[3*DM_PER_PHASE + dm_lane],              // phase 3 rise
                    ~i_dfi_wrdata_mask[2*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 2 fall
                    ~i_dfi_wrdata_mask[2*DM_PER_PHASE + dm_lane],              // phase 2 rise
                    ~i_dfi_wrdata_mask[1*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 1 fall
                    ~i_dfi_wrdata_mask[1*DM_PER_PHASE + dm_lane],              // phase 1 rise
                    ~i_dfi_wrdata_mask[0*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 0 fall
                    ~i_dfi_wrdata_mask[0*DM_PER_PHASE + dm_lane]               // phase 0 rise
                };
            end else begin : dm_stub
                assign tx_dm_data[dm_lane] = 8'hFF; // no mask: DM_n always high
            end
        end
    endgenerate

    // -----------------------------------------------------------------
    // Fabric Bitslip Barrel Shifter
    // RXTX_BITSLICE has no BITSLIP pin, so word alignment is done in
    // fabric logic. Method: concatenate {current_Q[7:0], previous_Q[7:0]}
    // into a 16-bit window and barrel-shift by the per-lane bitslip_count
    // (0-7). Gate training (MPR pattern match) determines bitslip_count.
    // Before training, bitslip_count=0 (no correction applied).
    // -----------------------------------------------------------------
    reg [7:0]  prev_iserdes_q [0:TOTAL_DQ-1];
    reg [3:0]  bitslip_count_q [0:BYTE_LANES-1];
    wire [7:0] aligned_dq [0:TOTAL_DQ-1];

    // PHY training FSM state registers
    reg [3:0] phy_state;
    reg [$clog2(BYTE_LANES > 1 ? BYTE_LANES : 2)-1:0] train_lane;
    reg [3:0] phy_timer;

    // Eye training registers (phase-aware range tracking)
    reg [8:0] sweep_tap;
    reg [8:0] cur_start;
    reg [8:0] cur_width;
    reg [3:0] cur_offset;
    reg       in_range;
    reg [8:0] best_start;
    reg [8:0] best_width;
    reg [3:0] best_offset;
    reg       best_valid;
    reg       pattern_found_q;
    reg [3:0] pattern_offset_q;
    reg       pattern_late_q;
    reg       cur_late;
    reg       best_late;
    reg       verify_mode;
    reg       eye_observe_verify;
    reg [3:0] eye_observe_count;
    reg       eye_observe_seen;
    reg [3:0] eye_observe_offset;
    reg [1:0] eye_verify_retries;
    reg [BYTE_LANES-1:0] rd_lat_extra;
    reg [NATIVE_RX_RETURN_DELAY-1:0] native_rddata_en_pipe;
    reg [2*SERDES_RATIO-1:0] ontime_shadow [0:TOTAL_DQ-1];
    // RXTX_BITSLICE FIFO output runs continuously under the registered
    // FIFO_EMPTY handshake above. Native word framing completes after the
    // DFI request, so return data after the fixed FIFO window has arrived.
    wire      dfi_read_expected = |i_dfi_rddata_en;
    wire      dfi_read_due = native_rddata_en_pipe[NATIVE_RX_RETURN_DELAY-1];
    reg [8:0] eye_center_tap [0:BYTE_LANES-1];
    reg [8:0] eye_best_width [0:BYTE_LANES-1];
    reg [8:0] eye_best_start [0:BYTE_LANES-1];

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

    // Write-leveling samples must be unanimous.  A mixed eight-UI capture
    // straddles a transition and is retried at the same delay rather than
    // becoming a false low/high decision.
    wire wl_feedback_zero = ~(|iserdes_dq_q[train_lane * DQ_BITS]);
    wire wl_feedback_one  =  &iserdes_dq_q[train_lane * DQ_BITS];

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
    // Bitslip Alignment (barrel-shift across two captures)
    // -----------------------------------------------------------------
    generate
        genvar bs_lane, bs_bit;
        for (bs_lane = 0; bs_lane < BYTE_LANES; bs_lane = bs_lane + 1) begin : gen_bs_lane
            for (bs_bit = 0; bs_bit < DQ_BITS; bs_bit = bs_bit + 1) begin : gen_bs_bit
                localparam integer BS_IDX = bs_lane * DQ_BITS + bs_bit;
                wire [15:0] iserdes_window = {iserdes_dq_q[BS_IDX], prev_iserdes_q[BS_IDX]};
                assign aligned_dq[BS_IDX] = iserdes_window[bitslip_count_q[bs_lane] +: 8];
            end
        end
    endgenerate

    // -----------------------------------------------------------------
    // Eye Training: Combinational Pattern Search
    // -----------------------------------------------------------------
    wire [15:0] train_window;
    assign train_window = {iserdes_dq_q[train_lane * DQ_BITS],
                           prev_iserdes_q[train_lane * DQ_BITS]};

    wire [8:0] offset_match;
    generate
        genvar om;
        for (om = 0; om <= 8; om = om + 1) begin : gen_offset_cmp
            assign offset_match[om] = (train_window[om +: 8] === MPR_PATTERN);
        end
    endgenerate

    wire pattern_found_comb = |offset_match;
    reg [3:0] pattern_offset_comb;
    always @* begin
        case (1'b1)
            offset_match[0]: pattern_offset_comb = 4'd0;
            offset_match[1]: pattern_offset_comb = 4'd1;
            offset_match[2]: pattern_offset_comb = 4'd2;
            offset_match[3]: pattern_offset_comb = 4'd3;
            offset_match[4]: pattern_offset_comb = 4'd4;
            offset_match[5]: pattern_offset_comb = 4'd5;
            offset_match[6]: pattern_offset_comb = 4'd6;
            offset_match[7]: pattern_offset_comb = 4'd7;
            default:         pattern_offset_comb = 4'd8;
        endcase
    end

    // -----------------------------------------------------------------
    // DFI Read Data Packing + rddata_valid + Training FSM
    // -----------------------------------------------------------------
    integer dfi_pack_lane, dfi_pack_bit, dfi_pack_phase, dfi_pack_idx;

    always @(posedge i_controller_clk) begin
        if (sync_rst) begin
            o_dfi_rddata       <= {(SERDES_RATIO*DFI_DATA_WIDTH){1'b0}};
            o_dfi_rddata_valid <= {SERDES_RATIO{1'b0}};
            o_dfi_rdlvl_resp   <= {BYTE_LANES{1'b0}};
            o_dfi_wrlvl_resp   <= {BYTE_LANES{1'b0}};
            for (dfi_pack_idx = 0; dfi_pack_idx < TOTAL_DQ; dfi_pack_idx = dfi_pack_idx + 1)
                prev_iserdes_q[dfi_pack_idx] <= 8'b0;
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
            end
            phy_state           <= PHY_IDLE;
            train_lane          <= 0;
            phy_timer           <= 4'b0;
            idelay_cntvalue     <= 9'b0;
            sweep_tap           <= 9'b0;
            cur_start           <= 9'b0;
            cur_width           <= 9'b0;
            cur_offset          <= 4'b0;
            in_range            <= 1'b0;
            best_start          <= 9'b0;
            best_width          <= 9'b0;
            best_offset         <= 4'b0;
            best_valid          <= 1'b0;
            pattern_found_q     <= 1'b0;
            pattern_offset_q    <= 4'b0;
            pattern_late_q      <= 1'b0;
            cur_late            <= 1'b0;
            best_late           <= 1'b0;
            verify_mode         <= 1'b0;
            eye_observe_verify  <= 1'b0;
            eye_observe_count   <= 4'b0;
            eye_observe_seen    <= 1'b0;
            eye_observe_offset  <= 4'b0;
            eye_verify_retries  <= 2'b0;
            rd_lat_extra        <= {BYTE_LANES{1'b0}};
            native_rddata_en_pipe <= {NATIVE_RX_RETURN_DELAY{1'b0}};
            odelay_dqs_cntvalue <= 9'b0;
            odelay_dq_cntvalue  <= 9'b0;
            wl_dqs_strobe       <= 1'b0;
            en_vtc_q            <= 1'b1;
            bitslice_en_vtc_q   <= 1'b1;
            vtc_settle_counter  <= 8'b0;
            gate_train_fail     <= {BYTE_LANES{1'b0}};
            eye_train_fail      <= {BYTE_LANES{1'b0}};
            wl_train_fail       <= {BYTE_LANES{1'b0}};
        end else begin
            // Default: deassert all LOAD pulses (single-cycle pulse)
            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                idelay_load_lane[dfi_pack_idx] <= 1'b0;
                odelay_dqs_load[dfi_pack_idx]  <= 1'b0;
                odelay_dq_load[dfi_pack_idx]   <= 1'b0;
            end
            wl_dqs_strobe <= 1'b0;

            // The controller is allowed to wait for dfi_rddata_valid.  This
            // fixed native-PHY return delay provides the two FIFO captures
            // needed to replace the DQS-preamble samples with BL8 beats 6..7.
            native_rddata_en_pipe <= {native_rddata_en_pipe[NATIVE_RX_RETURN_DELAY-2:0],
                                      dfi_read_expected};

            // Update previous ISERDES outputs for bitslip window
            for (dfi_pack_idx = 0; dfi_pack_idx < TOTAL_DQ; dfi_pack_idx = dfi_pack_idx + 1)
                prev_iserdes_q[dfi_pack_idx] <= iserdes_dq_q[dfi_pack_idx];


            o_dfi_rddata_valid <= {SERDES_RATIO{1'b0}};
            if (dfi_read_due) begin
                for (dfi_pack_lane = 0; dfi_pack_lane < BYTE_LANES; dfi_pack_lane = dfi_pack_lane + 1) begin
                    for (dfi_pack_bit = 0; dfi_pack_bit < DQ_BITS; dfi_pack_bit = dfi_pack_bit + 1) begin
                        dfi_pack_idx = dfi_pack_lane * DQ_BITS + dfi_pack_bit;
                        for (dfi_pack_phase = 0; dfi_pack_phase < SERDES_RATIO; dfi_pack_phase = dfi_pack_phase + 1) begin
                            o_dfi_rddata[dfi_pack_phase*DFI_DATA_WIDTH + dfi_pack_lane*DQ_BITS + dfi_pack_bit]
                                <= aligned_dq[dfi_pack_idx][2*dfi_pack_phase];
                            o_dfi_rddata[dfi_pack_phase*DFI_DATA_WIDTH + TOTAL_DQ + dfi_pack_lane*DQ_BITS + dfi_pack_bit]
                                <= aligned_dq[dfi_pack_idx][2*dfi_pack_phase + 1];
                        end
                    end
                end
                o_dfi_rddata_valid <= {SERDES_RATIO{1'b1}};
            end

            // ---------------------------------------------------------
            // PHY Training FSM
            // ---------------------------------------------------------
            begin
                case (phy_state)
                    PHY_IDLE: begin
                        if (i_dfi_rdlvl_gate_en) begin
                            phy_state <= PHY_GATE_DONE;
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
                            cur_late <= 1'b0;
                            best_late <= 1'b0;
                            verify_mode <= 1'b0;
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
                        o_dfi_rdlvl_resp <= {BYTE_LANES{1'b1}};
                        if (!i_dfi_rdlvl_gate_en) begin
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}};
                            phy_state <= PHY_IDLE;
                        end
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
                        if (pattern_found_comb && !eye_observe_seen) begin
                            eye_observe_seen <= 1'b1;
                            eye_observe_offset <= pattern_offset_comb;
                        end

                        if (eye_observe_count == NATIVE_RX_OBSERVE_CYCLES - 1'b1) begin
                            if (eye_observe_verify) begin
                                if (eye_observe_seen | pattern_found_comb) begin
                                    pattern_found_q <= 1'b1;
                                    pattern_offset_q <= eye_observe_seen ?
                                                        eye_observe_offset : pattern_offset_comb;
                                    verify_mode <= 1'b1;
                                    phy_state <= PHY_EYE_LATE;
                                end else if (eye_verify_retries != 2'd3) begin
                                    // The FIFO-return phase is asynchronous;
                                    // retry independent MPR reads before
                                    // rejecting a centre on one empty window.
                                    eye_verify_retries <= eye_verify_retries + 1'b1;
                                    phy_state <= PHY_EYE_VERIFY;
                                end else begin
                                    pattern_found_q <= 1'b0;
                                    verify_mode <= 1'b1;
                                    phy_state <= PHY_EYE_LATE;
                                end
                            end else begin
                                pattern_found_q <= eye_observe_seen | pattern_found_comb;
                                pattern_offset_q <= eye_observe_seen ?
                                                    eye_observe_offset : pattern_offset_comb;
                                pattern_late_q <= 1'b0;
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
                                cur_offset <= pattern_offset_q;
                                cur_late <= pattern_late_q;
                                in_range <= 1'b1;
                            end else begin
                                // RXTX FIFO word-boundary phase is independent
                                // of the analog DQ eye.  Consecutive valid MPR
                                // observations can report different 8-bit
                                // offsets without any change in eye margin.
                                cur_width <= cur_width + {5'd0, TAP_SWEEP_STEP};
                            end
                        end else begin
                            if (in_range) begin
                                if (!best_valid || cur_width > best_width) begin
                                    best_start <= cur_start;
                                    best_width <= cur_width;
                                    best_offset <= cur_offset;
                                    best_late <= cur_late;
                                    best_valid <= 1'b1;
                                end
                                in_range <= 1'b0;
                            end
                        end
                        if (sweep_tap == 9'd508) begin
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
                                best_offset <= cur_offset;
                                best_late <= cur_late;
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
                                cur_late <= 1'b0;
                                best_late <= 1'b0;
                                phy_timer <= 4'd4;
                                phy_state <= PHY_EYE_SWEEP;
                            end else begin
                                phy_state <= PHY_EYE_DONE;
                            end
                        end else begin
                            idelay_cntvalue <= best_start + (best_width >> 1);
                            eye_center_tap[train_lane] <= best_start + (best_width >> 1);
                            bitslip_count_q[train_lane] <= best_offset;
                            rd_lat_extra[train_lane] <= best_late;
                            eye_best_width[train_lane] <= best_width;
                            eye_best_start[train_lane] <= best_start;
                            eye_verify_retries <= 2'b0;
                            phy_timer <= 4'd4;
                            phy_state <= PHY_EYE_VERIFY;
                            `ifndef YOSYS
                                $display("[%0t] PHY eye: lane %0d best_start=%0d width=%0d center=%0d offset=%0d late=%0d",
                                    $realtime, train_lane, best_start, best_width,
                                    best_start + (best_width >> 1), best_offset, best_late);
                            `endif
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

                    PHY_EYE_LATE: begin
                        if (!verify_mode) begin
                            pattern_found_q <= pattern_found_comb;
                            pattern_offset_q <= pattern_offset_comb;
                            pattern_late_q <= pattern_found_comb;
                            phy_state <= PHY_EYE_TRACK;
                        end else begin
                            verify_mode <= 1'b0;
                            if (pattern_found_q) begin
                                // Centre verification is the final known MPR
                                // return. Use its recovered serial phase for
                                // the application-data barrel shifter.
                                // MPR is used to find the data-eye center.  Its
                                // repeated pattern cannot uniquely identify the
                                // native FIFO word boundary.  That boundary is
                                // instead fixed by the DDR4 read-DQS preamble:
                                // skip its two serial samples to form a BL8
                                // application-data window across two captures.
                                bitslip_count_q[train_lane] <= NATIVE_RX_PREAMBLE_BITS[3:0];
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
                                    cur_late <= 1'b0;
                                    best_late <= 1'b0;
                                    phy_timer <= 4'd4;
                                    phy_state <= PHY_EYE_SWEEP;
                                end else begin
                                    phy_state <= PHY_EYE_DONE;
                                    `ifndef YOSYS
                                        for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1)
                                            $display("[%0t] PHY eye done: lane %0d center=%0d bitslip=%0d rd_lat_extra=%0d",
                                                $realtime, dfi_pack_idx, eye_center_tap[dfi_pack_idx], bitslip_count_q[dfi_pack_idx], rd_lat_extra[dfi_pack_idx]);
                                    `endif
                                end
                            end else begin
                                eye_train_fail[train_lane] <= 1'b1;
                                `ifndef YOSYS
                                    $display("[%0t] PHY eye: lane %0d verify FAILED (late) at center tap", $realtime, train_lane);
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
                                    cur_late <= 1'b0;
                                    best_late <= 1'b0;
                                    phy_timer <= 4'd4;
                                    phy_state <= PHY_EYE_SWEEP;
                                end else begin
                                    phy_state <= PHY_EYE_DONE;
                                end
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
                                $display("[%0t] PHY WL sweep: lane %0d dqs_tap=%0d dq_tap=%0d low=%0b high=%0b seen_low=%0b", $realtime, train_lane, wl_tap[train_lane],
                                    wl_dq_tap[train_lane], wl_feedback_zero, wl_feedback_one, wl_seen_zero[train_lane]);
                            `endif
                            if (wl_feedback_zero)
                                wl_seen_zero[train_lane] <= 1'b1;

                            if (wl_seen_zero[train_lane] && wl_feedback_one) begin
                                // The required low-to-high response is found.
                                // Normalize a possible extra full tCK while
                                // preserving the DQS-to-DQ phase relationship.
                                wl_tap[train_lane] <= wl_final_dqs_tap;
                                wl_dq_tap[train_lane] <= wl_final_dqs_tap - dqs_initial_tap[train_lane];
                                odelay_dqs_cntvalue <= wl_final_dqs_tap;
                                odelay_dq_cntvalue <= wl_final_dqs_tap - dqs_initial_tap[train_lane];
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
                                odelay_dqs_cntvalue <= dqs_initial_tap[train_lane];
                                odelay_dq_cntvalue <= 9'd0;
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
                            bitslice_en_vtc_q <= 1'b1;
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
