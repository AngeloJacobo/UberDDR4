////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_phy_native.v
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  Top-level native-mode DDR4 PHY for Xilinx UltraScale+ FPGAs,
//  qualified through DDR4-2400 on the AXKU3 -2 device (see
//  HARDWARE_QUALIFICATION.md). Uses BITSLICE_CONTROL, RXTX_BITSLICE,
//  TX_BITSLICE and TX_BITSLICE_TRI with a local PLL per occupied I/O
//  clock region. The controller-facing DFI subset shares port shapes with
//  the component PHY, but latency, reset and training behavior differ.
//
// Architecture overview:
//  The PHY sits between the DFI 3.1 interface and the DDR4 SDRAM pins.
//  One controller clock cycle = 4 DDR4 CK periods = 8 data unit intervals.
//
//  Write path:  DFI wrdata -> TX_BITSLICE (8:1 DDR) -> IOBUF -> pad
//  Read path:   pad -> IOBUF -> RXTX_BITSLICE (1:8 DDR + FIFO) -> bitslip
//               barrel shifter -> DFI rddata
//  Clock path:  TX_BITSLICE (constant 01010101 toggle) -> OBUFDS -> CK/CK#
//  Cmd/Addr:    TX_BITSLICE (SDR 4:1, doubled bits) -> OBUF -> DDR4 CA pins
//
//  Training FSM (runs after reset sequencer completes, driven by MC):
//   ddr4_top selects POST_WL_READ_TRAINING: write leveling first, then
//   final read gate and eye training after the TX/DQS path is established.
//   Gate training uses RIU and per-nibble capture observations; eye training
//   sweeps RX delays and verifies MPR words before application reads.
//   Native-only observation/centering states are decoded in docs/DEBUGGING.md.
//
//   i_controller_clk drives the local PLL inputs; i_ref_clk is the RIU clock,
//   normally controller/2 from the same MMCM and phase. i_ddr4_clk is unused.
//   See docs/INTEGRATION.md and example_demo/axku3/README.md before porting.
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
    // UltraScale BIT_CTRL buses are dedicated physical nibble routes.  A
    // board wrapper must describe how its logical DDR pins occupy them:
    //   ACMD_PIN_MAP: one byte per logical CA pin, {nibble[4:0], position[2:0]}
    //   DQ_PIN_MAP:   one nibble per logical DQ, {upper_nibble, position[2:0]}
    // An all-ones map selects the canonical simulation layout.  Position 7
    // is deliberately invalid and therefore makes a safe auto-map sentinel.
              ACMD_NIBBLE_COUNT = 0,
    parameter [255:0] ACMD_PIN_MAP = {256{1'b1}},
    parameter [4*DQ_BITS*BYTE_LANES-1:0] DQ_PIN_MAP =
              {4*DQ_BITS*BYTE_LANES{1'b1}},
    // One PLL/CLKOUTPHY is required per clock region occupied by native
    // BITSLICE_CONTROLs.  Maps contain a three-bit PLL index per physical
    // ACMD nibble or byte lane.  The default is one region (all index zero).
              PLL_COUNT = 1,
    parameter [95:0] ACMD_PLL_MAP = 96'd0,
    parameter [3*BYTE_LANES-1:0] BYTE_PLL_MAP =
              {3*BYTE_LANES{1'b0}},
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
    // Board-bring-up TX-eye diagnostic.  The request is accepted only after
    // normal calibration, while the controller/prober has drained all memory
    // traffic.  DQ is a physical bit index across all byte lanes.
    (* mark_debug = "true" *) input  wire                             i_tx_diag_req,
    (* mark_debug = "true" *) input  wire [7:0]                       i_tx_diag_dq,
    (* mark_debug = "true" *) input  wire [8:0]                       i_tx_diag_tap,
    (* mark_debug = "true" *) output reg                              o_tx_diag_ack,
    (* mark_debug = "true" *) output reg                              o_tx_diag_error,
    (* mark_debug = "true" *) output wire [8:0]                       o_tx_diag_current_tap,
    (* mark_debug = "true" *) output reg  [8:0]                       o_tx_diag_previous_tap,
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
    // For the required 4:1 native DIV4 interface, CLKOUTPHY must equal the
    // DDR transfer rate (2/tCK).  The PLL supports three dedicated-PHY output
    // modes, so select a legal VCO without changing that output frequency:
    //
    //   tCK <= 1.333 ns: VCO = 4 * controller_clk, CLKOUTPHY = VCO_2X
    //   tCK >= 1.334 ns: VCO = 8 * controller_clk, CLKOUTPHY = VCO
    //
    // The second form is required by slower interfaces such as the AXKU3
    // tCK=1.600 ns configuration.  Using the first form there would produce
    // a 625 MHz VCO, below the PLLE3/PLLE4 750 MHz minimum.  Both forms still
    // deliver the identical 1.25 GHz CLKOUTPHY required at tCK=1.600 ns.
    localparam integer PLL_USE_VCO_MODE = (DDR4_CLK_PERIOD >= 1_334);
    localparam integer PLL_MULT = PLL_USE_VCO_MODE ?
                                  (2 * SERDES_RATIO) : SERDES_RATIO;
    // Match the legal UltraScale MIG PLL tuple.  CLKOUT0 is unused by this
    // PHY, but its divider is part of the validated VCO/VCO_2X configuration
    // and must track CLKOUTPHY_MODE (1 for VCO_2X, 2 for VCO).
    localparam integer PLL_CLKOUT0_DIVIDE = PLL_USE_VCO_MODE ? 2 : 1;
    localparam PLL_CLKOUTPHY_MODE = PLL_USE_VCO_MODE ? "VCO" : "VCO_2X";

    // -----------------------------------------------------------------
    // Address/Command pin count and physical nibble topology
    // -----------------------------------------------------------------
    localparam ACMD_PINS = 17 + BA_BITS + BG_BITS + 5 + 1; // addr+ba+bg+ctrl+ck
    localparam ACMD_AUTO_NIBBLES = (ACMD_PINS + 5) / 6;
    localparam ACMD_NIBBLES = (ACMD_NIBBLE_COUNT == 0) ?
                              ACMD_AUTO_NIBBLES : ACMD_NIBBLE_COUNT;

    // Resolve one logical CA pin to its physical BITSLICE_CONTROL nibble and
    // position.  The automatic layout is useful for device-independent
    // simulation; implemented designs pass the map obtained from their XDC
    // pinout because package-pin placement cannot be inferred by Verilog.
    function [7:0] acmd_map_entry;
        input integer logical_pin;
        begin
            if (&ACMD_PIN_MAP) begin
                acmd_map_entry[7:3] = logical_pin / 6;
                acmd_map_entry[2:0] = logical_pin % 6;
            end else
                acmd_map_entry = ACMD_PIN_MAP[logical_pin*8 +: 8];
        end
    endfunction

    // Invert the logical-to-physical map at elaboration so every physical
    // slot has at most one TX_BITSLICE and every logical output keeps its
    // original DDR4/DFI index.
    function integer acmd_pin_at_slot;
        input integer nibble;
        input integer position;
        integer map_pin;
        begin
            acmd_pin_at_slot = -1;
            for (map_pin = 0; map_pin < ACMD_PINS; map_pin = map_pin + 1)
                if (acmd_map_entry(map_pin) == (nibble*8 + position))
                    acmd_pin_at_slot = map_pin;
        end
    endfunction

    function integer acmd_slot_occupancy;
        input integer nibble;
        input integer position;
        integer map_pin;
        begin
            acmd_slot_occupancy = 0;
            for (map_pin = 0; map_pin < ACMD_PINS; map_pin = map_pin + 1)
                if (acmd_map_entry(map_pin) == (nibble*8 + position))
                    acmd_slot_occupancy = acmd_slot_occupancy + 1;
        end
    endfunction

    // -----------------------------------------------------------------
    // Native training encoding: state 6 differs from component mode, and
    // states 13..15 are native-only. See docs/DEBUGGING.md for both maps.
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
                    PHY_EYE_CENTER    = 4'd14,
                    PHY_EYE_REWIND    = 4'd15;

    // Declared with the state encoding because the native FIFO controller
    // below uses it to retain calibration's continuous-drain behavior.
    (* mark_debug = "true" *) reg [3:0] phy_state;

    // The controller requests page-0 MPR2 (0000_1111, sent MSB first). Keep
    // that non-periodic word for DQS-gate training because it uniquely marks
    // the BL8 boundary. During the subsequent DQ-eye sweep, alternate MPR0
    // and MPR2 on successive READs. JEDEC MPR0 is observed at the XiPHY FIFO
    // as 1010_1010, exercising every data transition, while MPR2 identifies
    // which command produced the returned word. Each newly loaded eye tap
    // consumes one tagged primer return before it is measured, so a FIFO word
    // retained from an earlier tap cannot qualify the new delay candidate.
    localparam [7:0] MPR_GATE_PATTERN = 8'b11110000;
    localparam [7:0] MPR_EYE_PATTERN  = 8'b10101010;
    localparam [7:0] MPR2_PATTERN     = MPR_GATE_PATTERN;
    localparam [7:0] MPR0_PATTERN     = MPR_EYE_PATTERN;

    // During the coarse DQS-gate search, any cyclic rotation of MPR page 0 is
    // sufficient evidence that the lane captured stable DRAM data.  This
    // deliberately does not define the application BL8 word boundary; the
    // later DQ eye sweep requires the exact, unrotated MPR word for that.
    function automatic mpr_rotation_match;
        input [7:0] sample;
        integer rotation;
        reg [15:0] repeated_sample;
        begin
            repeated_sample = {sample, sample};
            mpr_rotation_match = 1'b0;
            for (rotation = 0; rotation < 8; rotation = rotation + 1)
                if (repeated_sample[rotation +: 8] === MPR_GATE_PATTERN)
                    mpr_rotation_match = 1'b1;
        end
    endfunction

`ifdef SIM_NATIVE_DIAG_ROTATED_MPR_EYE
    function automatic [3:0] mpr_rotation_offset;
        input [7:0] rotation_matches;
        integer rotation_index;
        begin
            mpr_rotation_offset = 4'd8;
            // Iterate high-to-low so the lowest valid rotation wins.
            // Deterministic priority keeps this diagnostic helper
            // well-defined even when a periodic pattern matches more than
            // one rotation.
            for (rotation_index = 7; rotation_index >= 0;
                 rotation_index = rotation_index - 1)
                if (rotation_matches[rotation_index])
                    mpr_rotation_offset = rotation_index[3:0];
        end
    endfunction
`endif

    // Eye training sweep parameters
    localparam [3:0] TAP_SWEEP_STEP = 4'd4;
`ifdef SIM_NATIVE_TX_DEBUG_FAST_EYE
    // Directed XSim debug only: retain enough WL resolution to distinguish
    // an edge while avoiding a 128-command scan of an all-high model range.
    localparam [4:0] WL_TAP_STEP = 5'd16;
`else
    localparam [3:0] WL_TAP_STEP = 4'd4;
`endif
    // Last representable tap that can be reached exactly by the selected
    // stride.  Deriving this endpoint prevents a diagnostic (or future)
    // power-of-two stride from wrapping the nine-bit counter back to zero.
    localparam [8:0] WL_SWEEP_LAST = 9'h1ff - WL_TAP_STEP + 1'b1;
`ifdef SIM_NATIVE_GATE_DEBUG_EARLY
    // Short single-tap sweep used only while debugging the first gate window.
    localparam [4:0] GATE_TAP_STEP = 5'd1;
    localparam [8:0] GATE_SWEEP_LAST = 9'd15;
`elsif SIM_NATIVE_GATE_DEBUG_FAST
    // Full-range diagnostic gate scan at four-tap resolution.  Production
    // builds and regressions leave this undefined and test every tap.
    localparam [4:0] GATE_TAP_STEP = 5'd4;
    localparam [8:0] GATE_SWEEP_LAST = 9'd508;
`elsif SIM_NATIVE_DIAG_FAST_GATE_ONLY
    // Directed receive-boundary test: shorten only the DQS-gate sweep while
    // retaining the complete 0..508 DQ eye range.
    localparam [4:0] GATE_TAP_STEP = 5'd16;
    localparam [8:0] GATE_SWEEP_LAST = 9'd240;
`elsif SIM_NATIVE_TX_DEBUG_FAST_EYE
    // Keep the directed primitive debug run short. Production calibration
    // scans the complete RL_DLY transfer function at single-tap resolution.
    localparam [4:0] GATE_TAP_STEP = 5'd16;
    localparam [8:0] GATE_SWEEP_LAST = 9'd240;
`else
    // RL_DLY_FINE is only the fine component of the native XiPHY gate
    // control.  Sweep it at four-tap resolution while the complete coarse
    // field is searched below.  Four taps is well below the minimum accepted
    // gate window and keeps the bounded two-dimensional search practical in
    // both hardware and gate-level simulation.
    localparam [4:0] GATE_TAP_STEP = 5'd4;
    localparam [8:0] GATE_SWEEP_LAST = 9'd508;
`endif
    // A gate window clipped by an RL_DLY coarse boundary can still assert
    // GT_STATUS and complete FIFO words, but centering that short fragment
    // leaves too little margin for reset-to-reset phase and PVT movement.  In
    // hardware this appeared as a 24--28 tap fragment at the bottom of the
    // fine range while the same calibration produced 200+ tap windows on the
    // other bytes.  Do not resolve a byte from such a fragment: continue the
    // existing bounded coarse/mCL search until at least sixteen consecutive
    // production fine-sweep samples (64 taps) are valid, then center between
    // the measured edges.  This is a generic margin requirement, not a board
    // offset; it applies identically to every UltraScale/UltraScale+ byte.
    localparam [8:0] GATE_MIN_WINDOW_TAPS = 9'd64;
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

    // RXTX_BITSLICE forms one FIFO word from eight source-synchronous DQ
    // samples. A JEDEC write-leveling pulse contributes one rising and one
    // falling DQS sample, so four separate DFI strobes are required after a
    // primer. The controller's twrlvl_ww spacing lets the asynchronous DRAM
    // feedback settle between those pulses; never replace them with four
    // adjacent DQS edges in one serializer word.
    localparam [2:0] WL_CAPTURE_PULSES = 3'd4;
    // After exactly four pulses, wait for the asynchronous FIFO EMPTY and
    // registered-pop latency without generating another DQS edge. Additional
    // pulses here would leave a partial native word and change the modulo-8
    // receive phase established by read-eye training.
    localparam [4:0] WL_WORD_WAIT_MAX = 5'd31;

    // DFI Data Layout
    localparam TOTAL_DQ      = DQ_BITS * BYTE_LANES;
    localparam DM_PER_PHASE  = 2 * BYTE_LANES;
    localparam DM_ENABLED    = (DEVICE_WIDTH != 4);

`ifndef SYNTHESIS
    // Fail early on configurations that cannot match the fixed 1:4 native
    // BITSLICE datapath. These checks do not add hardware.
    integer pll_check_lane;
    initial begin
        if (SERDES_RATIO != 4)
            $error("ddr4_phy_native requires SERDES_RATIO=4");
        if (DQ_BITS != 8)
            $error("ddr4_phy_native requires eight DQ bits per byte lane");
        if (BYTE_LANES < 1)
            $error("ddr4_phy_native requires at least one byte lane");
        if ((PLL_COUNT < 1) || (PLL_COUNT > 8))
            $error("ddr4_phy_native PLL_COUNT must be in the range 1..8");
        if ((DEVICE_WIDTH != 4) && (DEVICE_WIDTH != 8) &&
            (DEVICE_WIDTH != 16))
            $error("ddr4_phy_native DEVICE_WIDTH must be x4, x8, or x16");
        if ((SIM_DEVICE != "ULTRASCALE") &&
            (SIM_DEVICE != "ULTRASCALE_PLUS"))
            $error("ddr4_phy_native SIM_DEVICE must select UltraScale or UltraScale+");
        for (pll_check_lane = 0; pll_check_lane < BYTE_LANES;
             pll_check_lane = pll_check_lane + 1)
            if (BYTE_PLL_MAP[pll_check_lane*3 +: 3] >= PLL_COUNT)
                $error("Native PHY byte %0d selects unavailable PLL %0d",
                       pll_check_lane,
                       BYTE_PLL_MAP[pll_check_lane*3 +: 3]);
    end
`endif

    // -----------------------------------------------------------------
    // PLL instances (one per occupied I/O clock region)
    // Generate CLKOUTPHY for bitslice serialization.
    // CLKIN = i_controller_clk.  For DDR4-1600 this is 200 MHz, yielding
    // VCO=800 MHz and CLKOUTPHY=1600 MHz in VCO_2X mode.  For the AXKU3
    // tCK=1.600 ns design it is 156.25 MHz, yielding VCO=CLKOUTPHY=1250 MHz
    // in VCO mode.  CLKOUTPHY stays on the dedicated XPHY route; it is never
    // promoted to a fabric/global high-speed clock.
    // i_ref_clk clocks only BITSLICE_CONTROL's low-speed RIU interface;
    // delay calibration itself still uses REFCLK_SRC=PLLCLK.
    // -----------------------------------------------------------------
    wire [PLL_COUNT-1:0] pll_clkoutphy;
    (* mark_debug = "true" *) wire [PLL_COUNT-1:0] pll_locked_i;
    (* mark_debug = "true" *) wire pll_locked;
    (* mark_debug = "true" *) wire pll_rst;
    (* mark_debug = "true" *) wire clkoutphy_en;

    /* verilator lint_off PINCONNECTEMPTY */
    // PLLE3_ADV is required by UltraScale; PLLE4_ADV is required by
    // UltraScale+.  Both expose the dedicated CLKOUTPHY path used below.
    generate
        genvar pll_region;
        for (pll_region = 0; pll_region < PLL_COUNT;
             pll_region = pll_region + 1) begin : gen_pll_region
        wire pll_clkfbout_i;
        wire pll_clkfbin_i;
        assign pll_clkfbin_i = pll_clkfbout_i;
        if (SIM_DEVICE == "ULTRASCALE") begin : gen_plle3
            PLLE3_ADV #(
                .CLKFBOUT_MULT   (PLL_MULT),
                // Match the UltraScale DDR4 MIG native-XPHY PLL.  DIV4
                // BITSLICE_CONTROL requires the dedicated PLL clock in
                // quadrature with the controller word clock so PHY_RDEN and
                // serialized data phase boundaries land on complete UI.
                .CLKFBOUT_PHASE  (90.000),
                .CLKIN_PERIOD    (CONTROLLER_CLK_PERIOD / 1000.0),
                .CLKOUT0_DIVIDE  (PLL_CLKOUT0_DIVIDE),
                .CLKOUT0_DUTY_CYCLE (0.500),
                .CLKOUT0_PHASE   (0.000),
                .CLKOUTPHY_MODE  (PLL_CLKOUTPHY_MODE),
                .COMPENSATION    ("INTERNAL"),
                .DIVCLK_DIVIDE   (1),
                .REF_JITTER      (0.010),
                .STARTUP_WAIT    ("FALSE")
            ) u_pll (
                .CLKIN        (i_controller_clk), .CLKFBIN(pll_clkfbin_i),
                .CLKFBOUT     (pll_clkfbout_i), .CLKOUT0(), .CLKOUT0B(),
                .CLKOUT1      (), .CLKOUT1B(),
                .CLKOUTPHY    (pll_clkoutphy[pll_region]),
                .LOCKED       (pll_locked_i[pll_region]),
                .CLKOUTPHYEN  (clkoutphy_en),
                .PWRDWN       (1'b0), .RST(pll_rst), .DADDR(7'd0),
                .DCLK         (1'b0), .DEN(1'b0), .DI(16'd0), .DO(),
                .DRDY         (), .DWE(1'b0)
            );
        end else begin : gen_plle4
            PLLE4_ADV #(
                .CLKFBOUT_MULT   (PLL_MULT),
                .CLKFBOUT_PHASE  (90.000),
                .CLKIN_PERIOD    (CONTROLLER_CLK_PERIOD / 1000.0),
                .CLKOUT0_DIVIDE  (PLL_CLKOUT0_DIVIDE),
                .CLKOUT0_DUTY_CYCLE (0.500),
                .CLKOUT0_PHASE   (0.000),
                .CLKOUTPHY_MODE  (PLL_CLKOUTPHY_MODE),
                .COMPENSATION    ("INTERNAL"),
                .DIVCLK_DIVIDE   (1),
                .REF_JITTER      (0.010),
                .STARTUP_WAIT    ("FALSE")
            ) u_pll (
                .CLKIN        (i_controller_clk), .CLKFBIN(pll_clkfbin_i),
                .CLKFBOUT     (pll_clkfbout_i), .CLKOUT0(), .CLKOUT1(),
                .CLKOUTPHY    (pll_clkoutphy[pll_region]),
                .LOCKED       (pll_locked_i[pll_region]),
                .CLKOUTPHYEN  (clkoutphy_en), .PWRDWN(1'b0), .RST(pll_rst),
                .DADDR        (7'd0), .DCLK(1'b0), .DEN(1'b0), .DI(16'd0),
                .DO           (), .DRDY(), .DWE(1'b0)
            );
        end
        end
    endgenerate
    /* verilator lint_on PINCONNECTEMPTY */

    // Do not release native calibration until every clock region is locked.
    assign pll_locked = &pll_locked_i;

    // -----------------------------------------------------------------
    // Reset Sequencer
    // -----------------------------------------------------------------
    (* mark_debug = "true" *) wire bsc_rst;
    (* mark_debug = "true" *) wire bitslice_rst;
    (* mark_debug = "true" *) wire rst_en_vtc;
    (* mark_debug = "true" *) wire rst_tbyte_en;
    (* mark_debug = "true" *) wire rst_phy_rden;
    (* mark_debug = "true" *) wire rst_init_complete;
    (* mark_debug = "true" *) wire [3:0] rst_phy_state;

    // BITSLICE_CONTROL and the RIU handshake run on i_ref_clk.  Assert their
    // reset immediately with the controller sequencer, then release it only
    // after two clean RIU clock edges.  This makes reset recovery independent
    // of the phase relationship between separately generated board clocks.
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
    reg [1:0] bsc_riu_rst_sync;
    always @(posedge i_ref_clk or posedge bsc_rst) begin
        if (bsc_rst)
            bsc_riu_rst_sync <= 2'b11;
        else
            bsc_riu_rst_sync <= {bsc_riu_rst_sync[0], 1'b0};
    end
    wire bsc_riu_rst = bsc_riu_rst_sync[1];
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
    reg [1:0] acmd_bsc_en_vtc_sync;

    // Consolidated native-calibration status.  UG571 requires every
    // BITSLICE_CONTROL in a native interface--including command/address
    // controls--to be ready before traffic or training can begin.
    wire [BYTE_LANES-1:0] byte_dly_rdy_raw;
    wire [BYTE_LANES-1:0] byte_vtc_rdy_raw;
    wire [ACMD_NIBBLES-1:0] acmd_dly_rdy_raw;
    wire [ACMD_NIBBLES-1:0] acmd_vtc_rdy_raw;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO", mark_debug = "true" *)
    reg [BYTE_LANES-1:0] byte_dly_rdy_meta, byte_dly_rdy;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO", mark_debug = "true" *)
    reg [BYTE_LANES-1:0] byte_vtc_rdy_meta, byte_vtc_rdy;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO", mark_debug = "true" *)
    reg [ACMD_NIBBLES-1:0] acmd_dly_rdy_meta, acmd_dly_rdy;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO", mark_debug = "true" *)
    reg [ACMD_NIBBLES-1:0] acmd_vtc_rdy_meta, acmd_vtc_rdy;

    // DLY_RDY and VTC_RDY are generated by BITSLICE_CONTROL on RIU_CLK.
    // Synchronize them before the controller reset/training sequencer uses
    // them. RL_DLY_RNK additionally requires the local-PLL input and RIU_CLK
    // to come from the same MMCM with the same phase shift (UG571).
    always @(posedge i_controller_clk) begin
        if (bsc_rst) begin
            byte_dly_rdy_meta <= {BYTE_LANES{1'b0}};
            byte_dly_rdy      <= {BYTE_LANES{1'b0}};
            byte_vtc_rdy_meta <= {BYTE_LANES{1'b0}};
            byte_vtc_rdy      <= {BYTE_LANES{1'b0}};
            acmd_dly_rdy_meta <= {ACMD_NIBBLES{1'b0}};
            acmd_dly_rdy      <= {ACMD_NIBBLES{1'b0}};
            acmd_vtc_rdy_meta <= {ACMD_NIBBLES{1'b0}};
            acmd_vtc_rdy      <= {ACMD_NIBBLES{1'b0}};
        end else begin
            byte_dly_rdy_meta <= byte_dly_rdy_raw;
            byte_dly_rdy      <= byte_dly_rdy_meta;
            byte_vtc_rdy_meta <= byte_vtc_rdy_raw;
            byte_vtc_rdy      <= byte_vtc_rdy_meta;
            acmd_dly_rdy_meta <= acmd_dly_rdy_raw;
            acmd_dly_rdy      <= acmd_dly_rdy_meta;
            acmd_vtc_rdy_meta <= acmd_vtc_rdy_raw;
            acmd_vtc_rdy      <= acmd_vtc_rdy_meta;
        end
    end
    (* mark_debug = "true" *) wire all_dly_rdy =
        (&byte_dly_rdy) & (&acmd_dly_rdy);
    (* mark_debug = "true" *) wire all_vtc_rdy =
        (&byte_vtc_rdy) & (&acmd_vtc_rdy);

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
        .o_tbyte_en       (rst_tbyte_en),
        .o_phy_rden       (rst_phy_rden),
        .o_init_complete  (rst_init_complete),
        .o_phy_state      (rst_phy_state)
    );
    /* verilator lint_on PINCONNECTEMPTY */

    // Fabric reset: held HIGH until reset sequencer completes
    (* mark_debug = "true" *) reg sync_rst;
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

    // EN_VTC split per UG571 native-mode bring-up and TIME/VAR_LOAD rules:
    //  - bsc_en_vtc: BITSLICE_CONTROL.EN_VTC (LOW during reset, HIGH after DLY_RDY)
    //  - bitslice_en_vtc: DQ RX TIME-mode maintenance. It is High normally,
    //    Low only while eye training performs VAR_LOAD updates. DQS and all TX
    //    FIXED delays keep EN_VTC High locally in the byte wrapper.
    reg en_vtc_q;
    (* mark_debug = "true" *) wire bsc_en_vtc =
        rst_init_complete ? en_vtc_q : rst_en_vtc;
    reg bitslice_en_vtc_q;
    // Declared before the EN_VTC mux so Verilog cannot infer a separate
    // one-bit implicit net for this reset guard.
    (* mark_debug = "true" *) wire rx_fifo_reset_active;
    // Consecutive controller clocks for which every native byte/nibble has
    // reported completed BISC before entering a DQ VAR_LOAD session.
    reg [3:0] eye_vtc_ready_count;
    // Every TIME-mode RXTX_BITSLICE must keep RX_EN_VTC High whenever its
    // receive datapath is reset.  Xilinx documents this as a BISC requirement:
    // resetting a slice while RX_EN_VTC is Low can leave DLY_RDY/VTC_RDY
    // permanently deasserted in hardware even though the UNISIM model carries
    // on.  Calibration may lower EN_VTC only after the reset/flush interval has
    // ended; the existing eye timer then supplies the required settling time
    // before the first VAR_LOAD pulse.
    (* mark_debug = "true" *) wire bitslice_en_vtc =
        (!rst_init_complete || rx_fifo_reset_active) ? 1'b1 :
        bitslice_en_vtc_q;

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
    reg eye_boundary_retime_done;

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
    (* mark_debug = "true" *) wire [SERDES_RATIO-1:0] dfi_read_command;
    (* mark_debug = "true" *) wire dfi_read_expected;
    // Consecutive eye-training READs use different MPR locations.  Combined
    // with the per-tap primer below, the measured return is both fresh for the
    // loaded delay and tagged with the location driven by its READ command.
    reg mpr_alt_page_q;
    reg [7:0] mpr_expected_pattern_q;
    wire [BA_BITS-1:0] native_mpr_bank = mpr_alt_page_q ?
        {{(BA_BITS-2){1'b0}}, 2'b10} : {BA_BITS{1'b0}};
    wire [4*BA_BITS-1:0] native_dfi_bank = i_dfi_rdlvl_en ?
        {native_mpr_bank, native_mpr_bank,
         native_mpr_bank, native_mpr_bank} : i_dfi_bank;
    always @(posedge i_controller_clk) begin
        if (sync_rst || !i_dfi_rdlvl_en) begin
            mpr_alt_page_q <= 1'b0;
            mpr_expected_pattern_q <= MPR0_PATTERN;
        end else begin
            if (|dfi_read_command) begin
                // Associate the observation with the MPR location driven by
                // this READ.  Eye candidates change the input delay between
                // commands, so comparing against the preceding command tag
                // can accept a stale FIFO word measured at the previous tap.
                // Alternating MPR0/MPR2 makes such a word fail until the
                // current command's distinct pattern reaches the reader.
                mpr_expected_pattern_q <=
                    mpr_alt_page_q ? MPR2_PATTERN : MPR0_PATTERN;
            end
            // Keep BA stable throughout command serialization.  The return
            // marker is many controller clocks before the next calibration
            // READ, so it is the unambiguous point to select the next page.
            if (dfi_read_expected)
                mpr_alt_page_q <= ~mpr_alt_page_q;
        end
    end
    generate
        genvar babit;
        for (babit = 0; babit < BA_BITS; babit = babit + 1) begin : gen_acmd_ba
            localparam integer PIN_IDX = 17 + babit;
            assign acmd_data[PIN_IDX*8 +: 8] = {
                native_dfi_bank[BA_BITS*3 + babit], native_dfi_bank[BA_BITS*3 + babit],
                native_dfi_bank[BA_BITS*2 + babit], native_dfi_bank[BA_BITS*2 + babit],
                native_dfi_bank[BA_BITS*1 + babit], native_dfi_bank[BA_BITS*1 + babit],
                native_dfi_bank[BA_BITS*0 + babit], native_dfi_bank[BA_BITS*0 + babit]};
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
    // Address/Command TX_BITSLICE + BITSLICE_CONTROL (per physical nibble)
    //
    // BIT_CTRL routing is dedicated silicon, not general fabric routing.
    // Consequently a BITSLICE_CONTROL may connect only to TX_BITSLICEs in
    // its own physical nibble.  Logical DDR pin order is unrelated to package
    // nibble order, and an upper nibble can use all seven positions (0..6).
    // ACMD_PIN_MAP provides that board/package topology without changing the
    // controller-facing signal order.
    // -----------------------------------------------------------------
    /* verilator lint_off PINCONNECTEMPTY */
    /* verilator lint_off PINMISSING */
    generate
        genvar nib, pos;
        for (nib = 0; nib < ACMD_NIBBLES; nib = nib + 1) begin : gen_acmd_nibble
            localparam [2:0] ACMD_PLL_INDEX =
                ACMD_PLL_MAP[nib*3 +: 3];
            // BIT_CTRL buses for this nibble
            wire [39:0] rx_bctrl_out [0:6];
            wire [39:0] tx_bctrl_out [0:6];
            wire [39:0] rx_bctrl_in  [0:6];
            wire [39:0] tx_bctrl_in  [0:6];

            // Every unused physical slot returns an idle bus to its control.
            // Active slots are driven exactly once by the slice below.
            genvar tiepos;
            for (tiepos = 0; tiepos < 7; tiepos = tiepos + 1) begin : gen_tie_default
                localparam integer SLOT_PIN = acmd_pin_at_slot(nib, tiepos);
                if (SLOT_PIN < 0) begin : tie_unused
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
                .PLL_CLK            (pll_clkoutphy[ACMD_PLL_INDEX]),
                .REFCLK             (1'b0),
                .RIU_CLK            (i_ref_clk),
                .RST                (bsc_riu_rst),
                .EN_VTC             (acmd_bsc_en_vtc_sync[1]),
                .DLY_RDY            (acmd_dly_rdy_raw[nib]),
                .VTC_RDY            (acmd_vtc_rdy_raw[nib]),
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

            // TX_BITSLICE instances for occupied physical positions.  The
            // primitive output reconnects to the original logical pin index,
            // so this placement mapping never permutes DDR commands.
            for (pos = 0; pos < 7; pos = pos + 1) begin : gen_acmd_txbs
                localparam integer LOGICAL_PIN = acmd_pin_at_slot(nib, pos);
                if (LOGICAL_PIN >= 0) begin : active_pin
                    TX_BITSLICE #(
                        .DATA_WIDTH     (8),
                        .DELAY_FORMAT   ("COUNT"),
                        .DELAY_TYPE     ("FIXED"),
                        .DELAY_VALUE    (0),
                        // CK and CA share the same native serializer boundary,
                        // so an unshifted CK edge has no board-skew margin from
                        // a CA transition. Shift only forwarded CK by 90
                        // degrees; the component PHY implements the same
                        // separation with a tCK/4 CK ODELAY, and UG571 defines
                        // OUTPUT_PHASE_90 for phase-shifting a generated clock
                        // relative to generated data. This keeps device-to-
                        // device fly-by skew away from the sampling boundary.
                        .OUTPUT_PHASE_90(
                            (LOGICAL_PIN == CK_PIN_IDX) ? "TRUE" : "FALSE"),
                        // The AXKU3 constraint requests DDR4 RDRV_240
                        // pre-emphasis.  UG571 requires the matching
                        // BITSLICE enable, and the generated DDR4 MIG sets
                        // it on every DDR4 TX/RXTX slice.  Keeping the
                        // primitive enable explicit also makes the generic
                        // PHY independent of tool-default changes.
                        .ENABLE_PRE_EMPHASIS("TRUE"),
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
                        .D              (acmd_data[LOGICAL_PIN*8 +: 8]),
                        .O              (acmd_out[LOGICAL_PIN]),
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

`ifndef SYNTHESIS
    // Fail early in simulation if a board profile is malformed.  Hardware
    // implementation would otherwise report a much less useful dedicated-
    // route placement error after a long synthesis run.
    integer acmd_check_pin;
    integer acmd_check_nib;
    integer acmd_check_pos;
    initial begin
        if ((ACMD_NIBBLES < 1) || (ACMD_NIBBLES > 31))
            $error("Native PHY ACMD_NIBBLE_COUNT=%0d is outside 1..31",
                   ACMD_NIBBLES);
        for (acmd_check_pin = 0; acmd_check_pin < ACMD_PINS;
             acmd_check_pin = acmd_check_pin + 1) begin
            if (((acmd_map_entry(acmd_check_pin) >> 3) >= ACMD_NIBBLES) ||
                ((acmd_map_entry(acmd_check_pin) & 8'h07) > 8'd6))
                $error("Native PHY ACMD pin %0d has invalid map entry 0x%02x",
                       acmd_check_pin, acmd_map_entry(acmd_check_pin));
        end
        for (acmd_check_nib = 0; acmd_check_nib < ACMD_NIBBLES;
             acmd_check_nib = acmd_check_nib + 1) begin
            if (ACMD_PLL_MAP[acmd_check_nib*3 +: 3] >= PLL_COUNT)
                $error("Native PHY ACMD nibble %0d selects unavailable PLL %0d",
                       acmd_check_nib,
                       ACMD_PLL_MAP[acmd_check_nib*3 +: 3]);
            for (acmd_check_pos = 0; acmd_check_pos < 7;
                 acmd_check_pos = acmd_check_pos + 1)
                if (acmd_slot_occupancy(acmd_check_nib,
                                        acmd_check_pos) > 1)
                    $error("Native PHY ACMD nibble %0d position %0d is mapped more than once",
                           acmd_check_nib, acmd_check_pos);
        end
    end
`endif
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
    (* mark_debug = "true" *) reg [8:0] idelay_cntvalue;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] idelay_load_lane;
    // The common byte sweep locates every individual DQ eye in one pass.
    // Centering then loads one relative TIME-mode offset per DQ, maximizing
    // margin without changing the external DFI contract.
    reg [DQ_BITS*9-1:0] idelay_cntvalue_per_dq;
    reg                 idelay_per_dq_mode;
    (* mark_debug = "true" *) reg [8:0] odelay_dqs_cntvalue;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] odelay_dqs_load;
    wire [8:0] odelay_dqs_cntvalueout [0:BYTE_LANES-1];
    // Per-bit DQ TX delay controls used only by the post-failure write-eye
    // diagnostic.  These controls remain idle and EN_VTC remains High during
    // every normal calibration and application transaction.
    (* mark_debug = "true" *) reg [8:0] tx_dq_cntvaluein;
    (* mark_debug = "true" *) reg [TOTAL_DQ-1:0] tx_dq_load;
    (* mark_debug = "true" *) reg [TOTAL_DQ-1:0] tx_dq_ce;
    (* mark_debug = "true" *) reg tx_dq_inc;
    (* mark_debug = "true" *) reg tx_dq_en_vtc;
    wire [DQ_BITS*9-1:0] tx_dq_cntvalueout [0:BYTE_LANES-1];
    wire [TOTAL_DQ*9-1:0] tx_dq_cntvalueout_flat;
    (* mark_debug = "true" *) wire [8:0]
        rx_max_relative_offset [0:BYTE_LANES-1];
    (* mark_debug = "true" *) wire [BYTE_LANES-1:0] rx_align_valid;

    // TX data signals
    wire [DQ_BITS*8-1:0] tx_dq_data [0:BYTE_LANES-1];
    wire [7:0] tx_dm_data [0:BYTE_LANES-1];
    reg  [7:0] dqs_pattern;

    // The native TX clock network launches a serial word after it is
    // presented to RXTX_BITSLICE. Preserve the DFI bundle at that boundary
    // while the exact native serializer phase is selected below.
    wire wrdata_en_any = |i_dfi_wrdata_en;
    reg [DFI_DATA_WIDTH*4-1:0] tx_wrdata_early;
    reg [DFI_DATA_WIDTH*4-1:0] tx_wrdata_pipe0;
    reg [DM_PER_PHASE*4-1:0]   tx_wrmask_early;
    reg [DM_PER_PHASE*4-1:0]   tx_wrmask_pipe0;
    reg [2:0]                  wrdata_en_shift;
    // The first BL8 after an idle interval has an otherwise unused native
    // serializer word immediately before its payload.  Present the captured
    // DFI word in that slot as well as in the payload slot so DQ/DM are
    // already stable when TBYTE opens the DQS preamble.  This is essential at
    // high data rates: without the predrive, unshifted DQ becomes valid only
    // one quarter-tCK before the first 90-degree DQS edge.  Do not predrive
    // later words in a contiguous run; their preceding serializer word is the
    // previous BL8 payload and must retain full-rate write throughput.
    (* mark_debug = "true" *) wire tx_predrive_first =
        wrdata_en_shift[0] && !wrdata_en_shift[1];
    wire [DFI_DATA_WIDTH*4-1:0] tx_wrdata_native =
        tx_predrive_first ? tx_wrdata_early : tx_wrdata_pipe0;
    wire [DM_PER_PHASE*4-1:0] tx_wrmask_native =
        tx_predrive_first ? tx_wrmask_early : tx_wrmask_pipe0;
    // Keep the physical write-window qualifier as a named net.  The AXKU3
    // debug constraint probes this boundary directly to distinguish write
    // turnaround from receive-gate activity; preserving it has no functional
    // effect on the PHY datapath.
    (* mark_debug = "true" *) wire output_enable;
    (* mark_debug = "true" *) wire rx_input_disable;
    (* mark_debug = "true" *) wire rx_dqs_input_disable;
    // Application writes can begin while a preceding native-FIFO return is
    // still crossing into DIV_CLK.  The turnaround controller below preserves
    // those older words and tracks the interval in which the old implementation
    // reset the RX FIFO.  RX_RST must not be used for that application-time
    // cleanup: it deasserts synchronously to the incoming DQS domain, which is
    // stopped between DDR4 reads, so the next burst would lose its first UI.
    (* mark_debug = "true" *) wire app_write_cleanup_window;
    wire wl_rx_fifo_reset;

    // RX data from FIFO
    wire [DQ_BITS*8-1:0] rx_dq_data [0:BYTE_LANES-1];
    wire [7:0] aligned_dq [0:DQ_BITS*BYTE_LANES-1];
    wire [DQ_BITS-1:0] fifo_empty [0:BYTE_LANES-1];
    // Calibration operates on independent DQS byte lanes.  Do not couple a
    // valid byte to another byte's asynchronous FIFO EMPTY flag: package and
    // board skew can legitimately make their synchronized flags arrive on
    // different DIV_CLK cycles.  Application traffic remains all-lane
    // coherent below.
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        calibration_fifo_pop_q;
    wire [DQ_BITS-1:0] fifo_rd_en_drive [0:BYTE_LANES-1];
    wire [BYTE_LANES-1:0] dqs_fifo_empty;
    wire [BYTE_LANES*8-1:0] dqs_fifo_data;
    (* mark_debug = "true" *) wire [BYTE_LANES*4-1:0]
        dbg_byte_nibble_ready;

    // RIU is used only by native DQS-gate training.  The state machine owns
    // the logical transaction; each byte below registers a physical copy so
    // the dedicated CLB-to-RIU routes never receive a cross-byte fanout.
    (* mark_debug = "true" *) reg [5:0] native_riu_addr;
    (* mark_debug = "true" *) reg [15:0] native_riu_wr_data;
    (* mark_debug = "true" *) reg native_riu_wr_en;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] native_riu_lower_sel;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] native_riu_sel;
    wire [15:0] native_riu_rd_data [0:BYTE_LANES-1];
    (* mark_debug = "true" *) wire [BYTE_LANES-1:0] native_riu_valid;
    (* mark_debug = "true" *) wire [BYTE_LANES-1:0]
        native_riu_gate_status_sticky;

    // Post-failure TX-eye diagnostics share the same physical RIU ingress as
    // calibration.  The override is asserted only after BIST has stopped all
    // memory traffic; normal gate/eye/write-level training retains exclusive
    // ownership of native_riu_* above.  Keeping arbitration at this boundary
    // also preserves the byte-local RIU registers required by implementation.
    localparam integer TX_DIAG_LANE_W =
        $clog2(BYTE_LANES > 1 ? BYTE_LANES : 2);
    (* mark_debug = "true" *) reg tx_diag_riu_override;
    (* mark_debug = "true" *) reg [5:0] tx_diag_riu_addr;
    (* mark_debug = "true" *) reg [15:0] tx_diag_riu_wr_data;
    (* mark_debug = "true" *) reg tx_diag_riu_wr_en;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        tx_diag_riu_lower_sel;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        tx_diag_riu_upper_sel;

    wire [5:0] byte_riu_addr = tx_diag_riu_override ?
        tx_diag_riu_addr : native_riu_addr;
    wire [15:0] byte_riu_wr_data = tx_diag_riu_override ?
        tx_diag_riu_wr_data : native_riu_wr_data;
    wire byte_riu_wr_en = tx_diag_riu_override ?
        tx_diag_riu_wr_en : native_riu_wr_en;
    wire [BYTE_LANES-1:0] byte_riu_lower_sel =
        tx_diag_riu_override ? tx_diag_riu_lower_sel :
                               native_riu_lower_sel;
    wire [BYTE_LANES-1:0] byte_riu_upper_sel =
        tx_diag_riu_override ? tx_diag_riu_upper_sel : native_riu_sel;
    // Keep nibble-wide BISC VT tracking enabled during the post-failure
    // transaction. NIBBLE_CTRL0[10] hands only TX delay ownership to RIU;
    // lowering BITSLICE_CONTROL.EN_VTC here also tears down the calibrated RX
    // gate and previously left all pending reads without a return strobe.
    wire byte_bsc_en_vtc = bsc_en_vtc;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
    reg [1:0] byte_bsc_en_vtc_sync;

    // EN_VTC is part of the RIU-clocked BITSLICE_CONTROL interface.  Its
    // synchronizers intentionally run beside the command toggle so a write
    // cannot reach the primitive before the requested VTC state is visible.
    always @(posedge i_ref_clk) begin
        if (bsc_riu_rst) begin
            acmd_bsc_en_vtc_sync <= 2'b00;
            byte_bsc_en_vtc_sync <= 2'b00;
        end else begin
            acmd_bsc_en_vtc_sync <=
                {acmd_bsc_en_vtc_sync[0], bsc_en_vtc};
            byte_bsc_en_vtc_sync <=
                {byte_bsc_en_vtc_sync[0], byte_bsc_en_vtc};
        end
    end

    // BITSLICE_CONTROL RIU registers used by read-gate calibration.
    localparam [5:0] RIU_ADDR_NIBBLE_CTRL0 = 6'h00;
    // UG571 Table 2-39 defines NIBBLE_CTRL0[9] as the read-only GT_STATUS
    // bit; reserved bits [15:12] and [7] are also not part of the writable
    // control image.  A live gate status change must therefore not make a
    // successfully restored register look like a failed RIU write.
    localparam [15:0] RIU_NIBBLE_CTRL0_VERIFY_MASK = 16'h0d7f;
    localparam [5:0] RIU_ADDR_BS_CTRL       = 6'h05;
    localparam [5:0] RIU_ADDR_WL_DLY_RNK0  = 6'h2c;
    localparam [5:0] RIU_ADDR_RL_DLY_RNK0  = 6'h30;
    localparam [15:0] RIU_GATE_CLEAR       = 16'h0130;
    localparam [15:0] RIU_GATE_RUN         = 16'h0030;
    // Physical positions populated by ddr4_phy_native_byte in either nibble:
    // bit 0 is DM/DQS, bits 2..5 are DQ, and bit 7 is the tristate slice.
    // Reset the complete byte atomically so every RX FIFO starts application
    // traffic from the same modulo-8 boundary.
    localparam [15:0] RIU_BS_RESET_MASK    = 16'h00bd;

    // Tristate
    (* mark_debug = "true" *) wire [3:0] tbyte_dq;
    (* mark_debug = "true" *) wire [3:0] tbyte_dqs;

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
    assign dfi_read_command =
        (~i_dfi_cs_n) & i_dfi_act_n & i_dfi_ras_n &
        (~i_dfi_cas_n) & i_dfi_we_n;
    assign dfi_read_expected = |i_dfi_rddata_en;

    function [5:0] native_auto_cl;
        input integer clock_period_ps;
        begin
            // Keep the PHY's nominal read latency identical to the CL that
            // ddr4_controller programs into MR0.  The DDR4-1333 bin is easy
            // to miss because the original native PHY targeted DDR4-1600
            // and faster; treating tCK=1.600 ns as CL=12 opens PHY_RDEN three
            // memory clocks after the DRAM's CL=9 return.
            native_auto_cl = (clock_period_ps >= 1500) ? 6'd9  :
                             (clock_period_ps >= 1250) ? 6'd12 :
                             (clock_period_ps >= 1071) ? 6'd14 :
                             (clock_period_ps >=  937) ? 6'd16 : 6'd18;
        end
    endfunction

    localparam integer RD_GATE_PIPE_BITS = 64;
    localparam [5:0] NATIVE_CL_NCK = native_auto_cl(DDR4_CLK_PERIOD);
    // JESD79-4 MPR exit requires the final training READ to precede the MRS
    // that disables MPR by RL + 4 + tMPRR + PL - 1 clocks.  For the serial
    // MPR format used here, tMPRR=1 and PL=0, so the minimum is RL+4 nCK.
    // Count at the 1:4 DFI clock and round upward.
    localparam integer NATIVE_MPR_EXIT_GAP_CTRL =
        (NATIVE_CL_NCK + 4 + SERDES_RATIO - 1) / SERDES_RATIO;
    // The command-to-DQS relationship includes primitive, package, and board
    // delay, so a simulation-derived fixed mCL is not a hardware-safe gate
    // point. Start at the nominal JEDEC read latency, search the bounded
    // adjacent PHY_RDEN cycles, and use RL_DLY_RNK0[12:9] for the fractional
    // native phase within each cycle. This combines the command-timed mask
    // with the XiPHY mechanism documented by UG571.
    //
    // Directed XSim tests may force one mCL for isolated timing checks.
    // Production starts at NATIVE_CL_NCK and searches seed, seed-1, seed-2,
    // seed+1, seed+2. This contains both historical VCO/VCO_2X offsets while
    // keeping every mask representable at the registered DFI boundary
    // (mCL >= 7).
`ifdef SIM_NATIVE_GATE_MCL_7
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd7};
`elsif SIM_NATIVE_GATE_MCL_8
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd8};
`elsif SIM_NATIVE_GATE_MCL_9
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd9};
`elsif SIM_NATIVE_GATE_MCL_10
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd10};
`elsif SIM_NATIVE_GATE_MCL_11
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd11};
`elsif SIM_NATIVE_GATE_MCL_12
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd12};
`elsif SIM_NATIVE_GATE_MCL_13
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd13};
`elsif SIM_NATIVE_GATE_MCL_14
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd14};
`elsif SIM_NATIVE_GATE_MCL_15
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd15};
`elsif SIM_NATIVE_GATE_MCL_16
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd16};
`elsif SIM_NATIVE_GATE_MCL_17
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd17};
`elsif SIM_NATIVE_GATE_MCL_18
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd18};
`elsif SIM_NATIVE_GATE_MCL_19
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd19};
`elsif SIM_NATIVE_GATE_MCL_20
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd20};
`elsif SIM_NATIVE_GATE_MCL_21
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd21};
`elsif SIM_NATIVE_GATE_MCL_22
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b1, 6'd22};
`else
    localparam [6:0] NATIVE_GATE_MCL_CONFIG = {1'b0, NATIVE_CL_NCK};
`endif
    localparam [5:0] NATIVE_GATE_MCL_INITIAL =
        NATIVE_GATE_MCL_CONFIG[5:0];
    localparam integer NATIVE_GATE_MCL_CANDIDATES =
        NATIVE_GATE_MCL_CONFIG[6] ? 1 : 5;
    // RL_DLY_CRSE is a four-bit native gate delay, not a modulo-four phase
    // selector.  UG571 defines the complete 0..15 range and Xilinx's own DQS
    // tracking logic treats 0 and 15 as the underflow/overflow boundaries.
    // Search all hardware-representable values: substituting a one-tCK shift
    // of PHY_RDEN for coarse values 4..15 is not equivalent inside the native
    // gate state machine and can leave otherwise healthy byte lanes without
    // any GT_STATUS window.
    localparam integer NATIVE_GATE_COARSE_CANDIDATES = 16;

    function [5:0] native_adjacent_mcl_candidate;
        input [5:0] base_mcl;
        input [2:0] candidate_index;
        begin
            case (candidate_index)
                3'd0: native_adjacent_mcl_candidate = base_mcl;
                3'd1: native_adjacent_mcl_candidate =
                    (base_mcl > 6'd7) ? base_mcl - 6'd1 : 6'd7;
                3'd2: native_adjacent_mcl_candidate =
                    (base_mcl > 6'd8) ? base_mcl - 6'd2 : 6'd7;
                3'd3: native_adjacent_mcl_candidate = base_mcl + 6'd1;
                default: native_adjacent_mcl_candidate = base_mcl + 6'd2;
            endcase
        end
    endfunction

    function [5:0] native_gate_mcl_candidate;
        input [2:0] candidate_index;
        begin
            native_gate_mcl_candidate = native_adjacent_mcl_candidate(
                NATIVE_GATE_MCL_INITIAL, candidate_index);
        end
    endfunction

    // Eye recovery gives the immediately-later command cycle first priority.
    // Gate capture is periodic and the hardware captures showed the valid MPR
    // copy one cycle later than the first GT_STATUS solution.  All adjacent
    // candidates are still covered; this ordering only avoids spending a full
    // eye sweep on less likely points before trying the observed solution.
    function [5:0] native_eye_mcl_candidate;
        input [5:0] base_mcl;
        input [2:0] candidate_index;
        begin
            case (candidate_index)
                3'd0: native_eye_mcl_candidate = base_mcl;
                3'd1: native_eye_mcl_candidate = base_mcl + 6'd1;
                3'd2: native_eye_mcl_candidate =
                    (base_mcl > 6'd7) ? base_mcl - 6'd1 : 6'd7;
                3'd3: native_eye_mcl_candidate = base_mcl + 6'd2;
                default: native_eye_mcl_candidate =
                    (base_mcl > 6'd8) ? base_mcl - 6'd2 : 6'd7;
            endcase
        end
    endfunction

    // Eye training can expose a native FIFO boundary ambiguity that the
    // periodic GT_STATUS gate sweep alone cannot resolve.  One RL_DLY_CRSE
    // code is half a PLL_CLK period (one half-UI in this topology).  Generate
    // the two immediately adjacent physical phases around the measured gate;
    // actual MPR eye width decides whether either alternative is usable.
    function [5:0] native_eye_gate_retry_mcl;
        input [5:0] raw_mcl;
        input [3:0] raw_coarse;
        input [1:0] retry_index;
        begin
            case (retry_index)
                2'd1: native_eye_gate_retry_mcl =
                    (raw_coarse == 4'd0 && raw_mcl > 6'd7) ?
                    raw_mcl - 1'b1 : raw_mcl;
                2'd2: native_eye_gate_retry_mcl =
                    (raw_coarse == 4'd15 && raw_mcl < 6'd63) ?
                    raw_mcl + 1'b1 : raw_mcl;
                default: native_eye_gate_retry_mcl = raw_mcl;
            endcase
        end
    endfunction

    function [3:0] native_eye_gate_retry_coarse;
        input [5:0] raw_mcl;
        input [3:0] raw_coarse;
        input [1:0] retry_index;
        begin
            case (retry_index)
                2'd1: native_eye_gate_retry_coarse =
                    (raw_coarse == 4'd0) ?
                    ((raw_mcl > 6'd7) ? 4'd3 : 4'd0) :
                    raw_coarse - 1'b1;
                2'd2: native_eye_gate_retry_coarse =
                    (raw_coarse == 4'd15) ?
                    ((raw_mcl < 6'd63) ? 4'd12 : 4'd15) :
                    raw_coarse + 1'b1;
                default: native_eye_gate_retry_coarse = raw_coarse;
            endcase
        end
    endfunction

    // Directed-simulation hook: start eye training one command cycle before
    // the gate result, proving that the bounded upper-nibble recovery path can
    // return to the real gate-selected point.  It has no synthesized effect
    // unless the diagnostic define is explicitly supplied to the simulator.
    function [5:0] native_eye_initial_upper_mcl;
        input [5:0] gate_mcl;
        begin
`ifdef SIM_NATIVE_EYE_DIAG_UPPER_MINUS1
            native_eye_initial_upper_mcl =
                (gate_mcl > 6'd7) ? gate_mcl - 1'b1 : 6'd7;
`else
            native_eye_initial_upper_mcl = gate_mcl;
`endif
        end
    endfunction

    // Gate training followed by exact, unrotated MPR verification proves the
    // command mask and RL_DLY setting as one pair.  Keep that proven pair for
    // application traffic.  Moving either side after verification changes the
    // native 1:8 FIFO word boundary and can splice adjacent BL8 bursts even
    // though the analog DQ eye itself remains open.
    function [5:0] native_app_read_mcl;
        input [5:0] trained_mcl;
        begin
`ifdef SIM_NATIVE_DIAG_APP_MCL_MINUS2
            native_app_read_mcl =
                (trained_mcl > 6'd8) ? trained_mcl - 2'd2 : 6'd7;
`elsif SIM_NATIVE_DIAG_APP_MCL_MINUS1
            native_app_read_mcl =
                (trained_mcl > 6'd7) ? trained_mcl - 1'b1 : 6'd7;
`elsif SIM_NATIVE_DIAG_APP_MCL_NOMINAL
            native_app_read_mcl = trained_mcl;
`elsif SIM_NATIVE_DIAG_FINALIZE_GATE_PLUS4
            // Directed boundary proof: move only the physical RX gate.  Keep
            // the command mask at the MPR-verified cycle so this experiment
            // measures the RL_DLY displacement without adding another tCK.
            native_app_read_mcl = trained_mcl;
`elsif SIM_NATIVE_DIAG_FINALIZE_GATE_PLUS2
            native_app_read_mcl = trained_mcl;
`elsif SIM_NATIVE_DIAG_WL_GATE_PLUS2
            // Directed boundary proof: keep the command mask at the exact
            // MPR-verified cycle while the post-WL RIU sequence advances the
            // DQS gate by two one-UI RL_DLY_CRSE steps.  Changing both the
            // command mask and RL_DLY would move four additional UIs and
            // would not isolate the effect under test.
            native_app_read_mcl = trained_mcl;
`elsif SIM_NATIVE_DIAG_WL_GATE_PLUS4
            native_app_read_mcl =
                (trained_mcl > 6'd7) ? trained_mcl - 1'b1 : 6'd7;
`else
            // Exact MPR verification includes the configured DDR4 preamble
            // and proves the command mask together with RL_DLY.  Reuse that
            // pair unchanged for application reads; adding a controller cycle
            // opens the native gate on the following BL8 boundary.
            native_app_read_mcl = trained_mcl;
`endif
        end
    endfunction

    (* mark_debug = "true" *) reg [8:0] gate_sweep_tap;
    (* mark_debug = "true" *) reg [5:0] gate_sweep_mcl;
    (* mark_debug = "true" *) reg [3:0] gate_sweep_coarse;
    (* mark_debug = "true" *) reg [2:0] gate_mcl_index;
    reg gate_advance_mcl;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] gate_lane_resolved;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] gate_lane_resolved_low;
    reg [5:0] gate_trained_mcl [0:BYTE_LANES-1];
    reg [5:0] gate_trained_mcl_low [0:BYTE_LANES-1];
    reg [3:0] gate_trained_coarse [0:BYTE_LANES-1];
    reg [3:0] gate_trained_coarse_low [0:BYTE_LANES-1];
    // Retained only by directed finalizer diagnostics below. Production
    // calibration never enables the former rank-wide normalization path.
    reg [5:0] app_read_mcl;
    reg [5:0] app_read_mcl_upper [0:BYTE_LANES-1];
    reg [5:0] app_read_mcl_lower [0:BYTE_LANES-1];
    reg [3:0] app_read_coarse_target [0:BYTE_LANES-1];
    reg gate_mcl_consensus_valid;
    reg [5:0] gate_mcl_consensus_lower;
    reg [5:0] gate_mcl_consensus_upper;
    localparam [1:0] GATE_CONS_LANE  = 2'd0,
                     GATE_CONS_ACCUM = 2'd1,
                     GATE_CONS_TARGET = 2'd2,
                     GATE_CONS_COMMIT = 2'd3;
    reg gate_consensus_active;
    reg [1:0] gate_consensus_phase;
    reg [$clog2(BYTE_LANES > 1 ? BYTE_LANES : 2)-1:0]
        gate_consensus_lane;
    reg [5:0] gate_consensus_lower_work;
    reg [5:0] gate_consensus_upper_work;
    reg [5:0] gate_consensus_lane_lower_q;
    reg [5:0] gate_consensus_lane_upper_q;
    reg signed [8:0] gate_consensus_target_q;
    reg [3:0] gate_consensus_first_target;
    reg [5:0] eye_last_read_age;
    // One-shot the physical MPR gate during each eye candidate.  The
    // controller is allowed to keep issuing calibration READs while the PHY
    // observes a candidate; admitting adjacent DQS bursts can make an
    // incorrectly framed native FIFO word look exact by borrowing the final
    // UIs from the following burst.  Admit only the first command after
    // entering SWEEP/VERIFY so every accepted match proves that one isolated
    // BL8 read produced all eight UIs of the native FIFO word.
    (* mark_debug = "true" *) reg eye_candidate_read_armed;
    reg [3:0] eye_candidate_state_q;
    // During one fine sweep every byte uses the same candidate mCL.  Once a
    // byte has a complete GT_STATUS/FIFO window, its coarse and fine results
    // are retained while unresolved bytes continue with adjacent candidates.
    wire [5:0] active_read_mcl = i_dfi_rdlvl_gate_en ?
        gate_sweep_mcl : gate_trained_mcl[0];
    (* mark_debug = "true" *) reg [RD_GATE_PIPE_BITS-1:0]
        read_gate_pipe [0:BYTE_LANES-1];
    (* mark_debug = "true" *) reg [RD_GATE_PIPE_BITS-1:0]
        read_gate_pipe_low [0:BYTE_LANES-1];
    reg [RD_GATE_PIPE_BITS-1:0]
        read_gate_pipe_next [0:BYTE_LANES-1];
    reg [RD_GATE_PIPE_BITS-1:0]
        read_gate_pipe_low_next [0:BYTE_LANES-1];
    // Gate calibration must associate exactly one command-timed mask with
    // each RL_DLY candidate.  MPR data is periodic, so allowing masks from
    // adjacent candidates to reach the bit slices can combine two clipped
    // half-bursts into one apparently valid FIFO word.
    (* mark_debug = "true" *) reg gate_capture_enable;
    integer read_gate_lane_comb, read_gate_lane_seq;
    integer read_gate_phase, read_gate_ui;
    integer read_gate_start;

    always @* begin
        for (read_gate_lane_comb = 0; read_gate_lane_comb < BYTE_LANES;
             read_gate_lane_comb = read_gate_lane_comb + 1) begin
            read_gate_pipe_next[read_gate_lane_comb] =
                read_gate_pipe[read_gate_lane_comb] >> SERDES_RATIO;
            read_gate_pipe_low_next[read_gate_lane_comb] =
                read_gate_pipe_low[read_gate_lane_comb] >> SERDES_RATIO;
            for (read_gate_phase = 0; read_gate_phase < SERDES_RATIO;
                 read_gate_phase = read_gate_phase + 1) begin
                if (dfi_read_command[read_gate_phase] &&
                    (!i_dfi_rdlvl_gate_en || gate_capture_enable)
                    && (!i_dfi_rdlvl_en || eye_candidate_read_armed)
                    ) begin
                    // The native CA serializer presents the command at the
                    // pins one 1:4 word after this DFI-boundary observation.
                    // Subtract that four-tCK pipeline from MIG's cal_rd_en
                    // placement.  Gate/eye calibration use the preamble-search
                    // timing while RL_DLY is trained on the first DQS edge.
                    //
                    // Gate training broadcasts one candidate; normal reads
                    // use the independently trained point for each byte.
                    read_gate_start =
                        (i_dfi_rdlvl_gate_en ? gate_sweep_mcl :
`ifdef SIM_NATIVE_DIAG_APP_MCL_PLUS1
                         (i_dfi_rdlvl_en ?
                          gate_trained_mcl[read_gate_lane_comb] :
                          (gate_trained_mcl[read_gate_lane_comb] + 6'd1))) -
`elsif SIM_NATIVE_DIAG_APP_MCL_PLUS2
                         (i_dfi_rdlvl_en ?
                          gate_trained_mcl[read_gate_lane_comb] :
                          (gate_trained_mcl[read_gate_lane_comb] + 6'd2))) -
`else
                          (i_dfi_rdlvl_en ?
                            (eye_boundary_retime_done ?
                             native_app_read_mcl(
                                 gate_trained_mcl[read_gate_lane_comb]) :
                             gate_trained_mcl[read_gate_lane_comb]) :
                           native_app_read_mcl(
                               gate_trained_mcl[
                                   read_gate_lane_comb]))) -
`endif
                        7 +
                        ((read_gate_phase >= 2) ? 2 : 0)
`ifdef SIM_NATIVE_DIAG_APP_RDEN_MINUS2UI
                        // Directed proof of the DFI-slot conversion.  MIG's
                        // four-tCK gate mask is correct, but the application
                        // command occupies the upper half of the 1:4 DFI word.
                        // Advance only that application mask by two tCK slots
                        // (one DDR UI pair); calibration keeps its trained
                        // command/RL_DLY relationship unchanged.
                        - ((i_dfi_rdlvl_gate_en || i_dfi_rdlvl_en) ? 0 : 2)
`endif
`ifdef SIM_NATIVE_DIAG_APP_RDEN_PLUS1TCK
                        // MIG rs2mask places an upper-half READ slot one
                        // pipeline position later than the generic >=2 phase
                        // correction above.  Keep this A/B application-only so
                        // the trained calibration point remains unchanged.
                        + ((i_dfi_rdlvl_gate_en || i_dfi_rdlvl_en) ? 0 : 1)
`endif
                        ;
                    // A BL8 return contains exactly four tCK (eight
                    // transfers).  Guard the variable index so a directed
                    // simulation override cannot create an invalid select.
                    for (read_gate_ui = 0; read_gate_ui < SERDES_RATIO + 1;
                         read_gate_ui = read_gate_ui + 1)
                        if ((read_gate_ui < SERDES_RATIO ||
`ifdef SIM_NATIVE_DIAG_RDEN_5TCK
                             !(i_dfi_rdlvl_gate_en || i_dfi_rdlvl_en)) &&
`else
                             1'b0) &&
`endif
                            ((read_gate_start + read_gate_ui) >= 0) &&
                            ((read_gate_start + read_gate_ui) <
                             RD_GATE_PIPE_BITS))
                            read_gate_pipe_next[read_gate_lane_comb]
                                [read_gate_start + read_gate_ui] = 1'b1;
                    read_gate_start =
                        (i_dfi_rdlvl_gate_en ? gate_sweep_mcl :
`ifdef SIM_NATIVE_DIAG_APP_MCL_PLUS1
                         (i_dfi_rdlvl_en ?
                          gate_trained_mcl_low[read_gate_lane_comb] :
                          (gate_trained_mcl_low[read_gate_lane_comb] + 6'd1))) -
`elsif SIM_NATIVE_DIAG_APP_MCL_PLUS2
                         (i_dfi_rdlvl_en ?
                          gate_trained_mcl_low[read_gate_lane_comb] :
                          (gate_trained_mcl_low[read_gate_lane_comb] + 6'd2))) -
`else
                          (i_dfi_rdlvl_en ?
                            (eye_boundary_retime_done ?
                             native_app_read_mcl(
                                 gate_trained_mcl_low[read_gate_lane_comb]) :
                             gate_trained_mcl_low[read_gate_lane_comb]) :
                           native_app_read_mcl(
                               gate_trained_mcl_low[
                                   read_gate_lane_comb]))) -
`endif
                        7 +
                        ((read_gate_phase >= 2) ? 2 : 0)
`ifdef SIM_NATIVE_DIAG_APP_RDEN_MINUS2UI
                        - ((i_dfi_rdlvl_gate_en || i_dfi_rdlvl_en) ? 0 : 2)
`endif
`ifdef SIM_NATIVE_DIAG_APP_RDEN_PLUS1TCK
                        + ((i_dfi_rdlvl_gate_en || i_dfi_rdlvl_en) ? 0 : 1)
`endif
                        ;
                    for (read_gate_ui = 0; read_gate_ui < SERDES_RATIO + 1;
                         read_gate_ui = read_gate_ui + 1)
                        if ((read_gate_ui < SERDES_RATIO ||
`ifdef SIM_NATIVE_DIAG_RDEN_5TCK
                             !(i_dfi_rdlvl_gate_en || i_dfi_rdlvl_en)) &&
`else
                             1'b0) &&
`endif
                            ((read_gate_start + read_gate_ui) >= 0) &&
                            ((read_gate_start + read_gate_ui) <
                             RD_GATE_PIPE_BITS))
                            read_gate_pipe_low_next[read_gate_lane_comb]
                                [read_gate_start + read_gate_ui] = 1'b1;
                end
            end
        end
    end

    always @(posedge i_controller_clk) begin
        for (read_gate_lane_seq = 0; read_gate_lane_seq < BYTE_LANES;
             read_gate_lane_seq = read_gate_lane_seq + 1) begin
            if (sync_rst) begin
                read_gate_pipe[read_gate_lane_seq] <=
                    {RD_GATE_PIPE_BITS{1'b0}};
                read_gate_pipe_low[read_gate_lane_seq] <=
                    {RD_GATE_PIPE_BITS{1'b0}};
            end else begin
                read_gate_pipe[read_gate_lane_seq] <=
                    read_gate_pipe_next[read_gate_lane_seq];
                read_gate_pipe_low[read_gate_lane_seq] <=
                    read_gate_pipe_low_next[read_gate_lane_seq];
            end
        end
    end

    // PHY_RDEN is a timing mask, not a permanent receiver-enable.  MIG drives
    // one four-tCK mask for each BL8 READ so BITSLICE_CONTROL rejects the DQS
    // preamble and the FPGA's own write strobes.  The trained mCL controls the
    // coarse placement and the RIU gate delay supplies the per-byte fine phase.
    wire [BYTE_LANES*SERDES_RATIO-1:0] phy_rden_mask;
    wire [BYTE_LANES*SERDES_RATIO-1:0] phy_rden_mask_low;
    generate
        genvar rden_lane;
        for (rden_lane = 0; rden_lane < BYTE_LANES;
             rden_lane = rden_lane + 1) begin : gen_phy_rden_mask
`ifdef SIM_NATIVE_GATE_BYPASS
            // The acceleration bypass skips the RIU DQS-gate sweep, so keep
            // the MPR receiver open during the following eye scan.
            assign phy_rden_mask[rden_lane*SERDES_RATIO +: SERDES_RATIO] =
                (i_dfi_rdlvl_en || i_dfi_wrlvl_en) ?
                {SERDES_RATIO{1'b1}} :
                read_gate_pipe[rden_lane][SERDES_RATIO-1:0];
            assign phy_rden_mask_low[rden_lane*SERDES_RATIO +: SERDES_RATIO] =
                (i_dfi_rdlvl_en || i_dfi_wrlvl_en) ?
                {SERDES_RATIO{1'b1}} :
                read_gate_pipe_low[rden_lane][SERDES_RATIO-1:0];
`else
            // Write leveling supplies its own bounded capture burst. All
            // other modes use this byte's trained command-timed mask.
            assign phy_rden_mask[rden_lane*SERDES_RATIO +: SERDES_RATIO] =
                i_dfi_wrlvl_en ? {SERDES_RATIO{1'b1}} :
                read_gate_pipe[rden_lane][SERDES_RATIO-1:0];
            assign phy_rden_mask_low[rden_lane*SERDES_RATIO +: SERDES_RATIO] =
                i_dfi_wrlvl_en ? {SERDES_RATIO{1'b1}} :
                read_gate_pipe_low[rden_lane][SERDES_RATIO-1:0];
`endif
        end
    endgenerate

    // Track the spacing from the most recent physical training READ.  The
    // final boundary retime can finish at any point in the controller's
    // periodic READ pump; using command age avoids replying immediately after
    // a new READ and therefore preserves the JEDEC MPR-to-MRS interval.
    always @(posedge i_controller_clk) begin
        if (sync_rst || !i_dfi_rdlvl_en)
            eye_last_read_age <= 6'h3f;
        else if (|dfi_read_command)
            eye_last_read_age <= 6'd0;
        else if (eye_last_read_age != 6'h3f)
            eye_last_read_age <= eye_last_read_age + 1'b1;
    end
    always @(posedge i_controller_clk) begin
        if (sync_rst || !i_dfi_rdlvl_en) begin
            eye_candidate_read_armed <= 1'b0;
            eye_candidate_state_q <= PHY_IDLE;
        end else begin
            eye_candidate_state_q <= phy_state;
            if (((phy_state == PHY_EYE_SWEEP) &&
                 (eye_candidate_state_q != PHY_EYE_SWEEP)) ||
                ((phy_state == PHY_EYE_VERIFY) &&
                 (eye_candidate_state_q != PHY_EYE_VERIFY)))
                eye_candidate_read_armed <= 1'b1;
            else if (|dfi_read_command)
                eye_candidate_read_armed <= 1'b0;
        end
    end
    // UG571 native-mode initialization gives the reset sequencer exclusive
    // ownership of PHY_RDEN after VTC_RDY.  Once initialization completes,
    // hand the primitive input to the normal command-timed read gate.  Do not
    // mask the reset value with output_enable: the startup all-high interval
    // is a primitive initialization requirement, not a memory read window.
    (* mark_debug = "true" *) wire [BYTE_LANES*SERDES_RATIO-1:0]
        phy_rden_ready =
        rst_init_complete ?
            (phy_rden_mask &
             {BYTE_LANES*SERDES_RATIO{~output_enable}}) :
            {BYTE_LANES*SERDES_RATIO{rst_phy_rden}};
    // One DQS pair owns the complete byte. The lower nibble imports the
    // upper nibble's PCLK/NCLK through EN_OTHER_PCLK/NCLK, so both controls
    // must receive the same command-timed gate. Independent mCL values split
    // the byte's DQ bits across adjacent BL8 returns.
    (* mark_debug = "true" *) wire [BYTE_LANES*SERDES_RATIO-1:0]
        phy_rden_ready_low = phy_rden_ready;

    // Current byte under calibration.  It is declared before the byte-lane
    // generate block because write leveling must pulse only that lane's DQS.
    (* mark_debug = "true" *)
    reg [$clog2(BYTE_LANES > 1 ? BYTE_LANES : 2)-1:0] train_lane;
    // -----------------------------------------------------------------
    generate
        genvar lane;
        for (lane = 0; lane < BYTE_LANES; lane = lane + 1) begin : gen_byte_lane
            localparam [2:0] BYTE_PLL_INDEX =
                BYTE_PLL_MAP[lane*3 +: 3];
`ifdef SIM_NATIVE_DIAG_WL_SELECTED_DQS
            // Write leveling is byte-lane training. Pulsing every DQS pair
            // while waiting on only one lane's feedback lets unrelated RX
            // FIFOs complete at different modulo-8 positions. Keep normal
            // write DQS broadcast behavior unchanged.
            wire [7:0] lane_dqs_pattern =
                (wl_active && (train_lane != lane)) ? 8'h00 : dqs_pattern;
            wire [3:0] lane_tbyte_dqs =
                (wl_active && (train_lane != lane)) ? 4'b0000 : tbyte_dqs;
`else
            wire [7:0] lane_dqs_pattern = dqs_pattern;
            wire [3:0] lane_tbyte_dqs = tbyte_dqs;
`endif
            ddr4_phy_native_byte #(
                .DQ_BITS(DQ_BITS),
                // The public PHY contract supplies the same 300 MHz delay
                // reference used by the component PHY.  This attribute sets
                // the calibrated TIME-mode tap scale; it is not the serial
                // CLKOUTPHY frequency.  Keep it at 300 MHz exactly as the
                // generated UltraScale DDR4 MIG does for its XiPHY slices.
                .REFCLK_FREQ(300.0),
                .SIM_DEVICE(SIM_DEVICE),
                .DQ_PIN_MAP(DQ_PIN_MAP[lane*DQ_BITS*4 +: DQ_BITS*4])
            ) u_byte (
                .i_pll_clkoutphy   (pll_clkoutphy[BYTE_PLL_INDEX]),
                .i_div_clk         (i_controller_clk),
                .i_riu_clk         (i_ref_clk),
                .i_bsc_rst         (bsc_rst),
                .i_riu_rst         (bsc_riu_rst),
                .i_bitslice_rst    (bitslice_rst),
                // RXTX_BITSLICE can observe its own transmitted DQS through
                // the IOB even while the read gate is closed.  Application
                // turnaround logic continuously drains and ignores unwanted
                // locally echoed words. RX_RST is reserved for native
                // bring-up and calibration; asserting it for an application
                // write would risk overlapping the following DRAM return.
                .i_rx_fifo_rst     (rx_fifo_reset_active),
                .o_dly_rdy         (byte_dly_rdy_raw[lane]),
                .o_vtc_rdy         (byte_vtc_rdy_raw[lane]),
                .i_bsc_en_vtc      (byte_bsc_en_vtc_sync[1]),
                .i_bitslice_en_vtc (bitslice_en_vtc),
                .i_tx_dq_data      (tx_dq_data[lane]),
                .i_tx_dqs_data     (lane_dqs_pattern),
                .i_tx_dm_data      (tx_dm_data[lane]),
                .i_tbyte_dq        (tbyte_dq),
                .i_tbyte_dqs       (lane_tbyte_dqs),
                // Application writes must not be recaptured by the native RX
                // FIFO. Write leveling is excluded because it deliberately
                // receives DRAM feedback using the transmitted DQS pulse.
                .i_rx_dq_input_disable (rx_input_disable),
                .i_rx_dqs_input_disable(rx_dqs_input_disable),
                .i_phy_rden_lower  (phy_rden_ready_low[
                                     lane*SERDES_RATIO +: SERDES_RATIO]),
                .i_phy_rden_upper  (phy_rden_ready[
                                     lane*SERDES_RATIO +: SERDES_RATIO]),
                .o_rx_dq_data      (rx_dq_data[lane]),
                .o_fifo_empty      (fifo_empty[lane]),
                .o_dqs_fifo_empty  (dqs_fifo_empty[lane]),
                .o_dqs_fifo_data   (dqs_fifo_data[lane*8 +: 8]),
                .o_dbg_nibble_ready(dbg_byte_nibble_ready[
                                     lane*4 +: 4]),
                .i_fifo_rd_en      (fifo_rd_en_drive[lane]),
                .i_riu_addr        (byte_riu_addr),
                .i_riu_wr_data     (byte_riu_wr_data),
                // native_riu_sel is a byte-lane transaction mask.  Each
                // selected byte targets only its DQS-owning upper nibble;
                // the lower nibble consumes the forwarded trained DQS clocks.
                .i_riu_wr_en       (byte_riu_wr_en &
                                    (byte_riu_lower_sel[lane] |
                                     byte_riu_upper_sel[lane])),
                .i_riu_lower_sel   (byte_riu_lower_sel[lane]),
                .i_riu_upper_sel   (byte_riu_upper_sel[lane]),
                .o_riu_rd_data     (native_riu_rd_data[lane]),
                .o_riu_valid       (native_riu_valid[lane]),
                .o_riu_gate_status_sticky(
                                     native_riu_gate_status_sticky[lane]),
                .i_rx_cntvaluein   (idelay_cntvalue),
                .i_rx_cntvaluein_per_dq(idelay_cntvalue_per_dq),
                .i_rx_per_dq_mode  (idelay_per_dq_mode),
                .i_rx_load         (idelay_load_lane[lane]),
                /* verilator lint_off PINCONNECTEMPTY */
                .o_rx_cntvalueout_dq0(),
                /* verilator lint_on PINCONNECTEMPTY */
                .o_rx_max_relative_offset(rx_max_relative_offset[lane]),
                .o_rx_align_valid  (rx_align_valid[lane]),
                .i_tx_dqs_cntvaluein(odelay_dqs_cntvalue),
                .i_tx_dqs_load     (odelay_dqs_load[lane]),
                .o_tx_dqs_cntvalueout(odelay_dqs_cntvalueout[lane]),
                .i_tx_dq_cntvaluein(tx_dq_cntvaluein),
                .i_tx_dq_load      (tx_dq_load[
                                     lane*DQ_BITS +: DQ_BITS]),
                .i_tx_dq_ce        (tx_dq_ce[
                                     lane*DQ_BITS +: DQ_BITS]),
                .i_tx_dq_inc       (tx_dq_inc),
                .i_tx_dq_en_vtc    (tx_dq_en_vtc),
                .o_tx_dq_cntvalueout(tx_dq_cntvalueout[lane]),
                .io_ddr4_dq        (io_ddr4_dq[lane*DQ_BITS +: DQ_BITS]),
                .io_ddr4_dqs_p     (io_ddr4_dqs_p[lane]),
                .io_ddr4_dqs_n     (io_ddr4_dqs_n[lane]),
                .o_ddr4_dm_n       (o_ddr4_dm_n[lane])
            );
            assign tx_dq_cntvalueout_flat[
                lane*DQ_BITS*9 +: DQ_BITS*9] = tx_dq_cntvalueout[lane];
        end
    endgenerate

    // -----------------------------------------------------------------
    // Post-failure per-bit TX-delay controller
    // -----------------------------------------------------------------
    // EN_DYN_ODLY_MODE gives the memory-mode BITSLICE_CONTROL ownership of the
    // output delays, so the individual RXTX_BITSLICE LOAD/CE pins are not the
    // active control path. Follow UG571's supported RIU sequence instead:
    // read NIBBLE_CTRL0, set DIS_DYN_MODE_TX, load the selected ODELAYxx, and
    // verify every address-tagged readback. Memory traffic is drained by the
    // prober for the complete handshake. Keep DIS_DYN_MODE_TX set after the
    // first update so BISC cannot silently overwrite the measured TX center.
    localparam [4:0] TX_DIAG_IDLE              = 5'd0,
                     TX_DIAG_WAIT_VTC_OFF      = 5'd1,
                     TX_DIAG_READ_CTRL         = 5'd2,
                     TX_DIAG_WAIT_CTRL_READ    = 5'd3,
                     TX_DIAG_WRITE_CTRL        = 5'd4,
                     TX_DIAG_WAIT_CTRL_WRITE   = 5'd5,
                     TX_DIAG_READ_DELAY        = 5'd6,
                     TX_DIAG_WAIT_DELAY_READ   = 5'd7,
                     TX_DIAG_WRITE_DELAY       = 5'd8,
                     TX_DIAG_WAIT_DELAY_WRITE  = 5'd9,
                     TX_DIAG_RESTORE_CTRL      = 5'd10,
                     TX_DIAG_WAIT_CTRL_RESTORE = 5'd11,
                     TX_DIAG_WAIT_POST         = 5'd12,
                     TX_DIAG_WAIT_VTC_ON       = 5'd13,
                     TX_DIAG_WAIT_REQ_LOW      = 5'd14;
    // RIU_VALID is an availability indication, not a fixed-latency response.
    // UG571 permits BISC to hold it Low while its own access is in progress;
    // even though normal output-delay writes complete in two RIU clocks, the
    // former 63-DIV_CLK limit could spuriously abort during arbitration. Keep
    // a finite watchdog, but make it long relative to BISC maintenance.
    localparam integer TX_DIAG_RIU_WAIT_W = 20;
    localparam [TX_DIAG_RIU_WAIT_W-1:0] TX_DIAG_RIU_TIMEOUT =
        {TX_DIAG_RIU_WAIT_W{1'b1}};
    localparam [3:0] TX_DIAG_RIU_RETRY_MAX = 4'd15;
    (* mark_debug = "true" *) reg [4:0] tx_diag_state;
    (* mark_debug = "true" *) reg [7:0] tx_diag_dq_q;
    (* mark_debug = "true" *) reg [8:0] tx_diag_target_q;
    (* mark_debug = "true" *) reg [TX_DIAG_RIU_WAIT_W-1:0]
        tx_diag_wait_q;
    (* mark_debug = "true" *) reg [3:0] tx_diag_retry_q;
    (* mark_debug = "true" *) reg [5:0] tx_diag_retry_count_q;
    (* mark_debug = "true" *) reg [TX_DIAG_LANE_W-1:0]
        tx_diag_lane_q;
    (* mark_debug = "true" *) reg [3:0] tx_diag_map_q;
    (* mark_debug = "true" *) reg [15:0] tx_diag_ctrl_q;
    (* mark_debug = "true" *) reg tx_diag_ctrl_valid_q;
    (* mark_debug = "true" *) reg [8:0] tx_diag_current_tap_q;
    reg tx_diag_previous_valid_q;
    (* mark_debug = "true" *) reg [15:0] tx_diag_riu_readback_q;
    // The byte wrapper captures RIU write payloads into a bundled-data CDC
    // holding register on this clock. Present address/data for one complete
    // controller cycle before pulsing write-enable so that wrapper never
    // captures the payload from the preceding read or write transaction.
    (* mark_debug = "true" *) reg tx_diag_write_armed_q;
    (* mark_debug = "true" *) wire tx_diag_ce_any = |tx_dq_ce;

    wire tx_diag_input_valid = (i_tx_diag_dq < TOTAL_DQ);
    wire tx_diag_riu_selected_valid =
        native_riu_valid[tx_diag_lane_q];
    wire [15:0] tx_diag_riu_selected_data =
        native_riu_rd_data[tx_diag_lane_q];
    assign o_tx_diag_current_tap = tx_diag_current_tap_q;

    function [3:0] tx_diag_map_entry;
        input integer logical_dq;
        integer byte_dq;
        begin
            byte_dq = logical_dq % DQ_BITS;
            if (&DQ_PIN_MAP) begin
                tx_diag_map_entry[3] = (byte_dq >= 4);
                tx_diag_map_entry[2:0] = (byte_dq % 4) + 2;
            end else begin
                tx_diag_map_entry = DQ_PIN_MAP[logical_dq*4 +: 4];
            end
        end
    endfunction

    always @(posedge i_controller_clk) begin
        if (sync_rst) begin
            tx_diag_state            <= TX_DIAG_IDLE;
            tx_diag_dq_q             <= 8'd0;
            tx_diag_target_q         <= 9'd0;
            tx_diag_wait_q           <= 6'd0;
            tx_diag_retry_q          <= 4'd0;
            tx_diag_retry_count_q    <= 6'd0;
            tx_diag_lane_q           <= {TX_DIAG_LANE_W{1'b0}};
            tx_diag_map_q            <= 4'd0;
            tx_diag_ctrl_q           <= 16'd0;
            tx_diag_ctrl_valid_q     <= 1'b0;
            tx_diag_current_tap_q    <= 9'd0;
            tx_diag_previous_valid_q <= 1'b0;
            tx_diag_riu_readback_q   <= 16'd0;
            tx_diag_write_armed_q    <= 1'b0;
            tx_diag_riu_override     <= 1'b0;
            tx_diag_riu_addr         <= 6'd0;
            tx_diag_riu_wr_data      <= 16'd0;
            tx_diag_riu_wr_en        <= 1'b0;
            tx_diag_riu_lower_sel    <= {BYTE_LANES{1'b0}};
            tx_diag_riu_upper_sel    <= {BYTE_LANES{1'b0}};
            tx_dq_cntvaluein         <= 9'd0;
            tx_dq_load               <= {TOTAL_DQ{1'b0}};
            tx_dq_ce                 <= {TOTAL_DQ{1'b0}};
            tx_dq_inc                <= 1'b0;
            tx_dq_en_vtc             <= 1'b1;
            o_tx_diag_ack            <= 1'b0;
            o_tx_diag_error          <= 1'b0;
            o_tx_diag_previous_tap   <= 9'd0;
        end else begin
            // RIU write-enable, obsolete direct-delay controls, and ACK are
            // one-controller-cycle pulses unless a state asserts them below.
            tx_diag_riu_wr_en <= 1'b0;
            tx_dq_load    <= {TOTAL_DQ{1'b0}};
            tx_dq_ce      <= {TOTAL_DQ{1'b0}};
            tx_dq_inc     <= 1'b0;
            tx_dq_en_vtc  <= 1'b1;
            o_tx_diag_ack <= 1'b0;

            // Write states override this after first presenting their payload.
            // All other states disarm the next transaction.
            if ((tx_diag_state != TX_DIAG_WRITE_CTRL) &&
                (tx_diag_state != TX_DIAG_WRITE_DELAY) &&
                (tx_diag_state != TX_DIAG_RESTORE_CTRL))
                tx_diag_write_armed_q <= 1'b0;

            case (tx_diag_state)
                TX_DIAG_IDLE: begin
                    tx_diag_riu_override <= 1'b0;
                    tx_diag_riu_lower_sel <= {BYTE_LANES{1'b0}};
                    tx_diag_riu_upper_sel <= {BYTE_LANES{1'b0}};
                    if (i_tx_diag_req) begin
                        o_tx_diag_error <= 1'b0;
                        if (!rst_init_complete || !tx_diag_input_valid) begin
                            o_tx_diag_error <= 1'b1;
                            o_tx_diag_ack <= 1'b1;
                            tx_diag_state <= TX_DIAG_WAIT_REQ_LOW;
                        end else begin
                            tx_diag_dq_q <= i_tx_diag_dq;
                            tx_diag_target_q <= i_tx_diag_tap;
                            tx_diag_retry_q <= 4'd0;
                            tx_diag_previous_valid_q <= 1'b0;
                            tx_diag_lane_q <= i_tx_diag_dq / DQ_BITS;
                            tx_diag_map_q <=
                                tx_diag_map_entry(i_tx_diag_dq);
                            tx_diag_ctrl_valid_q <= 1'b0;
                            tx_diag_riu_override <= 1'b1;
                            tx_diag_wait_q <= 6'd0;
                            tx_diag_state <= TX_DIAG_WAIT_VTC_OFF;
                        end
                    end
                end

                TX_DIAG_WAIT_VTC_OFF: begin
                    // Derive the byte/nibble selection and ODELAY register
                    // from the latched logical DQ after the request boundary.
                    tx_diag_riu_addr <= 6'h0b + tx_diag_map_q[2:0];
                    tx_diag_riu_lower_sel <= {BYTE_LANES{1'b0}};
                    tx_diag_riu_upper_sel <= {BYTE_LANES{1'b0}};
                    if (tx_diag_map_q[3])
                        tx_diag_riu_upper_sel[tx_diag_lane_q] <= 1'b1;
                    else
                        tx_diag_riu_lower_sel[tx_diag_lane_q] <= 1'b1;
                    if (tx_diag_wait_q < 6'd9) begin
                        tx_diag_wait_q <= tx_diag_wait_q + 1'b1;
                    end else begin
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_READ_CTRL;
                    end
                end

                TX_DIAG_READ_CTRL: begin
                    tx_diag_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                    tx_diag_wait_q <= 6'd0;
                    tx_diag_state <= TX_DIAG_WAIT_CTRL_READ;
                end

                TX_DIAG_WAIT_CTRL_READ: begin
                    if (tx_diag_riu_selected_valid) begin
                        tx_diag_ctrl_q <= tx_diag_riu_selected_data;
                        tx_diag_ctrl_valid_q <= 1'b1;
                        tx_diag_riu_readback_q <=
                            tx_diag_riu_selected_data;
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_WRITE_CTRL;
                    end else if (tx_diag_wait_q < TX_DIAG_RIU_TIMEOUT) begin
                        tx_diag_wait_q <= tx_diag_wait_q + 1'b1;
                    end else begin
                        o_tx_diag_error <= 1'b1;
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_WAIT_POST;
                    end
                end

                TX_DIAG_WRITE_CTRL: begin
                    // Bit 10 transfers output-delay ownership to RIU without
                    // disturbing the other live NIBBLE_CTRL0 gate controls.
                    tx_diag_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                    tx_diag_riu_wr_data <= tx_diag_ctrl_q | 16'h0400;
                    if (!tx_diag_write_armed_q) begin
                        tx_diag_write_armed_q <= 1'b1;
                        tx_diag_wait_q <= 6'd0;
                    end else if (tx_diag_riu_selected_valid) begin
                        tx_diag_riu_wr_en <= 1'b1;
                        tx_diag_write_armed_q <= 1'b0;
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_WAIT_CTRL_WRITE;
                    end else if (tx_diag_wait_q < TX_DIAG_RIU_TIMEOUT) begin
                        tx_diag_wait_q <= tx_diag_wait_q + 1'b1;
                    end else begin
                        o_tx_diag_error <= 1'b1;
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_WAIT_POST;
                    end
                end

                TX_DIAG_WAIT_CTRL_WRITE: begin
                    if (tx_diag_riu_selected_valid &&
                        tx_diag_riu_selected_data[10]) begin
                        tx_diag_riu_readback_q <=
                            tx_diag_riu_selected_data;
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_READ_DELAY;
                    end else if (tx_diag_wait_q < TX_DIAG_RIU_TIMEOUT) begin
                        tx_diag_wait_q <= tx_diag_wait_q + 1'b1;
                    end else begin
                        tx_diag_riu_readback_q <=
                            tx_diag_riu_selected_data;
                        o_tx_diag_error <= 1'b1;
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_RESTORE_CTRL;
                    end
                end

                TX_DIAG_READ_DELAY: begin
                    tx_diag_riu_addr <= 6'h0b + tx_diag_map_q[2:0];
                    tx_diag_wait_q <= 6'd0;
                    tx_diag_state <= TX_DIAG_WAIT_DELAY_READ;
                end

                TX_DIAG_WAIT_DELAY_READ: begin
                    if (tx_diag_riu_selected_valid) begin
                        tx_diag_riu_readback_q <=
                            tx_diag_riu_selected_data;
                        tx_diag_current_tap_q <=
                            tx_diag_riu_selected_data[8:0];
                        if (!tx_diag_previous_valid_q) begin
                            o_tx_diag_previous_tap <=
                                tx_diag_riu_selected_data[8:0];
                            tx_diag_previous_valid_q <= 1'b1;
                        end
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <=
                            (tx_diag_riu_selected_data[8:0] ==
                             tx_diag_target_q) ?
                            TX_DIAG_RESTORE_CTRL : TX_DIAG_WRITE_DELAY;
                    end else if (tx_diag_wait_q < TX_DIAG_RIU_TIMEOUT) begin
                        tx_diag_wait_q <= tx_diag_wait_q + 1'b1;
                    end else begin
                        o_tx_diag_error <= 1'b1;
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_RESTORE_CTRL;
                    end
                end

                TX_DIAG_WRITE_DELAY: begin
                    // INC=DEC=0 is the UG571 absolute-load operation.
                    tx_diag_riu_addr <= 6'h0b + tx_diag_map_q[2:0];
                    tx_diag_riu_wr_data <= {7'd0, tx_diag_target_q};
                    if (!tx_diag_write_armed_q) begin
                        tx_diag_write_armed_q <= 1'b1;
                        tx_diag_wait_q <= 6'd0;
                    end else if (tx_diag_riu_selected_valid) begin
                        tx_diag_riu_wr_en <= 1'b1;
                        tx_diag_write_armed_q <= 1'b0;
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_WAIT_DELAY_WRITE;
                    end else if (tx_diag_wait_q < TX_DIAG_RIU_TIMEOUT) begin
                        tx_diag_wait_q <= tx_diag_wait_q + 1'b1;
                    end else begin
                        o_tx_diag_error <= 1'b1;
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_RESTORE_CTRL;
                    end
                end

                TX_DIAG_WAIT_DELAY_WRITE: begin
                    if (tx_diag_riu_selected_valid &&
                        (tx_diag_riu_selected_data[8:0] ==
                         tx_diag_target_q)) begin
                        tx_diag_riu_readback_q <=
                            tx_diag_riu_selected_data;
                        tx_diag_current_tap_q <= tx_diag_target_q;
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_RESTORE_CTRL;
                    end else if (tx_diag_wait_q < TX_DIAG_RIU_TIMEOUT) begin
                        tx_diag_wait_q <= tx_diag_wait_q + 1'b1;
                    end else begin
                        tx_diag_riu_readback_q <=
                            tx_diag_riu_selected_data;
                        o_tx_diag_error <= 1'b1;
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_RESTORE_CTRL;
                    end
                end

                TX_DIAG_RESTORE_CTRL: begin
                    if (tx_diag_ctrl_valid_q) begin
                        tx_diag_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                        tx_diag_riu_wr_data <= tx_diag_ctrl_q | 16'h0400;
                        if (!tx_diag_write_armed_q) begin
                            tx_diag_write_armed_q <= 1'b1;
                            tx_diag_wait_q <= 6'd0;
                        end else if (tx_diag_riu_selected_valid) begin
                            tx_diag_riu_wr_en <= 1'b1;
                            tx_diag_write_armed_q <= 1'b0;
                            tx_diag_wait_q <= 6'd0;
                            tx_diag_state <= TX_DIAG_WAIT_CTRL_RESTORE;
                        end else if (tx_diag_wait_q <
                                     TX_DIAG_RIU_TIMEOUT) begin
                            tx_diag_wait_q <= tx_diag_wait_q + 1'b1;
                        end else begin
                            o_tx_diag_error <= 1'b1;
                            tx_diag_wait_q <= 6'd0;
                            tx_diag_state <= TX_DIAG_WAIT_POST;
                        end
                    end else begin
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_WAIT_POST;
                    end
                end

                TX_DIAG_WAIT_CTRL_RESTORE: begin
                    if (tx_diag_riu_selected_valid &&
                        (((tx_diag_riu_selected_data ^
                           (tx_diag_ctrl_q | 16'h0400)) &
                          RIU_NIBBLE_CTRL0_VERIFY_MASK) == 16'd0)) begin
                        tx_diag_riu_readback_q <=
                            tx_diag_riu_selected_data;
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_WAIT_POST;
                    end else if (tx_diag_wait_q < TX_DIAG_RIU_TIMEOUT) begin
                        tx_diag_wait_q <= tx_diag_wait_q + 1'b1;
                    end else begin
                        tx_diag_riu_readback_q <=
                            tx_diag_riu_selected_data;
                        o_tx_diag_error <= 1'b1;
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_WAIT_POST;
                    end
                end

                TX_DIAG_WAIT_POST: begin
                    if (tx_diag_wait_q < 6'd9) begin
                        tx_diag_wait_q <= tx_diag_wait_q + 1'b1;
                    end else begin
                        tx_diag_riu_override <= 1'b0;
                        tx_diag_riu_lower_sel <= {BYTE_LANES{1'b0}};
                        tx_diag_riu_upper_sel <= {BYTE_LANES{1'b0}};
                        tx_diag_wait_q <= 6'd0;
                        tx_diag_state <= TX_DIAG_WAIT_VTC_ON;
                    end
                end

                TX_DIAG_WAIT_VTC_ON: begin
                    if (tx_diag_wait_q < 6'd9) begin
                        tx_diag_wait_q <= tx_diag_wait_q + 1'b1;
                    end else if (all_vtc_rdy) begin
                        if (o_tx_diag_error &&
                            (tx_diag_retry_q <
                             TX_DIAG_RIU_RETRY_MAX)) begin
                            // Each delay write is an idempotent absolute load,
                            // and every failing path restores NIBBLE_CTRL0
                            // before reaching this state. Re-enter through the
                            // normal VTC handoff after a lost/late completion
                            // instead of reporting a recoverable transport
                            // event as a memory-training failure. Preserve the
                            // first delay readback so the caller still sees the
                            // original BISC-maintained tap after a retry.
                            o_tx_diag_error <= 1'b0;
                            tx_diag_retry_q <= tx_diag_retry_q + 1'b1;
                            if (tx_diag_retry_count_q != 6'h3f)
                                tx_diag_retry_count_q <=
                                    tx_diag_retry_count_q + 1'b1;
                            tx_diag_ctrl_valid_q <= 1'b0;
                            tx_diag_riu_override <= 1'b1;
                            tx_diag_wait_q <= 6'd0;
                            tx_diag_state <= TX_DIAG_WAIT_VTC_OFF;
                        end else begin
                            o_tx_diag_ack <= 1'b1;
                            tx_diag_wait_q <= 6'd0;
                            tx_diag_state <= TX_DIAG_WAIT_REQ_LOW;
                        end
                    end
                end

                TX_DIAG_WAIT_REQ_LOW: begin
                    if (!i_tx_diag_req)
                        tx_diag_state <= TX_DIAG_IDLE;
                end

                default: begin
                    tx_diag_riu_override <= 1'b0;
                    tx_diag_riu_lower_sel <= {BYTE_LANES{1'b0}};
                    tx_diag_riu_upper_sel <= {BYTE_LANES{1'b0}};
                    o_tx_diag_error <= 1'b1;
                    tx_diag_state <= TX_DIAG_IDLE;
                end
            endcase
        end
    end

    // -----------------------------------------------------------------
    // FIFO Read Enable
    //
    // Application data must advance as one coherent bus: issue exactly one
    // common FIFO_RD_EN after every DQ FIFO reports a word. Outside the trained
    // read window, locally transmitted DQS can still clock individual native
    // RX FIFOs even with PHY_RDEN Low (confirmed by hardware EMPTY captures).
    // Drain those ignored entries per bit with UG571's registered inverse-
    // EMPTY topology. Switch atomically to common bus pacing when the trained
    // PHY_RDEN window opens, before an application word can be completed.
    // -----------------------------------------------------------------
    (* mark_debug = "true" *) reg [4:0] rx_fifo_flush_count;
    (* mark_debug = "true" *) reg [2:0] rx_write_discard_count;
    (* mark_debug = "true" *) reg [3:0] phy_timer;
    reg [7:0] vtc_settle_counter;
    (* mark_debug = "true" *) reg [2:0] wl_capture_pulse_count;
    // One sticky bit per byte records that the current complete 8-UI
    // write-level word has received its single legal FIFO pop. Native FIFO
    // EMPTY is synchronized and can remain Low after the pointer advances;
    // using it as a level enable would overrun the read pointer.
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        wl_fifo_pop_issued;
    // Declared before the FIFO steering logic because the optional post-WL
    // drain directly selects its per-bit read-enable topology.
    localparam [3:0] WL_HANDOFF_CLEAR             = 4'd0,
                     WL_HANDOFF_WAIT_CLEAR        = 4'd1,
                     WL_HANDOFF_RELEASE           = 4'd2,
                     WL_HANDOFF_WAIT_RELEASE      = 4'd3,
                     WL_HANDOFF_DRAIN             = 4'd4,
                     WL_HANDOFF_COMPLETE          = 4'd5,
                     WL_HANDOFF_RX_RESET          = 4'd6,
                     WL_HANDOFF_RX_PRIME          = 4'd7,
                     WL_HANDOFF_WAIT_VTC_OFF      = 4'd8,
                     WL_HANDOFF_GATE_REWIND       = 4'd9,
                     WL_HANDOFF_WAIT_GATE_REWIND  = 4'd10,
                     WL_HANDOFF_GATE_COARSE       = 4'd11,
                     WL_HANDOFF_WAIT_GATE_COARSE  = 4'd12,
                     WL_HANDOFF_GATE_RESTORE      = 4'd13,
                     WL_HANDOFF_WAIT_GATE_RESTORE = 4'd14,
                     WL_HANDOFF_RX_DRAIN          = 4'd15;
    (* mark_debug = "true" *) reg [3:0] wl_handoff_phase;
    wire      rx_fifo_flush = |rx_fifo_flush_count;
    wire      rx_write_discard = output_enable |
                                     (rx_write_discard_count != 0);
    (* mark_debug = "true" *) wire [TOTAL_DQ-1:0] fifo_empty_flat;
    generate
        genvar fifo_flat_lane, fifo_flat_bit;
        for (fifo_flat_lane = 0; fifo_flat_lane < BYTE_LANES;
             fifo_flat_lane = fifo_flat_lane + 1) begin : gen_fifo_empty_lane
            for (fifo_flat_bit = 0; fifo_flat_bit < DQ_BITS;
                 fifo_flat_bit = fifo_flat_bit + 1) begin : gen_fifo_empty_bit
                assign fifo_empty_flat[fifo_flat_lane*DQ_BITS + fifo_flat_bit] =
                    fifo_empty[fifo_flat_lane][fifo_flat_bit];
            end
        end
    endgenerate
    wire calibration_request = i_dfi_rdlvl_gate_en || i_dfi_rdlvl_en ||
                               i_dfi_wrlvl_en;
    // A calibration request owns the receive path until its PHY operation has
    // returned to IDLE.  Do not tie the release to one particular training
    // enable: native PHY implementations may legally run write leveling before
    // or after read gate/eye training.  Clearing at PHY_IDLE also spans the
    // short controller/PHY handshake gap at the end of each operation without
    // leaving application accounting disabled after the final operation.
    (* mark_debug = "true" *) reg calibration_session;
    // READ commands already accepted by the controller can outlive the
    // training-enable handshake.  Keep their DFI due indications and native
    // FIFO words inside the calibration session; otherwise the final MPR
    // return can be mistaken for the first application-read token.
    (* mark_debug = "true" *) reg [4:0] calibration_read_outstanding;
    // Require a short all-idle interval after the last return has drained.
    // This covers the synchronized native-FIFO EMPTY indication without
    // encoding CL, mCL, or a board-specific flight time in the handoff.
    (* mark_debug = "true" *) reg [3:0] calibration_quiet_count;
    (* mark_debug = "true" *) reg [2:0] calibration_tail_guard_count;
    (* mark_debug = "true" *) reg [TOTAL_DQ-1:0]
        calibration_handoff_empty_seen;
    wire calibration_mode = calibration_request || calibration_session;
    // Once the final accepted training READ and its command-timed gate have
    // retired, allow the asynchronous EMPTY synchronizers to settle before
    // taking direct ownership of each FIFO reader.
    wire calibration_tail_base = calibration_session &&
        !calibration_request && (phy_state == PHY_IDLE) &&
        (calibration_read_outstanding == 0) &&
        !(|dfi_read_command) && !dfi_read_expected &&
        !(|phy_rden_ready);
    wire calibration_tail_drain = calibration_tail_base &&
        (calibration_tail_guard_count == 3'd7);
    wire [TOTAL_DQ-1:0] calibration_handoff_empty_after =
        calibration_handoff_empty_seen | fifo_empty_flat;
    wire app_account_mode = !calibration_mode && !rx_fifo_flush;
    wire app_mode = app_account_mode && !rx_write_discard;
    (* mark_debug = "true" *) reg [7:0] app_read_pending;
    // Command reservations protect RX state across direction changes, while
    // DFI rddata_en supplies the one-for-one arrival tokens.  Keeping these
    // counts separate prevents the synchronized EMPTY deassertion tail from
    // consuming reservations for later reads across refresh or idle gaps.
    (* mark_debug = "true" *) reg [7:0] app_read_due;
    // One arrival token per DFI expected-data indication.  This is separate
    // from app_read_due, which is retained until the ordered return is
    // presented to the controller.  Consuming this counter at native-FIFO
    // capture prevents synchronized EMPTY latency from spending a later READ
    // reservation on a stale Q value.
    (* mark_debug = "true" *) reg [7:0] app_capture_due_pending;
    // A write may legally start before the fabric has consumed every older
    // native-FIFO word.  Snapshot that pre-write population and continue
    // servicing exactly those returns through the turnaround.  Once drained,
    // reset the RX FIFO throughout the remaining TX ownership interval plus a
    // short DIV_CLK tail, removing any locally echoed write DQS before later
    // read data can become visible.
    (* mark_debug = "true" *) reg [7:0] app_prewrite_pending;
    (* mark_debug = "true" *) reg app_write_flush_pending;
    (* mark_debug = "true" *) reg [1:0] app_write_flush_tail;
    reg output_enable_q;
    wire app_prewrite_drain = app_write_flush_pending &&
                              (app_prewrite_pending != 0);
    // Every DQ bit owns an asynchronous native FIFO. Their synchronized EMPTY
    // flags can resolve on adjacent DIV_CLK cycles after the same DQS burst;
    // pacing the bus from one arbitrary bit tears a DFI word across lanes.
    // Advance none of the FIFOs until a complete word exists in all of them.
    wire fifo_pace_not_empty = ~(|fifo_empty_flat);
    wire [BYTE_LANES-1:0] fifo_lane_not_empty;
    wire wl_fifo_word_ready_to_pop = i_dfi_wrlvl_en &&
        (phy_state == PHY_WL_ADJUST) &&
        (wl_capture_pulse_count >= WL_CAPTURE_PULSES-1'b1);
    // Write leveling is trained one byte lane at a time.  Advancing every
    // non-empty RX FIFO here consumes stale words from lanes that have already
    // completed and destroys the common modulo-8 boundary established by read
    // training.  Keep the primitive read request local to the active lane;
    // application traffic below still advances all byte lanes coherently.
    wire [BYTE_LANES-1:0] wl_train_lane_mask =
        ({{(BYTE_LANES-1){1'b0}}, 1'b1} << train_lane);
    wire [BYTE_LANES-1:0] wl_fifo_pop_request =
        {BYTE_LANES{wl_fifo_word_ready_to_pop}} &
        fifo_lane_not_empty & wl_train_lane_mask & ~wl_fifo_pop_issued;
    wire wl_fifo_word_start = i_dfi_wrlvl_en &&
        (phy_state == PHY_WL_SAMPLE) && (phy_timer == 0) &&
        i_dfi_wrlvl_strobe && (wl_capture_pulse_count == 0);
    // The common enable is used only for application receive data.  Never
    // advance a native RX FIFO while the PHY has no reserved READ.  FIFO_EMPTY
    // is synchronized into DIV_CLK and trails pointer equality by two cycles;
    // treating a later Low indication as new idle data can therefore advance
    // the read pointer past equality and make it circulate through stale RAM.
    (* mark_debug = "true" *) reg app_fifo_rd_en_q;
    // Qualify the registered request with the synchronized all-bit EMPTY
    // status at the edge where RXTX_BITSLICE consumes FIFO_RD_EN.  EMPTY may
    // assert after the request was formed; without this final gate, the last
    // legal pop of an isolated DQS burst is followed by one pointer over-read.
    wire app_fifo_advance = app_fifo_rd_en_q && fifo_pace_not_empty;
    // RXTX_BITSLICE consumes FIFO_RD_EN on a rising FIFO_RD_CLK edge and then
    // publishes the selected word on Q.  Retain that post-pop Q value on the
    // following falling edge; this gives the primitive half a DIV_CLK cycle
    // to settle before the fabric return queue accepts the word.
    (* mark_debug = "true" *) reg app_fifo_pop_complete_q;
    // Hold FIFO_RD_EN Low after global reset until every synchronized EMPTY
    // has acknowledged the reset.  Once application operation starts, the
    // source-synchronous RX FIFO runs continuously; resetting it between read
    // groups is unsafe because a newly accepted READ can overlap the reset
    // pulse while its DQS burst is still in flight.
    (* mark_debug = "true" *) reg app_fifo_restart_wait;
    (* mark_debug = "true" *) reg [TOTAL_DQ-1:0]
        idle_fifo_rd_en_q;
    (* mark_debug = "true" *) reg app_capture_active;
    (* mark_debug = "true" *) reg [TOTAL_DQ-1:0]
        wl_handoff_empty_seen;
    // Final calibration-to-application ownership transfer.  Write leveling is
    // trained one byte lane at a time, so the per-bit asynchronous FIFOs can
    // contain different numbers of completed feedback words when the final
    // lane finishes.  Drain every bit to its own first synchronized EMPTY
    // indication before enabling the common application reader.  Once a bit
    // reports EMPTY it is never advanced again during this handoff, even if
    // the synchronized flag subsequently deasserts while DQS is stopped.
    (* mark_debug = "true" *) reg [TOTAL_DQ-1:0]
        wl_handoff_fifo_rd_en_q;
    wire [TOTAL_DQ-1:0] wl_handoff_empty_after =
        wl_handoff_empty_seen | fifo_empty_flat;
    wire wl_handoff_fifo_freeze;
    wire [TOTAL_DQ-1:0] fifo_freeze_seen = wl_handoff_empty_seen;
    // Reserve the receive FIFO entry at the READ command, not at the
    // controller's expected-data indication.  At high speed the shallow
    // native FIFO can fill before DFI rddata_en reaches the PHY.
    wire app_read_issued =
        app_account_mode & (|dfi_read_command);
    wire app_read_due_event =
        app_account_mode & dfi_read_expected;
    (* mark_debug = "true" *) wire app_fifo_service_mode =
        app_account_mode && (app_mode || app_prewrite_drain);
    wire app_receive_window = app_capture_active ||
        ((app_read_pending != 0) && (|phy_rden_ready));
    // The native FIFO is mesochronous: its synchronized EMPTY indication can
    // make a complete BL8 word visible before the controller's rddata_en due
    // token.  Keep FIFO_RD_EN on the UG571 registered-inverse-EMPTY topology,
    // retain each reserved word here, and present it only when DFI expects a
    // return.  Eight entries match the primitive FIFO depth and exceed the
    // controller's maximum in-flight read population.
    localparam integer APP_RETURN_DEPTH = 8;
    localparam integer APP_RETURN_PTR_BITS = 3;
    reg [SERDES_RATIO*DFI_DATA_WIDTH-1:0]
        app_return_fifo [0:APP_RETURN_DEPTH-1];
    reg [APP_RETURN_PTR_BITS-1:0] app_return_wr_ptr;
    reg [APP_RETURN_PTR_BITS-1:0] app_return_rd_ptr;
    (* mark_debug = "true" *) reg [APP_RETURN_PTR_BITS:0]
        app_return_count;
    // Native FIFO Q is first-word-present.  At tCK >= 1000 ps, sample the
    // current head on the falling edge for which registered FIFO_RD_EN is High;
    // the following rising edge consumes that same head.  Faster interfaces
    // use the pre-edge Q value captured on the consuming rising edge so this
    // high-fanout data path retains a full controller cycle for timing closure.
    reg [SERDES_RATIO*DFI_DATA_WIDTH-1:0] app_fifo_sample_data_q;
    (* mark_debug = "true" *) reg app_fifo_sample_valid_q;
    // Retained for ILA visibility of the preceding read request.  Q itself is
    // sampled from app_fifo_rd_en_q below and advances through the same
    // one-word elastic stage as the captured FIFO head.
    (* mark_debug = "true" *) reg app_fifo_rd_en_sample_q;
    // A production PHY_RDEN window begins at the trained BL8 data boundary, so
    // one legal native-FIFO pop produces one complete eight-UI DQ word. Hold
    // that post-pop word until the controller-clock return queue accepts it.
    reg [7:0] app_fifo_sample_word_q [0:TOTAL_DQ-1];
`ifdef SIM_NATIVE_DIAG_REFRAME_2UI
    // A native WL feedback session leaves the source-synchronous FIFO framed
    // two UI ahead of the canonical MPR word: the first FIFO head contains a
    // two-UI preamble followed by six data UI, and the following head begins
    // with the missing two UI.  Retain one head so the directed diagnostic can
    // reconstruct the canonical BL8 from two consecutive native words.
    reg [7:0] app_fifo_prev_word_q [0:TOTAL_DQ-1];
    (* mark_debug = "true" *) reg app_fifo_prev_valid_q;
    wire app_fifo_reframe_prime = app_fifo_sample_valid_q &&
                                  !app_fifo_prev_valid_q;
`endif
    // Retain EMPTY history for ILA visibility.  Functional freshness is
    // established by the reservation topology below: while no READ is
    // reserved, every native FIFO drains independently; accepting a READ stops
    // that discard path before its source-synchronous burst arrives.  Once the
    // matching DFI due token and all-DQ nonempty indication are present, Q is
    // therefore the first complete reserved BL8 word.  Do not add a second
    // Low-High-Low qualification here: it skips that first word after an idle
    // interval and makes the return boundary depend on synchronized EMPTY
    // latency rather than the registered FIFO_RD_EN contract.
    (* mark_debug = "true" *) reg app_fifo_empty_seen_q;
    (* mark_debug = "true" *) reg app_fifo_fresh_q;
`ifdef SIM_NATIVE_DIAG_FRESH_SETTLE
    reg app_fifo_fresh_ready_q;
`endif
    // Final write-level calibration consumes exactly one complete FIFO word
    // per byte and leaves every read pointer on a coherent word boundary.
    // Application ownership starts at that boundary.  Fresh application data
    // is qualified by a command reservation and the all-lanes nonempty test;
    // an additional idle Low-High-Low drain would consume the first real BL8.
    // After an idle interval, FIFO_EMPTY can deassert on the same DIV_CLK edge
    // that updates the primitive's output head.  Do not capture or advance Q
    // until the proven Low-High-Low freshness sequence has completed; the
    // extra registered cycle is what prevents the previous read group's tail
    // from becoming the first two UIs of the next BL8 return.
    wire app_fifo_accept_enabled = 1'b1;
    wire app_return_full =
        (app_return_count == APP_RETURN_DEPTH);
    wire [8:0] app_capture_reservations =
        {1'b0, app_read_pending} + {8'd0, app_read_issued};
    wire app_pending_available = (app_capture_reservations != 0);
    // A word may cross the native asynchronous FIFO before its DFI due
    // indication.  If that early word is already queued, the current due
    // event services it directly and must not also create a capture token for
    // the following READ.  An older outstanding due, however, is consumed
    // first, leaving the simultaneous new due available for another capture.
    wire app_due_services_early_return = app_read_due_event &&
        (app_read_due == 0) && (app_return_count != 0);
    wire app_capture_due_event_available = app_read_due_event &&
        !app_due_services_early_return;
    wire [8:0] app_capture_due_tokens =
        {1'b0, app_capture_due_pending} +
        {8'd0, app_capture_due_event_available};
    // FIFO_RD_EN is registered, as required by UG571.  A request currently at
    // the primitive and a post-pop word retained by the elastic capture stage
    // are two different in-flight returns during a continuous read stream.
    // Count both so the last reservation deasserts RD_EN before an extra FIFO
    // advance can occur at the burst tail.
    wire [1:0] app_fifo_pop_inflight =
        {1'b0, app_fifo_rd_en_q} + {1'b0, app_fifo_sample_valid_q};
    wire [8:0] app_capture_inflight =
        {7'd0, app_fifo_pop_inflight};
    wire app_capture_slot_available =
        app_capture_reservations > app_capture_inflight;
    wire [APP_RETURN_PTR_BITS:0] app_return_reserved_count =
        app_return_count + app_fifo_pop_inflight;
    // A READ command reserves the primitive FIFO before its source-synchronous
    // burst can arrive.  In the absence of a reservation, advance each DQ FIFO
    // independently with the registered inverse of its own EMPTY indication.
    // This removes calibration/write remnants without forcing skewed EMPTY
    // flags into one common pointer advance.  As soon as a READ is reserved,
    // all bits switch atomically to the common coherent reader.
    wire discard_fifo_enable = app_account_mode && !app_pending_available;
    // The elastic sampler asserts valid for the pre-advance FIFO head.
`ifdef SIM_NATIVE_DIAG_SIMPLE_APP_RETURN
    // Directed comparison with the archived, fully passing application path:
    // one registered all-lane pop is also the DFI return event.  This branch
    // is diagnostic until the topology is revalidated against the current
    // calibration and turnaround logic.
    wire app_fifo_pop_fire = app_fifo_service_mode && app_fifo_rd_en_q;
    wire app_pop_fire = app_fifo_pop_fire;
`else
`ifdef SIM_NATIVE_DIAG_REFRAME_2UI
    wire app_fifo_pop_fire = app_fifo_sample_valid_q &&
        app_fifo_prev_valid_q && app_pending_available &&
        !app_return_full;
`else
    wire app_fifo_pop_fire = app_fifo_sample_valid_q &&
        app_pending_available && !app_return_full;
`endif
    wire app_pop_fire = (app_return_count != 0) &&
        ((app_read_due != 0) || app_read_due_event);
`endif
    // The E3 input buffer blocks locally transmitted DQ/DQS from entering the
    // receive BITSLICE.  The application path therefore never resets the
    // source-synchronous FIFO between READ groups: reset release needs incoming
    // DQS and would consume the first new burst.
    // Account the read enable currently presented to RXTX_BITSLICE.  The next
    // registered enable is based on the remaining reservations, which avoids
    // issuing a duplicate pop while app_read_pending is being decremented.
    wire [8:0] app_pending_after_fifo_pop =
        {1'b0, app_read_pending} + {8'd0, app_read_issued} -
        {8'd0, app_fifo_pop_fire};
    wire [8:0] app_due_after_fifo_pop =
        {1'b0, app_read_due} + {8'd0, app_read_due_event} -
        {8'd0, app_pop_fire};
    // FIFO_RD_EN is registered.  A command reservation alone is not permission
    // to advance the primitive FIFO: its source-synchronous word may still be
    // assembling.  Wait for the corresponding DFI due token as well, and
    // reserve space for every request already crossing the pop pipeline.  This
    // is the boundary used by the proven previous/current 16-UI reconstruction
    // and prevents the first application word from being sampled while it
    // still contains the read preamble.
`ifdef SIM_NATIVE_DIAG_SIMPLE_APP_RETURN
    wire app_fifo_pop_request = app_fifo_service_mode && rst_phy_rden &&
        fifo_pace_not_empty &&
        (app_pending_after_fifo_pop != 0) &&
        (app_due_after_fifo_pop != 0);
`else
    wire app_fifo_pop_request = app_account_mode && rst_phy_rden &&
        app_fifo_accept_enabled && fifo_pace_not_empty &&
`ifdef SIM_NATIVE_DIAG_REFRAME_2UI
        ((app_capture_slot_available &&
          (app_capture_due_tokens > app_capture_inflight)) ||
         app_fifo_reframe_prime) &&
`else
        app_capture_slot_available &&
        (app_capture_due_tokens > app_capture_inflight) &&
`endif
        (app_return_reserved_count < APP_RETURN_DEPTH);
`endif
    // At the post-WL boundary, stop each asynchronous FIFO on its own first
    // synchronized EMPTY indication.  UG571 requires RD_EN to remain Low
    // after that indication until new receive data is present.  Continuing to
    // advance already-drained FIFOs while another nibble is still draining
    // wraps their read pointers and destroys byte-wide word coherency.
    generate
        genvar fifo_drive_lane;
        for (fifo_drive_lane = 0; fifo_drive_lane < BYTE_LANES;
             fifo_drive_lane = fifo_drive_lane + 1) begin : gen_fifo_rd_drive
`ifdef SIM_NATIVE_DIAG_WL_FINAL_RESET_PRIME
            // Final calibration handoff owns each FIFO independently.  Stop
            // a bit immediately on its first synchronized EMPTY assertion and
            // never re-enable it from the following stale Low indication.  A
            // direct EMPTY gate is explicitly supported by UG571 and avoids
            // the extra registered pop that would move the read pointer past
            // equality after the source-synchronous DQS clock has stopped.
            assign fifo_rd_en_drive[fifo_drive_lane] =
                ((phy_state == PHY_WL_DONE) &&
                 (wl_handoff_phase == WL_HANDOFF_RX_DRAIN)) ?
                    (~fifo_empty[fifo_drive_lane] &
                     ~wl_handoff_empty_seen[
                         fifo_drive_lane*DQ_BITS +: DQ_BITS]) :
                (calibration_mode ?
                    {DQ_BITS{calibration_fifo_pop_q[fifo_drive_lane] &
                             ~wl_handoff_fifo_freeze}} :
                    (app_receive_window ?
                        (!app_fifo_fresh_q ?
                            // Discard retained calibration/TX words only
                            // until the first real all-empty boundary.  Gate
                            // each bit directly with EMPTY so the registered
                            // reader cannot advance once past equality and
                            // manufacture a false Low tail from stale Q.
                            (~fifo_empty[fifo_drive_lane] &
                             {DQ_BITS{~app_fifo_empty_seen_q}}) :
                            {DQ_BITS{app_fifo_advance}}) :
                        {DQ_BITS{1'b0}}));
`else
            // Calibration retains its byte-local qualified drain.  During the
            // final WL handoff, stop each bit directly on its first synchronized
            // EMPTY assertion.  EMPTY already belongs to DIV_CLK; inserting a
            // further registered ~EMPTY stage issues one read after pointer
            // equality and can wrap skewed byte lanes through stale FIFO RAM.
            // A reserved application READ still advances every DQ FIFO
            // together after this one-way handoff has completed.
            assign fifo_rd_en_drive[fifo_drive_lane] =
                ((phy_state == PHY_WL_DONE) &&
                 (wl_handoff_phase == WL_HANDOFF_DRAIN)) ?
                (~fifo_empty[fifo_drive_lane] &
                 ~wl_handoff_empty_seen[
                     fifo_drive_lane*DQ_BITS +: DQ_BITS]) :
                calibration_tail_drain ?
                (~fifo_empty[fifo_drive_lane] &
                 ~calibration_handoff_empty_seen[
                     fifo_drive_lane*DQ_BITS +: DQ_BITS]) :
                calibration_mode ?
                {DQ_BITS{calibration_fifo_pop_q[fifo_drive_lane] &
                         ~wl_handoff_fifo_freeze}} :
`ifdef SIM_NATIVE_DIAG_SIMPLE_APP_RETURN
                {DQ_BITS{app_fifo_service_mode && app_fifo_rd_en_q}};
`else
                (discard_fifo_enable ?
                    idle_fifo_rd_en_q[
                        fifo_drive_lane*DQ_BITS +: DQ_BITS] :
                    {DQ_BITS{app_fifo_advance}});
`endif
`endif
            assign fifo_lane_not_empty[fifo_drive_lane] =
                ~(|fifo_empty[fifo_drive_lane]);
        end
    endgenerate
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        calibration_fifo_word_valid_q;
`ifdef SIM_NATIVE_DIAG_CALIB_ONE_POP
    reg [BYTE_LANES-1:0] calibration_read_pending_q;
    reg [BYTE_LANES-1:0] calibration_read_due_q;
    wire [BYTE_LANES-1:0] calibration_read_command_set =
        {BYTE_LANES{|dfi_read_command}};
    wire [BYTE_LANES-1:0] calibration_read_due_set =
        {BYTE_LANES{dfi_read_expected}};
    wire [BYTE_LANES-1:0] calibration_read_pending_armed =
        calibration_read_pending_q | calibration_read_command_set;
    wire [BYTE_LANES-1:0] calibration_read_due_armed =
        calibration_read_due_q | calibration_read_due_set;
    wire [BYTE_LANES-1:0] calibration_read_pop_request =
        {BYTE_LANES{i_dfi_rdlvl_gate_en || i_dfi_rdlvl_en}} &
        calibration_read_pending_armed & calibration_read_due_armed &
        fifo_lane_not_empty & ~calibration_fifo_pop_q;
`endif
    // Account a request when FIFO_RD_EN actually reaches the primitive.  This
    // permits one pop per controller cycle for legal back-to-back BL8 reads,
    // while still preventing an EMPTY flag's deassertion latency from causing
    // a duplicate pop after the last reserved return.
    wire app_returns_queued =
        (app_pending_after_fifo_pop != 0) || app_fifo_rd_en_q ||
        (app_return_count != 0);
    // Release the post-write cleanup as soon as a real read is queued.  A
    // fixed reset tail must never overlap that read's source-synchronous DQS.
    wire app_write_flush_release_for_read =
        app_write_flush_pending && (app_prewrite_pending == 0) &&
        !output_enable && app_returns_queued;
    assign app_write_cleanup_window = app_write_flush_pending &&
                                      (app_prewrite_pending == 0) &&
                                      !app_write_flush_release_for_read;
    integer app_queue_lane, app_queue_bit;
    integer app_queue_phase, app_queue_idx;
    always @(posedge i_controller_clk) begin
        if (sync_rst) begin
            rx_write_discard_count <= 3'd0;
            calibration_session <= 1'b0;
            calibration_read_outstanding <= 5'd0;
            calibration_quiet_count <= 4'd0;
            calibration_tail_guard_count <= 3'd0;
            calibration_handoff_empty_seen <= {TOTAL_DQ{1'b0}};
            app_read_pending <= 8'd0;
            app_read_due <= 8'd0;
            app_capture_due_pending <= 8'd0;
            app_prewrite_pending <= 8'd0;
            app_write_flush_pending <= 1'b0;
            app_write_flush_tail <= 2'd0;
            output_enable_q <= 1'b0;
            app_fifo_rd_en_q <= 1'b0;
            app_fifo_pop_complete_q <= 1'b0;
            app_fifo_restart_wait <= 1'b1;
            idle_fifo_rd_en_q <= {TOTAL_DQ{1'b0}};
            app_capture_active <= 1'b0;
            calibration_fifo_word_valid_q <= {BYTE_LANES{1'b0}};
            calibration_fifo_pop_q <= {BYTE_LANES{1'b0}};
            wl_fifo_pop_issued <= {BYTE_LANES{1'b0}};
            app_return_wr_ptr <= {APP_RETURN_PTR_BITS{1'b0}};
            app_return_rd_ptr <= {APP_RETURN_PTR_BITS{1'b0}};
            app_return_count <= {(APP_RETURN_PTR_BITS+1){1'b0}};
            app_fifo_empty_seen_q <= 1'b0;
            app_fifo_fresh_q <= 1'b0;
`ifdef SIM_NATIVE_DIAG_REFRAME_2UI
            app_fifo_prev_valid_q <= 1'b0;
            for (app_queue_idx = 0; app_queue_idx < TOTAL_DQ;
                 app_queue_idx = app_queue_idx + 1)
                app_fifo_prev_word_q[app_queue_idx] <= 8'b0;
`endif
`ifdef SIM_NATIVE_DIAG_FRESH_SETTLE
            app_fifo_fresh_ready_q <= 1'b0;
`endif
            wl_handoff_fifo_rd_en_q <= {TOTAL_DQ{1'b0}};
`ifdef SIM_NATIVE_DIAG_CALIB_ONE_POP
            calibration_read_pending_q <= {BYTE_LANES{1'b0}};
            calibration_read_due_q <= {BYTE_LANES{1'b0}};
`endif
        end else begin
            // The asynchronous native FIFO presents the word selected by this
            // cycle's read enable only after the active DIV_CLK edge.
            calibration_fifo_word_valid_q <= calibration_fifo_pop_q;
            if (!i_dfi_wrlvl_en)
                wl_fifo_pop_issued <= {BYTE_LANES{1'b0}};
            else if (wl_fifo_word_start)
                wl_fifo_pop_issued <= {BYTE_LANES{1'b0}};
`ifdef SIM_NATIVE_DIAG_MIG_WL_FIFO_RDEN
            // MIG keeps every native FIFO reader running throughout write
            // leveling.  Record that the active lane has therefore crossed
            // the read interface after the first registered enable cycle;
            // do not wait for synchronized EMPTY to request a discrete pop.
            else if (|calibration_fifo_pop_q)
                wl_fifo_pop_issued <= {BYTE_LANES{1'b1}};
`endif
            else
                wl_fifo_pop_issued <=
                    wl_fifo_pop_issued | wl_fifo_pop_request;
            if ((phy_state == PHY_WL_DONE) &&
                (wl_handoff_phase == WL_HANDOFF_DRAIN) &&
                (vtc_settle_counter == 0) &&
                (phy_timer == 0))
                wl_handoff_fifo_rd_en_q <=
                    ~fifo_empty_flat & ~wl_handoff_empty_seen;
            else
                wl_handoff_fifo_rd_en_q <= {TOTAL_DQ{1'b0}};
            // UG571's native asynchronous FIFO uses registered ~EMPTY feedback.
            // Follow each bit independently while idle; a READ reservation
            // disables this drain before the source-synchronous burst arrives.
            if (discard_fifo_enable)
                idle_fifo_rd_en_q <= ~fifo_empty_flat;
            else
                idle_fifo_rd_en_q <= {TOTAL_DQ{1'b0}};
            if (app_fifo_restart_wait && (&fifo_empty_flat))
                app_fifo_restart_wait <= 1'b0;
            // Register the common request before it reaches RXTX_BITSLICE.
            // This is the legal native-FIFO topology from UG571 and gives the
            // elastic sampler a stable pre-advance Q word.
            app_fifo_rd_en_q <= app_fifo_pop_request;
            app_fifo_pop_complete_q <=
                app_account_mode && app_fifo_advance;
            if (!app_account_mode) begin
                app_capture_active <= 1'b0;
            end else if ((app_read_pending != 0) &&
                         (|phy_rden_ready)) begin
                app_capture_active <= 1'b1;
            end else if ((app_pending_after_fifo_pop == 0) &&
                         (app_due_after_fifo_pop == 0) &&
                         !(|phy_rden_ready)) begin
                app_capture_active <= 1'b0;
            end
            output_enable_q <= output_enable;
            if (calibration_mode) begin
                case ({|dfi_read_command, dfi_read_expected})
                    2'b10: begin
                        if (calibration_read_outstanding != 5'h1f)
                            calibration_read_outstanding <=
                                calibration_read_outstanding + 1'b1;
                    end
                    2'b01: begin
                        if (calibration_read_outstanding != 0)
                            calibration_read_outstanding <=
                                calibration_read_outstanding - 1'b1;
                    end
                    default: calibration_read_outstanding <=
                        calibration_read_outstanding;
                endcase
            end else begin
                calibration_read_outstanding <= 5'd0;
            end

            if (calibration_request) begin
                calibration_session <= 1'b1;
                calibration_quiet_count <= 4'd0;
                calibration_tail_guard_count <= 3'd0;
                calibration_handoff_empty_seen <= {TOTAL_DQ{1'b0}};
            end else if (calibration_session) begin
                if (!calibration_tail_base) begin
                    calibration_quiet_count <= 4'd0;
                    calibration_tail_guard_count <= 3'd0;
                    calibration_handoff_empty_seen <=
                        {TOTAL_DQ{1'b0}};
                end else if (!calibration_tail_drain) begin
                    calibration_quiet_count <= 4'd0;
                    calibration_tail_guard_count <=
                        calibration_tail_guard_count + 1'b1;
                end else begin
                    calibration_handoff_empty_seen <=
                        calibration_handoff_empty_after;
                    if (&calibration_handoff_empty_after) begin
                        if (calibration_quiet_count == 4'd3) begin
                            calibration_session <= 1'b0;
                            calibration_quiet_count <= 4'd0;
                        end else begin
                            calibration_quiet_count <=
                                calibration_quiet_count + 1'b1;
                        end
                    end else begin
                        calibration_quiet_count <= 4'd0;
                    end
                end
            end else begin
                calibration_quiet_count <= 4'd0;
                calibration_tail_guard_count <= 3'd0;
                calibration_handoff_empty_seen <= {TOTAL_DQ{1'b0}};
            end

            // TBYTE and the serialized DQS word trail the fabric write
            // enable. Keep the calibration drain active briefly after write
            // leveling so its feedback word cannot precede application data.
            if (output_enable)
                rx_write_discard_count <= 3'd3;
            else if (rx_write_discard_count != 0)
                rx_write_discard_count <= rx_write_discard_count - 1'b1;

            // Establish one cleanup boundary per application TX burst.  The
            // snapshot uses the same next-count expression as the main read
            // reservation counter, so a pop accepted on this edge is not
            // counted twice.  New READs issued after the boundary remain in
            // app_read_pending but are intentionally excluded from the
            // pre-write population.
            if (output_enable && !output_enable_q && !calibration_mode) begin
                app_write_flush_pending <= 1'b1;
                app_prewrite_pending <= app_pending_after_fifo_pop[7:0];
                app_write_flush_tail <= 2'd3;
            end else if (app_write_flush_pending) begin
                if (app_prewrite_drain && app_fifo_pop_fire)
                    app_prewrite_pending <= app_prewrite_pending - 1'b1;

                if (output_enable) begin
                    app_write_flush_tail <= 2'd3;
                end else if (app_prewrite_pending == 0) begin
                    if (app_returns_queued) begin
                        app_write_flush_pending <= 1'b0;
                        app_write_flush_tail <= 2'd0;
                    end else if (app_write_flush_tail != 0)
                        app_write_flush_tail <= app_write_flush_tail - 1'b1;
                    else
                        app_write_flush_pending <= 1'b0;
                end
            end

            if (!app_account_mode) begin
                app_read_pending <= 8'd0;
                app_read_due <= 8'd0;
                app_capture_due_pending <= 8'd0;
            end else begin
                case ({app_read_issued, app_fifo_pop_fire})
                    2'b10: app_read_pending <= app_read_pending + 1'b1;
                    2'b01: app_read_pending <= app_read_pending - 1'b1;
                    default: app_read_pending <= app_read_pending;
                endcase
                case ({app_read_due_event, app_pop_fire})
                    2'b10: app_read_due <= app_read_due + 1'b1;
                    2'b01: app_read_due <= app_read_due - 1'b1;
                    default: app_read_due <= app_read_due;
                endcase
                // Match DFI due events and native-FIFO captures without
                // assuming which side of the mesochronous boundary arrives
                // first.  The first word after the proven EMPTY boundary may
                // be captured early; keep the balance at zero rather than
                // underflowing.  Its later due event is consumed by the
                // queued return and likewise must not mint a second token.
                case ({app_capture_due_event_available &&
                       !app_fifo_pop_fire,
                       app_fifo_pop_fire &&
                       !app_capture_due_event_available &&
                       (app_capture_due_pending != 0)})
                    2'b10: app_capture_due_pending <=
                        app_capture_due_pending + 1'b1;
                    2'b01: app_capture_due_pending <=
                        app_capture_due_pending - 1'b1;
                    default: app_capture_due_pending <=
                        app_capture_due_pending;
                endcase
            end

            if (!app_account_mode) begin
                app_return_wr_ptr <= {APP_RETURN_PTR_BITS{1'b0}};
                app_return_rd_ptr <= {APP_RETURN_PTR_BITS{1'b0}};
                app_return_count <= {(APP_RETURN_PTR_BITS+1){1'b0}};
                app_fifo_empty_seen_q <= 1'b0;
                app_fifo_fresh_q <= 1'b0;
`ifdef SIM_NATIVE_DIAG_REFRAME_2UI
                app_fifo_prev_valid_q <= 1'b0;
`endif
`ifdef SIM_NATIVE_DIAG_FRESH_SETTLE
                app_fifo_fresh_ready_q <= 1'b0;
`endif
            end else begin
`ifdef SIM_NATIVE_DIAG_REFRAME_2UI
                // FIFO_EMPTY is synchronized into DIV_CLK.  At pointer
                // equality Q can still show the just-consumed head, but UG571
                // forbids treating it as new data.  Invalidate history at the
                // coherent all-bit EMPTY boundary, then let the first sampled
                // head after EMPTY deasserts prime the adjacent-word splice.
                if (&fifo_empty_flat) begin
                    app_fifo_prev_valid_q <= 1'b0;
                end else if ((app_capture_reservations == 0) &&
                    !app_receive_window) begin
                    app_fifo_prev_valid_q <= 1'b0;
                end else if (app_fifo_sample_valid_q) begin
                    app_fifo_prev_valid_q <= 1'b1;
                    for (app_queue_idx = 0; app_queue_idx < TOTAL_DQ;
                         app_queue_idx = app_queue_idx + 1)
                        app_fifo_prev_word_q[app_queue_idx] <=
                            app_fifo_sample_word_q[app_queue_idx];
                end
`endif
`ifdef SIM_NATIVE_DIAG_FRESH_SETTLE
                if (!app_fifo_fresh_q)
                    app_fifo_fresh_ready_q <= 1'b0;
                else
                    app_fifo_fresh_ready_q <= 1'b1;
`endif
                if (app_read_issued && (app_read_pending == 0) &&
                    !app_capture_active) begin
                    app_fifo_empty_seen_q <= &fifo_empty_flat;
                    app_fifo_fresh_q <= 1'b0;
                end else if ((app_capture_reservations == 0) &&
                             !app_receive_window) begin
                    app_fifo_empty_seen_q <= 1'b0;
                    app_fifo_fresh_q <= 1'b0;
                end else if (!app_fifo_fresh_q) begin
                    if (&fifo_empty_flat)
                        app_fifo_empty_seen_q <= 1'b1;
                    if (app_fifo_empty_seen_q && fifo_pace_not_empty)
                        app_fifo_fresh_q <= 1'b1;
                end
                case ({app_fifo_pop_fire, app_pop_fire})
                    2'b10: app_return_count <= app_return_count + 1'b1;
                    2'b01: app_return_count <= app_return_count - 1'b1;
                    default: app_return_count <= app_return_count;
                endcase
                if (app_fifo_pop_fire) begin
                    app_return_wr_ptr <= app_return_wr_ptr + 1'b1;
                    // app_fifo_sample_word_q contains the reconstructed BL8
                    // word from two consecutive valid pre-pop FIFO heads.
                    for (app_queue_lane = 0;
                         app_queue_lane < BYTE_LANES;
                         app_queue_lane = app_queue_lane + 1) begin
                        for (app_queue_bit = 0;
                             app_queue_bit < DQ_BITS;
                             app_queue_bit = app_queue_bit + 1) begin
                            app_queue_idx = app_queue_lane * DQ_BITS +
                                            app_queue_bit;
                            for (app_queue_phase = 0;
                                 app_queue_phase < SERDES_RATIO;
                                 app_queue_phase = app_queue_phase + 1) begin
`ifdef SIM_NATIVE_DIAG_REFRAME_2UI
                                // Repack {previous[7:2], current[1:0]} in
                                // chronological UI order.  DFI phase zero is
                                // the low end of the destination vector.
                                if ((2*app_queue_phase) < 6)
                                    app_return_fifo[app_return_wr_ptr][
                                        app_queue_phase*DFI_DATA_WIDTH +
                                        app_queue_idx
                                    ] <= app_fifo_prev_word_q[
                                        app_queue_idx][2*app_queue_phase + 2];
                                else
                                    app_return_fifo[app_return_wr_ptr][
                                        app_queue_phase*DFI_DATA_WIDTH +
                                        app_queue_idx
                                    ] <= app_fifo_sample_word_q[
                                        app_queue_idx][2*app_queue_phase - 6];
                                if ((2*app_queue_phase + 1) < 6)
                                    app_return_fifo[app_return_wr_ptr][
                                        app_queue_phase*DFI_DATA_WIDTH +
                                        TOTAL_DQ + app_queue_idx
                                    ] <= app_fifo_prev_word_q[
                                        app_queue_idx][2*app_queue_phase + 3];
                                else
                                    app_return_fifo[app_return_wr_ptr][
                                        app_queue_phase*DFI_DATA_WIDTH +
                                        TOTAL_DQ + app_queue_idx
                                    ] <= app_fifo_sample_word_q[
                                        app_queue_idx][2*app_queue_phase - 5];
`else
                                app_return_fifo[app_return_wr_ptr][
                                    app_queue_phase*DFI_DATA_WIDTH +
                                    app_queue_idx
                                ] <= app_fifo_sample_word_q[app_queue_idx][
                                    2*app_queue_phase];
                                app_return_fifo[app_return_wr_ptr][
                                    app_queue_phase*DFI_DATA_WIDTH +
                                    TOTAL_DQ + app_queue_idx
                                ] <= app_fifo_sample_word_q[app_queue_idx][
                                    2*app_queue_phase + 1];
`endif
                            end
                        end
                    end
                end
                if (app_pop_fire)
                    app_return_rd_ptr <= app_return_rd_ptr + 1'b1;
            end

            // Each calibration byte drains as soon as all eight DQ FIFOs in
            // that byte contain a word.  This preserves the byte's BL8
            // boundary without requiring unrelated DQS lanes to become ready
            // on the same DIV_CLK cycle.  Application traffic above still
            // advances every DQ FIFO together from fifo_pace_not_empty.
`ifdef SIM_NATIVE_DIAG_CALIB_ONE_POP
            if (!(i_dfi_rdlvl_gate_en || i_dfi_rdlvl_en)) begin
                calibration_read_pending_q <= {BYTE_LANES{1'b0}};
                calibration_read_due_q <= {BYTE_LANES{1'b0}};
            end else begin
                calibration_read_pending_q <=
                    calibration_read_pending_armed &
                    ~calibration_read_pop_request;
                calibration_read_due_q <= calibration_read_due_armed &
                    ~calibration_read_pop_request;
            end
            calibration_fifo_pop_q <=
                ((phy_state == PHY_WL_DONE) &&
                 (wl_handoff_phase == WL_HANDOFF_DRAIN)) ?
                {BYTE_LANES{1'b0}} :
                (i_dfi_wrlvl_en ? fifo_lane_not_empty :
                                  calibration_read_pop_request);
`elsif SIM_NATIVE_DIAG_CONTINUOUS_FIFO_RDEN
            calibration_fifo_pop_q <=
                {BYTE_LANES{calibration_mode && rst_phy_rden &&
                            !wl_handoff_fifo_freeze}} &
                fifo_lane_not_empty;
`else
            calibration_fifo_pop_q <=
                {BYTE_LANES{calibration_mode && rst_phy_rden &&
                            !wl_handoff_fifo_freeze}} &
                fifo_lane_not_empty;
`endif
            // Write leveling produces exactly one FIFO word per four legal,
            // independently spaced DQS pulses. Pop that word once per byte;
            // do not let synchronized EMPTY latency create extra pointer
            // advances. This override applies to every calibration-reader
            // diagnostic topology above.
`ifdef SIM_NATIVE_DIAG_MIG_WL_FIFO_RDEN
            // Generated DDR4 MIG drives all XiPHY FIFO_RD_EN inputs High for
            // the complete write-leveling mode.  Calibration deliberately
            // permits reads while EMPTY is asserted; Q updates when each
            // source-synchronous word arrives and the read pointer follows
            // the writer without repeated stop/restart boundaries.
            // Relinquish the registered calibration-pop pipeline when the
            // final one-way EMPTY handoff begins.  The handoff has direct
            // ownership of FIFO_RD_EN and must be able to observe both this
            // register and its delayed word-valid copy return Low.
            if (i_dfi_wrlvl_en &&
                !((phy_state == PHY_WL_DONE) &&
                  (wl_handoff_phase == WL_HANDOFF_DRAIN)))
                calibration_fifo_pop_q <= {BYTE_LANES{1'b1}};
`else
            if (i_dfi_wrlvl_en)
                calibration_fifo_pop_q <= wl_fifo_pop_request;
`endif
            // The final calibration handoff owns each FIFO independently.
            // Its direct EMPTY-qualified reader replaces this registered
            // byte-wide request so no delayed enable can advance a bit past
            // pointer equality.
            if (calibration_tail_drain)
                calibration_fifo_pop_q <= {BYTE_LANES{1'b0}};
        end
    end

    // RXTX_BITSLICE is first-word-present: app_fifo_rd_en_q identifies the
    // current head. At tCK >= 1000 ps it is sampled on the falling edge and
    // consumed on the next rising DIV_CLK edge. At faster rates its stable
    // pre-edge value is captured on that consuming rising edge. The request
    // register was formed only after every bit reported nonempty, and no other reader
    // owns the FIFO in application mode, so the reservation cannot disappear
    // before it is consumed. Do not recompute the capture enable from
    // FIFO_EMPTY at the sampling boundary. Besides losing the final word when
    // EMPTY asserts after a legal pop, that path crosses the FIFO_EMPTY decode and a
    // 256-register CE fanout in one capture decision. At DDR4-1600 hardware
    // showed a single sample bit retaining the preceding word while all other
    // bits updated, precisely the failure mode of that marginal distributed
    // CE path.
    //
    // The Q-data bank is sampled unconditionally.  Only the compact valid bit
    // is qualified, so no FIFO status/control decode drives the data-register
    // clock enables. Hardware qualification places the safe crossover between
    // DDR4-1866 (1071 ps, falling-edge capture) and DDR4-2133 (937 ps,
    // rising-edge capture); the selection is static at elaboration.
    integer app_sample_lane, app_sample_bit;
    integer app_sample_phase, app_sample_idx;
    task app_fifo_capture;
    begin
        if (sync_rst || !app_account_mode) begin
            app_fifo_sample_data_q <=
                {(SERDES_RATIO*DFI_DATA_WIDTH){1'b0}};
            app_fifo_sample_valid_q <= 1'b0;
            app_fifo_rd_en_sample_q <= 1'b0;
            for (app_sample_idx = 0; app_sample_idx < TOTAL_DQ;
                 app_sample_idx = app_sample_idx + 1) begin
                app_fifo_sample_word_q[app_sample_idx] <= 8'b0;
            end
        end else begin
            app_fifo_rd_en_sample_q <= app_fifo_rd_en_q;
            app_fifo_sample_valid_q <= app_fifo_rd_en_q;
            for (app_sample_lane = 0;
                 app_sample_lane < BYTE_LANES;
                 app_sample_lane = app_sample_lane + 1) begin
                for (app_sample_bit = 0;
                     app_sample_bit < DQ_BITS;
                     app_sample_bit = app_sample_bit + 1) begin
                    app_sample_idx = app_sample_lane * DQ_BITS +
                                     app_sample_bit;
                    app_fifo_sample_word_q[app_sample_idx] <=
                        aligned_dq[app_sample_idx];
                    for (app_sample_phase = 0;
                         app_sample_phase < SERDES_RATIO;
                         app_sample_phase = app_sample_phase + 1) begin
                        app_fifo_sample_data_q[
                            app_sample_phase*DFI_DATA_WIDTH +
                            app_sample_idx
                        ] <= aligned_dq[app_sample_idx][
                            2*app_sample_phase];
                        app_fifo_sample_data_q[
                            app_sample_phase*DFI_DATA_WIDTH +
                            TOTAL_DQ + app_sample_idx
                        ] <= aligned_dq[app_sample_idx][
                            2*app_sample_phase + 1];
                    end
                end
            end
`ifdef SIM_NATIVE_RX_DEBUG
            if (app_fifo_rd_en_q || app_fifo_pop_complete_q ||
                app_fifo_sample_valid_q) begin
                $display("[%0t] NATIVE_RX_HALF: rden=%0b complete=%0b valid=%0b empty0=%h q0=%h word0=%02h",
                    $realtime, app_fifo_rd_en_q,
                    app_fifo_pop_complete_q, app_fifo_sample_valid_q,
                    fifo_empty[0], rx_dq_data[0],
                    app_fifo_sample_word_q[0]);
            end
`endif
        end
    end
    endtask

    generate
        if (DDR4_CLK_PERIOD >= 1000) begin : gen_app_fifo_capture_negedge
            always @(negedge i_controller_clk)
                app_fifo_capture;
        end else begin : gen_app_fifo_capture_posedge
            always @(posedge i_controller_clk)
                app_fifo_capture;
        end
    endgenerate

`ifndef SYNTHESIS
    // Training deliberately combines RX_RST with a priming PHY_RDEN pattern,
    // so only application reads are forbidden from overlapping reset.  Once
    // an application read has been accepted, either accounting counter being
    // nonzero means its source-synchronous burst can still be in flight.
    always @(posedge i_controller_clk) begin
        if (!sync_rst && !calibration_mode && rx_fifo_reset_active &&
            ((app_read_pending != 0) || (app_read_due != 0)))
            $error("Native PHY RX FIFO reset overlaps an outstanding application read");
    end
`endif

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
    // Native DQ/DQS data is retained for two DIV_CLK words.  The first word
    // gives the 90-degree TX_BITSLICE_TRI path time to serialize the one-tCK
    // preamble, and the second aligns the complete TBYTE ownership word with
    // the final DQ/DQS payload.  ddr4_top advertises this as tphy_wrlat=2 so
    // the physical first data UI remains exactly CWL clocks after WRITE.
    //
    // TBYTE_IN carries one active-high byte-enable bit per DFI phase.
    // BITSLICE_CONTROL serialises these controls alongside the matching 8:1
    // DQ/DQS word, so they must not be replaced by fabric T.
    // -----------------------------------------------------------------
    always @(posedge i_controller_clk) begin
        if (sync_rst) begin
            tx_wrdata_early <= {(DFI_DATA_WIDTH*4){1'b0}};
            tx_wrdata_pipe0 <= {(DFI_DATA_WIDTH*4){1'b0}};
            tx_wrmask_early <= {(DM_PER_PHASE*4){1'b0}};
            tx_wrmask_pipe0 <= {(DM_PER_PHASE*4){1'b0}};
            wrdata_en_shift <= 3'b000;
        end else begin
            tx_wrdata_early <= i_dfi_wrdata;
            tx_wrdata_pipe0 <= tx_wrdata_early;
            tx_wrmask_early <= i_dfi_wrdata_mask;
            tx_wrmask_pipe0 <= tx_wrmask_early;
            wrdata_en_shift <= {wrdata_en_shift[1:0], wrdata_en_any};
        end
    end

    // The DQS transmitter owns one tCK before the BL8 payload, matching the
    // programmed one-tCK write preamble, then all four phases of the payload
    // word.  DQ takes ownership one additional tCK earlier.  That second
    // predrive pair does not create a DRAM transfer because DQS remains Low;
    // it only lets the bidirectional DQ pads leave High-Z and settle before
    // the first sampling edge.  This is especially important at higher data
    // rates, where pad-enable skew can otherwise leave the phase-0 falling
    // UI at the previous bus value even though the serializer data was loaded
    // early.  Read-to-write turnaround already reserves substantially more
    // than this extra tCK, so DQ never overlaps a DRAM read response.
    //
    // For contiguous writes, wrdata_en_any and wrdata_en_shift[0] overlap;
    // the OR naturally keeps all four phases enabled between payload words.
    wire [3:0] tx_dqs_tbyte_window =
        ({4{wrdata_en_any}}      &
`ifdef SIM_NATIVE_DIAG_TBYTE_LAST_PAIR
         4'b0001) |
`else
         4'b1000) |
`endif
        ({4{wrdata_en_shift[0]}} & 4'b1111);
    wire [3:0] tx_dq_tbyte_window =
        ({4{wrdata_en_any}}      &
`ifdef SIM_NATIVE_DIAG_TBYTE_LAST_PAIR
         4'b0011) |
`else
         4'b1100) |
`endif
        ({4{wrdata_en_shift[0]}} & 4'b1111);
    assign output_enable = |tx_dq_tbyte_window;

    // Keep the input buffer disabled through the complete serialized write,
    // including the native serializer latency and DQS postamble.  The
    // output_enable fabric window ends before the final waveform reaches the
    // pins; rx_write_discard includes the existing three-DIV_CLK drain tail
    // that covers this physical interval.  Reopening earlier lets the final
    // local BL8 enter the RX FIFO and precede the next DRAM read return.
`ifdef SIM_NATIVE_DIAG_RX_INPUT_ALWAYS_ON
    // Diagnostic A/B for application word framing.  The command-timed native
    // DQS gate remains closed during writes; this isolates whether toggling
    // IBUFDISABLE itself disturbs the modulo-8 receive boundary.
    assign rx_input_disable = 1'b0;
`else
    assign rx_input_disable = !calibration_mode && rx_write_discard;
`endif
`ifdef SIM_NATIVE_DIAG_WL_DQS_RX_GUARD
    // IOBUFDSE3 drives O Low while IBUFDISABLE is asserted, which is also
    // the DQS_BIAS idle value.  During write leveling, keep only the DQS
    // receiver disabled while the FPGA takes/releases pad ownership.  DQ
    // remains enabled so the DRAM feedback is still captured.  Opening and
    // closing the DQS receiver while the driven pin is stably Low prevents
    // those ownership changes from becoming two false receive UIs.
    assign rx_dqs_input_disable = rx_input_disable |
        (i_dfi_wrlvl_en && !wl_dqs_rx_armed);
`else
    assign rx_dqs_input_disable = rx_input_disable;
`endif
    wire wl_active;

    // For the UltraScale RXTX_BITSLICE TBYTE_IN path, high enables the
    // byte transmitter and low releases the byte to the DRAM.  This is also
    // required while receiving MPR data during read calibration.
    // UG571 requires TBYTE_IN[3:0] to be pulled High for at least four DIV_CLK
    // cycles after VTC_RDY.  Preserve that reset-sequencer interval at the
    // actual BITSLICE_CONTROL pins, then switch atomically to the existing
    // transmit/write-level controls.  rst_tbyte_en remains High after its
    // startup state, so rst_init_complete is the explicit ownership boundary.
`ifdef SIM_NATIVE_DIAG_WL_BS_RESET
    // UG571 requires TBYTE_IN=0 while BS_CTRL.BS_RESET is asserted when the
    // tristate slice uses TX_OUTPUT_PHASE_90.
    wire wl_bs_reset_window = (phy_state == PHY_WL_DONE) &&
        ((wl_handoff_phase == WL_HANDOFF_CLEAR) ||
         (wl_handoff_phase == WL_HANDOFF_WAIT_CLEAR) ||
         (wl_handoff_phase == WL_HANDOFF_RELEASE) ||
         (wl_handoff_phase == WL_HANDOFF_WAIT_RELEASE));
`else
    wire wl_bs_reset_window = 1'b0;
`endif
    assign tbyte_dq = rst_init_complete ?
        (wl_bs_reset_window ? 4'b0000 : tx_dq_tbyte_window) :
        {4{rst_tbyte_en}};
`ifdef SIM_NATIVE_DIAG_WL_TRAIN_NO_DQS
    // True mode-only A/B diagnostic: program and restore WL_TRAIN without
    // taking ownership of the physical DQS pins.  Suppressing wl_dqs_strobe
    // alone is insufficient because wl_active normally enables TBYTE and
    // drives a static zero throughout the session; that Z-to-zero-to-Z pad
    // activity can itself clock or reframe the receive path.  Production
    // behavior is unchanged when the diagnostic define is absent.
    assign tbyte_dqs = rst_init_complete ?
        (wl_bs_reset_window ? 4'b0000 : tx_dqs_tbyte_window) :
        {4{rst_tbyte_en}};
`else
    assign tbyte_dqs = rst_init_complete ?
        (wl_bs_reset_window ? 4'b0000 :
         ((wl_active
`ifdef SIM_NATIVE_DIAG_WL_DQS_RX_GUARD
           || (wl_dqs_release_guard != 0)
`endif
          ) ? 4'b1111 : tx_dqs_tbyte_window)) :
        {4{rst_tbyte_en}};
`endif

    // -----------------------------------------------------------------
    // DQS Pattern Generation
    //
    // TX_BITSLICE D[7:0] for DQS (8:1 DDR):
    //   Normal write: 01_01_01_01 → one BL8 toggle word
    //   Preamble:     00_00_00_00 → LOW during phase-3 ownership
    //   Write Leveling capture: 00_00_00_01 -> one rising edge
    //   Idle: 00_00_00_00
    // -----------------------------------------------------------------
    reg wl_dqs_strobe;
    always @* begin
        if (wl_active) begin
            if (wl_dqs_strobe)
                // Native RX requires one complete 8-UI word before its legal
                // fabric observation point (FIFO Q) can be read. Every rising
                // edge has the same trained phase relative to CK, so this is
                // one bounded write-leveling capture burst, not normal data.
                // JESD79-4D 4.7.2 requires one DQS rising edge followed by
                // asynchronous DQ feedback. Four consecutive edges here
                // would retrigger the DRAM much faster than tWLO/tWLOE and
                // produce a mixed, non-leveling DQ word in hardware.
                dqs_pattern = 8'b00_00_00_01;
            else
                dqs_pattern = 8'b00_00_00_00;
        end else if (wrdata_en_shift[1]) begin
            dqs_pattern = 8'b01_01_01_01;
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
                    tx_wrdata_native[3*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 3 fall
                    tx_wrdata_native[3*DFI_DATA_WIDTH + DQ_IDX],            // phase 3 rise
                    tx_wrdata_native[2*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 2 fall
                    tx_wrdata_native[2*DFI_DATA_WIDTH + DQ_IDX],            // phase 2 rise
                    tx_wrdata_native[1*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 1 fall
                    tx_wrdata_native[1*DFI_DATA_WIDTH + DQ_IDX],            // phase 1 rise
                    tx_wrdata_native[0*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 0 fall
                    tx_wrdata_native[0*DFI_DATA_WIDTH + DQ_IDX]             // phase 0 rise
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
                    ~tx_wrmask_native[3*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 3 fall
                    ~tx_wrmask_native[3*DM_PER_PHASE + dm_lane],              // phase 3 rise
                    ~tx_wrmask_native[2*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 2 fall
                    ~tx_wrmask_native[2*DM_PER_PHASE + dm_lane],              // phase 2 rise
                    ~tx_wrmask_native[1*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 1 fall
                    ~tx_wrmask_native[1*DM_PER_PHASE + dm_lane],              // phase 1 rise
                    ~tx_wrmask_native[0*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 0 fall
                    ~tx_wrmask_native[0*DM_PER_PHASE + dm_lane]               // phase 0 rise
                };
            end else begin : dm_stub
                assign tx_dm_data[dm_lane] = 8'hFF; // no mask: DM_n always high
            end
        end
    endgenerate

    // -----------------------------------------------------------------
    // Native BL8 Word Boundary
    //
    // A rotating MPR value is sufficient to locate an analog DQ eye, but it
    // is not sufficient to prove the BL8 boundary.  Hardware captures showed
    // the consequence directly: accepting raw MPR E1 at offset one produced
    // application words 01/FE on DQ pins that should have been FF/00.  Those
    // are adjacent-burst fragments, not random bit errors.  A cyclic barrel
    // shift of one eight-UI FIFO word cannot repair that condition because the
    // missing samples belong to another burst, and a two-word fabric stitch
    // would make an isolated READ depend on a later READ.
    //
    // The DQS gate already establishes the physical BL8 boundary (the AXKU3
    // capture showed a complete 55 DQS word).  Eye training below therefore
    // accepts only the canonical, unrotated command-associated MPR0/MPR2 word
    // on every DQ in the byte. Application data can then consume Q directly,
    // exactly one FIFO word per READ, including an isolated READ.
    // -----------------------------------------------------------------
    reg [3:0]  bitslip_count_q [0:BYTE_LANES-1];

    // Native DQS-gate calibration sub-FSM. Each candidate mCL is held fixed
    // while the complete per-byte RL_DLY range is swept. Bytes that find a
    // window retain that coarse/fine pair; only unresolved bytes depend on the
    // following candidate. This is entirely PHY-local and does not alter DFI.
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
                     GATE_FLUSH_WAIT = 5'd19,
                     GATE_CANDIDATE_FLUSH = 5'd20,
                     GATE_COMMIT_MCL = 5'd21,
                     GATE_REWIND_WRITE = 5'd22,
                     GATE_REWIND_WAIT = 5'd23,
                     GATE_READ_STATUS_LOW = 5'd24,
                     GATE_WAIT_STATUS_LOW = 5'd25;
    (* mark_debug = "true" *) reg [4:0] gate_phase;
    reg [3:0] gate_observe_count;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] gate_fresh_seen;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] gate_fresh_seen_low;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] gate_match_seen;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] gate_status_seen;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] gate_status_seen_low;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] gate_in_range;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] gate_in_range_low;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] gate_best_valid;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] gate_best_valid_low;
    reg [8:0] gate_cur_start [0:BYTE_LANES-1];
    reg [8:0] gate_cur_width [0:BYTE_LANES-1];
    reg [8:0] gate_best_start [0:BYTE_LANES-1];
    reg [8:0] gate_best_width [0:BYTE_LANES-1];
    reg [8:0] gate_center [0:BYTE_LANES-1];
    reg [8:0] gate_cur_start_low [0:BYTE_LANES-1];
    reg [8:0] gate_cur_width_low [0:BYTE_LANES-1];
    reg [8:0] gate_best_start_low [0:BYTE_LANES-1];
    reg [8:0] gate_best_width_low [0:BYTE_LANES-1];
    reg [8:0] gate_center_low [0:BYTE_LANES-1];
    reg [8:0] gate_restore_tap;
    reg       gate_restore_lower;
    // RX_GATING is enabled on both data nibbles. Require both the primitive's
    // GT_STATUS gate-placement indication and a freshly completed FIFO word:
    // the status alone does not prove that all eight DQ slices produced a
    // complete BL8 word, while FIFO activity alone can occur at a clipped or
    // periodic DQS copy. MPR equality is deliberately deferred because DQ
    // IDELAY is still untrained here. The following full IDELAY eye sweep
    // requires repeatable cyclic MPR data, verifies the selected
    // mCL/RL_DLY tuple, and rejects a clipped word.
    wire [BYTE_LANES-1:0] gate_candidate_valid =
        gate_fresh_seen & gate_status_seen;
    // The DQS is physically connected to one BITSLICE_CONTROL.  Its imported
    // PCLK/NCLK feeds the companion nibble, whose command timing is trained
    // later with the exact MPR word.  GT_STATUS is therefore qualified only
    // on the DQS-owning control; treating the imported control as a second DQS
    // detector rejects valid hardware because it has no independent DQS edge
    // monitor.
    wire [BYTE_LANES-1:0] gate_candidate_valid_low =
        gate_candidate_valid;
    wire [BYTE_LANES-1:0] gate_window_closes =
        gate_in_range & ~gate_candidate_valid;
    wire [BYTE_LANES-1:0] gate_window_closes_low =
        gate_in_range_low & ~gate_candidate_valid_low;
    wire [BYTE_LANES-1:0] gate_window_wide;
    wire [BYTE_LANES-1:0] gate_window_wide_low;
    generate
        genvar gate_qual_lane;
        for (gate_qual_lane = 0; gate_qual_lane < BYTE_LANES;
             gate_qual_lane = gate_qual_lane + 1) begin : gen_gate_window_wide
            assign gate_window_wide[gate_qual_lane] =
                gate_cur_width[gate_qual_lane] >= GATE_MIN_WINDOW_TAPS;
            assign gate_window_wide_low[gate_qual_lane] =
                gate_cur_width_low[gate_qual_lane] >= GATE_MIN_WINDOW_TAPS;
        end
    endgenerate
    wire [BYTE_LANES-1:0] gate_window_qualifies =
        gate_window_closes & gate_window_wide;
    wire [BYTE_LANES-1:0] gate_window_qualifies_low =
        gate_window_closes_low & gate_window_wide_low;
    wire gate_all_first_windows_complete =
        &(gate_lane_resolved | gate_best_valid |
          gate_window_qualifies);
    wire [BYTE_LANES-1:0] gate_resolved_after_candidate =
        gate_lane_resolved | gate_best_valid;
    wire [BYTE_LANES-1:0] gate_resolved_after_candidate_low =
        gate_resolved_after_candidate;

    wire [BYTE_LANES-1:0] train_lane_mask =
        ({{(BYTE_LANES-1){1'b0}}, 1'b1} << train_lane);
    wire [8:0] gate_target_tap = gate_restore_lower ?
        gate_center_low[train_lane] : gate_center[train_lane];
    wire [3:0] gate_target_coarse_raw = gate_restore_lower ?
        gate_trained_coarse_low[train_lane] :
        gate_trained_coarse[train_lane];
    wire [3:0] gate_target_coarse = gate_target_coarse_raw;
    wire [8:0] gate_restore_next =
        delay_step_toward(gate_restore_tap, gate_target_tap);
    wire [8:0] gate_rewind_next =
        delay_step_toward(gate_restore_tap, 9'd0);

    // Eye training registers (phase-aware range tracking)
    (* mark_debug = "true" *) reg [8:0] sweep_tap;
    (* mark_debug = "true" *) reg [8:0] cur_start;
    (* mark_debug = "true" *) reg [8:0] cur_width;
    (* mark_debug = "true" *) reg in_range;
    (* mark_debug = "true" *) reg [8:0] best_start;
    (* mark_debug = "true" *) reg [8:0] best_width;
    (* mark_debug = "true" *) reg best_valid;
    (* mark_debug = "true" *) reg pattern_found_q;
    (* mark_debug = "true" *) reg [3:0] pattern_offset_q;
    (* mark_debug = "true" *) reg [3:0] cur_offset;
    (* mark_debug = "true" *) reg [3:0] best_offset;
    (* mark_debug = "true" *) reg [3:0] eye_reference_offset;
    reg       eye_observe_verify;
    reg [3:0] eye_observe_count;
    (* mark_debug = "true" *) reg eye_observe_seen;
    reg eye_prime_discard;
    (* mark_debug = "true" *) reg [3:0] eye_observe_offset;
    reg [1:0] eye_verify_retries;
    // Pipeline the complete-byte comparison before it reaches the eye FSM.
    // The state remains active for one extra clock so a match on the final
    // observed native word is retained without a primitive-to-control path.
    reg eye_observe_match_q;
    reg [3:0] eye_observe_offset_q;
    // A byte-wide exact match is still required for final verification, but
    // the analog eye sweep records each DQ independently.  XiPHY provides one
    // RX delay per data bit; collapsing those measurements to one byte-wide
    // intersection needlessly discards timing margin at high data rates.
    reg [DQ_BITS-1:0] eye_observe_bit_match_q;
    reg [DQ_BITS-1:0] eye_observe_bit_seen;
    reg [DQ_BITS-1:0] eye_bit_in_range;
    reg [DQ_BITS-1:0] eye_bit_best_valid;
    reg [8:0] eye_bit_cur_start [0:DQ_BITS-1];
    reg [8:0] eye_bit_cur_width [0:DQ_BITS-1];
    reg [8:0] eye_bit_best_start [0:DQ_BITS-1];
    reg [8:0] eye_bit_best_width [0:DQ_BITS-1];
    reg [8:0] eye_bit_center_tap [0:TOTAL_DQ-1];
    // Reduce the eight independently measured eyes over several calibration
    // clocks.  A combinational minimum across every DQ creates a long carry
    // cascade even though this result is needed only once per sweep.
    reg [1:0] eye_decide_phase;
    reg [3:0] eye_reduce_index;
    reg       eye_reduce_all_valid;
    reg [8:0] eye_reduce_min_width;
    reg       idelay_per_dq_at_center;
    integer eye_bit_reduce_idx;
    integer eye_bit_track_idx;
    always @* begin
        idelay_per_dq_at_center = 1'b1;
        for (eye_bit_reduce_idx = 0;
             eye_bit_reduce_idx < DQ_BITS;
             eye_bit_reduce_idx = eye_bit_reduce_idx + 1) begin
            if (idelay_cntvalue_per_dq[
                    eye_bit_reduce_idx*9 +: 9] !=
                eye_bit_center_tap[
                    train_lane*DQ_BITS + eye_bit_reduce_idx])
                idelay_per_dq_at_center = 1'b0;
        end
    end
    reg [BYTE_LANES-1:0] rd_lat_extra;
    reg [8:0] eye_center_tap [0:BYTE_LANES-1];
    reg [8:0] eye_best_width [0:BYTE_LANES-1];
    reg [8:0] eye_best_start [0:BYTE_LANES-1];
    wire [8:0] eye_center_candidate = best_start + (best_width >> 1);
`ifdef SIM_NATIVE_DIAG_FINAL_EYE_RESET_ALIGN
    // Directed simulation experiment for the calibration/application FIFO boundary.  The gate
    // and eye sweeps consume many source-synchronous words and can leave the
    // native FIFO pointers at an arbitrary modulo-8 phase.  While MPR mode is
    // still active, reset the RX FIFOs and require a fresh complete word from
    // every byte before acknowledging eye training.
    localparam [1:0] EYE_FINAL_ALIGN_RESET = 2'd0,
                     EYE_FINAL_ALIGN_DATA  = 2'd1,
                     EYE_FINAL_ALIGN_DRAIN = 2'd2;
    reg eye_final_align_active;
    reg [1:0] eye_final_align_phase;
    reg [BYTE_LANES-1:0] eye_final_align_seen;
`endif
    // GT_STATUS is periodic in DQS, so gate training can legitimately lock to
    // an adjacent command-cycle copy.  Eye training resolves that ambiguity
    // using actual MPR data. A byte has one DQS-owned gate, so search one
    // bounded mCL sequence and apply it identically to both nibble controls.
    // Independent upper/lower candidates can tear bits of one byte across
    // adjacent BL8 returns.
    localparam integer NATIVE_EYE_MCL_CANDIDATES = 5;
    // A single matching sample has no useful PVT margin and can be a clipped
    // periodic MPR copy.  Require five consecutive samples at the four-tap
    // sweep interval before accepting a physical DQ eye.
    localparam [8:0] EYE_MIN_WINDOW_TAPS = 9'd16;
    (* mark_debug = "true" *) reg [5:0] eye_mcl_base;
    (* mark_debug = "true" *) reg [5:0] eye_mcl_base_low;
    (* mark_debug = "true" *) reg [2:0] eye_mcl_upper_index;
    (* mark_debug = "true" *) reg [2:0] eye_mcl_index;
    wire eye_mcl_has_next =
        eye_mcl_upper_index < NATIVE_EYE_MCL_CANDIDATES - 1;
    wire [2:0] eye_mcl_next_upper_index =
        eye_mcl_upper_index + 1'b1;
    wire [2:0] eye_mcl_next_lower_index =
        eye_mcl_next_upper_index;
    wire [5:0] eye_mcl_next_upper = native_eye_mcl_candidate(
        eye_mcl_base, eye_mcl_next_upper_index);
    wire [5:0] eye_mcl_next_lower = eye_mcl_next_upper;
    (* mark_debug = "true" *) reg [5:0] eye_gate_raw_mcl;
    (* mark_debug = "true" *) reg [3:0] eye_gate_raw_coarse;
    (* mark_debug = "true" *) reg [1:0] eye_gate_phase_retry;
    (* mark_debug = "true" *) reg eye_gate_retry_program_pending;
    // A physical RL_DLY retry crosses the same BISC/VTC boundary as the
    // original gate-to-eye handoff.  Keep the retry in two explicit phases:
    // first qualify VTC before programming RL_DLY, then reopen VTC and capture
    // a fresh per-bit Align_Delay baseline before any DQ VAR_LOAD.  Reusing a
    // baseline captured before the gate moved can reduce the legal eye range
    // differently on every reset and is therefore not safe in hardware.
    (* mark_debug = "true" *) reg eye_gate_retry_programmed;
    (* mark_debug = "true" *) reg [3:0] eye_gate_retry_vtc_ready_count;
    // A lane-local physical gate retry can disturb the armed state of another
    // byte's native DQS gate while BITSLICE_CONTROL VTC is paused.  After all
    // byte eyes have passed, replay the saved RIU tuple and CLR_GATE/RUN
    // sequence for every lane before the final exact-MPR verification.  This
    // does not change any trained delay; it only restores deterministic native
    // gate state after the last lane-specific calibration transaction.
    (* mark_debug = "true" *) reg eye_gate_rearm_all;
    wire [1:0] eye_gate_next_retry = eye_gate_phase_retry + 1'b1;
    wire [5:0] eye_gate_retry_mcl = native_eye_gate_retry_mcl(
        eye_gate_raw_mcl, eye_gate_raw_coarse, eye_gate_next_retry);
    wire [3:0] eye_gate_retry_coarse = native_eye_gate_retry_coarse(
        eye_gate_raw_mcl, eye_gate_raw_coarse, eye_gate_next_retry);

    // Write leveling registers (ODELAYE3 DQS sweep)
    reg [8:0] wl_tap        [0:BYTE_LANES-1];
    reg [8:0] wl_dq_tap     [0:BYTE_LANES-1];
    reg [3:0] wl_coarse     [0:BYTE_LANES-1];
    (* mark_debug = "true" *) reg [3:0] wl_sweep_coarse;
    reg       wl_seen_zero  [0:BYTE_LANES-1];
    reg [8:0] dqs_initial_tap [0:BYTE_LANES-1];
    // -----------------------------------------------------------------
    // Native write-level feedback capture
    // -----------------------------------------------------------------
    // An UltraScale IOB receiver that feeds RXTX_BITSLICE.DATAIN cannot also
    // drive fabric (REQP-1922). Consequently write leveling must consume Q,
    // the bit slice's documented FPGA-side output. Q is an 8-UI FIFO word.
    // At each tap the first single-edge DQS pulse primes the DRAM response
    // while RX is held reset. Four later, independently spaced DFI strobes
    // contribute two samples each and form one settled native FIFO word.
    // This preserves the DFI one-strobe/one-pulse contract and the JEDEC tWLO
    // interval while remaining independent of package and board delay.
    (* mark_debug = "true" *) reg wl_prime_pending;
    (* mark_debug = "true" *) reg [4:0] wl_word_wait_count;
    (* mark_debug = "true" *) reg wl_fifo_reset_q;
`ifdef SIM_NATIVE_DIAG_WL_DQS_RX_GUARD
    (* mark_debug = "true" *) reg wl_dqs_rx_armed;
    (* mark_debug = "true" *) reg [2:0] wl_dqs_release_guard;
`endif
    (* mark_debug = "true" *) reg wl_feedback_clear;
    (* mark_debug = "true" *) reg wl_feedback_valid;
    (* mark_debug = "true" *) reg [7:0] wl_feedback_word;
    (* mark_debug = "true" *) reg [DQ_BITS*8-1:0]
        wl_feedback_lane_word;
    (* mark_debug = "true" *) reg [1:0] wl_mixed_retries;
    localparam [1:0] WL_RIU_PROGRAM = 2'd0,
                     WL_RIU_WAIT    = 2'd1,
                     WL_RIU_CAPTURE = 2'd2;
    (* mark_debug = "true" *) reg [1:0] wl_riu_phase;
    // Write leveling contributes complete 8-UI receive words.  The handoff
    // drains and freezes those words before application traffic.  Do not use
    // BS_CTRL.BS_RESET here: it restores bit-slice state after delay-clock
    // disturbances but does not select a DDR4 burst boundary, and therefore
    // cannot replace read-gate/word-alignment training.
    (* mark_debug = "true" *) reg [TOTAL_DQ-1:0]
        wl_handoff_nonempty_seen;
    reg [3:0] wl_handoff_gate_coarse;
    reg [8:0] wl_handoff_gate_tap;
    localparam [2:0] EYE_GATE_REWIND       = 3'd0,
                     EYE_GATE_WAIT_REWIND  = 3'd1,
                     EYE_GATE_COARSE       = 3'd2,
                     EYE_GATE_WAIT_COARSE  = 3'd3,
                     EYE_GATE_RESTORE      = 3'd4,
                     EYE_GATE_WAIT_RESTORE = 3'd5,
                     EYE_GATE_SETTLE       = 3'd6,
                     EYE_GATE_COMPLETE     = 3'd7;
`ifdef SIM_NATIVE_DIAG_FINALIZE_GATE_PLUS4
    localparam [3:0] EYE_GATE_COARSE_OFFSET = 4'd4;
`elsif SIM_NATIVE_DIAG_EYE_RETIME_PLUS1
    localparam [3:0] EYE_GATE_COARSE_OFFSET = 4'd1;
`elsif SIM_NATIVE_DIAG_EYE_RETIME_PLUS3
    localparam [3:0] EYE_GATE_COARSE_OFFSET = 4'd3;
`else
    localparam [3:0] EYE_GATE_COARSE_OFFSET = 4'd2;
`endif
    reg [2:0] eye_gate_finalize_phase;
    reg [3:0] eye_gate_finalize_coarse;
    reg [8:0] eye_gate_finalize_tap;
    reg eye_gate_finalize_started;
    reg [3:0] eye_gate_finalize_target_coarse_q;
`ifdef SIM_NATIVE_DIAG_FINALIZE_GATE_PLUS2
    wire [3:0] eye_gate_finalize_target_coarse =
        gate_trained_coarse[train_lane] + EYE_GATE_COARSE_OFFSET;
`elsif SIM_NATIVE_DIAG_FINALIZE_GATE_PLUS4
    wire [3:0] eye_gate_finalize_target_coarse =
        gate_trained_coarse[train_lane] + EYE_GATE_COARSE_OFFSET;
`else
    wire [3:0] eye_gate_finalize_target_coarse =
        eye_gate_finalize_target_coarse_q;
`endif
    wire [3:0] eye_gate_finalize_next_coarse =
        (eye_gate_finalize_coarse < eye_gate_finalize_target_coarse) ?
        (eye_gate_finalize_coarse + 1'b1) :
        (eye_gate_finalize_coarse - 1'b1);
`ifdef SIM_NATIVE_DIAG_FINALIZE_GATE_FINE_MINUS16
    wire [8:0] eye_gate_finalize_target_tap =
        (gate_center[train_lane] >= 9'd16) ?
        (gate_center[train_lane] - 9'd16) : 9'd0;
`else
    wire [8:0] eye_gate_finalize_target_tap =
        gate_center[train_lane];
`endif
`ifdef SIM_NATIVE_RIU_DEBUG
    reg [7:0] sim_wl_handoff_diag_div;
    reg [7:0] sim_eye_finalize_diag_div;
    always @(posedge i_controller_clk) begin
        if (sync_rst || (phy_state != PHY_WL_DONE)) begin
            sim_wl_handoff_diag_div <= 8'd0;
        end else begin
            sim_wl_handoff_diag_div <= sim_wl_handoff_diag_div + 1'b1;
            if (sim_wl_handoff_diag_div == 8'd0)
                $display("[%0t] NATIVE_WL_HANDOFF: phase=%0d timer=%0d lane=%0d en_vtc=%0b valid=%b rd0=%h rd1=%h rd2=%h rd3=%h empty=%h dqs_empty=%b pop=%b word_valid=%b dfi_wl=%0b wl_active=%0b strobe=%0b dqs_pat=%h tbyte_dqs=%b rden=%h dqs_ibufdis=%0b",
                    $realtime, wl_handoff_phase, phy_timer, train_lane,
                    en_vtc_q, native_riu_valid,
                    native_riu_rd_data[0], native_riu_rd_data[1],
                    native_riu_rd_data[2], native_riu_rd_data[3],
                    fifo_empty_flat, dqs_fifo_empty,
                    calibration_fifo_pop_q,
                    calibration_fifo_word_valid_q,
                    i_dfi_wrlvl_en, wl_active, wl_dqs_strobe,
                    dqs_pattern, tbyte_dqs, phy_rden_ready,
                    rx_dqs_input_disable);
        end
        if (sync_rst || (phy_state != PHY_EYE_DONE)) begin
            sim_eye_finalize_diag_div <= 8'd0;
        end else begin
            sim_eye_finalize_diag_div <= sim_eye_finalize_diag_div + 1'b1;
            if (sim_eye_finalize_diag_div == 8'd0)
                $display("[%0t] NATIVE_EYE_FINALIZE: phase=%0d timer=%0d lane=%0d coarse=%0d fine=%0d valid=%b rd=%h",
                    $realtime, eye_gate_finalize_phase, phy_timer,
                    train_lane, eye_gate_finalize_coarse,
                    eye_gate_finalize_tap, native_riu_valid,
                    native_riu_rd_data[train_lane]);
        end
    end
`endif
    // FIFO_EMPTY is synchronized by the source-synchronous write domain and
    // need not change after the final DQS edge.  Therefore it cannot terminate
    // a post-WL drain.  The WL FSM already waits for the one FIFO word produced
    // by each training pulse; freeze FIFO_RD_EN as soon as the last word is
    // accepted and preserve the canonical pointer phase established by MPR.
`ifdef SIM_NATIVE_DIAG_WL_FINAL_RESET_PRIME
    // The dedicated one-way handoff drain owns FIFO_RD_EN throughout
    // PHY_WL_DONE.  Freeze the normal byte-wide calibration reader so the two
    // policies cannot advance the same pointer on one DIV_CLK edge.
    assign wl_handoff_fifo_freeze = (phy_state == PHY_WL_DONE);
`elsif SIM_NATIVE_DIAG_WL_DRAIN_TO_EMPTY
    assign wl_handoff_fifo_freeze = 1'b0;
`else
    assign wl_handoff_fifo_freeze = (phy_state == PHY_WL_DONE);
`endif
    (* mark_debug = "true" *) reg wl_handoff_readback_ok;
    integer wl_handoff_lane;
    always @* begin
        wl_handoff_readback_ok = 1'b1;
        for (wl_handoff_lane = 0; wl_handoff_lane < BYTE_LANES;
             wl_handoff_lane = wl_handoff_lane + 1) begin
            if ((wl_handoff_phase != WL_HANDOFF_WAIT_GATE_REWIND) &&
                (wl_handoff_phase != WL_HANDOFF_WAIT_GATE_COARSE) &&
                (wl_handoff_phase != WL_HANDOFF_WAIT_GATE_RESTORE))
                wl_handoff_readback_ok = wl_handoff_readback_ok &&
                    native_riu_valid[wl_handoff_lane];
            case (wl_handoff_phase)
                WL_HANDOFF_WAIT_CLEAR,
                WL_HANDOFF_WAIT_RELEASE:
`ifdef SIM_NATIVE_DIAG_WL_BS_RESET
                    wl_handoff_readback_ok = wl_handoff_readback_ok &&
                        ((native_riu_rd_data[wl_handoff_lane] &
                          RIU_BS_RESET_MASK) ==
                         ((wl_handoff_phase == WL_HANDOFF_WAIT_CLEAR) ?
                          RIU_BS_RESET_MASK : 16'h0000));
`else
                    wl_handoff_readback_ok = wl_handoff_readback_ok &&
                        (native_riu_rd_data[wl_handoff_lane][5:4] == 2'b11) &&
                        (native_riu_rd_data[wl_handoff_lane][8] ==
                         (wl_handoff_phase == WL_HANDOFF_WAIT_CLEAR));
`endif
                WL_HANDOFF_WAIT_GATE_REWIND,
                WL_HANDOFF_WAIT_GATE_COARSE,
                WL_HANDOFF_WAIT_GATE_RESTORE:
                    if (wl_handoff_lane == train_lane)
                        wl_handoff_readback_ok = wl_handoff_readback_ok &&
                            native_riu_valid[wl_handoff_lane] &&
                            (native_riu_rd_data[wl_handoff_lane][12:9] ==
                             wl_handoff_gate_coarse) &&
                            (native_riu_rd_data[wl_handoff_lane][8:0] ==
                             wl_handoff_gate_tap);
                default:
                    wl_handoff_readback_ok = 1'b0;
            endcase
        end
    end

    wire eye_gate_finalize_readback_ok =
        native_riu_valid[train_lane] &&
        (native_riu_rd_data[train_lane][12:9] ==
         eye_gate_finalize_coarse) &&
        (native_riu_rd_data[train_lane][8:0] ==
         eye_gate_finalize_tap);
`ifdef SIM_NATIVE_DIAG_MANUAL_WL_TBYTE
    // Directed isolation: the native TBYTE controls already release every DQ
    // and enable only the selected DQS during write leveling.  Keep WL_TRAIN
    // clear while applying the same coordinated WL_DLY value to determine
    // whether the primitive mode transition, rather than the trained delay,
    // moves the receive FIFO's modulo-8 word boundary.
    wire wl_riu_train_mode = 1'b0;
`else
    wire wl_riu_train_mode = 1'b1;
`endif
    wire [15:0] wl_riu_train_value = {2'd0, wl_riu_train_mode,
                                      wl_sweep_coarse,
                                      wl_tap[train_lane]};
    wire [15:0] wl_riu_run_value = {3'd0,
                                    wl_coarse[train_lane],
                                    wl_tap[train_lane]};
`ifdef SIM_NATIVE_TX_DEBUG_FAST_WL
    // Preserve the production primer/capture/RIU sequence in fast XSim runs;
    // only shorten the all-high Micron-model sweep to its first candidate.
    // Bypassing PHY_WL_ADJUST changes the modulo-8 RX phase and cannot verify
    // the post-calibration application datapath.
    wire wl_sweep_at_end = 1'b1;
    wire wl_fine_at_end = 1'b1;
`else
    wire wl_sweep_at_end =
        (wl_sweep_coarse == 4'hf) &&
        (wl_tap[train_lane] >= WL_SWEEP_LAST);
    wire wl_fine_at_end = wl_tap[train_lane] >= WL_SWEEP_LAST;
`endif
    wire [8:0] wl_next_fine = wl_fine_at_end ? 9'd0 :
        (wl_tap[train_lane] + WL_TAP_STEP);
    wire [3:0] wl_next_coarse = wl_fine_at_end ?
        (wl_sweep_coarse + 1'b1) : wl_sweep_coarse;
    // Never let the local primer reset escape the DFI write-leveling window.
    // This also releases RX immediately if the MC aborts a timed-out attempt.
    // Each write-leveling observation starts from a known native FIFO state.
    // This reset is confined to the WL primer/capture sequence, while no
    // application READ can be outstanding, and is released before the DQS
    // feedback pulse. The post-WL byte-wide BS_CTRL transaction then establishes
    // the common application BL8 boundary.
`ifdef SIM_NATIVE_DIAG_WL_NO_RX_RESET
    assign wl_rx_fifo_reset = 1'b0;
`else
    assign wl_rx_fifo_reset = wl_fifo_reset_q & i_dfi_wrlvl_en;
`endif
    // A reset at a genuinely idle READ-group boundary removes locally echoed
    // TX fragments and establishes a common modulo-8 word boundary.  The full
    // pipeline-idle predicate above guarantees that it cannot overlap an
    // older source-synchronous burst; the DDR4 command-to-data interval gives
    // the primitive time to acknowledge and release reset before read DQS.
    assign rx_fifo_reset_active = bitslice_rst | rx_fifo_flush |
                                  wl_rx_fifo_reset;

    // Training failure latch registers
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] gate_train_fail;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] eye_train_fail;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0] wl_train_fail;

    reg  wl_feedback_zero;
    reg  wl_feedback_one;
    integer wl_feedback_bit;
    always @* begin
        // JESD79-4D requires every DQ in the selected byte to carry the same
        // write-level feedback. A mixed word is not a valid zero or one even
        // when the arbitrarily selected DQ0 happens to look stable.
        wl_feedback_zero = wl_feedback_valid;
        wl_feedback_one  = wl_feedback_valid;
        for (wl_feedback_bit = 0; wl_feedback_bit < DQ_BITS;
             wl_feedback_bit = wl_feedback_bit + 1) begin
            wl_feedback_zero = wl_feedback_zero &&
                (wl_feedback_lane_word[wl_feedback_bit*8 +: 8] === 8'h00);
            wl_feedback_one = wl_feedback_one &&
                (wl_feedback_lane_word[wl_feedback_bit*8 +: 8] === 8'hff);
        end
    end

    // FIFO_RD_EN is registered at cycle N and RXTX_BITSLICE updates Q after
    // that rising edge. calibration_fifo_word_valid_q marks the completed pop
    // at N+1. Capture Q on the following controller-clock edge, where it has
    // been stable for a complete cycle. Keeping this path entirely in the
    // controller-clock domain avoids a fragile half-cycle qualification path
    // and works identically for all supported controller frequencies.
    (* mark_debug = "true" *) wire wl_feedback_capture_qual =
        (phy_state == PHY_WL_ADJUST) && !wl_prime_pending &&
        (wl_capture_pulse_count >= WL_CAPTURE_PULSES-1'b1) &&
        calibration_fifo_word_valid_q[train_lane];
    always @(posedge i_controller_clk) begin
        if (sync_rst) begin
            wl_feedback_valid <= 1'b0;
            wl_feedback_word  <= 8'h00;
            wl_feedback_lane_word <= {(DQ_BITS*8){1'b0}};
        end else if (wl_feedback_clear) begin
            wl_feedback_valid <= 1'b0;
            wl_feedback_word  <= 8'h00;
            wl_feedback_lane_word <= {(DQ_BITS*8){1'b0}};
        end else if (!wl_feedback_valid && wl_feedback_capture_qual) begin
            wl_feedback_lane_word <= rx_dq_data[train_lane];
            wl_feedback_word <= rx_dq_data[train_lane][7:0];
            wl_feedback_valid <= 1'b1;
        end
    end

`ifdef SIM_NATIVE_WL_POP_DEBUG
    always @(posedge i_controller_clk) begin
        if (!sync_rst && i_dfi_wrlvl_en &&
            (wl_dqs_strobe || (|wl_fifo_pop_request) ||
             (|calibration_fifo_pop_q) ||
             (|calibration_fifo_word_valid_q) ||
             (phy_state == PHY_WL_DONE))) begin
            $display("[%0t] NATIVE_WL_POP: state=%0d handoff=%0d lane=%0d prime=%0b pulses=%0d strobe=%0b request=%b issued=%b pop=%b valid=%b empty=%h q0=%h qlast=%h feedback=%0b",
                $realtime, phy_state, wl_handoff_phase, train_lane,
                wl_prime_pending, wl_capture_pulse_count, wl_dqs_strobe,
                wl_fifo_pop_request, wl_fifo_pop_issued,
                calibration_fifo_pop_q, calibration_fifo_word_valid_q,
                fifo_empty_flat, rx_dq_data[0],
                rx_dq_data[BYTE_LANES-1], wl_feedback_valid);
        end
    end
`endif

`ifdef SIM_NATIVE_MIG_WL_DEBUG
    // Focused comparison with MIG's write-level reader.  MIG keeps every
    // native FIFO_RD_EN asserted throughout write leveling and samples Q in
    // fabric; it does not derive data validity from FIFO_EMPTY in this mode.
    // Report only physical pulse and settled-observation boundaries so this
    // diagnostic remains usable without generating a multi-gigabyte log.
    always @(posedge i_controller_clk) begin
        if (!sync_rst && i_dfi_wrlvl_en &&
            (wl_dqs_strobe ||
             ((phy_state == PHY_WL_ADJUST) && (phy_timer == 0)))) begin
            $display("[%0t] NATIVE_MIG_WL: state=%0d lane=%0d prime=%0b pulses=%0d strobe=%0b empty=%h q0=%h q1=%h q2=%h q3=%h feedback_valid=%0b feedback=%h",
                $realtime, phy_state, train_lane, wl_prime_pending,
                wl_capture_pulse_count, wl_dqs_strobe, fifo_empty_flat,
                rx_dq_data[0], rx_dq_data[1], rx_dq_data[2], rx_dq_data[3],
                wl_feedback_valid, wl_feedback_lane_word);
        end
    end
`endif

    assign wl_active = (phy_state == PHY_WL_SAMPLE) ||
                       (phy_state == PHY_WL_ADJUST) ||
                       (phy_state == PHY_WL_APPLY)  ||
                       (phy_state == PHY_WL_CHECK)
`ifdef SIM_NATIVE_DIAG_WL_GATE_PLUS2
                       || (phy_state == PHY_WL_DONE)
`elsif SIM_NATIVE_DIAG_WL_GATE_PLUS1
                       || (phy_state == PHY_WL_DONE)
`elsif SIM_NATIVE_DIAG_WL_GATE_PLUS4
                       || (phy_state == PHY_WL_DONE)
`elsif SIM_NATIVE_DIAG_WL_FINAL_RESET_PRIME
                       || (phy_state == PHY_WL_DONE)
`endif
                       ;

    // -----------------------------------------------------------------
    // Receive Word Alignment
    // -----------------------------------------------------------------
    // The trained DQS gate plus exact-MPR DQ eye establishes the canonical
    // native FIFO word.  Preserve it without a fabric rotation so an isolated
    // READ cannot borrow UIs from a preceding or following burst.
    generate
        genvar bs_lane, bs_bit;
        for (bs_lane = 0; bs_lane < BYTE_LANES; bs_lane = bs_lane + 1) begin : gen_bs_lane
            for (bs_bit = 0; bs_bit < DQ_BITS; bs_bit = bs_bit + 1) begin : gen_bs_bit
                localparam integer BS_IDX = bs_lane * DQ_BITS + bs_bit;
`ifdef SIM_NATIVE_DIAG_ROTATED_MPR_EYE
                // The selected offset is confined to this FIFO word. It never
                // stitches adjacent reads: a passing isolated-read test proves
                // that all eight BL8 UIs were captured by the native gate.
                assign aligned_dq[BS_IDX] =
                    ({iserdes_dq_q[BS_IDX], iserdes_dq_q[BS_IDX]} >>
                     bitslip_count_q[bs_lane]);
`else
                assign aligned_dq[BS_IDX] = iserdes_dq_q[BS_IDX];
`endif
            end
        end
    endgenerate

    // -----------------------------------------------------------------
    // Eye Training: canonical BL8 boundary and analog-eye comparison
    // -----------------------------------------------------------------
    // Gate training establishes a complete DQS-framed BL8 word before the DQ
    // eye sweep. Only the exact word requested by the associated MPR READ on
    // all eight DQ pins is a legal eye sample. Accepting a cyclic rotation or
    // the preceding command's MPR page would silently splice neighboring BL8
    // bursts at the native FIFO Q.
    wire [DQ_BITS-1:0] lane_mpr_bit_match [0:BYTE_LANES-1];
    wire [BYTE_LANES-1:0] lane_mpr_word_match;
`ifdef SIM_NATIVE_DIAG_ROTATED_MPR_EYE
    wire [7:0] lane_mpr_rotation_match [0:BYTE_LANES-1];
`endif
    generate
        genvar mpr_lane, mpr_bit;
        for (mpr_lane = 0; mpr_lane < BYTE_LANES;
             mpr_lane = mpr_lane + 1) begin : gen_mpr_lane
            for (mpr_bit = 0; mpr_bit < DQ_BITS;
                 mpr_bit = mpr_bit + 1) begin : gen_mpr_bit
                assign lane_mpr_bit_match[mpr_lane][mpr_bit] =
                    (iserdes_dq_q[mpr_lane * DQ_BITS + mpr_bit] ===
                     mpr_expected_pattern_q);
            end
            assign lane_mpr_word_match[mpr_lane] =
                &lane_mpr_bit_match[mpr_lane];
`ifdef SIM_NATIVE_DIAG_ROTATED_MPR_EYE
            for (genvar mpr_rotation = 0; mpr_rotation < 8;
                 mpr_rotation = mpr_rotation + 1) begin : gen_mpr_rotation
                wire [DQ_BITS-1:0] rotation_bit_match;
                for (genvar mpr_rotation_bit = 0;
                     mpr_rotation_bit < DQ_BITS;
                     mpr_rotation_bit = mpr_rotation_bit + 1) begin : gen_rotation_bit
                    localparam integer MPR_ROT_IDX =
                        mpr_lane * DQ_BITS + mpr_rotation_bit;
                    assign rotation_bit_match[mpr_rotation_bit] =
                        (({iserdes_dq_q[MPR_ROT_IDX],
                           iserdes_dq_q[MPR_ROT_IDX]} >> mpr_rotation) ===
                         mpr_expected_pattern_q);
                end
                assign lane_mpr_rotation_match[mpr_lane][mpr_rotation] =
                    &rotation_bit_match;
            end
`endif
        end
    endgenerate

`ifdef SIM_NATIVE_DIAG_FINAL_EYE_RESET_ALIGN
    wire [BYTE_LANES-1:0] eye_final_align_match =
        calibration_fifo_word_valid_q & lane_mpr_word_match;
    wire [BYTE_LANES-1:0] eye_final_align_seen_after =
        eye_final_align_seen | eye_final_align_match;
`endif

    // BISC preserves the individual DQ Align_Delay baselines, while the eye
    // offset is common to the byte.  Require the intersection of all eight DQ
    // eyes so a marginal bit cannot be hidden by using DQ0 alone.
`ifdef SIM_NATIVE_DIAG_ROTATED_MPR_EYE
    wire pattern_found_comb = |lane_mpr_rotation_match[train_lane];
    wire [3:0] pattern_offset_comb =
        mpr_rotation_offset(lane_mpr_rotation_match[train_lane]);
    wire selected_pattern_match = pattern_found_comb &&
        (pattern_offset_comb == best_offset);
`else
    wire pattern_found_comb = lane_mpr_word_match[train_lane];
    wire [3:0] pattern_offset_comb = pattern_found_comb ? 4'd0 : 4'd8;
    wire selected_pattern_match = pattern_found_comb;
`endif
    wire eye_observe_match = eye_observe_verify ?
        selected_pattern_match : pattern_found_comb;

    // Break the native Q-to-training-control path at a local data register.
    // Qualify match and offset together; the observation FSM below adds one
    // clock so the last comparison result is visible before it decides.
    always @(posedge i_controller_clk) begin
        if (sync_rst || (phy_state != PHY_EYE_OBSERVE)) begin
            eye_observe_match_q  <= 1'b0;
            eye_observe_offset_q <= 4'd8;
            eye_observe_bit_match_q <= {DQ_BITS{1'b0}};
        end else begin
            eye_observe_match_q <=
                calibration_fifo_word_valid_q[train_lane] &&
                eye_observe_match;
            eye_observe_offset_q <= pattern_offset_comb;
            eye_observe_bit_match_q <=
                {DQ_BITS{calibration_fifo_word_valid_q[train_lane]}} &
                lane_mpr_bit_match[train_lane];
        end
    end

    // This MPR observation is diagnostic during gate training. It must not
    // qualify GT_STATUS because the data FIFO can update after the native gate
    // detector has already closed its valid window. Eye training consumes it.
    wire [BYTE_LANES-1:0] gate_pattern_found;
    generate
        genvar gate_mpr_lane;
        for (gate_mpr_lane = 0; gate_mpr_lane < BYTE_LANES;
             gate_mpr_lane = gate_mpr_lane + 1) begin : gen_gate_mpr_ref
            // DQ[0] is the per-byte timing reference, as in the component
            // PHY. Requiring every untrained DQ bit here would make DQS-gate
            // placement depend on the worst DQ input-delay edge.
            assign gate_pattern_found[gate_mpr_lane] =
                mpr_rotation_match(
                    iserdes_dq_q[gate_mpr_lane * DQ_BITS]);
        end
    endgenerate

    // -----------------------------------------------------------------
    // Application Read Return
    //
    // FIFO_EMPTY is synchronized into DIV_CLK and FIFO_RD_EN is registered.
    // Q therefore holds the selected head on the edge that advances the read
    // pointer. Capture the complete word and its valid indication together.
    // -----------------------------------------------------------------
`ifdef SIM_NATIVE_DIAG_SIMPLE_APP_RETURN
    integer app_rd_lane, app_rd_bit, app_rd_phase, app_rd_idx;
`endif
    always @(posedge i_controller_clk) begin
        if (sync_rst) begin
            o_dfi_rddata       <= {(SERDES_RATIO*DFI_DATA_WIDTH){1'b0}};
            o_dfi_rddata_valid <= {SERDES_RATIO{1'b0}};
        end else begin
            o_dfi_rddata_valid <= {SERDES_RATIO{1'b0}};
            if (app_pop_fire) begin
`ifdef SIM_NATIVE_DIAG_SIMPLE_APP_RETURN
                // RXTX_BITSLICE Q is first-word-present.  On the registered
                // pop edge, capture the selected head before the primitive
                // advances to the following FIFO word.
                for (app_rd_lane = 0; app_rd_lane < BYTE_LANES;
                     app_rd_lane = app_rd_lane + 1) begin
                    for (app_rd_bit = 0; app_rd_bit < DQ_BITS;
                         app_rd_bit = app_rd_bit + 1) begin
                        app_rd_idx = app_rd_lane * DQ_BITS + app_rd_bit;
                        for (app_rd_phase = 0;
                             app_rd_phase < SERDES_RATIO;
                             app_rd_phase = app_rd_phase + 1) begin
                            o_dfi_rddata[
                                app_rd_phase*DFI_DATA_WIDTH + app_rd_idx
                            ] <= aligned_dq[app_rd_idx][2*app_rd_phase];
                            o_dfi_rddata[
                                app_rd_phase*DFI_DATA_WIDTH + TOTAL_DQ +
                                app_rd_idx
                            ] <= aligned_dq[app_rd_idx][2*app_rd_phase + 1];
                        end
                    end
                end
`else
                o_dfi_rddata <= app_return_fifo[app_return_rd_ptr];
`endif
                o_dfi_rddata_valid <= {SERDES_RATIO{1'b1}};
            end
        end
    end

`ifdef SIM_NATIVE_RX_DEBUG
    // Simulation diagnostics are observational only.  Keeping them outside
    // the functional FSM makes the synthesized ownership boundaries clear.
    always @(posedge i_controller_clk) begin
        if (!sync_rst) begin
            if ((phy_state == PHY_WL_DONE) &&
                (wl_handoff_phase == WL_HANDOFF_DRAIN) &&
                (vtc_settle_counter == 0)) begin
                $display("[%0t] NATIVE_RX_HANDOFF: timer=%0d vtc=%0d empty=%h seen=%h rden=%h pop=%b valid=%b",
                    $realtime, phy_timer, vtc_settle_counter, fifo_empty_flat,
                    wl_handoff_empty_seen, wl_handoff_fifo_rd_en_q,
                    calibration_fifo_pop_q,
                    calibration_fifo_word_valid_q);
            end
            if (app_account_mode && (|phy_rden_ready)) begin
                $display("[%0t] NATIVE_RX_GATE: rden=%b pipe=%h mCL=%0d cmd=%b",
                    $realtime, phy_rden_ready,
                    read_gate_pipe[0][23:0], active_read_mcl,
                    dfi_read_command);
            end
            if (app_account_mode &&
                ((|dfi_read_command) || dfi_read_expected)) begin
                $display("[%0t] NATIVE_RX_CMD: cmd=%b expected=%b app=%0b cal_req=%0b cal_session=%0b flush=%0d discard=%0d",
                    $realtime, dfi_read_command, i_dfi_rddata_en,
                    app_mode, calibration_request, calibration_session,
                    rx_fifo_flush_count, rx_write_discard_count);
            end
            if (app_read_issued || app_fifo_pop_fire || app_pop_fire ||
                app_returns_queued) begin
                $display("[%0t] NATIVE_RX: issue=%0b due_evt=%0b pending=%0d due=%0d after=%0d pop_req=%0b inflight=%0b fresh=%0b empty_seen=%0b capture=%0b rd_fire=%0b q_valid=%0b rden0=%h empty0=%h q_last=%h q_lane0=%h dfi_v=%b dfi=%h",
                    $realtime, app_read_issued, app_read_due_event,
                    app_read_pending, app_read_due,
                    app_pending_after_fifo_pop,
                    app_fifo_pop_request, app_fifo_pop_inflight,
                    app_fifo_fresh_q, app_fifo_empty_seen_q,
                    app_capture_active,
                    app_fifo_pop_fire,
                    app_pop_fire,
                    fifo_rd_en_drive[0], fifo_empty[0],
                    rx_dq_data[BYTE_LANES-1], rx_dq_data[0],
                    o_dfi_rddata_valid, o_dfi_rddata);
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
                wl_tap[dfi_pack_idx]           <= 9'b0;
                wl_dq_tap[dfi_pack_idx]        <= 9'b0;
                wl_coarse[dfi_pack_idx]        <= 4'b0;
                wl_seen_zero[dfi_pack_idx]     <= 1'b0;
                dqs_initial_tap[dfi_pack_idx]  <= 9'b0;
                eye_best_width[dfi_pack_idx]   <= 9'b0;
                eye_best_start[dfi_pack_idx]   <= 9'b0;
                gate_cur_start[dfi_pack_idx]   <= 9'b0;
                gate_cur_width[dfi_pack_idx]   <= 9'b0;
                gate_best_start[dfi_pack_idx]  <= 9'b0;
                gate_best_width[dfi_pack_idx]  <= 9'b0;
                gate_center[dfi_pack_idx]      <= 9'b0;
                gate_cur_start_low[dfi_pack_idx]  <= 9'b0;
                gate_cur_width_low[dfi_pack_idx]  <= 9'b0;
                gate_best_start_low[dfi_pack_idx] <= 9'b0;
                gate_best_width_low[dfi_pack_idx] <= 9'b0;
                gate_center_low[dfi_pack_idx]     <= 9'b0;
                gate_trained_mcl[dfi_pack_idx] <=
                    NATIVE_GATE_MCL_INITIAL;
                gate_trained_mcl_low[dfi_pack_idx] <=
                    NATIVE_GATE_MCL_INITIAL;
                gate_trained_coarse[dfi_pack_idx] <= 4'd0;
                gate_trained_coarse_low[dfi_pack_idx] <= 4'd0;
                app_read_mcl_upper[dfi_pack_idx] <= NATIVE_CL_NCK;
                app_read_mcl_lower[dfi_pack_idx] <= NATIVE_CL_NCK;
                app_read_coarse_target[dfi_pack_idx] <= 4'd0;
            end
            app_read_mcl <= NATIVE_CL_NCK;
            gate_mcl_consensus_valid <= 1'b0;
            gate_mcl_consensus_lower <= 6'd7;
            gate_mcl_consensus_upper <= 6'd63;
            gate_consensus_active <= 1'b0;
            gate_consensus_phase <= GATE_CONS_LANE;
            gate_consensus_lane <= 0;
            gate_consensus_lower_work <= 6'd7;
            gate_consensus_upper_work <= 6'd63;
            gate_consensus_lane_lower_q <= 6'd7;
            gate_consensus_lane_upper_q <= 6'd63;
            gate_consensus_target_q <= 9'sd0;
            gate_consensus_first_target <= 4'd0;
            phy_state           <= PHY_IDLE;
            train_lane          <= 0;
            phy_timer           <= 4'b0;
            idelay_cntvalue     <= 9'b0;
            idelay_cntvalue_per_dq <= {(DQ_BITS*9){1'b0}};
            idelay_per_dq_mode  <= 1'b0;
            sweep_tap           <= 9'b0;
            cur_start           <= 9'b0;
            cur_width           <= 9'b0;
            in_range            <= 1'b0;
            best_start          <= 9'b0;
            best_width          <= 9'b0;
            best_valid          <= 1'b0;
            pattern_found_q     <= 1'b0;
            pattern_offset_q    <= 4'd8;
            cur_offset          <= 4'd8;
            best_offset         <= 4'd8;
            eye_reference_offset <= 4'd0;
            eye_observe_verify  <= 1'b0;
            eye_observe_count   <= 4'b0;
            eye_observe_seen    <= 1'b0;
            eye_observe_bit_seen <= {DQ_BITS{1'b0}};
            eye_prime_discard   <= 1'b1;
            eye_observe_offset  <= 4'd8;
            eye_verify_retries  <= 2'b0;
            eye_mcl_base        <= 6'd7;
            eye_mcl_base_low    <= 6'd7;
            eye_mcl_upper_index <= 3'd0;
            eye_mcl_index       <= 3'd0;
            eye_gate_raw_mcl    <= 6'd7;
            eye_gate_raw_coarse <= 4'd0;
            eye_gate_phase_retry <= 2'd0;
            eye_gate_retry_program_pending <= 1'b0;
            eye_gate_retry_programmed <= 1'b0;
            eye_gate_retry_vtc_ready_count <= 4'd0;
            eye_gate_rearm_all <= 1'b0;
            eye_bit_in_range <= {DQ_BITS{1'b0}};
            eye_bit_best_valid <= {DQ_BITS{1'b0}};
            eye_decide_phase <= 2'd0;
            eye_reduce_index <= 4'd0;
            eye_reduce_all_valid <= 1'b0;
            eye_reduce_min_width <= 9'd0;
            for (eye_bit_track_idx = 0;
                 eye_bit_track_idx < DQ_BITS;
                 eye_bit_track_idx = eye_bit_track_idx + 1) begin
                eye_bit_cur_start[eye_bit_track_idx] <= 9'd0;
                eye_bit_cur_width[eye_bit_track_idx] <= 9'd0;
                eye_bit_best_start[eye_bit_track_idx] <= 9'd0;
                eye_bit_best_width[eye_bit_track_idx] <= 9'd0;
            end
            for (eye_bit_track_idx = 0;
                 eye_bit_track_idx < TOTAL_DQ;
                 eye_bit_track_idx = eye_bit_track_idx + 1)
                eye_bit_center_tap[eye_bit_track_idx] <= 9'd0;
            rd_lat_extra        <= {BYTE_LANES{1'b0}};
            odelay_dqs_cntvalue <= 9'b0;
            wl_dqs_strobe       <= 1'b0;
            wl_prime_pending    <= 1'b1;
            wl_capture_pulse_count <= 3'd0;
            wl_word_wait_count     <= 5'd0;
            // Global BITSLICE reset already initializes every native FIFO.
            // This additional reset belongs exclusively to a write-leveling
            // primer; keeping it asserted here would suppress gate/eye data.
            wl_fifo_reset_q     <= 1'b0;
`ifdef SIM_NATIVE_DIAG_WL_DQS_RX_GUARD
            wl_dqs_rx_armed     <= 1'b0;
            wl_dqs_release_guard <= 3'd0;
`endif
            wl_feedback_clear   <= 1'b1;
            wl_mixed_retries    <= 2'd0;
            wl_sweep_coarse     <= 4'b0;
            wl_riu_phase        <= WL_RIU_PROGRAM;
            wl_handoff_phase    <= WL_HANDOFF_CLEAR;
            wl_handoff_gate_coarse <= 4'd0;
            wl_handoff_gate_tap <= 9'd0;
            eye_gate_finalize_phase <= EYE_GATE_REWIND;
            eye_gate_finalize_coarse <= 4'd0;
            eye_gate_finalize_tap <= 9'd0;
            eye_gate_finalize_started <= 1'b0;
            eye_gate_finalize_target_coarse_q <= 4'd0;
            eye_boundary_retime_done <= 1'b0;
`ifdef SIM_NATIVE_DIAG_FINAL_EYE_RESET_ALIGN
            eye_final_align_active <= 1'b0;
            eye_final_align_phase <= EYE_FINAL_ALIGN_RESET;
            eye_final_align_seen <= {BYTE_LANES{1'b0}};
`endif
            wl_handoff_empty_seen <= {TOTAL_DQ{1'b0}};
            wl_handoff_nonempty_seen <= {TOTAL_DQ{1'b0}};
            en_vtc_q            <= 1'b1;
            // The reset sequencer holds RX/TX EN_VTC High through BISC. Keep
            // TIME-mode maintenance enabled until eye training deliberately
            // opens a legal VAR_LOAD update session.
            bitslice_en_vtc_q   <= 1'b1;
            eye_vtc_ready_count <= 4'd0;
            vtc_settle_counter  <= 8'b0;
            gate_train_fail     <= {BYTE_LANES{1'b0}};
            eye_train_fail      <= {BYTE_LANES{1'b0}};
            wl_train_fail       <= {BYTE_LANES{1'b0}};
            gate_phase          <= GATE_WRITE_ALL;
            gate_sweep_tap      <= 9'd0;
            gate_sweep_mcl      <= NATIVE_GATE_MCL_INITIAL;
            gate_sweep_coarse   <= 4'd0;
            gate_mcl_index      <= 3'd0;
            gate_advance_mcl    <= 1'b0;
            gate_lane_resolved  <= {BYTE_LANES{1'b0}};
            gate_lane_resolved_low <= {BYTE_LANES{1'b0}};
            gate_restore_tap    <= 9'd0;
            gate_restore_lower  <= 1'b0;
            gate_observe_count  <= 4'd0;
            gate_fresh_seen     <= {BYTE_LANES{1'b0}};
            gate_fresh_seen_low <= {BYTE_LANES{1'b0}};
            gate_match_seen     <= {BYTE_LANES{1'b0}};
            gate_status_seen    <= {BYTE_LANES{1'b0}};
            gate_status_seen_low <= {BYTE_LANES{1'b0}};
            gate_in_range       <= {BYTE_LANES{1'b0}};
            gate_in_range_low   <= {BYTE_LANES{1'b0}};
            gate_best_valid     <= {BYTE_LANES{1'b0}};
            gate_best_valid_low <= {BYTE_LANES{1'b0}};
            native_riu_addr     <= 6'd0;
            native_riu_wr_data  <= 16'd0;
            native_riu_wr_en    <= 1'b0;
            native_riu_lower_sel <= {BYTE_LANES{1'b0}};
            native_riu_sel      <= {BYTE_LANES{1'b0}};
            rx_fifo_flush_count <= 5'd0;
            gate_capture_enable <= 1'b0;
        end else begin
            // This block is the sole owner of the calibration flush timer.
            // Gate/eye state assignments later in this block have priority
            // over the normal countdown.  Do not reset RX when write
            // leveling ends: DQS is stopped at that boundary, while RX_RST
            // deassertion is synchronized to received DQS.  Holding RX_RST
            // until the first application read would discard that burst's
            // leading transfer.  Write-level feedback is consumed and
            // drained inside the WL state machine itself.
            if (rx_fifo_flush_count != 0)
                rx_fifo_flush_count <= rx_fifo_flush_count - 1'b1;

            // Default: deassert all LOAD pulses (single-cycle pulse)
            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                idelay_load_lane[dfi_pack_idx] <= 1'b0;
                odelay_dqs_load[dfi_pack_idx]  <= 1'b0;
            end
            wl_dqs_strobe <= 1'b0;
            wl_feedback_clear <= 1'b0;
            native_riu_wr_en <= 1'b0;
            native_riu_lower_sel <= {BYTE_LANES{1'b0}};
            native_riu_sel <= {BYTE_LANES{1'b0}};

            // ---------------------------------------------------------
            // PHY Training FSM
            // ---------------------------------------------------------
            case (phy_state)
                    PHY_IDLE: begin
                        if (!i_dfi_rdlvl_en)
                            eye_vtc_ready_count <= 4'd0;
                        if (i_dfi_rdlvl_gate_en) begin
                            // Native RX gating accepts DQS only inside the
                            // command-timed PHY_RDEN window. Sweep the complete
                            // rank-0 RL_DLY coarse/fine range while the
                            // controller supplies MPR reads; the eye stage then
                            // validates the selected command cycle against
                            // actual MPR data.
                            en_vtc_q <= 1'b0;
                            bitslice_en_vtc_q <= 1'b1;
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}};
                            gate_train_fail <= {BYTE_LANES{1'b0}};
                            gate_sweep_tap <= 9'd0;
                            gate_sweep_mcl <= NATIVE_GATE_MCL_INITIAL;
                            gate_sweep_coarse <= 4'd0;
                            gate_mcl_index <= 3'd0;
                            gate_advance_mcl <= 1'b0;
                            gate_lane_resolved <= {BYTE_LANES{1'b0}};
                            gate_lane_resolved_low <= {BYTE_LANES{1'b0}};
                            gate_observe_count <= 4'd0;
                            gate_fresh_seen <= {BYTE_LANES{1'b0}};
                            gate_fresh_seen_low <= {BYTE_LANES{1'b0}};
                            gate_match_seen <= {BYTE_LANES{1'b0}};
                            gate_status_seen <= {BYTE_LANES{1'b0}};
                            gate_status_seen_low <= {BYTE_LANES{1'b0}};
                            gate_capture_enable <= 1'b0;
                            gate_in_range <= {BYTE_LANES{1'b0}};
                            gate_in_range_low <= {BYTE_LANES{1'b0}};
                            gate_best_valid <= {BYTE_LANES{1'b0}};
                            gate_best_valid_low <= {BYTE_LANES{1'b0}};
                            gate_restore_lower <= 1'b0;
                            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES;
                                 dfi_pack_idx = dfi_pack_idx + 1) begin
                                gate_cur_start[dfi_pack_idx] <= 9'd0;
                                gate_cur_width[dfi_pack_idx] <= 9'd0;
                                gate_best_start[dfi_pack_idx] <= 9'd0;
                                gate_best_width[dfi_pack_idx] <= 9'd0;
                                gate_center[dfi_pack_idx] <= 9'd0;
                                gate_cur_start_low[dfi_pack_idx] <= 9'd0;
                                gate_cur_width_low[dfi_pack_idx] <= 9'd0;
                                gate_best_start_low[dfi_pack_idx] <= 9'd0;
                                gate_best_width_low[dfi_pack_idx] <= 9'd0;
                                gate_center_low[dfi_pack_idx] <= 9'd0;
                                gate_trained_mcl[dfi_pack_idx] <=
                                    NATIVE_GATE_MCL_INITIAL;
                                gate_trained_mcl_low[dfi_pack_idx] <=
                                    NATIVE_GATE_MCL_INITIAL;
                                gate_trained_coarse[dfi_pack_idx] <= 4'd0;
                                gate_trained_coarse_low[dfi_pack_idx] <= 4'd0;
                            end
                            gate_phase <= GATE_WRITE_ALL;
                            phy_state <= PHY_GATE_DONE;
                        end else if (i_dfi_rdlvl_en) begin
                            // Gate-RIU programming pauses BITSLICE_CONTROL VTC.
                            // Restore maintenance and wait for every physical
                            // nibble before opening the DQ VAR_LOAD session.
                            // This makes the EN_VTC falling edge a reliable
                            // per-bit Align_Delay capture point.
                            en_vtc_q <= 1'b1;
                            bitslice_en_vtc_q <= 1'b1;
                            // DLY_RDY/VTC_RDY can remain High for a few fabric
                            // clocks while a preceding RX reset or RIU update
                            // is still completing.  Require a full, consecutive
                            // ready interval with every TIME-mode bit slice held
                            // in VTC maintenance before lowering EN_VTC.  This
                            // prevents the UNISIM-only continuation case where
                            // hardware BISC would remain incomplete.
                            if (!(all_dly_rdy && all_vtc_rdy)) begin
                                eye_vtc_ready_count <= 4'd0;
                            end else if (eye_vtc_ready_count != 4'hf) begin
                                eye_vtc_ready_count <=
                                    eye_vtc_ready_count + 1'b1;
                            end else begin
                                en_vtc_q <= 1'b0;
                                bitslice_en_vtc_q <= 1'b0;
                                eye_vtc_ready_count <= 4'd0;
                                o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}};
                                eye_gate_finalize_started <= 1'b0;
                                eye_gate_finalize_phase <= EYE_GATE_REWIND;
                                eye_boundary_retime_done <= 1'b0;
                                train_lane <= 0;
                                sweep_tap <= 9'd0;
                                idelay_cntvalue <= 9'd0;
                                idelay_cntvalue_per_dq <=
                                    {(DQ_BITS*9){1'b0}};
                                idelay_per_dq_mode <= 1'b0;
                                eye_train_fail <= {BYTE_LANES{1'b0}};
                                in_range <= 1'b0;
                                best_valid <= 1'b0;
                                best_width <= 9'd0;
                                cur_width <= 9'd0;
                                pattern_offset_q <= 4'd8;
                                cur_offset <= 4'd8;
                                best_offset <= 4'd8;
                                eye_reference_offset <= 4'd0;
                                eye_observe_verify <= 1'b0;
                                eye_observe_count <= 4'b0;
                                eye_observe_seen <= 1'b0;
                                eye_observe_bit_seen <= {DQ_BITS{1'b0}};
                                eye_observe_offset <= 4'd8;
                                eye_bit_in_range <= {DQ_BITS{1'b0}};
                                eye_bit_best_valid <= {DQ_BITS{1'b0}};
                                for (eye_bit_track_idx = 0;
                                     eye_bit_track_idx < DQ_BITS;
                                     eye_bit_track_idx =
                                         eye_bit_track_idx + 1) begin
                                    eye_bit_cur_start[eye_bit_track_idx] <=
                                        9'd0;
                                    eye_bit_cur_width[eye_bit_track_idx] <=
                                        9'd0;
                                    eye_bit_best_start[eye_bit_track_idx] <=
                                        9'd0;
                                    eye_bit_best_width[eye_bit_track_idx] <=
                                        9'd0;
                                end
                                eye_mcl_base <= native_eye_initial_upper_mcl(
                                    gate_trained_mcl[0]);
                                eye_mcl_base_low <= gate_trained_mcl_low[0];
                                gate_trained_mcl[0] <=
                                    native_eye_initial_upper_mcl(
                                        gate_trained_mcl[0]);
                                eye_mcl_upper_index <= 3'd0;
                                eye_mcl_index <= 3'd0;
                                eye_gate_raw_mcl <= gate_trained_mcl[0];
                                eye_gate_raw_coarse <= gate_trained_coarse[0];
                                eye_gate_phase_retry <= 2'd0;
                                eye_gate_retry_program_pending <= 1'b0;
                                eye_gate_retry_programmed <= 1'b0;
                                eye_gate_retry_vtc_ready_count <= 4'd0;
                                // UG571 requires at least ten RX_CLK cycles
                                // with EN_VTC Low before the first TIME/VAR_LOAD
                                // update. LOAD occurs when this timer reaches
                                // three, providing twelve complete cycles.
                                phy_timer <= 4'd15;
                                phy_state <= PHY_EYE_REWIND;
                            end
                        end else if (i_dfi_wrlvl_en) begin
`ifdef SIM_NATIVE_DIAG_SKIP_WL_ACTIVITY
                            // Diagnostic only: acknowledge the controller's
                            // write-leveling session without emitting DQS or
                            // touching native delay/FIFO state.  This isolates
                            // whether WL activity disturbs the read word phase
                            // established by gate and eye training.
                            o_dfi_wrlvl_resp <= {BYTE_LANES{1'b1}};
                            phy_state <= PHY_WL_CHECK;
`else
                            en_vtc_q <= 1'b0;
                            bitslice_en_vtc_q <= 1'b1;
                            o_dfi_wrlvl_resp <= {BYTE_LANES{1'b0}};
                            train_lane <= 0;
                            wl_train_fail <= {BYTE_LANES{1'b0}};
                            wl_sweep_coarse <= 4'd0;
                            wl_riu_phase <= WL_RIU_PROGRAM;
                            odelay_dqs_cntvalue <= odelay_dqs_cntvalueout[0];
                            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                dqs_initial_tap[dfi_pack_idx] <= odelay_dqs_cntvalueout[dfi_pack_idx];
                                wl_tap[dfi_pack_idx]    <= 9'd0;
                                wl_dq_tap[dfi_pack_idx] <= 9'd0;
                                wl_coarse[dfi_pack_idx] <= 4'd0;
                                wl_seen_zero[dfi_pack_idx] <= 1'b0;
                            end
                            wl_prime_pending <= 1'b1;
                            wl_capture_pulse_count <= 3'd0;
                            wl_word_wait_count <= 5'd0;
                            wl_fifo_reset_q <= 1'b0;
`ifdef SIM_NATIVE_DIAG_WL_DQS_RX_GUARD
                            // TBYTE takes ownership while the DQS input is
                            // forced Low.  RIU programming and its settle
                            // timer provide several complete DIV_CLK cycles
                            // before the receiver is opened.
                            wl_dqs_rx_armed <= 1'b0;
                            wl_dqs_release_guard <= 3'd0;
`endif
                            wl_feedback_clear <= 1'b1;
                            wl_mixed_retries <= 2'd0;
                            phy_timer <= 4'd4;
                            phy_state <= PHY_WL_SAMPLE;
`endif
                        end
                    end

                    PHY_GATE_DONE: begin
                        case (gate_phase)
                            GATE_WRITE_ALL: begin
                                // Program the complete native read-gate delay:
                                // coarse half-PLL-clock phase plus fine taps.
                                // Each byte contains two independently
                                // controlled nibbles.  The upper nibble owns
                                // DQS and forwards its PCLK/NCLK to the lower
                                // nibble, but both BITSLICE_CONTROL instances
                                // still apply their own RL_DLY_RNK0.  Keep the
                                // pair coherent, as required by MIG's separate
                                // low/upper native read paths.
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_wr_data <= {3'd0,
                                                       gate_sweep_coarse,
                                                       gate_sweep_tap};
                                native_riu_wr_en <= 1'b1;
                                // DQS is attached to the upper control.  The
                                // lower control imports its PCLK/NCLK and must
                                // retain the BISC-established gate delay.
                                native_riu_lower_sel <= {BYTE_LANES{1'b0}};
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
                                         (native_riu_rd_data[0][12:9] ==
                                          gate_sweep_coarse) &&
                                         (native_riu_rd_data[0][8:0] ==
                                          gate_sweep_tap)) begin
                                    // RX_RST clears only the deserializer/FIFO
                                    // pointers; RX_RST_DLY remains deasserted,
                                    // so the candidate delay just programmed
                                    // above is preserved.  This prevents a
                                    // clipped burst from candidate N being
                                    // completed by candidate N+1.
                                    rx_fifo_flush_count <= 5'd8;
                                    gate_phase <= GATE_CANDIDATE_FLUSH;
                                end
                                else begin
`ifdef SIM_NATIVE_RIU_DEBUG
                                    $display("[%0t] NATIVE_RIU_WAIT: tap=%0d valid=%b rd0=%h",
                                        $realtime, gate_sweep_tap, native_riu_valid,
                                        native_riu_rd_data[0]);
`endif
                                    phy_timer <= 4'd15;
                                end
                            end

                            GATE_CANDIDATE_FLUSH: begin
                                if (rx_fifo_flush_count == 0)
                                    gate_phase <= GATE_CLEAR;
                            end

                            GATE_CLEAR: begin
                                // Clear the edge monitor while preserving
                                // RX/TX gating and each nibble's clock source.
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_wr_data <= RIU_GATE_CLEAR;
                                native_riu_wr_en <= 1'b1;
                                native_riu_lower_sel <= {BYTE_LANES{1'b0}};
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
                                native_riu_lower_sel <= {BYTE_LANES{1'b0}};
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                gate_fresh_seen <= {BYTE_LANES{1'b0}};
                                gate_fresh_seen_low <= {BYTE_LANES{1'b0}};
                                gate_match_seen <= {BYTE_LANES{1'b0}};
                                gate_status_seen <= {BYTE_LANES{1'b0}};
                                gate_status_seen_low <= {BYTE_LANES{1'b0}};
                                phy_timer <= 4'd15;
                                gate_phase <= GATE_WAIT_RELEASE;
                            end

                            GATE_WAIT_RELEASE: begin
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (&native_riu_valid) begin
                                    // Arm only the next training READ.  The
                                    // controller spaces calibration commands
                                    // far enough apart that dfi_read_expected
                                    // is observed before another READ can be
                                    // inserted into the native gate pipeline.
                                    gate_capture_enable <= 1'b1;
                                    gate_phase <= GATE_WAIT_READ;
                                end else begin
                                    phy_timer <= 4'd15;
                                end
                            end

                            GATE_WAIT_READ: begin
                                // Keep NIBBLE_CTRL0 selected throughout the
                                // physical DQS interval.  GT_STATUS is live,
                                // not a latched transaction response; the
                                // byte-local RIU logic accumulates it until
                                // this candidate's next CLR_GATE write.
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                // A short-board byte can complete its FIFO
                                // word before the controller's predicted
                                // rddata_en phase.  The candidate was already
                                // cleared in GATE_RELEASE_CLEAR, so retain any
                                // per-byte event observed while waiting rather
                                // than erasing it when the common DFI marker
                                // arrives.
                                for (dfi_pack_idx = 0;
                                     dfi_pack_idx < BYTE_LANES;
                                     dfi_pack_idx = dfi_pack_idx + 1) begin
                                    gate_fresh_seen[dfi_pack_idx] <=
                                        gate_fresh_seen[dfi_pack_idx] |
                                        calibration_fifo_pop_q[dfi_pack_idx];
                                    if (calibration_fifo_word_valid_q[
                                            dfi_pack_idx] &&
                                        gate_pattern_found[dfi_pack_idx])
                                        gate_match_seen[dfi_pack_idx] <= 1'b1;
                                end
                                if (dfi_read_expected) begin
                                    gate_capture_enable <= 1'b0;
                                    gate_observe_count <= 4'd0;
                                    gate_phase <= GATE_OBSERVE;
                                end
                            end

                            GATE_OBSERVE: begin
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                    gate_fresh_seen[dfi_pack_idx] <=
                                        gate_fresh_seen[dfi_pack_idx] |
                                        calibration_fifo_pop_q[dfi_pack_idx];
                                    if (calibration_fifo_word_valid_q[
                                            dfi_pack_idx] &&
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
                                    // GT_STATUS is the native DQS/gate phase
                                    // training result.  Qualify it separately
                                    // from the all-DQ FIFO event: DQ IDELAY is
                                    // deliberately still untrained here, so an
                                    // exact MPR comparison would make gate
                                    // success depend on an arbitrary DQ eye
                                    // edge.  The following read-eye stage owns
                                    // the exact MPR-data qualification.
                                    for (dfi_pack_idx = 0;
                                         dfi_pack_idx < BYTE_LANES;
                                         dfi_pack_idx = dfi_pack_idx + 1)
                                        gate_status_seen[dfi_pack_idx] <=
                                            native_riu_gate_status_sticky[
                                                dfi_pack_idx];
                                    `ifndef YOSYS
                                    `ifdef SIM_QUIET_TRAINING_LOG
                                        // Keep a full 512-tap calibration visible
                                        // in long regressions without the I/O
                                        // cost of one line per candidate.
                                if (gate_sweep_tap[3:0] == 4'd0)
                                            $display("[%0t] PHY native gate sweep: mCL %0d tap %0d / %0d",
                                                $realtime, gate_sweep_mcl,
                                                gate_sweep_tap,
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
                                    if (!gate_lane_resolved[dfi_pack_idx]) begin
                                        if (gate_candidate_valid[dfi_pack_idx]) begin
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
                                            // period, so later taps may expose
                                            // a periodic copy. Keep the first
                                            // complete window for this mCL.
                                            if (!gate_best_valid[dfi_pack_idx] &&
                                                (gate_cur_width[dfi_pack_idx] >=
                                                 GATE_MIN_WINDOW_TAPS)) begin
                                                gate_best_start[dfi_pack_idx] <= gate_cur_start[dfi_pack_idx];
                                                gate_best_width[dfi_pack_idx] <= gate_cur_width[dfi_pack_idx];
                                                gate_best_valid[dfi_pack_idx] <= 1'b1;
                                            end
                                            gate_in_range[dfi_pack_idx] <= 1'b0;
                                        end
                                    end
                                    if (!gate_lane_resolved_low[dfi_pack_idx]) begin
                                        if (gate_candidate_valid_low[dfi_pack_idx]) begin
                                            if (!gate_in_range_low[dfi_pack_idx]) begin
                                                gate_cur_start_low[dfi_pack_idx] <= gate_sweep_tap;
                                                gate_cur_width_low[dfi_pack_idx] <= GATE_TAP_STEP;
                                                gate_in_range_low[dfi_pack_idx] <= 1'b1;
                                            end else begin
                                                gate_cur_width_low[dfi_pack_idx] <=
                                                    gate_cur_width_low[dfi_pack_idx] +
                                                    GATE_TAP_STEP;
                                            end
                                        end else if (gate_in_range_low[dfi_pack_idx]) begin
                                            if (!gate_best_valid_low[dfi_pack_idx] &&
                                                (gate_cur_width_low[dfi_pack_idx] >=
                                                 GATE_MIN_WINDOW_TAPS)) begin
                                                gate_best_start_low[dfi_pack_idx] <=
                                                    gate_cur_start_low[dfi_pack_idx];
                                                gate_best_width_low[dfi_pack_idx] <=
                                                    gate_cur_width_low[dfi_pack_idx];
                                                gate_best_valid_low[dfi_pack_idx] <= 1'b1;
                                            end
                                            gate_in_range_low[dfi_pack_idx] <= 1'b0;
                                        end
                                    end
                                end

                                if (gate_all_first_windows_complete ||
                                    (gate_sweep_tap >= GATE_SWEEP_LAST)) begin
                                    gate_phase <= GATE_FINALIZE;
                                end else begin
                                    gate_sweep_tap <= gate_sweep_tap + GATE_TAP_STEP;
                                    gate_phase <= GATE_WRITE_ALL;
                                end
                            end

                            GATE_FINALIZE: begin
                                // Close a range that reaches tap 511, then
                                // commit this candidate on the following cycle.
                                for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                    if (!gate_lane_resolved[dfi_pack_idx] &&
                                        gate_in_range[dfi_pack_idx] &&
                                        !gate_best_valid[dfi_pack_idx] &&
                                        (gate_cur_width[dfi_pack_idx] >=
                                         GATE_MIN_WINDOW_TAPS)) begin
                                        gate_best_start[dfi_pack_idx] <= gate_cur_start[dfi_pack_idx];
                                        gate_best_width[dfi_pack_idx] <= gate_cur_width[dfi_pack_idx];
                                        gate_best_valid[dfi_pack_idx] <= 1'b1;
                                    end
                                    if (!gate_lane_resolved_low[dfi_pack_idx] &&
                                        gate_in_range_low[dfi_pack_idx] &&
                                        !gate_best_valid_low[dfi_pack_idx] &&
                                        (gate_cur_width_low[dfi_pack_idx] >=
                                         GATE_MIN_WINDOW_TAPS)) begin
                                        gate_best_start_low[dfi_pack_idx] <=
                                            gate_cur_start_low[dfi_pack_idx];
                                        gate_best_width_low[dfi_pack_idx] <=
                                            gate_cur_width_low[dfi_pack_idx];
                                        gate_best_valid_low[dfi_pack_idx] <= 1'b1;
                                    end
                                end
                                gate_phase <= GATE_COMMIT_MCL;
                            end

                            GATE_COMMIT_MCL: begin
                                // A complete fine window establishes this
                                // byte's native gate phase. Preserve its mCL,
                                // coarse, and fine tuple independently so
                                // lanes on opposite coarse boundaries do not
                                // force a board-specific common setting. The
                                // following eye stage validates the command-
                                // cycle component against actual MPR data.
                                for (dfi_pack_idx = 0;
                                     dfi_pack_idx < BYTE_LANES;
                                     dfi_pack_idx = dfi_pack_idx + 1) begin
                                    if (!gate_lane_resolved[dfi_pack_idx] &&
                                        gate_best_valid[dfi_pack_idx]) begin
                                        // Center the selected stable region so
                                        // PVT drift has margin in both
                                        // directions.  The following exact-MPR
                                        // eye verification resolves any
                                        // adjacent-cycle ambiguity.
                                        gate_center[dfi_pack_idx] <=
                                            gate_best_start[dfi_pack_idx] +
                                            (gate_best_width[dfi_pack_idx] >> 1);
                                        gate_trained_mcl[dfi_pack_idx] <=
                                            gate_sweep_mcl;
`ifdef SIM_NATIVE_DIAG_GATE_RESTORE_MINUS2_PRE_EYE
                                        // Diagnostic boundary trial: move the
                                        // selected DQS gate one DDR transfer
                                        // earlier while preserving the fine
                                        // center found by the complete sweep.
                                        // Four coarse codes span one tCK, so
                                        // crossing code zero is represented by
                                        // the preceding command-mask cycle and
                                        // a +4-code wrap.
                                        gate_trained_mcl[dfi_pack_idx] <=
                                            (gate_sweep_coarse < 4'd2) ?
                                            ((gate_sweep_mcl > 6'd7) ?
                                             gate_sweep_mcl - 6'd1 : 6'd7) :
                                            gate_sweep_mcl;
                                        gate_trained_coarse[dfi_pack_idx] <=
                                            (gate_sweep_coarse < 4'd2) ?
                                         gate_sweep_coarse + 4'd2 :
                                         gate_sweep_coarse - 4'd2;
`elsif SIM_NATIVE_DIAG_GATE_BOUNDARY_ALIGN_PRE_EYE
                                        // RX_GATING consumes the first DQS
                                        // preamble edge, but the native 1:8
                                        // FIFO boundary is established one UI
                                        // later.  Represent the required
                                        // boundary move with the equivalent
                                        // XiPHY coordinates: advance the
                                        // command mask by one tCK and add two
                                        // RL_DLY coarse codes (one UI).  Eye
                                        // training below validates this final
                                        // tuple using actual MPR data.
                                        gate_trained_mcl[dfi_pack_idx] <=
                                            (gate_sweep_mcl > 6'd7) ?
                                            gate_sweep_mcl - 1'b1 : 6'd7;
                                        gate_trained_coarse[dfi_pack_idx] <=
                                            gate_sweep_coarse + 4'd2;
`elsif SIM_NATIVE_DIAG_GATE_RESTORE_PLUS2_PRE_EYE
                                        // Diagnostic boundary trial: discard
                                        // the final read-preamble edge by
                                        // advancing the selected DQS gate one
                                        // DDR UI before DQ eye training. UG571
                                        // defines one RL_DLY_CRSE code as half
                                        // a PLL_CLK period; CLKOUTPHY is one
                                        // DDR UI here, so one UI is two coarse
                                        // codes. At coarse 14/15, move PHY_RDEN
                                        // one tCK later (+4 codes) and subtract
                                        // two codes, preserving the same net
                                        // +2-code displacement without wrap.
                                        gate_trained_mcl[dfi_pack_idx] <=
                                            gate_sweep_mcl +
                                            ((gate_sweep_coarse >= 4'd14) ?
                                             6'd1 : 6'd0);
                                        gate_trained_coarse[dfi_pack_idx] <=
                                            (gate_sweep_coarse >= 4'd14) ?
                                            gate_sweep_coarse - 4'd2 :
                                            gate_sweep_coarse + 4'd2;
`elsif SIM_NATIVE_DIAG_GATE_RESTORE_PLUS1_PRE_EYE
                                        gate_trained_mcl[dfi_pack_idx] <=
                                            gate_sweep_mcl +
                                            ((gate_sweep_coarse == 4'd15) ?
                                             6'd1 : 6'd0);
                                        gate_trained_coarse[dfi_pack_idx] <=
                                            (gate_sweep_coarse == 4'd15) ?
                                            4'd12 :
                                            gate_sweep_coarse + 4'd1;
`else
                                        gate_trained_coarse[dfi_pack_idx] <=
                                            gate_sweep_coarse;
`endif
                                        gate_lane_resolved[dfi_pack_idx] <=
                                            1'b1;
                                    end
                                    if (!gate_lane_resolved_low[dfi_pack_idx] &&
                                        gate_best_valid_low[dfi_pack_idx]) begin
                                        gate_center_low[dfi_pack_idx] <=
                                            gate_best_start_low[dfi_pack_idx] +
                                            (gate_best_width_low[dfi_pack_idx] >> 1);
                                        gate_trained_mcl_low[dfi_pack_idx] <=
                                            gate_sweep_mcl;
`ifdef SIM_NATIVE_DIAG_GATE_RESTORE_MINUS2_PRE_EYE
                                        gate_trained_mcl_low[dfi_pack_idx] <=
                                            (gate_sweep_coarse < 4'd2) ?
                                            ((gate_sweep_mcl > 6'd7) ?
                                             gate_sweep_mcl - 6'd1 : 6'd7) :
                                            gate_sweep_mcl;
                                        gate_trained_coarse_low[dfi_pack_idx] <=
                                            (gate_sweep_coarse < 4'd2) ?
                                             gate_sweep_coarse + 4'd2 :
                                             gate_sweep_coarse - 4'd2;
`elsif SIM_NATIVE_DIAG_GATE_BOUNDARY_ALIGN_PRE_EYE
                                        gate_trained_mcl_low[dfi_pack_idx] <=
                                            (gate_sweep_mcl > 6'd7) ?
                                            gate_sweep_mcl - 1'b1 : 6'd7;
                                        gate_trained_coarse_low[dfi_pack_idx] <=
                                            gate_sweep_coarse + 4'd2;
`elsif SIM_NATIVE_DIAG_GATE_RESTORE_PLUS2_PRE_EYE
                                        gate_trained_mcl_low[dfi_pack_idx] <=
                                            gate_sweep_mcl +
                                            ((gate_sweep_coarse >= 4'd14) ?
                                             6'd1 : 6'd0);
                                        gate_trained_coarse_low[dfi_pack_idx] <=
                                            (gate_sweep_coarse >= 4'd14) ?
                                            gate_sweep_coarse - 4'd2 :
                                            gate_sweep_coarse + 4'd2;
`elsif SIM_NATIVE_DIAG_GATE_RESTORE_PLUS1_PRE_EYE
                                        gate_trained_mcl_low[dfi_pack_idx] <=
                                            gate_sweep_mcl +
                                            ((gate_sweep_coarse == 4'd15) ?
                                             6'd1 : 6'd0);
                                        gate_trained_coarse_low[dfi_pack_idx] <=
                                            (gate_sweep_coarse == 4'd15) ?
                                            4'd12 :
                                            gate_sweep_coarse + 4'd1;
`else
                                        gate_trained_coarse_low[dfi_pack_idx] <=
                                            gate_sweep_coarse;
`endif
                                        gate_lane_resolved_low[dfi_pack_idx] <=
                                            1'b1;
                                    end
                                end

                                if ((&gate_resolved_after_candidate) &&
                                    (&gate_resolved_after_candidate_low)) begin
                                    train_lane <= 0;
                                    gate_restore_tap <= gate_sweep_tap;
                                    gate_restore_lower <= 1'b0;
                                    gate_phase <= GATE_WRITE_LANE;
                                end else if ((gate_sweep_coarse <
                                              NATIVE_GATE_COARSE_CANDIDATES - 1) ||
                                             (gate_mcl_index <
                                              NATIVE_GATE_MCL_CANDIDATES - 1)) begin
                                    // UG571 limits one native delay update to
                                    // eight taps. Rewind every byte legally
                                    // before incrementing RL_DLY_CRSE or moving
                                    // the command-timed mask by one full tCK.
                                    gate_advance_mcl <=
                                        gate_sweep_coarse >=
                                        NATIVE_GATE_COARSE_CANDIDATES - 1;
                                    gate_restore_tap <= gate_sweep_tap;
                                    gate_phase <= GATE_REWIND_WRITE;
                                end else begin
                                    // Only after all bounded coarse candidates
                                    // fail is an unresolved byte reported bad.
                                    for (dfi_pack_idx = 0;
                                         dfi_pack_idx < BYTE_LANES;
                                         dfi_pack_idx = dfi_pack_idx + 1) begin
                                        if (!gate_resolved_after_candidate[
                                                dfi_pack_idx]) begin
                                            gate_train_fail[dfi_pack_idx] <= 1'b1;
                                            gate_center[dfi_pack_idx] <= 9'd0;
                                            gate_trained_mcl[dfi_pack_idx] <=
                                                NATIVE_GATE_MCL_INITIAL;
                                            gate_trained_coarse[dfi_pack_idx] <=
                                                4'd0;
                                        end
                                        if (!gate_resolved_after_candidate_low[
                                                dfi_pack_idx]) begin
                                            gate_train_fail[dfi_pack_idx] <= 1'b1;
                                            gate_center_low[dfi_pack_idx] <= 9'd0;
                                            gate_trained_mcl_low[dfi_pack_idx] <=
                                                NATIVE_GATE_MCL_INITIAL;
                                            gate_trained_coarse_low[dfi_pack_idx] <=
                                                4'd0;
                                        end
                                    end
                                    train_lane <= 0;
                                    gate_restore_tap <= gate_sweep_tap;
                                    gate_restore_lower <= 1'b0;
                                    gate_phase <= GATE_WRITE_LANE;
                                end
                            end

                            GATE_REWIND_WRITE: begin
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_wr_data <= {3'd0,
                                                       gate_sweep_coarse,
                                                       gate_rewind_next};
                                gate_restore_tap <= gate_rewind_next;
                                native_riu_wr_en <= 1'b1;
                                native_riu_lower_sel <= {BYTE_LANES{1'b0}};
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                phy_timer <= 4'd15;
                                gate_phase <= GATE_REWIND_WAIT;
                            end

                            GATE_REWIND_WAIT: begin
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_sel <= {BYTE_LANES{1'b1}};
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if ((&native_riu_valid) &&
                                             (native_riu_rd_data[0][12:9] ==
                                              gate_sweep_coarse) &&
                                             (native_riu_rd_data[0][8:0] ==
                                              gate_restore_tap)) begin
                                    if (gate_restore_tap != 9'd0) begin
                                        gate_phase <= GATE_REWIND_WRITE;
                                    end else begin
                                        if (gate_advance_mcl) begin
                                            gate_mcl_index <=
                                                gate_mcl_index + 1'b1;
                                            gate_sweep_mcl <=
                                                native_gate_mcl_candidate(
                                                    gate_mcl_index + 1'b1);
                                            gate_sweep_coarse <= 4'd0;
                                            gate_advance_mcl <= 1'b0;
                                        end else begin
                                            gate_sweep_coarse <=
                                                gate_sweep_coarse + 1'b1;
                                        end
                                        gate_sweep_tap <= 9'd0;
                                        gate_in_range <=
                                            {BYTE_LANES{1'b0}};
                                        gate_in_range_low <=
                                            {BYTE_LANES{1'b0}};
                                        for (dfi_pack_idx = 0;
                                             dfi_pack_idx < BYTE_LANES;
                                             dfi_pack_idx = dfi_pack_idx + 1) begin
                                            if (!gate_lane_resolved[
                                                    dfi_pack_idx]) begin
                                                gate_cur_start[dfi_pack_idx] <= 9'd0;
                                                gate_cur_width[dfi_pack_idx] <= 9'd0;
                                                gate_best_start[dfi_pack_idx] <= 9'd0;
                                                gate_best_width[dfi_pack_idx] <= 9'd0;
                                                gate_best_valid[dfi_pack_idx] <= 1'b0;
                                            end
                                            if (!gate_lane_resolved_low[
                                                    dfi_pack_idx]) begin
                                                gate_cur_start_low[dfi_pack_idx] <= 9'd0;
                                                gate_cur_width_low[dfi_pack_idx] <= 9'd0;
                                                gate_best_start_low[dfi_pack_idx] <= 9'd0;
                                                gate_best_width_low[dfi_pack_idx] <= 9'd0;
                                                gate_best_valid_low[dfi_pack_idx] <= 1'b0;
                                            end
                                        end
                                        gate_phase <= GATE_WRITE_ALL;
                                    end
                                end else begin
                                    phy_timer <= 4'd15;
                                end
                            end

                            GATE_WRITE_LANE: begin
                                // Program the selected fine delay back into the
                                // byte currently being finalized. Use the same
                                // bounded eight-tap update policy as the native
                                // delay controls; avoiding a 511-to-low jump
                                // keeps the gate state and FIFO phase stable.
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_wr_data <= {3'd0,
                                                       gate_target_coarse,
                                                       gate_restore_next};
                                gate_restore_tap <= gate_restore_next;
                                native_riu_wr_en <= 1'b1;
                                native_riu_lower_sel <= {BYTE_LANES{1'b0}};
                                native_riu_sel <= train_lane_mask;
                                phy_timer <= 4'd15;
                                gate_phase <= GATE_WAIT_LANE;
                            end

                            GATE_WAIT_LANE: begin
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_lower_sel <= {BYTE_LANES{1'b0}};
                                native_riu_sel <= train_lane_mask;
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (native_riu_valid[train_lane] &&
                                             (native_riu_rd_data[train_lane][12:9] ==
                                              gate_target_coarse) &&
                                             (native_riu_rd_data[train_lane][8:0] ==
                                              gate_restore_tap)) begin
                                    if (gate_restore_tap !=
                                        gate_target_tap) begin
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
                                native_riu_lower_sel <= {BYTE_LANES{1'b0}};
                                native_riu_sel <= train_lane_mask;
                                phy_timer <= 4'd15;
                                gate_phase <= GATE_RESTORE_WAIT_CLEAR;
                            end

                            GATE_RESTORE_WAIT_CLEAR: begin
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_lower_sel <= {BYTE_LANES{1'b0}};
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
                                native_riu_lower_sel <= {BYTE_LANES{1'b0}};
                                native_riu_sel <= train_lane_mask;
                                phy_timer <= 4'd15;
                                gate_phase <= GATE_RESTORE_WAIT_RELEASE;
                            end

                            GATE_RESTORE_WAIT_RELEASE: begin
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_lower_sel <= {BYTE_LANES{1'b0}};
                                native_riu_sel <= train_lane_mask;
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (native_riu_valid[train_lane]) begin
`ifndef YOSYS
                                    $display("[%0t] PHY gate: lane %0d start_tap=%0d width=%0d center_tap=%0d mCL=%0d coarse=%0d fail=%0b",
                                        $realtime, train_lane,
                                        gate_best_start[train_lane],
                                        gate_best_width[train_lane],
                                        gate_center[train_lane],
                                        gate_trained_mcl[train_lane],
                                        gate_trained_coarse[train_lane],
                                        gate_train_fail[train_lane]);
`endif
                                    if (eye_gate_rearm_all) begin
                                        // Re-arm each measured byte gate in
                                        // turn without reopening VTC or
                                        // changing its trained delay tuple.
                                        if (train_lane < BYTE_LANES - 1) begin
                                            train_lane <= train_lane + 1'b1;
                                            gate_restore_lower <= 1'b0;
                                            gate_restore_tap <= gate_center[
                                                train_lane + 1'b1];
                                            gate_phase <= GATE_WRITE_LANE;
                                        end else begin
                                            eye_gate_rearm_all <= 1'b0;
                                            train_lane <= 0;
                                            eye_boundary_retime_done <= 1'b1;
                                            eye_verify_retries <= 2'd0;
                                            pattern_found_q <= 1'b0;
                                            phy_timer <= 4'd0;
                                            phy_state <= PHY_EYE_VERIFY;
                                        end
                                    end else if (eye_gate_retry_program_pending) begin
                                        // Eye qualification requested an
                                        // adjacent half-UI gate for this byte.
                                        // The acknowledged RL_DLY write plus
                                        // CLR_GATE/RUN sequence above has now
                                        // established that physical phase.
                                        // Reopen BISC/VTC maintenance and do
                                        // not resume the eye until every
                                        // nibble has reported stable ready and
                                        // a new Align_Delay baseline is taken.
                                        eye_gate_retry_programmed <= 1'b1;
                                        eye_gate_retry_vtc_ready_count <= 4'd0;
                                        en_vtc_q <= 1'b1;
                                        bitslice_en_vtc_q <= 1'b1;
                                        rx_fifo_flush_count <= 5'd16;
                                        phy_timer <= 4'd0;
                                        phy_state <= PHY_EYE_REWIND;
                                    end else if (train_lane < BYTE_LANES - 1) begin
                                        train_lane <= train_lane + 1'b1;
                                        gate_restore_lower <= 1'b0;
                                        gate_restore_tap <= gate_sweep_tap;
                                        gate_phase <= GATE_WRITE_LANE;
                                    end else begin
                                        gate_restore_lower <= 1'b0;
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
                                    bitslice_en_vtc_q <= 1'b1;
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
                            if (phy_timer == 4'd3) begin
                                idelay_load_lane[train_lane] <= 1'b1;
                                eye_prime_discard <= 1'b1;
                            end
                            phy_timer <= phy_timer - 1'b1;
                        end else if (|i_dfi_rddata_en) begin
                            eye_observe_verify <= 1'b0;
                            eye_observe_count <= 4'b0;
                            eye_observe_seen <= 1'b0;
                            eye_observe_bit_seen <= {DQ_BITS{1'b0}};
                            eye_observe_offset <= 4'd8;
                            phy_state <= PHY_EYE_OBSERVE;
                        end
                    end

                    // Observe the complete native FIFO-return window. The
                    // first valid match and its cyclic phase are retained so
                    // preamble/postamble X values cannot overwrite them.
                    PHY_EYE_OBSERVE: begin
`ifdef SIM_NATIVE_DIAG_MPR_ALTERNATE
                        if (calibration_fifo_word_valid_q[train_lane])
                            $display("[%0t] NATIVE_MPR_ALT: lane=%0d tap=%0d page=%0b exp=%h raw0=%h match=%0b",
                                $realtime, train_lane, sweep_tap,
                                mpr_alt_page_q, mpr_expected_pattern_q,
                                iserdes_dq_q[train_lane * DQ_BITS],
                                eye_observe_match);
`endif
                        if (eye_observe_match_q && !eye_observe_seen) begin
                            eye_observe_seen <= 1'b1;
                            eye_observe_offset <= eye_observe_offset_q;
                        end
                        eye_observe_bit_seen <= eye_observe_bit_seen |
                            eye_observe_bit_match_q;

                        if (eye_observe_count == NATIVE_RX_OBSERVE_CYCLES) begin
`ifdef SIM_NATIVE_RIU_DEBUG
                            $display("[%0t] NATIVE_EYE_SAMPLE: lane=%0d tap=%0d verify=%0b seen=%0b now=%0b off=%0d empty=%h cur=%h lane_words=%h",
                                $realtime, train_lane, sweep_tap,
                                eye_observe_verify, eye_observe_seen,
                                eye_observe_match_q,
                                eye_observe_offset_q,
                                fifo_empty[train_lane],
                                iserdes_dq_q[train_lane * DQ_BITS],
                                rx_dq_data[train_lane]);
`endif
                            if (eye_prime_discard) begin
                                // The asynchronous native FIFO can still
                                // present the word written by the preceding
                                // delay candidate.  Consume exactly one
                                // command-associated word before measuring the
                                // candidate that is now physically loaded.
                                eye_prime_discard <= 1'b0;
                                eye_observe_seen <= 1'b0;
                                eye_observe_bit_seen <= {DQ_BITS{1'b0}};
                                eye_observe_offset <= 4'd8;
                                phy_state <= eye_observe_verify ?
                                    PHY_EYE_VERIFY : PHY_EYE_SWEEP;
                            end else
                            if (eye_observe_verify) begin
                                if (eye_observe_seen |
                                    eye_observe_match_q) begin
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
                                                   eye_observe_match_q;
                                pattern_offset_q <= eye_observe_seen ?
                                    eye_observe_offset :
                                    eye_observe_offset_q;
                                phy_state <= PHY_EYE_TRACK;
                            end
                        end else begin
                            eye_observe_count <= eye_observe_count + 1'b1;
                        end
                    end

                    PHY_EYE_TRACK: begin
                        for (eye_bit_track_idx = 0;
                             eye_bit_track_idx < DQ_BITS;
                             eye_bit_track_idx = eye_bit_track_idx + 1) begin
                            if (eye_observe_bit_seen[eye_bit_track_idx] |
                                eye_observe_bit_match_q[
                                    eye_bit_track_idx]) begin
                                if (!eye_bit_in_range[eye_bit_track_idx]) begin
                                    eye_bit_cur_start[eye_bit_track_idx] <=
                                        sweep_tap;
                                    eye_bit_cur_width[eye_bit_track_idx] <=
                                        9'd0;
                                    eye_bit_in_range[eye_bit_track_idx] <=
                                        1'b1;
                                end else begin
                                    eye_bit_cur_width[eye_bit_track_idx] <=
                                        eye_bit_cur_width[
                                            eye_bit_track_idx] +
                                        {5'd0, TAP_SWEEP_STEP};
                                end
                            end else if (eye_bit_in_range[
                                             eye_bit_track_idx]) begin
                                if (!eye_bit_best_valid[
                                         eye_bit_track_idx] ||
                                    (eye_bit_cur_width[
                                         eye_bit_track_idx] >
                                     eye_bit_best_width[
                                         eye_bit_track_idx])) begin
                                    eye_bit_best_start[eye_bit_track_idx] <=
                                        eye_bit_cur_start[
                                            eye_bit_track_idx];
                                    eye_bit_best_width[eye_bit_track_idx] <=
                                        eye_bit_cur_width[
                                            eye_bit_track_idx];
                                    eye_bit_best_valid[eye_bit_track_idx] <=
                                        1'b1;
                                end
                                eye_bit_in_range[eye_bit_track_idx] <=
                                    1'b0;
                            end
                        end
                        if (pattern_found_q) begin
                            if (!in_range) begin
                                cur_start <= sweep_tap;
                                cur_width <= 9'd0;
                                cur_offset <= pattern_offset_q;
                                in_range <= 1'b1;
                            end else if (pattern_offset_q == cur_offset) begin
                                cur_width <= cur_width + {5'd0, TAP_SWEEP_STEP};
                            end else begin
                                // A phase transition is a digital FIFO-word
                                // boundary, not additional analog-eye width.
                                // Close the old range and start a new one at
                                // this tap with the newly observed rotation.
                                if (!best_valid || cur_width > best_width) begin
                                    best_start <= cur_start;
                                    best_width <= cur_width;
                                    best_offset <= cur_offset;
                                    best_valid <= 1'b1;
                                end
                                cur_start <= sweep_tap;
                                cur_width <= 9'd0;
                                cur_offset <= pattern_offset_q;
                            end
                        end else begin
                            if (in_range) begin
                                if (!best_valid || cur_width > best_width) begin
                                    best_start <= cur_start;
                                    best_width <= cur_width;
                                    best_offset <= cur_offset;
                                    best_valid <= 1'b1;
                                end
                                in_range <= 1'b0;
                            end
                        end
                        if ((sweep_tap >= EYE_SWEEP_LAST) ||
                            (({1'b0, sweep_tap} +
                              {6'd0, TAP_SWEEP_STEP}) >
                             {1'b0, rx_max_relative_offset[train_lane]})) begin
                            eye_decide_phase <= 2'd0;
                            phy_state <= PHY_EYE_DECIDE;
                        end else begin
                            sweep_tap <= sweep_tap + {5'd0, TAP_SWEEP_STEP};
                            idelay_cntvalue <= sweep_tap + {5'd0, TAP_SWEEP_STEP};
                            phy_timer <= 4'd4;
                            phy_state <= PHY_EYE_SWEEP;
                        end
                    end

                    PHY_EYE_DECIDE: begin
                        // Close every per-bit range before evaluating the
                        // candidate.  Then reduce one DQ per clock instead of
                        // placing an eight-way minimum cascade on a 1/4-rate
                        // controller path.  Calibration latency is immaterial,
                        // while every registered width remains authoritative.
                        if (eye_decide_phase == 2'd0) begin
                            if (in_range || (|eye_bit_in_range)) begin
                            if (in_range) begin
                                if (!best_valid || cur_width > best_width) begin
                                    best_start <= cur_start;
                                    best_width <= cur_width;
                                    best_offset <= cur_offset;
                                    best_valid <= 1'b1;
                                end
                                in_range <= 1'b0;
                            end
                            for (eye_bit_track_idx = 0;
                                 eye_bit_track_idx < DQ_BITS;
                                 eye_bit_track_idx =
                                     eye_bit_track_idx + 1) begin
                                if (eye_bit_in_range[eye_bit_track_idx]) begin
                                    if (!eye_bit_best_valid[
                                             eye_bit_track_idx] ||
                                        (eye_bit_cur_width[
                                             eye_bit_track_idx] >
                                         eye_bit_best_width[
                                             eye_bit_track_idx])) begin
                                        eye_bit_best_start[
                                            eye_bit_track_idx] <=
                                            eye_bit_cur_start[
                                                eye_bit_track_idx];
                                        eye_bit_best_width[
                                            eye_bit_track_idx] <=
                                            eye_bit_cur_width[
                                                eye_bit_track_idx];
                                        eye_bit_best_valid[
                                            eye_bit_track_idx] <= 1'b1;
                                    end
                                    eye_bit_in_range[
                                        eye_bit_track_idx] <= 1'b0;
                                end
                            end
                            end else begin
                                eye_reduce_index <= 4'd0;
                                eye_reduce_all_valid <= 1'b1;
                                eye_reduce_min_width <= 9'h1ff;
                                eye_decide_phase <= 2'd1;
                            end
                        end else if (eye_decide_phase == 2'd1) begin
                            eye_reduce_all_valid <= eye_reduce_all_valid &
                                eye_bit_best_valid[eye_reduce_index];
                            if (eye_bit_best_width[eye_reduce_index] <
                                eye_reduce_min_width)
                                eye_reduce_min_width <=
                                    eye_bit_best_width[eye_reduce_index];
                            if (eye_reduce_index == DQ_BITS - 1)
                                eye_decide_phase <= 2'd2;
                            else
                                eye_reduce_index <= eye_reduce_index + 1'b1;
                        end else if (!eye_reduce_all_valid ||
                                     (eye_reduce_min_width <
                                      EYE_MIN_WINDOW_TAPS)) begin
                            if (eye_mcl_has_next) begin
                                // The analog gate is valid, but this periodic
                                // command-cycle copy did not return MPR data.
                                // Keep the trained analog RL_DLY phase and try
                                // the next bounded byte-wide PHY_RDEN point.
                                eye_mcl_upper_index <=
                                    eye_mcl_next_upper_index;
                                eye_mcl_index <= eye_mcl_next_lower_index;
                                gate_trained_mcl[train_lane] <=
                                    eye_mcl_next_upper;
                                gate_trained_mcl_low[train_lane] <=
                                    eye_mcl_next_lower;
                                idelay_cntvalue <= delay_step_toward(
                                    idelay_cntvalue, 9'd0);
                                // Changing the command-cycle gate while FIFO
                                // state is retained can splice the tail of the
                                // rejected candidate into the next MPR word.
                                // Reset only RX datapath/FIFO state; the
                                // trained RL_DLY and RX delay remain intact.
                                rx_fifo_flush_count <= 5'd16;
                                phy_timer <= 4'd4;
                                phy_state <= PHY_EYE_REWIND;
                                `ifndef YOSYS
                                    $display("[%0t] PHY eye: lane %0d no MPR at upper/lower mCL %0d/%0d; trying %0d/%0d",
                                        $realtime, train_lane,
                                        gate_trained_mcl[train_lane],
                                        gate_trained_mcl_low[train_lane],
                                        eye_mcl_next_upper,
                                        eye_mcl_next_lower);
                                `endif
                            end else if (eye_gate_phase_retry < 2) begin
                                // All command-mask candidates at this physical
                                // gate were absent or too narrow to tolerate
                                // PVT drift.  Retry only this byte at the two
                                // adjacent half-UI RL_DLY phases.  This resolves
                                // the modulo-8 FIFO ambiguity using measured
                                // MPR width instead of a board-specific offset.
                                eye_gate_phase_retry <= eye_gate_next_retry;
                                gate_trained_mcl[train_lane] <=
                                    eye_gate_retry_mcl;
                                gate_trained_mcl_low[train_lane] <=
                                    eye_gate_retry_mcl;
                                gate_trained_coarse[train_lane] <=
                                    eye_gate_retry_coarse;
                                gate_trained_coarse_low[train_lane] <=
                                    eye_gate_retry_coarse;
                                eye_mcl_base <= eye_gate_retry_mcl;
                                eye_mcl_base_low <= eye_gate_retry_mcl;
                                eye_mcl_upper_index <= 3'd0;
                                eye_mcl_index <= 3'd0;
                                eye_gate_retry_program_pending <= 1'b1;
                                eye_gate_retry_programmed <= 1'b0;
                                eye_gate_retry_vtc_ready_count <= 4'd0;
                                idelay_cntvalue <= delay_step_toward(
                                    idelay_cntvalue, 9'd0);
                                rx_fifo_flush_count <= 5'd16;
                                phy_timer <= 4'd4;
                                phy_state <= PHY_EYE_REWIND;
`ifndef YOSYS
                                $display("[%0t] PHY eye: lane %0d rejected narrow gate phase (width=%0d); retry %0d at mCL=%0d coarse=%0d",
                                    $realtime, train_lane, best_width,
                                    eye_gate_next_retry, eye_gate_retry_mcl,
                                    eye_gate_retry_coarse);
`endif
                            end else begin
                                eye_train_fail[train_lane] <= 1'b1;
                                eye_center_tap[train_lane] <= 9'd0;
                                eye_best_width[train_lane] <= 9'd0;
                                eye_best_start[train_lane] <= 9'd0;
                                bitslip_count_q[train_lane] <= 4'd0;
                                if (train_lane < BYTE_LANES - 1) begin
                                    train_lane <= train_lane + 1'b1;
                                    eye_mcl_base <=
                                        native_eye_initial_upper_mcl(
                                            gate_trained_mcl[
                                                train_lane + 1'b1]);
                                    eye_mcl_base_low <= gate_trained_mcl_low[
                                        train_lane + 1'b1];
                                    gate_trained_mcl[train_lane + 1'b1] <=
                                        native_eye_initial_upper_mcl(
                                            gate_trained_mcl[
                                                train_lane + 1'b1]);
                                    eye_mcl_upper_index <= 3'd0;
                                    eye_mcl_index <= 3'd0;
                                    eye_gate_raw_mcl <=
                                        gate_trained_mcl[train_lane + 1'b1];
                                    eye_gate_raw_coarse <=
                                        gate_trained_coarse[train_lane + 1'b1];
                                    eye_gate_phase_retry <= 2'd0;
                                    eye_gate_retry_program_pending <= 1'b0;
                                    eye_gate_retry_programmed <= 1'b0;
                                    eye_gate_retry_vtc_ready_count <= 4'd0;
                                    sweep_tap <= 9'd0;
                                    idelay_cntvalue <= 9'd0;
                                    idelay_cntvalue_per_dq <=
                                        {(DQ_BITS*9){1'b0}};
                                    idelay_per_dq_mode <= 1'b0;
                                    in_range <= 1'b0;
                                    best_valid <= 1'b0;
                                    best_width <= 9'd0;
                                    cur_width <= 9'd0;
                                    cur_offset <= 4'd8;
                                    best_offset <= 4'd8;
                                    eye_observe_bit_seen <=
                                        {DQ_BITS{1'b0}};
                                    eye_bit_in_range <= {DQ_BITS{1'b0}};
                                    eye_bit_best_valid <= {DQ_BITS{1'b0}};
                                    for (eye_bit_track_idx = 0;
                                         eye_bit_track_idx < DQ_BITS;
                                         eye_bit_track_idx =
                                             eye_bit_track_idx + 1) begin
                                        eye_bit_cur_start[
                                            eye_bit_track_idx] <= 9'd0;
                                        eye_bit_cur_width[
                                            eye_bit_track_idx] <= 9'd0;
                                        eye_bit_best_start[
                                            eye_bit_track_idx] <= 9'd0;
                                        eye_bit_best_width[
                                            eye_bit_track_idx] <= 9'd0;
                                    end
                                    phy_timer <= 4'd4;
                                    phy_state <= PHY_EYE_SWEEP;
                                end else begin
                                    eye_gate_finalize_started <= 1'b0;
                                    eye_gate_finalize_phase <=
                                        EYE_GATE_COMPLETE;
                                    phy_state <= PHY_EYE_DONE;
                                end
                                `ifndef YOSYS
                                    $display("[%0t] PHY eye: lane %0d no valid range across mCL candidates",
                                        $realtime, train_lane);
                                `endif
                            end
                        end else begin
                            // Program the center of every measured DQ window,
                            // not merely the center of their intersection.
                            // Final exact-byte MPR verification below still
                            // rejects any incorrect word framing.
                            eye_center_tap[train_lane] <=
                                eye_bit_best_start[0] +
                                (eye_bit_best_width[0] >> 1);
                            if (train_lane == 0)
                                eye_reference_offset <= 4'd0;
`ifdef SIM_NATIVE_DIAG_ROTATED_MPR_EYE
                            bitslip_count_q[train_lane] <= best_offset;
`else
                            bitslip_count_q[train_lane] <= 4'd0;
`endif
                            rd_lat_extra[train_lane] <= 1'b0;
                            eye_best_width[train_lane] <=
                                eye_reduce_min_width;
                            eye_best_start[train_lane] <=
                                eye_bit_best_start[0];
                            eye_verify_retries <= 2'b0;
                            idelay_per_dq_mode <= 1'b1;
                            for (eye_bit_track_idx = 0;
                                 eye_bit_track_idx < DQ_BITS;
                                 eye_bit_track_idx =
                                     eye_bit_track_idx + 1) begin
                                eye_bit_center_tap[
                                    train_lane*DQ_BITS +
                                    eye_bit_track_idx] <=
                                    eye_bit_best_start[
                                        eye_bit_track_idx] +
                                    (eye_bit_best_width[
                                        eye_bit_track_idx] >> 1);
                                idelay_cntvalue_per_dq[
                                    eye_bit_track_idx*9 +: 9] <=
                                    delay_step_toward(
                                        idelay_cntvalue,
                                        eye_bit_best_start[
                                            eye_bit_track_idx] +
                                        (eye_bit_best_width[
                                            eye_bit_track_idx] >> 1));
                            end
                            idelay_cntvalue <= delay_step_toward(
                                idelay_cntvalue,
                                eye_bit_best_start[0] +
                                (eye_bit_best_width[0] >> 1));
                            phy_timer <= 4'd4;
                            phy_state <= PHY_EYE_CENTER;
                            `ifndef YOSYS
                                    $display("[%0t] PHY eye: lane %0d per-DQ minimum width=%0d DQ0 center=%0d",
                                        $realtime, train_lane,
                                        eye_reduce_min_width,
                                    eye_bit_best_start[0] +
                                    (eye_bit_best_width[0] >> 1));
                            `endif
                        end
                    end

                    PHY_EYE_CENTER: begin
                        if (phy_timer != 0) begin
                            if (phy_timer == 4'd3) begin
                                idelay_load_lane[train_lane] <= 1'b1;
                                eye_prime_discard <= 1'b1;
                            end
                            phy_timer <= phy_timer - 1'b1;
                        end else if (!idelay_per_dq_at_center) begin
                            // Every DQ owns a native RX delay element.  Walk all
                            // eight fields in parallel, while respecting the
                            // eight-tap TIME/VAR_LOAD limit of each bit slice.
                            // Five controller clocks separate LOAD pulses and
                            // every CNTVALUEIN field is stable before LOAD.
                            for (eye_bit_track_idx = 0;
                                 eye_bit_track_idx < DQ_BITS;
                                 eye_bit_track_idx =
                                     eye_bit_track_idx + 1) begin
                                idelay_cntvalue_per_dq[
                                    eye_bit_track_idx*9 +: 9] <=
                                    delay_step_toward(
                                        idelay_cntvalue_per_dq[
                                            eye_bit_track_idx*9 +: 9],
                                        eye_bit_center_tap[
                                            train_lane*DQ_BITS +
                                            eye_bit_track_idx]);
                            end
                            // Retain DQ0 in the legacy scalar for CSR/ILA
                            // visibility; per-DQ mode drives the primitives.
                            idelay_cntvalue <= delay_step_toward(
                                idelay_cntvalue_per_dq[0 +: 9],
                                eye_bit_center_tap[train_lane*DQ_BITS]);
                            phy_timer <= 4'd4;
                        end else begin
                            // The final center is already physically loaded.
                            // Wait for the next controller-scheduled MPR read
                            // and verify that the selected eye still matches.
                            phy_state <= PHY_EYE_VERIFY;
                        end
                    end

                    // TIME/VAR_LOAD RX delay updates are limited to eight taps
                    // per LOAD.  Rewind the same physical lane legally before
                    // retrying an adjacent command-cycle candidate.  Lane
                    // handoff does not LOAD the old lane and needs no rewind.
                    PHY_EYE_REWIND: begin
                        if (phy_timer != 0) begin
                            if (phy_timer == 4'd3) begin
                                // A TIME/VAR_LOAD is legal only after every
                                // byte has captured a stable per-bit BISC
                                // baseline.  Hold at the LOAD boundary instead
                                // of assuming a fixed CNTVALUEOUT latency.
                                if (&rx_align_valid)
                                    idelay_load_lane[train_lane] <= 1'b1;
                                if (&rx_align_valid)
                                    eye_prime_discard <= 1'b1;
                            end
                            if ((phy_timer != 4'd3) || (&rx_align_valid))
                                phy_timer <= phy_timer - 1'b1;
                        end else if (idelay_per_dq_mode &&
                                     (|idelay_cntvalue_per_dq)) begin
                            // A rejected centered candidate has independent DQ
                            // offsets installed.  Rewind every field before
                            // returning to the common zero-offset sweep.
                            for (eye_bit_track_idx = 0;
                                 eye_bit_track_idx < DQ_BITS;
                                 eye_bit_track_idx =
                                     eye_bit_track_idx + 1) begin
                                idelay_cntvalue_per_dq[
                                    eye_bit_track_idx*9 +: 9] <=
                                    delay_step_toward(
                                        idelay_cntvalue_per_dq[
                                            eye_bit_track_idx*9 +: 9],
                                        9'd0);
                            end
                            idelay_cntvalue <= delay_step_toward(
                                idelay_cntvalue_per_dq[0 +: 9], 9'd0);
                            phy_timer <= 4'd4;
                        end else if (idelay_per_dq_mode) begin
                            // All per-bit offsets are physically back at zero.
                            // Return to the scalar broadcast used by the sweep.
                            idelay_per_dq_mode <= 1'b0;
                            idelay_cntvalue <= 9'd0;
                            phy_timer <= 4'd4;
                        end else if (idelay_cntvalue != 9'd0) begin
                            idelay_cntvalue <= delay_step_toward(
                                idelay_cntvalue, 9'd0);
                            phy_timer <= 4'd4;
                        end else if ((rx_fifo_flush_count == 0) &&
                                     eye_gate_retry_program_pending &&
                                     !eye_gate_retry_programmed) begin
                            // The old DQ offset and FIFO contents are gone.
                            // Re-enter VTC maintenance and require the same
                            // consecutive-ready qualification used by the
                            // normal gate-to-eye handoff before touching
                            // RL_DLY.  Gate RIU programming itself keeps DQ
                            // EN_VTC Low and BITSLICE_CONTROL EN_VTC High.
                            en_vtc_q <= 1'b1;
                            bitslice_en_vtc_q <= 1'b1;
                            if (!(all_dly_rdy && all_vtc_rdy)) begin
                                eye_gate_retry_vtc_ready_count <= 4'd0;
                            end else if (eye_gate_retry_vtc_ready_count !=
                                         4'hf) begin
                                eye_gate_retry_vtc_ready_count <=
                                    eye_gate_retry_vtc_ready_count + 1'b1;
                            end else begin
                                en_vtc_q <= 1'b0;
                                bitslice_en_vtc_q <= 1'b1;
                                eye_gate_retry_vtc_ready_count <= 4'd0;
                                gate_restore_tap <= gate_center[train_lane];
                                gate_restore_lower <= 1'b0;
                                gate_phase <= GATE_WRITE_LANE;
                                phy_state <= PHY_GATE_DONE;
                            end
                        end else if ((rx_fifo_flush_count == 0) &&
                                     eye_gate_retry_program_pending &&
                                     eye_gate_retry_programmed) begin
                            // RL_DLY is installed.  Let BISC/VTC settle again,
                            // then lower both EN_VTC controls together.  The
                            // byte modules use this qualified falling interval
                            // to capture fresh per-bit TIME-mode baselines.
                            en_vtc_q <= 1'b1;
                            bitslice_en_vtc_q <= 1'b1;
                            if (!(all_dly_rdy && all_vtc_rdy)) begin
                                eye_gate_retry_vtc_ready_count <= 4'd0;
                            end else if (eye_gate_retry_vtc_ready_count !=
                                         4'hf) begin
                                eye_gate_retry_vtc_ready_count <=
                                    eye_gate_retry_vtc_ready_count + 1'b1;
                            end else begin
                                en_vtc_q <= 1'b0;
                                bitslice_en_vtc_q <= 1'b0;
                                eye_gate_retry_vtc_ready_count <= 4'd0;
                                eye_gate_retry_program_pending <= 1'b0;
                                eye_gate_retry_programmed <= 1'b0;
                                // Twelve complete controller clocks elapse
                                // before the zero-offset LOAD below, exceeding
                                // the UG571 ten-RX_CLK minimum with EN_VTC Low.
                                phy_timer <= 4'd15;
                            end
                        end else if (rx_fifo_flush_count == 0) begin
                            sweep_tap <= 9'd0;
                            idelay_cntvalue <= 9'd0;
                            idelay_cntvalue_per_dq <=
                                {(DQ_BITS*9){1'b0}};
                            idelay_per_dq_mode <= 1'b0;
                            in_range <= 1'b0;
                            best_valid <= 1'b0;
                            best_start <= 9'd0;
                            best_width <= 9'd0;
                            cur_start <= 9'd0;
                            cur_width <= 9'd0;
                            pattern_found_q <= 1'b0;
                            pattern_offset_q <= 4'd8;
                            cur_offset <= 4'd8;
                            best_offset <= 4'd8;
                            eye_observe_verify <= 1'b0;
                            eye_observe_seen <= 1'b0;
                            eye_observe_bit_seen <= {DQ_BITS{1'b0}};
                            eye_observe_offset <= 4'd8;
                            eye_bit_in_range <= {DQ_BITS{1'b0}};
                            eye_bit_best_valid <= {DQ_BITS{1'b0}};
                            for (eye_bit_track_idx = 0;
                                 eye_bit_track_idx < DQ_BITS;
                                 eye_bit_track_idx =
                                     eye_bit_track_idx + 1) begin
                                eye_bit_cur_start[eye_bit_track_idx] <= 9'd0;
                                eye_bit_cur_width[eye_bit_track_idx] <= 9'd0;
                                eye_bit_best_start[eye_bit_track_idx] <= 9'd0;
                                eye_bit_best_width[eye_bit_track_idx] <= 9'd0;
                            end
                            eye_verify_retries <= 2'd0;
                            phy_timer <= 4'd4;
                            phy_state <= PHY_EYE_SWEEP;
                        end
                    end

                    PHY_EYE_VERIFY: begin
                        if (eye_boundary_retime_done) begin
                            // The application RL_DLY point is different from
                            // the point used to sweep the analog eye.  Verify
                            // that final physical gate while MPR is still
                            // enabled instead of handing an unobserved FIFO
                            // boundary to normal traffic.  Each lane is checked
                            // with a fresh isolated MPR return below.
                            if (phy_timer != 0) begin
                                phy_timer <= phy_timer - 1'b1;
                            end else if (|i_dfi_rddata_en) begin
                                eye_observe_verify <= 1'b1;
                                eye_observe_count <= 4'b0;
                                eye_observe_seen <= 1'b0;
                                eye_observe_offset <= 4'd8;
                                phy_state <= PHY_EYE_OBSERVE;
                            end
                        end else
                        if (phy_timer != 0) begin
                            if (phy_timer == 4'd3)
                                idelay_load_lane[train_lane] <= 1'b1;
                            phy_timer <= phy_timer - 1'b1;
                        end else if (|i_dfi_rddata_en) begin
                            eye_observe_verify <= 1'b1;
                            eye_observe_count <= 4'b0;
                            eye_observe_seen <= 1'b0;
                            eye_observe_offset <= 4'd8;
                            phy_state <= PHY_EYE_OBSERVE;
                        end
                    end

                    PHY_EYE_VERIFY_DONE: begin
                        if (eye_boundary_retime_done) begin
                            if (!pattern_found_q) begin
                                // A shifted gate is not a valid application
                                // operating point unless it still returns the
                                // exact, unrotated MPR word.  Preserve the
                                // ordinary training failure contract.
                                eye_train_fail[train_lane] <= 1'b1;
                                eye_gate_finalize_started <= 1'b0;
                                eye_gate_finalize_phase <= EYE_GATE_COMPLETE;
                                phy_state <= PHY_EYE_DONE;
                            end else if (train_lane < BYTE_LANES - 1) begin
                                train_lane <= train_lane + 1'b1;
                                eye_verify_retries <= 2'd0;
                                pattern_found_q <= 1'b0;
                                phy_timer <= 4'd0;
                                phy_state <= PHY_EYE_VERIFY;
                            end else begin
                                // Every byte has now consumed and verified one
                                // complete MPR word at the final application
                                // gate.  The calibration reader performed the
                                // legal FIFO pop, so no pre-retime word can be
                                // returned as the first application burst.
                                eye_gate_finalize_started <= 1'b0;
                                eye_gate_finalize_phase <= EYE_GATE_COMPLETE;
                                phy_state <= PHY_EYE_DONE;
                            end
                        end else begin
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
                                eye_mcl_base <=
                                    native_eye_initial_upper_mcl(
                                        gate_trained_mcl[
                                            train_lane + 1'b1]);
                                eye_mcl_base_low <=
                                    gate_trained_mcl_low[train_lane + 1'b1];
                                gate_trained_mcl[train_lane + 1'b1] <=
                                    native_eye_initial_upper_mcl(
                                        gate_trained_mcl[
                                            train_lane + 1'b1]);
                                eye_mcl_upper_index <= 3'd0;
                                eye_mcl_index <= 3'd0;
                                eye_gate_raw_mcl <=
                                    gate_trained_mcl[train_lane + 1'b1];
                                eye_gate_raw_coarse <=
                                    gate_trained_coarse[train_lane + 1'b1];
                                eye_gate_phase_retry <= 2'd0;
                                eye_gate_retry_program_pending <= 1'b0;
                                eye_gate_retry_programmed <= 1'b0;
                                eye_gate_retry_vtc_ready_count <= 4'd0;
                                sweep_tap <= 9'd0;
                                idelay_cntvalue <= 9'd0;
                                idelay_cntvalue_per_dq <=
                                    {(DQ_BITS*9){1'b0}};
                                idelay_per_dq_mode <= 1'b0;
                                in_range <= 1'b0;
                                best_valid <= 1'b0;
                                best_width <= 9'd0;
                                cur_width <= 9'd0;
                                cur_offset <= 4'd8;
                                best_offset <= 4'd8;
                                eye_observe_bit_seen <=
                                    {DQ_BITS{1'b0}};
                                eye_bit_in_range <= {DQ_BITS{1'b0}};
                                eye_bit_best_valid <= {DQ_BITS{1'b0}};
                                for (eye_bit_track_idx = 0;
                                     eye_bit_track_idx < DQ_BITS;
                                     eye_bit_track_idx =
                                         eye_bit_track_idx + 1) begin
                                    eye_bit_cur_start[
                                        eye_bit_track_idx] <= 9'd0;
                                    eye_bit_cur_width[
                                        eye_bit_track_idx] <= 9'd0;
                                    eye_bit_best_start[
                                        eye_bit_track_idx] <= 9'd0;
                                    eye_bit_best_width[
                                        eye_bit_track_idx] <= 9'd0;
                                end
                                phy_timer <= 4'd4;
                                phy_state <= PHY_EYE_SWEEP;
                            end else begin
                                // The last lane has passed exact, unrotated MPR
                                // verification at its trained mask/RL_DLY pair.
                                // Preserve those physical settings.  A former
                                // post-verification +2 coarse retime was not
                                // itself data-verified and moved application Q
                                // by exactly two UI, joining two neighboring
                                // BL8 reads in every FIFO word.
`ifdef SIM_NATIVE_DIAG_FINALIZE_GATE_PLUS2
                                // Directed physical-boundary proof. Advance
                                // RL_DLY by two one-UI coarse steps while MPR
                                // DQS is still active, then exercise normal
                                // application traffic with the same mCL.
                                train_lane <= 0;
                                en_vtc_q <= 1'b0;
                                eye_boundary_retime_done <= 1'b0;
                                eye_gate_finalize_started <= 1'b1;
                                eye_gate_finalize_phase <= EYE_GATE_REWIND;
                                eye_gate_finalize_coarse <=
                                    gate_trained_coarse[0];
                                eye_gate_finalize_tap <= gate_center[0];
                                phy_timer <= 4'd15;
                                phy_state <= PHY_EYE_DONE;
`elsif SIM_NATIVE_DIAG_FINALIZE_GATE_PLUS4
                                // Directed one-tCK boundary proof.  UG571
                                // defines one RL_DLY_CRSE increment as half a
                                // PLL_CLK period.  With PLL_CLK at the DDR
                                // transfer rate, four increments move the RX
                                // word boundary by two UI (one DDR tCK).
                                train_lane <= 0;
                                en_vtc_q <= 1'b0;
                                eye_boundary_retime_done <= 1'b0;
                                eye_gate_finalize_started <= 1'b1;
                                eye_gate_finalize_phase <= EYE_GATE_REWIND;
                                eye_gate_finalize_coarse <=
                                    gate_trained_coarse[0];
                                eye_gate_finalize_tap <= gate_center[0];
                                phy_timer <= 4'd15;
                                phy_state <= PHY_EYE_DONE;
`elsif SIM_NATIVE_DIAG_FINAL_EYE_RESET_ALIGN
                                // MPR remains enabled until rdlvl_resp.  Hold
                                // RX_RST long enough to clear every native FIFO;
                                // subsequent controller-generated MPR READs
                                // provide the source DQS edges for synchronous
                                // reset release and complete 8-UI alignment.
                                rx_fifo_flush_count <= 5'd16;
                                eye_final_align_active <= 1'b1;
                                eye_final_align_phase <= EYE_FINAL_ALIGN_RESET;
                                eye_final_align_seen <= {BYTE_LANES{1'b0}};
                                eye_gate_finalize_started <= 1'b0;
                                phy_state <= PHY_EYE_DONE;
`else
                                train_lane <= 0;
                                // Later byte-lane RIU/VTC transactions can
                                // leave an earlier native DQS gate disarmed even
                                // though its RL_DLY readback is unchanged. Use
                                // the gate restore FSM to replay every saved
                                // tuple and its required CLR_GATE/RUN sequence,
                                // then enter the final all-lane verifier.
                                eye_gate_rearm_all <= 1'b1;
                                eye_boundary_retime_done <= 1'b0;
                                eye_gate_finalize_started <= 1'b0;
                                eye_gate_finalize_phase <= EYE_GATE_COMPLETE;
                                gate_restore_lower <= 1'b0;
                                gate_restore_tap <= gate_center[0];
                                gate_phase <= GATE_WRITE_LANE;
                                en_vtc_q <= 1'b0;
                                bitslice_en_vtc_q <= 1'b0;
                                phy_state <= PHY_GATE_DONE;
`endif
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
                            if (eye_mcl_has_next) begin
                                eye_mcl_upper_index <=
                                    eye_mcl_next_upper_index;
                                eye_mcl_index <= eye_mcl_next_lower_index;
                                gate_trained_mcl[train_lane] <=
                                    eye_mcl_next_upper;
                                gate_trained_mcl_low[train_lane] <=
                                    eye_mcl_next_lower;
                                idelay_cntvalue <= delay_step_toward(
                                    idelay_cntvalue, 9'd0);
                                rx_fifo_flush_count <= 5'd16;
                                phy_timer <= 4'd4;
                                phy_state <= PHY_EYE_REWIND;
                                `ifndef YOSYS
                                    $display("[%0t] PHY eye: lane %0d verify failed at upper/lower mCL %0d/%0d; trying %0d/%0d",
                                        $realtime, train_lane,
                                        gate_trained_mcl[train_lane],
                                        gate_trained_mcl_low[train_lane],
                                        eye_mcl_next_upper,
                                        eye_mcl_next_lower);
                                `endif
                            end else if (eye_gate_phase_retry < 2) begin
                                // The selected center did not reproduce the
                                // exact isolated MPR word.  Move only this byte
                                // to the next adjacent half-UI physical phase
                                // and repeat the complete bounded mCL search.
                                eye_gate_phase_retry <= eye_gate_next_retry;
                                gate_trained_mcl[train_lane] <=
                                    eye_gate_retry_mcl;
                                gate_trained_mcl_low[train_lane] <=
                                    eye_gate_retry_mcl;
                                gate_trained_coarse[train_lane] <=
                                    eye_gate_retry_coarse;
                                gate_trained_coarse_low[train_lane] <=
                                    eye_gate_retry_coarse;
                                eye_mcl_base <= eye_gate_retry_mcl;
                                eye_mcl_base_low <= eye_gate_retry_mcl;
                                eye_mcl_upper_index <= 3'd0;
                                eye_mcl_index <= 3'd0;
                                eye_gate_retry_program_pending <= 1'b1;
                                eye_gate_retry_programmed <= 1'b0;
                                eye_gate_retry_vtc_ready_count <= 4'd0;
                                idelay_cntvalue <= delay_step_toward(
                                    idelay_cntvalue, 9'd0);
                                rx_fifo_flush_count <= 5'd16;
                                phy_timer <= 4'd4;
                                phy_state <= PHY_EYE_REWIND;
`ifndef YOSYS
                                $display("[%0t] PHY eye: lane %0d center verification failed; retry %0d at mCL=%0d coarse=%0d",
                                    $realtime, train_lane,
                                    eye_gate_next_retry, eye_gate_retry_mcl,
                                    eye_gate_retry_coarse);
`endif
                            end else begin
                                eye_train_fail[train_lane] <= 1'b1;
                                eye_center_tap[train_lane] <= 9'd0;
                                eye_best_width[train_lane] <= 9'd0;
                                eye_best_start[train_lane] <= 9'd0;
                                bitslip_count_q[train_lane] <= 4'd0;
                                if (train_lane < BYTE_LANES - 1) begin
                                    train_lane <= train_lane + 1'b1;
                                    eye_mcl_base <=
                                        native_eye_initial_upper_mcl(
                                            gate_trained_mcl[
                                                train_lane + 1'b1]);
                                    eye_mcl_base_low <= gate_trained_mcl_low[
                                        train_lane + 1'b1];
                                    gate_trained_mcl[train_lane + 1'b1] <=
                                        native_eye_initial_upper_mcl(
                                            gate_trained_mcl[
                                                train_lane + 1'b1]);
                                    eye_mcl_upper_index <= 3'd0;
                                    eye_mcl_index <= 3'd0;
                                    eye_gate_raw_mcl <=
                                        gate_trained_mcl[train_lane + 1'b1];
                                    eye_gate_raw_coarse <=
                                        gate_trained_coarse[train_lane + 1'b1];
                                    eye_gate_phase_retry <= 2'd0;
                                    eye_gate_retry_program_pending <= 1'b0;
                                    eye_gate_retry_programmed <= 1'b0;
                                    eye_gate_retry_vtc_ready_count <= 4'd0;
                                    sweep_tap <= 9'd0;
                                    idelay_cntvalue <= 9'd0;
                                    idelay_cntvalue_per_dq <=
                                        {(DQ_BITS*9){1'b0}};
                                    idelay_per_dq_mode <= 1'b0;
                                    in_range <= 1'b0;
                                    best_valid <= 1'b0;
                                    best_width <= 9'd0;
                                    cur_width <= 9'd0;
                                    cur_offset <= 4'd8;
                                    best_offset <= 4'd8;
                                    eye_observe_bit_seen <=
                                        {DQ_BITS{1'b0}};
                                    eye_bit_in_range <= {DQ_BITS{1'b0}};
                                    eye_bit_best_valid <= {DQ_BITS{1'b0}};
                                    for (eye_bit_track_idx = 0;
                                         eye_bit_track_idx < DQ_BITS;
                                         eye_bit_track_idx =
                                             eye_bit_track_idx + 1) begin
                                        eye_bit_cur_start[
                                            eye_bit_track_idx] <= 9'd0;
                                        eye_bit_cur_width[
                                            eye_bit_track_idx] <= 9'd0;
                                        eye_bit_best_start[
                                            eye_bit_track_idx] <= 9'd0;
                                        eye_bit_best_width[
                                            eye_bit_track_idx] <= 9'd0;
                                    end
                                    phy_timer <= 4'd4;
                                    phy_state <= PHY_EYE_SWEEP;
                                end else begin
                                    eye_gate_finalize_started <= 1'b0;
                                    eye_gate_finalize_phase <=
                                        EYE_GATE_COMPLETE;
                                    phy_state <= PHY_EYE_DONE;
                                end
                                `ifndef YOSYS
                                    $display("[%0t] PHY eye: lane %0d verification failed across mCL candidates",
                                        $realtime, train_lane);
                                `endif
                            end
                        end
                        end
                    end

                    PHY_EYE_DONE: begin
`ifdef SIM_NATIVE_DIAG_FINAL_EYE_RESET_ALIGN
                        if (eye_final_align_active) begin
                            case (eye_final_align_phase)
                                EYE_FINAL_ALIGN_RESET: begin
                                    eye_final_align_seen <=
                                        {BYTE_LANES{1'b0}};
                                    if (rx_fifo_flush_count == 0)
                                        eye_final_align_phase <=
                                            EYE_FINAL_ALIGN_DATA;
                                end

                                EYE_FINAL_ALIGN_DATA: begin
                                    eye_final_align_seen <=
                                        eye_final_align_seen_after;
                                    if (&eye_final_align_seen_after)
                                        eye_final_align_phase <=
                                            EYE_FINAL_ALIGN_DRAIN;
                                end

                                default: begin
                                    // The calibration reader continuously
                                    // consumes MPR returns.  Respond only in a
                                    // quiet command interval after all FIFO and
                                    // fabric-pop state has drained to empty.
                                    if ((&fifo_empty_flat) &&
                                        !(|calibration_fifo_pop_q) &&
                                        !(|calibration_fifo_word_valid_q) &&
                                        !(|dfi_read_command) &&
                                        (eye_last_read_age >=
                                         NATIVE_MPR_EXIT_GAP_CTRL)) begin
                                        eye_final_align_active <= 1'b0;
                                        eye_gate_finalize_phase <=
                                            EYE_GATE_COMPLETE;
                                        o_dfi_rdlvl_resp <=
                                            {BYTE_LANES{1'b1}};
                                    end
                                end
                            endcase
                        end else
`endif
                        if (gate_consensus_active) begin
                            case (gate_consensus_phase)
                                GATE_CONS_LANE: begin
                                    // Derive this lane's representable common
                                    // mCL interval. One RL_DLY coarse code is
                                    // one half-UI, hence four codes per tCK.
                                    if (gate_trained_mcl[
                                            gate_consensus_lane] >
                                        ((6'd15 - gate_trained_coarse[
                                            gate_consensus_lane]) >> 2)) begin
                                        gate_consensus_lane_lower_q <=
                                            gate_trained_mcl[
                                                gate_consensus_lane] -
                                            ((6'd15 - gate_trained_coarse[
                                                gate_consensus_lane]) >> 2);
                                    end else begin
                                        gate_consensus_lane_lower_q <= 6'd7;
                                    end
                                    if ((gate_trained_mcl[
                                            gate_consensus_lane] -
                                         ((6'd15 - gate_trained_coarse[
                                            gate_consensus_lane]) >> 2)) <
                                        6'd7)
                                        gate_consensus_lane_lower_q <= 6'd7;

                                    if (({1'b0, gate_trained_mcl[
                                            gate_consensus_lane]} +
                                         (gate_trained_coarse[
                                            gate_consensus_lane] >> 2)) >
                                        7'd63)
                                        gate_consensus_lane_upper_q <= 6'd63;
                                    else
                                        gate_consensus_lane_upper_q <=
                                            gate_trained_mcl[
                                                gate_consensus_lane] +
                                            (gate_trained_coarse[
                                                gate_consensus_lane] >> 2);
                                    gate_consensus_phase <= GATE_CONS_ACCUM;
                                end

                                GATE_CONS_ACCUM: begin
                                    if (gate_consensus_lane_lower_q >
                                        gate_consensus_lower_work)
                                        gate_consensus_lower_work <=
                                            gate_consensus_lane_lower_q;
                                    if (gate_consensus_lane_upper_q <
                                        gate_consensus_upper_work)
                                        gate_consensus_upper_work <=
                                            gate_consensus_lane_upper_q;

                                    if (gate_consensus_lane ==
                                        BYTE_LANES - 1) begin
                                        gate_mcl_consensus_lower <=
                                            (gate_consensus_lane_lower_q >
                                             gate_consensus_lower_work) ?
                                            gate_consensus_lane_lower_q :
                                            gate_consensus_lower_work;
                                        gate_mcl_consensus_upper <=
                                            (gate_consensus_lane_upper_q <
                                             gate_consensus_upper_work) ?
                                            gate_consensus_lane_upper_q :
                                            gate_consensus_upper_work;

                                        if (((gate_consensus_lane_lower_q >
                                              gate_consensus_lower_work) ?
                                             gate_consensus_lane_lower_q :
                                             gate_consensus_lower_work) <=
                                            ((gate_consensus_lane_upper_q <
                                              gate_consensus_upper_work) ?
                                             gate_consensus_lane_upper_q :
                                             gate_consensus_upper_work)) begin
                                            gate_mcl_consensus_valid <= 1'b1;
                                            if (NATIVE_CL_NCK <
                                                ((gate_consensus_lane_lower_q >
                                                  gate_consensus_lower_work) ?
                                                 gate_consensus_lane_lower_q :
                                                 gate_consensus_lower_work))
                                                app_read_mcl <=
                                                    (gate_consensus_lane_lower_q >
                                                     gate_consensus_lower_work) ?
                                                    gate_consensus_lane_lower_q :
                                                    gate_consensus_lower_work;
                                            else if (NATIVE_CL_NCK >
                                                ((gate_consensus_lane_upper_q <
                                                  gate_consensus_upper_work) ?
                                                 gate_consensus_lane_upper_q :
                                                 gate_consensus_upper_work))
                                                app_read_mcl <=
                                                    (gate_consensus_lane_upper_q <
                                                     gate_consensus_upper_work) ?
                                                    gate_consensus_lane_upper_q :
                                                    gate_consensus_upper_work;
                                            else
                                                app_read_mcl <= NATIVE_CL_NCK;
                                            gate_consensus_lane <= 0;
                                            gate_consensus_phase <=
                                                GATE_CONS_TARGET;
                                        end else begin
                                            // No one READ epoch fits every
                                            // lane's four-bit coarse range.
                                            gate_mcl_consensus_valid <= 1'b0;
                                            gate_consensus_active <= 1'b0;
                                            eye_train_fail <=
                                                {BYTE_LANES{1'b1}};
                                            eye_gate_finalize_phase <=
                                                EYE_GATE_COMPLETE;
                                        end
                                    end else begin
                                        gate_consensus_lane <=
                                            gate_consensus_lane + 1'b1;
                                        gate_consensus_phase <=
                                            GATE_CONS_LANE;
                                    end
                                end

                                GATE_CONS_TARGET: begin
                                    // Preserve the measured absolute phase at
                                    // the selected common command epoch.
                                    gate_consensus_target_q <=
                                        $signed({5'b00000,
                                            gate_trained_coarse[
                                                gate_consensus_lane]}) +
                                        (($signed({3'b000,
                                            gate_trained_mcl[
                                                gate_consensus_lane]}) -
                                          $signed({3'b000,
                                            app_read_mcl})) <<< 2);
                                    gate_consensus_phase <=
                                        GATE_CONS_COMMIT;
                                end

                                default: begin // GATE_CONS_COMMIT
                                    if ((gate_consensus_target_q < 0) ||
                                        (gate_consensus_target_q > 15)) begin
                                        // The interval proof above makes this
                                        // unreachable; keep a hard calibration
                                        // failure if stored state is corrupted.
                                        gate_mcl_consensus_valid <= 1'b0;
                                        gate_consensus_active <= 1'b0;
                                        eye_train_fail <=
                                            {BYTE_LANES{1'b1}};
                                        eye_gate_finalize_phase <=
                                            EYE_GATE_COMPLETE;
                                    end else begin
                                        app_read_mcl_upper[
                                            gate_consensus_lane] <=
                                            app_read_mcl;
                                        app_read_mcl_lower[
                                            gate_consensus_lane] <=
                                            app_read_mcl;
                                        app_read_coarse_target[
                                            gate_consensus_lane] <=
                                            gate_consensus_target_q[3:0];
                                        if (gate_consensus_lane == 0)
                                            gate_consensus_first_target <=
                                                gate_consensus_target_q[3:0];

                                        if (gate_consensus_lane ==
                                            BYTE_LANES - 1) begin
                                            gate_consensus_active <= 1'b0;
                                            train_lane <= 0;
                                            en_vtc_q <= 1'b0;
                                            bitslice_en_vtc_q <= 1'b0;
                                            eye_boundary_retime_done <= 1'b0;
                                            eye_gate_finalize_started <= 1'b1;
                                            eye_gate_finalize_phase <=
                                                EYE_GATE_REWIND;
                                            eye_gate_finalize_coarse <=
                                                gate_trained_coarse[0];
                                            eye_gate_finalize_tap <=
                                                gate_center[0];
                                            eye_gate_finalize_target_coarse_q <=
                                                (gate_consensus_lane == 0) ?
                                                gate_consensus_target_q[3:0] :
                                                gate_consensus_first_target;
                                            phy_timer <= 4'd15;
                                        end else begin
                                            gate_consensus_lane <=
                                                gate_consensus_lane + 1'b1;
                                            gate_consensus_phase <=
                                                GATE_CONS_TARGET;
                                        end
                                    end
                                end
                            endcase
                        end else if (!eye_gate_finalize_started) begin
                            // A failed eye has no usable phase to retime.  Let
                            // the controller consume the recorded lane failure.
                            en_vtc_q <= 1'b1;
                            bitslice_en_vtc_q <= 1'b1;
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b1}};
                        end else case (eye_gate_finalize_phase)
                            EYE_GATE_REWIND: begin
                                eye_gate_finalize_tap <= delay_step_toward(
                                    eye_gate_finalize_tap, 9'd0);
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_wr_data <= {
                                    3'd0,
                                    eye_gate_finalize_coarse,
                                    delay_step_toward(
                                        eye_gate_finalize_tap, 9'd0)};
                                native_riu_wr_en <= 1'b1;
                                native_riu_sel <= train_lane_mask;
                                phy_timer <= 4'd15;
                                eye_gate_finalize_phase <=
                                    EYE_GATE_WAIT_REWIND;
                            end

                            EYE_GATE_WAIT_REWIND: begin
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_sel <= train_lane_mask;
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (eye_gate_finalize_readback_ok) begin
                                    eye_gate_finalize_phase <=
                                        (eye_gate_finalize_tap == 9'd0) ?
                                        EYE_GATE_COARSE : EYE_GATE_REWIND;
                                end else begin
                                    phy_timer <= 4'd15;
                                end
                            end

                            EYE_GATE_COARSE: begin
                                // Move toward the normalized coarse phase one
                                // code per acknowledged RIU transaction. A
                                // direct multi-code write can read back before
                                // the source-synchronous FIFO has traversed the
                                // intervening boundary phases.
                                if (eye_gate_finalize_coarse ==
                                    eye_gate_finalize_target_coarse) begin
                                    eye_gate_finalize_phase <=
                                        EYE_GATE_RESTORE;
                                end else begin
                                    eye_gate_finalize_coarse <=
                                        eye_gate_finalize_next_coarse;
                                    native_riu_addr <=
                                        RIU_ADDR_RL_DLY_RNK0;
                                    native_riu_wr_data <= {
                                        3'd0,
                                        eye_gate_finalize_next_coarse,
                                        eye_gate_finalize_tap};
                                    native_riu_wr_en <= 1'b1;
                                    native_riu_sel <= train_lane_mask;
                                    phy_timer <= 4'd2;
                                    eye_gate_finalize_phase <=
                                        EYE_GATE_WAIT_COARSE;
                                end
                            end

                            EYE_GATE_WAIT_COARSE: begin
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_sel <= train_lane_mask;
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (eye_gate_finalize_readback_ok) begin
                                    if (eye_gate_finalize_coarse ==
                                        eye_gate_finalize_target_coarse)
                                        eye_gate_finalize_phase <=
                                            EYE_GATE_RESTORE;
                                    else
                                        eye_gate_finalize_phase <=
                                            EYE_GATE_COARSE;
                                end else begin
                                    phy_timer <= 4'd2;
                                end
                            end

                            EYE_GATE_RESTORE: begin
                                eye_gate_finalize_tap <= delay_step_toward(
                                    eye_gate_finalize_tap,
                                    eye_gate_finalize_target_tap);
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_wr_data <= {
                                    3'd0,
                                    eye_gate_finalize_coarse,
                                    delay_step_toward(
                                        eye_gate_finalize_tap,
                                        eye_gate_finalize_target_tap)};
                                native_riu_wr_en <= 1'b1;
                                native_riu_sel <= train_lane_mask;
                                phy_timer <= 4'd15;
                                eye_gate_finalize_phase <=
                                    EYE_GATE_WAIT_RESTORE;
                            end

                            EYE_GATE_WAIT_RESTORE: begin
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_sel <= train_lane_mask;
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (eye_gate_finalize_readback_ok) begin
                                    if (eye_gate_finalize_tap !=
                                        eye_gate_finalize_target_tap) begin
                                        eye_gate_finalize_phase <=
                                            EYE_GATE_RESTORE;
                                    end else if (train_lane < BYTE_LANES - 1) begin
                                        gate_trained_mcl[train_lane] <=
                                            app_read_mcl;
                                        gate_trained_mcl_low[train_lane] <=
                                            app_read_mcl;
                                        gate_trained_coarse[train_lane] <=
                                            eye_gate_finalize_target_coarse;
                                        gate_trained_coarse_low[train_lane] <=
                                            eye_gate_finalize_target_coarse;
                                        train_lane <= train_lane + 1'b1;
                                        eye_gate_finalize_coarse <=
                                            gate_trained_coarse[
                                                train_lane + 1'b1];
                                        eye_gate_finalize_tap <=
                                            gate_center[train_lane + 1'b1];
                                        eye_gate_finalize_target_coarse_q <=
                                            app_read_coarse_target[
                                                train_lane + 1'b1];
                                        eye_gate_finalize_phase <=
                                            EYE_GATE_REWIND;
                                    end else begin
                                        gate_trained_mcl[train_lane] <=
                                            app_read_mcl;
                                        gate_trained_mcl_low[train_lane] <=
                                            app_read_mcl;
                                        gate_trained_coarse[train_lane] <=
                                            eye_gate_finalize_target_coarse;
                                        gate_trained_coarse_low[train_lane] <=
                                            eye_gate_finalize_target_coarse;
                                        en_vtc_q <= 1'b1;
                                        bitslice_en_vtc_q <= 1'b1;
                                        phy_timer <= 4'd15;
                                        eye_gate_finalize_phase <=
                                            EYE_GATE_SETTLE;
                                    end
                                end else begin
                                    phy_timer <= 4'd15;
                                end
                            end

                            EYE_GATE_SETTLE: begin
                                en_vtc_q <= 1'b1;
                                bitslice_en_vtc_q <= 1'b1;
                                if (phy_timer != 0)
                                    phy_timer <= phy_timer - 1'b1;
                                else if (all_dly_rdy && all_vtc_rdy) begin
                                    // Preserve the FIFO phase established by
                                    // the acknowledged RL_DLY writes.  Wait
                                    // for one masked controller return before
                                    // responding, so no new source-clocked
                                    // word can alter the calibrated boundary.
                                    eye_boundary_retime_done <= 1'b1;
                                    // Re-enter the exact-MPR verifier at lane
                                    // zero. The finalizer has now programmed a
                                    // common command epoch and the physically
                                    // equivalent gate into every byte lane.
                                    train_lane <= 0;
                                    eye_verify_retries <= 2'd0;
                                    pattern_found_q <= 1'b0;
                                    phy_timer <= 4'd0;
                                    phy_state <= PHY_EYE_VERIFY;
                                end
                                else
                                    phy_timer <= 4'd2;
                            end

                            default: begin
                                en_vtc_q <= 1'b1;
                                bitslice_en_vtc_q <= 1'b1;
                                o_dfi_rdlvl_resp <=
                                    {BYTE_LANES{1'b1}};
                            end
                        endcase

                        if (((!eye_gate_finalize_started) ||
                             (eye_gate_finalize_phase ==
                              EYE_GATE_COMPLETE)) &&
                            !i_dfi_rdlvl_en) begin
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}};
                            eye_gate_finalize_started <= 1'b0;
                            phy_state <= PHY_IDLE;
                        end
                    end

                    PHY_WL_SAMPLE: begin
                        case (wl_riu_phase)
                            WL_RIU_PROGRAM: begin
                                // UG571 WL_DLY_RNK0 is the coordinated native
                                // delay for DQS and every DQ in the nibble.
                                // WL_TRAIN releases DQ while DQS remains driven.
                                native_riu_addr <= RIU_ADDR_WL_DLY_RNK0;
                                native_riu_wr_data <= wl_riu_train_value;
                                native_riu_wr_en <= 1'b1;
                                native_riu_lower_sel <= train_lane_mask;
                                native_riu_sel <= train_lane_mask;
                                phy_timer <= 4'd15;
                                wl_riu_phase <= WL_RIU_WAIT;
                            end

                            WL_RIU_WAIT: begin
                                native_riu_addr <= RIU_ADDR_WL_DLY_RNK0;
                                native_riu_lower_sel <= train_lane_mask;
                                native_riu_sel <= train_lane_mask;
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (native_riu_valid[train_lane] &&
                                             (native_riu_rd_data[train_lane][13:0] ==
                                              wl_riu_train_value[13:0])) begin
                                    wl_riu_phase <= WL_RIU_CAPTURE;
                                    phy_timer <= 4'd4;
`ifdef SIM_NATIVE_DIAG_WL_DQS_RX_GUARD
                                    // The serialized DQS output has been Low
                                    // throughout the RIU wait.  Re-enable its
                                    // receiver now, still four controller
                                    // clocks before the first training pulse.
                                    wl_dqs_rx_armed <= 1'b1;
`endif
                                end else begin
                                    phy_timer <= 4'd15;
                                end
                            end

                            default: begin
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (i_dfi_wrlvl_strobe) begin
`ifdef SIM_NATIVE_DIAG_WL_TRAIN_NO_DQS
                                    // Diagnostic A/B: exercise the native
                                    // WL_TRAIN RIU transition for every lane,
                                    // then restore run mode without launching a
                                    // physical DQS edge.  Keeping this behind a
                                    // simulation-only define separates a mode-
                                    // programming side effect from an RX word-
                                    // boundary change caused by WL feedback.
                                    wl_tap[train_lane] <= 9'd0;
                                    wl_dq_tap[train_lane] <= 9'd0;
                                    wl_coarse[train_lane] <= 4'd0;
                                    wl_riu_phase <= WL_RIU_PROGRAM;
                                    phy_state <= PHY_WL_APPLY;
`else
                                    // One DFI strobe produces exactly one
                                    // physical DQS rising edge. The controller
                                    // spaces pulses by the JEDEC WL interval.
                                    wl_dqs_strobe <= 1'b1;
                                    phy_timer <= 4'd15;
                                    phy_state <= PHY_WL_ADJUST;
`endif
                                end
                            end
                        endcase
                    end

                    PHY_WL_ADJUST: begin
                        if (phy_timer != 0) begin
                            phy_timer <= phy_timer - 1'b1;
                        end else if (wl_prime_pending) begin
                            // Preserve the exact-MPR deserializer phase: a
                            // primer is one complete 8-UI word (four legal,
                            // independently spaced WL strobes), not an RX
                            // reset followed by one edge. Discard that word,
                            // then capture the next complete settled word.
                            if (wl_capture_pulse_count <
                                WL_CAPTURE_PULSES-1'b1) begin
                                wl_capture_pulse_count <=
                                    wl_capture_pulse_count + 1'b1;
                                wl_word_wait_count <= 5'd0;
                                phy_state <= PHY_WL_SAMPLE;
                            end else if (wl_fifo_pop_issued[train_lane]) begin
                                // Write leveling is performed one byte lane at
                                // a time.  Only that lane receives the selected
                                // DQS feedback pulse, so completion must be
                                // qualified by its sticky FIFO-pop record.
                                // The one-cycle word-valid pulse occurs while
                                // phy_timer is still enforcing the post-strobe
                                // settling interval; testing that pulse here
                                // after the timer expires would miss every
                                // legitimate primer word. Requiring every lane
                                // similarly rejects independent byte feedback.
                                wl_prime_pending <= 1'b0;
                                wl_capture_pulse_count <= 3'd0;
                                wl_word_wait_count <= 5'd0;
                                wl_fifo_reset_q <= 1'b0;
                                wl_feedback_clear <= 1'b1;
                                wl_mixed_retries <= 2'd0;
                                phy_state <= PHY_WL_SAMPLE;
                            end else if (wl_word_wait_count !=
                                         WL_WORD_WAIT_MAX) begin
                                wl_word_wait_count <=
                                    wl_word_wait_count + 1'b1;
                            end else if (wl_mixed_retries != 2'd2) begin
                                // Retry another complete primer word. Keeping
                                // every retry at eight UIs preserves phase.
                                wl_mixed_retries <= wl_mixed_retries + 1'b1;
                                wl_capture_pulse_count <= 3'd0;
                                wl_word_wait_count <= 5'd0;
                                wl_feedback_clear <= 1'b1;
                                phy_state <= PHY_WL_SAMPLE;
                            end else if (wl_sweep_at_end) begin
                                wl_train_fail[train_lane] <= 1'b1;
                                wl_tap[train_lane] <= 9'd0;
                                wl_dq_tap[train_lane] <= 9'd0;
                                wl_coarse[train_lane] <= 4'd0;
                                wl_capture_pulse_count <= 3'd0;
                                wl_word_wait_count <= 5'd0;
                                wl_feedback_clear <= 1'b1;
                                wl_mixed_retries <= 2'd0;
                                wl_riu_phase <= WL_RIU_PROGRAM;
                                phy_state <= PHY_WL_APPLY;
                            end else begin
                                wl_tap[train_lane] <= wl_next_fine;
                                wl_dq_tap[train_lane] <= wl_next_fine;
                                wl_sweep_coarse <= wl_next_coarse;
                                wl_capture_pulse_count <= 3'd0;
                                wl_word_wait_count <= 5'd0;
                                wl_feedback_clear <= 1'b1;
                                wl_mixed_retries <= 2'd0;
                                wl_riu_phase <= WL_RIU_PROGRAM;
                                phy_state <= PHY_WL_SAMPLE;
                            end
                        end else if (!wl_feedback_valid) begin
                            // Generate exactly four pulses, then wait for the
                            // synchronized EMPTY/pop latency without adding a
                            // fifth edge. Any retry is another complete 8-UI
                            // primer/capture pair.
                            if (wl_capture_pulse_count <
                                WL_CAPTURE_PULSES-1'b1) begin
                                wl_capture_pulse_count <=
                                    wl_capture_pulse_count + 1'b1;
                                wl_word_wait_count <= 5'd0;
                                phy_state <= PHY_WL_SAMPLE;
                            end else if (wl_word_wait_count !=
                                         WL_WORD_WAIT_MAX) begin
                                wl_word_wait_count <=
                                    wl_word_wait_count + 1'b1;
                            end else if (wl_mixed_retries != 2'd2) begin
                                wl_mixed_retries <= wl_mixed_retries + 1'b1;
                                wl_prime_pending <= 1'b1;
                                wl_capture_pulse_count <= 3'd0;
                                wl_word_wait_count <= 5'd0;
                                wl_feedback_clear <= 1'b1;
                                phy_state <= PHY_WL_SAMPLE;
                            end else if (wl_sweep_at_end) begin
                                // A complete native word never arrived after
                                // three independent sequences. Restore the
                                // pre-WL phase and report an uncorrectable RX
                                // capture failure for this byte.
                                wl_train_fail[train_lane] <= 1'b1;
                                wl_tap[train_lane] <= 9'd0;
                                wl_dq_tap[train_lane] <= 9'd0;
                                wl_coarse[train_lane] <= 4'd0;
                                wl_prime_pending <= 1'b1;
                                wl_capture_pulse_count <= 3'd0;
                                wl_word_wait_count <= 5'd0;
                                wl_feedback_clear <= 1'b1;
                                wl_mixed_retries <= 2'd0;
                                wl_riu_phase <= WL_RIU_PROGRAM;
                                phy_state <= PHY_WL_APPLY;
                            end else begin
                                // A local FIFO anomaly must not hang the
                                // complete calibration. Move to the next tap
                                // and require a fresh primer there.
                                wl_tap[train_lane] <= wl_next_fine;
                                wl_dq_tap[train_lane] <= wl_next_fine;
                                wl_sweep_coarse <= wl_next_coarse;
                                wl_prime_pending <= 1'b1;
                                wl_capture_pulse_count <= 3'd0;
                                wl_word_wait_count <= 5'd0;
                                wl_feedback_clear <= 1'b1;
                                wl_mixed_retries <= 2'd0;
                                wl_riu_phase <= WL_RIU_PROGRAM;
                                phy_state <= PHY_WL_SAMPLE;
                            end
                        end else if (!(wl_feedback_zero || wl_feedback_one)) begin
                            // JEDEC drives the same feedback on every DQ in a
                            // byte. A mixed lane word is therefore either a
                            // boundary/metastability sample or a disturbed
                            // capture. Retry the complete settled sequence.
                            if (wl_mixed_retries != 2'd2) begin
                                wl_mixed_retries <= wl_mixed_retries + 1'b1;
                                wl_prime_pending <= 1'b1;
                                wl_capture_pulse_count <= 3'd0;
                                wl_word_wait_count <= 5'd0;
                                wl_fifo_reset_q <= 1'b0;
                                wl_feedback_clear <= 1'b1;
                                phy_state <= PHY_WL_SAMPLE;
                            end else if (wl_sweep_at_end) begin
                                // The range ended on an indeterminate edge.
                                // Restore the calibrated pre-WL phase and
                                // report failure only if a low interval had
                                // already been established.
                                if (wl_seen_zero[train_lane])
                                    wl_train_fail[train_lane] <= 1'b1;
                                wl_tap[train_lane] <= 9'd0;
                                wl_dq_tap[train_lane] <= 9'd0;
                                wl_coarse[train_lane] <= 4'd0;
                                wl_prime_pending <= 1'b1;
                                wl_capture_pulse_count <= 3'd0;
                                wl_word_wait_count <= 5'd0;
                                wl_fifo_reset_q <= 1'b0;
                                wl_feedback_clear <= 1'b1;
                                wl_mixed_retries <= 2'd0;
                                wl_riu_phase <= WL_RIU_PROGRAM;
                                phy_state <= PHY_WL_APPLY;
                            end else begin
                                wl_tap[train_lane] <= wl_next_fine;
                                wl_dq_tap[train_lane] <= wl_next_fine;
                                wl_sweep_coarse <= wl_next_coarse;
                                wl_prime_pending <= 1'b1;
                                wl_capture_pulse_count <= 3'd0;
                                wl_word_wait_count <= 5'd0;
                                wl_fifo_reset_q <= 1'b0;
                                wl_feedback_clear <= 1'b1;
                                wl_mixed_retries <= 2'd0;
                                wl_riu_phase <= WL_RIU_PROGRAM;
                                phy_state <= PHY_WL_SAMPLE;
                            end
                        end else begin
`ifdef SIM_NATIVE_RIU_DEBUG
                            $display("[%0t] NATIVE_WL_WORD: lane=%0d coarse=%0d fine=%0d word=%h zero=%0b one=%0b empty=%h",
                                $realtime, train_lane, wl_sweep_coarse,
                                wl_tap[train_lane], wl_feedback_lane_word,
                                wl_feedback_zero, wl_feedback_one,
                                fifo_empty[train_lane]);
`endif
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
                                // Native WL_DLY coarse phase repeats every
                                // four codes (one complete tCK).  Retaining an
                                // integral-period component changes the DFI
                                // write latency even though it does not change
                                // the trained DQS-to-CK phase: for example,
                                // edge code 4 is phase-equivalent to code 0
                                // but launches the complete BL8 one tCK late.
                                // Keep the fine point and the phase remainder;
                                // this is the earliest equivalent trained edge
                                // and therefore preserves the configured CWL.
                                // WL_DLY still applies the same result to DQS
                                // and every DQ bit in the byte.
                                wl_dq_tap[train_lane] <= wl_tap[train_lane];
                                wl_coarse[train_lane] <=
                                    {2'b00, wl_sweep_coarse[1:0]};
                                wl_prime_pending <= 1'b1;
                                wl_capture_pulse_count <= 3'd0;
                                wl_word_wait_count <= 5'd0;
                                wl_fifo_reset_q <= 1'b0;
                                wl_feedback_clear <= 1'b1;
                                wl_mixed_retries <= 2'd0;
                                wl_riu_phase <= WL_RIU_PROGRAM;
                                phy_state <= PHY_WL_APPLY;
                            end else if (wl_sweep_at_end) begin
                                // If the entire range stayed high, no edge was
                                // reachable.  Retain the pre-WL BISC baseline.
                                // If a low was seen without a later high, report
                                // an actual failure but still restore safe taps.
                                if (!wl_seen_zero[train_lane] && !wl_feedback_zero) begin
                                    wl_tap[train_lane] <= 9'd0;
                                end else begin
                                    wl_train_fail[train_lane] <= 1'b1;
                                    wl_tap[train_lane] <= 9'd0;
                                end
                                wl_dq_tap[train_lane] <= 9'd0;
                                wl_coarse[train_lane] <= 4'd0;
                                wl_prime_pending <= 1'b1;
                                wl_capture_pulse_count <= 3'd0;
                                wl_word_wait_count <= 5'd0;
                                wl_fifo_reset_q <= 1'b0;
                                wl_feedback_clear <= 1'b1;
                                wl_mixed_retries <= 2'd0;
                                wl_riu_phase <= WL_RIU_PROGRAM;
                                phy_state <= PHY_WL_APPLY;
                            end else begin
                                wl_tap[train_lane] <= wl_next_fine;
                                wl_dq_tap[train_lane] <= wl_next_fine;
                                wl_sweep_coarse <= wl_next_coarse;
                                wl_prime_pending <= 1'b1;
                                wl_capture_pulse_count <= 3'd0;
                                wl_word_wait_count <= 5'd0;
                                wl_fifo_reset_q <= 1'b0;
                                wl_feedback_clear <= 1'b1;
                                wl_mixed_retries <= 2'd0;
                                wl_riu_phase <= WL_RIU_PROGRAM;
                                phy_state <= PHY_WL_SAMPLE;
                            end
                        end
                    end

                    // Commit the trained native write-level delay with
                    // WL_TRAIN cleared. Both nibbles are written so DQS and
                    // all eight DQ outputs retain one coherent TX phase.
                    PHY_WL_APPLY: begin
                        if (wl_riu_phase == WL_RIU_PROGRAM) begin
                            native_riu_addr <= RIU_ADDR_WL_DLY_RNK0;
                            native_riu_wr_data <= wl_riu_run_value;
                            native_riu_wr_en <= 1'b1;
                            native_riu_lower_sel <= train_lane_mask;
                            native_riu_sel <= train_lane_mask;
                            phy_timer <= 4'd15;
                            wl_riu_phase <= WL_RIU_WAIT;
                        end else begin
                            native_riu_addr <= RIU_ADDR_WL_DLY_RNK0;
                            native_riu_lower_sel <= train_lane_mask;
                            native_riu_sel <= train_lane_mask;
                            if (phy_timer != 0) begin
                                phy_timer <= phy_timer - 1'b1;
                            end else if (native_riu_valid[train_lane] &&
                                         (native_riu_rd_data[train_lane][13:0] ==
                                          wl_riu_run_value[13:0])) begin
                                phy_state <= PHY_WL_CHECK;
                            end else begin
                                phy_timer <= 4'd15;
                            end
                        end
                    end

                    PHY_WL_CHECK: begin
`ifdef SIM_NATIVE_DIAG_SKIP_WL_ACTIVITY
                        o_dfi_wrlvl_resp <= {BYTE_LANES{1'b1}};
                        if (!i_dfi_wrlvl_en) begin
                            o_dfi_wrlvl_resp <= {BYTE_LANES{1'b0}};
                            phy_state <= PHY_IDLE;
                        end
`else
                        `ifndef YOSYS
                            $display("[%0t] PHY WL: lane %0d coarse=%0d fine=%0d", $realtime, train_lane, wl_coarse[train_lane], wl_tap[train_lane]);
                        `endif
                        /* verilator lint_off WIDTHEXPAND */
                        if (train_lane < BYTE_LANES - 1) begin
                        /* verilator lint_on WIDTHEXPAND */
                            train_lane <= train_lane + 1'b1;
                            wl_tap[train_lane + 1'b1]    <= 9'd0;
                            wl_dq_tap[train_lane + 1'b1] <= 9'd0;
                            wl_coarse[train_lane + 1'b1] <= 4'd0;
                            wl_sweep_coarse <= 4'd0;
                            wl_seen_zero[train_lane + 1'b1] <= 1'b0;
                            wl_prime_pending <= 1'b1;
                            wl_capture_pulse_count <= 3'd0;
                            wl_word_wait_count <= 5'd0;
                            wl_fifo_reset_q <= 1'b0;
                            wl_feedback_clear <= 1'b1;
                            wl_mixed_retries <= 2'd0;
                            wl_riu_phase <= WL_RIU_PROGRAM;
                            phy_state <= PHY_WL_SAMPLE;
                        end else begin
                            // The final tap is already applied and no more WL
                            // DQS edges will occur. Release the WL-owned reset
                            // now; the existing post-WL flush handles the
                            // handoff to application traffic.
                            wl_fifo_reset_q <= 1'b0;
`ifdef SIM_NATIVE_DIAG_WL_DQS_RX_GUARD
                            // Close the DQS receiver while the transmitter is
                            // still driving Low.  Keep ownership for four more
                            // DIV_CLK cycles in PHY_WL_DONE before releasing
                            // the pad, so neither transition reaches the
                            // source-synchronous FIFO as a clock edge.
                            wl_dqs_rx_armed <= 1'b0;
                            wl_dqs_release_guard <= 3'd4;
`endif
                            wl_feedback_clear <= 1'b1;
                            en_vtc_q <= 1'b1;
                            bitslice_en_vtc_q <= 1'b1;
                            vtc_settle_counter <= VTC_SETTLE_CYCLES;
                            // Four bounded WL pulses produced and consumed one
                            // complete native FIFO word.  Preserve that modulo-8
                            // boundary and the trained DQS-gate state: CLR_GATE
                            // here restarts the source-synchronous gate while DQS
                            // is stopped and can move the first application word
                            // by a fractional burst.  Freeze calibration pops and
                            // drain only their two-register fabric pipeline.
`ifdef SIM_NATIVE_DIAG_WL_DEFER_GATE_RELOAD
                            // WL feedback consumes locally launched DQS edges,
                            // but those edges are not routed through the DQS
                            // input gate and therefore cannot transfer an
                            // RL_DLY shadow update.  Queue a reload of the
                            // already verified MPR gate point.  The first
                            // subsequent DRAM read preamble supplies the
                            // source-synchronous edge that commits it before
                            // the BL8 data edges arrive.
                            en_vtc_q <= 1'b0;
                            train_lane <= 0;
                            wl_handoff_gate_coarse <=
                                gate_trained_coarse[0];
                            wl_handoff_gate_tap <= gate_center[0];
                            phy_timer <= 4'd15;
                            wl_handoff_phase <= WL_HANDOFF_WAIT_VTC_OFF;
`elsif SIM_NATIVE_DIAG_WL_GATE_SHIFT_THEN_RESET_PRIME
                            // Establish the final application DQS-gate phase
                            // while RIU transfer is still live.  The receive
                            // FIFO is reset and primed only after every lane
                            // acknowledges the +2-UI RL_DLY adjustment.
                            en_vtc_q <= 1'b0;
                            train_lane <= 0;
                            wl_handoff_gate_coarse <=
                                gate_trained_coarse[0];
                            wl_handoff_gate_tap <= gate_center[0];
                            phy_timer <= 4'd15;
                            wl_handoff_phase <= WL_HANDOFF_WAIT_VTC_OFF;
`elsif SIM_NATIVE_DIAG_WL_GATE_PLUS1
                            // The two adjacent legal four-tCK PHY_RDEN masks
                            // straddle the required BL8 boundary: the earlier
                            // mask admits the two read-preamble edges, while
                            // the later mask drops the first two data edges.
                            // UG571 defines one RL_DLY_CRSE code as half a
                            // PLL_CLK period, exactly one DDR UI.  Apply that
                            // one-code correction only after write leveling,
                            // so WL feedback continues to use its calibrated
                            // gate and the application receiver starts on the
                            // first BL8 data edge.
                            en_vtc_q <= 1'b0;
                            train_lane <= 0;
                            wl_handoff_gate_coarse <=
                                gate_trained_coarse[0];
                            wl_handoff_gate_tap <= gate_center[0];
                            phy_timer <= 4'd15;
                            wl_handoff_phase <= WL_HANDOFF_WAIT_VTC_OFF;
`elsif SIM_NATIVE_DIAG_WL_GATE_PLUS2
                            // Shift the native DQS gate by the one-tCK read
                            // preamble. UG571 defines RL_DLY_CRSE as half a
                            // PLL_CLK period (one DDR UI), so the handoff FSM's
                            // normal +2 target selects the first data edge. The
                            // application mask moves one tCK earlier above,
                            // retaining all four data clocks of the BL8 burst.
                            en_vtc_q <= 1'b0;
                            train_lane <= 0;
                            wl_handoff_gate_coarse <=
                                gate_trained_coarse[0];
                            wl_handoff_gate_tap <= gate_center[0];
                            phy_timer <= 4'd15;
                            wl_handoff_phase <= WL_HANDOFF_WAIT_VTC_OFF;
`elsif SIM_NATIVE_DIAG_WL_GATE_PLUS3
                            // Diagnostic A/B: place the native 1:8 FIFO
                            // boundary one UI later than the normal +2
                            // post-eye setting while legal WL DQS pulses are
                            // still available to transfer the RIU update.
                            en_vtc_q <= 1'b0;
                            train_lane <= 0;
                            wl_handoff_gate_coarse <=
                                gate_trained_coarse[0];
                            wl_handoff_gate_tap <= gate_center[0];
                            phy_timer <= 4'd15;
                            wl_handoff_phase <= WL_HANDOFF_WAIT_VTC_OFF;
`elsif SIM_NATIVE_DIAG_WL_GATE_PLUS4
                            // A/B the application word-boundary correction at
                            // the final source-synchronous training boundary.
                            // No RX reset follows this sequence.
                            en_vtc_q <= 1'b0;
                            train_lane <= 0;
                            wl_handoff_gate_coarse <=
                                gate_trained_coarse[0];
                            wl_handoff_gate_tap <= gate_center[0];
                            phy_timer <= 4'd15;
                            wl_handoff_phase <= WL_HANDOFF_WAIT_VTC_OFF;
`elsif SIM_NATIVE_DIAG_WL_FINAL_RESET_PRIME
                            // Application traffic needs a FIFO boundary that
                            // contains no calibration or WL feedback. Assert
                            // RX_RST asynchronously while JEDEC write-level
                            // mode is still active, then use legal WL DQS
                            // pulses below to clock reset release and establish
                            // a known complete-word boundary. Trained RL_DLY,
                            // DQ eye delays, and TX WL delay are unchanged.
                            wl_fifo_reset_q <= 1'b1;
                            wl_handoff_phase <= WL_HANDOFF_RX_RESET;
                            phy_timer <= 4'd15;
`elsif SIM_NATIVE_DIAG_WL_CLEAR_GATE
                            // Directed A/B: separated JEDEC write-level
                            // strobes leave the native gate's modulo-8 state
                            // unrelated to the earlier MPR read boundary.
                            // Restart only the DQS gate before application
                            // traffic; trained RL_DLY and DQ eye taps remain
                            // unchanged.
                            phy_timer <= 4'd15;
                            wl_handoff_phase <= WL_HANDOFF_CLEAR;
`elsif SIM_NATIVE_DIAG_WL_BS_RESET
                            // Assert and release BS_CTRL.BS_RESET atomically in
                            // both nibbles of every byte after WL activity.
                            phy_timer <= 4'd15;
                            wl_handoff_phase <= WL_HANDOFF_CLEAR;
`elsif SIM_NATIVE_DIAG_MIG_WL_FIFO_RDEN
                            // MIG keeps the read pointer following the sparse
                            // WL write pointer, but Q still contains the final
                            // all-zero/all-one feedback word.  Transfer to the
                            // normal one-way EMPTY handoff before application
                            // ownership; going directly complete would expose
                            // that retained WL word as the first BIST return.
                            phy_timer <= 4'd15;
                            wl_handoff_phase <= WL_HANDOFF_DRAIN;
`elsif SIM_NATIVE_DIAG_WL_DIRECT_COMPLETE
                            // Diagnostic A/B: preserve the FIFO/read-gate
                            // state left by the complete, byte-lane-local WL
                            // capture sequence.  Skipping only the post-WL
                            // asynchronous FIFO drain isolates whether that
                            // reader stop/restart changes the modulo-8 word
                            // boundary established during read calibration.
                            phy_timer <= 4'd15;
                            wl_handoff_phase <= WL_HANDOFF_COMPLETE;
`else
                            wl_handoff_phase <= WL_HANDOFF_DRAIN;
`endif
                            wl_handoff_empty_seen <= {TOTAL_DQ{1'b0}};
                            phy_state <= PHY_WL_DONE;
                            `ifndef YOSYS
                                for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                    $display("[%0t] PHY WL done: lane %0d dqs_tap=%0d dq_tap=%0d", $realtime, dfi_pack_idx, wl_tap[dfi_pack_idx], wl_dq_tap[dfi_pack_idx]);
                                end
                            `endif
                        end
`endif
                    end

                    PHY_WL_DONE: begin
                        wl_fifo_reset_q <= 1'b0;
`ifdef SIM_NATIVE_DIAG_WL_DQS_RX_GUARD
                        if (wl_dqs_release_guard != 0)
                            wl_dqs_release_guard <=
                                wl_dqs_release_guard - 1'b1;
                        else
`endif
                        if (vtc_settle_counter != 0)
                            vtc_settle_counter <= vtc_settle_counter - 1'b1;
                        else case (wl_handoff_phase)
                            WL_HANDOFF_RX_RESET: begin
                                // RX_RST asserts asynchronously.  Hold it for
                                // a bounded DIV_CLK interval before releasing
                                // it while locally generated WL DQS is still
                                // available as the synchronous reset clock.
                                wl_fifo_reset_q <= 1'b1;
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= {BYTE_LANES{1'b0}};
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else begin
                                    wl_fifo_reset_q <= 1'b0;
                                    wl_word_wait_count <= 5'd0;
                                    wl_handoff_nonempty_seen <=
                                        {TOTAL_DQ{1'b0}};
                                    wl_handoff_empty_seen <=
                                        {TOTAL_DQ{1'b0}};
                                    // RX_RST releases synchronously in the
                                    // source-strobe domain.  Prime exactly one
                                    // native 8-UI word while JEDEC write-level
                                    // mode is still active, then drain it.  A
                                    // WL pulse supplies one DQS rise/fall pair
                                    // (two UIs), so four legally spaced pulses
                                    // establish the complete 1:8 boundary
                                    // without sacrificing the first real READ.
                                    phy_timer <= 4'd15;
                                    wl_handoff_phase <=
                                        WL_HANDOFF_RX_PRIME;
                                end
                            end

                            WL_HANDOFF_RX_PRIME: begin
                                wl_fifo_reset_q <= 1'b0;
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= {BYTE_LANES{1'b0}};
                                // The normal calibration reader is active in
                                // this phase and consumes a completed word on
                                // the cycle after EMPTY deasserts.  Remember
                                // that deassertion here; waiting until the
                                // following drain phase misses the entire
                                // legal nonempty interval and falsely reports
                                // that the restart produced no FIFO data.
                                wl_handoff_nonempty_seen <=
                                    wl_handoff_nonempty_seen |
                                    ~fifo_empty_flat;
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else begin
                                // RX_RST deassertion is synchronized to the
                                // source-synchronous DQS domain.  Supply four
                                // separated pulses for reset release before
                                // the four pulses that contribute one complete
                                // eight-UI disposable FIFO word.  Keeping the
                                // pulses individually spaced preserves the
                                // JEDEC write-level feedback interval.
                                wl_dqs_strobe <= 1'b1;
                                phy_timer <= 4'd15;
`ifdef SIM_NATIVE_DIAG_WL_PRIME_9_PULSES
                                if (wl_word_wait_count == 5'd8) begin
`else
                                if (wl_word_wait_count == 5'd7) begin
`endif
                                        wl_word_wait_count <= 5'd0;
                                        wl_handoff_phase <=
                                            WL_HANDOFF_RX_DRAIN;
                                    end else begin
                                        wl_word_wait_count <=
                                            wl_word_wait_count + 1'b1;
                                    end
                                end
                            end

                            WL_HANDOFF_RX_DRAIN: begin
                                wl_fifo_reset_q <= 1'b0;
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= {BYTE_LANES{1'b0}};
                                // Once a bit reaches EMPTY, its direct-gated
                                // RD_EN goes Low and remains Low even if the
                                // delayed status later changes.  This is the
                                // required one-way stop at pointer equality.
                                wl_handoff_empty_seen <=
                                    wl_handoff_empty_seen |
                                    fifo_empty_flat;
`ifndef YOSYS
`ifdef SIM_NATIVE_DIAG_WL_FINAL_RESET_PRIME
                                wl_word_wait_count <=
                                    wl_word_wait_count + 1'b1;
                                if (wl_word_wait_count == 0)
                                    $display("[%0t] NATIVE_WL_RX_DRAIN: nonempty=%h empty_seen=%h empty=%h pop=%b valid=%b",
                                        $realtime,
                                        wl_handoff_nonempty_seen,
                                        wl_handoff_empty_seen,
                                        fifo_empty_flat,
                                        calibration_fifo_pop_q,
                                        calibration_fifo_word_valid_q);
`endif
`endif
                                if ((&wl_handoff_empty_seen) &&
                                    !(|calibration_fifo_pop_q) &&
                                    !(|calibration_fifo_word_valid_q)) begin
                                    en_vtc_q <= 1'b1;
                                    bitslice_en_vtc_q <= 1'b1;
                                    vtc_settle_counter <=
                                        VTC_SETTLE_CYCLES;
                                    phy_timer <= 4'd15;
`ifdef SIM_NATIVE_DIAG_WL_GATE_SHIFT_THEN_RESET_PRIME
                                    // RL_DLY was finalized before RX_RST.  Do
                                    // not issue RIU writes after reset: the
                                    // primitive deasserts RIU_VALID until a
                                    // new source-synchronous capture session.
                                    wl_handoff_phase <=
                                        WL_HANDOFF_COMPLETE;
`elsif SIM_NATIVE_DIAG_WL_RESET_PRIME_GATE_SHIFT
                                    // Directed post-reset boundary test: move
                                    // RL_DLY by the two-UI preamble observed at
                                    // the pre-advance FIFO head, then transfer
                                    // each RIU update using legal WL DQS edges.
                                    en_vtc_q <= 1'b0;
                                    bitslice_en_vtc_q <= 1'b0;
                                    train_lane <= 0;
                                    wl_handoff_gate_coarse <=
                                        gate_trained_coarse[0];
                                    wl_handoff_gate_tap <= gate_center[0];
                                    wl_handoff_phase <=
                                        WL_HANDOFF_WAIT_VTC_OFF;
`elsif SIM_NATIVE_DIAG_WL_PRESERVE_GATE
                                    // The exact-MPR eye already proved the
                                    // trained gate across repeated READs. Do
                                    // not restart it with DQS stopped: that
                                    // operation can change the first native
                                    // FIFO word boundary without a source-
                                    // synchronous edge on which to settle.
                                    wl_handoff_phase <=
                                        WL_HANDOFF_COMPLETE;
`else
                                    wl_handoff_phase <= WL_HANDOFF_CLEAR;
`endif
                                end
                            end

                            WL_HANDOFF_CLEAR: begin
`ifdef SIM_NATIVE_DIAG_WL_BS_RESET
                                native_riu_addr <= RIU_ADDR_BS_CTRL;
                                native_riu_wr_data <= RIU_BS_RESET_MASK;
                                native_riu_wr_en <= 1'b1;
                                native_riu_lower_sel <= {BYTE_LANES{1'b1}};
                                native_riu_sel <= {BYTE_LANES{1'b1}};
`else
                                // CLR_GATE resets the native DQS-gate state;
                                // it does not modify the trained RL_DLY or the
                                // DQ/DQS write-level delay registers.
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_wr_data <= RIU_GATE_CLEAR;
                                native_riu_wr_en <= 1'b1;
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= {BYTE_LANES{1'b1}};
`endif
                                phy_timer <= 4'd15;
                                wl_handoff_phase <= WL_HANDOFF_WAIT_CLEAR;
                            end

                            WL_HANDOFF_WAIT_CLEAR: begin
`ifdef SIM_NATIVE_DIAG_WL_BS_RESET
                                native_riu_addr <= RIU_ADDR_BS_CTRL;
                                native_riu_lower_sel <= {BYTE_LANES{1'b1}};
                                native_riu_sel <= {BYTE_LANES{1'b1}};
`else
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= {BYTE_LANES{1'b1}};
`endif
                                if (phy_timer != 0)
                                    phy_timer <= phy_timer - 1'b1;
                                else if (wl_handoff_readback_ok)
                                    wl_handoff_phase <= WL_HANDOFF_RELEASE;
                                else
                                    phy_timer <= 4'd15;
                            end

                            WL_HANDOFF_RELEASE: begin
`ifdef SIM_NATIVE_DIAG_WL_BS_RESET
                                native_riu_addr <= RIU_ADDR_BS_CTRL;
                                native_riu_wr_data <= 16'h0000;
                                native_riu_wr_en <= 1'b1;
                                native_riu_lower_sel <= {BYTE_LANES{1'b1}};
                                native_riu_sel <= {BYTE_LANES{1'b1}};
`else
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_wr_data <= RIU_GATE_RUN;
                                native_riu_wr_en <= 1'b1;
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= {BYTE_LANES{1'b1}};
`endif
                                phy_timer <= 4'd15;
                                wl_handoff_phase <= WL_HANDOFF_WAIT_RELEASE;
                            end

                            WL_HANDOFF_WAIT_RELEASE: begin
`ifdef SIM_NATIVE_DIAG_WL_BS_RESET
                                native_riu_addr <= RIU_ADDR_BS_CTRL;
                                native_riu_lower_sel <= {BYTE_LANES{1'b1}};
                                native_riu_sel <= {BYTE_LANES{1'b1}};
`else
                                native_riu_addr <= RIU_ADDR_NIBBLE_CTRL0;
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= {BYTE_LANES{1'b1}};
`endif
                                if (phy_timer != 0)
                                    phy_timer <= phy_timer - 1'b1;
                                else if (wl_handoff_readback_ok) begin
                                    // Allow all source-synchronous FIFO status
                                    // flags and the registered calibration pop
                                    // path to settle before testing for empty.
                                    phy_timer <= 4'd15;
                                    wl_handoff_phase <= WL_HANDOFF_DRAIN;
                                end
                                else
                                    phy_timer <= 4'd15;
                            end

                            WL_HANDOFF_DRAIN: begin
                                // Write leveling is performed sequentially by
                                // byte lane, so its completed feedback words
                                // leave the independent DQ FIFOs at different
                                // read-pointer positions.  Advance each bit
                                // only until its first synchronized EMPTY, then
                                // hold it there permanently for the remainder
                                // of the handoff.  This creates the all-lane
                                // empty boundary required before the common
                                // application FIFO_RD_EN is enabled.  Do not
                                // assert BS_RESET here: reset release requires
                                // a running source-synchronous DQS and can clip
                                // the first application burst.
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= {BYTE_LANES{1'b0}};
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else begin
                                    wl_handoff_empty_seen <=
                                        wl_handoff_empty_after;
                                    if (!(|calibration_fifo_pop_q) &&
                                        !(|calibration_fifo_word_valid_q) &&
                                        (&wl_handoff_empty_after) &&
                                        !(|wl_handoff_fifo_rd_en_q)) begin
                                        en_vtc_q <= 1'b1;
                                        bitslice_en_vtc_q <= 1'b1;
                                        vtc_settle_counter <=
                                            VTC_SETTLE_CYCLES;
                                        phy_timer <= 4'd15;
                                        wl_handoff_phase <=
                                            WL_HANDOFF_COMPLETE;
                                    end
                                end
                            end

                            WL_HANDOFF_WAIT_VTC_OFF: begin
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= {BYTE_LANES{1'b0}};
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else begin
                                    wl_handoff_gate_coarse <=
                                        gate_trained_coarse[train_lane];
`ifdef SIM_NATIVE_DIAG_WL_DEFER_GATE_RELOAD
                                    // A same-value RL_DLY write is absorbed as
                                    // a no-op and does not reinitialize the
                                    // source-domain gate.  Force one genuine
                                    // fine-tap transfer while remaining one
                                    // tap from the MPR-verified eye center.
                                    wl_handoff_gate_tap <=
                                        (gate_center[train_lane] == 9'd511) ?
                                        9'd510 :
                                        (gate_center[train_lane] + 1'b1);
`else
                                    wl_handoff_gate_tap <=
                                        gate_center[train_lane];
`endif
`ifdef SIM_NATIVE_DIAG_WL_DEFER_GATE_RELOAD
                                    wl_handoff_phase <=
                                        WL_HANDOFF_GATE_RESTORE;
`else
                                    wl_handoff_phase <=
                                        WL_HANDOFF_GATE_REWIND;
`endif
                                end
                            end

                            WL_HANDOFF_GATE_REWIND: begin
                                // RL_DLY is a stateful native delay.  Reuse the
                                // same bounded update sequence that gate
                                // training uses: return fine delay to zero,
                                // advance coarse delay one step at a time, then
                                // restore the trained fine center.  A direct
                                // multi-coarse jump can leave RIU_VALID Low.
                                wl_handoff_gate_tap <= delay_step_toward(
                                    wl_handoff_gate_tap, 9'd0);
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_wr_data <= {
                                    3'd0,
                                    wl_handoff_gate_coarse,
                                    delay_step_toward(
                                        wl_handoff_gate_tap, 9'd0)};
                                native_riu_wr_en <= 1'b1;
                                // RIU_WR_DATA is captured into a shadow
                                // register on this RIU clock.  Do not launch
                                // the source-synchronous transfer edge in the
                                // same cycle: it can precede the pending
                                // shadow update.  The wait state below emits
                                // one legal WL DQS pulse after three complete
                                // DIV_CLK setup cycles.
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= train_lane_mask;
                                phy_timer <= 4'd15;
                                wl_handoff_phase <=
                                    WL_HANDOFF_WAIT_GATE_REWIND;
                            end

                            WL_HANDOFF_WAIT_GATE_REWIND: begin
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= train_lane_mask;
                                if (phy_timer != 0) begin
                                    if (phy_timer == 4'd12)
                                        wl_dqs_strobe <= 1'b1;
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (wl_handoff_readback_ok) begin
                                    if (wl_handoff_gate_tap != 9'd0)
                                        wl_handoff_phase <=
                                            WL_HANDOFF_GATE_REWIND;
                                    else
                                        wl_handoff_phase <=
                                            WL_HANDOFF_GATE_COARSE;
                                end else begin
                                    phy_timer <= 4'd15;
                                end
                            end

                            WL_HANDOFF_GATE_COARSE: begin
                                // The DDR4 one-tCK read preamble is two
                                // half-PLL-clock coarse units.  Move each unit
                                // through a separate RIU transaction.
                                wl_handoff_gate_coarse <=
                                    wl_handoff_gate_coarse + 1'b1;
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_wr_data <= {
                                    3'd0,
                                    wl_handoff_gate_coarse + 1'b1,
                                    9'd0};
                                native_riu_wr_en <= 1'b1;
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= train_lane_mask;
                                phy_timer <= 4'd15;
                                wl_handoff_phase <=
                                    WL_HANDOFF_WAIT_GATE_COARSE;
                            end

                            WL_HANDOFF_WAIT_GATE_COARSE: begin
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= train_lane_mask;
                                if (phy_timer != 0) begin
                                    if (phy_timer == 4'd12)
                                        wl_dqs_strobe <= 1'b1;
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (wl_handoff_readback_ok) begin
                                    if (wl_handoff_gate_coarse !=
                                        (gate_trained_coarse[train_lane] +
`ifdef SIM_NATIVE_DIAG_WL_GATE_PLUS1
                                         4'd1)) begin
`elsif SIM_NATIVE_DIAG_WL_GATE_PLUS3
                                         4'd3)) begin
`elsif SIM_NATIVE_DIAG_WL_GATE_PLUS4
                                         4'd4)) begin
`else
                                         4'd2)) begin
`endif
                                        wl_handoff_phase <=
                                            WL_HANDOFF_GATE_COARSE;
                                    end else begin
                                        wl_handoff_phase <=
                                            WL_HANDOFF_GATE_RESTORE;
                                    end
                                end else begin
                                    phy_timer <= 4'd15;
                                end
                            end

                            WL_HANDOFF_GATE_RESTORE: begin
`ifdef SIM_NATIVE_DIAG_WL_DEFER_GATE_RELOAD
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_wr_data <= {
                                    3'd0,
                                    wl_handoff_gate_coarse,
                                    wl_handoff_gate_tap};
                                native_riu_wr_en <= 1'b1;
`else
                                wl_handoff_gate_tap <= delay_step_toward(
                                    wl_handoff_gate_tap,
                                    gate_center[train_lane]);
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_wr_data <= {
                                    3'd0,
                                    wl_handoff_gate_coarse,
                                    delay_step_toward(
                                        wl_handoff_gate_tap,
                                        gate_center[train_lane])};
                                native_riu_wr_en <= 1'b1;
`endif
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= train_lane_mask;
                                phy_timer <= 4'd15;
                                wl_handoff_phase <=
                                    WL_HANDOFF_WAIT_GATE_RESTORE;
                            end

                            WL_HANDOFF_WAIT_GATE_RESTORE: begin
                                native_riu_addr <= RIU_ADDR_RL_DLY_RNK0;
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= train_lane_mask;
`ifdef SIM_NATIVE_DIAG_WL_DEFER_GATE_RELOAD
                                // Complete the DFI write-level handshake while
                                // the same-value RL_DLY reload remains pending.
                                // RIU_VALID returns only when an external DRAM
                                // DQS edge transfers the shadow register.  Do
                                // not synthesize a local substitute edge.
                                o_dfi_wrlvl_resp <=
                                    i_dfi_wrlvl_en ?
                                    {BYTE_LANES{1'b1}} :
                                    {BYTE_LANES{1'b0}};
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (wl_handoff_readback_ok) begin
                                    en_vtc_q <= 1'b1;
                                    bitslice_en_vtc_q <= 1'b1;
                                    vtc_settle_counter <=
                                        VTC_SETTLE_CYCLES;
                                    native_riu_lower_sel <=
                                        {BYTE_LANES{1'b0}};
                                    native_riu_sel <=
                                        {BYTE_LANES{1'b0}};
                                    phy_state <= PHY_IDLE;
                                end else begin
                                    phy_timer <= 4'd15;
                                end
`else
                                if (phy_timer != 0) begin
                                    if (phy_timer == 4'd12)
                                        wl_dqs_strobe <= 1'b1;
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (wl_handoff_readback_ok) begin
                                    if (wl_handoff_gate_tap !=
                                        gate_center[train_lane]) begin
                                        wl_handoff_phase <=
                                            WL_HANDOFF_GATE_RESTORE;
                                    end else if (train_lane < BYTE_LANES - 1) begin
                                        train_lane <= train_lane + 1'b1;
                                        wl_handoff_gate_coarse <=
                                            gate_trained_coarse[
                                                train_lane + 1'b1];
                                        wl_handoff_gate_tap <=
                                            gate_center[train_lane + 1'b1];
                                        wl_handoff_phase <=
                                            WL_HANDOFF_GATE_REWIND;
                                    end else begin
`ifdef SIM_NATIVE_DIAG_WL_GATE_SHIFT_THEN_RESET_PRIME
                                        // All final RL_DLY values have been
                                        // acknowledged. Reset the RX FIFO at
                                        // that gate phase, then establish one
                                        // disposable complete 8-UI word.
                                        wl_fifo_reset_q <= 1'b1;
                                        wl_handoff_empty_seen <=
                                            {TOTAL_DQ{1'b0}};
                                        wl_handoff_nonempty_seen <=
                                            {TOTAL_DQ{1'b0}};
                                        phy_timer <= 4'd15;
                                        wl_handoff_phase <=
                                            WL_HANDOFF_RX_RESET;
`elsif SIM_NATIVE_DIAG_WL_FINAL_RESET_PRIME
                                        // RIU updates above were transferred
                                        // with legal WL DQS pulses. Consume the
                                        // complete words they produced once,
                                        // then stop each DQ FIFO on its first
                                        // synchronized EMPTY indication.
                                        wl_handoff_empty_seen <=
                                            {TOTAL_DQ{1'b0}};
                                        wl_handoff_nonempty_seen <=
                                            {TOTAL_DQ{1'b0}};
                                        wl_handoff_phase <=
                                            WL_HANDOFF_RX_DRAIN;
`else
                                        en_vtc_q <= 1'b1;
                                        vtc_settle_counter <=
                                            VTC_SETTLE_CYCLES;
                                        native_riu_addr <= RIU_ADDR_BS_CTRL;
                                        native_riu_lower_sel <=
                                            {BYTE_LANES{1'b1}};
                                        native_riu_sel <=
                                            {BYTE_LANES{1'b1}};
                                        phy_timer <= 4'd15;
                                        wl_handoff_phase <=
                                            WL_HANDOFF_COMPLETE;
`endif
                                    end
                                end else begin
                                    phy_timer <= 4'd15;
                                end
`endif
                            end

                            WL_HANDOFF_COMPLETE: begin
                                native_riu_lower_sel <=
                                    {BYTE_LANES{1'b0}};
                                native_riu_sel <= {BYTE_LANES{1'b0}};
                                if (phy_timer != 0) begin
                                    phy_timer <= phy_timer - 1'b1;
                                end else if (all_dly_rdy && all_vtc_rdy) begin
                                    o_dfi_wrlvl_resp <=
                                        {BYTE_LANES{1'b1}};
                                    if (!i_dfi_wrlvl_en) begin
                                        native_riu_lower_sel <=
                                            {BYTE_LANES{1'b0}};
                                        native_riu_sel <=
                                            {BYTE_LANES{1'b0}};
                                        o_dfi_wrlvl_resp <=
                                            {BYTE_LANES{1'b0}};
                                        native_riu_lower_sel <=
                                            {BYTE_LANES{1'b0}};
                                        native_riu_sel <=
                                            {BYTE_LANES{1'b0}};
                                        phy_state <= PHY_IDLE;
                                    end
                                end else begin
                                    phy_timer <= 4'd15;
                                end
                            end

                            default: begin
                                wl_handoff_phase <= WL_HANDOFF_CLEAR;
                            end
                        endcase
                    end

                default: ;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // Native PHY implementation debug
    //
    // Keep implementation-facing signals grouped into packed buses.  Vivado
    // otherwise exposes each unpacked lane/bit array under generated names,
    // which makes a single hardware capture difficult to correlate.  These
    // aliases are observational only and are all sampled with controller_clk
    // when attached to an ILA.
    // -----------------------------------------------------------------
    (* mark_debug = "true" *) wire [2:0] dbg_training_request = {
        i_dfi_wrlvl_en, i_dfi_rdlvl_en, i_dfi_rdlvl_gate_en
    };
    (* mark_debug = "true" *) wire [2*BYTE_LANES-1:0]
        dbg_training_response = {o_dfi_wrlvl_resp, o_dfi_rdlvl_resp};
    (* mark_debug = "true" *) wire [SERDES_RATIO-1:0]
        dbg_dfi_rddata_en = i_dfi_rddata_en;
    (* mark_debug = "true" *) wire [SERDES_RATIO-1:0]
        dbg_dfi_rddata_valid = o_dfi_rddata_valid;
    (* mark_debug = "true" *) wire [3:0] dbg_gate_flags = {
        gate_capture_enable, |calibration_fifo_pop_q,
        |calibration_fifo_word_valid_q, pattern_found_q
    };
    (* mark_debug = "true" *) wire [BYTE_LANES-1:0]
        dbg_calibration_fifo_pop = calibration_fifo_pop_q;
    (* mark_debug = "true" *) wire [BYTE_LANES-1:0]
        dbg_calibration_fifo_word_valid = calibration_fifo_word_valid_q;
    (* mark_debug = "true" *) wire [BYTE_LANES-1:0]
        dbg_fifo_lane_not_empty = fifo_lane_not_empty;
    // Q and FIFO_EMPTY originate in the source-synchronous BITSLICE domain.
    // Register their stable fabric-side values on controller_clk before ILA so
    // Vivado sees an explicit debug clock domain and the probe cannot become a
    // new timing endpoint on the primitive output.
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        dbg_dqs_fifo_empty;
    (* mark_debug = "true" *) reg [BYTE_LANES*8-1:0]
        dbg_dqs_fifo_data;
    always @(posedge i_controller_clk) begin
        if (sync_rst) begin
            dbg_dqs_fifo_empty <= {BYTE_LANES{1'b1}};
            dbg_dqs_fifo_data <= {BYTE_LANES*8{1'b0}};
        end else begin
            dbg_dqs_fifo_empty <= dqs_fifo_empty;
            dbg_dqs_fifo_data <= dqs_fifo_data;
        end
    end

    // Persistent gate-session flight recorder.  A normal ILA window is much
    // shorter than the complete mCL/coarse/fine search, so observing only the
    // live signals can make an early FIFO event look as if it never happened.
    // These controller-clocked registers retain each lane's milestones and
    // the exact search candidate where its DQS FIFO first became non-empty.
    //
    // Only fabric-side BITSLICE outputs are observed.  In particular, DQS
    // DATAIN, PCLK/NCLK and FIFO_WRCLK_OUT must not be tapped: those are
    // dedicated native-I/O routes and adding a fabric load violates Vivado
    // REQP-1922 or consumes unsupported clock routing.
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        dbg_gate_ever_dqs_fifo;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        dbg_gate_ever_dq_fifo;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        dbg_gate_ever_gt_status;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        dbg_gate_ever_mpr_match;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        dbg_gate_ever_mpr_exact;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        dbg_gate_nibble_not_ready;
    (* mark_debug = "true" *) reg [BYTE_LANES*6-1:0]
        dbg_gate_first_dqs_mcl;
    (* mark_debug = "true" *) reg [BYTE_LANES*4-1:0]
        dbg_gate_first_dqs_coarse;
    (* mark_debug = "true" *) reg [BYTE_LANES*9-1:0]
        dbg_gate_first_dqs_tap;
    (* mark_debug = "true" *) reg [15:0] dbg_gate_read_count;
    // Sticky eye evidence remains readable after MPR is disabled.  The first
    // vector proves an exact command-associated MPR word existed somewhere in
    // the sweep; the second proves the selected center reproduced the exact
    // requested word during the independent center verification read.
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        dbg_eye_ever_mpr_exact;
    (* mark_debug = "true" *) reg [BYTE_LANES-1:0]
        dbg_eye_center_verified;

    reg dbg_gate_session_q;
    reg dbg_eye_session_q;
    integer dbg_gate_lane;
    always @(posedge i_controller_clk) begin
        if (sync_rst) begin
            dbg_gate_session_q          <= 1'b0;
            dbg_eye_session_q           <= 1'b0;
            dbg_gate_ever_dqs_fifo      <= {BYTE_LANES{1'b0}};
            dbg_gate_ever_dq_fifo       <= {BYTE_LANES{1'b0}};
            dbg_gate_ever_gt_status     <= {BYTE_LANES{1'b0}};
            dbg_gate_ever_mpr_match     <= {BYTE_LANES{1'b0}};
            dbg_gate_ever_mpr_exact     <= {BYTE_LANES{1'b0}};
            dbg_gate_nibble_not_ready   <= {BYTE_LANES{1'b0}};
            dbg_gate_first_dqs_mcl      <= {BYTE_LANES*6{1'b0}};
            dbg_gate_first_dqs_coarse   <= {BYTE_LANES*4{1'b0}};
            dbg_gate_first_dqs_tap      <= {BYTE_LANES*9{1'b0}};
            dbg_gate_read_count         <= 16'd0;
            dbg_eye_ever_mpr_exact      <= {BYTE_LANES{1'b0}};
            dbg_eye_center_verified     <= {BYTE_LANES{1'b0}};
        end else begin
            dbg_gate_session_q <= i_dfi_rdlvl_gate_en;
            dbg_eye_session_q  <= i_dfi_rdlvl_en;

            // Clear once at the beginning of each complete gate-training
            // session.  The record intentionally survives the later eye and
            // write-level stages so it is still available at init_failed.
            if (i_dfi_rdlvl_gate_en && !dbg_gate_session_q) begin
                dbg_gate_ever_dqs_fifo    <= {BYTE_LANES{1'b0}};
                dbg_gate_ever_dq_fifo     <= {BYTE_LANES{1'b0}};
                dbg_gate_ever_gt_status   <= {BYTE_LANES{1'b0}};
                dbg_gate_ever_mpr_match   <= {BYTE_LANES{1'b0}};
                dbg_gate_ever_mpr_exact   <= {BYTE_LANES{1'b0}};
                dbg_gate_nibble_not_ready <= {BYTE_LANES{1'b0}};
                dbg_gate_first_dqs_mcl    <= {BYTE_LANES*6{1'b0}};
                dbg_gate_first_dqs_coarse <= {BYTE_LANES*4{1'b0}};
                dbg_gate_first_dqs_tap    <= {BYTE_LANES*9{1'b0}};
                dbg_gate_read_count       <= 16'd0;
            end else if (i_dfi_rdlvl_gate_en) begin
                if ((|dfi_read_command) && !(&dbg_gate_read_count))
                    dbg_gate_read_count <= dbg_gate_read_count + 1'b1;

                for (dbg_gate_lane = 0;
                     dbg_gate_lane < BYTE_LANES;
                     dbg_gate_lane = dbg_gate_lane + 1) begin
                    if (!dqs_fifo_empty[dbg_gate_lane]) begin
                        // Capture the first candidate only; later events set
                        // the sticky bit without overwriting the evidence.
                        if (!dbg_gate_ever_dqs_fifo[dbg_gate_lane]) begin
                            dbg_gate_first_dqs_mcl[
                                dbg_gate_lane*6 +: 6
                            ] <= gate_sweep_mcl;
                            dbg_gate_first_dqs_coarse[
                                dbg_gate_lane*4 +: 4
                            ] <= gate_sweep_coarse;
                            dbg_gate_first_dqs_tap[
                                dbg_gate_lane*9 +: 9
                            ] <= gate_sweep_tap;
                        end
                        dbg_gate_ever_dqs_fifo[dbg_gate_lane] <= 1'b1;
                    end
                    if (fifo_lane_not_empty[dbg_gate_lane])
                        dbg_gate_ever_dq_fifo[dbg_gate_lane] <= 1'b1;
                    if (gate_status_seen[dbg_gate_lane])
                        dbg_gate_ever_gt_status[dbg_gate_lane] <= 1'b1;
                    if (gate_pattern_found[dbg_gate_lane])
                        dbg_gate_ever_mpr_match[dbg_gate_lane] <= 1'b1;
                    if (lane_mpr_word_match[dbg_gate_lane])
                        dbg_gate_ever_mpr_exact[dbg_gate_lane] <= 1'b1;
                    if (dbg_byte_nibble_ready[
                            dbg_gate_lane*4 +: 4
                        ] != 4'b1111)
                        dbg_gate_nibble_not_ready[dbg_gate_lane] <= 1'b1;
                end
            end

            if (i_dfi_rdlvl_en && !dbg_eye_session_q) begin
                dbg_eye_ever_mpr_exact  <= {BYTE_LANES{1'b0}};
                dbg_eye_center_verified <= {BYTE_LANES{1'b0}};
            end else if (i_dfi_rdlvl_en) begin
                if (lane_mpr_word_match[train_lane])
                    dbg_eye_ever_mpr_exact[train_lane] <= 1'b1;
                if ((phy_state == PHY_EYE_VERIFY_DONE) &&
                    pattern_found_q)
                    dbg_eye_center_verified[train_lane] <= 1'b1;
            end
        end
    end
    (* mark_debug = "true" *) wire [3:0] dbg_eye_flags = {
        eye_observe_verify, eye_observe_seen, in_range, best_valid
    };
    (* mark_debug = "true" *) wire [4:0] dbg_wl_flags = {
        wl_active, wl_dqs_strobe, wl_feedback_valid,
        wl_feedback_zero, wl_feedback_one
    };
    (* mark_debug = "true" *) wire [4:0] dbg_rx_path_flags = {
        app_read_issued, app_fifo_pop_request, app_pop_fire,
        app_returns_queued, calibration_session
    };
    // Application FIFO policy, ordered MSB-to-LSB:
    //   [7] read-due token present, [6] READ reservation present,
    //   [5] reserved (independent FIFO popping is intentionally forbidden),
    //   [4] output_enable, [3] output_enable_q, [2] returns_queued,
    //   [1] read_issued, [0] read_due_event.
    (* mark_debug = "true" *) wire [7:0] dbg_write_flush_flags = {
        (app_read_due != 0), (app_read_pending != 0),
        1'b0, output_enable, output_enable_q,
        app_returns_queued, app_read_issued, app_read_due_event
    };
    // Actual primitive-reset ownership and application turnaround evidence:
    //   [7] applied RX_RST, [6] global BITSLICE reset, [5] calibration flush,
    //   [4] WL reset, [3] old cleanup window, [2] any PHY_RDEN,
    //   [1] output enable, [0] READ accepted.
    (* mark_debug = "true" *) wire [7:0] dbg_rx_reset_flags = {
        rx_fifo_reset_active, bitslice_rst, rx_fifo_flush,
        wl_rx_fifo_reset, app_write_cleanup_window,
        |phy_rden_ready, output_enable, app_read_issued
    };
    // Records any DQ FIFO that reported non-empty during TX. This is
    // diagnostic only; the functional path never advances bits independently.
    (* mark_debug = "true" *) reg [TOTAL_DQ-1:0]
        dbg_tx_fifo_nonempty;
    always @(posedge i_controller_clk) begin
        if (sync_rst || (output_enable && !output_enable_q))
            dbg_tx_fifo_nonempty <= {TOTAL_DQ{1'b0}};
        else if (output_enable && (app_read_pending == 0))
            dbg_tx_fifo_nonempty <= dbg_tx_fifo_nonempty |
                                    ~fifo_empty_flat;
    end
    // Final application-return boundary.  Raw FIFO Q is already available in
    // dbg_rx_dq_words; these probes distinguish a primitive capture problem
    // from the synchronous return transfer and DFI packing without tapping any
    // dedicated BITSLICE route.
    (* mark_debug = "true" *) wire [SERDES_RATIO*DFI_DATA_WIDTH-1:0]
        dbg_dfi_rddata = o_dfi_rddata;
    (* mark_debug = "true" *) wire [3:0] dbg_app_return_flags = {
        |o_dfi_rddata_valid, fifo_pace_not_empty,
        app_fifo_pop_fire, app_fifo_pop_request
    };
    // Native-TX boundary diagnostic.  These are aliases of existing DFI and
    // serializer signals only; they add no state or functional muxing to the
    // write path.  Together they show whether a WRITE command, its advertised
    // DFI data word, the registered native word, byte mask, packed lane-0 DQ,
    // DQS pattern, and TBYTE ownership all occupy the intended DIV_CLK slots.
    // Probing four lane-0 DQ serializers is sufficient because the AXKU3 BIST
    // pattern repeats every 32 bits across all four byte lanes.
    (* mark_debug = "true" *) wire [SERDES_RATIO-1:0]
        dbg_dfi_write_command =
            (~i_dfi_cs_n) & i_dfi_act_n & i_dfi_ras_n &
            (~i_dfi_cas_n) & (~i_dfi_we_n);
    (* mark_debug = "true" *) wire [SERDES_RATIO-1:0]
        dbg_dfi_wrdata_en = i_dfi_wrdata_en;
    (* mark_debug = "true" *) wire [31:0] dbg_dfi_wrdata_lo =
        i_dfi_wrdata[31:0];
    (* mark_debug = "true" *) wire [31:0] dbg_tx_wrdata_lo =
        tx_wrdata_native[31:0];
    (* mark_debug = "true" *) wire [31:0] dbg_tx_wrdata_early_lo =
        tx_wrdata_early[31:0];
    (* mark_debug = "true" *) wire [31:0] dbg_tx_wrdata_pipe0_lo =
        tx_wrdata_pipe0[31:0];
    (* mark_debug = "true" *) wire [31:0] dbg_tx_lane0_dq_lo =
        tx_dq_data[0][31:0];
    // The DDR4-1600 hardware failure isolated to lane 0 DQ6, UI1.  Preserve
    // that serializer's complete eight-UI parallel word rather than relying
    // on the low-32-bit lane alias, which contains only DQ0..DQ3.
    (* mark_debug = "true" *) wire [7:0] dbg_tx_lane0_dq6 =
        tx_dq_data[0][6*8 +: 8];
    // Focused DDR4-1600 TX-boundary instrumentation.  These three words show
    // whether lane-0 DQ6 changed before, inside, or after the native first-word
    // predrive mux.  Keeping the same UI ordering as tx_dq_data makes a saved
    // ILA capture directly comparable with the eight bits serialized at the
    // pad; none of these aliases participate in functional logic.
    (* mark_debug = "true" *) wire [7:0] dbg_dfi_wrdata_dq6 = {
        i_dfi_wrdata[3*DFI_DATA_WIDTH + TOTAL_DQ + 6],
        i_dfi_wrdata[3*DFI_DATA_WIDTH + 6],
        i_dfi_wrdata[2*DFI_DATA_WIDTH + TOTAL_DQ + 6],
        i_dfi_wrdata[2*DFI_DATA_WIDTH + 6],
        i_dfi_wrdata[1*DFI_DATA_WIDTH + TOTAL_DQ + 6],
        i_dfi_wrdata[1*DFI_DATA_WIDTH + 6],
        i_dfi_wrdata[0*DFI_DATA_WIDTH + TOTAL_DQ + 6],
        i_dfi_wrdata[0*DFI_DATA_WIDTH + 6]
    };
    (* mark_debug = "true" *) wire [7:0] dbg_tx_wrdata_early_dq6 = {
        tx_wrdata_early[3*DFI_DATA_WIDTH + TOTAL_DQ + 6],
        tx_wrdata_early[3*DFI_DATA_WIDTH + 6],
        tx_wrdata_early[2*DFI_DATA_WIDTH + TOTAL_DQ + 6],
        tx_wrdata_early[2*DFI_DATA_WIDTH + 6],
        tx_wrdata_early[1*DFI_DATA_WIDTH + TOTAL_DQ + 6],
        tx_wrdata_early[1*DFI_DATA_WIDTH + 6],
        tx_wrdata_early[0*DFI_DATA_WIDTH + TOTAL_DQ + 6],
        tx_wrdata_early[0*DFI_DATA_WIDTH + 6]
    };
    (* mark_debug = "true" *) wire [7:0] dbg_tx_wrdata_pipe0_dq6 = {
        tx_wrdata_pipe0[3*DFI_DATA_WIDTH + TOTAL_DQ + 6],
        tx_wrdata_pipe0[3*DFI_DATA_WIDTH + 6],
        tx_wrdata_pipe0[2*DFI_DATA_WIDTH + TOTAL_DQ + 6],
        tx_wrdata_pipe0[2*DFI_DATA_WIDTH + 6],
        tx_wrdata_pipe0[1*DFI_DATA_WIDTH + TOTAL_DQ + 6],
        tx_wrdata_pipe0[1*DFI_DATA_WIDTH + 6],
        tx_wrdata_pipe0[0*DFI_DATA_WIDTH + TOTAL_DQ + 6],
        tx_wrdata_pipe0[0*DFI_DATA_WIDTH + 6]
    };
    (* mark_debug = "true" *) wire [7:0] dbg_tx_dqs_pattern = dqs_pattern;
    (* mark_debug = "true" *) wire [2:0] dbg_tx_wrdata_en_shift =
        wrdata_en_shift;
    (* mark_debug = "true" *) wire [1:0] dbg_tx_mask_any = {
        |tx_wrmask_native, |i_dfi_wrdata_mask
    };
    (* mark_debug = "true" *) wire [7:0] dbg_tx_dm_lane0 = tx_dm_data[0];
    (* mark_debug = "true" *) wire [3:0] dbg_gate_observe_count =
        gate_observe_count;
    (* mark_debug = "true" *) wire [3:0] dbg_eye_observe_count =
        eye_observe_count;
    (* mark_debug = "true" *) wire [5:0] dbg_active_read_mcl =
        active_read_mcl;
    (* mark_debug = "true" *) wire [SERDES_RATIO-1:0]
        dbg_phy_rden_mask = phy_rden_mask[SERDES_RATIO-1:0];
    (* mark_debug = "true" *) wire [BYTE_LANES*SERDES_RATIO-1:0]
        dbg_phy_rden_mask_all = phy_rden_mask;
    (* mark_debug = "true" *) wire [3:0] dbg_gate_sweep_coarse =
        gate_sweep_coarse;
    (* mark_debug = "true" *) wire [2:0] dbg_gate_mcl_index =
        gate_mcl_index;
    (* mark_debug = "true" *) wire [BYTE_LANES-1:0]
        dbg_gate_lane_resolved = gate_lane_resolved;
    (* mark_debug = "true" *) wire [BYTE_LANES-1:0]
        dbg_gate_pattern_found = gate_pattern_found;
    (* mark_debug = "true" *) wire [BYTE_LANES-1:0]
        dbg_lane_mpr_word_match = lane_mpr_word_match;
    (* mark_debug = "true" *) wire [BYTE_LANES*DQ_BITS-1:0]
        dbg_lane_mpr_bit_match;
    (* mark_debug = "true" *) wire [BYTE_LANES*16-1:0]
        dbg_native_riu_rd_data;
    (* mark_debug = "true" *) wire [TOTAL_DQ*8-1:0]
        dbg_rx_dq_words;
    (* mark_debug = "true" *) wire [TOTAL_DQ-1:0]
        dbg_fifo_rd_en;
    (* mark_debug = "true" *) wire [BYTE_LANES*6-1:0]
        dbg_gate_trained_mcl;
    (* mark_debug = "true" *) wire [BYTE_LANES*6-1:0]
        dbg_gate_trained_mcl_low;
    (* mark_debug = "true" *) wire [BYTE_LANES*4-1:0]
        dbg_gate_trained_coarse;
    (* mark_debug = "true" *) wire [BYTE_LANES*9-1:0]
        dbg_odelay_dqs_countout;
    (* mark_debug = "true" *) wire [BYTE_LANES*9-1:0]
        dbg_gate_cur_start;
    (* mark_debug = "true" *) wire [BYTE_LANES*9-1:0]
        dbg_gate_cur_width;
    (* mark_debug = "true" *) wire [BYTE_LANES*9-1:0]
        dbg_gate_best_start;
    (* mark_debug = "true" *) wire [BYTE_LANES*9-1:0]
        dbg_gate_best_width;
    (* mark_debug = "true" *) wire [BYTE_LANES*9-1:0]
        dbg_gate_center;
    (* mark_debug = "true" *) wire [BYTE_LANES*9-1:0]
        dbg_eye_center;
    (* mark_debug = "true" *) wire [BYTE_LANES*9-1:0]
        dbg_eye_best_start;
    (* mark_debug = "true" *) wire [BYTE_LANES*9-1:0]
        dbg_eye_best_width;
    (* mark_debug = "true" *) wire [BYTE_LANES*9-1:0]
        dbg_wl_dqs_tap;
    (* mark_debug = "true" *) wire [BYTE_LANES*9-1:0]
        dbg_wl_dq_tap;
    (* mark_debug = "true" *) wire [BYTE_LANES*4-1:0]
        dbg_wl_coarse;
    (* mark_debug = "true" *) wire [BYTE_LANES*9-1:0]
        dbg_dqs_initial_tap;
    (* mark_debug = "true" *) wire [BYTE_LANES*4-1:0]
        dbg_bitslip;

    generate
        genvar ila_lane, ila_bit;
        for (ila_lane = 0; ila_lane < BYTE_LANES;
             ila_lane = ila_lane + 1) begin : gen_native_ila_lane
            assign dbg_native_riu_rd_data[ila_lane*16 +: 16] =
                native_riu_rd_data[ila_lane];
            assign dbg_fifo_rd_en[ila_lane*DQ_BITS +: DQ_BITS] =
                fifo_rd_en_drive[ila_lane];
            assign dbg_gate_trained_mcl[ila_lane*6 +: 6] =
                gate_trained_mcl[ila_lane];
            assign dbg_gate_trained_mcl_low[ila_lane*6 +: 6] =
                gate_trained_mcl_low[ila_lane];
            assign dbg_gate_trained_coarse[ila_lane*4 +: 4] =
                gate_trained_coarse[ila_lane];
            assign dbg_odelay_dqs_countout[ila_lane*9 +: 9] =
                odelay_dqs_cntvalueout[ila_lane];
            assign dbg_gate_cur_start[ila_lane*9 +: 9] =
                gate_cur_start[ila_lane];
            assign dbg_gate_cur_width[ila_lane*9 +: 9] =
                gate_cur_width[ila_lane];
            assign dbg_gate_best_start[ila_lane*9 +: 9] =
                gate_best_start[ila_lane];
            assign dbg_gate_best_width[ila_lane*9 +: 9] =
                gate_best_width[ila_lane];
            assign dbg_gate_center[ila_lane*9 +: 9] =
                gate_center[ila_lane];
            assign dbg_eye_center[ila_lane*9 +: 9] =
                eye_center_tap[ila_lane];
            assign dbg_eye_best_start[ila_lane*9 +: 9] =
                eye_best_start[ila_lane];
            assign dbg_eye_best_width[ila_lane*9 +: 9] =
                eye_best_width[ila_lane];
            assign dbg_wl_dqs_tap[ila_lane*9 +: 9] = wl_tap[ila_lane];
            assign dbg_wl_dq_tap[ila_lane*9 +: 9] = wl_dq_tap[ila_lane];
            assign dbg_wl_coarse[ila_lane*4 +: 4] = wl_coarse[ila_lane];
            assign dbg_dqs_initial_tap[ila_lane*9 +: 9] =
                dqs_initial_tap[ila_lane];
            assign dbg_bitslip[ila_lane*4 +: 4] = bitslip_count_q[ila_lane];
            for (ila_bit = 0; ila_bit < DQ_BITS;
                 ila_bit = ila_bit + 1) begin : gen_native_ila_bit
                assign dbg_lane_mpr_bit_match[ila_lane*DQ_BITS + ila_bit] =
                    lane_mpr_bit_match[ila_lane][ila_bit];
                assign dbg_rx_dq_words[
                    (ila_lane*DQ_BITS + ila_bit)*8 +: 8
                ] = iserdes_dq_q[ila_lane*DQ_BITS + ila_bit];
            end
        end
    endgenerate

    // -----------------------------------------------------------------
    // Public debug status
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
