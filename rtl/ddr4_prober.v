////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_prober.v
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  Combined BIST engine and debug CSR register file. The BIST
//  exercises the DDR4 data path via sequential (burst), stress-addressed
//  (row/bank thrashing), and alternating write/read patterns through a
//  Wishbone B4 master port. The debug CSR provides read-only register
//  access to controller, PHY, and BIST status.
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

module ddr4_prober #(
    parameter WB_ADDR_BITS       = 27,
              WB_DATA_BITS       = 128,
              WB_SEL_BITS        = WB_DATA_BITS / 8,
    // Number of 8-bit byte lanes (typically 2 for x8, 2 for x16, 2+ for x4)
              BYTE_LANES         = 2,
              NUM_BANKS          = 16,
              /* verilator lint_off UNUSEDPARAM */
              ROW_BITS           = 16,
              /* verilator lint_on UNUSEDPARAM */
    // Set to 1 when simulating with Micron DDR4 model (adjusts timing checks)
    parameter[0:0] MICRON_SIM    = 0,
    // BIST_MODE: 0=disabled, 1=half-range (each phase covers half the address space),
    //            2=full-range (each phase covers the full address space)
    parameter[1:0] BIST_MODE     = 2,
    // BIST burst write per-byte-lane masking: 0=disabled (full-word writes),
    // 1=enabled (cycles through byte lanes one at a time to stress DM path)
    parameter[0:0] BIST_DM_TEST     = 1,
    // Hardware diagnostic: after the first BIST mismatch, drain all older
    // requests and reread that exact address repeatedly without rewriting it.
    // This distinguishes a stored/write-path error from an intermittent
    // receive-path error. Keep disabled in production builds.
    parameter[0:0] BIST_REREAD_DIAG = 0,
    // Debug CSR register file: 0=disabled (saves area), 1=enabled
    parameter      DEBUG_CSR_ENABLE = 1
) (
    input  wire                     i_clk,              // System clock (same as controller clock)
    input  wire                     i_rst_n,            // Active-low synchronous reset
    // Calibration status from controller
    (* mark_debug = "true" *) input  wire                     i_calib_complete,   // Pulses high when DDR4 init + PHY training finishes successfully
    (* mark_debug = "true" *) input  wire                     i_calib_error,      // High if calibration retries exhausted (unrecoverable training failure)
    // Init status (sticky — set once after calibration + optional BIST)
    (* mark_debug = "true" *) output reg                      o_init_done,        // Latches high once calibration passes AND BIST passes (or BIST disabled)
    (* mark_debug = "true" *) output reg                      o_init_failed,      // Latches high on calibration error OR BIST data mismatch; mutually exclusive with o_init_done
    // BIST status
    (* mark_debug = "true" *) output wire                     o_bist_busy,        // High while BIST FSM is actively issuing/checking memory transactions
    (* mark_debug = "true" *) output reg                      o_bist_failed_reset_req,   // Auto-reset request: asserted after BIST fail when CSR auto_reset_en is set
    (* mark_debug = "true" *) output reg                      o_soft_reset_req,   // CSR-triggered one-shot: resets controller + PHY to re-run full calibration
    // Wishbone B4 Master — BIST drives this to issue R/W to the DDR4 controller
    output reg                      o_wb_cyc,           // Bus cycle active (held high for entire BIST transaction burst)
    output reg                      o_wb_stb,           // Strobe: valid request on addr/data/we this cycle
    output reg                      o_wb_we,            // Write enable: 1=write, 0=read
    output reg  [WB_ADDR_BITS-1:0]  o_wb_addr,          // DDR4 word address
    output reg  [WB_DATA_BITS-1:0]  o_wb_data,          // Write data (full cache-line width)
    output reg  [WB_SEL_BITS-1:0]   o_wb_sel,           // Byte-lane select (one-hot when BIST_DM_TEST=1, all-ones otherwise)
    input  wire                     i_wb_stall,         // Backpressure from controller: request not accepted this cycle
    input  wire                     i_wb_ack,           // Acknowledge: read data valid or write committed
    input  wire [WB_DATA_BITS-1:0]  i_wb_data,          // Read data returned by controller
    // Wishbone B4 — Debug CSR port (pipelined, zero-wait-state, independent of DRAM path)
    input  wire                     i_wb_dbg_cyc,       // CSR bus cycle
    input  wire                     i_wb_dbg_stb,       // CSR strobe
    input  wire                     i_wb_dbg_we,        // CSR write enable
    input  wire [3:0]               i_wb_dbg_addr,      // CSR register address (selects 1 of 16 registers)
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0]              i_wb_dbg_data,      // CSR write data (only [2:0] used for control reg)
    input  wire [3:0]               i_wb_dbg_sel,       // CSR byte select (unused, always full-word)
    /* verilator lint_on UNUSEDSIGNAL */
    output wire                     o_wb_dbg_stall,     // Always 0: CSR port never stalls
    output reg                      o_wb_dbg_ack,       // Registered ACK (1-cycle latency)
    output reg  [31:0]              o_wb_dbg_data,      // CSR read data
    // Status from controller (exposed via CSR for debug visibility)
    (* mark_debug = "true" *) input  wire [3:0]               i_calib_state,      // Controller calibration FSM state (0=IDLE..13=DONE, 14=ERROR)
    (* mark_debug = "true" *) input  wire                     i_stage1_pending,   // A new WB request is latched, waiting for stage 2
    (* mark_debug = "true" *) input  wire                     i_stage2_pending,   // A decoded request is being scheduled (issuing PRE/ACT/RD/WR)
    (* mark_debug = "true" *) input  wire                     i_stage2_we,        // Stage 2 request type: 1=write, 0=read
    (* mark_debug = "true" *) input  wire                     i_refresh_idle,     // Refresh timer in idle countdown — scheduler free to issue user commands
    (* mark_debug = "true" *) input  wire [NUM_BANKS-1:0]     i_bank_status,      // Per-bank status: 1=row open (active), 0=precharged (idle)
    // Status from PHY (flat packed, exposed via CSR for training debug)
    (* mark_debug = "true" *) input  wire [3:0]               i_phy_state,        // PHY training FSM state (0=IDLE, 3=GATE_DONE, 7=EYE_DONE, 11=WL_DONE)
    input  wire [9*BYTE_LANES-1:0]  i_phy_idelay_center, // 9b per lane: IDELAY tap at center of read data eye
    input  wire [9*BYTE_LANES-1:0]  i_phy_wl_tap,       // 9b per lane: ODELAY tap where DQS aligns to CK at DRAM
    input  wire [4*BYTE_LANES-1:0]  i_phy_bitslip,      // 4b per lane: ISERDES barrel-shift aligning capture to burst boundary
    input  wire [3*BYTE_LANES-1:0]  i_phy_train_fail,   // Per lane: {wl_fail, eye_fail, gate_fail} — sticky failure flags
    // Extended training debug (CSR 0x8, 0x9, 0xD, 0xE readback)
    input  wire [9*BYTE_LANES-1:0]  i_phy_best_width,   // 9b per lane: eye width in IDELAY taps
    input  wire [9*BYTE_LANES-1:0]  i_phy_best_start,   // 9b per lane: first passing IDELAY tap
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [9*BYTE_LANES-1:0]  i_phy_wl_dq_tap,    // 9b per lane: DQ ODELAYE3 tap after WL (MSB truncated in CSR)
    input  wire [9*BYTE_LANES-1:0]  i_phy_dqs_initial_tap, // 9b per lane: BISC-calibrated DQS baseline (MSB truncated in CSR)
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [BYTE_LANES-1:0]    i_phy_rd_lat_extra,  // 1b per lane: read data arrives 1 CLKDIV late
    (* mark_debug = "true" *) input  wire                     i_phy_en_vtc,        // 1 = voltage-temperature compensation active
    (* mark_debug = "true" *) input  wire [5:0]               i_instruction_address, // ROM step 0-35 (init progress)
    (* mark_debug = "true" *) input  wire                     i_pause_counter,     // 1 = ROM frozen by training FSM
    (* mark_debug = "true" *) input  wire                     i_reset_done,        // 1 = init ROM completed
    (* mark_debug = "true" *) input  wire                     i_pipe_stall,        // 1 = WB pipeline stalled
    (* mark_debug = "true" *) input  wire [1:0]               i_calib_retry_count, // Training retry attempts (0-3)
    // Native-PHY post-failure TX-eye diagnostic handshake.  These remain
    // inactive in component mode and in normal production builds where
    // BIST_REREAD_DIAG is disabled.
    input  wire                     i_phy_tx_diag_supported,
    (* mark_debug = "true" *) output reg                      o_phy_tx_diag_req,
    (* mark_debug = "true" *) output reg  [7:0]               o_phy_tx_diag_dq,
    (* mark_debug = "true" *) output reg  [8:0]               o_phy_tx_diag_tap,
    (* mark_debug = "true" *) input  wire                     i_phy_tx_diag_ack,
    (* mark_debug = "true" *) input  wire                     i_phy_tx_diag_error,
    (* mark_debug = "true" *) input  wire [8:0]               i_phy_tx_diag_current_tap,
    (* mark_debug = "true" *) input  wire [8:0]               i_phy_tx_diag_previous_tap
);

    // -----------------------------------------------------------------
    // Lane-organized PHY debug probes
    //
    // The PHY status ports above are compact CSR transports. Marking those
    // packed buses directly makes ILA display one hard-to-read aggregate
    // probe. These aliases only name individual slices; they infer no logic
    // and preserve the CSR interface unchanged. Each generated lane appears
    // in ILA as gen_phy_lane_debug[n].<signal>.
    // -----------------------------------------------------------------
    (* mark_debug = "true" *) wire [BYTE_LANES-1:0] phy_train_fail_eye = i_phy_train_fail[2*BYTE_LANES-1:BYTE_LANES];
    (* mark_debug = "true" *) wire [BYTE_LANES-1:0] phy_train_fail_wl = i_phy_train_fail[3*BYTE_LANES-1:2*BYTE_LANES];

    generate
        genvar dbg_lane;
        for (dbg_lane = 0; dbg_lane < BYTE_LANES; dbg_lane = dbg_lane + 1) begin : gen_phy_lane_debug
            (* mark_debug = "true" *) wire [8:0] phy_idelay_center_lane = i_phy_idelay_center[dbg_lane*9 +: 9];
            (* mark_debug = "true" *) wire [8:0] phy_wl_dqs_tap_lane    = i_phy_wl_tap[dbg_lane*9 +: 9];
            (* mark_debug = "true" *) wire [8:0] phy_wl_dq_tap_lane     = i_phy_wl_dq_tap[dbg_lane*9 +: 9];
            (* mark_debug = "true" *) wire [8:0] phy_dqs_initial_tap_lane = i_phy_dqs_initial_tap[dbg_lane*9 +: 9];
            (* mark_debug = "true" *) wire [8:0] phy_eye_width_lane     = i_phy_best_width[dbg_lane*9 +: 9];
            (* mark_debug = "true" *) wire [8:0] phy_eye_start_lane     = i_phy_best_start[dbg_lane*9 +: 9];
            (* mark_debug = "true" *) wire [3:0] phy_bitslip_lane       = i_phy_bitslip[dbg_lane*4 +: 4];
            (* mark_debug = "true" *) wire       phy_rd_lat_extra_lane  = i_phy_rd_lat_extra[dbg_lane];
        end
    endgenerate

    // -----------------------------------------------------------------
    // BIST Architecture Overview
    // -----------------------------------------------------------------
    // The BIST engine exercises the DDR4 data path in three phases:
    //
    //   Phase 1 - Burst:       Sequential write then sequential
    //                          read-back (streaming throughput test).
    //   Phase 2 - Stress:      Writes then reads using stress_addr()
    //                          mapping that distributes counter bits
    //                          across row/bank/BG fields, forcing
    //                          precharge/activate on nearly every
    //                          access (timing margin stress test).
    //   Phase 3 - Alternating: Write one address, immediately read it
    //                          back, repeat (tight write-to-read
    //                          turnaround test).
    //
    // All three phases always run.  BIST_MODE controls addressing:
    //   0 = disabled (BIST does not run)
    //   1 = partitioned: phases cover contiguous, non-overlapping
    //       slices that together span the full BIST address range
    //       (burst: 0→BURST_END, random: BURST_END+1→RANDOM_END,
    //        alt: RANDOM_END+1→ALT_END)
    //   2 = full-range: every phase independently covers the entire
    //       address space starting from 0
    //
    // BIST can be triggered two ways:
    //   1. Auto-start -- on rising edge of i_calib_complete from ddr4_top
    //   2. CSR trigger -- software writes 1 to bit[0] of CSR register 0xC.
    //
    // The engine uses Wishbone B4 pipelined mode, tracking outstanding
    // requests so it can overlap writes with read-back ACKs.
    // -----------------------------------------------------------------

    // -----------------------------------------------------------------
    // BIST FSM States
    // -----------------------------------------------------------------
    localparam [2:0] BIST_IDLE           = 3'd0,
                     BIST_BURST_WRITE    = 3'd1,
                     BIST_BURST_READ     = 3'd2,
                     BIST_RANDOM_WRITE   = 3'd3,
                     BIST_RANDOM_READ    = 3'd4,
                     BIST_ALT_WRITE_READ = 3'd5,
                     BIST_FINISH         = 3'd6,
                     BIST_DONE           = 3'd7;

    localparam BIST_ADDR_BITS = MICRON_SIM ? 10 : WB_ADDR_BITS;
    // Phase end addresses:
    //   Mode 1: partitioned — burst gets 1/4, random gets 1/2, alt gets 1/4
    //   Mode 2: full-range  — all three END values equal the max address
    localparam [BIST_ADDR_BITS-1:0] BURST_END  = {{2{BIST_MODE[1]}}, {(BIST_ADDR_BITS-2){1'b1}}};
    localparam [BIST_ADDR_BITS-1:0] RANDOM_END = {1'b1, BIST_MODE[1], {(BIST_ADDR_BITS-2){1'b1}}};
    localparam [BIST_ADDR_BITS-1:0] ALT_END    = {BIST_ADDR_BITS{1'b1}};

    (* mark_debug = "true" *) reg [2:0]  bist_state;
    (* mark_debug = "true" *) reg [31:0] correct_count;
    (* mark_debug = "true" *) reg [31:0] error_count;
    (* mark_debug = "true" *) reg        bist_fail_sticky;
    (* mark_debug = "true" *) reg        auto_reset_en;
    localparam [3:0] BIST_AUTO_RESET_MAX = 4'd15;
    (* mark_debug = "true" *) reg [3:0]  bist_auto_reset_count;
    reg                                  bist_failed_reset_req_q;
    (* mark_debug = "true" *) wire       bist_pass;
    wire bist_diag_active;
    wire bist_auto_reset_available = auto_reset_en &&
        (bist_auto_reset_count < BIST_AUTO_RESET_MAX);

    // Module-scope CSR write-enable decode (visible to both gen_bist and gen_csr)
    wire csr_we = i_wb_dbg_cyc && i_wb_dbg_stb && i_wb_dbg_we;

    // -----------------------------------------------------------------
    // BIST Auto-Start (rising edge of calib_complete when BIST enabled)
    // -----------------------------------------------------------------
    wire prober_internal_reset = !i_rst_n || o_soft_reset_req || o_bist_failed_reset_req;

    // Keep recovery history across the internal DDR reset it requests.  This
    // makes a recovered final PASS auditable and also bounds repeated recovery
    // when a persistent hardware fault cannot be repaired by retraining.
    always @(posedge i_clk) begin
        if (!i_rst_n) begin
            bist_auto_reset_count <= 4'd0;
            bist_failed_reset_req_q <= 1'b0;
        end else begin
            bist_failed_reset_req_q <= o_bist_failed_reset_req;
            if (o_bist_failed_reset_req && !bist_failed_reset_req_q &&
                (bist_auto_reset_count < BIST_AUTO_RESET_MAX))
                bist_auto_reset_count <= bist_auto_reset_count + 1'b1;
        end
    end

    reg calib_complete_q;
    always @(posedge i_clk) begin
        if (prober_internal_reset) begin // re-arm edge detector after soft/BIST reset
            calib_complete_q <= 1'b0;
        end
        else begin
            calib_complete_q <= i_calib_complete;
        end
    end
    wire bist_auto_start = i_calib_complete && !calib_complete_q && (BIST_MODE != 0);

    // -----------------------------------------------------------------
    // BIST Logic (gated by BIST_MODE)
    // -----------------------------------------------------------------
    generate if (BIST_MODE != 0) begin : gen_bist

        reg bist_csr_start_r, bist_csr_start_d;
        always @(posedge i_clk) begin
            if (!i_rst_n) begin
                bist_csr_start_r <= 1'b0;
                bist_csr_start_d <= 1'b0;
                o_soft_reset_req <= 1'b0;
                auto_reset_en    <= 1'b1;
            end else begin
                bist_csr_start_d <= bist_csr_start_r;
                bist_csr_start_r <= 1'b0;
                o_soft_reset_req <= 1'b0;
                if (csr_we && i_wb_dbg_addr == 4'hC) begin  // CSR register 0xC (Control)
                    if (i_wb_dbg_data[0]) begin// CSR bit[0] = 1 triggers BIST re-start (W1S)
                        bist_csr_start_r <= 1'b1;
                    end
                    if (i_wb_dbg_data[1]) begin// CSR bit[1] = 1 triggers soft reset (W1S)
                        o_soft_reset_req <= 1'b1;
                    end
                    auto_reset_en <= i_wb_dbg_data[2]; // CSR bit[2] = 1 enables auto-reset on BIST fail (R/W)
                end
            end
        end


        // Start from auto-start (calib_complete edge) or CSR write.
        // Two-stage register catches a single-cycle CSR pulse reliably.
        wire bist_start_any = bist_auto_start || bist_csr_start_r || bist_csr_start_d;

        // Keep the three BIST cursors independently visible.  In particular,
        // write_addr lets an ILA trigger on the original memory write that is
        // later identified by diag_fail_addr; observing only the failure-time
        // read cannot distinguish a stored TX error from an RX return error.
        (* mark_debug = "true" *) reg [BIST_ADDR_BITS-1:0] write_addr;
        (* mark_debug = "true" *) reg [BIST_ADDR_BITS-1:0] read_addr;
        (* mark_debug = "true" *) reg [BIST_ADDR_BITS-1:0] check_addr;
        reg alt_phase;
        reg last_read_scrambled;
        reg [$clog2(WB_SEL_BITS)-1:0] write_byte_counter;

        localparam integer BIST_DIAG_READS = 32;
        localparam integer BIST_DIAG_STREAM_WRITES = 64;
        // A tap must survive both maximum simultaneous switching and the
        // data-dependent neighbour/UI combinations of normal traffic.  Keep
        // every stress word at a distinct address so no earlier error is
        // hidden by a later overwrite.  Half the region alternates all-zero
        // and all-one words; the other half uses the normal address-derived
        // BIST pattern. The scratch region is aligned around the address that
        // actually failed, so it need not repeat the entire pre-failure BIST
        // interval at every tap. The selected center is still accepted only
        // after the complete BIST restarts at address zero; a false local pass
        // therefore advances the exhaustive center-out candidate search rather
        // than weakening the final qualification criterion.
        localparam integer BIST_TX_EYE_WORDS = 8192;
        localparam integer BIST_TX_EYE_ADDR_BITS =
            $clog2(BIST_TX_EYE_WORDS);
        localparam integer BIST_TX_EYE_COUNT_BITS =
            $clog2(BIST_TX_EYE_WORDS + 1);
        localparam integer PHYSICAL_DQ_BITS = 8 * BYTE_LANES;
        localparam [3:0] DIAG_REREAD_ORIGINAL = 4'd0,
                         DIAG_REWRITE         = 4'd1,
                         DIAG_REREAD_REWRITE  = 4'd2,
                         DIAG_REREAD_CONTROL  = 4'd3,
                         DIAG_STREAM_REWRITE  = 4'd4,
                         DIAG_REREAD_STREAM   = 4'd5,
                         DIAG_TX_EYE_REQUEST  = 4'd6,
                         DIAG_TX_EYE_WRITE    = 4'd7,
                         DIAG_TX_EYE_READ     = 4'd8,
                         DIAG_TX_EYE_ANALYZE  = 4'd9,
                         DIAG_TX_EYE_FINALIZE = 4'd10,
                         DIAG_TX_EYE_APPLY_GAP = 4'd11,
                         DIAG_TX_EYE_APPLY_REQUEST = 4'd12;
        (* mark_debug = "true" *) reg diag_pending;
        (* mark_debug = "true" *) reg diag_running;
        (* mark_debug = "true" *) reg diag_done;
        (* mark_debug = "true" *) reg [3:0] diag_phase;
        (* mark_debug = "true" *) reg diag_rewrite_accepted;
        (* mark_debug = "true" *) reg [BIST_ADDR_BITS-1:0]
            diag_fail_addr;
        reg [WB_ADDR_BITS-1:0] diag_fail_wb_addr;
        reg [WB_DATA_BITS-1:0] diag_expected_data;
        (* mark_debug = "true" *) reg [WB_DATA_BITS-1:0]
            diag_first_bad_xor;
        // The AXKU3 ILA has finite probe width.  The observed DDR4-1600 fault
        // is within the first two 32-bit UI words, so retain a compact view of
        // that exact first-failure mask alongside the complete CSR/debug copy.
        // This alias is diagnostic-only and has no functional fanout.
        (* mark_debug = "true" *) wire [63:0]
            dbg_diag_first_bad_xor_low64 = diag_first_bad_xor[63:0];
        (* mark_debug = "true" *) reg [WB_DATA_BITS-1:0]
            diag_retry_last_xor;
        (* mark_debug = "true" *) reg [WB_DATA_BITS-1:0]
            diag_retry_xor_or;
        (* mark_debug = "true" *) reg [WB_DATA_BITS-1:0]
            diag_retry_xor_and;
        (* mark_debug = "true" *) reg [5:0] diag_reads_issued;
        (* mark_debug = "true" *) reg [5:0] diag_reads_returned;
        (* mark_debug = "true" *) reg [5:0] diag_match_count;
        (* mark_debug = "true" *) reg [5:0] diag_mismatch_count;
        (* mark_debug = "true" *) reg [5:0] diag_post_reads_issued;
        (* mark_debug = "true" *) reg [5:0] diag_post_reads_returned;
        (* mark_debug = "true" *) reg [5:0] diag_post_match_count;
        (* mark_debug = "true" *) reg [5:0] diag_post_mismatch_count;
        (* mark_debug = "true" *) reg [WB_DATA_BITS-1:0]
            diag_post_last_xor;
        (* mark_debug = "true" *) reg [WB_DATA_BITS-1:0]
            diag_post_xor_or;
        (* mark_debug = "true" *) reg [WB_DATA_BITS-1:0]
            diag_post_xor_and;

        // A rewrite of the failing address exercises both the isolated write
        // and isolated read boundaries, so it cannot by itself distinguish a
        // TX boundary error from an RX boundary error.  Retain one word that
        // the normal streaming BIST verified, then reread that known-good
        // location in isolation before performing the rewrite.  If this
        // control word also fails, the isolated RX path is responsible; if it
        // remains clean while the rewritten failing word is bad, the isolated
        // TX path is responsible.
        reg last_good_valid;
        reg [BIST_ADDR_BITS-1:0] last_good_addr;
        reg [WB_ADDR_BITS-1:0] last_good_wb_addr;
        reg [WB_DATA_BITS-1:0] last_good_expected_data;
        (* mark_debug = "true" *) reg diag_control_valid;
        (* mark_debug = "true" *) reg [BIST_ADDR_BITS-1:0]
            diag_control_addr;
        reg [WB_ADDR_BITS-1:0] diag_control_wb_addr;
        reg [WB_DATA_BITS-1:0] diag_control_expected_data;
        (* mark_debug = "true" *) reg [5:0] diag_control_reads_issued;
        (* mark_debug = "true" *) reg [5:0] diag_control_reads_returned;
        (* mark_debug = "true" *) reg [5:0] diag_control_match_count;
        (* mark_debug = "true" *) reg [5:0] diag_control_mismatch_count;

        // A single isolated rewrite intentionally starts from an idle bus and
        // therefore cannot reproduce a fault that exists only between
        // contiguous native serializer words.  After the isolated rewrite has
        // been classified, issue a 64-word uninterrupted write stream with
        // the failed address in its middle.  Its immediate neighbours use the
        // opposite all-zero/all-one value so every DQ, including the observed
        // lane-0 DQ6, changes at the word boundary.  This diagnostic runs only
        // after a real BIST failure and never changes a passing execution.
        reg [WB_ADDR_BITS-1:0] diag_stream_base_wb_addr;
        reg [6:0] diag_stream_target_index;
        (* mark_debug = "true" *) reg [6:0]
            diag_stream_writes_accepted;
        (* mark_debug = "true" *) reg diag_stream_target_seen;
        (* mark_debug = "true" *) reg [5:0] diag_stream_reads_issued;
        (* mark_debug = "true" *) reg [5:0] diag_stream_reads_returned;
        (* mark_debug = "true" *) reg [5:0] diag_stream_match_count;
        (* mark_debug = "true" *) reg [5:0] diag_stream_mismatch_count;
        (* mark_debug = "true" *) reg [7:0] diag_stream_xor_or;
        (* mark_debug = "true" *) reg [7:0] diag_stream_xor_and;

        // Per-bit TX-eye sweep.  Sixty-five samples cover absolute taps
        // 0,8,...,504,511.  The native PHY performs every intermediate <=8
        // tap VAR_LOAD step and returns the pre-sweep BISC-maintained value so
        // it can be restored after measurement.
        (* mark_debug = "true" *) reg [7:0] diag_tx_eye_bad_data_bit;
        (* mark_debug = "true" *) reg [7:0] diag_tx_eye_dq;
        (* mark_debug = "true" *) reg [6:0] diag_tx_eye_sample;
        (* mark_debug = "true" *) reg [64:0] diag_tx_eye_pass_map;
        (* mark_debug = "true" *) reg [6:0] diag_tx_eye_pass_count;
        (* mark_debug = "true" *) reg [8:0] diag_tx_eye_baseline_tap;
        (* mark_debug = "true" *) reg diag_tx_eye_baseline_valid;
        (* mark_debug = "true" *) reg [8:0] diag_tx_eye_first_pass;
        (* mark_debug = "true" *) reg [8:0] diag_tx_eye_last_pass;
        (* mark_debug = "true" *) reg diag_tx_eye_pass_found;
        (* mark_debug = "true" *) reg diag_tx_eye_bad_seen;
        (* mark_debug = "true" *) reg [BIST_TX_EYE_COUNT_BITS-1:0]
            diag_tx_eye_writes_accepted;
        (* mark_debug = "true" *) reg [BIST_TX_EYE_COUNT_BITS-1:0]
            diag_tx_eye_reads_issued;
        (* mark_debug = "true" *) reg [BIST_TX_EYE_COUNT_BITS-1:0]
            diag_tx_eye_reads_returned;
        reg [WB_ADDR_BITS-1:0] diag_tx_eye_base_wb_addr;
        (* mark_debug = "true" *) reg diag_tx_eye_restore_pending;
        (* mark_debug = "true" *) reg diag_tx_eye_update_error;
        (* mark_debug = "true" *) reg diag_tx_eye_done;
        // Scan the 64 uniformly spaced samples twice to find a passing
        // interval that wraps through tap 511/0.  The resulting correction
        // is retained per physical DQ and validated by restarting the entire
        // BIST, rather than accepting the diagnostic reads as a shortcut.
        (* mark_debug = "true" *) reg [7:0] diag_tx_eye_scan_index;
        (* mark_debug = "true" *) reg [6:0] diag_tx_eye_scan_run;
        (* mark_debug = "true" *) reg [6:0] diag_tx_eye_best_run;
        (* mark_debug = "true" *) reg [6:0] diag_tx_eye_best_end;
        (* mark_debug = "true" *) reg [8:0] diag_tx_eye_center_tap;
        (* mark_debug = "true" *) reg [PHYSICAL_DQ_BITS-1:0]
            diag_tx_eye_tuned_mask;
        reg [8:0] diag_tx_eye_seed_tap [0:PHYSICAL_DQ_BITS-1];
        reg [8:0] diag_tx_eye_original_tap [0:PHYSICAL_DQ_BITS-1];
        // A bounded full-BIST retry rank is retained independently for every
        // physical DQ.  The short eye sweep identifies candidate taps, but
        // only the complete BIST can qualify a candidate for production use.
        // If that authoritative check fails, the next candidate is selected
        // center-out from the fixed seed measured once for that DQ. Keeping
        // only the rank (rather than a board-specific tap) lets retries
        // exhaust the circular delay range portably across UltraScale/
        // UltraScale+ devices and PCB layouts.
        reg [9:0] diag_tx_eye_retry_rank [0:PHYSICAL_DQ_BITS-1];
        (* mark_debug = "true" *) reg [9:0]
            diag_tx_eye_retry_rank_current;
        integer diag_tx_eye_reset_dq;
        // Locate the first failing serialized data bit after traffic has
        // stopped.  A one-bit-per-controller-cycle scan is deliberately
        // used here: a combinational priority encoder across the complete
        // Wishbone word would otherwise sit directly on the normal read ACK
        // path and needlessly constrain DDR4-1600 timing.  Diagnostic entry
        // is already waiting for all outstanding requests to drain, so this
        // bounded post-failure latency has no functional cost.
        localparam integer DIAG_BAD_INDEX_BITS =
            (WB_DATA_BITS <= 2) ? 1 : $clog2(WB_DATA_BITS);
        reg [DIAG_BAD_INDEX_BITS-1:0] diag_bad_bit_scan_index;
        reg [WB_DATA_BITS-1:0] diag_bad_bit_scan_mask;
        reg diag_bad_bit_scan_loaded;
        reg diag_bad_bit_found;

        // Outstanding request tracking for WB pipelining.
        // Circular buffer FIFO tracks W/R type per outstanding request.
        // ack_rd_ptr points to the oldest entry (next to be ACK'd).
        // ack_wr_ptr points to the next free slot for new requests.
        // 1=write, 0=read. Since WB B4 ACKs are in-order, the entry at
        // ack_rd_ptr always identifies whether the next ACK is a write or read.
        reg [4:0] outstanding;
        reg [15:0] ack_type_q;
        reg [3:0] ack_wr_ptr;
        reg [3:0] ack_rd_ptr;

        // Pattern generation -- deterministic from address.
        // XOR-folds the address with a swapped copy and a constant to
        // produce a 32-bit seed, then replicates it across the full
        // data width.  The constant 0xA55A3CC3 ensures addr=0 still
        // produces a non-trivial pattern (no stuck-at-0 masking).
        function [WB_DATA_BITS-1:0] gen_pattern;
            input [BIST_ADDR_BITS-1:0] addr;
            reg [31:0] seed;
            reg [31:0] a;
            begin
                a = {{(32-BIST_ADDR_BITS){1'b0}}, addr};
                seed = a ^ {a[15:0], a[31:16]} ^ 32'hA55A3CC3;
                gen_pattern = {(WB_DATA_BITS/32){seed}};
            end
        endfunction

        function [WB_DATA_BITS-1:0] tx_eye_pattern;
            input [BIST_ADDR_BITS-1:0] addr;
            input [BIST_TX_EYE_COUNT_BITS-1:0] index;
            begin
                if (!index[BIST_TX_EYE_ADDR_BITS-1])
                    tx_eye_pattern = index[0] ?
                        {WB_DATA_BITS{1'b1}} :
                        {WB_DATA_BITS{1'b0}};
                else
                    tx_eye_pattern = gen_pattern(addr);
            end
        endfunction

        function [8:0] tx_eye_tap_for_sample;
            input [6:0] sample;
            begin
                tx_eye_tap_for_sample = (sample >= 7'd64) ?
                    9'd511 : {sample[5:0], 3'b000};
            end
        endfunction

        function [7:0] physical_dq_for_data_bit;
            input [7:0] data_bit;
            begin
                // One DFI UI contains eight physical DQ bits per byte lane.
                // Modulo maps any of the eight serialized UIs back to its pad.
                physical_dq_for_data_bit = data_bit % (8 * BYTE_LANES);
            end
        endfunction

        // One physical DQ is serialized across every DFI UI in the Wishbone
        // word.  A single ODELAY setting must therefore work for all of those
        // UI positions, not merely for the position that exposed the first
        // BIST error.  Reducing the fixed-stride positions here makes the TX
        // eye the intersection of all serialized-bit windows and prevents a
        // later UI on the same pad from failing after apparent centering.
        function physical_dq_read_mismatch;
            input [WB_DATA_BITS-1:0] actual_data;
            input [WB_DATA_BITS-1:0] expected_word;
            input [7:0] physical_dq;
            integer ui_index;
            begin
                physical_dq_read_mismatch = 1'b0;
                for (ui_index = 0;
                     ui_index < (WB_DATA_BITS / PHYSICAL_DQ_BITS);
                     ui_index = ui_index + 1) begin
                    if (actual_data[ui_index*PHYSICAL_DQ_BITS +
                                    physical_dq] !=
                        expected_word[ui_index*PHYSICAL_DQ_BITS +
                                      physical_dq])
                        physical_dq_read_mismatch = 1'b1;
                end
            end
        endfunction

        function [8:0] tx_eye_seed_from_run;
            input [6:0] best_end;
            input [6:0] best_run;
            input [8:0] fallback_tap;
            reg [6:0] start_sample;
            reg [6:0] run_minus_one;
            reg [8:0] start_tap;
            reg [8:0] span_taps;
            begin
                // If the eight-tap grid sees a passing interval, seed at its
                // circular midpoint. A narrow eye can fall entirely between
                // grid samples; in that case the original BISC-maintained tap
                // is a deterministic, device-independent fallback seed.
                if (best_run == 0) begin
                    tx_eye_seed_from_run = fallback_tap;
                end else begin
                    start_sample = best_end - best_run + 1'b1;
                    run_minus_one = best_run - 1'b1;
                    start_tap = {start_sample[5:0], 3'b000};
                    span_taps = {run_minus_one[5:0], 3'b000};
                    tx_eye_seed_from_run = start_tap + (span_taps >> 1);
                end
            end
        endfunction

        function [8:0] tx_eye_candidate_from_seed;
            input [8:0] seed_tap;
            input [9:0] retry_rank;
            begin
                // Enumerate all 512 delay taps exactly once in circular
                // center-out order around a seed that remains fixed for this
                // physical DQ across every full-BIST retry:
                // center, +1, -1, +2, -2, ... , +256.  This is exhaustive,
                // bounded, and independent of device/board tap calibration.
                if (retry_rank == 0)
                    tx_eye_candidate_from_seed = seed_tap;
                else if (retry_rank[0])
                    tx_eye_candidate_from_seed = seed_tap +
                        ((retry_rank + 1'b1) >> 1);
                else
                    tx_eye_candidate_from_seed = seed_tap -
                        (retry_rank >> 1);
            end
        endfunction

        // Data-mask test pattern: returns full data word with only the byte
        // at position byte_idx carrying the real pattern; all other bytes
        // are filled with 0xAA (a canary value that will be caught during
        // read-back if the data mask fails to suppress the write).
        function [WB_DATA_BITS-1:0] dm_pattern;
            input [BIST_ADDR_BITS-1:0] addr;
            input [$clog2(WB_SEL_BITS)-1:0] byte_idx;
            reg [WB_DATA_BITS-1:0] pat;
            integer i;
            begin
                pat = gen_pattern(addr);
                for (i = 0; i < WB_SEL_BITS; i = i + 1) begin
                    if (i[$clog2(WB_SEL_BITS)-1:0] == byte_idx)
                        dm_pattern[8*i +: 8] = pat[8*i +: 8];
                    else
                        dm_pattern[8*i +: 8] = 8'hAA;
                end
            end
        endfunction

        // Bit-reversal scramble for data pattern uniqueness.
        function [BIST_ADDR_BITS-1:0] scramble_addr;
            input [BIST_ADDR_BITS-1:0] addr;
            integer i;
            begin
                for (i = 0; i < BIST_ADDR_BITS; i = i + 1) begin
                    scramble_addr[i] = addr[BIST_ADDR_BITS-1-i];
                end
            end
        endfunction

        // Stress address mapping: a one-to-one permutation of the BIST
        // counter for the controller's ADDR_MAPPING=1 layout:
        //   WB[0]     (BG)       <- counter[4]    (changes every 16)
        //   WB[8:9]   (BA)       <- counter[3:2]  (changes every 4)
        //   WB[10:11] (row LSbs) <- counter[1:0]  (changes every access)
        //   WB[1:7]   (column)   <- counter[11:5]
        //   remaining row bits retain their corresponding counter bits.
        //
        // Every counter bit occurs exactly once in the output. A stress
        // transform may rearrange addresses, but must never discard high
        // bits and alias many BIST patterns onto one DRAM location. This is
        // also safe for MICRON_SIM's smaller BIST address range: every
        // available counter bit still maps to one unique WB bit.
        /* verilator lint_off UNUSEDSIGNAL */
        function [WB_ADDR_BITS-1:0] stress_addr;
            input [BIST_ADDR_BITS-1:0] addr;
            integer i;
            begin
                stress_addr = {WB_ADDR_BITS{1'b0}};
                for (i = 0; i < BIST_ADDR_BITS; i = i + 1) begin
                    case (i)
                        0:       stress_addr[10] = addr[i];
                        1:       stress_addr[11] = addr[i];
                        2:       stress_addr[8]  = addr[i];
                        3:       stress_addr[9]  = addr[i];
                        4:       stress_addr[0]  = addr[i];
                        5:       stress_addr[1]  = addr[i];
                        6:       stress_addr[2]  = addr[i];
                        7:       stress_addr[3]  = addr[i];
                        8:       stress_addr[4]  = addr[i];
                        9:       stress_addr[5]  = addr[i];
                        10:      stress_addr[6]  = addr[i];
                        11:      stress_addr[7]  = addr[i];
                        default: stress_addr[i]   = addr[i];
                    endcase
                end
            end
        endfunction
        /* verilator lint_on UNUSEDSIGNAL */

        // Expected data for read verification (covers trailing ACKs across phases)
        wire uses_scramble = (bist_state == BIST_RANDOM_READ) ||
                             ((bist_state == BIST_RANDOM_WRITE ||
                               bist_state == BIST_ALT_WRITE_READ ||
                               bist_state == BIST_FINISH) && last_read_scrambled);
        wire [WB_DATA_BITS-1:0] expected_data;
        assign expected_data = uses_scramble ? gen_pattern(scramble_addr(check_addr))
                                             : gen_pattern(check_addr);

        always @(posedge i_clk) begin
            if (prober_internal_reset) begin
                bist_state      <= BIST_IDLE;
                write_addr      <= {BIST_ADDR_BITS{1'b0}};
                read_addr       <= {BIST_ADDR_BITS{1'b0}};
                check_addr      <= {BIST_ADDR_BITS{1'b0}};
                correct_count   <= 32'd0;
                error_count     <= 32'd0;
                bist_fail_sticky <= 1'b0;
                o_bist_failed_reset_req <= 1'b0;
                alt_phase       <= 1'b0;
                last_read_scrambled <= 1'b0;
                write_byte_counter <= {$clog2(WB_SEL_BITS){1'b0}};
                diag_pending       <= 1'b0;
                diag_running       <= 1'b0;
                diag_done          <= 1'b0;
                diag_phase         <= DIAG_REREAD_ORIGINAL;
                diag_rewrite_accepted <= 1'b0;
                diag_fail_addr     <= {BIST_ADDR_BITS{1'b0}};
                diag_fail_wb_addr  <= {WB_ADDR_BITS{1'b0}};
                diag_expected_data <= {WB_DATA_BITS{1'b0}};
                diag_first_bad_xor <= {WB_DATA_BITS{1'b0}};
                diag_retry_last_xor <= {WB_DATA_BITS{1'b0}};
                diag_retry_xor_or  <= {WB_DATA_BITS{1'b0}};
                diag_retry_xor_and <= {WB_DATA_BITS{1'b1}};
                diag_reads_issued  <= 6'd0;
                diag_reads_returned <= 6'd0;
                diag_match_count   <= 6'd0;
                diag_mismatch_count <= 6'd0;
                diag_post_reads_issued <= 6'd0;
                diag_post_reads_returned <= 6'd0;
                diag_post_match_count <= 6'd0;
                diag_post_mismatch_count <= 6'd0;
                diag_post_last_xor <= {WB_DATA_BITS{1'b0}};
                diag_post_xor_or <= {WB_DATA_BITS{1'b0}};
                diag_post_xor_and <= {WB_DATA_BITS{1'b1}};
                last_good_valid <= 1'b0;
                last_good_addr <= {BIST_ADDR_BITS{1'b0}};
                last_good_wb_addr <= {WB_ADDR_BITS{1'b0}};
                last_good_expected_data <= {WB_DATA_BITS{1'b0}};
                diag_control_valid <= 1'b0;
                diag_control_addr <= {BIST_ADDR_BITS{1'b0}};
                diag_control_wb_addr <= {WB_ADDR_BITS{1'b0}};
                diag_control_expected_data <= {WB_DATA_BITS{1'b0}};
                diag_control_reads_issued <= 6'd0;
                diag_control_reads_returned <= 6'd0;
                diag_control_match_count <= 6'd0;
                diag_control_mismatch_count <= 6'd0;
                diag_stream_base_wb_addr <= {WB_ADDR_BITS{1'b0}};
                diag_stream_target_index <= 7'd0;
                diag_stream_writes_accepted <= 7'd0;
                diag_stream_target_seen <= 1'b0;
                diag_stream_reads_issued <= 6'd0;
                diag_stream_reads_returned <= 6'd0;
                diag_stream_match_count <= 6'd0;
                diag_stream_mismatch_count <= 6'd0;
                diag_stream_xor_or <= 8'd0;
                diag_stream_xor_and <= 8'hff;
                diag_tx_eye_bad_data_bit <= 8'd0;
                diag_tx_eye_dq <= 8'd0;
                diag_tx_eye_sample <= 7'd0;
                diag_tx_eye_pass_map <= 65'd0;
                diag_tx_eye_pass_count <= 7'd0;
                diag_tx_eye_baseline_tap <= 9'd0;
                diag_tx_eye_baseline_valid <= 1'b0;
                diag_tx_eye_first_pass <= 9'd0;
                diag_tx_eye_last_pass <= 9'd0;
                diag_tx_eye_pass_found <= 1'b0;
                diag_tx_eye_bad_seen <= 1'b0;
                diag_tx_eye_writes_accepted <=
                    {BIST_TX_EYE_COUNT_BITS{1'b0}};
                diag_tx_eye_reads_issued <=
                    {BIST_TX_EYE_COUNT_BITS{1'b0}};
                diag_tx_eye_reads_returned <=
                    {BIST_TX_EYE_COUNT_BITS{1'b0}};
                diag_tx_eye_base_wb_addr <= {WB_ADDR_BITS{1'b0}};
                diag_tx_eye_restore_pending <= 1'b0;
                diag_tx_eye_update_error <= 1'b0;
                diag_tx_eye_done <= 1'b0;
                diag_tx_eye_scan_index <= 8'd0;
                diag_tx_eye_scan_run <= 7'd0;
                diag_tx_eye_best_run <= 7'd0;
                diag_tx_eye_best_end <= 7'd0;
                diag_tx_eye_center_tap <= 9'd0;
                diag_tx_eye_tuned_mask <= {PHYSICAL_DQ_BITS{1'b0}};
                diag_tx_eye_retry_rank_current <= 10'd0;
                for (diag_tx_eye_reset_dq = 0;
                     diag_tx_eye_reset_dq < PHYSICAL_DQ_BITS;
                     diag_tx_eye_reset_dq = diag_tx_eye_reset_dq + 1) begin
                    diag_tx_eye_retry_rank[diag_tx_eye_reset_dq] <= 10'd0;
                    diag_tx_eye_seed_tap[diag_tx_eye_reset_dq] <= 9'd0;
                    diag_tx_eye_original_tap[diag_tx_eye_reset_dq] <= 9'd0;
                end
                diag_bad_bit_scan_index <= {DIAG_BAD_INDEX_BITS{1'b0}};
                diag_bad_bit_scan_mask <= {WB_DATA_BITS{1'b0}};
                diag_bad_bit_scan_loaded <= 1'b0;
                diag_bad_bit_found <= 1'b0;
                o_phy_tx_diag_req <= 1'b0;
                o_phy_tx_diag_dq <= 8'd0;
                o_phy_tx_diag_tap <= 9'd0;
                o_wb_cyc        <= 1'b0;
                o_wb_stb        <= 1'b0;
                o_wb_we         <= 1'b0;
                o_wb_addr       <= {WB_ADDR_BITS{1'b0}};
                o_wb_data       <= {WB_DATA_BITS{1'b0}};
                o_wb_sel        <= {WB_SEL_BITS{1'b1}};
                outstanding     <= 5'd0;
                ack_type_q      <= 16'd0;
                ack_wr_ptr      <= 4'd0;
                ack_rd_ptr      <= 4'd0;
            end else begin

                // Track outstanding requests + type FIFO (circular buffer)
                if (o_wb_stb && !i_wb_stall) begin // new request accepted this cycle so push to FIFO 
                    ack_type_q[ack_wr_ptr] <= o_wb_we;
                    ack_wr_ptr <= ack_wr_ptr + 1'b1;
                    outstanding <= outstanding + 1'b1;
                end
                if (i_wb_ack) begin // an ACK arrived this cycle so pop from FIFO
                    ack_rd_ptr <= ack_rd_ptr + 1'b1;
                    outstanding <= outstanding - 1'b1;
                end
                if (o_wb_stb && !i_wb_stall && i_wb_ack) begin // new request accepted and ACK arrived this cycle so push and pop FIFO so outstanding stays the same
                    outstanding <= outstanding;
                end

                // Data check on read ACK (head entry is read type)
                if (i_wb_ack && !ack_type_q[ack_rd_ptr] && // ACK is a read
                    bist_state != BIST_IDLE &&
                    bist_state != BIST_DONE &&
                    bist_state != BIST_BURST_WRITE) begin
                    `ifndef YOSYS
                        if (check_addr < 20) begin
                            $display("[%0t] BIST CHK: addr=%0d exp=%0h got=%0h state=%0d", $realtime, check_addr, expected_data, i_wb_data, bist_state);
                        end
                    `endif
                    if (BIST_REREAD_DIAG && diag_running &&
                        (diag_phase == DIAG_REREAD_ORIGINAL)) begin
                        // Every retry targets diag_fail_addr. Preserve the
                        // first sample, final sample, and the OR of all error
                        // masks so one ILA capture shows whether the same
                        // stored bit is repeatably wrong or the RX result
                        // varies from read to read.
                        diag_reads_returned <= diag_reads_returned + 1'b1;
                        diag_retry_last_xor <= i_wb_data ^
                                               diag_expected_data;
                        diag_retry_xor_or <= diag_retry_xor_or |
                                             (i_wb_data ^ diag_expected_data);
                        diag_retry_xor_and <= diag_retry_xor_and &
                                              (i_wb_data ^ diag_expected_data);
                        if (i_wb_data == diag_expected_data) begin
                            diag_match_count <= diag_match_count + 1'b1;
                            correct_count <= correct_count + 1'b1;
                        end else begin
                            diag_mismatch_count <=
                                diag_mismatch_count + 1'b1;
                            error_count <= error_count + 1'b1;
                        end
                    end else if (BIST_REREAD_DIAG && diag_running &&
                                 (diag_phase == DIAG_REREAD_CONTROL)) begin
                        diag_control_reads_returned <=
                            diag_control_reads_returned + 1'b1;
                        if (i_wb_data == diag_control_expected_data) begin
                            diag_control_match_count <=
                                diag_control_match_count + 1'b1;
                            correct_count <= correct_count + 1'b1;
                        end else begin
                            diag_control_mismatch_count <=
                                diag_control_mismatch_count + 1'b1;
                            error_count <= error_count + 1'b1;
                        end
                    end else if (BIST_REREAD_DIAG && diag_running &&
                                 (diag_phase == DIAG_REREAD_REWRITE)) begin
                        // After both read-only classifications, rewrite the
                        // expected word once and repeat the failing-address
                        // reads.  Interpret this result together with the
                        // known-good isolated control above: the pair
                        // distinguishes isolated TX and RX boundary failures
                        // without changing any delay or training setting.
                        diag_post_reads_returned <=
                            diag_post_reads_returned + 1'b1;
                        diag_post_last_xor <= i_wb_data ^
                                              diag_expected_data;
                        diag_post_xor_or <= diag_post_xor_or |
                                            (i_wb_data ^ diag_expected_data);
                        diag_post_xor_and <= diag_post_xor_and &
                                             (i_wb_data ^ diag_expected_data);
                        if (i_wb_data == diag_expected_data) begin
                            diag_post_match_count <=
                                diag_post_match_count + 1'b1;
                            correct_count <= correct_count + 1'b1;
                        end else begin
                            diag_post_mismatch_count <=
                                diag_post_mismatch_count + 1'b1;
                            error_count <= error_count + 1'b1;
                        end
                    end else if (BIST_REREAD_DIAG && diag_running &&
                                 (diag_phase == DIAG_REREAD_STREAM)) begin
                        diag_stream_reads_returned <=
                            diag_stream_reads_returned + 1'b1;
                        diag_stream_xor_or <= diag_stream_xor_or |
                            (i_wb_data[42:35] ^ diag_expected_data[42:35]);
                        diag_stream_xor_and <= diag_stream_xor_and &
                            (i_wb_data[42:35] ^ diag_expected_data[42:35]);
                        if (i_wb_data == diag_expected_data) begin
                            diag_stream_match_count <=
                                diag_stream_match_count + 1'b1;
                            correct_count <= correct_count + 1'b1;
                        end else begin
                            diag_stream_mismatch_count <=
                                diag_stream_mismatch_count + 1'b1;
                            error_count <= error_count + 1'b1;
                        end
                    end else if (BIST_REREAD_DIAG && diag_running &&
                                 (diag_phase == DIAG_TX_EYE_READ)) begin
                        // The sweep classifies only the physical DQ implicated
                        // by the first failure.  Other bits are deliberately
                        // ignored here so a separate marginal pad cannot hide
                        // this bit's complete pass window.
                        diag_tx_eye_reads_returned <=
                            diag_tx_eye_reads_returned + 1'b1;
                        if (physical_dq_read_mismatch(
                                i_wb_data,
                                tx_eye_pattern(
                                    diag_tx_eye_base_wb_addr[
                                        BIST_ADDR_BITS-1:0] +
                                    diag_tx_eye_reads_returned,
                                    diag_tx_eye_reads_returned),
                                diag_tx_eye_dq))
                            diag_tx_eye_bad_seen <= 1'b1;
                    end else begin
                        if (i_wb_data == expected_data) begin // read data matches expected pattern
                            correct_count <= correct_count + 1'b1;
                            // Preserve the most recently verified normal-BIST
                            // word.  While diag_pending drains already-issued
                            // requests this naturally selects a known-good
                            // neighbor after the first failure at address 0.
                            last_good_valid <= 1'b1;
                            last_good_addr <= check_addr;
                            last_good_wb_addr <= uses_scramble ?
                                stress_addr(check_addr) :
                                {{(WB_ADDR_BITS-BIST_ADDR_BITS){1'b0}},
                                 check_addr};
                            last_good_expected_data <= expected_data;
                        end else begin // read data mismatch so increment error count and latch fail sticky
                            error_count <= error_count + 1'b1;
                            bist_fail_sticky <= 1'b1;
                            if (BIST_REREAD_DIAG && !diag_pending &&
                                !diag_running && !diag_done) begin
                                diag_pending <= 1'b1;
                                diag_fail_addr <= check_addr;
                                diag_fail_wb_addr <= uses_scramble ?
                                    stress_addr(check_addr) :
                                    {{(WB_ADDR_BITS-BIST_ADDR_BITS){1'b0}},
                                     check_addr};
                                diag_expected_data <= expected_data;
                                diag_first_bad_xor <= i_wb_data ^ expected_data;
                                diag_tx_eye_bad_data_bit <= 8'd0;
                                diag_tx_eye_dq <= 8'd0;
                                diag_bad_bit_scan_index <=
                                    {DIAG_BAD_INDEX_BITS{1'b0}};
                                diag_bad_bit_scan_mask <=
                                    {WB_DATA_BITS{1'b0}};
                                diag_bad_bit_scan_loaded <= 1'b0;
                                diag_bad_bit_found <= 1'b0;
                            end
                            `ifndef YOSYS
                                $display("[%0t] BIST FAIL: addr=%0h expected=%0h got=%0h", $realtime, check_addr, expected_data, i_wb_data);
                            `endif
                        end
                        check_addr <= check_addr + 1'b1;
                    end
                end

                if (BIST_REREAD_DIAG && diag_pending) begin
                    // Stop issuing new traffic, then wait until every request
                    // older than the failing response has retired. Starting
                    // the retry stream earlier would associate its ACKs with
                    // addresses still present in the B4 request FIFO.
                    o_wb_stb <= 1'b0;
                    if (!diag_bad_bit_scan_loaded) begin
                        // Load from the registered failure mask, never from
                        // the live ACK bus.  Thereafter a shift register
                        // exposes one bit per cycle without a 256:1 mux.
                        diag_bad_bit_scan_mask <= diag_first_bad_xor;
                        diag_bad_bit_scan_loaded <= 1'b1;
                    end else if (!diag_bad_bit_found) begin
                        if (diag_bad_bit_scan_mask[0]) begin
                            diag_tx_eye_bad_data_bit <=
                                {{(8-DIAG_BAD_INDEX_BITS){1'b0}},
                                 diag_bad_bit_scan_index};
                            diag_tx_eye_dq <= physical_dq_for_data_bit(
                                {{(8-DIAG_BAD_INDEX_BITS){1'b0}},
                                 diag_bad_bit_scan_index});
                            diag_bad_bit_found <= 1'b1;
                        end else begin
                            diag_bad_bit_scan_mask <=
                                diag_bad_bit_scan_mask >> 1;
                            diag_bad_bit_scan_index <=
                                diag_bad_bit_scan_index + 1'b1;
                        end
                    end else if (outstanding == 0) begin
                        diag_pending <= 1'b0;
                        // Make the already-trained decision only after the
                        // request pipe is idle.  Keeping this indexed mask
                        // lookup out of the 256-bit mismatch encoder is
                        // essential to normal BIST timing at DDR4-1600.
                        // A failure after an earlier correction rejects that
                        // candidate but does not prove the DQ untrainable.
                        // The per-DQ seed was fixed by its first measured eye,
                        // so rescanning the same 65 coarse points here adds no
                        // information and makes a bounded fine search take
                        // minutes per candidate. Advance directly to the next
                        // center-out tap; it still has to survive the complete
                        // BIST before init_done is allowed. An as-yet untuned
                        // DQ follows the full classification/measurement path.
                        if (diag_tx_eye_tuned_mask[diag_tx_eye_dq]) begin
                            diag_running <= 1'b1;
                            diag_tx_eye_done <= 1'b0;
                            diag_tx_eye_update_error <= 1'b0;
                            diag_tx_eye_restore_pending <= 1'b0;
                            o_phy_tx_diag_dq <= diag_tx_eye_dq;
                            if (diag_tx_eye_retry_rank[diag_tx_eye_dq] <
                                10'd511) begin
                                diag_tx_eye_retry_rank[diag_tx_eye_dq] <=
                                    diag_tx_eye_retry_rank[diag_tx_eye_dq] +
                                    1'b1;
                                diag_tx_eye_retry_rank_current <=
                                    diag_tx_eye_retry_rank[
                                        diag_tx_eye_dq] + 1'b1;
                                diag_tx_eye_center_tap <=
                                    tx_eye_candidate_from_seed(
                                        diag_tx_eye_seed_tap[
                                            diag_tx_eye_dq],
                                        diag_tx_eye_retry_rank[
                                            diag_tx_eye_dq] + 1'b1);
                                o_phy_tx_diag_tap <=
                                    tx_eye_candidate_from_seed(
                                        diag_tx_eye_seed_tap[
                                            diag_tx_eye_dq],
                                        diag_tx_eye_retry_rank[
                                            diag_tx_eye_dq] + 1'b1);
                                o_phy_tx_diag_req <= 1'b1;
                                diag_phase <= DIAG_TX_EYE_APPLY_REQUEST;
                            end else begin
                                // Rank 511 was the final unique tap. Restore
                                // the BISC-maintained value before reporting a
                                // genuinely exhaustive failure.
                                diag_tx_eye_retry_rank_current <= 10'd512;
                                diag_tx_eye_restore_pending <= 1'b1;
                                o_phy_tx_diag_tap <=
                                    diag_tx_eye_original_tap[
                                        diag_tx_eye_dq];
                                o_phy_tx_diag_req <= 1'b1;
                                diag_phase <= DIAG_TX_EYE_REQUEST;
                            end
                        end else begin
                            diag_tx_eye_retry_rank_current <=
                                diag_tx_eye_retry_rank[diag_tx_eye_dq];
                            diag_running <= 1'b1;
                            diag_phase <= DIAG_REREAD_ORIGINAL;
                            diag_rewrite_accepted <= 1'b0;
                            diag_reads_issued <= 6'd0;
                            diag_reads_returned <= 6'd0;
                            diag_match_count <= 6'd0;
                            diag_mismatch_count <= 6'd0;
                            diag_retry_last_xor <= {WB_DATA_BITS{1'b0}};
                            diag_retry_xor_or <= {WB_DATA_BITS{1'b0}};
                            diag_retry_xor_and <= {WB_DATA_BITS{1'b1}};
                            diag_post_reads_issued <= 6'd0;
                            diag_post_reads_returned <= 6'd0;
                            diag_post_match_count <= 6'd0;
                            diag_post_mismatch_count <= 6'd0;
                            diag_post_last_xor <= {WB_DATA_BITS{1'b0}};
                            diag_post_xor_or <= {WB_DATA_BITS{1'b0}};
                            diag_post_xor_and <= {WB_DATA_BITS{1'b1}};
                            diag_control_valid <= last_good_valid;
                            diag_control_addr <= last_good_addr;
                            diag_control_wb_addr <= last_good_wb_addr;
                            diag_control_expected_data <=
                                last_good_expected_data;
                            diag_control_reads_issued <= 6'd0;
                            diag_control_reads_returned <= 6'd0;
                            diag_control_match_count <= 6'd0;
                            diag_control_mismatch_count <= 6'd0;
                            diag_stream_base_wb_addr <=
                                (diag_fail_wb_addr >= 32) ?
                                (diag_fail_wb_addr - 32) :
                                {WB_ADDR_BITS{1'b0}};
                            diag_stream_target_index <=
                                (diag_fail_wb_addr >= 32) ?
                                7'd32 : diag_fail_wb_addr[6:0];
                            // Keep the TX-eye scratch stream inside one
                            // naturally aligned stress region. The complete
                            // region remains in range at either memory end.
                            diag_tx_eye_base_wb_addr <=
                                {diag_fail_wb_addr[
                                     WB_ADDR_BITS-1:BIST_TX_EYE_ADDR_BITS],
                                 {BIST_TX_EYE_ADDR_BITS{1'b0}}};
                            diag_stream_writes_accepted <= 7'd0;
                            diag_stream_target_seen <= 1'b0;
                            diag_stream_reads_issued <= 6'd0;
                            diag_stream_reads_returned <= 6'd0;
                            diag_stream_match_count <= 6'd0;
                            diag_stream_mismatch_count <= 6'd0;
                            diag_stream_xor_or <= 8'd0;
                            diag_stream_xor_and <= 8'hff;
                            o_wb_stb <= 1'b1;
                            o_wb_we <= 1'b0;
                            o_wb_addr <= diag_fail_wb_addr;
                            check_addr <= diag_fail_addr;
                        end
                    end
                end else if (BIST_REREAD_DIAG && diag_running) begin
                    case (diag_phase)
                        DIAG_REREAD_ORIGINAL: begin
                            o_wb_we <= 1'b0;
                            o_wb_addr <= diag_fail_wb_addr;
                            if (o_wb_stb && !i_wb_stall) begin
                                diag_reads_issued <=
                                    diag_reads_issued + 1'b1;
                                if (diag_reads_issued == BIST_DIAG_READS-1)
                                    o_wb_stb <= 1'b0;
                            end
                            if ((diag_reads_issued == BIST_DIAG_READS) &&
                                (diag_reads_returned == BIST_DIAG_READS) &&
                                (outstanding == 0)) begin
                                o_wb_stb <= 1'b1;
                                if (diag_control_valid) begin
                                    diag_phase <= DIAG_REREAD_CONTROL;
                                    o_wb_we <= 1'b0;
                                    o_wb_addr <= diag_control_wb_addr;
                                    check_addr <= diag_control_addr;
                                end else begin
                                    diag_phase <= DIAG_REWRITE;
                                    diag_rewrite_accepted <= 1'b0;
                                    o_wb_we <= 1'b1;
                                    o_wb_addr <= diag_fail_wb_addr;
                                    o_wb_data <= diag_expected_data;
                                    o_wb_sel <= {WB_SEL_BITS{1'b1}};
                                end
                            end
                        end

                        DIAG_REREAD_CONTROL: begin
                            o_wb_we <= 1'b0;
                            o_wb_addr <= diag_control_wb_addr;
                            if (o_wb_stb && !i_wb_stall) begin
                                diag_control_reads_issued <=
                                    diag_control_reads_issued + 1'b1;
                                if (diag_control_reads_issued ==
                                    BIST_DIAG_READS-1)
                                    o_wb_stb <= 1'b0;
                            end
                            if ((diag_control_reads_issued ==
                                 BIST_DIAG_READS) &&
                                (diag_control_reads_returned ==
                                 BIST_DIAG_READS) &&
                                (outstanding == 0)) begin
                                diag_phase <= DIAG_REWRITE;
                                diag_rewrite_accepted <= 1'b0;
                                o_wb_stb <= 1'b1;
                                o_wb_we <= 1'b1;
                                o_wb_addr <= diag_fail_wb_addr;
                                o_wb_data <= diag_expected_data;
                                o_wb_sel <= {WB_SEL_BITS{1'b1}};
                            end
                        end

                        DIAG_REWRITE: begin
                            o_wb_we <= 1'b1;
                            o_wb_addr <= diag_fail_wb_addr;
                            o_wb_data <= diag_expected_data;
                            o_wb_sel <= {WB_SEL_BITS{1'b1}};
                            if (o_wb_stb && !i_wb_stall) begin
                                diag_rewrite_accepted <= 1'b1;
                                o_wb_stb <= 1'b0;
                            end
                            if (diag_rewrite_accepted &&
                                (outstanding == 0)) begin
                                diag_phase <= DIAG_REREAD_REWRITE;
                                diag_post_reads_issued <= 6'd0;
                                diag_post_reads_returned <= 6'd0;
                                diag_post_match_count <= 6'd0;
                                diag_post_mismatch_count <= 6'd0;
                                diag_post_last_xor <=
                                    {WB_DATA_BITS{1'b0}};
                                diag_post_xor_or <=
                                    {WB_DATA_BITS{1'b0}};
                                diag_post_xor_and <=
                                    {WB_DATA_BITS{1'b1}};
                                o_wb_stb <= 1'b1;
                                o_wb_we <= 1'b0;
                                check_addr <= diag_fail_addr;
                            end
                        end

                        DIAG_REREAD_REWRITE: begin
                            o_wb_we <= 1'b0;
                            o_wb_addr <= diag_fail_wb_addr;
                            if (o_wb_stb && !i_wb_stall) begin
                                diag_post_reads_issued <=
                                    diag_post_reads_issued + 1'b1;
                                if (diag_post_reads_issued ==
                                    BIST_DIAG_READS-1)
                                    o_wb_stb <= 1'b0;
                            end
                            if ((diag_post_reads_issued == BIST_DIAG_READS) &&
                                (diag_post_reads_returned == BIST_DIAG_READS) &&
                                (outstanding == 0)) begin
                                // Keep ownership and follow the proven
                                // isolated transaction with a continuous write
                                // stream.  Starting with STB Low gives the
                                // stream state one clean setup cycle for its
                                // base address and stress word.
                                diag_phase <= DIAG_STREAM_REWRITE;
                                o_wb_stb <= 1'b0;
                                o_wb_we <= 1'b1;
                            end
                        end

                        DIAG_STREAM_REWRITE: begin
                            o_wb_we <= 1'b1;
                            o_wb_sel <= {WB_SEL_BITS{1'b1}};

                            if (!o_wb_stb &&
                                (diag_stream_writes_accepted == 0) &&
                                (outstanding == 0)) begin
                                o_wb_stb <= 1'b1;
                                o_wb_addr <= diag_stream_base_wb_addr;
                                o_wb_data <=
                                    (diag_stream_target_index == 0) ?
                                    diag_expected_data :
                                    {WB_DATA_BITS{1'b1}};
                            end else if (o_wb_stb && !i_wb_stall) begin
                                diag_stream_writes_accepted <=
                                    diag_stream_writes_accepted + 1'b1;
                                if (diag_stream_writes_accepted ==
                                    diag_stream_target_index)
                                    diag_stream_target_seen <= 1'b1;

                                if (diag_stream_writes_accepted ==
                                    BIST_DIAG_STREAM_WRITES-1) begin
                                    o_wb_stb <= 1'b0;
                                end else begin
                                    o_wb_addr <= diag_stream_base_wb_addr +
                                        diag_stream_writes_accepted + 1'b1;
                                    if ((diag_stream_writes_accepted + 1'b1) ==
                                        diag_stream_target_index)
                                        o_wb_data <= diag_expected_data;
                                    else if ((diag_stream_writes_accepted +
                                              1'b1) & 1'b1)
                                        o_wb_data <= {WB_DATA_BITS{1'b0}};
                                    else
                                        o_wb_data <= {WB_DATA_BITS{1'b1}};
                                end
                            end

                            if ((diag_stream_writes_accepted ==
                                 BIST_DIAG_STREAM_WRITES) &&
                                (outstanding == 0) && !o_wb_stb) begin
                                diag_phase <= DIAG_REREAD_STREAM;
                                diag_stream_reads_issued <= 6'd0;
                                diag_stream_reads_returned <= 6'd0;
                                diag_stream_match_count <= 6'd0;
                                diag_stream_mismatch_count <= 6'd0;
                                diag_stream_xor_or <= 8'd0;
                                diag_stream_xor_and <= 8'hff;
                                o_wb_stb <= 1'b1;
                                o_wb_we <= 1'b0;
                                o_wb_addr <= diag_fail_wb_addr;
                                check_addr <= diag_fail_addr;
                            end
                        end

                        DIAG_REREAD_STREAM: begin
                            o_wb_we <= 1'b0;
                            o_wb_addr <= diag_fail_wb_addr;
                            if (o_wb_stb && !i_wb_stall) begin
                                diag_stream_reads_issued <=
                                    diag_stream_reads_issued + 1'b1;
                                if (diag_stream_reads_issued ==
                                    BIST_DIAG_READS-1)
                                    o_wb_stb <= 1'b0;
                            end
                            if ((diag_stream_reads_issued ==
                                 BIST_DIAG_READS) &&
                                (diag_stream_reads_returned ==
                                 BIST_DIAG_READS) &&
                                (outstanding == 0)) begin
                                o_wb_stb <= 1'b0;
                                if (i_phy_tx_diag_supported) begin
                                    // Begin at absolute tap zero.  The PHY
                                    // captures the BISC-maintained starting
                                    // value before it performs this request.
                                    diag_phase <= DIAG_TX_EYE_REQUEST;
                                    diag_tx_eye_sample <= 7'd0;
                                    diag_tx_eye_pass_map <= 65'd0;
                                    diag_tx_eye_pass_count <= 7'd0;
                                    diag_tx_eye_baseline_valid <= 1'b0;
                                    diag_tx_eye_pass_found <= 1'b0;
                                    diag_tx_eye_bad_seen <= 1'b0;
                                    diag_tx_eye_restore_pending <= 1'b0;
                                    diag_tx_eye_update_error <= 1'b0;
                                    diag_tx_eye_done <= 1'b0;
                                    diag_tx_eye_scan_index <= 8'd0;
                                    diag_tx_eye_scan_run <= 7'd0;
                                    diag_tx_eye_best_run <= 7'd0;
                                    diag_tx_eye_best_end <= 7'd0;
                                    diag_tx_eye_center_tap <= 9'd0;
                                    o_phy_tx_diag_dq <= diag_tx_eye_dq;
                                    o_phy_tx_diag_tap <= 9'd0;
                                    o_phy_tx_diag_req <= 1'b1;
                                end else begin
                                    diag_running <= 1'b0;
                                    diag_done <= 1'b1;
                                    o_wb_cyc <= 1'b0;
                                    bist_state <= BIST_DONE;
                                    if (bist_auto_reset_available)
                                        o_bist_failed_reset_req <= 1'b1;
                                end
                            end
                        end

                        DIAG_TX_EYE_REQUEST: begin
                            // No memory transaction is allowed while a native
                            // output delay is moving.
                            o_wb_stb <= 1'b0;
                            if (i_phy_tx_diag_ack) begin
                                o_phy_tx_diag_req <= 1'b0;
                                if (i_phy_tx_diag_error) begin
                                    diag_tx_eye_update_error <= 1'b1;
                                    diag_tx_eye_done <= 1'b1;
                                    diag_running <= 1'b0;
                                    diag_done <= 1'b1;
                                    o_wb_cyc <= 1'b0;
                                    bist_state <= BIST_DONE;
                                    if (bist_auto_reset_available)
                                        o_bist_failed_reset_req <= 1'b1;
                                end else if (diag_tx_eye_restore_pending) begin
                                    // The production/BISC-maintained tap is
                                    // back in place; leave no diagnostic state
                                    // in the functional datapath.
                                    diag_tx_eye_done <= 1'b1;
                                    diag_running <= 1'b0;
                                    diag_done <= 1'b1;
                                    o_wb_cyc <= 1'b0;
                                    bist_state <= BIST_DONE;
                                    if (bist_auto_reset_available)
                                        o_bist_failed_reset_req <= 1'b1;
                                end else begin
                                    if (!diag_tx_eye_baseline_valid) begin
                                        diag_tx_eye_baseline_tap <=
                                            i_phy_tx_diag_previous_tap;
                                        diag_tx_eye_baseline_valid <= 1'b1;
                                        if (!diag_tx_eye_tuned_mask[
                                                diag_tx_eye_dq])
                                            diag_tx_eye_original_tap[
                                                diag_tx_eye_dq] <=
                                                i_phy_tx_diag_previous_tap;
                                    end
                                    diag_tx_eye_writes_accepted <=
                                        {BIST_TX_EYE_COUNT_BITS{1'b0}};
                                    diag_tx_eye_bad_seen <= 1'b0;
                                    diag_tx_eye_reads_issued <=
                                        {BIST_TX_EYE_COUNT_BITS{1'b0}};
                                    diag_tx_eye_reads_returned <=
                                        {BIST_TX_EYE_COUNT_BITS{1'b0}};
                                    o_wb_we <= 1'b1;
                                    diag_phase <= DIAG_TX_EYE_WRITE;
                                end
                            end
                        end

                        DIAG_TX_EYE_WRITE: begin
                            o_wb_we <= 1'b1;
                            o_wb_sel <= {WB_SEL_BITS{1'b1}};

                            if (!o_wb_stb &&
                                (diag_tx_eye_writes_accepted == 0) &&
                                (outstanding == 0)) begin
                                o_wb_stb <= 1'b1;
                                o_wb_addr <= diag_tx_eye_base_wb_addr;
                                o_wb_data <= tx_eye_pattern(
                                    diag_tx_eye_base_wb_addr[
                                        BIST_ADDR_BITS-1:0],
                                    {BIST_TX_EYE_COUNT_BITS{1'b0}});
                            end else if (o_wb_stb && !i_wb_stall) begin
                                diag_tx_eye_writes_accepted <=
                                    diag_tx_eye_writes_accepted + 1'b1;

                                if (diag_tx_eye_writes_accepted ==
                                    BIST_TX_EYE_WORDS-1) begin
                                    o_wb_stb <= 1'b0;
                                end else begin
                                    o_wb_addr <= diag_tx_eye_base_wb_addr +
                                        diag_tx_eye_writes_accepted + 1'b1;
                                    // Preserve both calibration pattern
                                    // classes at separate addresses so every
                                    // returned word remains independently
                                    // checkable.
                                    o_wb_data <= tx_eye_pattern(
                                        diag_tx_eye_base_wb_addr[
                                            BIST_ADDR_BITS-1:0] +
                                        diag_tx_eye_writes_accepted + 1'b1,
                                        diag_tx_eye_writes_accepted + 1'b1);
                                end
                            end

                            if ((diag_tx_eye_writes_accepted ==
                                 BIST_TX_EYE_WORDS) &&
                                (outstanding == 0) && !o_wb_stb) begin
                                diag_tx_eye_reads_issued <=
                                    {BIST_TX_EYE_COUNT_BITS{1'b0}};
                                diag_tx_eye_reads_returned <=
                                    {BIST_TX_EYE_COUNT_BITS{1'b0}};
                                diag_tx_eye_bad_seen <= 1'b0;
                                o_wb_stb <= 1'b1;
                                o_wb_we <= 1'b0;
                                o_wb_addr <= diag_tx_eye_base_wb_addr;
                                diag_phase <= DIAG_TX_EYE_READ;
                            end
                        end

                        DIAG_TX_EYE_READ: begin
                            o_wb_we <= 1'b0;
                            if (o_wb_stb && !i_wb_stall) begin
                                diag_tx_eye_reads_issued <=
                                    diag_tx_eye_reads_issued + 1'b1;
                                if (diag_tx_eye_reads_issued ==
                                    BIST_TX_EYE_WORDS-1)
                                    o_wb_stb <= 1'b0;
                                else
                                    o_wb_addr <= diag_tx_eye_base_wb_addr +
                                        diag_tx_eye_reads_issued + 1'b1;
                            end

                            if ((diag_tx_eye_reads_issued ==
                                 BIST_TX_EYE_WORDS) &&
                                (diag_tx_eye_reads_returned ==
                                 BIST_TX_EYE_WORDS) &&
                                (outstanding == 0)) begin
                                diag_tx_eye_pass_map[diag_tx_eye_sample] <=
                                    !diag_tx_eye_bad_seen;
                                if (!diag_tx_eye_bad_seen) begin
                                    diag_tx_eye_pass_count <=
                                        diag_tx_eye_pass_count + 1'b1;
                                    diag_tx_eye_last_pass <=
                                        tx_eye_tap_for_sample(
                                            diag_tx_eye_sample);
                                    if (!diag_tx_eye_pass_found) begin
                                        diag_tx_eye_first_pass <=
                                            tx_eye_tap_for_sample(
                                                diag_tx_eye_sample);
                                        diag_tx_eye_pass_found <= 1'b1;
                                    end
                                end

                                o_wb_stb <= 1'b0;
                                if (diag_tx_eye_sample == 7'd64) begin
                                    // All 65 measurements are complete. Tap
                                    // 511 is adjacent to tap zero and is kept
                                    // as a diagnostic endpoint; the circular
                                    // center calculation uses uniform samples
                                    // 0..63 twice.
                                    diag_phase <= DIAG_TX_EYE_ANALYZE;
                                    diag_tx_eye_scan_index <= 8'd0;
                                    diag_tx_eye_scan_run <= 7'd0;
                                    diag_tx_eye_best_run <= 7'd0;
                                    diag_tx_eye_best_end <= 7'd0;
                                end else begin
                                    diag_phase <= DIAG_TX_EYE_REQUEST;
                                    o_phy_tx_diag_req <= 1'b1;
                                    diag_tx_eye_sample <=
                                        diag_tx_eye_sample + 1'b1;
                                    o_phy_tx_diag_tap <=
                                        tx_eye_tap_for_sample(
                                            diag_tx_eye_sample + 1'b1);
                                end
                            end
                        end

                        DIAG_TX_EYE_ANALYZE: begin
                            o_wb_stb <= 1'b0;
                            // One extra cycle after index 127 lets the final
                            // nonblocking best-run update settle before the
                            // center is calculated.
                            if (diag_tx_eye_scan_index == 8'd128) begin
                                diag_phase <= DIAG_TX_EYE_FINALIZE;
                            end else begin
                                if (diag_tx_eye_pass_map[
                                        diag_tx_eye_scan_index[5:0]]) begin
                                    if (diag_tx_eye_scan_run < 7'd64)
                                        diag_tx_eye_scan_run <=
                                            diag_tx_eye_scan_run + 1'b1;
                                    if (((diag_tx_eye_scan_run < 7'd64) ?
                                         (diag_tx_eye_scan_run + 1'b1) :
                                         7'd64) > diag_tx_eye_best_run) begin
                                        diag_tx_eye_best_run <=
                                            (diag_tx_eye_scan_run < 7'd64) ?
                                            (diag_tx_eye_scan_run + 1'b1) :
                                            7'd64;
                                        diag_tx_eye_best_end <=
                                            diag_tx_eye_scan_index[6:0];
                                    end
                                end else begin
                                    diag_tx_eye_scan_run <= 7'd0;
                                end
                                diag_tx_eye_scan_index <=
                                    diag_tx_eye_scan_index + 1'b1;
                            end
                        end

                        DIAG_TX_EYE_FINALIZE: begin
                            o_wb_stb <= 1'b0;
                            if (diag_tx_eye_retry_rank_current >= 10'd512) begin
                                // Every physical tap has failed the
                                // authoritative full BIST. Restore the original
                                // BISC value before reporting a genuine,
                                // bounded uncorrectable failure.
                                diag_tx_eye_restore_pending <= 1'b1;
                                o_phy_tx_diag_tap <=
                                    diag_tx_eye_original_tap[
                                        diag_tx_eye_dq];
                                o_phy_tx_diag_req <= 1'b1;
                                diag_phase <= DIAG_TX_EYE_REQUEST;
                            end else begin
                                if (!diag_tx_eye_tuned_mask[
                                        diag_tx_eye_dq]) begin
                                    diag_tx_eye_seed_tap[
                                        diag_tx_eye_dq] <=
                                        tx_eye_seed_from_run(
                                            diag_tx_eye_best_end,
                                            diag_tx_eye_best_run,
                                            diag_tx_eye_baseline_tap);
                                    diag_tx_eye_center_tap <=
                                        tx_eye_candidate_from_seed(
                                            tx_eye_seed_from_run(
                                                diag_tx_eye_best_end,
                                                diag_tx_eye_best_run,
                                                diag_tx_eye_baseline_tap),
                                            diag_tx_eye_retry_rank_current);
                                end else begin
                                    diag_tx_eye_center_tap <=
                                        tx_eye_candidate_from_seed(
                                            diag_tx_eye_seed_tap[
                                                diag_tx_eye_dq],
                                            diag_tx_eye_retry_rank_current);
                                end
                                // The sweep ended at tap 511. Cross the
                                // circular boundary by one tap, then advance
                                // in <=8-tap updates to the selected center.
                                o_phy_tx_diag_tap <= 9'd0;
                                o_phy_tx_diag_req <= 1'b1;
                                diag_phase <= DIAG_TX_EYE_APPLY_REQUEST;
                            end
                        end

                        DIAG_TX_EYE_APPLY_REQUEST: begin
                            o_wb_stb <= 1'b0;
                            if (i_phy_tx_diag_ack) begin
                                o_phy_tx_diag_req <= 1'b0;
                                if (i_phy_tx_diag_error) begin
                                    diag_tx_eye_update_error <= 1'b1;
                                    diag_tx_eye_done <= 1'b1;
                                    diag_running <= 1'b0;
                                    diag_done <= 1'b1;
                                    o_wb_cyc <= 1'b0;
                                    bist_state <= BIST_DONE;
                                    if (bist_auto_reset_available)
                                        o_bist_failed_reset_req <= 1'b1;
                                end else if (o_phy_tx_diag_tap ==
                                             diag_tx_eye_center_tap) begin
                                    // Retain the centered tap and rerun every
                                    // BIST phase from address zero. A later
                                    // failure on another physical DQ starts
                                    // its independent eye measurement; a
                                    // failure on this DQ rejects this tap and
                                    // advances its bounded candidate rank.
                                    diag_tx_eye_tuned_mask[
                                        diag_tx_eye_dq] <= 1'b1;
                                    diag_tx_eye_done <= 1'b1;
                                    diag_running <= 1'b0;
                                    diag_pending <= 1'b0;
                                    diag_done <= 1'b0;
                                    bist_state <= BIST_BURST_WRITE;
                                    write_addr <= {BIST_ADDR_BITS{1'b0}};
                                    read_addr <= {BIST_ADDR_BITS{1'b0}};
                                    check_addr <= {BIST_ADDR_BITS{1'b0}};
                                    correct_count <= 32'd0;
                                    error_count <= 32'd0;
                                    bist_fail_sticky <= 1'b0;
                                    o_bist_failed_reset_req <= 1'b0;
                                    alt_phase <= 1'b0;
                                    last_read_scrambled <= 1'b0;
                                    last_good_valid <= 1'b0;
                                    o_wb_cyc <= 1'b1;
                                    o_wb_stb <= 1'b1;
                                    o_wb_we <= 1'b1;
                                    o_wb_addr <= {WB_ADDR_BITS{1'b0}};
                                    o_wb_data <= gen_pattern(
                                        {BIST_ADDR_BITS{1'b0}});
                                    o_wb_sel <= BIST_DM_TEST ?
                                        {{(WB_SEL_BITS-1){1'b0}}, 1'b1} :
                                        {WB_SEL_BITS{1'b1}};
                                    write_byte_counter <=
                                        {$clog2(WB_SEL_BITS){1'b0}};
                                    outstanding <= 5'd0;
                                    ack_type_q <= 16'd0;
                                    ack_wr_ptr <= 4'd0;
                                    ack_rd_ptr <= 4'd0;
                                end else begin
                                    diag_phase <= DIAG_TX_EYE_APPLY_GAP;
                                end
                            end
                        end

                        DIAG_TX_EYE_APPLY_GAP: begin
                            // The PHY accepts a new transaction only after it
                            // observes REQ low following ACK.
                            o_phy_tx_diag_req <= 1'b1;
                            if ((o_phy_tx_diag_tap + 9'd8) <
                                diag_tx_eye_center_tap)
                                o_phy_tx_diag_tap <=
                                    o_phy_tx_diag_tap + 9'd8;
                            else
                                o_phy_tx_diag_tap <=
                                    diag_tx_eye_center_tap;
                            diag_phase <= DIAG_TX_EYE_APPLY_REQUEST;
                        end

                        default: begin
                            diag_running <= 1'b0;
                            diag_done <= 1'b1;
                            o_wb_cyc <= 1'b0;
                            o_wb_stb <= 1'b0;
                            bist_state <= BIST_DONE;
                            if (bist_auto_reset_available)
                                o_bist_failed_reset_req <= 1'b1;
                        end
                    endcase
                end else case (bist_state)
                    BIST_IDLE: begin
                        o_bist_failed_reset_req <= 1'b0;
                        if (bist_start_any) begin // triggered by auto-start or CSR write
                            `ifndef YOSYS
                                $display("[%0t] BIST START: first wr data=%0h", $realtime, gen_pattern({BIST_ADDR_BITS{1'b0}}));
                            `endif
                            bist_state    <= BIST_BURST_WRITE;
                            write_addr    <= {BIST_ADDR_BITS{1'b0}};
                            read_addr     <= {BIST_ADDR_BITS{1'b0}};
                            check_addr    <= {BIST_ADDR_BITS{1'b0}};
                            correct_count <= 32'd0;
                            error_count   <= 32'd0;
                            bist_fail_sticky <= 1'b0;
                            diag_pending <= 1'b0;
                            diag_running <= 1'b0;
                            diag_done <= 1'b0;
                            diag_control_valid <= 1'b0;
                            diag_control_reads_issued <= 6'd0;
                            diag_control_reads_returned <= 6'd0;
                            diag_control_match_count <= 6'd0;
                            diag_control_mismatch_count <= 6'd0;
                            diag_stream_writes_accepted <= 7'd0;
                            diag_stream_target_seen <= 1'b0;
                            diag_stream_reads_issued <= 6'd0;
                            diag_stream_reads_returned <= 6'd0;
                            diag_stream_match_count <= 6'd0;
                            diag_stream_mismatch_count <= 6'd0;
                            diag_stream_xor_or <= 8'd0;
                            diag_stream_xor_and <= 8'hff;
                            diag_tx_eye_bad_data_bit <= 8'd0;
                            diag_tx_eye_dq <= 8'd0;
                            diag_tx_eye_sample <= 7'd0;
                            diag_tx_eye_pass_map <= 65'd0;
                            diag_tx_eye_pass_count <= 7'd0;
                            diag_tx_eye_baseline_tap <= 9'd0;
                            diag_tx_eye_baseline_valid <= 1'b0;
                            diag_tx_eye_first_pass <= 9'd0;
                            diag_tx_eye_last_pass <= 9'd0;
                            diag_tx_eye_pass_found <= 1'b0;
                            diag_tx_eye_bad_seen <= 1'b0;
                            diag_tx_eye_writes_accepted <=
                                {BIST_TX_EYE_COUNT_BITS{1'b0}};
                            diag_tx_eye_reads_issued <=
                                {BIST_TX_EYE_COUNT_BITS{1'b0}};
                            diag_tx_eye_reads_returned <=
                                {BIST_TX_EYE_COUNT_BITS{1'b0}};
                            diag_tx_eye_base_wb_addr <=
                                {WB_ADDR_BITS{1'b0}};
                            diag_tx_eye_restore_pending <= 1'b0;
                            diag_tx_eye_update_error <= 1'b0;
                            diag_tx_eye_done <= 1'b0;
                            diag_tx_eye_scan_index <= 8'd0;
                            diag_tx_eye_scan_run <= 7'd0;
                            diag_tx_eye_best_run <= 7'd0;
                            diag_tx_eye_best_end <= 7'd0;
                            diag_tx_eye_center_tap <= 9'd0;
                            o_phy_tx_diag_req <= 1'b0;
                            o_phy_tx_diag_dq <= 8'd0;
                            o_phy_tx_diag_tap <= 9'd0;
                            last_good_valid <= 1'b0;
                            o_wb_cyc      <= 1'b1;
                            o_wb_stb      <= 1'b1;
                            o_wb_we       <= 1'b1;
                            o_wb_addr     <= {WB_ADDR_BITS{1'b0}};
                            o_wb_data     <= gen_pattern({BIST_ADDR_BITS{1'b0}});
                            o_wb_sel      <= BIST_DM_TEST ? {{(WB_SEL_BITS-1){1'b0}}, 1'b1} : {WB_SEL_BITS{1'b1}};
                            write_byte_counter <= {$clog2(WB_SEL_BITS){1'b0}};
                            outstanding   <= 5'd0;
                        end
                    end

                    // -- Burst Sequential Write --
                    // When BIST_DM_TEST=1, writes each byte lane individually
                    // (one-hot o_wb_sel) with 0xAA fill on masked bytes to
                    // stress the data-mask path. Address advances only after
                    // all byte lanes are written. When BIST_DM_TEST=0, streams
                    // full-word writes at full WB throughput.
                    BIST_BURST_WRITE: begin
                        if (!i_wb_stall) begin
                            if (BIST_DM_TEST) begin
                                // -- Per-byte-lane data-mask stress test --
                                // Each address is written WB_SEL_BITS times (once per byte
                                // lane). Only one byte lane is enabled per write (one-hot
                                // o_wb_sel). Unselected bytes carry 0xAA on the bus — if
                                // the data mask fails, 0xAA will corrupt the location and
                                // be caught during read-back.
                                //
                                // Advance byte counter; set next byte's sel/data
                                write_byte_counter <= write_byte_counter + 1'b1;
                                // One-hot byte select shifted to the NEXT byte lane
                                o_wb_sel <= {{(WB_SEL_BITS-1){1'b0}}, 1'b1} << (write_byte_counter + 1'b1);
                                // Real pattern byte at active lane, 0xAA everywhere else
                                o_wb_data <= dm_pattern(write_addr, write_byte_counter + 1'b1);

                                // All byte lanes done for this address — advance to next
                                if (write_byte_counter == {$clog2(WB_SEL_BITS){1'b1}}) begin
                                    write_addr <= write_addr + 1'b1;
                                    o_wb_addr  <= write_addr + 1'b1;
                                    // Reset byte counter to lane 0 for new address
                                    o_wb_sel  <= {{(WB_SEL_BITS-1){1'b0}}, 1'b1};
                                    o_wb_data <= dm_pattern(write_addr + 1'b1, {$clog2(WB_SEL_BITS){1'b0}});
                                    // Check if this was the last address
                                    if (write_addr == BURST_END) begin
                                        `ifndef YOSYS
                                            $display("[%0t] BIST W->R: outstanding=%0d", $realtime, outstanding);
                                        `endif
                                        bist_state <= BIST_BURST_READ;
                                        last_read_scrambled <= 1'b0;
                                        o_wb_we    <= 1'b0;
                                        o_wb_sel   <= {WB_SEL_BITS{1'b1}};
                                        read_addr  <= {BIST_ADDR_BITS{1'b0}};
                                        check_addr <= {BIST_ADDR_BITS{1'b0}};
                                        o_wb_addr  <= {WB_ADDR_BITS{1'b0}};
                                    end
                                end
                            end else begin
                                // -- Full-word burst write (original behavior) --
                                write_addr <= write_addr + 1'b1;
                                o_wb_addr  <= write_addr + 1'b1;
                                o_wb_data  <= gen_pattern(write_addr + 1'b1);
                                if (write_addr == BURST_END) begin
                                    `ifndef YOSYS
                                        $display("[%0t] BIST W->R: outstanding=%0d", $realtime, outstanding);
                                    `endif
                                    bist_state <= BIST_BURST_READ;
                                    last_read_scrambled <= 1'b0;
                                    o_wb_we    <= 1'b0;
                                    read_addr  <= {BIST_ADDR_BITS{1'b0}};
                                    check_addr <= {BIST_ADDR_BITS{1'b0}};
                                    o_wb_addr  <= {WB_ADDR_BITS{1'b0}};
                                end
                            end
                        end
                    end

                    // -- Burst Sequential Read --
                    // Issues reads for the same addresses just written.
                    // Verification happens asynchronously in the ACK checker
                    // above (using check_addr / expected_data).
                    BIST_BURST_READ: begin
                        if (!i_wb_stall) begin // not stalled, so issue next read
                            read_addr <= read_addr + 1'b1;
                            o_wb_addr <= read_addr + 1'b1;
                            if (read_addr == BURST_END) begin
                                o_wb_stb   <= 1'b0;   // stop issuing reads
                                bist_state <= BIST_RANDOM_WRITE;
                            end
                        end
                    end

                    // -- Stress Write (row/bank thrashing) --
                    // Uses stress_addr() for o_wb_addr to distribute counter
                    // bits across row/bank/BG fields, forcing precharge/activate
                    // on nearly every access. Data pattern still uses
                    // scramble_addr() (bit-reversal) for unique verification.
                    BIST_RANDOM_WRITE: begin
                        if (!o_wb_stb) begin // first cycle of this phase, so start issuing writes
                            o_wb_stb <= 1'b1;
                            o_wb_we  <= 1'b1;
                            o_wb_sel <= {WB_SEL_BITS{1'b1}}; // full-word writes for stress phase
                            if (BIST_MODE == 2) begin // full-range: restart at addr=0
                                write_addr <= {BIST_ADDR_BITS{1'b0}};
                                o_wb_addr  <= stress_addr({BIST_ADDR_BITS{1'b0}});
                                o_wb_data  <= gen_pattern(scramble_addr({BIST_ADDR_BITS{1'b0}}));
                            end else begin // partitioned: continue from BURST_END+1
                                write_addr <= BURST_END + 1'b1;
                                o_wb_addr  <= stress_addr(BURST_END + 1'b1);
                                o_wb_data  <= gen_pattern(scramble_addr(BURST_END + 1'b1));
                            end
                        end else if (o_wb_stb && !i_wb_stall) begin // not stalled, so issue next write
                            write_addr <= write_addr + 1'b1;
                            o_wb_addr  <= stress_addr(write_addr + 1'b1);
                            o_wb_data  <= gen_pattern(scramble_addr(write_addr + 1'b1));
                            if (write_addr == RANDOM_END) begin // last write issued, so transition to read phase (notice stb is still high for this current transaction)
                                bist_state <= BIST_RANDOM_READ;
                                last_read_scrambled <= 1'b1;
                                o_wb_we <= 1'b0;
                                if (BIST_MODE == 2) begin // full-range: restart read at addr=0
                                    read_addr  <= {BIST_ADDR_BITS{1'b0}};
                                    check_addr <= {BIST_ADDR_BITS{1'b0}};
                                    o_wb_addr  <= stress_addr({BIST_ADDR_BITS{1'b0}});
                                end else begin // partitioned: read back from BURST_END+1
                                    read_addr  <= BURST_END + 1'b1;
                                    check_addr <= BURST_END + 1'b1;
                                    o_wb_addr  <= stress_addr(BURST_END + 1'b1);
                                end
                            end
                        end
                    end

                    // -- Stress Read (row/bank thrashing) --
                    // Reads back from the same stress_addr() locations written
                    // above; expected data derived from scramble_addr(counter).
                    BIST_RANDOM_READ: begin
                        if (!i_wb_stall && o_wb_stb) begin // not stalled, so issue next read
                            read_addr <= read_addr + 1'b1;
                            o_wb_addr <= stress_addr(read_addr + 1'b1);
                            if (read_addr == RANDOM_END) begin
                                o_wb_stb   <= 1'b0;  // last read issued, so stop issuing reads
                                bist_state <= BIST_ALT_WRITE_READ;
                            end
                        end
                    end

                    // -- Alternating Write/Read (fully pipelined W-R-W-R) --
                    // Issues W0-R0-W1-R1-W2-R2-... back-to-back every cycle
                    // (when !stall). Exercises the controller's in-order ACK
                    // guarantee with interleaved write/read traffic.
                    // Drains prior-phase ACKs first so check_addr is aligned.
                    BIST_ALT_WRITE_READ: begin
                        if (!o_wb_stb) begin
                            if (outstanding == 0) begin // all prior-phase ACKs drained, so start issuing alternating W/R
                                last_read_scrambled <= 1'b0;
                                alt_phase <= 1'b0;
                                o_wb_stb  <= 1'b1;
                                o_wb_we   <= 1'b1;
                                if (BIST_MODE == 2) begin
                                    write_addr <= {BIST_ADDR_BITS{1'b0}};
                                    check_addr <= {BIST_ADDR_BITS{1'b0}};
                                    o_wb_addr  <= {WB_ADDR_BITS{1'b0}};
                                    o_wb_data  <= gen_pattern({BIST_ADDR_BITS{1'b0}});
                                end else begin
                                    write_addr <= RANDOM_END + 1'b1;
                                    check_addr <= RANDOM_END + 1'b1;
                                    o_wb_addr  <= {{(WB_ADDR_BITS-BIST_ADDR_BITS){1'b0}}, RANDOM_END + 1'b1};
                                    o_wb_data  <= gen_pattern(RANDOM_END + 1'b1);
                                end
                            end
                        end else if (!i_wb_stall) begin // not stalled, so issue next W/R
                            if (!alt_phase) begin // current phase is write, so next is read
                                alt_phase <= 1'b1; 
                                o_wb_we   <= 1'b0; 
                                o_wb_addr <= write_addr; // read the same address just written
                            end else begin // current phase is read, so next is write
                                alt_phase  <= 1'b0;
                                write_addr <= write_addr + 1'b1;
                                o_wb_we    <= 1'b1;
                                o_wb_addr  <= write_addr + 1'b1; // write the next address in sequence
                                o_wb_data  <= gen_pattern(write_addr + 1'b1);
                                if (write_addr == ALT_END) begin // last write issued, so stop issuing W/R and transition to finish state
                                    o_wb_stb   <= 1'b0;
                                    bist_state <= BIST_FINISH;
                                end
                            end
                        end
                    end

                    // -- Drain outstanding ACKs --
                    // All phases have finished issuing requests. Wait for
                    // the last in-flight reads to return and be checked.
                    BIST_FINISH: begin
                        o_wb_stb <= 1'b0;  // no more requests
                        if (outstanding == 0) begin
                            o_wb_cyc   <= 1'b0;  // release WB bus ownership
                            bist_state <= BIST_DONE;
                            if (bist_fail_sticky &&
                                bist_auto_reset_available) begin
                                // BIST failed and bounded auto-reset recovery
                                // is enabled, so request a full DDR retrain.
                                o_bist_failed_reset_req <= 1'b1;
                            end
                        end
                    end

                    // Terminal state — stays here until retriggered.
                    BIST_DONE: begin
                        if (bist_start_any) begin
                            bist_state <= BIST_IDLE;
                            o_bist_failed_reset_req <= 1'b0;
                        end
                    end

                    default: bist_state <= BIST_IDLE;
                endcase
            end
        end

        assign o_bist_busy = (bist_state != BIST_IDLE) && (bist_state != BIST_DONE);
        assign bist_pass   = (bist_state == BIST_DONE) && !bist_fail_sticky;
        assign bist_diag_active = diag_pending || diag_running;

    end else begin : gen_no_bist

        assign o_bist_busy = 1'b0;
        assign bist_pass   = 1'b0;
        assign bist_diag_active = 1'b0;

        always @(posedge i_clk) begin
            bist_state           <= BIST_IDLE;
            correct_count        <= 32'd0;
            error_count          <= 32'd0;
            bist_fail_sticky     <= 1'b0;
            // Keep CSR 0xC's reset value consistent even when BIST is
            // compiled out. With BIST enabled, the equivalent reset path
            // above also defaults this fail-safe recovery feature to enabled.
            auto_reset_en        <= 1'b1;
            o_bist_failed_reset_req <= 1'b0;
            o_soft_reset_req     <= 1'b0;
            o_wb_cyc             <= 1'b0;
            o_wb_stb             <= 1'b0;
            o_wb_we              <= 1'b0;
            o_wb_addr            <= {WB_ADDR_BITS{1'b0}};
            o_wb_data            <= {WB_DATA_BITS{1'b0}};
            o_wb_sel             <= {WB_SEL_BITS{1'b0}};
            o_phy_tx_diag_req    <= 1'b0;
            o_phy_tx_diag_dq     <= 8'd0;
            o_phy_tx_diag_tap    <= 9'd0;
        end

    end endgenerate

    // -----------------------------------------------------------------
    // Debug CSR WB Slave + Register File (gated by DEBUG_CSR_ENABLE)
    // -----------------------------------------------------------------
    // WB B4 pipelined, zero-wait-state: STALL=0 always, registered ACK
    // with 1-cycle latency. CSR write enable derived internally.
    //
    // CSR Map (see also the register table in the always @* block below):
    //   0x0 -- STATUS:        Controller + PHY state summary
    //   0x1 -- BANK_STATUS:   Per-bank active/idle (1 bit per bank)
    //   0x2 -- TRAIN_FAIL:    Training failure flags + calib retry count
    //   0x3 -- CORRECT_COUNT: BIST correct read count
    //   0x4 -- ERROR_COUNT:   BIST error read count
    //   0x5 -- BIST_STATUS:   BIST FSM state, busy, pass, fail, init_done/failed
    //   0x6 -- LANE0_TRAINING: IDELAY center, WL DQS tap, bitslip, best_start
    //   0x7 -- LANE1_TRAINING: Same fields (if BYTE_LANES > 1)
    //   0x8 -- EYE_HEALTH:    Eye width per lane, rd_lat_extra, en_vtc
    //   0x9 -- WRITE_PATH:    DQ ODELAY taps, DQS BISC baselines (both lanes)
    //   0xA -- CONFIG:        Static readback (BYTE_LANES, BIST_MODE)
    //   0xB -- VERSION:       IP version (major.minor, currently 0.1)
    //   0xC -- CONTROL:       bit[0] BIST start (W1S), bit[1] soft reset (W1S),
    //                         bit[2] auto-reset on BIST fail (R/W, default 1)
    //   0xD -- INIT_PROGRESS: ROM step, pause, reset_done, pipe_stall
    //
    generate if (DEBUG_CSR_ENABLE) begin : gen_csr

        assign o_wb_dbg_stall = 1'b0;

        reg [31:0] csr_data_r;

        always @(posedge i_clk) begin
            if (!i_rst_n) begin
                o_wb_dbg_ack  <= 1'b0;
                o_wb_dbg_data <= 32'd0;
            end else begin
                o_wb_dbg_ack  <= i_wb_dbg_cyc && i_wb_dbg_stb;
                o_wb_dbg_data <= csr_data_r;
            end
        end

        always @* begin
            case (i_wb_dbg_addr)
                // --- 0x0: STATUS ---
                // [3:0]=PHY FSM state, [7:4]=controller calib state,
                // [8]=stage1 pending, [9]=stage2 pending, [10]=rsvd,
                // [11]=stage2_we, [12]=refresh_idle
                4'h0: csr_data_r = {19'd0,
                                    i_refresh_idle,
                                    i_stage2_we, 1'b0,
                                    i_stage2_pending,
                                    i_stage1_pending,
                                    i_calib_state,
                                    i_phy_state};
                // --- 0x1: BANK_STATUS ---
                // 1 bit per bank: 1=row active, 0=precharged
                4'h1: csr_data_r = {{(32-NUM_BANKS){1'b0}}, i_bank_status};
                // --- 0x2: TRAIN_FAIL ---
                // [BL-1:0]=gate_fail, [2*BL-1:BL]=eye_fail,
                // [3*BL-1:2*BL]=wl_fail, [3*BL+1:3*BL]=calib_retry_count
                4'h2: csr_data_r = {{(32-3*BYTE_LANES-2){1'b0}},
                                    i_calib_retry_count,
                                    i_phy_train_fail};
                // --- 0x3: CORRECT_COUNT ---
                4'h3: csr_data_r = correct_count;
                // --- 0x4: ERROR_COUNT ---
                4'h4: csr_data_r = error_count;
                // --- 0x5: BIST_STATUS ---
                // [2:0]=BIST FSM state, [3]=bist_busy, [4]=bist_pass,
                // [5]=bist_fail_sticky, [6]=init_done, [7]=init_failed
                4'h5: begin
                    csr_data_r = 32'd0;
                    csr_data_r[6] = o_init_done;
                    csr_data_r[7] = o_init_failed;
                    if (BIST_MODE != 0) begin
                        csr_data_r[2:0] = bist_state;
                        csr_data_r[3]   = o_bist_busy;
                        csr_data_r[4]   = bist_pass;
                        csr_data_r[5]   = bist_fail_sticky;
                    end
                end
                // --- 0x6: LANE0_TRAINING ---
                // [8:0]=IDELAY center, [17:9]=WL DQS tap,
                // [21:18]=bitslip, [30:22]=best_start
                4'h6: csr_data_r = {1'd0,
                                    i_phy_best_start[8:0],
                                    i_phy_bitslip[3:0],
                                    i_phy_wl_tap[8:0],
                                    i_phy_idelay_center[8:0]};
                // --- 0x7: LANE1_TRAINING ---
                // [8:0]=IDELAY center, [17:9]=WL DQS tap,
                // [21:18]=bitslip, [30:22]=best_start
                4'h7: begin
                    csr_data_r = 32'd0;
                    if (BYTE_LANES > 1)
                        csr_data_r = {1'd0,
                                      i_phy_best_start[17:9],
                                      i_phy_bitslip[7:4],
                                      i_phy_wl_tap[17:9],
                                      i_phy_idelay_center[17:9]};
                end
                // --- 0x8: EYE_HEALTH ---
                // [8:0]=Lane 0 best_width, [17:9]=Lane 1 best_width,
                // [19:18]=rd_lat_extra, [20]=en_vtc
                4'h8: csr_data_r = {11'd0,
                                    i_phy_en_vtc,
                                    i_phy_rd_lat_extra,
                                    i_phy_best_width[17:9],
                                    i_phy_best_width[8:0]};
                // --- 0x9: WRITE_PATH ---
                // [7:0]=Lane 0 wl_dq_tap, [15:8]=Lane 1 wl_dq_tap,
                // [23:16]=Lane 0 dqs_initial_tap, [31:24]=Lane 1 dqs_initial_tap
                4'h9: csr_data_r = {i_phy_dqs_initial_tap[16:9],
                                    i_phy_dqs_initial_tap[7:0],
                                    i_phy_wl_dq_tap[16:9],
                                    i_phy_wl_dq_tap[7:0]};
                // --- 0xA: CONFIG ---
                // [1:0]=BIST_MODE, [7:4]=BYTE_LANES
                4'hA: csr_data_r = {24'd0, BYTE_LANES[3:0], 2'd0, BIST_MODE};
                // --- 0xB: VERSION ---
                // [7:0]=minor, [15:8]=major
                4'hB: csr_data_r = {16'd0, 8'd0, 8'd1};
                // --- 0xC: CONTROL ---
                // [0]=bist_start (W), [1]=soft_reset (W), [2]=auto_reset_en (RW)
                4'hC: csr_data_r = {29'd0, auto_reset_en, 2'b00};
                // --- 0xD: INIT_PROGRESS ---
                // [5:0]=instruction_address, [6]=pause_counter,
                // [7]=reset_done, [8]=pipe_stall
                4'hD: csr_data_r = {23'd0,
                                    i_pipe_stall,
                                    i_reset_done,
                                    i_pause_counter,
                                    i_instruction_address};
                default: csr_data_r = 32'd0;
            endcase
        end


    end else begin : gen_no_csr

        assign o_wb_dbg_stall = 1'b0;

        always @(posedge i_clk) begin
            o_wb_dbg_ack  <= 1'b0;
            o_wb_dbg_data <= 32'd0;
        end

    end endgenerate

    // -----------------------------------------------------------------
    // Init Status (sticky registers)
    // -----------------------------------------------------------------
    always @(posedge i_clk) begin
        if (prober_internal_reset) begin
            o_init_done   <= 1'b0;
            o_init_failed <= 1'b0;
        end else begin
            if (i_calib_error)
                o_init_failed <= 1'b1;
            if (!o_init_done && !o_init_failed && !i_calib_error) begin
                if (i_calib_complete && (BIST_MODE == 0))
                    o_init_done <= 1'b1;
                if (i_calib_complete && BIST_MODE != 0 && bist_pass)
                    o_init_done <= 1'b1;
                if (i_calib_complete && BIST_MODE != 0 &&
                    bist_fail_sticky &&
                    !(BIST_REREAD_DIAG && bist_diag_active))
                    o_init_failed <= 1'b1;
            end
        end
    end

endmodule
