////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_prober.v
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  Combined BIST engine and debug CSR register file. The BIST
//  exercises the DDR4 data path via sequential, random-order, and alternating
//  write/read patterns through a Wishbone B4 master port. The debug CSR
//  provides read-only register access to controller, PHY, and BIST status.
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
              ROW_BITS           = 16,
    // Set to 1 when simulating with Micron DDR4 model (adjusts timing checks)
    parameter[0:0] MICRON_SIM    = 0,
    // BIST_MODE: 0=disabled, 1=burst sequential only, 2=full (burst+random+alternating)
    parameter[1:0] BIST_MODE     = 2,
    // Debug CSR register file: 0=disabled (saves area), 1=enabled
    parameter      DEBUG_CSR_ENABLE = 1
) (
    input  wire                     i_clk,              // System clock (same as controller clock)
    input  wire                     i_rst_n,            // Active-low synchronous reset
    // Calibration status from controller
    input  wire                     i_calib_complete,   // Pulses high when DDR4 init + PHY training finishes successfully
    input  wire                     i_calib_error,      // High if calibration retries exhausted (unrecoverable training failure)
    // Init status (sticky — set once after calibration + optional BIST)
    output reg                      o_init_done,        // Latches high once calibration passes AND BIST passes (or BIST disabled)
    output reg                      o_init_failed,      // Latches high on calibration error OR BIST data mismatch; mutually exclusive with o_init_done
    // BIST status
    output wire                     o_bist_busy,        // High while BIST FSM is actively issuing/checking memory transactions
    output reg                      o_bist_failed_reset_req,   // Auto-reset request: asserted after BIST fail when CSR auto_reset_en is set
    output reg                      o_soft_reset_req,   // CSR-triggered one-shot: resets controller + PHY to re-run full calibration
    // Wishbone B4 Master — BIST drives this to issue R/W to the DDR4 controller
    output reg                      o_wb_cyc,           // Bus cycle active (held high for entire BIST transaction burst)
    output reg                      o_wb_stb,           // Strobe: valid request on addr/data/we this cycle
    output reg                      o_wb_we,            // Write enable: 1=write, 0=read
    output reg  [WB_ADDR_BITS-1:0]  o_wb_addr,          // DDR4 word address
    output reg  [WB_DATA_BITS-1:0]  o_wb_data,          // Write data (full cache-line width)
    output reg  [WB_SEL_BITS-1:0]   o_wb_sel,           // Byte-lane select (always all-ones for BIST)
    input  wire                     i_wb_stall,         // Backpressure from controller: request not accepted this cycle
    input  wire                     i_wb_ack,           // Acknowledge: read data valid or write committed
    input  wire [WB_DATA_BITS-1:0]  i_wb_data,          // Read data returned by controller
    // Wishbone B4 — Debug CSR port (pipelined, zero-wait-state, independent of DRAM path)
    input  wire                     i_wb_dbg_cyc,       // CSR bus cycle
    input  wire                     i_wb_dbg_stb,       // CSR strobe
    input  wire                     i_wb_dbg_we,        // CSR write enable
    input  wire [3:0]               i_wb_dbg_addr,      // CSR register address (selects 1 of 16 registers)
    input  wire [31:0]              i_wb_dbg_data,      // CSR write data
    input  wire [3:0]               i_wb_dbg_sel,       // CSR byte select (unused, always full-word)
    output wire                     o_wb_dbg_stall,     // Always 0: CSR port never stalls
    output reg                      o_wb_dbg_ack,       // Registered ACK (1-cycle latency)
    output reg  [31:0]              o_wb_dbg_data,      // CSR read data
    // Status from controller (exposed via CSR for debug visibility)
    input  wire [3:0]               i_calib_state,      // Controller calibration FSM state (0=IDLE..13=DONE, 14=ERROR)
    input  wire                     i_stage1_pending,   // A new WB request is latched, waiting for stage 2
    input  wire                     i_stage2_pending,   // A decoded request is being scheduled (issuing PRE/ACT/RD/WR)
    input  wire                     i_stage2_we,        // Stage 2 request type: 1=write, 0=read
    input  wire                     i_refresh_idle,     // Refresh timer in idle countdown — scheduler free to issue user commands
    input  wire [NUM_BANKS-1:0]     i_bank_status,      // Per-bank status: 1=row open (active), 0=precharged (idle)
    // Status from PHY (flat packed, exposed via CSR for training debug)
    input  wire [3:0]               i_phy_state,        // PHY training FSM state (0=IDLE, 3=GATE_DONE, 7=EYE_DONE, 11=WL_DONE)
    input  wire [9*BYTE_LANES-1:0]  i_phy_idelay_center, // 9b per lane: IDELAY tap at center of read data eye
    input  wire [9*BYTE_LANES-1:0]  i_phy_wl_tap,       // 9b per lane: ODELAY tap where DQS aligns to CK at DRAM
    input  wire [4*BYTE_LANES-1:0]  i_phy_bitslip,      // 4b per lane: ISERDES barrel-shift aligning capture to burst boundary
    input  wire [3*BYTE_LANES-1:0]  i_phy_train_fail    // Per lane: {wl_fail, eye_fail, gate_fail} — sticky failure flags
);

    // -----------------------------------------------------------------
    // BIST Architecture Overview
    // -----------------------------------------------------------------
    // The BIST engine exercises the DDR4 data path in three phases:
    //
    //   Phase 1 - Burst:       Sequential write of the entire address
    //                          range, then sequential read-back.
    //   Phase 2 - Random:      Bit-reversed (scrambled) address write
    //                          then read-back.  Forces frequent row
    //                          precharge/activate to stress timing.
    //   Phase 3 - Alternating: Write one address, immediately read it
    //                          back, repeat for the full range.  Tests
    //                          tight write-to-read turnaround.
    //
    // BIST_MODE selects which phases run (0=disabled, 1=burst only,
    // 2=all three).  BURST_END / RANDOM_END / ALT_END control how
    // deep each phase sweeps.
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

    // Limit address range to 10 bits when simulating with Micron model
    // (1024 addresses is enough to verify data path without waiting hours).
    localparam BIST_ADDR_BITS = MICRON_SIM ? 10 : WB_ADDR_BITS;
    // Each phase covers a different slice of address space.
    // BIST_MODE[1]=1 enables full-range; =0 reduces to half for mode 1.
    localparam [BIST_ADDR_BITS-1:0] BURST_END  = {{2{BIST_MODE[1]}}, {(BIST_ADDR_BITS-2){1'b1}}};
    localparam [BIST_ADDR_BITS-1:0] RANDOM_END = {1'b1, BIST_MODE[1], {(BIST_ADDR_BITS-2){1'b1}}};
    localparam [BIST_ADDR_BITS-1:0] ALT_END    = {BIST_ADDR_BITS{1'b1}};

    reg [2:0]  bist_state;
    reg [31:0] correct_count;
    reg [31:0] error_count;
    reg        bist_fail_sticky;
    reg        auto_reset_en;
    wire       bist_pass;

    // Module-scope CSR write-enable decode (visible to both gen_bist and gen_csr)
    wire csr_we = i_wb_dbg_cyc && i_wb_dbg_stb && i_wb_dbg_we;

    // -----------------------------------------------------------------
    // BIST Auto-Start (rising edge of calib_complete when BIST enabled)
    // -----------------------------------------------------------------
    wire prober_internal_reset = !i_rst_n || o_soft_reset_req || o_bist_failed_reset_req;

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
                auto_reset_en    <= 1'b0;
            end else begin
                bist_csr_start_d <= bist_csr_start_r;
                bist_csr_start_r <= 1'b0;
                o_soft_reset_req <= 1'b0;
                if (csr_we && i_wb_dbg_addr == 4'hC) begin
                    if (i_wb_dbg_data[0])
                        bist_csr_start_r <= 1'b1;
                    if (i_wb_dbg_data[1])
                        o_soft_reset_req <= 1'b1;
                    auto_reset_en <= i_wb_dbg_data[2];
                end
            end
        end


        // Start from auto-start (calib_complete edge) or CSR write.
        // Two-stage register catches a single-cycle CSR pulse reliably.
        wire bist_start_any = bist_auto_start || bist_csr_start_r || bist_csr_start_d;

        reg [BIST_ADDR_BITS-1:0] write_addr;
        reg [BIST_ADDR_BITS-1:0] read_addr;
        reg [BIST_ADDR_BITS-1:0] check_addr;
        reg alt_phase;
        reg last_read_scrambled;

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

        // Bit-reversal address scramble -- forces row changes on sequential
        // counter values, maximizing precharge/activate stress.
        function [BIST_ADDR_BITS-1:0] scramble_addr;
            input [BIST_ADDR_BITS-1:0] addr;
            integer i;
            begin
                for (i = 0; i < BIST_ADDR_BITS; i = i + 1) begin
                    scramble_addr[i] = addr[BIST_ADDR_BITS-1-i];
                end
            end
        endfunction

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
                    if (i_wb_data == expected_data) begin // read data matches expected pattern
                        correct_count <= correct_count + 1'b1;
                    end else begin // read data mismatch so increment error count and latch fail sticky
                        error_count <= error_count + 1'b1;
                        bist_fail_sticky <= 1'b1;
                        `ifndef YOSYS 
                            $display("[%0t] BIST FAIL: addr=%0h expected=%0h got=%0h", $realtime, check_addr, expected_data, i_wb_data);
                        `endif
                    end
                    check_addr <= check_addr + 1'b1;
                end

                case (bist_state)
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
                            o_wb_cyc      <= 1'b1;
                            o_wb_stb      <= 1'b1;
                            o_wb_we       <= 1'b1;
                            o_wb_addr     <= {WB_ADDR_BITS{1'b0}};
                            o_wb_data     <= gen_pattern({BIST_ADDR_BITS{1'b0}});
                            outstanding   <= 5'd0;
                        end
                    end

                    // -- Burst Sequential Write --
                    // Streams writes at full WB throughput (one per cycle when
                    // not stalled). Writes addr 0..BURST_END with deterministic
                    // pattern, then transitions to read-back.
                    BIST_BURST_WRITE: begin
                        if (!i_wb_stall) begin // not stalled, so issue next write
                            write_addr <= write_addr + 1'b1;
                            o_wb_addr  <= write_addr + 1'b1;  // pre-compute next address
                            o_wb_data  <= gen_pattern(write_addr + 1'b1);
                            if (write_addr == BURST_END) begin // last write issued, so transition to read phase (notice stb is still high for this current transaction)
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

                    // -- Random-Order Write (bit-reversed addresses) --
                    BIST_RANDOM_WRITE: begin
                        if (!o_wb_stb) begin // first cycle of this phase, so start issuing writes
                            o_wb_stb <= 1'b1;
                            o_wb_we  <= 1'b1;
                            if (BIST_MODE == 2) begin // full-range random write, so start at addr=0
                                write_addr <= {BIST_ADDR_BITS{1'b0}};
                                o_wb_addr  <= scramble_addr({BIST_ADDR_BITS{1'b0}});
                                o_wb_data  <= gen_pattern(scramble_addr({BIST_ADDR_BITS{1'b0}}));
                            end else begin // half-range random write, so start at addr=BURST_END+1
                                write_addr <= BURST_END + 1'b1;
                                o_wb_addr  <= scramble_addr(BURST_END + 1'b1);
                                o_wb_data  <= gen_pattern(scramble_addr(BURST_END + 1'b1));
                            end
                        end else if (o_wb_stb && !i_wb_stall) begin // not stalled, so issue next write
                            write_addr <= write_addr + 1'b1;
                            o_wb_addr  <= scramble_addr(write_addr + 1'b1);
                            o_wb_data  <= gen_pattern(scramble_addr(write_addr + 1'b1));
                            if (write_addr == RANDOM_END) begin // last write issued, so transition to read phase (notice stb is still high for this current transaction)
                                bist_state <= BIST_RANDOM_READ;
                                last_read_scrambled <= 1'b1;
                                o_wb_we <= 1'b0;
                                if (BIST_MODE == 2) begin // full-range random read, so start at addr=0
                                    read_addr  <= {BIST_ADDR_BITS{1'b0}};
                                    check_addr <= {BIST_ADDR_BITS{1'b0}};
                                    o_wb_addr  <= scramble_addr({BIST_ADDR_BITS{1'b0}});
                                end else begin // half-range random read, so start at addr=BURST_END+1
                                    read_addr  <= BURST_END + 1'b1;
                                    check_addr <= BURST_END + 1'b1;
                                    o_wb_addr  <= scramble_addr(BURST_END + 1'b1);
                                end
                            end
                        end
                    end

                    // -- Random-Order Read --
                    BIST_RANDOM_READ: begin
                        if (!i_wb_stall && o_wb_stb) begin // not stalled, so issue next read
                            read_addr <= read_addr + 1'b1;
                            o_wb_addr <= scramble_addr(read_addr + 1'b1);
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
                                write_addr <= {BIST_ADDR_BITS{1'b0}};
                                check_addr <= {BIST_ADDR_BITS{1'b0}};
                                o_wb_stb  <= 1'b1;
                                o_wb_we   <= 1'b1;
                                o_wb_addr <= {WB_ADDR_BITS{1'b0}};
                                o_wb_data <= gen_pattern({BIST_ADDR_BITS{1'b0}});
                                alt_phase <= 1'b0;
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
                            if (bist_fail_sticky && auto_reset_en) begin // BIST failed and auto-reset is enabled, so request full DDR reset
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

    end else begin : gen_no_bist

        assign o_bist_busy = 1'b0;
        assign bist_pass   = 1'b0;

        always @(posedge i_clk) begin
            bist_state           <= BIST_IDLE;
            correct_count        <= 32'd0;
            error_count          <= 32'd0;
            bist_fail_sticky     <= 1'b0;
            auto_reset_en        <= 1'b0;
            o_bist_failed_reset_req <= 1'b0;
            o_soft_reset_req     <= 1'b0;
            o_wb_cyc             <= 1'b0;
            o_wb_stb             <= 1'b0;
            o_wb_we              <= 1'b0;
            o_wb_addr            <= {WB_ADDR_BITS{1'b0}};
            o_wb_data            <= {WB_DATA_BITS{1'b0}};
            o_wb_sel             <= {WB_SEL_BITS{1'b0}};
        end

    end endgenerate

    // -----------------------------------------------------------------
    // Debug CSR WB Slave + Register File (gated by DEBUG_CSR_ENABLE)
    // -----------------------------------------------------------------
    // WB B4 pipelined, zero-wait-state: STALL=0 always, registered ACK
    // with 1-cycle latency. CSR write enable derived internally.
    //
    // CSR Map:
    //   0x0 -- Controller + PHY state summary
    //   0x1 -- Per-bank active/idle status
    //   0x2 -- Training failure status {wl[BL-1:0], eye[BL-1:0], gate[BL-1:0]}
    //   0x3 -- BIST correct count
    //   0x4 -- BIST error count
    //   0x5 -- BIST FSM state + pass/fail flags
    //   0x6 -- PHY lane 0: IDELAY center, write-leveling tap, bitslip
    //   0x7 -- PHY lane 1 (same fields, if BYTE_LANES > 1)
    //   0xA -- Configuration readback (BYTE_LANES, BIST_MODE)
    //   0xB -- IP version (currently 1.0)
    //   0xC -- Control: bit[0] BIST re-start (W1S), bit[1] soft reset (W1S),
    //                   bit[2] auto-reset on BIST fail enable (R/W, default 0)
    //
    generate if (DEBUG_CSR_ENABLE) begin : gen_csr

        assign o_wb_dbg_stall = 1'b0;

        always @(posedge i_clk) begin
            if (!i_rst_n) begin
                o_wb_dbg_ack  <= 1'b0;
                o_wb_dbg_data <= 32'd0;
            end else begin
                o_wb_dbg_ack  <= i_wb_dbg_cyc && i_wb_dbg_stb;
                o_wb_dbg_data <= csr_data_r;
            end
        end

        reg [31:0] csr_data_r;

        always @* begin
            case (i_wb_dbg_addr)
                // [3:0]=PHY FSM state, [7:4]=controller calib state,
                // [8]=stage1 pending, [9]=stage2 pending, [10]=rsvd,
                // [11]=stage2_we, [12]=refresh_idle, [31:13]=0
                4'h0: csr_data_r = {19'd0,
                                    i_refresh_idle,
                                    i_stage2_we, 1'b0,
                                    i_stage2_pending,
                                    i_stage1_pending,
                                    i_calib_state,
                                    i_phy_state};
                4'h1: csr_data_r = {{(32-NUM_BANKS){1'b0}}, i_bank_status}; // 1 bit per bank: 1=active, 0=idle
                // [BL-1:0]=gate_fail, [2*BL-1:BL]=eye_fail, [3*BL-1:2*BL]=wl_fail
                4'h2: csr_data_r = {{(32-3*BYTE_LANES){1'b0}}, i_phy_train_fail};
                4'h3: csr_data_r = correct_count;
                4'h4: csr_data_r = error_count;
                4'h5: begin
                    csr_data_r = 32'd0;
                    if (BIST_MODE != 0) begin
                        csr_data_r[2:0] = bist_state;
                        csr_data_r[3]   = o_bist_busy;
                        csr_data_r[4]   = bist_pass;
                        csr_data_r[5]   = bist_fail_sticky;
                    end
                end
                // Lane 0: [3:0]=PHY state, [12:4]=IDELAY center tap,
                //          [21:13]=WL DQS tap, [25:22]=bitslip count
                4'h6: csr_data_r = {6'd0,
                                    i_phy_bitslip[3:0],
                                    i_phy_wl_tap[8:0],
                                    i_phy_idelay_center[8:0],
                                    i_phy_state};
                // Lane 1 (same layout without phy_state): [8:0]=IDELAY,
                //          [17:9]=WL tap, [21:18]=bitslip
                4'h7: begin
                    csr_data_r = 32'd0;
                    if (BYTE_LANES > 1)
                        csr_data_r = {10'd0,
                                      i_phy_bitslip[7:4],
                                      i_phy_wl_tap[17:9],
                                      i_phy_idelay_center[17:9]};
                end
                // [1:0]=BIST_MODE, [7:4]=BYTE_LANES — static config readback
                4'hA: csr_data_r = {24'd0, BYTE_LANES[3:0], 2'd0, BIST_MODE};
                4'hB: csr_data_r = {16'd0, 8'd0, 8'd1}; // IP version: major=0, minor=1
                4'hC: csr_data_r = {29'd0, auto_reset_en, 2'b00};
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
                if (i_calib_complete && BIST_MODE != 0 && bist_fail_sticky)
                    o_init_failed <= 1'b1;
            end
        end
    end

endmodule