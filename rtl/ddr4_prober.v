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
) ( // Q: add correct in-line commment to each ports
    input  wire                     i_clk,
    input  wire                     i_rst_n,
    // Calibration status from controller
    input  wire                     i_calib_complete,
    input  wire                     i_calib_error,
    // Init status (sticky — set once after calibration + optional BIST)
    output wire                     o_init_done,
    output wire                     o_init_failed,
    // BIST status
    output wire                     o_bist_busy,
    output wire                     o_bist_pass,
    output wire                     o_bist_fail,
    output wire                     o_bist_reset_req, // Q: is it possible to make this writable via CSR to reset the ddr4 controller + phy? Is it safe? or i_rst_n (porb) is safer if user wants to reset and repeat calibration?
    // Wishbone B4 Master
    output wire                     o_wb_cyc,
    output wire                     o_wb_stb,
    output wire                     o_wb_we,
    output wire [WB_ADDR_BITS-1:0]  o_wb_addr,
    output wire [WB_DATA_BITS-1:0]  o_wb_data,
    output wire [WB_SEL_BITS-1:0]   o_wb_sel,
    input  wire                     i_wb_stall,
    input  wire                     i_wb_ack,
    input  wire [WB_DATA_BITS-1:0]  i_wb_data,
    // Wishbone B4 — Debug CSR port (pipelined, zero-wait-state)
    input  wire                     i_wb_dbg_cyc,
    input  wire                     i_wb_dbg_stb,
    input  wire                     i_wb_dbg_we,
    input  wire [3:0]               i_wb_dbg_addr,
    input  wire [31:0]              i_wb_dbg_data,
    input  wire [3:0]               i_wb_dbg_sel,
    output wire                     o_wb_dbg_stall,
    output wire                     o_wb_dbg_ack,
    output wire [31:0]              o_wb_dbg_data,
    // Status from controller
    input  wire [3:0]               i_calib_state,
    input  wire                     i_stage1_pending,
    input  wire                     i_stage2_pending,
    input  wire                     i_stage2_we,
    input  wire                     i_refresh_idle,
    input  wire [NUM_BANKS-1:0]     i_bank_status,
    // Status from PHY (flat packed)
    input  wire [3:0]               i_phy_state,
    input  wire [9*BYTE_LANES-1:0]  i_phy_idelay_center,
    input  wire [9*BYTE_LANES-1:0]  i_phy_wl_tap,
    input  wire [3*BYTE_LANES-1:0]  i_phy_bitslip,
    input  wire [3*BYTE_LANES-1:0]  i_phy_train_fail
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
    //   1. Auto-start -- ddr4_top pulses i_start on rising edge of
    //      calib_complete (one-shot).
    //   2. CSR trigger -- software writes bit[0] of CSR register 0xC.
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

    wire [2:0] bist_state_w;
    wire [31:0] correct_count_w;
    wire [31:0] error_count_w;

    // Module-scope CSR write-enable decode (visible to both gen_bist and gen_csr)
    wire csr_we = i_wb_dbg_cyc && i_wb_dbg_stb && i_wb_dbg_we;

    // -----------------------------------------------------------------
    // BIST Auto-Start (rising edge of calib_complete when BIST enabled)
    // -----------------------------------------------------------------
    reg calib_complete_q;
    always @(posedge i_clk) begin
        if (!i_rst_n)
            calib_complete_q <= 1'b0;
        else
            calib_complete_q <= i_calib_complete;
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
            end else begin
                bist_csr_start_d <= bist_csr_start_r;
                if (csr_we && i_wb_dbg_addr == 4'hC && i_wb_dbg_data[0])
                    bist_csr_start_r <= 1'b1;
                else
                    bist_csr_start_r <= 1'b0;
            end
        end

        // Start from auto-start (calib_complete edge) or CSR write.
        // Two-stage register catches a single-cycle CSR pulse reliably.
        wire bist_start_any = bist_auto_start || bist_csr_start_r || bist_csr_start_d;

        reg [2:0] bist_state;
        reg [BIST_ADDR_BITS-1:0] write_addr;  // next address to write
        reg [BIST_ADDR_BITS-1:0] read_addr;   // next address to issue read for
        reg [BIST_ADDR_BITS-1:0] check_addr;  // next address whose ACK we expect to verify
        reg [31:0] correct_count;              // total matching reads
        reg [31:0] error_count;                // total mismatching reads
        reg bist_fail_sticky;                  // latches on first mismatch, never clears until restart
        reg bist_reset_req_r;                  // requests full DDR reset if BIST failed
        reg alt_phase;             // 0=write phase, 1=read phase in ALT_WRITE_READ
        reg last_read_scrambled;   // tracks which gen_pattern to use for late-arriving ACKs

        reg wb_cyc_r, wb_stb_r, wb_we_r;   // WB bus control (cyc=bus ownership, stb=valid xfer, we=write)
        reg [WB_ADDR_BITS-1:0] wb_addr_r;  // current WB address being driven
        reg [WB_DATA_BITS-1:0] wb_data_r;  // current WB write data being driven

        // Outstanding request tracking for WB pipelining.
        // `outstanding` counts total in-flight requests (writes + reads).
        // `wr_acks_pending` counts write ACKs not yet received.  WB B4
        // returns ACKs in-order, so when wr_acks_pending==0 the next ACK
        // is guaranteed to be a read -- that is when we compare data.
        reg [4:0] outstanding;
        reg [8:0] wr_acks_pending;

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
                for (i = 0; i < BIST_ADDR_BITS; i = i + 1)
                    scramble_addr[i] = addr[BIST_ADDR_BITS-1-i];
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
            if (!i_rst_n) begin
                bist_state      <= BIST_IDLE;
                write_addr      <= {BIST_ADDR_BITS{1'b0}};
                read_addr       <= {BIST_ADDR_BITS{1'b0}};
                check_addr      <= {BIST_ADDR_BITS{1'b0}};
                correct_count   <= 32'd0;
                error_count     <= 32'd0;
                bist_fail_sticky <= 1'b0;
                bist_reset_req_r <= 1'b0;
                alt_phase       <= 1'b0;
                last_read_scrambled <= 1'b0;
                wb_cyc_r        <= 1'b0;
                wb_stb_r        <= 1'b0;
                wb_we_r         <= 1'b0;
                wb_addr_r       <= {WB_ADDR_BITS{1'b0}};
                wb_data_r       <= {WB_DATA_BITS{1'b0}};
                outstanding     <= 4'd0;
                wr_acks_pending <= 9'd0;
            end else begin

                // Track outstanding requests
                if (wb_stb_r && !i_wb_stall && i_wb_ack)
                    outstanding <= outstanding; // issued+acked same cycle
                else if (wb_stb_r && !i_wb_stall)
                    outstanding <= outstanding + 1'b1;
                else if (i_wb_ack)
                    outstanding <= outstanding - 1'b1;

                // Track outstanding write ACKs (WB B4 returns ACKs in order)
                if (wb_stb_r && !i_wb_stall && wb_we_r && i_wb_ack && wr_acks_pending > 0)
                    wr_acks_pending <= wr_acks_pending; // issued write + consumed write ack
                else if (wb_stb_r && !i_wb_stall && wb_we_r)
                    wr_acks_pending <= wr_acks_pending + 1'b1;
                else if (i_wb_ack && wr_acks_pending > 0)
                    wr_acks_pending <= wr_acks_pending - 1'b1;

                // Data check on ACK (wr_acks_pending==0 guarantees it is a read ACK)
                if (i_wb_ack && wr_acks_pending == 0 &&
                    bist_state != BIST_IDLE &&
                    bist_state != BIST_DONE &&
                    bist_state != BIST_BURST_WRITE) begin
                    `ifndef YOSYS
                    if (check_addr < 20)
                        $display("[%0t] BIST CHK: addr=%0d exp=%0h got=%0h state=%0d",
                            $realtime, check_addr, expected_data, i_wb_data, bist_state);
                    `endif
                    if (i_wb_data == expected_data) begin
                        correct_count <= correct_count + 1'b1;
                    end else begin
                        error_count <= error_count + 1'b1;
                        bist_fail_sticky <= 1'b1;
                        `ifndef YOSYS
                        $display("[%0t] BIST FAIL: addr=%0h expected=%0h got=%0h",
                            $realtime, check_addr, expected_data, i_wb_data);
                        `endif
                    end
                    check_addr <= check_addr + 1'b1;
                end

                case (bist_state)
                    BIST_IDLE: begin
                        bist_reset_req_r <= 1'b0;
                        if (bist_start_any && i_calib_complete) begin
                            `ifndef YOSYS
                            $display("[%0t] BIST START: first wr data=%0h",
                                $realtime, gen_pattern({BIST_ADDR_BITS{1'b0}}));
                            `endif
                            bist_state    <= BIST_BURST_WRITE;
                            write_addr    <= {BIST_ADDR_BITS{1'b0}};
                            read_addr     <= {BIST_ADDR_BITS{1'b0}};
                            check_addr    <= {BIST_ADDR_BITS{1'b0}};
                            correct_count <= 32'd0;
                            error_count   <= 32'd0;
                            bist_fail_sticky <= 1'b0;
                            wb_cyc_r      <= 1'b1;
                            wb_stb_r      <= 1'b1;
                            wb_we_r       <= 1'b1;
                            wb_addr_r     <= {WB_ADDR_BITS{1'b0}};
                            wb_data_r     <= gen_pattern({BIST_ADDR_BITS{1'b0}});
                            outstanding   <= 4'd0;
                        end
                    end

                    // -- Burst Sequential Write --
                    // Streams writes at full WB throughput (one per cycle when
                    // not stalled). Writes addr 0..BURST_END with deterministic
                    // pattern, then transitions to read-back.
                    BIST_BURST_WRITE: begin
                        if (!i_wb_stall) begin
                            write_addr <= write_addr + 1'b1;
                            wb_addr_r  <= write_addr + 1'b1;  // pre-compute next address
                            wb_data_r  <= gen_pattern(write_addr + 1'b1);
                            if (write_addr == BURST_END) begin
                                `ifndef YOSYS
                                $display("[%0t] BIST W->R: wr_pend=%0d outstanding=%0d",
                                    $realtime, wr_acks_pending, outstanding);
                                `endif
                                bist_state <= BIST_BURST_READ;
                                last_read_scrambled <= 1'b0;
                                wb_we_r    <= 1'b0;
                                read_addr  <= {BIST_ADDR_BITS{1'b0}};
                                check_addr <= {BIST_ADDR_BITS{1'b0}};
                                wb_addr_r  <= {WB_ADDR_BITS{1'b0}};
                            end
                        end
                    end

                    // -- Burst Sequential Read --
                    // Issues reads for the same addresses just written.
                    // Verification happens asynchronously in the ACK checker
                    // above (using check_addr / expected_data).
                    BIST_BURST_READ: begin
                        if (!i_wb_stall) begin
                            read_addr <= read_addr + 1'b1;
                            wb_addr_r <= read_addr + 1'b1;
                            if (read_addr == BURST_END) begin
                                wb_stb_r   <= 1'b0;   // stop issuing reads
                                bist_state <= BIST_RANDOM_WRITE;
                            end
                        end
                    end

                    // -- Random-Order Write (bit-reversed addresses) --
                    // Waits for all burst-read ACKs to drain (outstanding==0)
                    // before starting. Uses bit-reversed addresses so sequential
                    // counter values map to maximally different row addresses,
                    // stressing precharge/activate timing in the controller.
                    BIST_RANDOM_WRITE: begin
                        if (outstanding == 0 && !wb_stb_r) begin
                            wb_stb_r <= 1'b1;
                            wb_we_r  <= 1'b1;
                            if (BIST_MODE == 2) begin
                                write_addr <= {BIST_ADDR_BITS{1'b0}};
                                wb_addr_r  <= scramble_addr({BIST_ADDR_BITS{1'b0}});
                                wb_data_r  <= gen_pattern(scramble_addr({BIST_ADDR_BITS{1'b0}}));
                            end else begin
                                wb_addr_r  <= scramble_addr(write_addr);
                                wb_data_r  <= gen_pattern(scramble_addr(write_addr));
                            end
                        end else if (wb_stb_r && !i_wb_stall) begin
                            write_addr <= write_addr + 1'b1;
                            wb_addr_r  <= scramble_addr(write_addr + 1'b1);
                            wb_data_r  <= gen_pattern(scramble_addr(write_addr + 1'b1));
                            if (write_addr == RANDOM_END) begin
                                bist_state <= BIST_RANDOM_READ;
                                last_read_scrambled <= 1'b1;
                                wb_we_r <= 1'b0;
                                if (BIST_MODE == 2) begin
                                    read_addr  <= {BIST_ADDR_BITS{1'b0}};
                                    check_addr <= {BIST_ADDR_BITS{1'b0}};
                                    wb_addr_r  <= scramble_addr({BIST_ADDR_BITS{1'b0}});
                                end else begin
                                    read_addr  <= BURST_END + 1'b1;
                                    check_addr <= BURST_END + 1'b1;
                                    wb_addr_r  <= scramble_addr(BURST_END + 1'b1);
                                end
                            end
                        end
                    end

                    // -- Random-Order Read --
                    BIST_RANDOM_READ: begin
                        if (!i_wb_stall && wb_stb_r) begin
                            read_addr <= read_addr + 1'b1;
                            wb_addr_r <= scramble_addr(read_addr + 1'b1);
                            if (read_addr == RANDOM_END) begin
                                wb_stb_r   <= 1'b0;
                                bist_state <= BIST_ALT_WRITE_READ;
                            end
                        end
                    end

                    // -- Alternating Write/Read (serialized per address) --
                    // Write one address, wait for ACK, read it back, wait for
                    // ACK, compare. Repeat for all addresses. Tests tight
                    // write-to-read turnaround (tWTR) at the controller level.
                    // Serialized: only 1 outstanding request at a time.
                    BIST_ALT_WRITE_READ: begin
                        if (outstanding == 0 && !wb_stb_r) begin
                            if (!alt_phase) begin
                                last_read_scrambled <= 1'b0;
                                wb_stb_r  <= 1'b1;
                                wb_we_r   <= 1'b1;
                                wb_addr_r <= write_addr;
                                wb_data_r <= gen_pattern(write_addr);
                            end else begin
                                wb_stb_r   <= 1'b1;
                                wb_we_r    <= 1'b0;
                                wb_addr_r  <= write_addr;
                                check_addr <= write_addr;
                            end
                        end else if (wb_stb_r && !i_wb_stall) begin
                            wb_stb_r <= 1'b0;
                            if (!alt_phase) begin
                                alt_phase <= 1'b1;
                            end else begin
                                alt_phase  <= 1'b0;
                                write_addr <= write_addr + 1'b1;
                                if (write_addr == ALT_END)
                                    bist_state <= BIST_FINISH;
                            end
                        end
                    end

                    // -- Drain outstanding ACKs --
                    // All phases have finished issuing requests. Wait for
                    // the last in-flight reads to return and be checked.
                    BIST_FINISH: begin
                        wb_stb_r <= 1'b0;  // no more requests
                        if (outstanding == 0) begin
                            wb_cyc_r   <= 1'b0;  // release WB bus ownership
                            bist_state <= BIST_DONE;
                            if (bist_fail_sticky)
                                bist_reset_req_r <= 1'b1;  // signal top to re-init DDR
                        end
                    end

                    // Terminal state — stays here until retriggered.
                    BIST_DONE: begin
                        if (bist_start_any) begin
                            bist_state <= BIST_IDLE;
                            bist_reset_req_r <= 1'b0;
                        end
                    end

                    default: bist_state <= BIST_IDLE;
                endcase
            end
        end

        assign o_bist_busy      = (bist_state != BIST_IDLE) && (bist_state != BIST_DONE); // high while BIST is running
        assign o_bist_pass      = (bist_state == BIST_DONE) && !bist_fail_sticky;       // only valid after BIST completes
        assign o_bist_fail      = bist_fail_sticky;  // sticky — stays high once any mismatch seen
        assign correct_count_w  = correct_count;
        assign error_count_w    = error_count;
        assign o_bist_reset_req = bist_reset_req_r;
        assign o_wb_cyc  = wb_cyc_r;
        assign o_wb_stb  = wb_stb_r;
        assign o_wb_we   = wb_we_r;
        assign o_wb_addr = wb_addr_r;
        assign o_wb_data = wb_data_r;
        assign o_wb_sel  = {WB_SEL_BITS{1'b1}};  // always full-width (all bytes selected)
        assign bist_state_w = bist_state;         // exported for CSR readback

    end else begin : gen_no_bist

        assign o_bist_busy      = 1'b0;
        assign o_bist_pass      = 1'b0;
        assign o_bist_fail      = 1'b0;
        assign correct_count_w  = 32'd0;
        assign error_count_w    = 32'd0;
        assign o_bist_reset_req = 1'b0;
        assign bist_state_w = 3'd0;
        assign o_wb_cyc  = 1'b0;
        assign o_wb_stb  = 1'b0;
        assign o_wb_we   = 1'b0;
        assign o_wb_addr = {WB_ADDR_BITS{1'b0}};
        assign o_wb_data = {WB_DATA_BITS{1'b0}};
        assign o_wb_sel  = {WB_SEL_BITS{1'b0}};

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
    //   0xC -- Write-only: bit[0] triggers BIST start
    //
    generate if (DEBUG_CSR_ENABLE) begin : gen_csr

        // WB slave: zero-wait-state, registered ACK
        assign o_wb_dbg_stall = 1'b0;

        reg wb_dbg_ack_r;
        reg [31:0] wb_dbg_data_r;
        always @(posedge i_clk) begin
            if (!i_rst_n) begin
                wb_dbg_ack_r  <= 1'b0;
                wb_dbg_data_r <= 32'd0;
            end else begin
                wb_dbg_ack_r  <= i_wb_dbg_cyc && i_wb_dbg_stb;
                wb_dbg_data_r <= csr_data_r;
            end
        end
        assign o_wb_dbg_ack  = wb_dbg_ack_r;
        assign o_wb_dbg_data = wb_dbg_data_r;

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
                4'h3: csr_data_r = correct_count_w;  // reads verified OK
                4'h4: csr_data_r = error_count_w;    // reads with mismatch
                // [2:0]=BIST FSM state, [3]=busy, [4]=pass, [5]=fail
                4'h5: begin
                    csr_data_r = 32'd0;
                    if (BIST_MODE != 0) begin
                        csr_data_r[2:0] = bist_state_w;
                        csr_data_r[3]   = o_bist_busy;
                        csr_data_r[4]   = o_bist_pass;
                        csr_data_r[5]   = o_bist_fail;
                    end
                end
                // Lane 0: [3:0]=PHY state, [12:4]=IDELAY center tap,
                //          [21:13]=WL DQS tap, [24:22]=bitslip count
                4'h6: csr_data_r = {7'd0,
                                    i_phy_bitslip[2:0],
                                    i_phy_wl_tap[8:0],
                                    i_phy_idelay_center[8:0],
                                    i_phy_state};
                // Lane 1 (same layout without phy_state): [8:0]=IDELAY,
                //          [17:9]=WL tap, [20:18]=bitslip
                4'h7: begin
                    csr_data_r = 32'd0;
                    if (BYTE_LANES > 1)
                        csr_data_r = {11'd0,
                                      i_phy_bitslip[5:3],
                                      i_phy_wl_tap[17:9],
                                      i_phy_idelay_center[17:9]};
                end
                // [1:0]=BIST_MODE, [7:4]=BYTE_LANES — static config readback
                4'hA: csr_data_r = {24'd0, BYTE_LANES[3:0], 2'd0, BIST_MODE};
                4'hB: csr_data_r = {16'd0, 8'd0, 8'd1}; // IP version: major=0, minor=1
                4'hC: csr_data_r = 32'd0; // write-only: bit[0] triggers BIST re-run
                default: csr_data_r = 32'd0;
            endcase
        end


    end else begin : gen_no_csr

        assign o_wb_dbg_stall = 1'b0;
        assign o_wb_dbg_ack   = 1'b0;
        assign o_wb_dbg_data  = 32'd0;

    end endgenerate

    // -----------------------------------------------------------------
    // Init Status (sticky registers)
    // -----------------------------------------------------------------
    // init_done:   calibration OK and (BIST passed or BIST disabled)
    // init_failed: calibration error OR BIST failure
    reg init_done_q, init_failed_q;
    always @(posedge i_clk) begin
        if (!i_rst_n) begin
            init_done_q   <= 1'b0;
            init_failed_q <= 1'b0;
        end else begin
            if (i_calib_error)
                init_failed_q <= 1'b1;
            if (!init_done_q && !init_failed_q && !i_calib_error) begin
                if (i_calib_complete && (BIST_MODE == 0))
                    init_done_q <= 1'b1;
                if (i_calib_complete && BIST_MODE != 0 && o_bist_pass)
                    init_done_q <= 1'b1;
                if (i_calib_complete && BIST_MODE != 0 && o_bist_fail)
                    init_failed_q <= 1'b1;
            end
        end
    end
    assign o_init_done   = init_done_q;
    assign o_init_failed = init_failed_q;

endmodule