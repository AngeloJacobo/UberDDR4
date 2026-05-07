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
              BYTE_LANES         = 2,
              NUM_BANKS          = 16,
              ROW_BITS           = 16,
    parameter[0:0] MICRON_SIM    = 0,
    parameter[1:0] BIST_MODE     = 2,
    parameter      DEBUG_CSR_ENABLE = 1
) (
    input  wire                     i_clk,
    input  wire                     i_rst_n,
    // BIST control
    input  wire                     i_start,
    input  wire                     i_calib_complete,
    output wire                     o_bist_busy,
    output wire                     o_bist_pass,
    output wire                     o_bist_fail,
    output wire [31:0]              o_correct_count,
    output wire [31:0]              o_error_count,
    output wire                     o_bist_reset_req,
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
    // Debug CSR port
    input  wire [3:0]               i_csr_sel,
    input  wire                     i_csr_we,
    input  wire [31:0]              i_csr_wdata,
    output wire [31:0]              o_csr_data,
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
    input  wire [3*BYTE_LANES-1:0]  i_phy_bitslip
);

    // ═══════════════════════════════════════════════════════════════════
    // §1 — BIST FSM States
    // ═══════════════════════════════════════════════════════════════════
    localparam [2:0] BIST_IDLE           = 3'd0,
                     BIST_BURST_WRITE    = 3'd1,
                     BIST_BURST_READ     = 3'd2,
                     BIST_RANDOM_WRITE   = 3'd3,
                     BIST_RANDOM_READ    = 3'd4,
                     BIST_ALT_WRITE_READ = 3'd5,
                     BIST_FINISH         = 3'd6,
                     BIST_DONE           = 3'd7;

    localparam BIST_ADDR_BITS = MICRON_SIM ? 10 : WB_ADDR_BITS;
    localparam [BIST_ADDR_BITS-1:0] BURST_END  = {{2{BIST_MODE[1]}}, {(BIST_ADDR_BITS-2){1'b1}}};
    localparam [BIST_ADDR_BITS-1:0] RANDOM_END = {1'b1, BIST_MODE[1], {(BIST_ADDR_BITS-2){1'b1}}};
    localparam [BIST_ADDR_BITS-1:0] ALT_END    = {BIST_ADDR_BITS{1'b1}};

    wire [2:0] bist_state_w;

    // ═══════════════════════════════════════════════════════════════════
    // §2 — BIST Logic (gated by BIST_MODE)
    // ═══════════════════════════════════════════════════════════════════
    generate if (BIST_MODE != 0) begin : gen_bist

        reg bist_csr_start_r, bist_csr_start_d;
        always @(posedge i_clk) begin
            if (!i_rst_n) begin
                bist_csr_start_r <= 1'b0;
                bist_csr_start_d <= 1'b0;
            end else begin
                bist_csr_start_d <= bist_csr_start_r;
                if (i_csr_we && i_csr_sel == 4'hC && i_csr_wdata[0])
                    bist_csr_start_r <= 1'b1;
                else
                    bist_csr_start_r <= 1'b0;
            end
        end

        wire bist_start_any = i_start || bist_csr_start_r || bist_csr_start_d;

        reg [2:0] bist_state;
        reg [BIST_ADDR_BITS-1:0] write_addr;
        reg [BIST_ADDR_BITS-1:0] read_addr;
        reg [BIST_ADDR_BITS-1:0] check_addr;
        reg [31:0] correct_count;
        reg [31:0] error_count;
        reg bist_fail_sticky;
        reg bist_reset_req_r;
        reg alt_phase; // 0=write, 1=read for ALT_WRITE_READ
        reg last_read_scrambled; // remembers if drain phase needs scramble_addr

        reg wb_cyc_r, wb_stb_r, wb_we_r;
        reg [WB_ADDR_BITS-1:0] wb_addr_r;
        reg [WB_DATA_BITS-1:0] wb_data_r;

        // Outstanding request tracking for WB pipelining
        reg [4:0] outstanding;
        reg [8:0] wr_acks_pending;

        // Pattern generation — deterministic from address (XOR-fold full width)
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

        // Bit-reversal address scramble — forces row changes on sequential
        // counter values, maximizing precharge/activate stress
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
                    if (i_wb_data === expected_data) begin
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

                    // ── Phase 1: Burst Sequential Write ──
                    BIST_BURST_WRITE: begin
                        if (!i_wb_stall) begin
                            write_addr <= write_addr + 1'b1;
                            wb_addr_r  <= write_addr + 1'b1;
                            wb_data_r  <= gen_pattern(write_addr + 1'b1);
                            if (write_addr == BURST_END) begin
                                `ifndef YOSYS
                                $display("[%0t] BIST W→R: wr_pend=%0d outstanding=%0d",
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

                    // ── Phase 1: Burst Sequential Read ──
                    BIST_BURST_READ: begin
                        if (!i_wb_stall) begin
                            read_addr <= read_addr + 1'b1;
                            wb_addr_r <= read_addr + 1'b1;
                            if (read_addr == BURST_END) begin
                                wb_stb_r   <= 1'b0;
                                bist_state <= BIST_RANDOM_WRITE;
                            end
                        end
                    end

                    // ── Phase 2: Random-Order Write ──
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

                    // ── Phase 2: Random-Order Read ──
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

                    // ── Phase 3: Alternating Write/Read (serialized per pair) ──
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

                    // ── Drain outstanding ACKs ──
                    BIST_FINISH: begin
                        wb_stb_r <= 1'b0;
                        if (outstanding == 0) begin
                            wb_cyc_r   <= 1'b0;
                            bist_state <= BIST_DONE;
                            if (bist_fail_sticky)
                                bist_reset_req_r <= 1'b1;
                        end
                    end

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

        assign o_bist_busy      = (bist_state != BIST_IDLE) && (bist_state != BIST_DONE);
        assign o_bist_pass      = (bist_state == BIST_DONE) && !bist_fail_sticky;
        assign o_bist_fail      = bist_fail_sticky;
        assign o_correct_count  = correct_count;
        assign o_error_count    = error_count;
        assign o_bist_reset_req = bist_reset_req_r;
        assign o_wb_cyc  = wb_cyc_r;
        assign o_wb_stb  = wb_stb_r;
        assign o_wb_we   = wb_we_r;
        assign o_wb_addr = wb_addr_r;
        assign o_wb_data = wb_data_r;
        assign o_wb_sel  = {WB_SEL_BITS{1'b1}};
        assign bist_state_w = bist_state;

    end else begin : gen_no_bist

        assign o_bist_busy      = 1'b0;
        assign o_bist_pass      = 1'b0;
        assign o_bist_fail      = 1'b0;
        assign o_correct_count  = 32'd0;
        assign o_error_count    = 32'd0;
        assign o_bist_reset_req = 1'b0;
        assign bist_state_w = 3'd0;
        assign o_wb_cyc  = 1'b0;
        assign o_wb_stb  = 1'b0;
        assign o_wb_we   = 1'b0;
        assign o_wb_addr = {WB_ADDR_BITS{1'b0}};
        assign o_wb_data = {WB_DATA_BITS{1'b0}};
        assign o_wb_sel  = {WB_SEL_BITS{1'b0}};

    end endgenerate

    // ═══════════════════════════════════════════════════════════════════
    // §3 — Debug CSR Register File (gated by DEBUG_CSR_ENABLE)
    // ═══════════════════════════════════════════════════════════════════
    generate if (DEBUG_CSR_ENABLE) begin : gen_csr

        reg [31:0] csr_data_r;

        always @* begin
            case (i_csr_sel)
                4'h0: csr_data_r = {19'd0,
                                    i_refresh_idle,
                                    i_stage2_we, 1'b0,
                                    i_stage2_pending,
                                    i_stage1_pending,
                                    i_calib_state,
                                    i_phy_state};
                4'h1: csr_data_r = {{(32-NUM_BANKS){1'b0}}, i_bank_status};
                4'h3: csr_data_r = o_correct_count;
                4'h4: csr_data_r = o_error_count;
                4'h5: begin
                    csr_data_r = 32'd0;
                    if (BIST_MODE != 0) begin
                        csr_data_r[2:0] = bist_state_w;
                        csr_data_r[3]   = o_bist_busy;
                        csr_data_r[4]   = o_bist_pass;
                        csr_data_r[5]   = o_bist_fail;
                    end
                end
                4'h6: csr_data_r = {7'd0,
                                    i_phy_bitslip[2:0],
                                    i_phy_wl_tap[8:0],
                                    i_phy_idelay_center[8:0],
                                    i_phy_state};
                4'h7: begin
                    csr_data_r = 32'd0;
                    if (BYTE_LANES > 1)
                        csr_data_r = {11'd0,
                                      i_phy_bitslip[5:3],
                                      i_phy_wl_tap[17:9],
                                      i_phy_idelay_center[17:9]};
                end
                4'hA: csr_data_r = {24'd0, BYTE_LANES[3:0], 2'd0, BIST_MODE};
                4'hB: csr_data_r = {16'd0, 8'd0, 8'd1}; // version 1.0
                4'hC: csr_data_r = 32'd0; // write-only control register
                default: csr_data_r = 32'd0;
            endcase
        end

        assign o_csr_data = csr_data_r;

    end else begin : gen_no_csr

        assign o_csr_data = 32'd0;

    end endgenerate

endmodule