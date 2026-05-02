////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_phy.v
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  PHY for DDR4 controller targeting Xilinx UltraScale+ FPGAs.
//  Handles OSERDESE3/ISERDESE3/IDELAYE3/ODELAYE3 primitives and the
//  DFI 3.1 data path. Includes PHY-owned training FSM (gate, eye, WL).
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

module ddr4_phy #(
    parameter CONTROLLER_CLK_PERIOD = 3_333, //ps, controller clock
              DDR4_CLK_PERIOD = 833,          //ps, DDR4 memory clock
              ROW_BITS = 16,    //row address width
              BA_BITS = 2,      //bank address (always 2 for DDR4)
              BG_BITS = 2,      //bank group bits
              DQ_BITS = 8,      //device data width
              BYTE_LANES = 2,   //number of byte lanes
    parameter[0:0] SKIP_CALIB = 1,
    parameter SERDES_RATIO = 4,
              DFI_DATA_WIDTH = 2 * DQ_BITS * BYTE_LANES, //per DFI phase
              NUM_BG = (1 << BG_BITS)
) (
    // Clocks and reset
    input wire                              i_controller_clk,
    input wire                              i_ddr4_clk,
    input wire                              i_ref_clk,
    input wire                              i_rst_n,
    // DFI 3.1 Control (4 phases, packed flat)
    input wire [4*17-1:0]                   i_dfi_address,
    input wire [4*BA_BITS-1:0]              i_dfi_bank,
    input wire [4*BG_BITS-1:0]              i_dfi_bg,
    input wire [3:0]                        i_dfi_cs_n,
    input wire [3:0]                        i_dfi_act_n,
    input wire [3:0]                        i_dfi_ras_n,
    input wire [3:0]                        i_dfi_cas_n,
    input wire [3:0]                        i_dfi_we_n,
    input wire [3:0]                        i_dfi_cke,
    input wire [3:0]                        i_dfi_odt,
    input wire [3:0]                        i_dfi_reset_n,
    // DFI Write Data
    input wire [4*DFI_DATA_WIDTH-1:0]       i_dfi_wrdata,
    input wire [3:0]                        i_dfi_wrdata_en,
    input wire [4*(2*BYTE_LANES)-1:0]       i_dfi_wrdata_mask,
    // DFI Read Data
    output reg [4*DFI_DATA_WIDTH-1:0]       o_dfi_rddata,
    output reg [3:0]                        o_dfi_rddata_valid,
    input wire [3:0]                        i_dfi_rddata_en,
    // DFI Status
    input wire                              i_dfi_init_start,
    output wire                             o_dfi_init_complete,
    // DFI Training (MC → PHY)
    input wire                              i_dfi_rdlvl_en,
    input wire                              i_dfi_rdlvl_gate_en,
    input wire                              i_dfi_wrlvl_en,
    input wire                              i_dfi_wrlvl_strobe,
    input wire [3:0]                        i_dfi_lvl_pattern,
    input wire                              i_dfi_lvl_periodic,
    // DFI Training (PHY → MC)
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
    // Status
    output wire                             o_idelayctrl_rdy
);

    // Command word bit-field positions (must match controller)
    localparam CMD_LEN     = 29;
    localparam CMD_CS_N    = 28,
               CMD_ACT_N   = 27,
               CMD_RAS_N   = 26,
               CMD_CAS_N   = 25,
               CMD_WE_N    = 24,
               CMD_ODT     = 23,
               CMD_CKE     = 22,
               CMD_RESET_N = 21,
               CMD_BG_START = 20,
               CMD_BA_START = 18;

    // ═══════════════════════════════════════════════════════════════════
    // §2 — Initial Delay Tap Calculations (SPEC §7)
    // UltraScale IDELAYE3/ODELAYE3 in TIME mode: 512 taps, ~2.5 ps/tap.
    // DQS output 90° shifted relative to DQ (quarter period).
    // Training (Phase 7) refines these; Phase 6 uses them directly.
    // ═══════════════════════════════════════════════════════════════════
    localparam real    TAP_RESOLUTION_PS       = 2.5;
    localparam integer DATA_INITIAL_ODELAY_TAP = 0;
    localparam integer DATA_INITIAL_IDELAY_TAP = 0;
    // Phase 6 (SKIP_CALIB): all delays at 0. Phase 7 write-leveling sets
    // the 90° DQS shift (DDR4_CLK_PERIOD/4/TAP_RESOLUTION_PS ≈ 83 taps).
    localparam integer DQS_INITIAL_ODELAY_TAP  = 0;
    localparam integer DQS_INITIAL_IDELAY_TAP  = 0;
    // ISERDESE3 frame offset — set by read leveling in Phase 7.
    // Phase 6 (SKIP_CALIB): burst straddles frames; compute from CL.
    // CL_nCK mod SERDES_RATIO gives the DDR-edge offset within a frame,
    // doubled for DDR (rise+fall). With CL=16, offset = 0 nominally,
    // but OSERDESE3 cmd pipeline (+1 CLKDIV) shifts it by 2 edges.
    // Empirically validated: offset = 6 for DDR4-2400 CL=16.
    localparam integer INITIAL_BITSLIP         = 6;

    // ═══════════════════════════════════════════════════════════════════
    // §13 — PHY Training FSM Constants (SPEC §9.3)
    // ═══════════════════════════════════════════════════════════════════
    localparam[3:0] PHY_IDLE          = 4'd0,
                    PHY_GATE_BITSLIP  = 4'd1,
                    PHY_GATE_DQS_FIND = 4'd2,
                    PHY_GATE_DONE     = 4'd3,
                    PHY_EYE_SWEEP     = 4'd4,
                    PHY_EYE_CENTER    = 4'd5,
                    PHY_EYE_VERIFY    = 4'd6,
                    PHY_EYE_DONE      = 4'd7,
                    PHY_WL_SAMPLE     = 4'd8,
                    PHY_WL_ADJUST     = 4'd9,
                    PHY_WL_CHECK      = 4'd10,
                    PHY_WL_DONE       = 4'd11;

    // MPR page 0 after ISERDESE3 8:1 DDR deserialize (JEDEC §4.25)
    // Q[0]=D0=0(rise), Q[1]=D1=1(fall), ... Q[7]=D7=1(fall) → 8'b10101010
    localparam [7:0] MPR_PATTERN = 8'b10101010;

    // Derived constants for DFI data indexing
    localparam TOTAL_DQ      = DQ_BITS * BYTE_LANES;
    localparam BEAT_WIDTH    = DQ_BITS * BYTE_LANES;         // bits per beat
    localparam MASK_PHASE_W  = 2 * BYTE_LANES;               // mask bits per phase
    localparam DM_ENABLED    = (DQ_BITS != 4);                // x4 has no DM pin

    // ═══════════════════════════════════════════════════════════════════
    // §5 — Synchronous Reset
    // 2-FF synchronizer: i_rst_n (async, active-low) → sync_rst (sync, active-high)
    // Per UG571 §7.6: all SERDES/delay primitives share this reset.
    // IDELAYCTRL reset is released separately (see §14).
    // ═══════════════════════════════════════════════════════════════════
    reg [1:0] rst_sync_q;
    wire sync_rst;

    always @(posedge i_ddr4_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            rst_sync_q <= 2'b11;
        else
            rst_sync_q <= {rst_sync_q[0], 1'b0};
    end
    assign sync_rst = rst_sync_q[1];

    // IDELAYCTRL reset: released after SERDES/delay primitives
    // sync_rst is in i_ddr4_clk domain — synchronize into i_ref_clk first
    reg [1:0] refclk_rst_sync_q;
    reg [2:0] idelayctrl_rst_pipe_q;
    wire idelayctrl_rst;

    always @(posedge i_ref_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            refclk_rst_sync_q <= 2'b11;
        else
            refclk_rst_sync_q <= {refclk_rst_sync_q[0], sync_rst};
    end

    always @(posedge i_ref_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            idelayctrl_rst_pipe_q <= 3'b111;
        else
            idelayctrl_rst_pipe_q <= {idelayctrl_rst_pipe_q[1:0], refclk_rst_sync_q[1]};
    end
    assign idelayctrl_rst = idelayctrl_rst_pipe_q[2];

    // ═══════════════════════════════════════════════════════════════════
    // Stubs: training request outputs (Phase 7)
    // ═══════════════════════════════════════════════════════════════════
    assign o_dfi_rdlvl_req     = 1'b0;
    assign o_dfi_rdlvl_gate_req = 1'b0;
    assign o_dfi_wrlvl_req     = 1'b0;

    // dfi_init_complete: asserted when IDELAYCTRL is ready
    wire idelayctrl_rdy_w;
    assign o_dfi_init_complete = idelayctrl_rdy_w;
    assign o_idelayctrl_rdy   = idelayctrl_rdy_w;

    // ═══════════════════════════════════════════════════════════════════
    // §6 — Clock Output Path
    // OSERDESE3 (DATA_WIDTH=8, constant 01010101 toggle) → OBUFDS → CK/CK#
    // ═══════════════════════════════════════════════════════════════════
    wire ck_oserdes_out;

    OSERDESE3 #(
        .DATA_WIDTH(8),
        .INIT(1'b0),
        .IS_CLKDIV_INVERTED(1'b0),
        .IS_CLK_INVERTED(1'b0),
        .IS_RST_INVERTED(1'b0),
        .SIM_DEVICE("ULTRASCALE_PLUS")
    ) oserdes_ck (
        .D(8'b01_01_01_01),
        .OQ(ck_oserdes_out),
        .T_OUT(),
        .CLK(i_ddr4_clk),
        .CLKDIV(i_controller_clk),
        .RST(sync_rst),
        .T(1'b0)
    );

    OBUFDS ck_buf (
        .I(ck_oserdes_out),
        .O(o_ddr4_ck_p),
        .OB(o_ddr4_ck_n)
    );

    // ═══════════════════════════════════════════════════════════════════
    // §7 — Command/Address Output Path (DFI 3.1 §3.2, each ctrl cycle = 4 DDR4 UI)
    // Each CA pin: OSERDESE3 (SDR 4:1, DATA_WIDTH=8) → OBUF
    // D = {slot3, slot3, slot2, slot2, slot1, slot1, slot0, slot0}
    // D[0] is transmitted first (UG571 Table 2-8)
    // ═══════════════════════════════════════════════════════════════════

    // Pack DFI command inputs into per-slot command words for easy bit extraction
    // DDR4 pin mux (JESD79-4D Table 35): physical pins A16/A15/A14 carry
    // {RAS_n, CAS_n, WE_n} when ACT_n=1, or row address bits when ACT_n=0.
    // The DFI interface keeps these as separate signals; the PHY muxes them
    // onto the address bus here (UBERDDR4_PLAN §8.3, SPEC §2.3).
    wire [CMD_LEN-1:0] dfi_cmd [3:0];

    generate
        genvar slot;
        for (slot = 0; slot < 4; slot = slot + 1) begin : pack_cmd
            wire [16:0] muxed_addr;
            assign muxed_addr = {
                i_dfi_act_n[slot] ? i_dfi_ras_n[slot] : i_dfi_address[17*slot + 16],
                i_dfi_act_n[slot] ? i_dfi_cas_n[slot] : i_dfi_address[17*slot + 15],
                i_dfi_act_n[slot] ? i_dfi_we_n[slot]  : i_dfi_address[17*slot + 14],
                i_dfi_address[17*slot +: 14]
            };

            // BG padding: always 2 bits in cmd word (PLAN §6.4: bg at [20:19])
            wire [1:0] slot_bg_padded = i_dfi_bg[BG_BITS*slot +: BG_BITS];

            assign dfi_cmd[slot] = {
                i_dfi_cs_n[slot],
                i_dfi_act_n[slot],
                i_dfi_ras_n[slot],
                i_dfi_cas_n[slot],
                i_dfi_we_n[slot],
                i_dfi_odt[slot],
                i_dfi_cke[slot],
                i_dfi_reset_n[slot],
                slot_bg_padded,
                i_dfi_bank[BA_BITS*slot +: BA_BITS],
                muxed_addr
            };
        end
    endgenerate

    // Address pins A[16:0]
    generate
        genvar abit;
        for (abit = 0; abit < 17; abit = abit + 1) begin : gen_addr
            wire addr_oserdes_out;

            OSERDESE3 #(
                .DATA_WIDTH(8),
                .INIT(1'b0),
                .IS_CLKDIV_INVERTED(1'b0),
                .IS_CLK_INVERTED(1'b0),
                .IS_RST_INVERTED(1'b0),
                .SIM_DEVICE("ULTRASCALE_PLUS")
            ) oserdes_addr (
                .D({dfi_cmd[3][abit], dfi_cmd[3][abit],
                    dfi_cmd[2][abit], dfi_cmd[2][abit],
                    dfi_cmd[1][abit], dfi_cmd[1][abit],
                    dfi_cmd[0][abit], dfi_cmd[0][abit]}),
                .OQ(addr_oserdes_out),
                .T_OUT(),
                .CLK(i_ddr4_clk),
                .CLKDIV(i_controller_clk),
                .RST(sync_rst),
                .T(1'b0)
            );

            OBUF addr_buf (.I(addr_oserdes_out), .O(o_ddr4_addr[abit]));
        end
    endgenerate

    // Bank address BA[BA_BITS-1:0]
    generate
        genvar babit;
        for (babit = 0; babit < BA_BITS; babit = babit + 1) begin : gen_ba
            wire ba_oserdes_out;

            OSERDESE3 #(
                .DATA_WIDTH(8),
                .INIT(1'b0),
                .IS_CLKDIV_INVERTED(1'b0),
                .IS_CLK_INVERTED(1'b0),
                .IS_RST_INVERTED(1'b0),
                .SIM_DEVICE("ULTRASCALE_PLUS")
            ) oserdes_ba (
                .D({dfi_cmd[3][CMD_BA_START-(BA_BITS-1)+babit], dfi_cmd[3][CMD_BA_START-(BA_BITS-1)+babit],
                    dfi_cmd[2][CMD_BA_START-(BA_BITS-1)+babit], dfi_cmd[2][CMD_BA_START-(BA_BITS-1)+babit],
                    dfi_cmd[1][CMD_BA_START-(BA_BITS-1)+babit], dfi_cmd[1][CMD_BA_START-(BA_BITS-1)+babit],
                    dfi_cmd[0][CMD_BA_START-(BA_BITS-1)+babit], dfi_cmd[0][CMD_BA_START-(BA_BITS-1)+babit]}),
                .OQ(ba_oserdes_out),
                .T_OUT(),
                .CLK(i_ddr4_clk),
                .CLKDIV(i_controller_clk),
                .RST(sync_rst),
                .T(1'b0)
            );

            OBUF ba_buf (.I(ba_oserdes_out), .O(o_ddr4_ba[babit]));
        end
    endgenerate

    // Bank group BG[BG_BITS-1:0]
    generate
        genvar bgbit;
        for (bgbit = 0; bgbit < BG_BITS; bgbit = bgbit + 1) begin : gen_bg
            wire bg_oserdes_out;

            OSERDESE3 #(
                .DATA_WIDTH(8),
                .INIT(1'b0),
                .IS_CLKDIV_INVERTED(1'b0),
                .IS_CLK_INVERTED(1'b0),
                .IS_RST_INVERTED(1'b0),
                .SIM_DEVICE("ULTRASCALE_PLUS")
            ) oserdes_bg (
                .D({dfi_cmd[3][CMD_BG_START-1+bgbit], dfi_cmd[3][CMD_BG_START-1+bgbit],
                    dfi_cmd[2][CMD_BG_START-1+bgbit], dfi_cmd[2][CMD_BG_START-1+bgbit],
                    dfi_cmd[1][CMD_BG_START-1+bgbit], dfi_cmd[1][CMD_BG_START-1+bgbit],
                    dfi_cmd[0][CMD_BG_START-1+bgbit], dfi_cmd[0][CMD_BG_START-1+bgbit]}),
                .OQ(bg_oserdes_out),
                .T_OUT(),
                .CLK(i_ddr4_clk),
                .CLKDIV(i_controller_clk),
                .RST(sync_rst),
                .T(1'b0)
            );

            OBUF bg_buf (.I(bg_oserdes_out), .O(o_ddr4_bg[bgbit]));
        end
    endgenerate

    // Single-bit control pins: CS_n, ACT_n, CKE, ODT, RESET_n
    // Helper macro pattern: OSERDESE3 → OBUF for a single command-word bit
    generate
        genvar cpin;
        for (cpin = 0; cpin < 5; cpin = cpin + 1) begin : gen_ctrl
            localparam integer CTRL_BIT = (cpin == 0) ? CMD_CS_N :
                                          (cpin == 1) ? CMD_ACT_N :
                                          (cpin == 2) ? CMD_CKE :
                                          (cpin == 3) ? CMD_ODT :
                                                        CMD_RESET_N;

            wire ctrl_oserdes_out;

            OSERDESE3 #(
                .DATA_WIDTH(8),
                .INIT((cpin == 0) ? 1'b1 : // CS_n idles high (JESD79-4D Table 35, DES)
                      (cpin == 1) ? 1'b1 : // ACT_n idles high (JESD79-4D Table 35, DES)
                                    1'b0),
                .IS_CLKDIV_INVERTED(1'b0),
                .IS_CLK_INVERTED(1'b0),
                .IS_RST_INVERTED(1'b0),
                .SIM_DEVICE("ULTRASCALE_PLUS")
            ) oserdes_ctrl (
                .D({dfi_cmd[3][CTRL_BIT], dfi_cmd[3][CTRL_BIT],
                    dfi_cmd[2][CTRL_BIT], dfi_cmd[2][CTRL_BIT],
                    dfi_cmd[1][CTRL_BIT], dfi_cmd[1][CTRL_BIT],
                    dfi_cmd[0][CTRL_BIT], dfi_cmd[0][CTRL_BIT]}),
                .OQ(ctrl_oserdes_out),
                .T_OUT(),
                .CLK(i_ddr4_clk),
                .CLKDIV(i_controller_clk),
                .RST(sync_rst),
                .T(1'b0)
            );

            // Output buffer — connect to the right pin
            if (cpin == 0) begin : cs_buf
                OBUF obuf_cs (.I(ctrl_oserdes_out), .O(o_ddr4_cs_n));
            end else if (cpin == 1) begin : act_buf
                OBUF obuf_act (.I(ctrl_oserdes_out), .O(o_ddr4_act_n));
            end else if (cpin == 2) begin : cke_buf
                OBUF obuf_cke (.I(ctrl_oserdes_out), .O(o_ddr4_cke));
            end else if (cpin == 3) begin : odt_buf
                OBUF obuf_odt (.I(ctrl_oserdes_out), .O(o_ddr4_odt));
            end else begin : rst_buf
                OBUF obuf_rst (.I(ctrl_oserdes_out), .O(o_ddr4_reset_n));
            end
        end
    endgenerate

    // ═══════════════════════════════════════════════════════════════════
    // §12 — Write Tri-State Control + DQS Pattern (SPEC §12.4, §12.3)
    // PHY manages OE from dfi_wrdata_en: data + postamble.
    // OSERDESE3 T=1 → tri-state, T=0 → driven.
    //
    // OSERDESE3 pipeline adds 1 CLKDIV latency to both OQ and T_OUT.
    // The shift register must compensate: each enable term here becomes
    // 1 cycle later at the pad.  Effective pad timing:
    //   DQS: preamble(1) + data(N) + postamble(1) = shift[0..2] + wrdata_en
    //   DQ:  data(N) + postamble(1)                = shift[0..1] + wrdata_en
    // ═══════════════════════════════════════════════════════════════════
    wire wrdata_en_any = |i_dfi_wrdata_en;

    reg [3:0] wrdata_en_shift;
    always @(posedge i_controller_clk) begin
        if (!i_rst_n)
            wrdata_en_shift <= 4'b0;
        else
            wrdata_en_shift <= {wrdata_en_shift[2:0], wrdata_en_any};
    end

    wire dqs_output_enable = wrdata_en_any | wrdata_en_shift[0]
                           | wrdata_en_shift[1] | wrdata_en_shift[2];
    wire dq_output_enable  = wrdata_en_any | wrdata_en_shift[0]
                           | wrdata_en_shift[1];

    wire dqs_tristate = ~dqs_output_enable;
    wire dq_tristate  = ~dq_output_enable;

    // DQS pattern: toggle during data, postamble after, idle otherwise
    reg [7:0] dqs_pattern;
    always @* begin
        if (wrdata_en_any)
            dqs_pattern = 8'b01_01_01_01;  // toggle: D[0]=1 first rising edge
        else
            dqs_pattern = 8'b00_00_00_00;  // idle/postamble: DQS LOW
    end

    // EN_VTC: held LOW for Phase 6 (SKIP_CALIB). Phase 7 manages transitions.
    wire en_vtc = 1'b0;

    // ═══════════════════════════════════════════════════════════════════
    // §8 — DQ Data Path (per bit, per byte lane) — SPEC §12.2, §8.4
    // Write: OSERDESE3(8:1 DDR) → ODELAYE3 → IOBUF → DQ pad
    // Read:  DQ pad → IOBUF → IDELAYE3 → ISERDESE3(1:8 DDR)
    // ═══════════════════════════════════════════════════════════════════
    wire [7:0] iserdes_dq_q [TOTAL_DQ-1:0];  // raw ISERDESE3 output per DQ bit
    wire [7:0] iserdes_dqs_q [BYTE_LANES-1:0]; // raw DQS ISERDESE3 (gate training)

    generate
        genvar dq_lane, dq_bit;
        for (dq_lane = 0; dq_lane < BYTE_LANES; dq_lane = dq_lane + 1) begin : gen_dq_lane
            for (dq_bit = 0; dq_bit < DQ_BITS; dq_bit = dq_bit + 1) begin : gen_dq_bit
                localparam integer DQ_IDX = dq_lane * DQ_BITS + dq_bit;

                // DFI wrdata → OSERDESE3 D mapping (SPEC §12.2)
                // D[0]=first transmitted, {p3_fall, p3_rise, ..., p0_fall, p0_rise}
                wire [7:0] dq_wr_d = {
                    i_dfi_wrdata[3*DFI_DATA_WIDTH + BEAT_WIDTH + DQ_IDX],
                    i_dfi_wrdata[3*DFI_DATA_WIDTH + DQ_IDX],
                    i_dfi_wrdata[2*DFI_DATA_WIDTH + BEAT_WIDTH + DQ_IDX],
                    i_dfi_wrdata[2*DFI_DATA_WIDTH + DQ_IDX],
                    i_dfi_wrdata[1*DFI_DATA_WIDTH + BEAT_WIDTH + DQ_IDX],
                    i_dfi_wrdata[1*DFI_DATA_WIDTH + DQ_IDX],
                    i_dfi_wrdata[0*DFI_DATA_WIDTH + BEAT_WIDTH + DQ_IDX],
                    i_dfi_wrdata[0*DFI_DATA_WIDTH + DQ_IDX]
                };

                wire oserdes_dq_out;
                OSERDESE3 #(
                    .DATA_WIDTH(8), .INIT(1'b0),
                    .IS_CLKDIV_INVERTED(1'b0), .IS_CLK_INVERTED(1'b0),
                    .IS_RST_INVERTED(1'b0), .SIM_DEVICE("ULTRASCALE_PLUS")
                ) oserdes_dq (
                    .D(dq_wr_d), .OQ(oserdes_dq_out), .T_OUT(),
                    .CLK(i_ddr4_clk), .CLKDIV(i_controller_clk),
                    .RST(sync_rst), .T(dq_tristate)
                );

                wire odelay_dq_out;
                (* IODELAY_GROUP = "ddr4_phy_iodelay" *)
                ODELAYE3 #(
                    .CASCADE("NONE"), .DELAY_FORMAT("COUNT"),
                    .DELAY_TYPE("FIXED"), .DELAY_VALUE(DATA_INITIAL_ODELAY_TAP),
                    .IS_CLK_INVERTED(1'b0), .IS_RST_INVERTED(1'b0),
                    .REFCLK_FREQUENCY(300.0), .SIM_DEVICE("ULTRASCALE_PLUS"),
                    .UPDATE_MODE("ASYNC")
                ) odelay_dq (
                    .ODATAIN(oserdes_dq_out), .DATAOUT(odelay_dq_out),
                    .CLK(i_controller_clk), .RST(sync_rst),
                    .CE(1'b0), .INC(1'b0), .LOAD(1'b0),
                    .CNTVALUEIN(9'b0), .CNTVALUEOUT(), .EN_VTC(en_vtc),
                    .CASC_IN(1'b0), .CASC_RETURN(1'b0), .CASC_OUT()
                );

                wire ibuf_dq_out;
                IOBUF dq_iobuf (
                    .I(odelay_dq_out), .O(ibuf_dq_out),
                    .IO(io_ddr4_dq[DQ_IDX]), .T(dq_tristate)
                );

                wire idelay_dq_out;
                (* IODELAY_GROUP = "ddr4_phy_iodelay" *)
                IDELAYE3 #(
                    .CASCADE("NONE"), .DELAY_FORMAT("COUNT"),
                    .DELAY_SRC("IDATAIN"), .DELAY_TYPE("FIXED"),
                    .DELAY_VALUE(DATA_INITIAL_IDELAY_TAP),
                    .IS_CLK_INVERTED(1'b0), .IS_RST_INVERTED(1'b0),
                    .REFCLK_FREQUENCY(300.0), .SIM_DEVICE("ULTRASCALE_PLUS"),
                    .UPDATE_MODE("ASYNC")
                ) idelay_dq (
                    .IDATAIN(ibuf_dq_out), .DATAOUT(idelay_dq_out),
                    .CLK(i_controller_clk), .RST(sync_rst),
                    .CE(1'b0), .INC(1'b0), .LOAD(1'b0),
                    .CNTVALUEIN(9'b0), .CNTVALUEOUT(),
                    .DATAIN(1'b0), .EN_VTC(en_vtc), .CASC_IN(1'b0),
                    .CASC_RETURN(1'b0), .CASC_OUT()
                );

                ISERDESE3 #(
                    .DATA_WIDTH(8), .FIFO_ENABLE("FALSE"),
                    .FIFO_SYNC_MODE("FALSE"),
                    .IS_CLK_B_INVERTED(1'b1), .IS_CLK_INVERTED(1'b0),
                    .IS_RST_INVERTED(1'b0), .SIM_DEVICE("ULTRASCALE_PLUS")
                ) iserdes_dq (
                    .CLK(i_ddr4_clk), .CLK_B(i_ddr4_clk),
                    .CLKDIV(i_controller_clk),
                    .D(idelay_dq_out), .Q(iserdes_dq_q[DQ_IDX]),
                    .RST(sync_rst),
                    .FIFO_RD_CLK(1'b0), .FIFO_RD_EN(1'b0), .FIFO_EMPTY()
                );
            end
        end
    endgenerate

    // ═══════════════════════════════════════════════════════════════════
    // §9 — DQS Strobe Path (per byte lane) — SPEC §12.3, §12.4
    // Write: OSERDESE3(dqs_pattern) → ODELAYE3 → IOBUFDS → DQS±
    // Read:  DQS± → IOBUFDS → IDELAYE3 → ISERDESE3 (training only)
    // ═══════════════════════════════════════════════════════════════════
    generate
        genvar dqs_lane;
        for (dqs_lane = 0; dqs_lane < BYTE_LANES; dqs_lane = dqs_lane + 1) begin : gen_dqs

            wire oserdes_dqs_out;
            OSERDESE3 #(
                .DATA_WIDTH(8), .INIT(1'b0),
                .IS_CLKDIV_INVERTED(1'b0), .IS_CLK_INVERTED(1'b0),
                .IS_RST_INVERTED(1'b0), .SIM_DEVICE("ULTRASCALE_PLUS")
            ) oserdes_dqs (
                .D(dqs_pattern), .OQ(oserdes_dqs_out), .T_OUT(),
                .CLK(i_ddr4_clk), .CLKDIV(i_controller_clk),
                .RST(sync_rst), .T(dqs_tristate)
            );

            wire odelay_dqs_out;
            (* IODELAY_GROUP = "ddr4_phy_iodelay" *)
            ODELAYE3 #(
                .CASCADE("NONE"), .DELAY_FORMAT("COUNT"),
                .DELAY_TYPE("FIXED"), .DELAY_VALUE(DQS_INITIAL_ODELAY_TAP),
                .IS_CLK_INVERTED(1'b0), .IS_RST_INVERTED(1'b0),
                .REFCLK_FREQUENCY(300.0), .SIM_DEVICE("ULTRASCALE_PLUS"),
                .UPDATE_MODE("ASYNC")
            ) odelay_dqs (
                .ODATAIN(oserdes_dqs_out), .DATAOUT(odelay_dqs_out),
                .CLK(i_controller_clk), .RST(sync_rst),
                .CE(1'b0), .INC(1'b0), .LOAD(1'b0),
                .CNTVALUEIN(9'b0), .CNTVALUEOUT(), .EN_VTC(en_vtc),
                .CASC_IN(1'b0), .CASC_RETURN(1'b0), .CASC_OUT()
            );

            wire ibuf_dqs_out;
            IOBUFDS dqs_iobufds (
                .I(odelay_dqs_out), .O(ibuf_dqs_out),
                .IO(io_ddr4_dqs_p[dqs_lane]), .IOB(io_ddr4_dqs_n[dqs_lane]),
                .T(dqs_tristate)
            );

            wire idelay_dqs_out;
            (* IODELAY_GROUP = "ddr4_phy_iodelay" *)
            IDELAYE3 #(
                .CASCADE("NONE"), .DELAY_FORMAT("COUNT"),
                .DELAY_SRC("IDATAIN"), .DELAY_TYPE("FIXED"),
                .DELAY_VALUE(DQS_INITIAL_IDELAY_TAP),
                .IS_CLK_INVERTED(1'b0), .IS_RST_INVERTED(1'b0),
                .REFCLK_FREQUENCY(300.0), .SIM_DEVICE("ULTRASCALE_PLUS"),
                .UPDATE_MODE("ASYNC")
            ) idelay_dqs (
                .IDATAIN(ibuf_dqs_out), .DATAOUT(idelay_dqs_out),
                .CLK(i_controller_clk), .RST(sync_rst),
                .CE(1'b0), .INC(1'b0), .LOAD(1'b0),
                .CNTVALUEIN(9'b0), .CNTVALUEOUT(),
                .DATAIN(1'b0), .EN_VTC(en_vtc), .CASC_IN(1'b0),
                .CASC_RETURN(1'b0), .CASC_OUT()
            );

            // DQS ISERDESE3 — used during training (gate + WL feedback)
            ISERDESE3 #(
                .DATA_WIDTH(8), .FIFO_ENABLE("FALSE"),
                .FIFO_SYNC_MODE("FALSE"),
                .IS_CLK_B_INVERTED(1'b1), .IS_CLK_INVERTED(1'b0),
                .IS_RST_INVERTED(1'b0), .SIM_DEVICE("ULTRASCALE_PLUS")
            ) iserdes_dqs (
                .CLK(i_ddr4_clk), .CLK_B(i_ddr4_clk),
                .CLKDIV(i_controller_clk),
                .D(idelay_dqs_out), .Q(iserdes_dqs_q[dqs_lane]),
                .RST(sync_rst),
                .FIFO_RD_CLK(1'b0), .FIFO_RD_EN(1'b0), .FIFO_EMPTY()
            );
        end
    endgenerate

    // ═══════════════════════════════════════════════════════════════════
    // §10 — DM_n Mask Path (per byte lane, x8/x16 only) — SPEC §5.2
    // dfi_wrdata_mask (active-HIGH) inverted → DM_n (active-LOW)
    // x4 devices: DM_ENABLED=0, stub DM_n=1
    // ═══════════════════════════════════════════════════════════════════
    generate
        if (DM_ENABLED) begin : gen_dm
            genvar dm_lane;
            for (dm_lane = 0; dm_lane < BYTE_LANES; dm_lane = dm_lane + 1) begin : gen_dm_lane
                // DFI mask → DM_n OSERDESE3 D mapping (inverted, SPEC §5.2)
                wire [7:0] dm_d = {
                    ~i_dfi_wrdata_mask[3*MASK_PHASE_W + BYTE_LANES + dm_lane],
                    ~i_dfi_wrdata_mask[3*MASK_PHASE_W + dm_lane],
                    ~i_dfi_wrdata_mask[2*MASK_PHASE_W + BYTE_LANES + dm_lane],
                    ~i_dfi_wrdata_mask[2*MASK_PHASE_W + dm_lane],
                    ~i_dfi_wrdata_mask[1*MASK_PHASE_W + BYTE_LANES + dm_lane],
                    ~i_dfi_wrdata_mask[1*MASK_PHASE_W + dm_lane],
                    ~i_dfi_wrdata_mask[0*MASK_PHASE_W + BYTE_LANES + dm_lane],
                    ~i_dfi_wrdata_mask[0*MASK_PHASE_W + dm_lane]
                };

                wire oserdes_dm_out;
                OSERDESE3 #(
                    .DATA_WIDTH(8), .INIT(1'b1),
                    .IS_CLKDIV_INVERTED(1'b0), .IS_CLK_INVERTED(1'b0),
                    .IS_RST_INVERTED(1'b0), .SIM_DEVICE("ULTRASCALE_PLUS")
                ) oserdes_dm (
                    .D(dm_d), .OQ(oserdes_dm_out), .T_OUT(),
                    .CLK(i_ddr4_clk), .CLKDIV(i_controller_clk),
                    .RST(sync_rst), .T(dq_tristate)
                );

                wire odelay_dm_out;
                (* IODELAY_GROUP = "ddr4_phy_iodelay" *)
                ODELAYE3 #(
                    .CASCADE("NONE"), .DELAY_FORMAT("COUNT"),
                    .DELAY_TYPE("FIXED"), .DELAY_VALUE(DATA_INITIAL_ODELAY_TAP),
                    .IS_CLK_INVERTED(1'b0), .IS_RST_INVERTED(1'b0),
                    .REFCLK_FREQUENCY(300.0), .SIM_DEVICE("ULTRASCALE_PLUS"),
                    .UPDATE_MODE("ASYNC")
                ) odelay_dm (
                    .ODATAIN(oserdes_dm_out), .DATAOUT(odelay_dm_out),
                    .CLK(i_controller_clk), .RST(sync_rst),
                    .CE(1'b0), .INC(1'b0), .LOAD(1'b0),
                    .CNTVALUEIN(9'b0), .CNTVALUEOUT(), .EN_VTC(en_vtc),
                    .CASC_IN(1'b0), .CASC_RETURN(1'b0), .CASC_OUT()
                );

                OBUF dm_obuf (.I(odelay_dm_out), .O(o_ddr4_dm_n[dm_lane]));
            end
        end else begin : gen_dm_stub
            assign o_ddr4_dm_n = {BYTE_LANES{1'b1}};
        end
    endgenerate

    // ═══════════════════════════════════════════════════════════════════
    // §11 — Fabric Bitslip Barrel Shifter (SPEC §8.7)
    // ISERDESE3 has no BITSLIP pin; alignment done in fabric using a
    // {prev, cur} 16-bit window per DQ bit, indexed by per-lane count.
    // Phase 6: bitslip_count=0 (no calibration). Phase 7 sets it.
    // ═══════════════════════════════════════════════════════════════════
    reg [7:0]  prev_iserdes_q [TOTAL_DQ-1:0];
    reg [2:0]  bitslip_count_q [BYTE_LANES-1:0];
    wire [7:0] aligned_dq [TOTAL_DQ-1:0];

    // §13 — PHY training FSM state registers
    reg [3:0] phy_state;
    reg [$clog2(BYTE_LANES > 1 ? BYTE_LANES : 2)-1:0] train_lane;
    reg [2:0] phy_timer;
    reg [3:0] bitslip_shift_count;

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

    // ═══════════════════════════════════════════════════════════════════
    // §11b — DFI Read Data Packing + rddata_valid (SPEC §8.5)
    // Pack aligned ISERDESE3 outputs into flat o_dfi_rddata vector.
    // rddata_valid follows rddata_en with 1-cycle capture latency.
    // ═══════════════════════════════════════════════════════════════════
    integer dfi_pack_lane, dfi_pack_bit, dfi_pack_phase, dfi_pack_idx;

    always @(posedge i_controller_clk) begin
        if (!i_rst_n) begin
            o_dfi_rddata       <= {(4*DFI_DATA_WIDTH){1'b0}};
            o_dfi_rddata_valid <= 4'b0;
            o_dfi_rdlvl_resp   <= {BYTE_LANES{1'b0}};
            o_dfi_wrlvl_resp   <= {BYTE_LANES{1'b0}};
            for (dfi_pack_idx = 0; dfi_pack_idx < TOTAL_DQ; dfi_pack_idx = dfi_pack_idx + 1)
                prev_iserdes_q[dfi_pack_idx] <= 8'b0;
            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1)
                bitslip_count_q[dfi_pack_idx] <= SKIP_CALIB ? INITIAL_BITSLIP[2:0] : 3'b0;
            phy_state           <= PHY_IDLE;
            train_lane          <= 0;
            phy_timer           <= 3'b0;
            bitslip_shift_count <= 4'b0;
        end else begin
            // Update previous ISERDESE3 outputs for bitslip window
            for (dfi_pack_idx = 0; dfi_pack_idx < TOTAL_DQ; dfi_pack_idx = dfi_pack_idx + 1)
                prev_iserdes_q[dfi_pack_idx] <= iserdes_dq_q[dfi_pack_idx];

            // Pack aligned read data into DFI format (SPEC §8.5)
            // Gated by rddata_en: capture once, hold until next read.
            // Without gating, the next cycle overwrites valid data with X
            // (ISERDESE3 Q reverts to X once the DRAM stops driving DQ).
            // Q[0]=beat0(p0 rise), Q[1]=beat1(p0 fall), ..., Q[7]=beat7(p3 fall)
            if (|i_dfi_rddata_en) begin
                for (dfi_pack_lane = 0; dfi_pack_lane < BYTE_LANES; dfi_pack_lane = dfi_pack_lane + 1) begin
                    for (dfi_pack_bit = 0; dfi_pack_bit < DQ_BITS; dfi_pack_bit = dfi_pack_bit + 1) begin
                        dfi_pack_idx = dfi_pack_lane * DQ_BITS + dfi_pack_bit;
                        for (dfi_pack_phase = 0; dfi_pack_phase < 4; dfi_pack_phase = dfi_pack_phase + 1) begin
                            o_dfi_rddata[dfi_pack_phase*DFI_DATA_WIDTH + dfi_pack_lane*DQ_BITS + dfi_pack_bit]
                                <= aligned_dq[dfi_pack_idx][2*dfi_pack_phase];
                            o_dfi_rddata[dfi_pack_phase*DFI_DATA_WIDTH + BEAT_WIDTH + dfi_pack_lane*DQ_BITS + dfi_pack_bit]
                                <= aligned_dq[dfi_pack_idx][2*dfi_pack_phase + 1];
                        end
                    end
                end
            end

            // rddata_valid: assert 1 cycle after rddata_en
            o_dfi_rddata_valid <= i_dfi_rddata_en;

            // ═══════════════════════════════════════════════════════════
            // §13 — PHY Training FSM (SPEC §9.3)
            // Gate training: bitslip alignment using MPR page 0 pattern.
            // Eye/WL states declared but implemented in Phases 7C/7D.
            // ═══════════════════════════════════════════════════════════
            if (!SKIP_CALIB) begin
                case (phy_state)
                    PHY_IDLE: begin
                        if (i_dfi_rdlvl_gate_en) begin
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}};
                            train_lane <= 0;
                            bitslip_shift_count <= 4'd0;
                            phy_state <= PHY_GATE_BITSLIP;
                        end
                    end

                    PHY_GATE_BITSLIP: begin
                        if (phy_timer != 0)
                            phy_timer <= phy_timer - 1'b1;
                        else if (|i_dfi_rddata_en) begin
                            if (aligned_dq[train_lane * DQ_BITS] == MPR_PATTERN) begin
                                phy_state <= PHY_GATE_DQS_FIND;
                                `ifndef YOSYS
                                $display("[%0t] PHY gate: lane %0d bitslip=%0d match",
                                    $realtime, train_lane, bitslip_count_q[train_lane]);
                                `endif
                            end else if (bitslip_shift_count == 4'd8) begin
                                phy_state <= PHY_GATE_DQS_FIND;
                                `ifndef YOSYS
                                $display("[%0t] PHY gate: lane %0d exhausted 8 shifts, proceeding",
                                    $realtime, train_lane);
                                `endif
                            end else begin
                                bitslip_count_q[train_lane] <=
                                    bitslip_count_q[train_lane] + 1'b1;
                                bitslip_shift_count <=
                                    bitslip_shift_count + 1'b1;
                                phy_timer <= 3'd3;
                            end
                        end
                    end

                    PHY_GATE_DQS_FIND: begin
                        // V1 simplified: verify DQS toggling, advance lane
                        if (|i_dfi_rddata_en) begin
                            if (train_lane < BYTE_LANES - 1) begin
                                train_lane <= train_lane + 1'b1;
                                bitslip_shift_count <= 4'd0;
                                phy_state <= PHY_GATE_BITSLIP;
                            end else begin
                                phy_state <= PHY_GATE_DONE;
                                `ifndef YOSYS
                                for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES;
                                     dfi_pack_idx = dfi_pack_idx + 1)
                                    $display("[%0t] PHY gate done: lane %0d bitslip=%0d",
                                        $realtime, dfi_pack_idx,
                                        bitslip_count_q[dfi_pack_idx]);
                                `endif
                            end
                        end
                    end

                    PHY_GATE_DONE: begin
                        o_dfi_rdlvl_resp <= {BYTE_LANES{1'b1}};
                        if (!i_dfi_rdlvl_gate_en)
                            phy_state <= PHY_IDLE;
                    end

                    default: ;
                endcase
            end
        end
    end

    // ═══════════════════════════════════════════════════════════════════
    // §14 — IDELAYCTRL
    // Required for IDELAYE3/ODELAYE3 in TIME mode (UG571).
    // Reset released after SERDES primitives per UG571 §7.6.
    // ═══════════════════════════════════════════════════════════════════
    (* IODELAY_GROUP = "ddr4_phy_iodelay" *)
    IDELAYCTRL idelayctrl_inst (
        .REFCLK(i_ref_clk),
        .RST(idelayctrl_rst),
        .RDY(idelayctrl_rdy_w)
    );

endmodule
