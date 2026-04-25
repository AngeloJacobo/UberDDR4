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
    reg [2:0] idelayctrl_rst_pipe_q;
    wire idelayctrl_rst;

    always @(posedge i_ref_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            idelayctrl_rst_pipe_q <= 3'b111;
        else
            idelayctrl_rst_pipe_q <= {idelayctrl_rst_pipe_q[1:0], sync_rst};
    end
    assign idelayctrl_rst = idelayctrl_rst_pipe_q[2];

    // ═══════════════════════════════════════════════════════════════════
    // Stubs: training request outputs (Phase 7), DM output (Phase 6)
    // ═══════════════════════════════════════════════════════════════════
    assign o_dfi_rdlvl_req     = 1'b0;
    assign o_dfi_rdlvl_gate_req = 1'b0;
    assign o_dfi_wrlvl_req     = 1'b0;
    assign o_ddr4_dm_n         = {BYTE_LANES{1'b1}};

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
    // §7 — Command/Address Output Path
    // Each CA pin: OSERDESE3 (SDR 4:1, DATA_WIDTH=8) → OBUF
    // D = {slot3, slot3, slot2, slot2, slot1, slot1, slot0, slot0}
    // D[0] is transmitted first (UG571 Table 2-8)
    // ═══════════════════════════════════════════════════════════════════

    // Pack DFI command inputs into per-slot command words for easy bit extraction
    wire [CMD_LEN-1:0] dfi_cmd [3:0];

    generate
        genvar slot;
        for (slot = 0; slot < 4; slot = slot + 1) begin : pack_cmd
            assign dfi_cmd[slot] = {
                i_dfi_cs_n[slot],
                i_dfi_act_n[slot],
                i_dfi_ras_n[slot],
                i_dfi_cas_n[slot],
                i_dfi_we_n[slot],
                i_dfi_odt[slot],
                i_dfi_cke[slot],
                i_dfi_reset_n[slot],
                i_dfi_bg[BG_BITS*slot +: BG_BITS],
                i_dfi_bank[BA_BITS*slot +: BA_BITS],
                i_dfi_address[17*slot +: 17]
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
                .D({dfi_cmd[3][CMD_BG_START-(BG_BITS-1)+bgbit], dfi_cmd[3][CMD_BG_START-(BG_BITS-1)+bgbit],
                    dfi_cmd[2][CMD_BG_START-(BG_BITS-1)+bgbit], dfi_cmd[2][CMD_BG_START-(BG_BITS-1)+bgbit],
                    dfi_cmd[1][CMD_BG_START-(BG_BITS-1)+bgbit], dfi_cmd[1][CMD_BG_START-(BG_BITS-1)+bgbit],
                    dfi_cmd[0][CMD_BG_START-(BG_BITS-1)+bgbit], dfi_cmd[0][CMD_BG_START-(BG_BITS-1)+bgbit]}),
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
                .INIT((cpin == 0) ? 1'b1 : // CS_n idles high
                      (cpin == 1) ? 1'b1 : // ACT_n idles high
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

    // ═══════════════════════════════════════════════════════════════════
    // Stub: reg output defaults (Phase 6+ fills in data path)
    // ═══════════════════════════════════════════════════════════════════
    always @(posedge i_controller_clk) begin
        if (!i_rst_n) begin
            o_dfi_rddata       <= {(4*DFI_DATA_WIDTH){1'b0}};
            o_dfi_rddata_valid <= 4'b0;
            o_dfi_rdlvl_resp   <= {BYTE_LANES{1'b0}};
            o_dfi_wrlvl_resp   <= {BYTE_LANES{1'b0}};
        end else begin
            // Phase 6: DQ/DQS data path
            // Phase 7: Training FSM
        end
    end

endmodule
