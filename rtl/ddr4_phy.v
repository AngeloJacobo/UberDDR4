////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_phy.v
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  PHY for DDR4 controller targeting Xilinx UltraScale+ FPGAs.
//  Handles OSERDESE3/ISERDESE3/IDELAYE3/ODELAYE3 primitives and the
//  DFI 3.1 data path. Includes PHY-owned training FSM (gate, eye, WL).
//
// Architecture overview:
//  The PHY sits between the DFI 3.1 interface and the DDR4 SDRAM pins.
//  One controller clock cycle = 4 DDR4 unit intervals (8:1 DDR SERDES).
//
//  Write path:  DFI wrdata -> OSERDESE3 (8:1 DDR) -> ODELAYE3 -> IOBUF -> pad
//  Read path:   pad -> IOBUF -> IDELAYE3 -> ISERDESE3 (1:8 DDR) -> bitslip
//               barrel shifter -> DFI rddata
//  Clock path:  OSERDESE3 (constant 01010101 toggle) -> OBUFDS -> CK/CK#
//  Cmd/Addr:    OSERDESE3 (SDR 4:1, doubled bits) -> OBUF -> DDR4 CA pins
//
//  Training FSM (runs once after IDELAYCTRL ready, driven by MC):
//   1. Gate training:  Bitslip alignment using MPR page 0 pattern.
//                      ISERDESE3 has no BITSLIP pin, so we do it in fabric
//                      with a 16-bit barrel shifter per DQ bit.
//   2. Eye training:   Sweep IDELAYE3 taps across the DQ data eye, find
//                      first/last passing tap, load the center tap.
//   3. Write leveling: Sweep ODELAYE3 DQS tap to find the 0->1 CK edge
//                      on DQ[0]. DQ ODELAYE3 tracks DQS to keep 90 deg.
//
//  EN_VTC (voltage-temperature compensation): held LOW during training
//  so delay taps can be changed. Set HIGH in normal operation so the
//  IDELAYE3/ODELAYE3 primitives track PVT drift automatically.
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
    // Number of 8-bit byte lanes (typically 2 for x8, 2 for x16, 2+ for x4)
              BYTE_LANES = 2,
    // Derived from DEVICE_WIDTH -- do not override
    parameter BA_BITS = 2,      //bank address (always 2 for DDR4)
              BG_BITS = (DEVICE_WIDTH == 16) ? 1 : 2, //JESD79-4D Table 2
              DQ_BITS = 8,      //always 8 (byte-lane granularity)
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
    // DFI Training (MC -> PHY)
    input wire                              i_dfi_rdlvl_en,
    input wire                              i_dfi_rdlvl_gate_en,
    input wire                              i_dfi_wrlvl_en,
    input wire                              i_dfi_wrlvl_strobe,
    input wire [3:0]                        i_dfi_lvl_pattern,
    input wire                              i_dfi_lvl_periodic,
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
    // Status
    output wire                             o_idelayctrl_rdy,
    // Debug status (flat packed for synthesis)
    output wire [3:0]                       o_phy_state,
    output wire [9*BYTE_LANES-1:0]          o_phy_idelay_center,
    output wire [9*BYTE_LANES-1:0]          o_phy_wl_tap,
    output wire [3*BYTE_LANES-1:0]          o_phy_bitslip
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

    // -----------------------------------------------------------------
    // ODELAYE3/IDELAYE3 delay configuration
    // DQS output is 90 deg shifted relative to DQ via ODELAYE3 (ps).
    // BISC calibrates ps->taps automatically (UG571 ch.2, p.183).
    // Write leveling reads CNTVALUEOUT for the BISC-calibrated starting tap.
    // -----------------------------------------------------------------
    localparam integer DATA_INITIAL_ODELAY_TAP = 0;
    localparam integer DATA_INITIAL_IDELAY_TAP = 0;
    localparam integer DQS_ODELAY_PS = DDR4_CLK_PERIOD / 4;
    localparam integer DQS_INITIAL_IDELAY_TAP  = 0;
    // -----------------------------------------------------------------
    // PHY Training FSM state encoding
    // -----------------------------------------------------------------
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

    // MPR page 0 after ISERDESE3 8:1 DDR deserialize (JEDEC JESD79-4D ch.4.25)
    // Q[0]=D0=0(rise), Q[1]=D1=1(fall), ... Q[7]=D7=1(fall) -> 8'b10101010
    localparam [7:0] MPR_PATTERN = 8'b10101010;

    // Eye training: sweep IDELAYE3 in steps of 4 (512/4 = 128 iterations)
    localparam [3:0] TAP_SWEEP_STEP = 4'd4;

    // Write leveling: sweep ODELAYE3 DQS in steps of 4
    localparam [3:0] WL_TAP_STEP = 4'd4;

    // VTC settle: ~200 controller_clk cycles after EN_VTC assertion (UG571)
    localparam [7:0] VTC_SETTLE_CYCLES = 8'd200;

    // Derived constants for DFI data indexing
    localparam TOTAL_DQ      = DQ_BITS * BYTE_LANES;
    localparam BEAT_WIDTH    = DQ_BITS * BYTE_LANES;         // bits per beat
    localparam MASK_PHASE_W  = 2 * BYTE_LANES;               // mask bits per phase
    localparam DM_ENABLED    = (DEVICE_WIDTH != 4);            // x4 has no DM pin (JESD79-4D Table 28)

    // -----------------------------------------------------------------
    // Synchronous Reset
    // 2-FF synchronizer: i_rst_n (async, active-low) -> sync_rst (sync, active-high)
    // Per UG571 ch.7.6: all SERDES/delay primitives share this reset.
    // IDELAYCTRL reset is released separately (see IDELAYCTRL section below).
    //
    // Why 2-FF? Async assert, sync deassert avoids metastability on the
    // deassert edge. The shift register flushes in 2 clocks.
    // -----------------------------------------------------------------
    reg [1:0] rst_sync_q;
    wire sync_rst;

    always @(posedge i_ddr4_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            rst_sync_q <= 2'b11;
        else
            rst_sync_q <= {rst_sync_q[0], 1'b0};
    end
    assign sync_rst = rst_sync_q[1];

    // 2-FF synchronizer for i_controller_clk domain (training FSM reset)
    reg [1:0] ctrl_rst_sync_q;
    wire ctrl_rst_n;

    always @(posedge i_controller_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            ctrl_rst_sync_q <= 2'b00;
        else
            ctrl_rst_sync_q <= {ctrl_rst_sync_q[0], 1'b1};
    end
    assign ctrl_rst_n = ctrl_rst_sync_q[1];

    // IDELAYCTRL reset: released after SERDES/delay primitives.
    // sync_rst is in i_ddr4_clk domain -- synchronize into i_ref_clk first.
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

    // -----------------------------------------------------------------
    // DFI training request outputs
    // PHY-initiated training requests are not used; MC drives training.
    // -----------------------------------------------------------------
    assign o_dfi_rdlvl_req     = 1'b0;
    assign o_dfi_rdlvl_gate_req = 1'b0;
    assign o_dfi_wrlvl_req     = 1'b0;

    // dfi_init_complete: asserted when IDELAYCTRL is ready
    wire idelayctrl_rdy_w;
    assign o_dfi_init_complete = idelayctrl_rdy_w;
    assign o_idelayctrl_rdy   = idelayctrl_rdy_w;

    // -----------------------------------------------------------------
    // Clock Output Path
    // OSERDESE3 (DATA_WIDTH=8, constant 01010101 toggle) -> OBUFDS -> CK/CK#
    // The SERDES toggles every UI, producing the DDR4 memory clock.
    // -----------------------------------------------------------------
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

    // -----------------------------------------------------------------
    // Command/Address Output Path (DFI 3.1, each ctrl cycle = 4 DDR4 UI)
    // Each CA pin: OSERDESE3 (SDR 4:1, DATA_WIDTH=8) -> OBUF
    // D = {slot3, slot3, slot2, slot2, slot1, slot1, slot0, slot0}
    // D[0] is transmitted first (UG571 Table 2-8).
    // Each DFI phase maps to one DDR4 command slot. Bits are doubled
    // because OSERDESE3 DATA_WIDTH=8 in DDR mode gives 4 edges, but
    // CA pins are SDR (active on rising edge only). Doubling each bit
    // ensures the same value appears on both the rising and falling
    // edge of each UI, so the DRAM sees a clean SDR command.
    // -----------------------------------------------------------------

    // Pack DFI command inputs into per-slot command words for easy bit extraction
    // DDR4 pin mux (JESD79-4D Table 35): physical pins A16/A15/A14 carry
    // {RAS_n, CAS_n, WE_n} when ACT_n=1, or row address bits when ACT_n=0.
    // The DFI interface keeps these as separate signals; the PHY muxes them
    // onto the address bus here.
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

            // BG padding: always 2 bits in cmd word (bg at [20:19])
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
    // Helper macro pattern: OSERDESE3 -> OBUF for a single command-word bit
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

            // Output buffer -- connect to the right pin
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

    // -----------------------------------------------------------------
    // Write Tri-State Control + DQS Pattern
    // PHY manages OE from dfi_wrdata_en: data + postamble.
    // OSERDESE3 T=1 -> tri-state, T=0 -> driven.
    //
    // OSERDESE3 pipeline adds 1 CLKDIV latency to both OQ and T_OUT.
    // The shift register must compensate: each enable term here becomes
    // 1 cycle later at the pad. Effective pad timing:
    //   DQS: preamble(1) + data(N) + postamble(1) = shift[0..2] + wrdata_en
    //   DQ:  data(N) + postamble(1)                = shift[0..1] + wrdata_en
    // -----------------------------------------------------------------
    wire wrdata_en_any = |i_dfi_wrdata_en;

    reg [3:0] wrdata_en_shift;
    always @(posedge i_controller_clk) begin
        if (!ctrl_rst_n)
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

    // WL state detection -- used by DQS pattern and tristate overrides
    wire wl_active;

    // DQS pattern: toggle during data, single rising edge during WL strobe
    reg [7:0] dqs_pattern;
    reg       wl_dqs_strobe;
    always @* begin
        if (wl_active) begin
            if (wl_dqs_strobe)
                dqs_pattern = 8'b00_00_00_01;
            else
                dqs_pattern = 8'b00_00_00_00;
        end else if (wrdata_en_any)
            dqs_pattern = 8'b01_01_01_01;
        else
            dqs_pattern = 8'b00_00_00_00;
    end

    reg  wl_dqs_strobe_d1;
    wire wl_dqs_drive = wl_dqs_strobe | wl_dqs_strobe_d1;
    wire dqs_tristate_wl = wl_active ? ~wl_dqs_drive : dqs_tristate;

    // EN_VTC: LOW during training (tap changes), HIGH in normal operation (UG571).
    // TIME mode requires VTC active after calibration for PVT drift compensation.
    reg en_vtc_q;

    // -----------------------------------------------------------------
    // DQ Data Path (per bit, per byte lane)
    // Write: OSERDESE3(8:1 DDR) -> ODELAYE3 -> IOBUF -> DQ pad
    // Read:  DQ pad -> IOBUF -> IDELAYE3 -> ISERDESE3(1:8 DDR)
    //
    // ISERDESE3 Q[7:0] mapping (8:1 DDR deserialize, UG571 Table 3-2):
    //   Q[0] = first bit captured (phase 0, rise)
    //   Q[1] = second bit captured (phase 0, fall)
    //   ...
    //   Q[7] = eighth bit captured (phase 3, fall)
    // So Q[2*p] = DFI phase p rise beat, Q[2*p+1] = DFI phase p fall beat.
    //
    // OSERDESE3 D[7:0] mapping (8:1 DDR serialize, UG571 Table 2-8):
    //   D[0] = first bit transmitted (phase 0, rise)
    //   D[7] = last bit transmitted (phase 3, fall)
    // -----------------------------------------------------------------
    wire [7:0] iserdes_dq_q  [TOTAL_DQ-1:0];
    wire [7:0] iserdes_dqs_q [BYTE_LANES-1:0]; // raw DQS ISERDESE3 (gate training)

    // Eye training: per-lane IDELAYE3 LOAD pulse and shared tap value
    reg  idelay_load_lane [BYTE_LANES-1:0];
    reg  [8:0] idelay_cntvalue;

    // Write leveling: per-lane ODELAYE3 LOAD pulses and shared tap values
    reg  odelay_dqs_load [BYTE_LANES-1:0];
    reg  [8:0] odelay_dqs_cntvalue;
    reg  odelay_dq_load  [BYTE_LANES-1:0];
    reg  [8:0] odelay_dq_cntvalue;
    wire [8:0] odelay_dqs_cntvalueout [BYTE_LANES-1:0];

    generate
        genvar dq_lane, dq_bit;
        for (dq_lane = 0; dq_lane < BYTE_LANES; dq_lane = dq_lane + 1) begin : gen_dq_lane
            for (dq_bit = 0; dq_bit < DQ_BITS; dq_bit = dq_bit + 1) begin : gen_dq_bit
                localparam integer DQ_IDX = dq_lane * DQ_BITS + dq_bit;

                // DFI wrdata -> OSERDESE3 D mapping
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

                // DQ write path: OSERDESE3 -> ODELAYE3 -> IOBUF
                wire odelay_dq_out;
                (* IODELAY_GROUP = "ddr4_phy_iodelay" *)
                ODELAYE3 #(
                    .CASCADE("NONE"), .DELAY_FORMAT("TIME"),
                    .DELAY_TYPE("VAR_LOAD"), .DELAY_VALUE(DATA_INITIAL_ODELAY_TAP),
                    .IS_CLK_INVERTED(1'b0), .IS_RST_INVERTED(1'b0),
                    .REFCLK_FREQUENCY(300.0), .SIM_DEVICE("ULTRASCALE_PLUS"),
                    .UPDATE_MODE("ASYNC")
                ) odelay_dq (
                    .ODATAIN(oserdes_dq_out), .DATAOUT(odelay_dq_out),
                    .CLK(i_controller_clk), .RST(sync_rst),
                    .CE(1'b0), .INC(1'b0),
                    .LOAD(odelay_dq_load[dq_lane]),
                    .CNTVALUEIN(odelay_dq_cntvalue),
                    .CNTVALUEOUT(), .EN_VTC(en_vtc_q),
                    .CASC_IN(1'b0), .CASC_RETURN(1'b0), .CASC_OUT()
                );

                wire ibuf_dq_out;
                IOBUF dq_iobuf (
                    .I(odelay_dq_out), .O(ibuf_dq_out),
                    .IO(io_ddr4_dq[DQ_IDX]), .T(dq_tristate)
                );

                wire idelay_dq_out;
                // VAR_LOAD: training FSM loads tap via idelay_load_lane
                (* IODELAY_GROUP = "ddr4_phy_iodelay" *)
                IDELAYE3 #(
                    .CASCADE("NONE"), .DELAY_FORMAT("TIME"),
                    .DELAY_SRC("IDATAIN"), .DELAY_TYPE("VAR_LOAD"),
                    .DELAY_VALUE(DATA_INITIAL_IDELAY_TAP),
                    .IS_CLK_INVERTED(1'b0), .IS_RST_INVERTED(1'b0),
                    .REFCLK_FREQUENCY(300.0), .SIM_DEVICE("ULTRASCALE_PLUS"),
                    .UPDATE_MODE("ASYNC")
                ) idelay_dq (
                    .IDATAIN(ibuf_dq_out), .DATAOUT(idelay_dq_out),
                    .CLK(i_controller_clk), .RST(sync_rst),
                    .CE(1'b0), .INC(1'b0),
                    .LOAD(idelay_load_lane[dq_lane]),
                    .CNTVALUEIN(idelay_cntvalue),
                    .CNTVALUEOUT(),
                    .DATAIN(1'b0), .EN_VTC(en_vtc_q), .CASC_IN(1'b0),
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

    // -----------------------------------------------------------------
    // DQS Strobe Path (per byte lane)
    // Write: OSERDESE3(dqs_pattern) -> ODELAYE3 -> IOBUFDS -> DQS+/-
    // Read:  DQS+/- -> IOBUFDS -> IDELAYE3 -> ISERDESE3 (training only)
    // The DQS ISERDESE3 is only used during gate training and write
    // leveling feedback. Normal reads use the DQ ISERDESE3 outputs.
    // -----------------------------------------------------------------
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
                .RST(sync_rst), .T(dqs_tristate_wl)
            );

            wire odelay_dqs_out;
            (* IODELAY_GROUP = "ddr4_phy_iodelay" *)
            ODELAYE3 #(
                .CASCADE("NONE"), .DELAY_FORMAT("TIME"),
                .DELAY_TYPE("VAR_LOAD"), .DELAY_VALUE(DQS_ODELAY_PS),
                .IS_CLK_INVERTED(1'b0), .IS_RST_INVERTED(1'b0),
                .REFCLK_FREQUENCY(300.0), .SIM_DEVICE("ULTRASCALE_PLUS"),
                .UPDATE_MODE("ASYNC")
            ) odelay_dqs (
                .ODATAIN(oserdes_dqs_out), .DATAOUT(odelay_dqs_out),
                .CLK(i_controller_clk), .RST(sync_rst),
                .CE(1'b0), .INC(1'b0),
                .LOAD(odelay_dqs_load[dqs_lane]),
                .CNTVALUEIN(odelay_dqs_cntvalue),
                .CNTVALUEOUT(odelay_dqs_cntvalueout[dqs_lane]),
                .EN_VTC(en_vtc_q),
                .CASC_IN(1'b0), .CASC_RETURN(1'b0), .CASC_OUT()
            );

            wire ibuf_dqs_out;
            IOBUFDS #(
                .DQS_BIAS("TRUE")
            ) dqs_iobufds (
                .I(odelay_dqs_out), .O(ibuf_dqs_out),
                .IO(io_ddr4_dqs_p[dqs_lane]), .IOB(io_ddr4_dqs_n[dqs_lane]),
                .T(dqs_tristate_wl)
            );

            wire idelay_dqs_out;
            (* IODELAY_GROUP = "ddr4_phy_iodelay" *)
            IDELAYE3 #(
                .CASCADE("NONE"), .DELAY_FORMAT("TIME"),
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
                .DATAIN(1'b0), .EN_VTC(en_vtc_q), .CASC_IN(1'b0),
                .CASC_RETURN(1'b0), .CASC_OUT()
            );

            // DQS ISERDESE3 -- used during training (gate + WL feedback)
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

    // -----------------------------------------------------------------
    // DM_n Mask Path (per byte lane, x8/x16 only)
    // dfi_wrdata_mask (active-HIGH) inverted -> DM_n (active-LOW on DRAM)
    // x4 devices: DM_ENABLED=0, DM_n tied high (no mask pin)
    // -----------------------------------------------------------------
    generate
        if (DM_ENABLED) begin : gen_dm
            genvar dm_lane;
            for (dm_lane = 0; dm_lane < BYTE_LANES; dm_lane = dm_lane + 1) begin : gen_dm_lane
                // DFI mask -> DM_n OSERDESE3 D mapping (inverted for active-low)
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
                    .CASCADE("NONE"), .DELAY_FORMAT("TIME"),
                    .DELAY_TYPE("VAR_LOAD"), .DELAY_VALUE(DATA_INITIAL_ODELAY_TAP),
                    .IS_CLK_INVERTED(1'b0), .IS_RST_INVERTED(1'b0),
                    .REFCLK_FREQUENCY(300.0), .SIM_DEVICE("ULTRASCALE_PLUS"),
                    .UPDATE_MODE("ASYNC")
                ) odelay_dm (
                    .ODATAIN(oserdes_dm_out), .DATAOUT(odelay_dm_out),
                    .CLK(i_controller_clk), .RST(sync_rst),
                    .CE(1'b0), .INC(1'b0),
                    .LOAD(odelay_dq_load[dm_lane]),
                    .CNTVALUEIN(odelay_dq_cntvalue),
                    .CNTVALUEOUT(), .EN_VTC(en_vtc_q),
                    .CASC_IN(1'b0), .CASC_RETURN(1'b0), .CASC_OUT()
                );

                OBUF dm_obuf (.I(odelay_dm_out), .O(o_ddr4_dm_n[dm_lane]));
            end
        end else begin : gen_dm_stub
            assign o_ddr4_dm_n = {BYTE_LANES{1'b1}};
        end
    endgenerate

    // -----------------------------------------------------------------
    // Fabric Bitslip Barrel Shifter
    // ISERDESE3 has no BITSLIP pin (unlike ISERDESE2), so alignment is
    // done in fabric. We keep {prev_cycle, cur_cycle} = 16-bit window
    // per DQ bit and barrel-shift by the per-lane bitslip_count.
    // Gate training determines the correct bitslip_count for each lane.
    // Before training, bitslip_count=0 (no correction applied).
    // -----------------------------------------------------------------
    reg [7:0]  prev_iserdes_q [TOTAL_DQ-1:0];
    reg [2:0]  bitslip_count_q [BYTE_LANES-1:0];
    wire [7:0] aligned_dq [TOTAL_DQ-1:0];

    // PHY training FSM state registers
    reg [3:0] phy_state;
    reg [$clog2(BYTE_LANES > 1 ? BYTE_LANES : 2)-1:0] train_lane;
    reg [2:0] phy_timer;
    reg [3:0] bitslip_shift_count;

    // Eye training registers (IDELAYE3 sweep)
    reg [8:0] sweep_tap;
    reg [8:0] first_pass_tap [BYTE_LANES-1:0];
    reg [8:0] last_pass_tap  [BYTE_LANES-1:0];
    reg       eye_found      [BYTE_LANES-1:0];

    // Write leveling registers (ODELAYE3 DQS sweep)
    reg [8:0] wl_tap        [BYTE_LANES-1:0];
    reg [8:0] wl_dq_tap     [BYTE_LANES-1:0];
    reg       wl_prev_dq0   [BYTE_LANES-1:0];
    reg [8:0] dqs_initial_tap [BYTE_LANES-1:0];
    reg [7:0] vtc_settle_counter;

    assign wl_active = (phy_state == PHY_WL_SAMPLE) || (phy_state == PHY_WL_ADJUST)
                     || (phy_state == PHY_WL_CHECK)  || (phy_state == PHY_WL_DONE);

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
    // DFI Read Data Packing + rddata_valid
    // Pack aligned ISERDESE3 outputs into flat o_dfi_rddata vector.
    // rddata_valid follows rddata_en with 1-cycle capture latency.
    //
    // How packing works:
    //   For each DQ bit, the aligned_dq[idx] byte contains 8 beats.
    //   aligned_dq[idx][2*phase]     -> DFI rddata rise beat for that phase
    //   aligned_dq[idx][2*phase + 1] -> DFI rddata fall beat for that phase
    //   The flat DFI vector groups bits as:
    //     [phase*DFI_DATA_WIDTH + lane*DQ_BITS + bit] = rise beat
    //     [phase*DFI_DATA_WIDTH + BEAT_WIDTH + lane*DQ_BITS + bit] = fall beat
    // -----------------------------------------------------------------
    integer dfi_pack_lane, dfi_pack_bit, dfi_pack_phase, dfi_pack_idx;

    always @(posedge i_controller_clk) begin
        if (!ctrl_rst_n) begin
            o_dfi_rddata       <= {(4*DFI_DATA_WIDTH){1'b0}};
            o_dfi_rddata_valid <= 4'b0;
            o_dfi_rdlvl_resp   <= {BYTE_LANES{1'b0}};
            o_dfi_wrlvl_resp   <= {BYTE_LANES{1'b0}};
            for (dfi_pack_idx = 0; dfi_pack_idx < TOTAL_DQ; dfi_pack_idx = dfi_pack_idx + 1)
                prev_iserdes_q[dfi_pack_idx] <= 8'b0;
            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                bitslip_count_q[dfi_pack_idx] <= 3'b0;
                idelay_load_lane[dfi_pack_idx] <= 1'b0;
                first_pass_tap[dfi_pack_idx]   <= 9'b0;
                last_pass_tap[dfi_pack_idx]    <= 9'b0;
                eye_found[dfi_pack_idx]        <= 1'b0;
                odelay_dqs_load[dfi_pack_idx]  <= 1'b0;
                odelay_dq_load[dfi_pack_idx]   <= 1'b0;
                wl_tap[dfi_pack_idx]           <= 9'b0;
                wl_dq_tap[dfi_pack_idx]        <= 9'b0;
                wl_prev_dq0[dfi_pack_idx]      <= 1'b0;
                dqs_initial_tap[dfi_pack_idx]  <= 9'b0;
            end
            phy_state           <= PHY_IDLE;
            train_lane          <= 0;
            phy_timer           <= 3'b0;
            bitslip_shift_count <= 4'b0;
            idelay_cntvalue     <= 9'b0;
            sweep_tap           <= 9'b0;
            odelay_dqs_cntvalue <= 9'b0;
            odelay_dq_cntvalue  <= 9'b0;
            wl_dqs_strobe       <= 1'b0;
            wl_dqs_strobe_d1    <= 1'b0;
            en_vtc_q            <= 1'b1;
            vtc_settle_counter  <= 8'b0;
        end else begin
            // Default: deassert all LOAD pulses (single-cycle pulse)
            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                idelay_load_lane[dfi_pack_idx] <= 1'b0;
                odelay_dqs_load[dfi_pack_idx]  <= 1'b0;
                odelay_dq_load[dfi_pack_idx]   <= 1'b0;
            end
            wl_dqs_strobe <= 1'b0;
            wl_dqs_strobe_d1 <= wl_dqs_strobe;

            // Update previous ISERDESE3 outputs for bitslip window
            for (dfi_pack_idx = 0; dfi_pack_idx < TOTAL_DQ; dfi_pack_idx = dfi_pack_idx + 1)
                prev_iserdes_q[dfi_pack_idx] <= iserdes_dq_q[dfi_pack_idx];

            // Pack aligned read data into DFI format.
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

            // ---------------------------------------------------------
            // PHY Training FSM
            // Gate training: bitslip alignment using MPR page 0.
            // Eye training: IDELAYE3 DQ tap sweep, find eye, center.
            // Write leveling: ODELAYE3 DQS tap sweep, find 0->1 on DQ[0].
            // ---------------------------------------------------------
            begin
                case (phy_state)
                    PHY_IDLE: begin
                        if (i_dfi_rdlvl_gate_en) begin
                            en_vtc_q <= 1'b0;
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}};
                            train_lane <= 0;
                            bitslip_shift_count <= 4'd0;
                            phy_state <= PHY_GATE_BITSLIP;
                        end else if (i_dfi_rdlvl_en) begin
                            en_vtc_q <= 1'b0;
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}};
                            train_lane <= 0;
                            sweep_tap <= 9'd0;
                            idelay_cntvalue <= 9'd0;
                            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES;
                                 dfi_pack_idx = dfi_pack_idx + 1) begin
                                eye_found[dfi_pack_idx] <= 1'b0;
                                first_pass_tap[dfi_pack_idx] <= 9'd0;
                                last_pass_tap[dfi_pack_idx] <= 9'd0;
                            end
                            phy_timer <= 3'd4;
                            phy_state <= PHY_EYE_SWEEP;
                        end else if (i_dfi_wrlvl_en) begin
                            en_vtc_q <= 1'b0;
                            o_dfi_wrlvl_resp <= {BYTE_LANES{1'b0}};
                            train_lane <= 0;
                            odelay_dqs_cntvalue <= odelay_dqs_cntvalueout[0];
                            odelay_dq_cntvalue  <= 9'd0;
                            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES;
                                 dfi_pack_idx = dfi_pack_idx + 1) begin
                                dqs_initial_tap[dfi_pack_idx] <= odelay_dqs_cntvalueout[dfi_pack_idx];
                                wl_tap[dfi_pack_idx]    <= odelay_dqs_cntvalueout[dfi_pack_idx];
                                wl_dq_tap[dfi_pack_idx] <= 9'd0;
                                wl_prev_dq0[dfi_pack_idx] <= 1'b0;
                            end
                            phy_timer <= 3'd4;
                            phy_state <= PHY_WL_SAMPLE;
                        end
                    end

                    // -- Gate training (bitslip alignment) -------------
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

                    // -- Eye training (IDELAYE3 DQ sweep) -------------
                    PHY_EYE_SWEEP: begin
                        if (phy_timer != 0) begin
                            if (phy_timer == 3'd3)
                                idelay_load_lane[train_lane] <= 1'b1;
                            phy_timer <= phy_timer - 1'b1;
                        end else if (|i_dfi_rddata_en) begin
                            if (aligned_dq[train_lane * DQ_BITS] == MPR_PATTERN) begin
                                // Data matches at this tap
                                if (!eye_found[train_lane]) begin
                                    first_pass_tap[train_lane] <= sweep_tap;
                                    eye_found[train_lane] <= 1'b1;
                                end
                                last_pass_tap[train_lane] <= sweep_tap;
                                // Advance to next tap or finish if at 511
                                if (sweep_tap >= 9'd508) begin
                                    // At or past last valid step -- eye stays open to end
                                    phy_state <= PHY_EYE_CENTER;
                                end else begin
                                    sweep_tap <= sweep_tap + {5'b0, TAP_SWEEP_STEP};
                                    idelay_cntvalue <= sweep_tap + {5'b0, TAP_SWEEP_STEP};
                                    phy_timer <= 3'd4;
                                end
                            end else begin
                                // Data mismatch
                                if (eye_found[train_lane]) begin
                                    // Eye has closed -- we have both boundaries
                                    phy_state <= PHY_EYE_CENTER;
                                end else begin
                                    // Haven't found eye yet -- keep sweeping
                                    if (sweep_tap >= 9'd508) begin
                                        // Exhausted all taps without finding eye
                                        `ifndef YOSYS
                                        $display("[%0t] PHY eye: lane %0d no eye found",
                                            $realtime, train_lane);
                                        `endif
                                        phy_state <= PHY_EYE_CENTER;
                                    end else begin
                                        sweep_tap <= sweep_tap + {5'b0, TAP_SWEEP_STEP};
                                        idelay_cntvalue <= sweep_tap + {5'b0, TAP_SWEEP_STEP};
                                        phy_timer <= 3'd4;
                                    end
                                end
                            end
                        end
                    end

                    PHY_EYE_CENTER: begin
                        idelay_cntvalue <=
                            ({1'b0, first_pass_tap[train_lane]} + {1'b0, last_pass_tap[train_lane]}) >> 1;
                        phy_timer <= 3'd4;
                        phy_state <= PHY_EYE_VERIFY;
                        `ifndef YOSYS
                        $display("[%0t] PHY eye: lane %0d first=%0d last=%0d center=%0d",
                            $realtime, train_lane,
                            first_pass_tap[train_lane], last_pass_tap[train_lane],
                            ({1'b0, first_pass_tap[train_lane]} + {1'b0, last_pass_tap[train_lane]}) >> 1);
                        `endif
                    end

                    PHY_EYE_VERIFY: begin
                        if (phy_timer != 0) begin
                            if (phy_timer == 3'd3)
                                idelay_load_lane[train_lane] <= 1'b1;
                            phy_timer <= phy_timer - 1'b1;
                        end else if (|i_dfi_rddata_en) begin
                            if (aligned_dq[train_lane * DQ_BITS] == MPR_PATTERN) begin
                                // Verified -- advance to next lane or finish
                                if (train_lane < BYTE_LANES - 1) begin
                                    train_lane <= train_lane + 1'b1;
                                    sweep_tap <= 9'd0;
                                    idelay_cntvalue <= 9'd0;
                                    eye_found[train_lane + 1'b1] <= 1'b0;
                                    phy_timer <= 3'd4;
                                    phy_state <= PHY_EYE_SWEEP;
                                end else begin
                                    phy_state <= PHY_EYE_DONE;
                                    `ifndef YOSYS
                                    for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES;
                                         dfi_pack_idx = dfi_pack_idx + 1)
                                        $display("[%0t] PHY eye done: lane %0d tap=%0d",
                                            $realtime, dfi_pack_idx,
                                            (first_pass_tap[dfi_pack_idx]
                                             + last_pass_tap[dfi_pack_idx]) >> 1);
                                    `endif
                                end
                            end else begin
                                // Verification failed -- flag and proceed
                                `ifndef YOSYS
                                $display("[%0t] PHY eye: lane %0d verify FAILED at center tap",
                                    $realtime, train_lane);
                                `endif
                                if (train_lane < BYTE_LANES - 1) begin
                                    train_lane <= train_lane + 1'b1;
                                    sweep_tap <= 9'd0;
                                    idelay_cntvalue <= 9'd0;
                                    eye_found[train_lane + 1'b1] <= 1'b0;
                                    phy_timer <= 3'd4;
                                    phy_state <= PHY_EYE_SWEEP;
                                end else begin
                                    phy_state <= PHY_EYE_DONE;
                                end
                            end
                        end
                    end

                    PHY_EYE_DONE: begin
                        o_dfi_rdlvl_resp <= {BYTE_LANES{1'b1}};
                        if (!i_dfi_rdlvl_en)
                            phy_state <= PHY_IDLE;
                    end

                    // -- Write leveling (ODELAYE3 DQS sweep) ----------
                    PHY_WL_SAMPLE: begin
                        if (phy_timer != 0) begin
                            if (phy_timer == 3'd3) begin
                                odelay_dqs_load[train_lane] <= 1'b1;
                                odelay_dq_load[train_lane]  <= 1'b1;
                            end
                            phy_timer <= phy_timer - 1'b1;
                        end else if (i_dfi_wrlvl_strobe) begin
                            wl_dqs_strobe <= 1'b1;
                            phy_timer <= 3'd4;
                            phy_state <= PHY_WL_ADJUST;
                        end
                    end

                    PHY_WL_ADJUST: begin
                        if (phy_timer != 0)
                            phy_timer <= phy_timer - 1'b1;
                        else begin
                            // Write leveling edge detection (JESD79-4D ch.4.18):
                            // prev initialized to 0; detect 0->1 CK crossing.
                            // Lockstep: both DQ and DQS taps incremented
                            // together to preserve the 90 deg offset.
                            `ifndef YOSYS
                            $display("[%0t] PHY WL sweep: lane %0d dqs_tap=%0d dq_tap=%0d DQ=%0b prev=%0b",
                                $realtime, train_lane, wl_tap[train_lane],
                                wl_dq_tap[train_lane],
                                |iserdes_dq_q[train_lane * DQ_BITS],
                                wl_prev_dq0[train_lane]);
                            `endif
                            if (!wl_prev_dq0[train_lane]
                                && |iserdes_dq_q[train_lane * DQ_BITS]) begin
                                phy_state <= PHY_WL_CHECK;
                            end else begin
                                wl_prev_dq0[train_lane] <=
                                    |iserdes_dq_q[train_lane * DQ_BITS];
                                if (wl_tap[train_lane] >= 9'd508) begin
                                    `ifndef YOSYS
                                    $display("[%0t] PHY WL: lane %0d exhausted taps",
                                        $realtime, train_lane);
                                    `endif
                                    phy_state <= PHY_WL_CHECK;
                                end else begin
                                    wl_tap[train_lane] <= wl_tap[train_lane]
                                        + {5'b0, WL_TAP_STEP};
                                    wl_dq_tap[train_lane] <= wl_dq_tap[train_lane]
                                        + {5'b0, WL_TAP_STEP};
                                    odelay_dqs_cntvalue <= wl_tap[train_lane]
                                        + {5'b0, WL_TAP_STEP};
                                    odelay_dq_cntvalue <= wl_dq_tap[train_lane]
                                        + {5'b0, WL_TAP_STEP};
                                    phy_timer <= 3'd4;
                                    phy_state <= PHY_WL_SAMPLE;
                                end
                            end
                        end
                    end

                    PHY_WL_CHECK: begin
                        `ifndef YOSYS
                        $display("[%0t] PHY WL: lane %0d dqs_tap=%0d dq_tap=%0d",
                            $realtime, train_lane, wl_tap[train_lane],
                            wl_dq_tap[train_lane]);
                        `endif
                        if (train_lane < BYTE_LANES - 1) begin
                            train_lane <= train_lane + 1'b1;
                            wl_tap[train_lane + 1'b1]    <= dqs_initial_tap[train_lane + 1'b1];
                            wl_dq_tap[train_lane + 1'b1] <= 9'd0;
                            wl_prev_dq0[train_lane + 1'b1] <= 1'b0;
                            odelay_dqs_cntvalue <= dqs_initial_tap[train_lane + 1'b1];
                            odelay_dq_cntvalue  <= 9'd0;
                            phy_timer <= 3'd4;
                            phy_state <= PHY_WL_SAMPLE;
                        end else begin
                            en_vtc_q <= 1'b1;
                            vtc_settle_counter <= VTC_SETTLE_CYCLES;
                            phy_state <= PHY_WL_DONE;
                            `ifndef YOSYS
                            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES;
                                 dfi_pack_idx = dfi_pack_idx + 1)
                                $display("[%0t] PHY WL done: lane %0d dqs_tap=%0d dq_tap=%0d",
                                    $realtime, dfi_pack_idx,
                                    wl_tap[dfi_pack_idx],
                                    wl_dq_tap[dfi_pack_idx]);
                            `endif
                        end
                    end

                    PHY_WL_DONE: begin
                        if (vtc_settle_counter != 0)
                            vtc_settle_counter <= vtc_settle_counter - 1'b1;
                        else begin
                            o_dfi_wrlvl_resp <= {BYTE_LANES{1'b1}};
                            if (!i_dfi_wrlvl_en)
                                phy_state <= PHY_IDLE;
                        end
                    end

                    default: ;
                endcase
            end
        end
    end

    // -----------------------------------------------------------------
    // IDELAYCTRL
    // Required for IDELAYE3/ODELAYE3 in TIME mode (UG571).
    // Reset released after SERDES primitives per UG571 ch.7.6.
    // Once RDY asserts, the delay taps are calibrated and the PHY
    // signals dfi_init_complete to the memory controller.
    // -----------------------------------------------------------------
    (* IODELAY_GROUP = "ddr4_phy_iodelay" *)
    IDELAYCTRL idelayctrl_inst (
        .REFCLK(i_ref_clk),
        .RST(idelayctrl_rst),
        .RDY(idelayctrl_rdy_w)
    );

    // -----------------------------------------------------------------
    // Debug Status Assigns
    // Expose training results for ILA / chipscope probing.
    // -----------------------------------------------------------------
    assign o_phy_state = phy_state;
    generate
        genvar dbg_lane;
        for (dbg_lane = 0; dbg_lane < BYTE_LANES; dbg_lane = dbg_lane + 1) begin : gen_dbg
            assign o_phy_idelay_center[dbg_lane*9 +: 9] =
                (first_pass_tap[dbg_lane] + last_pass_tap[dbg_lane]) >> 1;
            assign o_phy_wl_tap[dbg_lane*9 +: 9] = wl_tap[dbg_lane];
            assign o_phy_bitslip[dbg_lane*3 +: 3] = bitslip_count_q[dbg_lane];
        end
    endgenerate

endmodule
