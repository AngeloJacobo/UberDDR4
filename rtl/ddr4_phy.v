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
              BG_BITS = (DEVICE_WIDTH == 16) ? 1 : 2, //JESD79-4D Table 4
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
    // DFI 3.1 Control (SERDES_RATIO phases, packed flat)
    input wire [SERDES_RATIO*17-1:0]        i_dfi_address,
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
    input wire                              i_dfi_init_start,
    output wire                             o_dfi_init_complete,
    // DFI Training (MC -> PHY)
    input wire                              i_dfi_rdlvl_en,
    input wire                              i_dfi_rdlvl_gate_en,
    input wire                              i_dfi_wrlvl_en,
    input wire                              i_dfi_wrlvl_strobe,
    input wire [SERDES_RATIO-1:0]           i_dfi_lvl_pattern,
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
    output wire [3*BYTE_LANES-1:0]          o_phy_bitslip,
    output wire [BYTE_LANES-1:0]            o_phy_train_fail_gate,
    output wire [BYTE_LANES-1:0]            o_phy_train_fail_eye,
    output wire [BYTE_LANES-1:0]            o_phy_train_fail_wl
);



    // -----------------------------------------------------------------
    // ODELAYE3/IDELAYE3 delay configuration (all in TIME mode, ps units)
    // DQS ODELAYE3 adds DDR4_CLK_PERIOD/4 ps = 90° phase shift so DQS
    // edges are centered in the DQ data eye at the DRAM receiver.
    // IODELAY BISC (Built-In Self-Calibration) converts the ps value to taps
    // automatically at power-up (UG571, DELAY_FORMAT=TIME section).
    // Write leveling reads CNTVALUEOUT to get the IODELAY BISC-calibrated
    // starting tap before sweeping additional delay.
    // -----------------------------------------------------------------
    localparam integer DATA_INITIAL_ODELAY_TAP = 0;
    localparam integer DATA_INITIAL_IDELAY_TAP = 0;
    localparam integer DQS_ODELAY_PS = DDR4_CLK_PERIOD / 4;
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

    // MPR page 0, MPR2 register value = 8'h0F = 00001111 (JESD79-4D Table 56).
    // Serial readout sends bit[7] first: UI0=0, UI1=0, UI2=0, UI3=0,
    // UI4=1, UI5=1, UI6=1, UI7=1.
    // ISERDESE3 8:1 DDR captures Q[0]=first bit received (UG571 Table 2-5):
    //   Q[0]=UI0=0, Q[1]=UI1=0, ..., Q[4]=UI4=1, ..., Q[7]=UI7=1
    //   -> Q[7:0] = 8'b11110000
    // Period = 8 UI: uniquely identifies all 8 possible bitslip values.
    localparam [7:0] MPR_PATTERN = 8'b11110000;

    // Eye training: sweep IDELAYE3 from tap 0 to 508 in steps of 4 (~128 iterations).
    // Stops at 508 (not 511) to avoid 9-bit overflow on the +4 addition.
    localparam [3:0] TAP_SWEEP_STEP = 4'd4;

    // Write leveling: sweep ODELAYE3 DQS in steps of 4
    localparam [3:0] WL_TAP_STEP = 4'd4;

    // VTC settle: guard time after EN_VTC goes HIGH before normal operation.
    // Per UG571 VAR_LOAD procedure step 8: "Set EN_VTC High for VT compensation"
    // then wait before resuming. 200 cycles is conservative for IODELAY BISC re-lock.
    localparam [7:0] VTC_SETTLE_CYCLES = 8'd200; // UG571 has no exact count; MIG uses 10-16. 200 is safe (one-shot per training).

    // -----------------------------------------------------------------
    // DFI Data Layout
    //
    // The DFI interface carries 4 phases × 2 edges (rise+fall) of data
    // per controller clock. Each edge transfers TOTAL_DQ bits in parallel
    // -----------------------------------------------------------------
    localparam TOTAL_DQ      = DQ_BITS * BYTE_LANES;  // DQ bits per clock edge (= half of DFI_DATA_WIDTH)
    localparam DM_PER_PHASE  = 2 * BYTE_LANES;       // DM bits per DFI phase: 1 per lane × 2 edges
    localparam DM_ENABLED    = (DEVICE_WIDTH != 4);   // x4 has no DM pin (JESD79-4D Table 28)

    // -----------------------------------------------------------------
    // Reset Generation (UG571 "Component Mode Reset Sequence", p.188-189)
    //
    // All primitive RST ports (OSERDESE3, ISERDESE3, IDELAYE3, ODELAYE3,
    // IDELAYCTRL) are ASYNCHRONOUS — they don't require any specific
    // clock domain. We generate all resets from i_controller_clk using
    // a simple counter, which satisfies:
    //
    //   UG571 Release Reset sequence p.189:
    //     Step 2c: Release IDELAY/ODELAY/ISERDES/OSERDES resets
    //     Step 2d: AFTER step 2c, release IDELAYCTRL reset
    //     Step 2e: Wait for IDELAYCTRL.RDY
    //
    //   Timing: IODELAY minimum reset pulse = 52ns (DS931 Table 34,
    //   T_MINPER_RST). IODELAY_RST_DELAY holds sync_rst long enough
    //   to guarantee >52ns. IDELAYCTRL_RST_EXTRA adds cycles after
    //   sync_rst deasserts before releasing IDELAYCTRL (ordering).
    //
    // -----------------------------------------------------------------
    localparam IODELAY_RST_DELAY = (52_000 / CONTROLLER_CLK_PERIOD) + 2;
    localparam IDELAYCTRL_RST_EXTRA = 4;

    reg [$clog2(IODELAY_RST_DELAY + IDELAYCTRL_RST_EXTRA + 1):0] rst_cnt;
    reg sync_rst;
    reg idelayctrl_rst;


    always @(posedge i_controller_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            rst_cnt       <= 0;
            sync_rst      <= 1'b1;
            idelayctrl_rst <= 1'b1;
        end else begin
            if (!(&rst_cnt)) begin // count up until max (saturating)
                rst_cnt <= rst_cnt + 1;
            end

            // Step 2c: release SERDES/IDELAY/ODELAY reset after IODELAY_RST_DELAY
            if (rst_cnt == IODELAY_RST_DELAY) begin 
                sync_rst <= 1'b0;
            end

            // Step 2d: release IDELAYCTRL reset AFTER sync_rst (extra margin)
            if (rst_cnt == IODELAY_RST_DELAY + IDELAYCTRL_RST_EXTRA) begin
                idelayctrl_rst <= 1'b0;
            end
        end
    end

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
    // UG571 Table 2-8: TIP: The data applied to SerDes input D0 is the 
    // first bit to be transmitted in all cases.
    // Each DFI phase maps to one DDR4 command slot. Bits are doubled
    // because OSERDESE3 DATA_WIDTH=8 in DDR mode gives 4 edges, but
    // CA pins are SDR (active on rising edge only). Doubling each bit
    // ensures the same value appears on both the rising and falling
    // edge of each UI, so the DRAM sees a clean SDR command.
    // -----------------------------------------------------------------

    // -----------------------------------------------------------------
    // Command/Address Output Path
    //
    // Each DDR4 CA pin gets one OSERDESE3 (8:1 SDR, doubled for DDR clock).
    // DFI provides 4 phases per controller clock. Each phase value is
    // repeated on rise+fall edges (SDR command bus), giving 8 bits to OSERDES:
    //   D[7:0] = {phase3, phase3, phase2, phase2, phase1, phase1, phase0, phase0}
    //
    // DDR4 address pin mapping (JESD79-4D Table 35):
    //   A[13:0]  = row/column address from dfi_address
    //   A14      = WE_n  (when ACT_n=1) or row addr bit (when ACT_n=0)
    //   A15      = CAS_n (when ACT_n=1) or row addr bit (when ACT_n=0)
    //   A16      = RAS_n (when ACT_n=1) or row addr bit (when ACT_n=0)
    // Per DFI 3.1, dfi_ras_n/cas_n/we_n always carry the correct value
    // for pins A[16:14] regardless of ACT_n. dfi_address[16:14] is unused.
    // -----------------------------------------------------------------

    // Address pins A[16:0]
    generate
        genvar abit;
        for (abit = 0; abit < 17; abit = abit + 1) begin : gen_addr
            // A[13:0] from dfi_address, A[16:14] from ras_n/cas_n/we_n
            wire [3:0] addr_phases;
            if (abit < 14) begin : lo_addr
                assign addr_phases = {i_dfi_address[17*3 + abit],   // phase 3
                                      i_dfi_address[17*2 + abit],   // phase 2
                                      i_dfi_address[17*1 + abit],   // phase 1
                                      i_dfi_address[17*0 + abit]};  // phase 0
            end else if (abit == 14) begin : a14_we
                assign addr_phases = i_dfi_we_n;
            end else if (abit == 15) begin : a15_cas
                assign addr_phases = i_dfi_cas_n;
            end else begin : a16_ras
                assign addr_phases = i_dfi_ras_n;
            end

            wire addr_oserdes_out;
            OSERDESE3 #(
                .DATA_WIDTH(8), .INIT(1'b0),
                .IS_CLKDIV_INVERTED(1'b0), .IS_CLK_INVERTED(1'b0),
                .IS_RST_INVERTED(1'b0), .SIM_DEVICE("ULTRASCALE_PLUS")
            ) oserdes_addr (
                .D({addr_phases[3], addr_phases[3],
                    addr_phases[2], addr_phases[2],
                    addr_phases[1], addr_phases[1],
                    addr_phases[0], addr_phases[0]}),
                .OQ(addr_oserdes_out), .T_OUT(),
                .CLK(i_ddr4_clk), .CLKDIV(i_controller_clk),
                .RST(sync_rst), .T(1'b0)
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
                .DATA_WIDTH(8), .INIT(1'b0),
                .IS_CLKDIV_INVERTED(1'b0), .IS_CLK_INVERTED(1'b0),
                .IS_RST_INVERTED(1'b0), .SIM_DEVICE("ULTRASCALE_PLUS")
            ) oserdes_ba (
                .D({i_dfi_bank[BA_BITS*3 + babit], i_dfi_bank[BA_BITS*3 + babit],   // phase 3
                    i_dfi_bank[BA_BITS*2 + babit], i_dfi_bank[BA_BITS*2 + babit],   // phase 2
                    i_dfi_bank[BA_BITS*1 + babit], i_dfi_bank[BA_BITS*1 + babit],   // phase 1
                    i_dfi_bank[BA_BITS*0 + babit], i_dfi_bank[BA_BITS*0 + babit]}), // phase 0
                .OQ(ba_oserdes_out), .T_OUT(),
                .CLK(i_ddr4_clk), .CLKDIV(i_controller_clk),
                .RST(sync_rst), .T(1'b0)
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
                .DATA_WIDTH(8), .INIT(1'b0),
                .IS_CLKDIV_INVERTED(1'b0), .IS_CLK_INVERTED(1'b0),
                .IS_RST_INVERTED(1'b0), .SIM_DEVICE("ULTRASCALE_PLUS")
            ) oserdes_bg (
                .D({i_dfi_bg[BG_BITS*3 + bgbit], i_dfi_bg[BG_BITS*3 + bgbit],   // phase 3
                    i_dfi_bg[BG_BITS*2 + bgbit], i_dfi_bg[BG_BITS*2 + bgbit],   // phase 2
                    i_dfi_bg[BG_BITS*1 + bgbit], i_dfi_bg[BG_BITS*1 + bgbit],   // phase 1
                    i_dfi_bg[BG_BITS*0 + bgbit], i_dfi_bg[BG_BITS*0 + bgbit]}), // phase 0
                .OQ(bg_oserdes_out), .T_OUT(),
                .CLK(i_ddr4_clk), .CLKDIV(i_controller_clk),
                .RST(sync_rst), .T(1'b0)
            );
            OBUF bg_buf (.I(bg_oserdes_out), .O(o_ddr4_bg[bgbit]));
        end
    endgenerate

    // Control pins: CS_n, ACT_n, CKE, ODT, RESET_n
    // Each is a single-bit signal with 4 DFI phases → one OSERDES each.
    generate
        genvar cpin;
        for (cpin = 0; cpin < 5; cpin = cpin + 1) begin : gen_ctrl
            wire [3:0] ctrl_phases = (cpin == 0) ? i_dfi_cs_n :
                                     (cpin == 1) ? i_dfi_act_n :
                                     (cpin == 2) ? i_dfi_cke :
                                     (cpin == 3) ? i_dfi_odt :
                                                   i_dfi_reset_n;
            wire ctrl_oserdes_out;
            OSERDESE3 #(
                .DATA_WIDTH(8),
                .INIT((cpin == 0 || cpin == 1) ? 1'b1 : 1'b0), // CS_n, ACT_n idle high
                .IS_CLKDIV_INVERTED(1'b0), .IS_CLK_INVERTED(1'b0),
                .IS_RST_INVERTED(1'b0), .SIM_DEVICE("ULTRASCALE_PLUS")
            ) oserdes_ctrl (
                .D({ctrl_phases[3], ctrl_phases[3],     // phase 3  
                    ctrl_phases[2], ctrl_phases[2],     // phase 2
                    ctrl_phases[1], ctrl_phases[1],     // phase 1
                    ctrl_phases[0], ctrl_phases[0]}),   // phase 0
                .OQ(ctrl_oserdes_out), .T_OUT(),
                .CLK(i_ddr4_clk), .CLKDIV(i_controller_clk),
                .RST(sync_rst), .T(1'b0)
            );

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
    // Write Tri-State Control
    //
    // PURPOSE: DQ/DQS are bidirectional pins. They must be high-Z when
    // not writing, otherwise the PHY and DRAM would fight on reads.
    // OSERDESE3 T pin controls this: T=1 → high-Z (off), T=0 → driven.
    //
    // HOW IT WORKS:
    //
    // 1) wrdata_en_any = OR of all 4 DFI phase enables → collapses to
    //    a single "is there a write THIS controller clock?" flag.
    //    (One controller clock already covers all 4 DDR phases.)
    //
    // 2) A 2-stage shift register delays the flag by 1 and 2 controller
    //    clocks (each = 4 DDR clocks). This keeps the bus driven for
    //    2 extra controller clocks (8 DDR clocks) after wrdata_en drops,
    //    covering the DDR4 postamble (tWPST = 0.5 tCK) plus OSERDES +
    //    ODELAYE3 pipeline flush time.
    //
    // 3) output_enable = wrdata_en_any | shift[0] | shift[1]
    //    Both DQ and DQS use the same enable window.
    //
    // 4) Inversion: ~enable → tristate, because T=1 means OFF in OSERDESE3.
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
    wire dq_tristate   = ~output_enable;

    // -----------------------------------------------------------------
    // DQS Pattern Generation
    //
    // The OSERDESE3 for DQS gets an 8-bit pattern (4 phases × rise/fall):
    //   Normal write: 01_01_01_01 → continuous toggle, edges centered on DQ
    //   Idle:         00_00_00_00 → DQS held low (pin is tri-stated anyway)
    //
    // Write Leveling (WL): DRAM calibration mode where the controller
    // sends a single DQS rising edge and reads back DQ to find the
    // optimal clock-to-DQS alignment (JEDEC DDR4 §4.7.2).
    //   WL strobe:    00_00_00_01 → one rising edge only
    //   WL idle:      00_00_00_00 → hold low between strobes
    // -----------------------------------------------------------------
    wire wl_active;

    reg [7:0] dqs_pattern;
    reg       wl_dqs_strobe;
    always @* begin
        if (wl_active) begin
            if (wl_dqs_strobe)
                dqs_pattern = 8'b00_00_00_01; // single-edge DQS for WL
            else
                dqs_pattern = 8'b00_00_00_00; // hold low between WL strobes
        end else if (wrdata_en_any) begin
            dqs_pattern = 8'b01_01_01_01; // normal write: toggle every UI
        end else begin
            dqs_pattern = 8'b00_00_00_00; // idle: pin is tri-stated
        end
    end

    // WL tri-state: drive DQS only during the strobe pulse + 1 cycle
    // after (wl_dqs_strobe_d1) to complete the rising edge on the wire.
    reg  wl_dqs_strobe_d1;
    wire wl_dqs_drive = wl_dqs_strobe | wl_dqs_strobe_d1;
    wire dqs_tristate_wl = wl_active ? ~wl_dqs_drive : ~output_enable;

    // EN_VTC: LOW during training so IDELAYE3/ODELAYE3 tap values can be
    // loaded without the IDELAYCTRL overwriting them. HIGH in normal operation
    // so the IDELAYCTRL continuously compensates delay for PVT drift (UG571).
    reg en_vtc_q;

    // -----------------------------------------------------------------
    // DQ Data Path (per bit, per byte lane)
    // Write: OSERDESE3(8:1 DDR) -> ODELAYE3 -> IOBUF -> DQ pad
    // Read:  DQ pad -> IOBUF -> IDELAYE3 -> ISERDESE3(1:8 DDR)
    //
    // ISERDESE3 Q[7:0] mapping (8:1 DDR deserialize, UG571 Table 2-5):
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

                // DFI wrdata -> OSERDESE3 D[7:0] mapping
                // D[0] is transmitted first (UG571 Table 2-8).
                // Concatenation order (MSB..LSB): p3_fall, p3_rise, ..., p0_fall, p0_rise
                wire [7:0] dq_wr_d = {
                    i_dfi_wrdata[3*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 3 fall
                    i_dfi_wrdata[3*DFI_DATA_WIDTH + DQ_IDX],            // phase 3 rise
                    i_dfi_wrdata[2*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 2 fall
                    i_dfi_wrdata[2*DFI_DATA_WIDTH + DQ_IDX],            // phase 2 rise
                    i_dfi_wrdata[1*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 1 fall
                    i_dfi_wrdata[1*DFI_DATA_WIDTH + DQ_IDX],            // phase 1 rise
                    i_dfi_wrdata[0*DFI_DATA_WIDTH + TOTAL_DQ + DQ_IDX], // phase 0 fall
                    i_dfi_wrdata[0*DFI_DATA_WIDTH + DQ_IDX]             // phase 0 rise
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
    //        ODELAYE3 adds ~90° (DDR4_CLK_PERIOD/4 ps) so DQS edges
    //        are center-aligned with DQ data at the DRAM receiver.
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

            // DQS_BIAS="TRUE" (UG571 p.63): weak keeper holds the floating
            // differential pair to a known state between bursts. Without it,
            // noise on undriven DQS causes false edges at ISERDESE3.
            // Does NOT affect normal operation — active drivers easily
            // overdrive the weak pull. Supported for DIFF_POD (DDR4).
            IOBUFDS #(
                .DQS_BIAS("TRUE")
            ) dqs_iobufds (
                .I(odelay_dqs_out), .O(),
                .IO(io_ddr4_dqs_p[dqs_lane]), .IOB(io_ddr4_dqs_n[dqs_lane]),
                .T(dqs_tristate_wl)
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
                    ~i_dfi_wrdata_mask[3*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 3 fall
                    ~i_dfi_wrdata_mask[3*DM_PER_PHASE + dm_lane],              // phase 3 rise
                    ~i_dfi_wrdata_mask[2*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 2 fall
                    ~i_dfi_wrdata_mask[2*DM_PER_PHASE + dm_lane],              // phase 2 rise
                    ~i_dfi_wrdata_mask[1*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 1 fall
                    ~i_dfi_wrdata_mask[1*DM_PER_PHASE + dm_lane],              // phase 1 rise
                    ~i_dfi_wrdata_mask[0*DM_PER_PHASE + BYTE_LANES + dm_lane], // phase 0 fall
                    ~i_dfi_wrdata_mask[0*DM_PER_PHASE + dm_lane]               // phase 0 rise
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
    // ISERDESE3 has no BITSLIP pin (UG571 lists this as removed vs.
    // ISERDESE2), so word alignment is done in fabric logic.
    // Method: concatenate {current_Q[7:0], previous_Q[7:0]} into a
    // 16-bit window and barrel-shift by the per-lane bitslip_count
    // (0-7). This effectively selects the correct 8-bit word boundary.
    // Gate training (MPR pattern match) determines bitslip_count.
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
    reg [1:0] coarse_tap_idx;

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

    // Training failure latch registers (sticky — cleared on training start,
    // set on failure, visible on prober or in waveforms for post-mortem debug)
    reg [BYTE_LANES-1:0] gate_train_fail;
    reg [BYTE_LANES-1:0] eye_train_fail;
    reg [BYTE_LANES-1:0] wl_train_fail;

    assign wl_active = (phy_state == PHY_WL_SAMPLE) || (phy_state == PHY_WL_ADJUST)
                     || (phy_state == PHY_WL_CHECK)  || (phy_state == PHY_WL_DONE);

    // -----------------------------------------------------------------
    // Bitslip Alignment (barrel-shift across two ISERDESE3 captures)
    // -----------------------------------------------------------------
    // Problem: ISERDESE3 captures 8 serial bits per CLKDIV cycle, but
    // the byte boundary is unknown — the first captured bit may not be
    // the first transmitted bit. We need to "slip" (rotate) the 8-bit
    // window to align it with the DRAM's burst boundary.
    //
    // Solution: concatenate the CURRENT capture (iserdes_dq_q, bits
    // from this cycle) with the PREVIOUS capture (prev_iserdes_q, bits
    // from last cycle) into a 16-bit sliding window:
    //
    //   iserdes_window[15:0] = { current[7:0], previous[7:0] }
    //                            ^^^^^^^^^^^   ^^^^^^^^^^^^
    //                            newest bits   oldest bits
    //
    // Then extract 8 contiguous bits starting at offset `bitslip_count`
    // (0..7, determined during gate training). This is equivalent to a
    // barrel shifter / bitslip by N positions:
    //
    //   bitslip=0 → window[7:0]   (all from previous capture)
    //   bitslip=3 → window[10:3]  (5 from previous, 3 from current)
    //   bitslip=7 → window[14:7]  (all 8 from current, shifted)
    //
    // The training FSM finds the correct bitslip value by comparing
    // aligned_dq against the known MPR2 pattern (8'b11110000).
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
    //     [phase*DFI_DATA_WIDTH + TOTAL_DQ + lane*DQ_BITS + bit] = fall beat
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
            coarse_tap_idx      <= 2'b0;
            idelay_cntvalue     <= 9'b0;
            sweep_tap           <= 9'b0;
            odelay_dqs_cntvalue <= 9'b0;
            odelay_dq_cntvalue  <= 9'b0;
            wl_dqs_strobe       <= 1'b0;
            wl_dqs_strobe_d1    <= 1'b0;
            en_vtc_q            <= 1'b1;
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
            wl_dqs_strobe_d1 <= wl_dqs_strobe;

            // Update previous ISERDESE3 outputs for bitslip window
            for (dfi_pack_idx = 0; dfi_pack_idx < TOTAL_DQ; dfi_pack_idx = dfi_pack_idx + 1)
                prev_iserdes_q[dfi_pack_idx] <= iserdes_dq_q[dfi_pack_idx];

            // DFI Read Data Packing: capture aligned ISERDESE3 outputs on rddata_en.
            // Gated by rddata_en: capture once, hold until next read.
            if (|i_dfi_rddata_en) begin
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
            end

            // rddata_valid: 1 cycle after rddata_en
            o_dfi_rddata_valid <= i_dfi_rddata_en;

            // ---------------------------------------------------------
            // PHY Training FSM
            //
            // Three-phase training sequence controlled by the memory
            // controller via DFI training interface signals:
            //
            // Phase 1 — GATE TRAINING (rdlvl_gate_en):
            //   Aligns the ISERDESE3 bitslip so that the 8-bit deserialized
            //   word boundary matches the DRAM burst boundary. Uses MPR2
            //   (period-8 pattern 8'b11110000) for unique alignment.
            //   Algorithm: for each lane, try all 8 bitslip offsets. If no
            //   match, nudge IDELAY by ~20 taps (coarse_tap_idx) and retry.
            //   This escapes edge-sampling glitches where DQ transitions
            //   exactly at the capture clock edge.
            //
            // Phase 2 — EYE TRAINING (rdlvl_en):
            //   Sweeps IDELAYE3 from tap 0 to 508 in steps of 4, looking
            //   for the tap range where MPR2 data reads correctly (the
            //   "eye"). Records first_pass_tap and last_pass_tap, then
            //   centers IDELAY at (first + last) / 2. Verifies the center
            //   tap reads correctly.
            //
            // Phase 3 — WRITE LEVELING (wrlvl_en):
            //   Per JESD79-4D §4.7: sweeps DQS ODELAYE3 until the DRAM
            //   reports a 0→1 transition on DQ (indicating DQS rising edge
            //   is now aligned with CK rising edge). DQ ODELAY tracks DQS
            //   to maintain the 90° write data-to-strobe offset.
            //
            // After each phase completes, the FSM asserts the corresponding
            // DFI resp signal and returns to IDLE. The controller sequences
            // the three phases in order during initialization.
            //
            // Failure handling: training proceeds even if a lane fails
            // (to complete remaining lanes). Failures are latched in
            // gate_train_fail, eye_train_fail, wl_train_fail registers
            // for post-mortem debug via prober or waveform inspection.
            // ---------------------------------------------------------
            begin
                case (phy_state)
                    // Waits for controller to assert a training enable signal.
                    // Disables VTC (voltage-temperature compensation) so that
                    // IDELAY/ODELAY taps can be loaded without VTC interference.
                    PHY_IDLE: begin
                        if (i_dfi_rdlvl_gate_en) begin // Gate training starts on rdlvl_gate_en
                            en_vtc_q <= 1'b0;
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}};
                            train_lane <= 0;
                            bitslip_shift_count <= 4'd0;
                            coarse_tap_idx <= 2'd0;
                            gate_train_fail <= {BYTE_LANES{1'b0}};
                            phy_state <= PHY_GATE_BITSLIP;
                        end else if (i_dfi_rdlvl_en) begin // Eye training starts on rdlvl_en
                            en_vtc_q <= 1'b0;
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}};
                            train_lane <= 0;
                            sweep_tap <= 9'd0;
                            idelay_cntvalue <= 9'd0;
                            eye_train_fail <= {BYTE_LANES{1'b0}};
                            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                eye_found[dfi_pack_idx] <= 1'b0;
                                first_pass_tap[dfi_pack_idx] <= 9'd0;
                                last_pass_tap[dfi_pack_idx] <= 9'd0;
                            end
                            phy_timer <= 3'd4;
                            phy_state <= PHY_EYE_SWEEP;
                        end else if (i_dfi_wrlvl_en) begin // Write leveling starts on wrlvl_en
                            en_vtc_q <= 1'b0;
                            o_dfi_wrlvl_resp <= {BYTE_LANES{1'b0}};
                            train_lane <= 0;
                            wl_train_fail <= {BYTE_LANES{1'b0}};
                            odelay_dqs_cntvalue <= odelay_dqs_cntvalueout[0];
                            odelay_dq_cntvalue  <= 9'd0;
                            // DQS ODELAY was initialized to tCK/4 (90° DQS-to-DQ
                            // centering). IODELAY BISC converts that ps value to taps.
                            // Read back actual tap via CNTVALUEOUT so WL sweeps
                            // from the calibrated 90° baseline, finding the
                            // additional delay for DQS-to-CK alignment at DRAM.
                            for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                dqs_initial_tap[dfi_pack_idx] <= odelay_dqs_cntvalueout[dfi_pack_idx];
                                wl_tap[dfi_pack_idx]    <= odelay_dqs_cntvalueout[dfi_pack_idx];
                                wl_dq_tap[dfi_pack_idx] <= 9'd0;
                                wl_prev_dq0[dfi_pack_idx] <= 1'b0;
                            end
                            phy_timer <= 3'd4;
                            phy_state <= PHY_WL_SAMPLE;
                        end
                    end

                    // -- Gate training (bitslip alignment using MPR2 period-8) --
                    // Tries all 8 bitslip offsets at the current IDELAYE3 tap.
                    // If no match, advances to next coarse tap (0→20→40→60)
                    // to escape edge-sampling. All coarse taps stay within
                    // one UI (~417ps @ DDR4-2400), preserving bitslip validity.
                    // phy_timer: wait cycles for bitslip/IDELAY to settle
                    // before sampling. rddata_en comes from controller to
                    // indicate the DRAM is actively driving MPR data.
                    PHY_GATE_BITSLIP: begin
                        if (phy_timer != 0)
                            phy_timer <= phy_timer - 1'b1;
                        else if (|i_dfi_rddata_en) begin // Wait for controller to assert rddata_en (DRAM is driving MPR data)
                            if (aligned_dq[train_lane * DQ_BITS] == MPR_PATTERN) begin // Compare DQ[0] of current lane against expected MPR2
                                phy_state <= PHY_GATE_DQS_FIND;
                                `ifndef YOSYS // Display bitslip result for this lane at the end of gate training for the lane 
                                    $display("[%0t] PHY gate: lane %0d bitslip=%0d match (tap=%0d)", $realtime, train_lane, bitslip_count_q[train_lane], {3'b0, coarse_tap_idx, 4'b0});
                                `endif
                            end else if (bitslip_shift_count == 4'd8) begin // Tried all 8 bitslip positions with no match at this tap so move to next coarse tap
                                if (coarse_tap_idx == 2'd3) begin // Tried all 4 coarse taps with no match -- gate training failed for this lane
                                    coarse_tap_idx <= 2'd0;
                                    idelay_cntvalue <= 9'd0;
                                    idelay_load_lane[train_lane] <= 1'b1;
                                    gate_train_fail[train_lane] <= 1'b1;
                                    phy_state <= PHY_GATE_DQS_FIND;
                                    `ifndef YOSYS // Display failure message for this lane but continue training remaining lanes
                                        $display("[%0t] PHY gate failed: lane %0d exhausted all coarse taps, proceeding", $realtime, train_lane);
                                    `endif
                                end else begin // Try next coarse tap
                                    coarse_tap_idx <= coarse_tap_idx + 1'b1;
                                    // (idx+1) << 4 = tap 16, 32, or 48 -- enough to escape edge-sampling without losing bitslip validity
                                    idelay_cntvalue <= {3'b0, coarse_tap_idx + 1'b1, 4'b0};
                                    idelay_load_lane[train_lane] <= 1'b1;
                                    bitslip_count_q[train_lane] <= 3'b0;
                                    bitslip_shift_count <= 4'd0;
                                    phy_timer <= 3'd4;
                                    `ifndef YOSYS // Display new tap value for next attempt
                                        $display("[%0t] PHY gate: lane %0d trying coarse tap %0d", $realtime, train_lane, {3'b0, coarse_tap_idx + 1'b1, 4'b0});
                                    `endif
                                end
                            end else begin // Data mismatch, try next bitslip position
                                bitslip_count_q[train_lane] <= bitslip_count_q[train_lane] + 1'b1;
                                bitslip_shift_count <= bitslip_shift_count + 1'b1;
                                phy_timer <= 3'd3;
                            end
                        end
                    end

                    // Advance to next lane or finish gate training.
                    // Resets coarse_tap_idx for the new lane. On last lane,
                    // resets IDELAY to 0 across all lanes so eye training
                    // starts from a known tap position.
                    PHY_GATE_DQS_FIND: begin
                        if (|i_dfi_rddata_en) begin
                            if (train_lane < BYTE_LANES - 1) begin // Advance to next lane
                                train_lane <= train_lane + 1'b1;
                                bitslip_shift_count <= 4'd0;
                                coarse_tap_idx <= 2'd0;
                                phy_state <= PHY_GATE_BITSLIP;
                            end else begin // Last lane done -- finish gate training and prepare for eye training
                                idelay_cntvalue <= 9'd0;
                                for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                    idelay_load_lane[dfi_pack_idx] <= 1'b1;
                                end
                                phy_state <= PHY_GATE_DONE;
                                `ifndef YOSYS // Display final bitslip results for all lanes at the end of gate training
                                    for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin 
                                        $display("[%0t] PHY gate done: lane %0d bitslip=%0d", $realtime, dfi_pack_idx, bitslip_count_q[dfi_pack_idx]);
                                    end
                                `endif
                            end
                        end
                    end

                    // Signal gate training complete to controller.
                    // Holds resp high until controller deasserts rdlvl_gate_en.
                    // Per DFI 3.1 Fig.57: resp deasserts when enable drops.
                    PHY_GATE_DONE: begin
                        o_dfi_rdlvl_resp <= {BYTE_LANES{1'b1}}; // assert completion to MC
                        if (!i_dfi_rdlvl_gate_en) begin
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}}; // deassert per DFI handshake
                            phy_state <= PHY_IDLE;
                        end
                    end

                    // -- Eye training (IDELAYE3 DQ sweep) -------------
                    // Sweeps IDELAY tap from 0 to 508 in steps of 4.
                    // At each tap, waits for rddata_en then compares
                    // aligned_dq against MPR_PATTERN. Tracks the first
                    // and last passing taps to define the data eye window.
                    PHY_EYE_SWEEP: begin
                        if (phy_timer != 0) begin // Wait for IDELAY to settle before sampling
                            if (phy_timer == 3'd3) begin
                                idelay_load_lane[train_lane] <= 1'b1;
                            end
                            phy_timer <= phy_timer - 1'b1;
                        end else if (|i_dfi_rddata_en) begin // Wait for controller to assert rddata_en (DRAM is driving MPR data)
                            if (aligned_dq[train_lane * DQ_BITS] == MPR_PATTERN) begin // Data matches at this tap
                                if (!eye_found[train_lane]) begin // First time we've seen a passing tap -- record it as first pass tap
                                    first_pass_tap[train_lane] <= sweep_tap;
                                    eye_found[train_lane] <= 1'b1;
                                end
                                last_pass_tap[train_lane] <= sweep_tap; // Update last pass tap to the most recent passing tap (will end up as the last passing tap after the sweep finishes)
                                // Advance to next tap or finish if at 511
                                if (sweep_tap == 9'd508) begin
                                    // Last valid step -- eye stays open to end
                                    phy_state <= PHY_EYE_CENTER;
                                end else begin // Continue sweeping
                                    sweep_tap <= sweep_tap + {5'b0, TAP_SWEEP_STEP};
                                    idelay_cntvalue <= sweep_tap + {5'b0, TAP_SWEEP_STEP};
                                    phy_timer <= 3'd4;
                                end
                            end else begin // Data mismatch
                                if (eye_found[train_lane]) begin
                                    // Eye has closed -- we have both boundaries
                                    phy_state <= PHY_EYE_CENTER;
                                end else begin
                                    // Haven't found eye yet -- keep sweeping
                                    if (sweep_tap == 9'd508) begin
                                        // Exhausted all taps without finding eye
                                        `ifndef YOSYS
                                            $display("[%0t] PHY eye: lane %0d no eye found", $realtime, train_lane);
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

                    // Compute center tap = (first_pass + last_pass) / 2 and load it into IDELAY for verification.
                    PHY_EYE_CENTER: begin
                        idelay_cntvalue <= ({1'b0, first_pass_tap[train_lane]} + {1'b0, last_pass_tap[train_lane]}) >> 1;
                        phy_timer <= 3'd4;
                        phy_state <= PHY_EYE_VERIFY;
                        `ifndef YOSYS // Display eye sweep results for this lane at the end of the sweep (before verification)
                            $display("[%0t] PHY eye: lane %0d first=%0d last=%0d center=%0d", $realtime, train_lane, first_pass_tap[train_lane], last_pass_tap[train_lane],
                                ({1'b0, first_pass_tap[train_lane]} + {1'b0, last_pass_tap[train_lane]}) >> 1);
                        `endif
                    end

                    // Re-read MPR at the center tap to confirm it still passes.
                    // If verified, advance to next lane or finish eye training.
                    // If failed, latch eye_train_fail and proceed anyway.
                    PHY_EYE_VERIFY: begin
                        if (phy_timer != 0) begin // Wait for IDELAY to settle before sampling
                            if (phy_timer == 3'd3) begin // Load the center tap value into the IDELAYE3 for this lane
                                idelay_load_lane[train_lane] <= 1'b1;
                            end
                            phy_timer <= phy_timer - 1'b1;
                        end else if (|i_dfi_rddata_en) begin // Wait for controller to assert rddata_en (DRAM is driving MPR data)
                            if (aligned_dq[train_lane * DQ_BITS] == MPR_PATTERN) begin // Verified -- advance to next lane or finish
                                if (train_lane < BYTE_LANES - 1) begin // Advance to next lane
                                    train_lane <= train_lane + 1'b1;
                                    sweep_tap <= 9'd0;
                                    idelay_cntvalue <= 9'd0;
                                    eye_found[train_lane + 1'b1] <= 1'b0;
                                    phy_timer <= 3'd4;
                                    phy_state <= PHY_EYE_SWEEP;
                                end else begin // Last lane done -- finish eye training
                                    phy_state <= PHY_EYE_DONE;
                                    `ifndef YOSYS // Display final eye training results for all lanes at the end of eye training
                                        for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                            $display("[%0t] PHY eye done: lane %0d tap=%0d", $realtime, dfi_pack_idx, (first_pass_tap[dfi_pack_idx] + last_pass_tap[dfi_pack_idx]) >> 1);
                                        end
                                    `endif
                                end
                            end else begin
                                // Verification failed — latch failure, proceed to next lane
                                eye_train_fail[train_lane] <= 1'b1;
                                `ifndef YOSYS // Display failure message for this lane but continue training remaining lanes
                                    $display("[%0t] PHY eye: lane %0d verify FAILED at center tap", $realtime, train_lane);
                                `endif
                                if (train_lane < BYTE_LANES - 1) begin // Advance to next lane
                                    train_lane <= train_lane + 1'b1;
                                    sweep_tap <= 9'd0;
                                    idelay_cntvalue <= 9'd0;
                                    eye_found[train_lane + 1'b1] <= 1'b0;
                                    phy_timer <= 3'd4;
                                    phy_state <= PHY_EYE_SWEEP;
                                end else begin // Last lane done -- finish eye training
                                    phy_state <= PHY_EYE_DONE;
                                end
                            end
                        end
                    end

                    // Signal eye training complete. Holds resp until
                    // controller deasserts rdlvl_en.
                    // Per DFI 3.1 Fig.57: resp deasserts when enable drops.
                    PHY_EYE_DONE: begin
                        o_dfi_rdlvl_resp <= {BYTE_LANES{1'b1}}; // assert completion to MC
                        if (!i_dfi_rdlvl_en) begin
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}}; // deassert per DFI handshake
                            phy_state <= PHY_IDLE;
                        end
                    end

                    // -- Write leveling (ODELAYE3 DQS sweep) ----------
                    // Load new DQS/DQ ODELAY values, then wait for the
                    // controller to pulse wrlvl_strobe (triggers one DQS
                    // toggle so the DRAM samples CK and returns the result).
                    PHY_WL_SAMPLE: begin
                        if (phy_timer != 0) begin // Wait for ODELAY to settle before pulsing strobe
                            if (phy_timer == 3'd3) begin
                                odelay_dqs_load[train_lane] <= 1'b1;
                                odelay_dq_load[train_lane]  <= 1'b1;
                            end
                            phy_timer <= phy_timer - 1'b1;
                        end else if (i_dfi_wrlvl_strobe) begin // Wait for controller strobe
                            wl_dqs_strobe <= 1'b1;
                            phy_timer <= 3'd4;
                            phy_state <= PHY_WL_ADJUST;
                        end
                    end

                    // Read back DRAM's WL response after strobe settles.
                    // Detect 0→1 transition on DQ[0] (OR-reduced across bits):
                    // previous=0, current=1 means DQS now leads CK — done.
                    // Otherwise increment DQS/DQ ODELAY and retry.
                    PHY_WL_ADJUST: begin
                        if (phy_timer != 0) // Wait for strobe to settle before sampling
                            phy_timer <= phy_timer - 1'b1;
                        else begin
                            // Write leveling edge detection (JESD79-4D §4.7):
                            // DRAM samples CK with the rising DQS edge and feeds
                            // back the result on ALL DQ bits. The controller sweeps
                            // DQS delay until DQ transitions 0->1, indicating DQS
                            // rising edge is now aligned with CK rising edge.
                            // DQ ODELAYE3 tracks DQS to preserve the 90° write offset.
                            `ifndef YOSYS
                                $display("[%0t] PHY WL sweep: lane %0d dqs_tap=%0d dq_tap=%0d DQ=%0b prev=%0b", $realtime, train_lane, wl_tap[train_lane],
                                    wl_dq_tap[train_lane], |iserdes_dq_q[train_lane * DQ_BITS], wl_prev_dq0[train_lane]);
                            `endif
                            if (!wl_prev_dq0[train_lane] && |iserdes_dq_q[train_lane * DQ_BITS]) begin // Detected 0->1 transition -- write leveling for this lane is done
                                phy_state <= PHY_WL_CHECK;
                            end else begin // No transition yet -- increment taps and try again
                                wl_prev_dq0[train_lane] <= |iserdes_dq_q[train_lane * DQ_BITS]; // Update previous DQ state for next transition detection
                                if (wl_tap[train_lane][8:2] == 7'b1111111) begin // Reached max tap 508-511 without seeing transition -- WL failed for this lane
                                    wl_train_fail[train_lane] <= 1'b1;
                                    `ifndef YOSYS
                                        $display("[%0t] PHY WL Failed: lane %0d exhausted taps", $realtime, train_lane);
                                    `endif
                                    phy_state <= PHY_WL_CHECK;
                                end else begin // Increment ODELAY taps and try again
                                    wl_tap[train_lane] <= wl_tap[train_lane] + {5'b0, WL_TAP_STEP};
                                    wl_dq_tap[train_lane] <= wl_dq_tap[train_lane] + {5'b0, WL_TAP_STEP};
                                    odelay_dqs_cntvalue <= wl_tap[train_lane] + {5'b0, WL_TAP_STEP};
                                    odelay_dq_cntvalue <= wl_dq_tap[train_lane] + {5'b0, WL_TAP_STEP};
                                    phy_timer <= 3'd4;
                                    phy_state <= PHY_WL_SAMPLE;
                                end
                            end
                        end
                    end

                    // Advance to next lane or finish write leveling.
                    // Re-enables VTC and waits for it to settle before
                    // signaling completion to the controller.
                    PHY_WL_CHECK: begin
                        `ifndef YOSYS
                            $display("[%0t] PHY WL: lane %0d dqs_tap=%0d dq_tap=%0d", $realtime, train_lane, wl_tap[train_lane], wl_dq_tap[train_lane]);
                        `endif
                        if (train_lane < BYTE_LANES - 1) begin
                            train_lane <= train_lane + 1'b1;
                            wl_tap[train_lane + 1'b1]    <= dqs_initial_tap[train_lane + 1'b1]; // resume from 90° baseline (tCK/4 tap set by IODELAY BISC)
                            wl_dq_tap[train_lane + 1'b1] <= 9'd0;              // DQ has no initial offset — tracks DQS delta after WL
                            wl_prev_dq0[train_lane + 1'b1] <= 1'b0;
                            odelay_dqs_cntvalue <= dqs_initial_tap[train_lane + 1'b1]; // load DQS ODELAY to same 90° baseline
                            odelay_dq_cntvalue  <= 9'd0;                       // DQ ODELAY starts at 0, incremented in lockstep with DQS
                            phy_timer <= 3'd4;
                            phy_state <= PHY_WL_SAMPLE;
                        end else begin // Last lane done -- finish write leveling and prepare for normal operation
                            en_vtc_q <= 1'b1;
                            vtc_settle_counter <= VTC_SETTLE_CYCLES;
                            phy_state <= PHY_WL_DONE;
                            `ifndef YOSYS // Display final write leveling results for all lanes at the end of write leveling
                                for (dfi_pack_idx = 0; dfi_pack_idx < BYTE_LANES; dfi_pack_idx = dfi_pack_idx + 1) begin
                                    $display("[%0t] PHY WL done: lane %0d dqs_tap=%0d dq_tap=%0d", $realtime, dfi_pack_idx, wl_tap[dfi_pack_idx], wl_dq_tap[dfi_pack_idx]);
                                end
                            `endif
                        end
                    end

                    // Wait for VTC to re-lock after re-enabling EN_VTC,
                    // then signal write leveling complete to controller.
                    // Per DFI 3.1 Fig.59: resp deasserts when enable drops.
                    PHY_WL_DONE: begin
                        if (vtc_settle_counter != 0)
                            vtc_settle_counter <= vtc_settle_counter - 1'b1;
                        else begin
                            o_dfi_wrlvl_resp <= {BYTE_LANES{1'b1}}; // assert completion to MC
                            if (!i_dfi_wrlvl_en) begin
                                o_dfi_wrlvl_resp <= {BYTE_LANES{1'b0}}; // deassert per DFI handshake
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

    assign o_phy_train_fail_gate = gate_train_fail;
    assign o_phy_train_fail_eye  = eye_train_fail;
    assign o_phy_train_fail_wl   = wl_train_fail;

endmodule
