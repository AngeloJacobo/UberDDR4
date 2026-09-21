////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_phy.v
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  PHY for DDR4 controller targeting Xilinx UltraScale+ FPGAs.
//  Handles OSERDESE3/ISERDESE3/IDELAYE3/ODELAYE3 primitives and the
//  implemented DFI 3.1 data path. Includes PHY training (gate, eye, WL).
//
// Architecture overview:
//  The PHY sits between the DFI 3.1 interface and the DDR4 SDRAM pins.
//  One controller clock cycle = 4 DDR4 CK periods = 8 data unit intervals.
//
//  Write path:  DFI wrdata -> OSERDESE3 (8:1 DDR) -> ODELAYE3 -> IOBUF -> pad
//  Read path:   pad -> IOBUF -> IDELAYE3 -> ISERDESE3 (1:8 DDR) -> bitslip
//               barrel shifter -> DFI rddata
//  Clock path:  OSERDESE3 (constant 01010101 toggle) -> OBUFDS -> CK/CK#
//  Cmd/Addr:    OSERDESE3 (SDR 4:1, doubled bits) -> OBUF -> DDR4 CA pins
//
//  Training FSM (after IDELAYCTRL ready, driven by MC; repeats on reset/retry):
//   1. Gate handshake: Acknowledge without a separate gate search; the eye
//                      search also establishes fabric bitslip alignment.
//   2. Eye training:   Sweep IDELAYE3 taps and offsets 0..8 in the 16-bit
//                      previous/current capture window; load and verify the
//                      selected eye center, bitslip and on-time/late setting.
//   3. Write leveling: Sweep ODELAYE3 DQS tap to find the 0->1 CK edge
//                      on DQ[0]. DQ ODELAYE3 tracks DQS to keep 90 deg.
//
//  EN_VTC (voltage-temperature compensation): held LOW during eye/WL training
//  so delay taps can be changed. Set HIGH in normal operation so the
//  IDELAYE3/ODELAYE3 primitives track PVT drift automatically.
//
// Clocking and packing contract: docs/INTEGRATION.md and docs/ARCHITECTURE.md.
// The component delay primitives use REFCLK_FREQUENCY=300.0; supply 300 MHz
// on i_ref_clk and phase-related CK/quarter-rate clocks.
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
    // Number of 8-bit byte lanes (typically 2 for x8, 2 for x16, 2+ for x4)
              BYTE_LANES = 2,
    // Derived from DEVICE_WIDTH -- do not override
    parameter BA_BITS = 2,      //bank address (always 2 for DDR4)
              BG_BITS = (DEVICE_WIDTH == 16) ? 1 : 2, //JESD79-4D Table 4
              DQ_BITS = 8,      //always 8 (byte-lane granularity)
    parameter SERDES_RATIO = 4,
              DFI_DATA_WIDTH = 2 * DQ_BITS * BYTE_LANES //per DFI phase
) (
    // Clocks and reset
    input wire                              i_controller_clk,
    input wire                              i_ddr4_clk,
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
    // Extended training debug (CSR 0x8, 0x9, 0xE readback)
    output wire [9*BYTE_LANES-1:0]          o_phy_best_width,       // 9b per lane: eye width in IDELAY taps
    output wire [9*BYTE_LANES-1:0]          o_phy_best_start,       // 9b per lane: first passing IDELAY tap
    output wire [9*BYTE_LANES-1:0]          o_phy_wl_dq_tap,        // 9b per lane: DQ ODELAYE3 tap after WL
    output wire [9*BYTE_LANES-1:0]          o_phy_dqs_initial_tap,  // 9b per lane: BISC-calibrated DQS baseline
    output wire [BYTE_LANES-1:0]            o_phy_rd_lat_extra,     // 1b per lane: read data arrives 1 CLKDIV late
    output wire                             o_phy_en_vtc            // 1 = voltage-temperature compensation active
);



    // -----------------------------------------------------------------
    // ODELAYE3/IDELAYE3 delay configuration (all in TIME mode, ps units)
    // DQS ODELAYE3 adds DDR4_CLK_PERIOD/4 ps = 90 deg phase shift so DQS
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
                    PHY_WL_APPLY      = 4'd12;

    // MPR page 0, MPR2 register value = 8'h0F = 00001111 (JESD79-4D Table 56).
    // Serial readout sends bit[7] first: UI0=0, UI1=0, UI2=0, UI3=0,
    // UI4=1, UI5=1, UI6=1, UI7=1.
    // ISERDESE3 8:1 DDR captures Q[0]=first bit received (UG571, ISERDESE3 timing description):
    //   Q[0]=UI0=0, Q[1]=UI1=0, ..., Q[4]=UI4=1, ..., Q[7]=UI7=1
    //   -> Q[7:0] = 8'b11110000
    // Period = 8 UI: uniquely identifies all 8 possible bitslip values.
    localparam [7:0] MPR_PATTERN = 8'b11110000;

    // Eye training: sweep IDELAYE3 from tap 0 to 508 in steps of 4 (~128 iterations).
    // Stops at 508 (not 511) to avoid 9-bit overflow on the +4 addition.
    localparam [3:0] TAP_SWEEP_STEP = 4'd4;

    // Write leveling: sweep ODELAYE3 DQS in steps of 4.  The IODELAY BISC
    // result for DQS_INITIAL_ODELAY_PS is one quarter of tCK, so four times
    // its measured tap count is the local, calibrated tCK estimate used to
    // unwrap a valid 0-to-1 feedback edge.
    localparam [3:0] WL_TAP_STEP = 4'd4;

    // VTC settle: guard time after EN_VTC goes HIGH before normal operation.
    // Per UG571 VAR_LOAD procedure step 8: "Set EN_VTC High for VT compensation"
    // then wait before resuming. 200 cycles is conservative for IODELAY BISC re-lock.
    localparam [7:0] VTC_SETTLE_CYCLES = 8'd200; // UG571 has no exact count; MIG uses 10-16. 200 is safe (one-shot per training).

    // -----------------------------------------------------------------
    // DFI Data Layout
    //
    // The DFI interface carries 4 phases x 2 edges (rise+fall) of data
    // per controller clock. Each edge transfers TOTAL_DQ bits in parallel
    // -----------------------------------------------------------------
    localparam TOTAL_DQ      = DQ_BITS * BYTE_LANES;  // DQ bits per clock edge (= half of DFI_DATA_WIDTH)
    localparam DM_PER_PHASE  = 2 * BYTE_LANES;       // DM bits per DFI phase: 1 per lane x 2 edges
    localparam DM_ENABLED    = (DEVICE_WIDTH != 4);   // x4 has no DM pin (JESD79-4D Table 28)

    // -----------------------------------------------------------------
    // Reset Generation (UG571 "Component Mode Reset Sequence", p.188-189)
    //
    // The IODELAY and SERDES primitive resets are generated in the controller
    // clock domain.  IDELAYCTRL has a separate REFCLK domain, so its reset is
    // asserted asynchronously but released through a two-flop synchronizer
    // clocked by i_ref_clk.  This prevents a controller-clock signal from
    // directly deasserting the IDELAYCTRL RST pin.
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
    localparam integer IODELAY_RST_DELAY = (52_000 / CONTROLLER_CLK_PERIOD) + 2;
    localparam integer IDELAYCTRL_RST_EXTRA = 4;
    /* verilator lint_off WIDTHTRUNC */
    localparam [$clog2(IODELAY_RST_DELAY + IDELAYCTRL_RST_EXTRA + 1):0] RST_RELEASE_SERDES = IODELAY_RST_DELAY;
    localparam [$clog2(IODELAY_RST_DELAY + IDELAYCTRL_RST_EXTRA + 1):0] RST_RELEASE_CTRL   = IODELAY_RST_DELAY + IDELAYCTRL_RST_EXTRA;
    /* verilator lint_on WIDTHTRUNC */

    reg [$clog2(IODELAY_RST_DELAY + IDELAYCTRL_RST_EXTRA + 1):0] rst_cnt;
    reg sync_rst;
    reg idelayctrl_release_req;
    (* ASYNC_REG = "TRUE" *) reg [1:0] idelayctrl_release_sync;
    wire idelayctrl_rst;


    always @(posedge i_controller_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            rst_cnt       <= 0;
            sync_rst      <= 1'b1;
            idelayctrl_release_req <= 1'b0;
        end else begin
            if (!(&rst_cnt)) begin // count up until max (saturating)
                rst_cnt <= rst_cnt + 1;
            end

            // Step 2c: release SERDES/IDELAY/ODELAY reset after IODELAY_RST_DELAY
            if (rst_cnt == RST_RELEASE_SERDES) begin
                sync_rst <= 1'b0;
            end

            // Step 2d: request IDELAYCTRL reset release AFTER sync_rst.
            // The request is synchronized into i_ref_clk below.
            if (rst_cnt == RST_RELEASE_CTRL) begin
                idelayctrl_release_req <= 1'b1;
            end
        end
    end

    // IDELAYCTRL.RST may assert asynchronously, but deassertion must be clean
    // with respect to REFCLK. Use two-flop CDC synchronizer
    always @(posedge i_ref_clk or negedge i_rst_n) begin
        if (!i_rst_n)
            idelayctrl_release_sync <= 2'b00;
        else
            idelayctrl_release_sync <= {idelayctrl_release_sync[0],
                                        idelayctrl_release_req};
    end
    assign idelayctrl_rst = ~idelayctrl_release_sync[1];

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

    // EN_VTC: LOW during training so IDELAYE3/ODELAYE3 tap values can be
    // loaded without the IDELAYCTRL overwriting them. HIGH in normal operation
    // so the IDELAYCTRL continuously compensates delay for PVT drift (UG571).
    reg en_vtc_q;

    // -----------------------------------------------------------------
    // Clock Output Path
    // OSERDESE3 (DATA_WIDTH=8, constant 01010101 toggle) -> OBUFDS -> CK/CK#
    // The SERDES toggles every UI, producing the DDR4 memory clock.
    // -----------------------------------------------------------------
    wire ck_oserdes_out;

    /* verilator lint_off PINCONNECTEMPTY */
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

    localparam integer CK_ODELAY_PS = DDR4_CLK_PERIOD / 4;

    wire ck_delayed;
    (* IODELAY_GROUP = "ddr4_phy_iodelay" *)
    ODELAYE3 #(
        .CASCADE("NONE"), .DELAY_FORMAT("TIME"),
        .DELAY_TYPE("VAR_LOAD"), .DELAY_VALUE(CK_ODELAY_PS),
        .IS_CLK_INVERTED(1'b0), .IS_RST_INVERTED(1'b0),
        .REFCLK_FREQUENCY(300.0), .SIM_DEVICE("ULTRASCALE_PLUS"),
        .UPDATE_MODE("ASYNC")
    ) odelay_ck (
        .ODATAIN(ck_oserdes_out), .DATAOUT(ck_delayed),
        .CLK(i_controller_clk), .RST(sync_rst),
        .CE(1'b0), .INC(1'b0),
        .LOAD(1'b0),
        .CNTVALUEIN(9'b0),
        .CNTVALUEOUT(),
        .EN_VTC(en_vtc_q),
        .CASC_IN(1'b0), .CASC_RETURN(1'b0), .CASC_OUT()
    );

    OBUFDS ck_buf (
        .I(ck_delayed),
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
    // Each is a single-bit signal with 4 DFI phases -> one OSERDES each.
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
    // OSERDESE3 T pin controls this: T=1 -> high-Z (off), T=0 -> driven.
    //
    // HOW IT WORKS:
    //
    // 1) wrdata_en_any = OR of all 4 DFI phase enables -> collapses to
    //    a single "is there a write THIS controller clock?" flag.
    //    (One controller clock already covers all 4 DDR phases.)
    //
    // 2) A 3-stage shift register delays the flag by 1, 2, and 3 controller
    //    clocks (each = 4 DDR clocks). This keeps the bus driven for
    //    3 extra controller clocks (12 DDR clocks) after wrdata_en drops.
    //    The third stage is required because the OSERDESE3 T_OUT path is
    //    not delayed by the per-lane ODELAYE3. It guarantees that the
    //    delayed DQS falling edge and its low tWPST postamble reach the
    //    DRAM pin before the IOBUF/IOBUFDS goes high-Z.
    //
    // 3) output_enable = wrdata_en_any | shift[0] | shift[1] | shift[2]
    //    Both DQ and DQS use the same enable window.
    //
    // 4) Inversion: ~enable -> tristate, because T=1 means OFF in OSERDESE3.
    // -----------------------------------------------------------------
    wire wrdata_en_any = |i_dfi_wrdata_en;

    reg [2:0] wrdata_en_shift;
    always @(posedge i_controller_clk) begin
        if (sync_rst)
            wrdata_en_shift <= 3'b0;
        else
            wrdata_en_shift <= {wrdata_en_shift[1:0], wrdata_en_any};
    end

    wire output_enable = wrdata_en_any | (|wrdata_en_shift);
    wire dq_tristate   = ~output_enable;

    // -----------------------------------------------------------------
    // DQS Pattern Generation
    //
    // The OSERDESE3 for DQS gets an 8-bit pattern (4 phases x rise/fall):
    //   Normal write: 01_01_01_01 -> continuous toggle, edges centered on DQ
    //   Idle:         00_00_00_00 -> DQS held low (pin is tri-stated anyway)
    //
    // Write Leveling (WL): DRAM calibration mode where the controller
    // sends a single DQS rising edge and reads back DQ to find the
    // optimal clock-to-DQS alignment (JEDEC DDR4 sec 4.7.2).
    //   WL strobe:    00_00_00_01 -> one rising edge only
    //   WL idle:      00_00_00_00 -> hold low between strobes
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
        end else if (wrdata_en_shift[0]) begin
            // Write postamble: after last data beat (DQS_t LOW), drive
            // one UI HIGH per JEDEC tWPST >= 0.33 tCK (we provide 0.5 tCK).
            // D[0] is first transmitted -> first UI on wire is HIGH.
            dqs_pattern = 8'b00_00_00_01;
        end else begin
            dqs_pattern = 8'b00_00_00_00; // idle: pin is tri-stated
        end
    end

    // WL tri-state: DQS held driven (T=0) for the entire WL phase per
    // JESD79-4D sec 4.7.2 - controller drives DQS LOW between strobes.
    wire dqs_tristate_wl = wl_active ? 1'b0 : ~output_enable;


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
    (* mark_debug = "true" *)  wire [7:0] iserdes_dq_q  [TOTAL_DQ-1:0];
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
                wire dq_tristate_oserdes;
                OSERDESE3 #(
                    .DATA_WIDTH(8), .INIT(1'b0),
                    .IS_CLKDIV_INVERTED(1'b0), .IS_CLK_INVERTED(1'b0),
                    .IS_RST_INVERTED(1'b0), .SIM_DEVICE("ULTRASCALE_PLUS")
                ) oserdes_dq (
                    .D(dq_wr_d), .OQ(oserdes_dq_out), .T_OUT(dq_tristate_oserdes),
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
                    .IO(io_ddr4_dq[DQ_IDX]), .T(dq_tristate_oserdes)
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
                    .FIFO_RD_CLK(1'b0), .FIFO_RD_EN(1'b0), .FIFO_EMPTY(),
                    .INTERNAL_DIVCLK()
                );
            end
        end
    endgenerate

    // -----------------------------------------------------------------
    // DQS Strobe Path (per byte lane)
    // Write: OSERDESE3(dqs_pattern) -> ODELAYE3 -> IOBUFDS -> DQS+/-
    //        ODELAYE3 adds ~90 deg (DDR4_CLK_PERIOD/4 ps) so DQS edges
    //        are center-aligned with DQ data at the DRAM receiver.
    // -----------------------------------------------------------------
    generate
        genvar dqs_lane;
        for (dqs_lane = 0; dqs_lane < BYTE_LANES; dqs_lane = dqs_lane + 1) begin : gen_dqs

            wire oserdes_dqs_out;
            wire dqs_tristate_wl_oserdes;
            OSERDESE3 #(
                .DATA_WIDTH(8), .INIT(1'b0),
                .IS_CLKDIV_INVERTED(1'b0), .IS_CLK_INVERTED(1'b0),
                .IS_RST_INVERTED(1'b0), .SIM_DEVICE("ULTRASCALE_PLUS")
            ) oserdes_dqs (
                .D(dqs_pattern), .OQ(oserdes_dqs_out), .T_OUT(dqs_tristate_wl_oserdes),
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
            // Does NOT affect normal operation - active drivers easily
            // overdrive the weak pull. Supported for DIFF_POD (DDR4).
            IOBUFDS #(
                .DQS_BIAS("TRUE")
            ) dqs_iobufds (
                .I(odelay_dqs_out), .O(),
                .IO(io_ddr4_dqs_p[dqs_lane]), .IOB(io_ddr4_dqs_n[dqs_lane]),
                .T(dqs_tristate_wl_oserdes)
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
    /* verilator lint_on PINCONNECTEMPTY */

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
    reg [3:0]  bitslip_count_q [BYTE_LANES-1:0];
    wire [7:0] aligned_dq [TOTAL_DQ-1:0];



    // PHY training FSM state registers
    reg [3:0] phy_state;
    (* mark_debug = "true" *)  reg [$clog2(BYTE_LANES > 1 ? BYTE_LANES : 2)-1:0] train_lane;
    reg [3:0] phy_timer;

    // Eye training registers (phase-aware range tracking)
    //
    // Algorithm: sweep IDELAYE3 taps 0->508 (step=4). At each tap, search
    // the 16-bit iserdes_window for MPR_PATTERN at offsets 0-8. A contiguous
    // run of taps with the SAME offset is a "stable range" - the eye is open.
    // When the offset changes or pattern disappears, the range closes. The
    // widest range found across the full sweep is selected; its center tap
    // is loaded into IDELAYE3 and its offset becomes the bitslip value.
    //
    // This subsumes gate training: the offset search at every tap finds the
    // correct bitslip automatically, so gate training is a no-op.
    reg [8:0] sweep_tap;            // current IDELAYE3 tap being tested (0-508, step 4)
    reg [8:0] cur_start;            // first tap of the range currently being built
    reg [8:0] cur_width;            // running width of current range (incremented by TAP_SWEEP_STEP)
    reg [3:0] cur_offset;           // pattern offset of the current range (0-8)
    reg       in_range;             // 1 = currently inside a valid (PASS) range
    (* mark_debug = "true" *) reg [8:0] best_start;           // first tap of the widest range found so far
    (* mark_debug = "true" *) reg [8:0] best_width;           // width of the widest range found so far
    reg [3:0] best_offset;          // pattern offset of the widest range
    (* mark_debug = "true" *) reg best_valid;           // 1 = at least one valid range has been recorded
    reg       pattern_found_q;      // pipeline reg: MPR pattern found at current tap (registered from comb)
    reg [3:0] pattern_offset_q;     // pipeline reg: offset where pattern was found (registered from comb)
    reg       pattern_late_q;       // pipeline reg: pattern arrived 1 CLKDIV cycle after rddata_en
    reg       cur_late;             // current range: data arrives late (1 cycle after rddata_en)
    reg       best_late;            // best range: data arrives late
    reg       verify_mode;          // 1 = PHY_EYE_LATE is doing verify check (not sweep late-check)
    reg [BYTE_LANES-1:0] rd_lat_extra; // per-lane: 1 = read data arrives 1 CLKDIV cycle late
    reg [SERDES_RATIO-1:0] rddata_en_d1; // 1-cycle delayed rddata_en for late-lane capture
    reg [2*SERDES_RATIO-1:0] ontime_shadow [TOTAL_DQ-1:0]; // shadow reg: holds on-time lane ISERDES data for 1 cycle
    reg [8:0] eye_center_tap [BYTE_LANES-1:0]; // final center tap per lane (for debug readback)
    reg [8:0] eye_best_width [BYTE_LANES-1:0]; // per-lane: widest eye range (IDELAY taps)
    reg [8:0] eye_best_start [BYTE_LANES-1:0]; // per-lane: first passing tap of widest range

    // Write leveling registers (ODELAYE3 DQS sweep)
    reg [8:0] wl_tap        [BYTE_LANES-1:0];
    reg [8:0] wl_dq_tap     [BYTE_LANES-1:0];

    // The DRAM WL response is asynchronous to the PHY.  A tap is accepted
    // only when the whole 8-UI ISERDES capture is unanimously low or high;
    // a mixed capture is retried at that tap instead of becoming a false
    // edge.  DQ[0] is the designated per-byte feedback bit (JEDEC 4.7).
    wire wl_feedback_zero = ~(|iserdes_dq_q[train_lane * DQ_BITS]);
    wire wl_feedback_one  =  &iserdes_dq_q[train_lane * DQ_BITS];

    reg       wl_seen_zero   [BYTE_LANES-1:0]; // zero observed before current rising-edge search
    reg [8:0] dqs_initial_tap [BYTE_LANES-1:0];
    reg [7:0] vtc_settle_counter;

    // Training failure latch registers (sticky - cleared on training start,
    // set on failure, visible on prober or in waveforms for post-mortem debug)
    reg [BYTE_LANES-1:0] gate_train_fail;
    reg [BYTE_LANES-1:0] eye_train_fail;
    reg [BYTE_LANES-1:0] wl_train_fail;

    // Keep the period estimate wider than a physical ODELAY setting.  This
    // makes the comparison well defined even if an unusual BISC result is
    // greater than 127 taps (and therefore four times it exceeds 511).
    wire [10:0] wl_period_estimate = {dqs_initial_tap[train_lane], 2'b00};
    wire         wl_edge_has_wrap  = ({2'b00, wl_tap[train_lane]} >=
                                      ({2'b00, dqs_initial_tap[train_lane]} + wl_period_estimate));
    wire [8:0]   wl_final_dqs_tap  = wl_edge_has_wrap
                                   ? (wl_tap[train_lane] - wl_period_estimate[8:0])
                                   :  wl_tap[train_lane];

    assign wl_active = (phy_state == PHY_WL_SAMPLE) || (phy_state == PHY_WL_ADJUST)
                     || (phy_state == PHY_WL_APPLY)  || (phy_state == PHY_WL_CHECK)
                     || (phy_state == PHY_WL_DONE);

    // -----------------------------------------------------------------
    // Bitslip Alignment (barrel-shift across two ISERDESE3 captures)
    // -----------------------------------------------------------------
    // Problem: ISERDESE3 captures 8 serial bits per CLKDIV cycle, but
    // the byte boundary is unknown - the first captured bit may not be
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
    // (0..8, determined during eye training). This is equivalent to a
    // barrel shifter / bitslip by N positions:
    //
    //   bitslip=0 -> window[7:0]   (all from previous capture)
    //   bitslip=3 -> window[10:3]  (5 from previous, 3 from current)
    //   bitslip=7 -> window[14:7] (1 from previous, 7 from current)
    //   bitslip=8 -> window[15:8] (all from current capture)
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
    // Eye Training: Combinational Pattern Search
    // Searches DQ[0] of the current training lane for MPR_PATTERN at
    // all 9 possible offsets (0-8) in the 16-bit iserdes_window.
    // Output is registered in the FSM for timing closure.
    // -----------------------------------------------------------------
    wire [15:0] train_window;
    assign train_window = {iserdes_dq_q[train_lane * DQ_BITS],
                           prev_iserdes_q[train_lane * DQ_BITS]};

    // Case-equality (===) is used instead of == because prev_iserdes_q
    // contains X in simulation when DQ is tri-stated between read bursts.
    // With ==, (X == value) produces X which poisons the priority encoder.
    // With ===, (X === value) returns definite 0, preventing false matches.
    // In synthesis, === behaves identically to == (hardware has no X).
    wire [8:0] offset_match;
    generate
        genvar om;
        for (om = 0; om <= 8; om = om + 1) begin : gen_offset_cmp
            assign offset_match[om] = (train_window[om +: 8] === MPR_PATTERN);
        end
    endgenerate

    // Priority encoder: find lowest offset where MPR_PATTERN matches.
    // At most 2 offsets match simultaneously (0 and 8, when perfectly aligned).
    // The priority encoder selects the lowest, which is the canonical bitslip.
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
    // DFI Read Data Packing + rddata_valid
    // Pack aligned ISERDESE3 outputs into flat o_dfi_rddata vector.
    // rddata_valid follows the capture pipeline; trained late lanes add a
    // controller cycle. See rd_lat_extra and rddata_en_d1 below.
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
            rd_lat_extra        <= {BYTE_LANES{1'b0}};
            rddata_en_d1        <= {SERDES_RATIO{1'b0}};
            odelay_dqs_cntvalue <= 9'b0;
            odelay_dq_cntvalue  <= 9'b0;
            wl_dqs_strobe       <= 1'b0;
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

            // Update previous ISERDESE3 outputs for bitslip window
            for (dfi_pack_idx = 0; dfi_pack_idx < TOTAL_DQ; dfi_pack_idx = dfi_pack_idx + 1)
                prev_iserdes_q[dfi_pack_idx] <= iserdes_dq_q[dfi_pack_idx];

            // Delayed rddata_en for late-lane capture (1 CLKDIV cycle delay)
            rddata_en_d1 <= i_dfi_rddata_en;

            // DFI Read Data Packing: capture aligned ISERDESE3 outputs.
            // When lanes have asymmetric rd_lat_extra (some late, some on-time),
            // on-time lanes are captured into a shadow register at rddata_en,
            // then assembled into o_dfi_rddata at rddata_en_d1 alongside late
            // lanes. This prevents the next read's on-time data from overwriting
            // the current read's data during back-to-back reads.
            if (|rd_lat_extra) begin
                // Shadow capture: on-time lanes latch at rddata_en
                if (|i_dfi_rddata_en) begin
                    for (dfi_pack_lane = 0; dfi_pack_lane < BYTE_LANES; dfi_pack_lane = dfi_pack_lane + 1) begin
                        if (!rd_lat_extra[dfi_pack_lane]) begin
                            for (dfi_pack_bit = 0; dfi_pack_bit < DQ_BITS; dfi_pack_bit = dfi_pack_bit + 1) begin
                                dfi_pack_idx = dfi_pack_lane * DQ_BITS + dfi_pack_bit;
                                ontime_shadow[dfi_pack_idx] <= aligned_dq[dfi_pack_idx];
                            end
                        end
                    end
                end
                // Final assembly at rddata_en_d1: late lanes from live aligned_dq,
                // on-time lanes from shadow register (captured previous cycle)
                if (|rddata_en_d1) begin
                    for (dfi_pack_lane = 0; dfi_pack_lane < BYTE_LANES; dfi_pack_lane = dfi_pack_lane + 1) begin
                        for (dfi_pack_bit = 0; dfi_pack_bit < DQ_BITS; dfi_pack_bit = dfi_pack_bit + 1) begin
                            dfi_pack_idx = dfi_pack_lane * DQ_BITS + dfi_pack_bit;
                            for (dfi_pack_phase = 0; dfi_pack_phase < SERDES_RATIO; dfi_pack_phase = dfi_pack_phase + 1) begin
                                if (rd_lat_extra[dfi_pack_lane]) begin
                                    o_dfi_rddata[dfi_pack_phase*DFI_DATA_WIDTH + dfi_pack_lane*DQ_BITS + dfi_pack_bit]
                                        <= aligned_dq[dfi_pack_idx][2*dfi_pack_phase];
                                    o_dfi_rddata[dfi_pack_phase*DFI_DATA_WIDTH + TOTAL_DQ + dfi_pack_lane*DQ_BITS + dfi_pack_bit]
                                        <= aligned_dq[dfi_pack_idx][2*dfi_pack_phase + 1];
                                end else begin
                                    o_dfi_rddata[dfi_pack_phase*DFI_DATA_WIDTH + dfi_pack_lane*DQ_BITS + dfi_pack_bit]
                                        <= ontime_shadow[dfi_pack_idx][2*dfi_pack_phase];
                                    o_dfi_rddata[dfi_pack_phase*DFI_DATA_WIDTH + TOTAL_DQ + dfi_pack_lane*DQ_BITS + dfi_pack_bit]
                                        <= ontime_shadow[dfi_pack_idx][2*dfi_pack_phase + 1];
                                end
                            end
                        end
                    end
                end
            end else begin
                // No late lanes: direct capture at rddata_en (original path)
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
            end

            // rddata_valid: asserts when ALL lanes have valid data captured.
            // If any lane is late, valid follows rddata_en_d1 (1 cycle later).
            // Controller pipe_stall mechanism tolerates the extra cycle.
            o_dfi_rddata_valid <= (|rd_lat_extra) ? rddata_en_d1 : i_dfi_rddata_en;

            // ---------------------------------------------------------
            // PHY Training FSM
            //
            // Three-phase training sequence controlled by the memory
            // controller via DFI training interface signals:
            //
            // Phase 1 - GATE TRAINING (rdlvl_gate_en):
            //   No-op. Eye training subsumes gate training by searching
            //   all 9 offsets at every tap. Responds immediately.
            //
            // Phase 2 - EYE TRAINING (rdlvl_en):
            //   Phase-aware: sweeps IDELAYE3 taps 0->508, at each tap
            //   searches iserdes_window for MPR_PATTERN at offsets 0-8.
            //   Tracks the widest contiguous range of taps with same
            //   offset. Centers IDELAY at best range midpoint and sets
            //   bitslip to the offset found there.
            //
            // Phase 3 - WRITE LEVELING (wrlvl_en):
            //   Per JESD79-4D sec 4.7: sweeps DQS ODELAYE3 until the DRAM
            //   reports a 0->1 transition on DQ (indicating DQS rising edge
            //   is now aligned with CK rising edge). DQ ODELAY tracks DQS
            //   to maintain the 90 deg write data-to-strobe offset.
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
                        if (i_dfi_rdlvl_gate_en) begin
                            phy_state <= PHY_GATE_DONE;
                        end else if (i_dfi_rdlvl_en) begin // Eye training starts on rdlvl_en
                            en_vtc_q <= 1'b0;
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
                            phy_timer <= 4'd4;
                            phy_state <= PHY_EYE_SWEEP;
                        end else if (i_dfi_wrlvl_en) begin // Write leveling starts on wrlvl_en
                            en_vtc_q <= 1'b0;
                            o_dfi_wrlvl_resp <= {BYTE_LANES{1'b0}};
                            train_lane <= 0;
                            wl_train_fail <= {BYTE_LANES{1'b0}};
                            odelay_dqs_cntvalue <= odelay_dqs_cntvalueout[0];
                            odelay_dq_cntvalue  <= 9'd0;
                            // DQS ODELAY was initialized to tCK/4 (90 deg DQS-to-DQ
                            // centering). IODELAY BISC converts that ps value to taps.
                            // Read back actual tap via CNTVALUEOUT so WL sweeps
                            // from the calibrated 90 deg baseline, finding the
                            // additional delay for DQS-to-CK alignment at DRAM.
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

                    // -- Gate training: no-op (eye training subsumes it) --
                    // Eye training searches all 9 offsets at every IDELAYE3 tap,
                    // which IS the bitslip search. Gate training's output would
                    // be overwritten by eye training anyway, so we skip it entirely.
                    // DFI handshake: assert resp immediately, deassert when MC drops enable.
                    // Per DFI 3.1 Fig.57: resp deasserts when enable drops.
                    PHY_GATE_DONE: begin
                        o_dfi_rdlvl_resp <= {BYTE_LANES{1'b1}}; // assert completion to MC
                        if (!i_dfi_rdlvl_gate_en) begin
                            o_dfi_rdlvl_resp <= {BYTE_LANES{1'b0}}; // deassert per DFI handshake
                            phy_state <= PHY_IDLE;
                        end
                    end

                    // -- Eye training: phase-aware IDELAYE3 sweep ------
                    // Settle IDELAYE3 at current tap, then register the
                    // combinational pattern search output (pipeline stage 1).
                    // Timer sequence: 4(idle) -> 3(LOAD pulse) -> 2,1(settle) -> 0(sample).
                    // At timer=0 we wait for rddata_en which indicates DRAM is
                    // actively driving MPR data in response to a controller READ.
                    PHY_EYE_SWEEP: begin
                        if (phy_timer != 0) begin
                            if (phy_timer == 4'd3)
                                idelay_load_lane[train_lane] <= 1'b1;
                            phy_timer <= phy_timer - 1'b1;
                        end else if (|i_dfi_rddata_en) begin
                            if (pattern_found_comb) begin
                                // Pattern found on the rddata_en cycle (on-time arrival)
                                pattern_found_q <= 1'b1;
                                pattern_offset_q <= pattern_offset_comb;
                                pattern_late_q <= 1'b0;
                                phy_state <= PHY_EYE_TRACK;
                            end else begin
                                // Not found - IDELAY may have pushed data to next CLKDIV cycle.
                                // Wait 1 more cycle and re-check (PHY_EYE_LATE).
                                phy_state <= PHY_EYE_LATE;
                            end
                        end
                    end

                    // Range tracking (pipeline stage 2): use registered comparator
                    // results to maintain the current and best ranges. Then advance
                    // the sweep tap or finish if we've reached the end (tap 508).
                    //
                    // Range identity = (offset, late). A range closes when either changes.
                    // Three cases per tap:
                    //   1. Pattern found, same (offset,late) as current range -> extend
                    //   2. Pattern found, different (offset,late) -> close current, open new
                    //   3. Pattern not found -> close current range (edge/metastable zone)
                    //
                    // On close: if current range is wider than best, promote it.
                    // After full sweep, the widest stable region is in best_*.
                    PHY_EYE_TRACK: begin
                        if (pattern_found_q) begin
                            if (!in_range) begin
                                // Open a new range at this tap
                                cur_start <= sweep_tap;
                                cur_width <= 9'd0;
                                cur_offset <= pattern_offset_q;
                                cur_late <= pattern_late_q;
                                in_range <= 1'b1;
                            end else if (pattern_offset_q == cur_offset && pattern_late_q == cur_late) begin
                                // Same offset AND same latency - extend current range
                                cur_width <= cur_width + {5'd0, TAP_SWEEP_STEP};
                            end else begin
                                // Offset or latency changed - close current range, open new.
                                // This happens when IDELAYE3 pushes DQ past a clock edge
                                // or past a full CLKDIV boundary (on-time -> late transition).
                                if (!best_valid || cur_width > best_width) begin
                                    best_start <= cur_start;
                                    best_width <= cur_width;
                                    best_offset <= cur_offset;
                                    best_late <= cur_late;
                                    best_valid <= 1'b1;
                                end
                                cur_start <= sweep_tap;
                                cur_width <= 9'd0;
                                cur_offset <= pattern_offset_q;
                                cur_late <= pattern_late_q;
                            end
                        end else begin
                            // No pattern found (metastable/edge zone) - close range
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
                        // Advance sweep or transition to decision
                        if (sweep_tap == 9'd508) begin
                            phy_state <= PHY_EYE_DECIDE;
                        end else begin
                            sweep_tap <= sweep_tap + {5'd0, TAP_SWEEP_STEP};
                            idelay_cntvalue <= sweep_tap + {5'd0, TAP_SWEEP_STEP};
                            phy_timer <= 4'd4;
                            phy_state <= PHY_EYE_SWEEP;
                        end
                    end

                    // Decision: close any still-open range (if sweep ended mid-range),
                    // then select the widest range and compute center tap.
                    // Takes 2 cycles if a range was open (close on cycle 1, decide on cycle 2).
                    // Sets bitslip_count_q to best_offset - this is the byte boundary
                    // position at the chosen IDELAYE3 tap (overrides any prior value).
                    PHY_EYE_DECIDE: begin
                        if (in_range) begin
                            // Final close: range was still open at end of sweep
                            if (!best_valid || cur_width > best_width) begin
                                best_start <= cur_start;
                                best_width <= cur_width;
                                best_offset <= cur_offset;
                                best_late <= cur_late;
                                best_valid <= 1'b1;
                            end
                            in_range <= 1'b0;
                        end else if (!best_valid) begin
                            // No valid range found at any tap - lane is broken
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
                            // Load center of widest range into IDELAYE3.
                            // Set bitslip to the offset where MPR was found in that range -
                            // this is the correct byte boundary for all subsequent reads.
                            // rd_lat_extra: if best range was "late", normal reads arrive
                            // 1 CLKDIV cycle after rddata_en - capture path uses rddata_en_d1.
                            idelay_cntvalue <= best_start + (best_width >> 1);
                            eye_center_tap[train_lane] <= best_start + (best_width >> 1);
                            bitslip_count_q[train_lane] <= best_offset;
                            rd_lat_extra[train_lane] <= best_late;
                            eye_best_width[train_lane] <= best_width;
                            eye_best_start[train_lane] <= best_start;
                            phy_timer <= 4'd4;
                            phy_state <= PHY_EYE_VERIFY;
                            `ifndef YOSYS
                                $display("[%0t] PHY eye: lane %0d best_start=%0d width=%0d center=%0d offset=%0d late=%0d",
                                    $realtime, train_lane, best_start, best_width,
                                    best_start + (best_width >> 1), best_offset, best_late);
                            `endif
                        end
                    end

                    // Re-read MPR at the center tap to confirm the chosen
                    // bitslip + IDELAYE3 combination actually produces correct data.
                    // Uses aligned_dq (barrel shifter output with new bitslip) so
                    // this validates the exact path used during normal operation.
                    // For late lanes, data arrives 1 cycle after rddata_en so we
                    // transition to PHY_EYE_LATE with verify_mode=1.
                    // On success: advance to next lane or finish.
                    // On failure: latch eye_train_fail, proceed anyway.
                    PHY_EYE_VERIFY: begin
                        if (phy_timer != 0) begin
                            if (phy_timer == 4'd3)
                                idelay_load_lane[train_lane] <= 1'b1;
                            phy_timer <= phy_timer - 1'b1;
                        end else if (|i_dfi_rddata_en) begin
                            if (rd_lat_extra[train_lane]) begin
                                // Late lane: data arrives next cycle, defer check
                                verify_mode <= 1'b1;
                                phy_state <= PHY_EYE_LATE;
                            end else if (aligned_dq[train_lane * DQ_BITS] === MPR_PATTERN) begin
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
                                    $display("[%0t] PHY eye: lane %0d verify FAILED at center tap", $realtime, train_lane);
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

                    // Late-arrival check (dual-purpose state):
                    // verify_mode=0: Sweep late-check. Sample train_window one CLKDIV
                    //   cycle after rddata_en. At high IDELAY taps, BL8 burst crosses
                    //   the CLKDIV boundary and arrives in the next cycle.
                    // verify_mode=1: Verify late-check. Confirm aligned_dq matches MPR
                    //   pattern at the center tap for a lane whose best eye is "late".
                    PHY_EYE_LATE: begin
                        if (!verify_mode) begin
                            // Sweep late-check: sample comparator result, proceed to TRACK
                            pattern_found_q <= pattern_found_comb;
                            pattern_offset_q <= pattern_offset_comb;
                            pattern_late_q <= pattern_found_comb;
                            phy_state <= PHY_EYE_TRACK;
                        end else begin
                            // Verify late-check: confirm aligned_dq matches MPR
                            verify_mode <= 1'b0;
                            if (aligned_dq[train_lane * DQ_BITS] === MPR_PATTERN) begin
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

                    // Signal eye training complete to controller.
                    // Holds resp high until controller deasserts rdlvl_en.
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
                            if (phy_timer == 4'd3) begin
                                odelay_dqs_load[train_lane] <= 1'b1;
                                odelay_dq_load[train_lane]  <= 1'b1;
                            end
                            phy_timer <= phy_timer - 1'b1;
                        end else if (i_dfi_wrlvl_strobe) begin // Wait for controller strobe
                            wl_dqs_strobe <= 1'b1;
                            phy_timer <= 4'd15; // set to maximum
                            phy_state <= PHY_WL_ADJUST;
                        end
                    end

                    // WL response evaluation.
                    //
                    // JESD79-4D write leveling requires an actual DQS-sampled
                    // CK 0-to-1 response; accepting the initial high level is
                    // unsafe.  Once that transition is observed, its absolute
                    // ODELAY count may include one complete tCK if the initial
                    // 90-degree baseline happened to be aligned.  The BISC-
                    // calibrated initial DQS delay is tCK/4, so 4*initial_tap
                    // gives this lane's calibrated tCK estimate.  Removing one
                    // such whole period from DQS and DQ preserves their pin
                    // phase and avoids a one-clock write-data burst shift.
                    PHY_WL_ADJUST: begin
                        if (phy_timer != 0) begin
                            phy_timer <= phy_timer - 1'b1;
                        end else if (!(wl_feedback_zero || wl_feedback_one)) begin
                            // The response changed within this 8-UI capture.
                            // Leave the delay untouched and ask for a fresh,
                            // fully-settled response at the same tap.
                            phy_timer <= 4'd4;
                            phy_state <= PHY_WL_SAMPLE;
                        end else begin
                            if (wl_feedback_zero)
                                wl_seen_zero[train_lane] <= 1'b1;

                            if (wl_seen_zero[train_lane] && wl_feedback_one) begin
                                wl_tap[train_lane] <= wl_final_dqs_tap;
                                wl_dq_tap[train_lane] <= wl_final_dqs_tap - dqs_initial_tap[train_lane];
                                odelay_dqs_cntvalue <= wl_final_dqs_tap;
                                odelay_dq_cntvalue <= wl_final_dqs_tap - dqs_initial_tap[train_lane];
                                phy_timer <= 4'd4;
                                phy_state <= PHY_WL_APPLY;
                                `ifndef YOSYS
                                    $display("[%0t] PHY WL phase: lane %0d edge=%0d estimated_tCK=%0d baseline=%0d final_dqs=%0d final_dq=%0d",
                                        $realtime, train_lane, wl_tap[train_lane], wl_period_estimate,
                                        dqs_initial_tap[train_lane], wl_final_dqs_tap,
                                        wl_final_dqs_tap - dqs_initial_tap[train_lane]);
                                `endif
                            end else if (wl_tap[train_lane][8:2] == 7'b1111111) begin
                                // No low was ever observed, so the entire
                                // programmable range was inside one high
                                // feedback interval and no new 0-to-1 edge was
                                // reachable. Do not fail solely for that range
                                // limit, but restore the pre-WL 90-degree DQS
                                // baseline and zero DQ delay before continuing.
                                // Include the current sample: wl_seen_zero is
                                // updated non-blockingly, so a low seen on this
                                // final tap must not be mistaken for all-high.
                                if (!wl_seen_zero[train_lane] && !wl_feedback_zero) begin
                                    wl_tap[train_lane] <= dqs_initial_tap[train_lane];
                                    wl_dq_tap[train_lane] <= 9'd0;
                                    odelay_dqs_cntvalue <= dqs_initial_tap[train_lane];
                                    odelay_dq_cntvalue <= 9'd0;
                                    phy_timer <= 4'd4;
                                    phy_state <= PHY_WL_APPLY;
                                    `ifndef YOSYS
                                        $display("[%0t] PHY WL fallback: lane %0d had no 0-to-1 edge; restoring DQS baseline tap %0d", $realtime, train_lane, dqs_initial_tap[train_lane]);
                                    `endif
                                end else begin
                                    // A low response was observed but it never
                                    // returned high. This cannot establish WL
                                    // phase, so fail the lane but first restore
                                    // its pre-WL DQS baseline and zero DQ delay.
                                    wl_train_fail[train_lane] <= 1'b1;
                                    wl_tap[train_lane] <= dqs_initial_tap[train_lane];
                                    wl_dq_tap[train_lane] <= 9'd0;
                                    odelay_dqs_cntvalue <= dqs_initial_tap[train_lane];
                                    odelay_dq_cntvalue <= 9'd0;
                                    phy_timer <= 4'd4;
                                    phy_state <= PHY_WL_APPLY;
                                    `ifndef YOSYS
                                        $display("[%0t] PHY WL failed: lane %0d saw low but no 0-to-1 edge; restoring DQS baseline tap %0d", $realtime, train_lane, dqs_initial_tap[train_lane]);
                                    `endif
                                end
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

                    // Apply the normalized taps after CNTVALUEIN has been
                    // stable for a complete controller clock.
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

                    // Advance to next lane or finish write leveling.
                    // Re-enables VTC and waits for it to settle before
                    // signaling completion to the controller.
                    PHY_WL_CHECK: begin
                        `ifndef YOSYS
                            $display("[%0t] PHY WL: lane %0d dqs_tap=%0d dq_tap=%0d", $realtime, train_lane, wl_tap[train_lane], wl_dq_tap[train_lane]);
                        `endif
                        /* verilator lint_off WIDTHEXPAND */
                        if (train_lane < BYTE_LANES - 1) begin
                        /* verilator lint_on WIDTHEXPAND */
                            train_lane <= train_lane + 1'b1;
                            wl_tap[train_lane + 1'b1]    <= dqs_initial_tap[train_lane + 1'b1]; // resume from 90 deg baseline (tCK/4 tap set by IODELAY BISC)
                            wl_dq_tap[train_lane + 1'b1] <= 9'd0;              // DQ has no initial offset - tracks DQS delta after WL
                            wl_seen_zero[train_lane + 1'b1] <= 1'b0;
                            odelay_dqs_cntvalue <= dqs_initial_tap[train_lane + 1'b1]; // load DQS ODELAY to same 90 deg baseline
                            odelay_dq_cntvalue  <= 9'd0;                       // DQ ODELAY starts at 0, incremented in lockstep with DQS
                            phy_timer <= 4'd4;
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
    IDELAYCTRL #(
        .SIM_DEVICE("ULTRASCALE")
    )idelayctrl_inst (
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
