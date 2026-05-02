////////////////////////////////////////////////////////////////////////////////
//
// Filename: ddr4_controller.v
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  DDR4 SDRAM controller targeting Xilinx UltraScale+ FPGAs.
//  4:1 memory controller with DFI 3.1 PHY interface, 16-bank tracking
//  with bank group awareness, and Wishbone B4 pipelined host interface.
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

module ddr4_controller #(
    parameter CONTROLLER_CLK_PERIOD = 3_333, //ps, controller clock (300 MHz → DDR4-2400)
              DDR4_CLK_PERIOD = 833,          //ps, DDR4 memory clock (1200 MHz → DDR4-2400)
              ROW_BITS = 16,    //row address width (14–17, density dependent)
              COL_BITS = 10,    //column address width (10 for x8/x16, 10–11 for x4)
              BA_BITS = 2,      //bank address (always 2 for DDR4)
              BG_BITS = 2,      //bank group (2 for x4/x8, 1 for x16)
              DQ_BITS = 8,      //device data width
              BYTE_LANES = 2,   //number of byte lanes
              DENSITY = 8,      //device density in Gb (2, 4, 8, 16)
    parameter[0:0] MICRON_SIM = 0,   //shorten init delays for Micron model
                   SKIP_CALIB = 0,   //skip PHY calibration (sim only)
    parameter[1:0] ADDR_MAPPING = 1, //0={row,bg,ba,col}, 1=BG-interleaved (default)
    parameter[2:0] RTT_NOM  = 3'b001, //MR1 A10:A8 (001=RZQ/4)
                   RTT_WR   = 3'b000, //MR2 A11,A10:A9 (000=off)
                   RTT_PARK = 3'b000, //MR5 A8:A6 (000=off)
    parameter[0:0] DRIVE_IMP = 0,    //MR1 A2:A1 (0=RZQ/7, 1=RZQ/5)
    // Override CL/CWL: set nonzero for manual, 0 = auto from clock period
    parameter[5:0] CL = 0,
    parameter[4:0] CWL_PARAM = 0,
    // The next parameters act more like localparams but are here to simplify port declarations
    parameter SERDES_RATIO = 4,
              NUM_BG = (1 << BG_BITS),
              NUM_BANKS = NUM_BG * (1 << BA_BITS),
              DFI_DATA_WIDTH = 2 * DQ_BITS * BYTE_LANES, //per DFI phase
              WB_DATA_BITS = DQ_BITS * BYTE_LANES * 2 * SERDES_RATIO,
              WB_SEL_BITS = WB_DATA_BITS / 8,
              COL_LOW = $clog2(SERDES_RATIO * 2 * DQ_BITS * BYTE_LANES / 8),
              WB_ADDR_BITS = ROW_BITS + BG_BITS + BA_BITS + COL_BITS - COL_LOW,
              CMD_LEN = 29 //packed command word width
) (
    input wire i_controller_clk,
    input wire i_rst_n,
    // Wishbone B4 Pipelined Interface
    input wire                       i_wb_cyc,
    input wire                       i_wb_stb,
    input wire                       i_wb_we,
    input wire[WB_ADDR_BITS-1:0]     i_wb_addr,
    input wire[WB_DATA_BITS-1:0]     i_wb_data,
    input wire[WB_SEL_BITS-1:0]      i_wb_sel,
    output reg                       o_wb_stall,
    output wire                      o_wb_ack,
    output reg[WB_DATA_BITS-1:0]     o_wb_data,
    // DFI 3.1 Control (4 phases, packed flat)
    output reg[4*17-1:0]             o_dfi_address,
    output reg[4*BA_BITS-1:0]        o_dfi_bank,
    output reg[4*BG_BITS-1:0]        o_dfi_bg,
    output reg[3:0]                  o_dfi_cs_n,
    output reg[3:0]                  o_dfi_act_n,
    output reg[3:0]                  o_dfi_ras_n,
    output reg[3:0]                  o_dfi_cas_n,
    output reg[3:0]                  o_dfi_we_n,
    output reg[3:0]                  o_dfi_cke,
    output reg[3:0]                  o_dfi_odt,
    output reg[3:0]                  o_dfi_reset_n,
    // DFI Write Data
    output reg[4*DFI_DATA_WIDTH-1:0] o_dfi_wrdata,
    output reg[3:0]                  o_dfi_wrdata_en,
    output reg[4*(2*BYTE_LANES)-1:0] o_dfi_wrdata_mask,
    // DFI Read Data
    input wire[4*DFI_DATA_WIDTH-1:0] i_dfi_rddata,
    input wire[3:0]                  i_dfi_rddata_valid,
    output reg[3:0]                  o_dfi_rddata_en,
    // DFI Status
    output wire                      o_dfi_init_start,
    input wire                       i_dfi_init_complete,
    // DFI Training (MC → PHY)
    output reg                       o_dfi_rdlvl_en,
    output reg                       o_dfi_rdlvl_gate_en,
    output reg                       o_dfi_wrlvl_en,
    output reg                       o_dfi_wrlvl_strobe,
    output reg[3:0]                  o_dfi_lvl_pattern,
    output reg                       o_dfi_lvl_periodic,
    // DFI Training (PHY → MC)
    input wire[BYTE_LANES-1:0]       i_dfi_rdlvl_resp,
    input wire[BYTE_LANES-1:0]       i_dfi_wrlvl_resp,
    input wire                       i_dfi_rdlvl_req,
    input wire                       i_dfi_rdlvl_gate_req,
    input wire                       i_dfi_wrlvl_req,
    // Status
    output reg                       o_calib_complete,
    output reg                       o_calib_error
);

    // ═══════════════════════════════════════════════════════════════════
    // §2 — DDR4 Command Encoding
    // JEDEC JESD79-4D Table 35: {ACT_n, RAS_n/A16, CAS_n/A15, WE_n/A14}
    // ═══════════════════════════════════════════════════════════════════
    localparam[3:0] CMD_MRS  = 4'b1_000,
                    CMD_REF  = 4'b1_001,
                    CMD_PRE  = 4'b1_010,
                    CMD_ACT  = 4'b0_000, //ACT_n=0: {RAS,CAS,WE} carry row addr A16:A14
                    CMD_WR   = 4'b1_100,
                    CMD_RD   = 4'b1_101,
                    CMD_NOP  = 4'b1_111,
                    CMD_ZQCL = 4'b1_110,
                    CMD_DES  = 4'b1_111; //same as NOP, cs_n=1 makes it DES

    // ROM control field: {RST_DONE, USE_TIMER, A10, CKE, RESET_N}
    localparam[4:0] CTL_CKE0_RST0 = 5'b01000, //power-on: CKE=0, RESET_n=0
                    CTL_CKE0_RST1 = 5'b01001, //pre-CKE: CKE=0, RESET_n=1
                    CTL_TIMER     = 5'b01011, //normal: CKE=1, RESET_n=1
                    CTL_TIMER_A10 = 5'b01111, //+ A10=1: PRE ALL, ZQCL
                    CTL_DONE      = 5'b11011; //RST_DONE: init complete

    // ROM instruction bit fields (32 bits)
    localparam ROM_RST_DONE  = 31,
               ROM_USE_TIMER = 30,
               ROM_A10       = 29,
               ROM_CKE       = 28,
               ROM_RESET_N   = 27;
    //bits [26:23] = CMD, [22:20] = MRS_SELECT, [19:0] = timer/addr

    // Named ROM address constants
    localparam[5:0] ROM_ADDR_RD_CAL    = 22,
                    ROM_ADDR_WL_CAL    = 27,
                    ROM_ADDR_NORMAL    = 32,
                    ROM_ADDR_REF_START = 33,
                    ROM_ADDR_REF_END   = 35;

    // MRS select — {BG0, BA1, BA0}
    localparam[2:0] MRS_MR0 = 3'b000, MRS_MR1 = 3'b001, MRS_MR2 = 3'b010,
                    MRS_MR3 = 3'b011, MRS_MR4 = 3'b100, MRS_MR5 = 3'b101,
                    MRS_MR6 = 3'b110;

    // Calibration window delay — large enough for training FSM
    localparam integer CALIBRATION_DELAY = 1000;

    // DFI 3.1 training pump timing (SPEC §9.3.2)
    localparam T_RDLVL_EN   = 4;    // min DFI clks: rdlvl_en → first READ
    localparam T_RDLVL_RR   = 16;   // min DFI clks between training READs
    localparam T_RDLVL_MAX  = 4096; // timeout for rdlvl_resp
    localparam T_WRLVL_EN   = 4;    // min DFI clks: wrlvl_en → first strobe
    localparam T_WRLVL_WW   = 32;   // min DFI clks between strobe pulses
    localparam T_WRLVL_MAX  = 4096; // timeout for wrlvl_resp
    localparam CALIB_RETRY_MAX = 3;

    // Training command pump states (SPEC §9.2)
    localparam[3:0] CALIB_IDLE       = 4'd0,
                    CALIB_GATE_EN    = 4'd1,
                    CALIB_GATE_READ  = 4'd2,
                    CALIB_GATE_WAIT  = 4'd3,
                    CALIB_GATE_EXIT  = 4'd4,
                    CALIB_EYE_EN     = 4'd5,
                    CALIB_EYE_READ   = 4'd6,
                    CALIB_EYE_WAIT   = 4'd7,
                    CALIB_EYE_EXIT   = 4'd8,
                    CALIB_WL_EN      = 4'd9,
                    CALIB_WL_STROBE  = 4'd10,
                    CALIB_WL_WAIT    = 4'd11,
                    CALIB_WL_EXIT    = 4'd12,
                    CALIB_DONE       = 4'd13,
                    CALIB_ERROR      = 4'd14;

    // Packed command word bit-field positions (29 bits, see PLAN §6.4)
    localparam CMD_CS_N     = 28,
               CMD_ACT_N    = 27,
               CMD_RAS_N    = 26, //or A16 when ACT_n=0
               CMD_CAS_N    = 25, //or A15 when ACT_n=0
               CMD_WE_N     = 24, //or A14 when ACT_n=0
               CMD_ODT      = 23,
               CMD_CKE      = 22,
               CMD_RESET_N  = 21,
               CMD_BG_START = 20, //bg[1:0] at [20:19]
               CMD_BA_START = 18; //ba[1:0] at [18:17]
               //addr[16:0] at [16:0]

    /************************************************************
     * §3 — DDR4 Timing Parameters
     * JEDEC JESD79-4D Tables 172–173
     * All ps values use worst-case grade per speed bin
     ************************************************************/
    localparam integer CL_nCK  = CL_generator(DDR4_CLK_PERIOD);
    localparam integer CWL_nCK = CWL_generator(DDR4_CLK_PERIOD);
    localparam integer WR_nCK = ps_to_nCK(15_000); //tWR always 15ns

    // Core timing
    localparam tRAS_ps = (DDR4_CLK_PERIOD >= 1_250) ? 35_000 : //DDR4-1600
                         (DDR4_CLK_PERIOD >= 1_071) ? 34_000 : //DDR4-1866
                         (DDR4_CLK_PERIOD >= 937)   ? 33_000 : //DDR4-2133
                                                      32_000;  //DDR4-2400+

    // tRCD/tRP: worst-case speed grade per bin (JESD79-4D Table 173)
    localparam tRCD_ps = (DDR4_CLK_PERIOD >= 1_250) ? 13_750 : //DDR4-1600 (L grade)
                         (DDR4_CLK_PERIOD >= 937)   ? 13_130 : //DDR4-1866/2133 (P grade)
                         (DDR4_CLK_PERIOD >= 833)   ? 14_160 : //DDR4-2400 (U grade)
                         (DDR4_CLK_PERIOD >= 750)   ? 13_750 : //DDR4-2666 (T grade)
                                                      13_750;  //DDR4-3200

    localparam tRP_ps  = tRCD_ps; //symmetric for standard grades
    localparam tRC_ps  = tRAS_ps + tRP_ps;
    localparam tWR_ps  = 15_000; //JESD79-4D §4.30 — always 15ns for DDR4
    localparam tRTP_ps = max_fn(DDR4_CLK_PERIOD * 4, 7_500); //JESD79-4D §4.28 — max(4nCK, 7.5ns)

    // Bank-group-dependent timing — the key DDR4 addition
    localparam tCCD_L_ps = max_fn(DDR4_CLK_PERIOD * 5,
        (DDR4_CLK_PERIOD >= 1_250) ? 6_250 :  //DDR4-1600
        (DDR4_CLK_PERIOD >= 937)   ? 5_355 :  //DDR4-1866/2133
                                     5_000);   //DDR4-2400+
    localparam tCCD_S_nCK = 4; //JESD79-4D Table 169 — always 4nCK across BGs
    localparam tWTR_L_ps = max_fn(DDR4_CLK_PERIOD * 4, 7_500);
    localparam tWTR_S_ps = max_fn(DDR4_CLK_PERIOD * 2, 2_500);

    // Page size → tRRD/tFAW category (JESD79-4D Table 167)
    localparam PAGE_SIZE = (1 << COL_BITS) * DQ_BITS / 8; //bytes

    localparam tRRD_L_ps = max_fn(DDR4_CLK_PERIOD * 4,
        (PAGE_SIZE >= 2048) ?
            ((DDR4_CLK_PERIOD >= 1_250) ? 7_500 : 6_400) :  //2KB page
        (DDR4_CLK_PERIOD >= 1_250) ? 6_000 :  //1KB, DDR4-1600
        (DDR4_CLK_PERIOD >= 937)   ? 5_300 :  //1KB, DDR4-1866/2133
                                     4_900);   //1KB, DDR4-2400+

    localparam tRRD_S_ps = max_fn(DDR4_CLK_PERIOD * 4,
        (PAGE_SIZE >= 2048) ?
            ((DDR4_CLK_PERIOD >= 1_250) ? 6_000 : 5_300) :  //2KB
        (DDR4_CLK_PERIOD >= 1_250) ? 5_000 :
        (DDR4_CLK_PERIOD >= 1_071) ? 4_200 :
        (DDR4_CLK_PERIOD >= 937)   ? 3_700 :
                                     3_300);

    localparam tFAW_nCK_min = (PAGE_SIZE >= 2048) ? 28 : 20;
    localparam tFAW_ps = max_fn(DDR4_CLK_PERIOD * tFAW_nCK_min,
        (PAGE_SIZE >= 2048) ?
            ((DDR4_CLK_PERIOD >= 1_250) ? 35_000 : 30_000) :
            ((DDR4_CLK_PERIOD >= 1_250) ? 25_000 :
             (DDR4_CLK_PERIOD >= 1_071) ? 23_000 : 21_000));

    // MRS / init timing (JESD79-4D Table 3)
    localparam tMRD_nCK    = 8;  //JESD79-4D Table 3 — 8nCK all speed bins
    localparam tMOD_ps     = max_fn(DDR4_CLK_PERIOD * 24, 15_000); //JESD79-4D Table 3 — max(24nCK, 15ns)
    localparam tZQinit_nCK = 1024; //JESD79-4D §4.18, Table 135
    localparam tZQoper_nCK = 512;  //JESD79-4D §4.18, Table 135
    localparam tZQCS_nCK   = 128;  //JESD79-4D §4.18, Table 135

    // DLL lock — JESD79-4D §4.21, Table 141
    localparam tDLLK_nCK = (DDR4_CLK_PERIOD >= 1_071) ? 597 :
                           (DDR4_CLK_PERIOD >= 833)   ? 768 : 1024;

    // Refresh (density dependent, tRFC1 — JESD79-4D Table 131)
    localparam tRFC_ps  = (DENSITY == 16) ? 550_000 :
                          (DENSITY == 8)  ? 350_000 :
                          (DENSITY == 4)  ? 260_000 :
                                            160_000;
    localparam tREFI_ps = 7_800_000; //JESD79-4D §4.15 — 7.8µs at ≤85°C

    // Write leveling — JESD79-4D §4.26, Table 157
    localparam tWLMRD_nCK   = 40;
    localparam tWLDQSEN_nCK = 25;

    // Init — JESD79-4D §3.3 Figure 7 (shortened for sim when MICRON_SIM=1)
    localparam POWER_ON_RESET_HIGH_ps = MICRON_SIM ? 10_000 : 200_000_000; //tPW_RESET ≥200µs
    localparam INITIAL_CKE_LOW_ps     = MICRON_SIM ? 10_000 : 500_000_000; //≥500µs after RESET_n deassert
    localparam tXPR_ps = max_fn(5 * DDR4_CLK_PERIOD, tRFC_ps + 10_000);

    // ═══════════════════════════════════════
    // §4 — Command Slot Assignment
    // DFI 3.1 §3.2 — slot assignment for 4-phase command interface
    // ═══════════════════════════════════════
    localparam integer READ_SLOT      = get_slot(CMD_RD);
    localparam integer WRITE_SLOT     = get_slot(CMD_WR);
    localparam integer ACTIVATE_SLOT  = get_slot(CMD_ACT);
    localparam integer PRECHARGE_SLOT = get_slot(CMD_PRE);

    // ═══════════════════════════════════════════════════════════════
    // §5 — Computed Delay Counters (controller clock cycles)
    // Derived from JESD79-4D timing params via find_delay()
    // Each value = minimum controller cycles to wait between commands
    // ═══════════════════════════════════════════════════════════════

    // Per-bank delays
    localparam ACTIVATE_TO_READWRITE_DELAY =
        find_delay(ps_to_nCK(tRCD_ps), ACTIVATE_SLOT, (CL_nCK > CWL_nCK) ? READ_SLOT : WRITE_SLOT);
    localparam READ_TO_PRECHARGE_DELAY =
        find_delay(max_fn(4, ps_to_nCK(tRTP_ps)), READ_SLOT, PRECHARGE_SLOT);
    localparam WRITE_TO_PRECHARGE_DELAY =
        find_delay(CWL_nCK + 4 + ps_to_nCK(tWR_ps), WRITE_SLOT, PRECHARGE_SLOT);
    localparam PRECHARGE_TO_ACTIVATE_DELAY =
        find_delay(ps_to_nCK(tRP_ps), PRECHARGE_SLOT, ACTIVATE_SLOT);
    localparam ACTIVATE_TO_PRECHARGE_DELAY =
        find_delay(ps_to_nCK(tRAS_ps), ACTIVATE_SLOT, PRECHARGE_SLOT);
    // read-to-write turnaround — JESD79-4D §4.12: CL + BL/2 + tRPST - CWL
    localparam READ_TO_WRITE_DELAY =
        find_delay(CL_nCK + 4 + 2 - CWL_nCK, READ_SLOT, WRITE_SLOT);

    // Bank-group-dependent delays (new for DDR4)
    localparam CAS_TO_CAS_DELAY_SAME_BG =
        find_delay(ps_to_nCK(tCCD_L_ps), READ_SLOT, READ_SLOT);
    localparam CAS_TO_CAS_DELAY_DIFF_BG =
        find_delay(tCCD_S_nCK, READ_SLOT, READ_SLOT);
    localparam WRITE_TO_READ_DELAY_SAME_BG =
        find_delay(CWL_nCK + 4 + ps_to_nCK(tWTR_L_ps), WRITE_SLOT, READ_SLOT);
    localparam WRITE_TO_READ_DELAY_DIFF_BG =
        find_delay(CWL_nCK + 4 + ps_to_nCK(tWTR_S_ps), WRITE_SLOT, READ_SLOT);
    localparam ACTIVATE_TO_ACTIVATE_DELAY_SAME_BG =
        find_delay(ps_to_nCK(tRRD_L_ps), ACTIVATE_SLOT, ACTIVATE_SLOT);
    localparam ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG =
        find_delay(ps_to_nCK(tRRD_S_ps), ACTIVATE_SLOT, ACTIVATE_SLOT);

    // tFAW in controller cycles (tracked by sliding window)
    localparam TFAW_CYCLES = nCK_to_cycles(ps_to_nCK(tFAW_ps));

    // Counter width sizing
    localparam MAX_PRECHARGE_DELAY = max_fn(max_fn(ACTIVATE_TO_PRECHARGE_DELAY,
                                    WRITE_TO_PRECHARGE_DELAY), READ_TO_PRECHARGE_DELAY);
    localparam MAX_ACTIVATE_DELAY  = PRECHARGE_TO_ACTIVATE_DELAY;
    localparam MAX_WRITE_DELAY     = max_fn(ACTIVATE_TO_READWRITE_DELAY, READ_TO_WRITE_DELAY);
    localparam MAX_READ_DELAY      = ACTIVATE_TO_READWRITE_DELAY;
    localparam MAX_CCD_DELAY       = CAS_TO_CAS_DELAY_SAME_BG;
    localparam MAX_WTR_DELAY       = WRITE_TO_READ_DELAY_SAME_BG;
    localparam MAX_RRD_DELAY       = ACTIVATE_TO_ACTIVATE_DELAY_SAME_BG;

    // ── Read/Write data enable pipeline depths (SPEC §4.3, §5.3) ──
    // Controller cycles from READ command to dfi_rddata_en assertion.
    // +4 accounts for: OSERDESE3 cmd pipeline (absorbed by find_delay),
    // CL propagation, ISERDESE3 deserialization latency (+1 CLKDIV),
    // and prev_iserdes_q registration for bitslip window (+1 CLKDIV).
    localparam READ_DELAY = find_delay(CL_nCK, READ_SLOT, READ_SLOT);
    localparam RDDATA_EN_PIPE_WIDTH = READ_DELAY + 4;
    // Controller cycles from WRITE command to dfi_wrdata_en assertion
    localparam WRITE_DATA_DELAY = find_delay(CWL_nCK, WRITE_SLOT, WRITE_SLOT);

    // ROM delay counter width — enough for longest init timer
    localparam DELAY_COUNTER_WIDTH = 20;

    // Refresh loop timer — adjusted so REF-to-REF ≤ tREFI (JESD79-4D §4.15)
    // Loop: PRE(33) → REF(34) → idle(35) → PRE(33)
    // Period = (T_RP + 1) + (T_RFC + 1) + (T_REFI + 1) controller cycles
    //        = T_RP + T_RFC + T_REFI + 3
    // Using floor(tREFI/CTRL_CLK) guarantees period ≤ tREFI for all configs.
    localparam REFRESH_TREFI_TIMER = tREFI_ps / CONTROLLER_CLK_PERIOD
                                     - 3 - ps_to_cycles(tRP_ps) - ps_to_cycles(tRFC_ps);

    // ════════════════════════════════════════════════════
    // §6 — Mode Register Construction
    // JEDEC JESD79-4D Tables 13–31, Appendix B
    // ════════════════════════════════════════════════════
    localparam[4:0] cl_enc  = CL_encoding(CL_nCK[5:0]);
    localparam[2:0] cwl_enc = CWL_encoding(CWL_nCK[4:0]);
    localparam[3:0] wr_enc  = WR_RTP_encoding(WR_nCK);

    // MR0: BL8, CAS Latency, DLL Reset, Write Recovery (JESD79-4D Table 13)
    localparam[13:0] MR0 = {
        wr_enc[3],        //A13: WR/RTP bit 3
        cl_enc[4],        //A12: CL bit 4
        wr_enc[2:0],      //A11:A9: WR/RTP bits 2:0
        1'b1,             //A8: DLL Reset = yes (self-clearing)
        1'b0,             //A7: Test Mode = normal
        cl_enc[3:1],      //A6:A4: CL bits 3:1
        1'b0,             //A3: Read Burst Type = sequential
        cl_enc[0],        //A2: CL bit 0
        2'b00             //A1:A0: BL = BL8 fixed
    };

    // MR1: DLL, RTT_NOM, output driver, write leveling (JESD79-4D Table 16)
    localparam[13:0] MR1_WL_DIS = {
        1'b0,             //A13: reserved
        1'b0,             //A12: Qoff = enabled
        1'b0,             //A11: TDQS = disabled
        RTT_NOM,          //A10:A8
        1'b0,             //A7: Write Leveling = off
        2'b00,            //A6:A5: reserved
        2'b00,            //A4:A3: AL = 0
        1'b0, DRIVE_IMP,  //A2:A1: output driver
        1'b1              //A0: DLL = on
    };
    localparam[13:0] MR1_WL_EN = {
        1'b0, 1'b0, 1'b0,
        RTT_NOM,
        1'b1,             //A7: Write Leveling = on
        2'b00, 2'b00,
        1'b0, DRIVE_IMP,
        1'b1
    };

    // MR2: CWL, RTT_WR (JESD79-4D Table 19)
    localparam[13:0] MR2 = {
        1'b0,             //A13: reserved
        1'b0,             //A12: Write CRC = off
        RTT_WR,           //A11,A10:A9
        1'b0,             //A8: reserved
        2'b00,            //A7:A6: LP ASR = manual normal
        cwl_enc,          //A5:A3: CWL
        3'b000            //A2:A0: reserved
    };

    // MR3: MPR (JESD79-4D Table 22)
    localparam[13:0] MR3_MPR_DIS = 14'b00_00_000_0_0_0_00_00;
    localparam[13:0] MR3_MPR_EN  = 14'b00_00_000_0_0_0_01_00; //A2=1 (MPR enable)

    // MR4: preamble, temperature (JESD79-4D Table 26) — all defaults for V1
    localparam[13:0] MR4 = 14'b00_0_000_00_0_0_0_000;

    // MR5: DM, DBI, RTT_PARK (JESD79-4D Table 28)
    localparam[0:0] DM_ENABLED = (DQ_BITS != 4); //x4 has no DM_n
    localparam[13:0] MR5 = {
        1'b0,             //A13: reserved
        1'b0,             //A12: Read DBI = off
        1'b0,             //A11: Write DBI = off
        DM_ENABLED,       //A10: Data Mask
        1'b0,             //A9: reserved
        RTT_PARK,         //A8:A6
        3'b000,           //A5:A3: reserved
        3'b000            //A2:A0: CA Parity Latency = off
    };

    // MR6: VrefDQ, tCCD_L (JESD79-4D Table 31)
    localparam[2:0] tCCD_L_enc = (ps_to_nCK(tCCD_L_ps) <= 4) ? 3'b000 :
                                  (ps_to_nCK(tCCD_L_ps) == 5) ? 3'b001 :
                                  (ps_to_nCK(tCCD_L_ps) == 6) ? 3'b010 :
                                  (ps_to_nCK(tCCD_L_ps) == 7) ? 3'b011 : 3'b100;
    localparam[13:0] MR6 = {
        1'b0,             //A13: reserved
        tCCD_L_enc,       //A12:A10: tCCD_L encoding
        2'b00,            //A9:A8: reserved
        1'b0,             //A7: VrefDQ Training = off
        1'b0,             //A6: VrefDQ Range = Range 1
        6'b011001         //A5:A0: VrefDQ ≈ 76% (step 25, Range 1)
    };

    // ═══════════════════════════════════════
    // §7 — Address Mapping
    // ═══════════════════════════════════════
    // COL_LOW = burst-alignment bits removed from WB address (defined in params above)
    // WB_ADDR_BITS = ROW_BITS + BG_BITS + BA_BITS + COL_BITS - COL_LOW (defined above)
    //
    // ADDR_MAPPING=0: {row, bg, ba, col} — legacy sequential
    // ADDR_MAPPING=1: {row, ba, col_hi, bg, col_lo=0} — BG-interleaved (default)
    //   Sequential WB accesses hit different bank groups → exploit tCCD_S < tCCD_L (JESD79-4D §4.7)
    localparam COL_USED    = COL_BITS - COL_LOW; //column bits present in WB address
    localparam COL_HI_BITS = (ADDR_MAPPING == 1) ? (COL_USED - BG_BITS) : COL_USED;

    // ═══════════════════════════════════════════════════════════════════
    // §8 — Registers and Wires
    // ═══════════════════════════════════════════════════════════════════

    reg reset_done;

    // Per-bank delay counters (logic added in Phase 4)
    reg[$clog2(MAX_PRECHARGE_DELAY):0] delay_before_precharge_counter_q [NUM_BANKS-1:0];
    reg[$clog2(MAX_ACTIVATE_DELAY):0]  delay_before_activate_counter_q  [NUM_BANKS-1:0];
    reg[$clog2(MAX_WRITE_DELAY):0]     delay_before_write_counter_q     [NUM_BANKS-1:0];
    reg[$clog2(MAX_READ_DELAY):0]      delay_before_read_counter_q      [NUM_BANKS-1:0];
    reg[NUM_BANKS-1:0]                 bank_status_q; //0=idle, 1=active
    reg[ROW_BITS-1:0]                  bank_active_row_q [NUM_BANKS-1:0];

    // Per-bank-group delay counters (new for DDR4, logic added in Phase 4)
    reg[$clog2(MAX_CCD_DELAY):0] ccd_counter_q [NUM_BG-1:0];
    reg[$clog2(MAX_WTR_DELAY):0] wtr_counter_q [NUM_BG-1:0];
    reg[$clog2(MAX_RRD_DELAY):0] rrd_counter_q [NUM_BG-1:0];

    // Packed command slots (internal, decomposed to DFI in §12)
    reg[CMD_LEN-1:0] cmd_d [SERDES_RATIO-1:0];

    // ROM / init sequence
    reg[5:0] instruction_address;
    reg[DELAY_COUNTER_WIDTH-1:0] delay_counter;
    reg delay_counter_is_zero;
    reg pause_counter;

    // ── §14 Training pump state (logic in sequential block) ──
    reg [3:0] calib_state;
    reg [$clog2(T_RDLVL_MAX):0] calib_timer;
    reg [$clog2(T_WRLVL_WW):0]  calib_rr_timer;
    reg [1:0] calib_retry_count;
    reg calib_act_done;

    reg rom_cke_hold;
    reg rom_reset_n_hold;
    wire[31:0] rom_instruction;
    wire rom_cmd_is_mrs;
    wire rom_use_timer;

    assign rom_instruction = read_rom_instruction(instruction_address);
    assign rom_cmd_is_mrs  = (rom_instruction[26:23] == CMD_MRS);
    assign rom_use_timer   = rom_instruction[ROM_USE_TIMER];

    wire init_firing  = delay_counter_is_zero && !pause_counter;
    wire init_cke     = init_firing ? (rom_cmd_is_mrs ? 1'b1 : rom_instruction[ROM_CKE])     : rom_cke_hold;
    wire init_reset_n = init_firing ? (rom_cmd_is_mrs ? 1'b1 : rom_instruction[ROM_RESET_N]) : rom_reset_n_hold;

    // ── §13 Read/Write data enable pipelines (SPEC §4.4, §5.3) ──
    reg[RDDATA_EN_PIPE_WIDTH-1:0] rddata_en_pipe_q;
    reg[WRITE_DATA_DELAY:0]       wrdata_en_pipe_q;
    reg                           write_ack_q;
    reg                           read_ack_q;

    // ── Static outputs ──
    assign o_wb_ack = i_rst_n && reset_done && (write_ack_q || read_ack_q);
    assign o_dfi_init_start = ~reset_done; // request PHY init until ROM completes

    // ═══════════════════════════════════════════════════════════════════
    // §7.5 — Address Decode
    // WB address → {row, bg, ba, col} per ADDR_MAPPING (PLAN §6.12)
    // ADDR_MAPPING=0: {row, bg, ba, col} — sequential
    // ADDR_MAPPING=1: {row, ba, col, bg} — BG-interleaved (default)
    // ═══════════════════════════════════════════════════════════════════
    wire[COL_BITS-1:0]        wb_col;
    wire[BA_BITS-1:0]         wb_ba;
    wire[BG_BITS-1:0]         wb_bg;
    wire[ROW_BITS-1:0]        wb_row;
    wire[BG_BITS+BA_BITS-1:0] wb_bank;
    wire[WB_ADDR_BITS-1:0]    wb_addr_next = i_wb_addr + 1'b1;
    wire[BG_BITS-1:0]         wb_next_bg;
    wire[BA_BITS-1:0]         wb_next_ba;
    wire[ROW_BITS-1:0]        wb_next_row;
    wire[BG_BITS+BA_BITS-1:0] wb_next_bank;

    generate
        if (ADDR_MAPPING == 0) begin : addr_map_0
            assign wb_col = {i_wb_addr[COL_USED-1:0], {COL_LOW{1'b0}}};
            assign wb_ba  = i_wb_addr[COL_USED +: BA_BITS];
            assign wb_bg  = i_wb_addr[COL_USED + BA_BITS +: BG_BITS];
            assign wb_row = i_wb_addr[COL_USED + BA_BITS + BG_BITS +: ROW_BITS];
            assign wb_next_bg  = wb_addr_next[COL_USED + BA_BITS +: BG_BITS];
            assign wb_next_ba  = wb_addr_next[COL_USED +: BA_BITS];
            assign wb_next_row = wb_addr_next[COL_USED + BA_BITS + BG_BITS +: ROW_BITS];
        end else begin : addr_map_1
            // BG at lowest position — sequential WB accesses cycle through bank groups,
            // exploiting tCCD_S < tCCD_L for streaming workloads (JESD79-4D §4.7)
            assign wb_bg  = i_wb_addr[BG_BITS-1:0];
            assign wb_col = {i_wb_addr[BG_BITS +: COL_USED], {COL_LOW{1'b0}}};
            assign wb_ba  = i_wb_addr[BG_BITS + COL_USED +: BA_BITS];
            assign wb_row = i_wb_addr[BG_BITS + COL_USED + BA_BITS +: ROW_BITS];
            assign wb_next_bg  = wb_addr_next[BG_BITS-1:0];
            assign wb_next_ba  = wb_addr_next[BG_BITS + COL_USED +: BA_BITS];
            assign wb_next_row = wb_addr_next[BG_BITS + COL_USED + BA_BITS +: ROW_BITS];
        end
    endgenerate
    assign wb_bank      = {wb_bg, wb_ba};
    assign wb_next_bank = {wb_next_bg, wb_next_ba};

    // ── Stage 1 pipeline registers ──
    reg                       stage1_pending;
    reg                       stage1_we;
    reg[WB_DATA_BITS-1:0]    stage1_data;
    reg[WB_SEL_BITS-1:0]     stage1_dm;
    reg[COL_BITS-1:0]        stage1_col;
    reg[BA_BITS-1:0]         stage1_ba;
    reg[BG_BITS-1:0]         stage1_bg;
    reg[ROW_BITS-1:0]        stage1_row;
    reg[BG_BITS+BA_BITS-1:0] stage1_bank;
    reg[BG_BITS+BA_BITS-1:0] stage1_next_bank; // anticipation: pre-ACT target
    reg[ROW_BITS-1:0]        stage1_next_row;

    // ── Stage 2 pipeline registers (scheduling logic added in Phase 4B) ──
    reg                       stage2_pending;
    reg                       stage2_we;
    reg[WB_DATA_BITS-1:0]    stage2_data;
    reg[WB_SEL_BITS-1:0]     stage2_dm;
    // Write data delay pipeline (SPEC §5.4) — mirrors wrdata_en_pipe_q
    // structure (same depth, shift direction, load position) so data and
    // enable stay aligned. Prevents stage2_data overwrite on back-to-back writes.
    reg[WB_DATA_BITS-1:0]    wr_data_pipe_q [WRITE_DATA_DELAY:0];
    reg[WB_SEL_BITS-1:0]     wr_dm_pipe_q   [WRITE_DATA_DELAY:0];
    reg[COL_BITS-1:0]        stage2_col;
    reg[BA_BITS-1:0]         stage2_ba;
    reg[BG_BITS-1:0]         stage2_bg;
    reg[ROW_BITS-1:0]        stage2_row;
    reg[BG_BITS+BA_BITS-1:0] stage2_bank;

    // ── tFAW sliding window (PLAN §6.5.3) — blocks 5th ACT within window ──
    reg[$clog2(TFAW_CYCLES):0] activate_timestamp_q [3:0];
    reg[1:0]                   activate_index_q;

    // ── Combinational next-state (decremented each cycle, loaded by scheduler in 4B) ──
    reg[$clog2(MAX_PRECHARGE_DELAY):0] delay_before_precharge_counter_d [NUM_BANKS-1:0];
    reg[$clog2(MAX_ACTIVATE_DELAY):0]  delay_before_activate_counter_d  [NUM_BANKS-1:0];
    reg[$clog2(MAX_WRITE_DELAY):0]     delay_before_write_counter_d     [NUM_BANKS-1:0];
    reg[$clog2(MAX_READ_DELAY):0]      delay_before_read_counter_d      [NUM_BANKS-1:0];
    reg[NUM_BANKS-1:0]                 bank_status_d;
    reg[ROW_BITS-1:0]                  bank_active_row_d [NUM_BANKS-1:0];
    reg[$clog2(MAX_CCD_DELAY):0]       ccd_counter_d [NUM_BG-1:0];
    reg[$clog2(MAX_WTR_DELAY):0]       wtr_counter_d [NUM_BG-1:0];
    reg[$clog2(MAX_RRD_DELAY):0]       rrd_counter_d [NUM_BG-1:0];
    reg[$clog2(TFAW_CYCLES):0]         activate_timestamp_d [3:0];

    // Scheduler runs during the tREFI idle window: instruction_address has
    // wrapped back to REF_START but the delay counter is still counting down.
    // During PRE ALL (addr 33 fire) and REF (addr 34), the scheduler is blocked.
    wire refresh_idle = (instruction_address == ROM_ADDR_REF_START)
                        && !delay_counter_is_zero;
    wire refresh_active = reset_done && !refresh_idle;

    // Detect ROM PRE ALL — clears all bank status (addrs 19, 30, 33)
    wire rom_firing = delay_counter_is_zero && !pause_counter
                      && (!reset_done || instruction_address >= ROM_ADDR_REF_START);
    wire rom_precharge_all = rom_firing && !rom_cmd_is_mrs
                             && (rom_instruction[26:23] == CMD_PRE)
                             && rom_instruction[ROM_A10];

    wire wb_accept = i_wb_cyc && i_wb_stb && !o_wb_stall;

    // ── Scheduler decision flags (set in §11c, used by sequential block) ──
    reg cmd_odt;
    reg stage2_update;
    reg sched_precharge;
    reg sched_activate;
    reg sched_write;
    reg sched_read;
    reg sched_anticipate;

    // tFAW: block 5th ACT if oldest timestamp hasn't expired
    wire tfaw_blocked = |activate_timestamp_q[activate_index_q];

    // Next-bank BG extraction for anticipation
    wire[BG_BITS-1:0] stage1_next_bg = stage1_next_bank[BG_BITS+BA_BITS-1:BA_BITS];

    // BG padding to 2 bits for cmd_d construction (PLAN §6.4: bg always [20:19])
    // Verilog zero-extends naturally: BG_BITS=2 → pass-through, BG_BITS=1 → {0, bg[0]}
    wire [1:0] stage2_bg_padded = stage2_bg;
    wire [1:0] stage1_next_bg_padded = stage1_next_bg;

    // Row padding to 17 bits for ACT command construction (SPEC §3.3)
    wire[16:0] stage2_row_padded     = {{(17-ROW_BITS){1'b0}}, stage2_row};
    wire[16:0] stage1_next_row_padded = {{(17-ROW_BITS){1'b0}}, stage1_next_row};

    // ═══════════════════════════════════════════════════════════════════
    // §11a — Combinational Counter Decrement
    // Saturating decrement: (|counter) is 1 when nonzero, 0 when zero.
    // Phase 4B: scheduler overrides _d values to load counters on issue.
    // ═══════════════════════════════════════════════════════════════════
    integer ci;
    always @* begin
        for (ci = 0; ci < NUM_BANKS; ci = ci + 1) begin
            delay_before_precharge_counter_d[ci] = delay_before_precharge_counter_q[ci]
                - (|delay_before_precharge_counter_q[ci]);
            delay_before_activate_counter_d[ci] = delay_before_activate_counter_q[ci]
                - (|delay_before_activate_counter_q[ci]);
            delay_before_write_counter_d[ci] = delay_before_write_counter_q[ci]
                - (|delay_before_write_counter_q[ci]);
            delay_before_read_counter_d[ci] = delay_before_read_counter_q[ci]
                - (|delay_before_read_counter_q[ci]);
            bank_status_d[ci] = bank_status_q[ci];
            bank_active_row_d[ci] = bank_active_row_q[ci];
        end
        for (ci = 0; ci < NUM_BG; ci = ci + 1) begin
            ccd_counter_d[ci] = ccd_counter_q[ci] - (|ccd_counter_q[ci]);
            wtr_counter_d[ci] = wtr_counter_q[ci] - (|wtr_counter_q[ci]);
            rrd_counter_d[ci] = rrd_counter_q[ci] - (|rrd_counter_q[ci]);
        end
        for (ci = 0; ci < 4; ci = ci + 1)
            activate_timestamp_d[ci] = activate_timestamp_q[ci]
                - (|activate_timestamp_q[ci]);

        // ROM PRE ALL closes all banks (refresh loop addr 33)
        if (rom_precharge_all) begin
            for (ci = 0; ci < NUM_BANKS; ci = ci + 1)
                bank_status_d[ci] = 1'b0;
        end

        // ═══════════════════════════════════════════════════════════════
        // §11c — Stage 2 Command Scheduling + Counter Loading
        // PRE→ACT→RD/WR with bank group awareness (PLAN §6.6, SPEC §2–3)
        // Uses counter_q <= 1 optimization (PLAN §6.6 stall path)
        // ═══════════════════════════════════════════════════════════════
        cmd_odt = 1'b0;
        stage2_update = !stage2_pending;
        sched_precharge = 1'b0;
        sched_activate  = 1'b0;
        sched_write     = 1'b0;
        sched_read      = 1'b0;
        sched_anticipate = 1'b0;

        if (stage2_pending && refresh_idle) begin

            // ── Bank active, wrong row → PRECHARGE (single bank) ──
            if (bank_status_q[stage2_bank]
                && (bank_active_row_q[stage2_bank] != stage2_row)
                && (delay_before_precharge_counter_q[stage2_bank] <= 1)) begin
                sched_precharge = 1'b1;
                delay_before_activate_counter_d[stage2_bank] =
                    PRECHARGE_TO_ACTIVATE_DELAY[$clog2(MAX_ACTIVATE_DELAY):0];
                bank_status_d[stage2_bank] = 1'b0;
            end

            // ── Bank idle → ACTIVATE ──
            else if (!bank_status_q[stage2_bank]
                     && (delay_before_activate_counter_q[stage2_bank] <= 1)
                     && (rrd_counter_q[stage2_bg] <= 1)
                     && !tfaw_blocked) begin
                sched_activate = 1'b1;
                // tRAS — minimum time bank must stay active (JESD79-4D §4.22)
                delay_before_precharge_counter_d[stage2_bank] =
                    ACTIVATE_TO_PRECHARGE_DELAY[$clog2(MAX_PRECHARGE_DELAY):0];
                // tRCD — only raise (protect lingering higher delay)
                if (delay_before_write_counter_d[stage2_bank]
                    < ACTIVATE_TO_READWRITE_DELAY[$clog2(MAX_WRITE_DELAY):0])
                    delay_before_write_counter_d[stage2_bank] =
                        ACTIVATE_TO_READWRITE_DELAY[$clog2(MAX_WRITE_DELAY):0];
                if (delay_before_read_counter_d[stage2_bank]
                    < ACTIVATE_TO_READWRITE_DELAY[$clog2(MAX_READ_DELAY):0])
                    delay_before_read_counter_d[stage2_bank] =
                        ACTIVATE_TO_READWRITE_DELAY[$clog2(MAX_READ_DELAY):0];
                bank_status_d[stage2_bank] = 1'b1;
                bank_active_row_d[stage2_bank] = stage2_row;
                // Per-BG tRRD: same BG = LONG, diff BG = SHORT only-raise (SPEC §2.4)
                for (ci = 0; ci < NUM_BG; ci = ci + 1) begin
                    if (ci[BG_BITS-1:0] == stage2_bg)
                        rrd_counter_d[ci] =
                            ACTIVATE_TO_ACTIVATE_DELAY_SAME_BG[$clog2(MAX_RRD_DELAY):0];
                    else if (rrd_counter_d[ci]
                             < ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_RRD_DELAY):0])
                        rrd_counter_d[ci] =
                            ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_RRD_DELAY):0];
                end
                // Per-bank activate counter: only-raise for all other banks
                // (BREAKDOWN counter loading table, belt-and-suspenders with rrd)
                for (ci = 0; ci < NUM_BANKS; ci = ci + 1) begin
                    if (ci[BG_BITS+BA_BITS-1:0] != stage2_bank
                        && delay_before_activate_counter_d[ci]
                           < ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_ACTIVATE_DELAY):0])
                        delay_before_activate_counter_d[ci] =
                            ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_ACTIVATE_DELAY):0];
                end
                // tFAW — record this activate's timestamp (PLAN §6.5.3)
                activate_timestamp_d[activate_index_q] =
                    TFAW_CYCLES[$clog2(TFAW_CYCLES):0];
            end

            // ── Bank active, correct row → WRITE or READ ──
            else if (bank_status_q[stage2_bank]
                     && (bank_active_row_q[stage2_bank] == stage2_row)) begin

                // WRITE — ODT on (SPEC §6.1)
                if (stage2_we
                    && (delay_before_write_counter_q[stage2_bank] <= 1)
                    && (ccd_counter_q[stage2_bg] <= 1)) begin
                    sched_write = 1'b1;
                    cmd_odt = 1'b1;
                    stage2_update = 1'b1;
                    // tWR — precharge: only raise to protect tRAS
                    if (delay_before_precharge_counter_d[stage2_bank]
                        < WRITE_TO_PRECHARGE_DELAY[$clog2(MAX_PRECHARGE_DELAY):0])
                        delay_before_precharge_counter_d[stage2_bank] =
                            WRITE_TO_PRECHARGE_DELAY[$clog2(MAX_PRECHARGE_DELAY):0];
                    // Per-BG tCCD + tWTR (SPEC §2.3)
                    for (ci = 0; ci < NUM_BG; ci = ci + 1) begin
                        if (ci[BG_BITS-1:0] == stage2_bg) begin
                            ccd_counter_d[ci] =
                                CAS_TO_CAS_DELAY_SAME_BG[$clog2(MAX_CCD_DELAY):0];
                            wtr_counter_d[ci] =
                                WRITE_TO_READ_DELAY_SAME_BG[$clog2(MAX_WTR_DELAY):0];
                        end else begin
                            if (ccd_counter_d[ci]
                                < CAS_TO_CAS_DELAY_DIFF_BG[$clog2(MAX_CCD_DELAY):0])
                                ccd_counter_d[ci] =
                                    CAS_TO_CAS_DELAY_DIFF_BG[$clog2(MAX_CCD_DELAY):0];
                            if (wtr_counter_d[ci]
                                < WRITE_TO_READ_DELAY_DIFF_BG[$clog2(MAX_WTR_DELAY):0])
                                wtr_counter_d[ci] =
                                    WRITE_TO_READ_DELAY_DIFF_BG[$clog2(MAX_WTR_DELAY):0];
                        end
                    end
                end

                // READ — ODT off (SPEC §6.1)
                else if (!stage2_we
                         && (delay_before_read_counter_q[stage2_bank] <= 1)
                         && (ccd_counter_q[stage2_bg] <= 1)
                         && (wtr_counter_q[stage2_bg] <= 1)) begin
                    sched_read = 1'b1;
                    stage2_update = 1'b1;
                    // tRTP — precharge: only raise
                    if (delay_before_precharge_counter_d[stage2_bank]
                        < READ_TO_PRECHARGE_DELAY[$clog2(MAX_PRECHARGE_DELAY):0])
                        delay_before_precharge_counter_d[stage2_bank] =
                            READ_TO_PRECHARGE_DELAY[$clog2(MAX_PRECHARGE_DELAY):0];
                    // RD→WR turnaround: all banks (global bus/ODT settling)
                    for (ci = 0; ci < NUM_BANKS; ci = ci + 1) begin
                        if (delay_before_write_counter_d[ci]
                            < READ_TO_WRITE_DELAY[$clog2(MAX_WRITE_DELAY):0])
                            delay_before_write_counter_d[ci] =
                                READ_TO_WRITE_DELAY[$clog2(MAX_WRITE_DELAY):0];
                    end
                    // Per-BG tCCD (SPEC §2.3)
                    for (ci = 0; ci < NUM_BG; ci = ci + 1) begin
                        if (ci[BG_BITS-1:0] == stage2_bg)
                            ccd_counter_d[ci] =
                                CAS_TO_CAS_DELAY_SAME_BG[$clog2(MAX_CCD_DELAY):0];
                        else if (ccd_counter_d[ci]
                                 < CAS_TO_CAS_DELAY_DIFF_BG[$clog2(MAX_CCD_DELAY):0])
                            ccd_counter_d[ci] =
                                CAS_TO_CAS_DELAY_DIFF_BG[$clog2(MAX_CCD_DELAY):0];
                    end
                end
            end
        end

        // ── Bank anticipation: pre-ACT next bank while Stage 2 issues WR/RD ──
        // Only fires when stage2_update (WR/RD completing or idle) and Stage 1
        // has a pending request whose next-bank is idle (PLAN §6.6)
        if (stage2_update && stage1_pending && refresh_idle
            && !bank_status_d[stage1_next_bank]
            && (delay_before_activate_counter_d[stage1_next_bank] == 0)
            && (rrd_counter_d[stage1_next_bg] == 0)
            && (activate_timestamp_d[activate_index_q] == 0)) begin
            sched_anticipate = 1'b1;
            delay_before_precharge_counter_d[stage1_next_bank] =
                ACTIVATE_TO_PRECHARGE_DELAY[$clog2(MAX_PRECHARGE_DELAY):0];
            if (delay_before_write_counter_d[stage1_next_bank]
                < ACTIVATE_TO_READWRITE_DELAY[$clog2(MAX_WRITE_DELAY):0])
                delay_before_write_counter_d[stage1_next_bank] =
                    ACTIVATE_TO_READWRITE_DELAY[$clog2(MAX_WRITE_DELAY):0];
            if (delay_before_read_counter_d[stage1_next_bank]
                < ACTIVATE_TO_READWRITE_DELAY[$clog2(MAX_READ_DELAY):0])
                delay_before_read_counter_d[stage1_next_bank] =
                    ACTIVATE_TO_READWRITE_DELAY[$clog2(MAX_READ_DELAY):0];
            bank_status_d[stage1_next_bank] = 1'b1;
            bank_active_row_d[stage1_next_bank] = stage1_next_row;
            for (ci = 0; ci < NUM_BG; ci = ci + 1) begin
                if (ci[BG_BITS-1:0] == stage1_next_bg)
                    rrd_counter_d[ci] =
                        ACTIVATE_TO_ACTIVATE_DELAY_SAME_BG[$clog2(MAX_RRD_DELAY):0];
                else if (rrd_counter_d[ci]
                         < ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_RRD_DELAY):0])
                    rrd_counter_d[ci] =
                        ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_RRD_DELAY):0];
            end
            for (ci = 0; ci < NUM_BANKS; ci = ci + 1) begin
                if (ci[BG_BITS+BA_BITS-1:0] != stage1_next_bank
                    && delay_before_activate_counter_d[ci]
                       < ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_ACTIVATE_DELAY):0])
                    delay_before_activate_counter_d[ci] =
                        ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_ACTIVATE_DELAY):0];
            end
            activate_timestamp_d[activate_index_q] =
                TFAW_CYCLES[$clog2(TFAW_CYCLES):0];
        end
    end

    // ─── Stall: combinational, reflects current registered state ───
    always @* begin
        o_wb_stall = stage1_pending || !reset_done || refresh_active
                     || (!SKIP_CALIB && !o_calib_complete);
    end

    // ── Sequential block ──
    integer bank_i;
    always @(posedge i_controller_clk) begin
        if (!i_rst_n) begin
            o_dfi_cs_n     <= 4'b1111;
            o_dfi_act_n    <= 4'b1111;
            o_dfi_ras_n    <= 4'b1111;
            o_dfi_cas_n    <= 4'b1111;
            o_dfi_we_n     <= 4'b1111;
            o_dfi_cke      <= 4'b0000;
            o_dfi_odt      <= 4'b0000;
            o_dfi_reset_n  <= 4'b0000;
            o_dfi_address  <= {(4*17){1'b0}};
            o_dfi_bank     <= {(4*BA_BITS){1'b0}};
            o_dfi_bg       <= {(4*BG_BITS){1'b0}};
            o_dfi_wrdata   <= {(4*DFI_DATA_WIDTH){1'b0}};
            o_dfi_wrdata_en   <= 4'b0000;
            o_dfi_wrdata_mask <= {(4*(2*BYTE_LANES)){1'b0}};
            o_dfi_rddata_en   <= 4'b0000;
            o_dfi_rdlvl_en      <= 1'b0;
            o_dfi_rdlvl_gate_en <= 1'b0;
            o_dfi_wrlvl_en      <= 1'b0;
            o_dfi_wrlvl_strobe  <= 1'b0;
            o_dfi_lvl_pattern   <= 4'b0000;
            o_dfi_lvl_periodic  <= 1'b0;
            o_wb_data  <= {WB_DATA_BITS{1'b0}};
            reset_done <= 1'b0;
            instruction_address <= 6'd0;
            bank_status_q <= {NUM_BANKS{1'b0}};
            delay_counter <= {DELAY_COUNTER_WIDTH{1'b0}};
            delay_counter_is_zero <= 1'b1;
            pause_counter <= 1'b0;
            calib_state <= CALIB_IDLE;
            calib_timer <= 0;
            calib_rr_timer <= 0;
            calib_retry_count <= 2'b00;
            calib_act_done <= 1'b0;
            o_calib_complete <= 1'b0;
            o_calib_error <= 1'b0;
            rom_cke_hold <= 1'b0;
            rom_reset_n_hold <= 1'b0;
            for (bank_i = 0; bank_i < SERDES_RATIO; bank_i = bank_i + 1) begin
                cmd_d[bank_i] <= {1'b1, CMD_NOP, 1'b0, 1'b0, 1'b0, 2'b00, 2'b00, 17'b0};
            end
            for (bank_i = 0; bank_i < NUM_BANKS; bank_i = bank_i + 1) begin
                bank_active_row_q[bank_i] <= {ROW_BITS{1'b0}};
                delay_before_precharge_counter_q[bank_i] <= 0;
                delay_before_activate_counter_q[bank_i]  <= 0;
                delay_before_write_counter_q[bank_i]     <= 0;
                delay_before_read_counter_q[bank_i]      <= 0;
            end
            for (bank_i = 0; bank_i < NUM_BG; bank_i = bank_i + 1) begin
                ccd_counter_q[bank_i] <= 0;
                wtr_counter_q[bank_i] <= 0;
                rrd_counter_q[bank_i] <= 0;
            end
            for (bank_i = 0; bank_i < 4; bank_i = bank_i + 1)
                activate_timestamp_q[bank_i] <= 0;
            activate_index_q <= 2'b00;
            rddata_en_pipe_q <= {RDDATA_EN_PIPE_WIDTH{1'b0}};
            wrdata_en_pipe_q <= {(WRITE_DATA_DELAY+1){1'b0}};
            for (bank_i = 0; bank_i <= WRITE_DATA_DELAY; bank_i = bank_i + 1) begin
                wr_data_pipe_q[bank_i] <= {WB_DATA_BITS{1'b0}};
                wr_dm_pipe_q[bank_i]   <= {WB_SEL_BITS{1'b0}};
            end
            write_ack_q <= 1'b0;
            read_ack_q  <= 1'b0;
            stage1_pending <= 1'b0;
            stage1_we      <= 1'b0;
            stage1_data    <= {WB_DATA_BITS{1'b0}};
            stage1_dm      <= {WB_SEL_BITS{1'b0}};
            stage1_col     <= {COL_BITS{1'b0}};
            stage1_ba      <= {BA_BITS{1'b0}};
            stage1_bg      <= {BG_BITS{1'b0}};
            stage1_row     <= {ROW_BITS{1'b0}};
            stage1_bank    <= {(BG_BITS+BA_BITS){1'b0}};
            stage1_next_bank <= {(BG_BITS+BA_BITS){1'b0}};
            stage1_next_row  <= {ROW_BITS{1'b0}};
            stage2_pending <= 1'b0;
            stage2_we      <= 1'b0;
            stage2_data    <= {WB_DATA_BITS{1'b0}};
            stage2_dm      <= {WB_SEL_BITS{1'b0}};
            stage2_col     <= {COL_BITS{1'b0}};
            stage2_ba      <= {BA_BITS{1'b0}};
            stage2_bg      <= {BG_BITS{1'b0}};
            stage2_row     <= {ROW_BITS{1'b0}};
            stage2_bank    <= {(BG_BITS+BA_BITS){1'b0}};
        end else begin
            // ═══════════════════════════════════════════════════════════
            // §11 — Command Scheduler Placeholder (NOP defaults)
            // All 4 slots default to DES (cs_n=1, NOP encoding).
            // The ROM controller or scheduler overrides specific slots.
            // ═══════════════════════════════════════════════════════════
            for (bank_i = 0; bank_i < SERDES_RATIO; bank_i = bank_i + 1) begin
                cmd_d[bank_i] <= {
                    1'b1,       //cs_n = 1 (deselected)
                    CMD_NOP,    //{act_n=1, ras_n=1, cas_n=1, we_n=1}
                    cmd_odt,    //odt (broadcast to all slots per SPEC §6.3)
                    reset_done ? 1'b1 : init_cke,     //cke (muxed: rom_instruction on fire, hold on countdown)
                    reset_done ? 1'b1 : init_reset_n, //reset_n (muxed: rom_instruction on fire, hold on countdown)
                    2'b00,      //bg
                    2'b00,      //ba
                    17'b0       //addr
                };
            end

            // ═══════════════════════════════════════════════════════════
            // §10 — ROM Controller (implements JESD79-4D §3.3 init FSM)
            // Drives init sequence, then continues running the refresh
            // loop (addrs 33-35) after init completes.
            // ═══════════════════════════════════════════════════════════
            if (!reset_done || instruction_address >= ROM_ADDR_REF_START) begin
                // Delay counter management
                if (!delay_counter_is_zero) begin
                    delay_counter <= delay_counter - 1'b1;
                    delay_counter_is_zero <= (delay_counter == {{(DELAY_COUNTER_WIDTH-1){1'b0}}, 1'b1});
                end else if (!pause_counter) begin
                    // Issue the command from ROM on slot 0
                    if (rom_cmd_is_mrs) begin
                        // MRS: cs_n=0, CMD_MRS, bg/ba from MRS_SELECT, addr from instruction
                        cmd_d[0] <= {
                            1'b0,                        //cs_n = 0
                            CMD_MRS,                     //{act_n=1, ras_n=0, cas_n=0, we_n=0}
                            1'b0,                        //odt = 0
                            1'b1,                        //cke = 1
                            1'b1,                        //reset_n = 1
                            1'b0, rom_instruction[22],   //bg[1:0] = {0, BG0}
                            rom_instruction[21:20],      //ba[1:0]
                            3'b000,                      //A16:A14 don't care for MRS
                            rom_instruction[13:0]        //A13:A0 = MRS address
                        };
                    end else begin
                        // Timer/command instruction
                        cmd_d[0] <= {
                            (rom_instruction[26:23] == CMD_DES) ? 1'b1 : 1'b0, //cs_n
                            rom_instruction[26:23],      //cmd
                            1'b0,                        //odt = 0
                            rom_instruction[ROM_CKE],    //cke
                            rom_instruction[ROM_RESET_N],//reset_n
                            2'b00,                       //bg
                            2'b00,                       //ba
                            {6'b0, rom_instruction[ROM_A10], 10'b0} //A10 for PRE ALL, ZQCL
                        };
                    end

                    // Latch CKE/RESET_N for the duration of this ROM phase.
                    // MRS instructions always have CKE=1, RESET_N=1.
                    if (rom_cmd_is_mrs) begin
                        rom_cke_hold     <= 1'b1;
                        rom_reset_n_hold <= 1'b1;
                    end else begin
                        rom_cke_hold     <= rom_instruction[ROM_CKE];
                        rom_reset_n_hold <= rom_instruction[ROM_RESET_N];
                    end

                    // Load delay counter for timer instructions
                    if (rom_use_timer) begin
                        delay_counter <= rom_instruction[DELAY_COUNTER_WIDTH-1:0];
                        delay_counter_is_zero <= (rom_instruction[DELAY_COUNTER_WIDTH-1:0] == 0);
                    end

                    // Check for RST_DONE
                    if (rom_instruction[ROM_RST_DONE]) begin
                        reset_done <= 1'b1;
                    end

                    // Advance instruction address
                    if (instruction_address == ROM_ADDR_REF_END)
                        instruction_address <= ROM_ADDR_REF_START;
                    else
                        instruction_address <= instruction_address + 1'b1;
                end
            end
            // ═══════════════════════════════════════════════════════════
            // §11d — Scheduler Command Construction (SPEC §3.3–3.4)
            // Driven by sched_* flags from combinational §11c.
            // Scheduler only fires during tREFI idle window.
            // ═══════════════════════════════════════════════════════════
            if (sched_precharge) begin
                cmd_d[PRECHARGE_SLOT] <= {
                    1'b0,           //cs_n = 0
                    CMD_PRE,        //{act_n=1, ras_n=0, cas_n=1, we_n=0}
                    cmd_odt, 1'b1, 1'b1,  //odt, cke=1, reset_n=1
                    stage2_bg_padded, //bg[1:0] (PLAN §6.4)
                    stage2_ba,      //ba
                    7'b0, 1'b0, 9'b0  //A10=0 (single bank precharge)
                };
            end
            if (sched_activate) begin
                cmd_d[ACTIVATE_SLOT] <= {
                    1'b0,           //cs_n = 0
                    1'b0,           //act_n = 0 (ACTIVATE)
                    stage2_row_padded[16],  //ras_n → A16
                    stage2_row_padded[15],  //cas_n → A15
                    stage2_row_padded[14],  //we_n  → A14
                    cmd_odt, 1'b1, 1'b1,
                    stage2_bg_padded, //bg[1:0] (PLAN §6.4)
                    stage2_ba,
                    stage2_row_padded  //addr[16:0] = full row address
                };
            end
            if (sched_write) begin
                cmd_d[WRITE_SLOT] <= {
                    1'b0,           //cs_n = 0
                    CMD_WR,         //{act_n=1, ras_n=1, cas_n=0, we_n=0}
                    cmd_odt, 1'b1, 1'b1,
                    stage2_bg_padded, //bg[1:0] (PLAN §6.4)
                    stage2_ba,
                    3'b000,         //A16:A14
                    1'b0,           //A13
                    1'b0,           //A12 (BL8, no BC4)
                    (COL_BITS > 10) ? stage2_col[10] : 1'b0, //A11: col[10] for x4
                    1'b0,           //A10 = 0 (no auto-precharge)
                    stage2_col[9:0] //A9:A0 = column
                };
            end
            if (sched_read) begin
                cmd_d[READ_SLOT] <= {
                    1'b0,           //cs_n = 0
                    CMD_RD,         //{act_n=1, ras_n=1, cas_n=0, we_n=1}
                    cmd_odt, 1'b1, 1'b1,
                    stage2_bg_padded, //bg[1:0] (PLAN §6.4)
                    stage2_ba,
                    3'b000,         //A16:A14
                    1'b0,           //A13
                    1'b0,           //A12 (BL8, no BC4)
                    (COL_BITS > 10) ? stage2_col[10] : 1'b0, //A11: col[10] for x4
                    1'b0,           //A10 = 0 (no auto-precharge)
                    stage2_col[9:0] //A9:A0 = column
                };
            end
            if (sched_anticipate) begin
                cmd_d[ACTIVATE_SLOT] <= {
                    1'b0,           //cs_n = 0
                    1'b0,           //act_n = 0 (ACTIVATE)
                    stage1_next_row_padded[16],
                    stage1_next_row_padded[15],
                    stage1_next_row_padded[14],
                    cmd_odt, 1'b1, 1'b1,
                    stage1_next_bg_padded, //bg[1:0] (PLAN §6.4)
                    stage1_next_bank[BA_BITS-1:0],
                    stage1_next_row_padded
                };
            end

            // ═══════════════════════════════════════════════════════════
            // §13 — Read/Write Data Enable Pipelines + WB ACK
            // Shift registers track when dfi_rddata_en / dfi_wrdata_en
            // should assert after a READ / WRITE command. WB ACK is
            // generated from write command issue and dfi_rddata_valid.
            // ═══════════════════════════════════════════════════════════

            // Write data enable shift register (SPEC §5.3)
            wrdata_en_pipe_q <= {1'b0, wrdata_en_pipe_q[WRITE_DATA_DELAY:1]};
            if (sched_write)
                wrdata_en_pipe_q[WRITE_DATA_DELAY] <= 1'b1;
            o_dfi_wrdata_en <= {4{wrdata_en_pipe_q[0]}};

            // Write data delay pipeline (SPEC §5.4) — mirrors wrdata_en_pipe_q
            // shift structure: right-shift, load at [WD], read at [0].
            for (bank_i = 0; bank_i < WRITE_DATA_DELAY; bank_i = bank_i + 1) begin
                wr_data_pipe_q[bank_i] <= wr_data_pipe_q[bank_i + 1];
                wr_dm_pipe_q[bank_i]   <= wr_dm_pipe_q[bank_i + 1];
            end
            wr_data_pipe_q[WRITE_DATA_DELAY] <= {WB_DATA_BITS{1'b0}};
            wr_dm_pipe_q[WRITE_DATA_DELAY]   <= {WB_SEL_BITS{1'b0}};
            if (sched_write) begin
                wr_data_pipe_q[WRITE_DATA_DELAY] <= stage2_data;
                wr_dm_pipe_q[WRITE_DATA_DELAY]   <= stage2_dm;
            end
            o_dfi_wrdata      <= wr_data_pipe_q[0];
            o_dfi_wrdata_mask <= ~wr_dm_pipe_q[0];

            // Read data enable shift register (SPEC §4.4)
            rddata_en_pipe_q <= {1'b0, rddata_en_pipe_q[RDDATA_EN_PIPE_WIDTH-1:1]};
            if (sched_read)
                rddata_en_pipe_q[RDDATA_EN_PIPE_WIDTH-1] <= 1'b1;
            o_dfi_rddata_en <= {4{rddata_en_pipe_q[0]}};

            // Read data capture from DFI (SPEC §4.5)
            if (|i_dfi_rddata_valid)
                o_wb_data <= i_dfi_rddata;

            // WB ACK generation (SPEC §4.6)
            // Write ACK: 1 cycle after WR command issues
            // Read ACK: 1 cycle after dfi_rddata_valid (data latch delay)
            write_ack_q <= sched_write;
            read_ack_q  <= |i_dfi_rddata_valid;

            // ═══════════════════════════════════════════════════════════
            // §14 — Training Command Pump (DFI 3.1 Full Training Mode)
            // MC-side calibration FSM: drives DFI training enables and
            // pumps READ commands / wrlvl strobes during calibration
            // windows opened by the init ROM (addrs 22, 27). The pump
            // takes over cmd_d directly while pause_counter is held.
            // See SPEC §9.2 for state descriptions.
            // ═══════════════════════════════════════════════════════════
            if (SKIP_CALIB) begin
                o_calib_complete <= reset_done;
            end else begin
                if (calib_timer != 0)
                    calib_timer <= calib_timer - 1'b1;
                if (calib_rr_timer != 0)
                    calib_rr_timer <= calib_rr_timer - 1'b1;

                case (calib_state)
                    CALIB_IDLE: begin
                        if (instruction_address == ROM_ADDR_RD_CAL
                            && delay_counter_is_zero
                            && i_dfi_init_complete) begin
                            pause_counter <= 1'b1;
                            calib_state <= CALIB_GATE_EN;
                            calib_rr_timer <= T_RDLVL_EN[$clog2(T_WRLVL_WW):0];
                        end
                    end

                    CALIB_GATE_EN: begin
                        o_dfi_rdlvl_gate_en <= 1'b1;
                        if (calib_rr_timer == 0) begin
                            calib_state <= CALIB_GATE_READ;
                            calib_timer <= T_RDLVL_MAX[$clog2(T_RDLVL_MAX):0];
                        end
                    end

                    CALIB_GATE_READ: begin
                        if (!calib_act_done) begin
                            // ACT BG0/BA0/row0 before first training READ
                            cmd_d[ACTIVATE_SLOT] <= {
                                1'b0, 1'b0, 3'b000,
                                cmd_odt, 1'b1, 1'b1,
                                2'b00, 2'b00, 17'b0
                            };
                            calib_act_done <= 1'b1;
                            calib_rr_timer <=
                                ACTIVATE_TO_READWRITE_DELAY[$clog2(T_WRLVL_WW):0];
                        end else if (calib_rr_timer == 0) begin
                            cmd_d[READ_SLOT] <= {
                                1'b0, CMD_RD, cmd_odt, 1'b1, 1'b1,
                                2'b00, 2'b00, 17'b0
                            };
                            rddata_en_pipe_q[RDDATA_EN_PIPE_WIDTH-1] <= 1'b1;
                            calib_rr_timer <= T_RDLVL_RR[$clog2(T_WRLVL_WW):0];
                            calib_state <= CALIB_GATE_WAIT;
                        end
                    end

                    CALIB_GATE_WAIT: begin
                        if (calib_timer == 0) begin
                            if (calib_retry_count < CALIB_RETRY_MAX) begin
                                calib_retry_count <= calib_retry_count + 1'b1;
                                calib_state <= CALIB_GATE_EN;
                                calib_rr_timer <=
                                    T_RDLVL_EN[$clog2(T_WRLVL_WW):0];
                            end else
                                calib_state <= CALIB_ERROR;
                        end else if (&i_dfi_rdlvl_resp) begin
                            calib_state <= CALIB_GATE_EXIT;
                        end else if (calib_rr_timer == 0) begin
                            cmd_d[READ_SLOT] <= {
                                1'b0, CMD_RD, cmd_odt, 1'b1, 1'b1,
                                2'b00, 2'b00, 17'b0
                            };
                            rddata_en_pipe_q[RDDATA_EN_PIPE_WIDTH-1] <= 1'b1;
                            calib_rr_timer <= T_RDLVL_RR[$clog2(T_WRLVL_WW):0];
                        end
                    end

                    CALIB_GATE_EXIT: begin
                        o_dfi_rdlvl_gate_en <= 1'b0;
                        calib_state <= CALIB_EYE_EN;
                        calib_rr_timer <= T_RDLVL_EN[$clog2(T_WRLVL_WW):0];
                        calib_retry_count <= 2'b00;
                    end

                    CALIB_EYE_EN: begin
                        o_dfi_rdlvl_en <= 1'b1;
                        if (calib_rr_timer == 0) begin
                            calib_state <= CALIB_EYE_READ;
                            calib_timer <= T_RDLVL_MAX[$clog2(T_RDLVL_MAX):0];
                        end
                    end

                    CALIB_EYE_READ: begin
                        // bank already activated from gate training
                        cmd_d[READ_SLOT] <= {
                            1'b0, CMD_RD, cmd_odt, 1'b1, 1'b1,
                            2'b00, 2'b00, 17'b0
                        };
                        rddata_en_pipe_q[RDDATA_EN_PIPE_WIDTH-1] <= 1'b1;
                        calib_rr_timer <= T_RDLVL_RR[$clog2(T_WRLVL_WW):0];
                        calib_state <= CALIB_EYE_WAIT;
                    end

                    CALIB_EYE_WAIT: begin
                        if (calib_timer == 0) begin
                            if (calib_retry_count < CALIB_RETRY_MAX) begin
                                calib_retry_count <= calib_retry_count + 1'b1;
                                calib_state <= CALIB_EYE_EN;
                                calib_rr_timer <=
                                    T_RDLVL_EN[$clog2(T_WRLVL_WW):0];
                            end else
                                calib_state <= CALIB_ERROR;
                        end else if (&i_dfi_rdlvl_resp) begin
                            calib_state <= CALIB_EYE_EXIT;
                        end else if (calib_rr_timer == 0) begin
                            cmd_d[READ_SLOT] <= {
                                1'b0, CMD_RD, cmd_odt, 1'b1, 1'b1,
                                2'b00, 2'b00, 17'b0
                            };
                            rddata_en_pipe_q[RDDATA_EN_PIPE_WIDTH-1] <= 1'b1;
                            calib_rr_timer <= T_RDLVL_RR[$clog2(T_WRLVL_WW):0];
                        end
                    end

                    CALIB_EYE_EXIT: begin
                        o_dfi_rdlvl_en <= 1'b0;
                        if (calib_act_done) begin
                            // precharge BG0/BA0 before ROM issues MRS
                            cmd_d[PRECHARGE_SLOT] <= {
                                1'b0, CMD_PRE, cmd_odt, 1'b1, 1'b1,
                                2'b00, 2'b00,
                                7'b0, 1'b0, 9'b0
                            };
                            calib_act_done <= 1'b0;
                            calib_rr_timer <=
                                PRECHARGE_TO_ACTIVATE_DELAY[$clog2(T_WRLVL_WW):0];
                        end else if (calib_rr_timer == 0 && pause_counter) begin
                            pause_counter <= 1'b0;
                        end
                        // wait for ROM to reach write leveling window
                        if (instruction_address == ROM_ADDR_WL_CAL
                            && delay_counter_is_zero && !pause_counter) begin
                            pause_counter <= 1'b1;
                            calib_state <= CALIB_WL_EN;
                            calib_rr_timer <= T_WRLVL_EN[$clog2(T_WRLVL_WW):0];
                            calib_retry_count <= 2'b00;
                        end
                    end

                    CALIB_WL_EN: begin
                        o_dfi_wrlvl_en <= 1'b1;
                        if (calib_rr_timer == 0) begin
                            calib_state <= CALIB_WL_STROBE;
                            calib_timer <= T_WRLVL_MAX[$clog2(T_RDLVL_MAX):0];
                        end
                    end

                    CALIB_WL_STROBE: begin
                        o_dfi_wrlvl_strobe <= 1'b1;
                        calib_rr_timer <= T_WRLVL_WW[$clog2(T_WRLVL_WW):0];
                        calib_state <= CALIB_WL_WAIT;
                    end

                    CALIB_WL_WAIT: begin
                        o_dfi_wrlvl_strobe <= 1'b0;
                        if (calib_timer == 0) begin
                            if (calib_retry_count < CALIB_RETRY_MAX) begin
                                calib_retry_count <= calib_retry_count + 1'b1;
                                calib_state <= CALIB_WL_EN;
                                calib_rr_timer <=
                                    T_WRLVL_EN[$clog2(T_WRLVL_WW):0];
                            end else
                                calib_state <= CALIB_ERROR;
                        end else if (&i_dfi_wrlvl_resp) begin
                            calib_state <= CALIB_WL_EXIT;
                        end else if (calib_rr_timer == 0) begin
                            o_dfi_wrlvl_strobe <= 1'b1;
                            calib_rr_timer <=
                                T_WRLVL_WW[$clog2(T_WRLVL_WW):0];
                        end
                    end

                    CALIB_WL_EXIT: begin
                        o_dfi_wrlvl_en <= 1'b0;
                        o_dfi_wrlvl_strobe <= 1'b0;
                        if (!pause_counter) begin
                            // already released — wait for init to finish
                            if (reset_done) begin
                                calib_state <= CALIB_DONE;
                                o_calib_complete <= 1'b1;
                            end
                        end else begin
                            pause_counter <= 1'b0;
                        end
                    end

                    CALIB_DONE: begin
                        o_calib_complete <= 1'b1;
                    end

                    CALIB_ERROR: begin
                        o_calib_error <= 1'b1;
                    end
                endcase
            end

            // ═══════════════════════════════════════════════════════════
            // §11b — Counter Latch + Pipeline Handoff + Stage 1 WB Accept
            // ═══════════════════════════════════════════════════════════

            // Latch combinational next-state into registers
            for (bank_i = 0; bank_i < NUM_BANKS; bank_i = bank_i + 1) begin
                delay_before_precharge_counter_q[bank_i] <= delay_before_precharge_counter_d[bank_i];
                delay_before_activate_counter_q[bank_i]  <= delay_before_activate_counter_d[bank_i];
                delay_before_write_counter_q[bank_i]     <= delay_before_write_counter_d[bank_i];
                delay_before_read_counter_q[bank_i]      <= delay_before_read_counter_d[bank_i];
                bank_status_q[bank_i]     <= bank_status_d[bank_i];
                bank_active_row_q[bank_i] <= bank_active_row_d[bank_i];
            end
            for (bank_i = 0; bank_i < NUM_BG; bank_i = bank_i + 1) begin
                ccd_counter_q[bank_i] <= ccd_counter_d[bank_i];
                wtr_counter_q[bank_i] <= wtr_counter_d[bank_i];
                rrd_counter_q[bank_i] <= rrd_counter_d[bank_i];
            end
            for (bank_i = 0; bank_i < 4; bank_i = bank_i + 1)
                activate_timestamp_q[bank_i] <= activate_timestamp_d[bank_i];

            // tFAW index advance on any ACT issue (scheduler or anticipation)
            if (sched_activate || sched_anticipate)
                activate_index_q <= activate_index_q + 1'b1;

            // Stage 1→2 handoff: zero-bubble pipeline (PLAN §6.6)
            // stage2_update=1 when Stage 2 idle OR just issued WR/RD (completing).
            // Consume Stage 1's request immediately → no wasted cycles.
            if (stage2_update) begin
                if (stage1_pending) begin
                    stage2_pending <= 1'b1;
                    stage2_we      <= stage1_we;
                    stage2_data    <= stage1_data;
                    stage2_dm      <= stage1_dm;
                    stage2_col     <= stage1_col;
                    stage2_ba      <= stage1_ba;
                    stage2_bg      <= stage1_bg;
                    stage2_row     <= stage1_row;
                    stage2_bank    <= stage1_bank;
                    stage1_pending <= 1'b0;
                end else begin
                    stage2_pending <= 1'b0;
                end
            end

            // Stage 1: latch decoded WB request
            if (wb_accept) begin
                stage1_pending   <= 1'b1;
                stage1_we        <= i_wb_we;
                stage1_data      <= i_wb_data;
                stage1_dm        <= i_wb_sel;
                stage1_col       <= wb_col;
                stage1_ba        <= wb_ba;
                stage1_bg        <= wb_bg;
                stage1_row       <= wb_row;
                stage1_bank      <= wb_bank;
                stage1_next_bank <= wb_next_bank;
                stage1_next_row  <= wb_next_row;
            end

            // ═══════════════════════════════════════════════════════════
            // §12 — DFI Signal Mapping (DFI 3.1 §3.2, 4-phase, 1:4 ratio)
            // Decompose packed cmd_d[] slots into flat DFI output vectors
            // ═══════════════════════════════════════════════════════════
            for (bank_i = 0; bank_i < SERDES_RATIO; bank_i = bank_i + 1) begin
                o_dfi_cs_n[bank_i]    <= cmd_d[bank_i][CMD_CS_N];
                o_dfi_act_n[bank_i]   <= cmd_d[bank_i][CMD_ACT_N];
                o_dfi_ras_n[bank_i]   <= cmd_d[bank_i][CMD_RAS_N];
                o_dfi_cas_n[bank_i]   <= cmd_d[bank_i][CMD_CAS_N];
                o_dfi_we_n[bank_i]    <= cmd_d[bank_i][CMD_WE_N];
                o_dfi_odt[bank_i]     <= cmd_d[bank_i][CMD_ODT];
                o_dfi_cke[bank_i]     <= cmd_d[bank_i][CMD_CKE];
                o_dfi_reset_n[bank_i] <= cmd_d[bank_i][CMD_RESET_N];
                o_dfi_bg[BG_BITS*bank_i +: BG_BITS]   <= cmd_d[bank_i][CMD_BG_START-1 +: BG_BITS];
                o_dfi_bank[BA_BITS*bank_i +: BA_BITS]  <= cmd_d[bank_i][CMD_BA_START:CMD_BA_START-(BA_BITS-1)];
                o_dfi_address[17*bank_i +: 17]         <= cmd_d[bank_i][16:0];
            end
        end
    end

    // ══════════════════════════════════════════════════════════════
    // §9 — Reset/Refresh ROM (JESD79-4D §3.3, Figure 7)
    // 36 addresses (0–35): init sequence + calibration windows + refresh loop
    // ══════════════════════════════════════════════════════════════

    function [31:0] rom_timer(input [4:0] ctl, input [3:0] cmd, input integer timer);
        rom_timer = {ctl, cmd, 3'b000, timer[19:0]};
    endfunction

    function [31:0] rom_mrs(input [2:0] mrs_sel, input [13:0] mrs_addr);
        rom_mrs = {2'b00, mrs_addr[10], 2'b11, CMD_MRS, mrs_sel, 6'b0, mrs_addr};
    endfunction

    function [31:0] read_rom_instruction(input [5:0] addr);
        case (addr)
            // ── Power-on reset (JESD79-4D §3.3, Figure 7) ──
            6'd0:  read_rom_instruction = rom_timer(CTL_CKE0_RST0, CMD_NOP, ps_to_cycles(POWER_ON_RESET_HIGH_ps)); // RESET_n=0, CKE=0, wait ≥200µs
            6'd1:  read_rom_instruction = rom_timer(CTL_CKE0_RST1, CMD_NOP, ps_to_cycles(INITIAL_CKE_LOW_ps));     // RESET_n=1, CKE=0, wait ≥500µs
            6'd2:  read_rom_instruction = rom_timer(CTL_TIMER,      CMD_DES, ps_to_cycles(tXPR_ps));                // CKE=1, deselect, wait tXPR

            // ── Mode register writes (MR3→MR6→MR5→MR4→MR2→MR1→MR0) ──
            6'd3:  read_rom_instruction = rom_mrs  (MRS_MR3, MR3_MPR_DIS);                          // MR3: MPR off
            6'd4:  read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, nCK_to_cycles(tMRD_nCK)); // wait tMRD
            6'd5:  read_rom_instruction = rom_mrs  (MRS_MR6, MR6);                                  // MR6: VrefDQ, tCCD_L
            6'd6:  read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, nCK_to_cycles(tMRD_nCK)); // wait tMRD
            6'd7:  read_rom_instruction = rom_mrs  (MRS_MR5, MR5);                                  // MR5: DM, RTT_PARK
            6'd8:  read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, nCK_to_cycles(tMRD_nCK)); // wait tMRD
            6'd9:  read_rom_instruction = rom_mrs  (MRS_MR4, MR4);                                  // MR4: preamble, temp readout
            6'd10: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, nCK_to_cycles(tMRD_nCK)); // wait tMRD
            6'd11: read_rom_instruction = rom_mrs  (MRS_MR2, MR2);                                  // MR2: CWL, RTT_WR
            6'd12: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, nCK_to_cycles(tMRD_nCK)); // wait tMRD
            6'd13: read_rom_instruction = rom_mrs  (MRS_MR1, MR1_WL_DIS);                           // MR1: DLL on, drive, RTT_NOM, WL off
            6'd14: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, nCK_to_cycles(tMRD_nCK)); // wait tMRD
            6'd15: read_rom_instruction = rom_mrs  (MRS_MR0, MR0);                                  // MR0: BL8, CL, DLL reset, WR
            6'd16: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, ps_to_cycles(tMOD_ps));    // wait tMOD

            // ── ZQCL + DLL lock ──
            6'd17: read_rom_instruction = rom_timer(CTL_TIMER_A10,  CMD_ZQCL, nCK_to_cycles(tZQinit_nCK)); // ZQCL (A10=1), wait tZQinit
            6'd18: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, nCK_to_cycles(tDLLK_nCK));    // wait tDLLK (DLL lock)
            6'd19: read_rom_instruction = rom_timer(CTL_TIMER_A10,  CMD_PRE, ps_to_cycles(tRP_ps));        // PRE ALL (A10=1), wait tRP

            // ── Read calibration window (MPR mode, JESD79-4D §4.25) ──
            6'd20: read_rom_instruction = rom_mrs  (MRS_MR3, MR3_MPR_EN);                              // MR3: MPR enable (A2=1)
            6'd21: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, ps_to_cycles(tMOD_ps));    // wait tMOD
            6'd22: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, CALIBRATION_DELAY);        // read leveling window
            6'd23: read_rom_instruction = rom_mrs  (MRS_MR3, MR3_MPR_DIS);                             // MR3: MPR disable
            6'd24: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, ps_to_cycles(tMOD_ps));    // wait tMOD

            // ── Write leveling window (JESD79-4D §4.26) ──
            6'd25: read_rom_instruction = rom_mrs  (MRS_MR1, MR1_WL_EN);                               // MR1: write leveling on (A7=1)
            6'd26: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, nCK_to_cycles(tWLMRD_nCK)); // wait tWLMRD
            6'd27: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, CALIBRATION_DELAY);        // write leveling window
            6'd28: read_rom_instruction = rom_mrs  (MRS_MR1, MR1_WL_DIS);                              // MR1: write leveling off
            6'd29: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, ps_to_cycles(tMOD_ps));    // wait tMOD

            // ── Final refresh + done ──
            6'd30: read_rom_instruction = rom_timer(CTL_TIMER_A10,  CMD_PRE, ps_to_cycles(tRP_ps));  // PRE ALL, wait tRP
            6'd31: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_REF, ps_to_cycles(tRFC_ps)); // REF, wait tRFC
            6'd32: read_rom_instruction = rom_timer(CTL_DONE,       CMD_NOP, 0);                     // reset_done=1, init complete

            // ── Refresh loop (repeats 33→34→35→33) ──
            6'd33: read_rom_instruction = rom_timer(CTL_TIMER_A10,  CMD_PRE, ps_to_cycles(tRP_ps));   // PRE ALL, wait tRP
            6'd34: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_REF, ps_to_cycles(tRFC_ps));  // REF, wait tRFC
            6'd35: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, REFRESH_TREFI_TIMER);    // NOP, wait adjusted tREFI
            default: read_rom_instruction = rom_timer(CTL_TIMER, CMD_NOP, 0);
        endcase
    endfunction

    // ══════════════════════════════════════════════════════════════
    // §16 — Debug $display
    // ══════════════════════════════════════════════════════════════
`ifndef YOSYS
    initial begin
        $display("══════════════════════════════════════════════════════════════");
        $display("UberDDR4 Controller Configuration");
        $display("══════════════════════════════════════════════════════════════");

        $display("── Device ──");
        $display("  DDR4_CLK_PERIOD       = %0d ps", DDR4_CLK_PERIOD);
        $display("  CONTROLLER_CLK_PERIOD = %0d ps", CONTROLLER_CLK_PERIOD);
        $display("  SERDES_RATIO          = %0d", SERDES_RATIO);
        $display("  ROW_BITS              = %0d", ROW_BITS);
        $display("  COL_BITS              = %0d", COL_BITS);
        $display("  BA_BITS               = %0d", BA_BITS);
        $display("  BG_BITS               = %0d", BG_BITS);
        $display("  DQ_BITS               = %0d", DQ_BITS);
        $display("  BYTE_LANES            = %0d", BYTE_LANES);
        $display("  DENSITY               = %0d Gb", DENSITY);
        $display("  PAGE_SIZE             = %0d bytes", PAGE_SIZE);
        $display("  NUM_BANKS             = %0d", NUM_BANKS);
        $display("  NUM_BG                = %0d", NUM_BG);

        $display("── Latency ──");
        $display("  CL                    = %0d nCK", CL_nCK);
        $display("  CWL                   = %0d nCK", CWL_nCK);
        $display("  WR                    = %0d nCK", WR_nCK);

        $display("── Core Timing ──");
        $display("  tRAS                  = %0d ps (%0d nCK)", tRAS_ps, ps_to_nCK(tRAS_ps));
        $display("  tRCD                  = %0d ps (%0d nCK)", tRCD_ps, ps_to_nCK(tRCD_ps));
        $display("  tRP                   = %0d ps (%0d nCK)", tRP_ps, ps_to_nCK(tRP_ps));
        $display("  tRC                   = %0d ps (%0d nCK)", tRC_ps, ps_to_nCK(tRC_ps));
        $display("  tWR                   = %0d ps (%0d nCK)", tWR_ps, ps_to_nCK(tWR_ps));
        $display("  tRTP                  = %0d ps (%0d nCK)", tRTP_ps, ps_to_nCK(tRTP_ps));

        $display("── Bank-Group Timing ──");
        $display("  tCCD_L                = %0d ps (%0d nCK)", tCCD_L_ps, ps_to_nCK(tCCD_L_ps));
        $display("  tCCD_S                = %0d nCK", tCCD_S_nCK);
        $display("  tWTR_L                = %0d ps (%0d nCK)", tWTR_L_ps, ps_to_nCK(tWTR_L_ps));
        $display("  tWTR_S                = %0d ps (%0d nCK)", tWTR_S_ps, ps_to_nCK(tWTR_S_ps));
        $display("  tRRD_L                = %0d ps (%0d nCK)", tRRD_L_ps, ps_to_nCK(tRRD_L_ps));
        $display("  tRRD_S                = %0d ps (%0d nCK)", tRRD_S_ps, ps_to_nCK(tRRD_S_ps));
        $display("  tFAW                  = %0d ps (%0d nCK, %0d ctrl)", tFAW_ps, ps_to_nCK(tFAW_ps), TFAW_CYCLES);

        $display("── Init/MRS Timing ──");
        $display("  tMRD                  = %0d nCK (%0d ctrl)", tMRD_nCK, nCK_to_cycles(tMRD_nCK));
        $display("  tMOD                  = %0d ps (%0d ctrl)", tMOD_ps, ps_to_cycles(tMOD_ps));
        $display("  tZQinit               = %0d nCK (%0d ctrl)", tZQinit_nCK, nCK_to_cycles(tZQinit_nCK));
        $display("  tDLLK                 = %0d nCK (%0d ctrl)", tDLLK_nCK, nCK_to_cycles(tDLLK_nCK));
        $display("  tXPR                  = %0d ps (%0d ctrl)", tXPR_ps, ps_to_cycles(tXPR_ps));
        $display("  tWLMRD                = %0d nCK (%0d ctrl)", tWLMRD_nCK, nCK_to_cycles(tWLMRD_nCK));
        $display("  POWER_ON_RESET        = %0d ps (%0d ctrl)", POWER_ON_RESET_HIGH_ps, ps_to_cycles(POWER_ON_RESET_HIGH_ps));
        $display("  INITIAL_CKE_LOW       = %0d ps (%0d ctrl)", INITIAL_CKE_LOW_ps, ps_to_cycles(INITIAL_CKE_LOW_ps));

        $display("── Refresh ──");
        $display("  tRFC                  = %0d ps (%0d ctrl)", tRFC_ps, ps_to_cycles(tRFC_ps));
        $display("  tREFI                 = %0d ps (%0d ctrl)", tREFI_ps, ps_to_cycles(tREFI_ps));
        $display("  Refresh loop period   = %0d ctrl (%0d ps)",
                 ps_to_cycles(tRP_ps) + ps_to_cycles(tRFC_ps) + REFRESH_TREFI_TIMER + 3,
                 (ps_to_cycles(tRP_ps) + ps_to_cycles(tRFC_ps) + REFRESH_TREFI_TIMER + 3)
                 * CONTROLLER_CLK_PERIOD);

        $display("── Computed Delays (controller cycles) ──");
        $display("  ACT->RD/WR            = %0d", ACTIVATE_TO_READWRITE_DELAY);
        $display("  RD->PRE               = %0d", READ_TO_PRECHARGE_DELAY);
        $display("  WR->PRE               = %0d", WRITE_TO_PRECHARGE_DELAY);
        $display("  PRE->ACT              = %0d", PRECHARGE_TO_ACTIVATE_DELAY);
        $display("  ACT->PRE              = %0d", ACTIVATE_TO_PRECHARGE_DELAY);
        $display("  RD->WR                = %0d", READ_TO_WRITE_DELAY);
        $display("  CAS->CAS (same BG)    = %0d", CAS_TO_CAS_DELAY_SAME_BG);
        $display("  CAS->CAS (diff BG)    = %0d", CAS_TO_CAS_DELAY_DIFF_BG);
        $display("  WR->RD (same BG)      = %0d", WRITE_TO_READ_DELAY_SAME_BG);
        $display("  WR->RD (diff BG)      = %0d", WRITE_TO_READ_DELAY_DIFF_BG);
        $display("  ACT->ACT (same BG)    = %0d", ACTIVATE_TO_ACTIVATE_DELAY_SAME_BG);
        $display("  ACT->ACT (diff BG)    = %0d", ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG);
        $display("  tFAW                  = %0d", TFAW_CYCLES);

        $display("── Slot Assignment ──");
        $display("  READ_SLOT             = %0d", READ_SLOT);
        $display("  WRITE_SLOT            = %0d", WRITE_SLOT);
        $display("  ACTIVATE_SLOT         = %0d", ACTIVATE_SLOT);
        $display("  PRECHARGE_SLOT        = %0d", PRECHARGE_SLOT);

        $display("── Address Mapping ──");
        $display("  ADDR_MAPPING          = %0d", ADDR_MAPPING);
        $display("  WB_ADDR_BITS          = %0d", WB_ADDR_BITS);
        $display("  WB_DATA_BITS          = %0d", WB_DATA_BITS);
        $display("  COL_LOW               = %0d", COL_LOW);

        $display("── Mode Registers ──");
        $display("  MR0                   = 14'h%04h", MR0);
        $display("  MR1 (WL off)          = 14'h%04h", MR1_WL_DIS);
        $display("  MR2                   = 14'h%04h", MR2);
        $display("  MR3 (MPR off)         = 14'h%04h", MR3_MPR_DIS);
        $display("  MR4                   = 14'h%04h", MR4);
        $display("  MR5                   = 14'h%04h", MR5);
        $display("  MR6                   = 14'h%04h", MR6);

        $display("── Sim Flags ──");
        $display("  MICRON_SIM            = %0d", MICRON_SIM);
        $display("  SKIP_CALIB            = %0d", SKIP_CALIB);
        $display("══════════════════════════════════════════════════════════════");
    end
`endif

    // ══════════════════════════════════════════════════════════════
    // §17 — Helper Functions
    // NOTE: Verilog elaboration resolves functions before localparams,
    // so these can be placed at the bottom even though §3–§6 call them
    // ══════════════════════════════════════════════════════════════

    function integer max_fn(input integer a, input integer b);
        max_fn = (a > b) ? a : b;
    endfunction

    function integer ps_to_nCK(input integer ps);
        //ceiling division: convert picoseconds to DDR4 clock cycles
        ps_to_nCK = (ps + DDR4_CLK_PERIOD - 1) / DDR4_CLK_PERIOD;
    endfunction

    function integer ps_to_cycles(input integer ps);
        //convert picoseconds to controller clock cycles
        ps_to_cycles = (ps + CONTROLLER_CLK_PERIOD - 1) / CONTROLLER_CLK_PERIOD;
    endfunction

    function integer nCK_to_cycles(input integer nck);
        //convert DDR4 clock cycles to controller clock cycles
        nCK_to_cycles = (nck + SERDES_RATIO - 1) / SERDES_RATIO;
    endfunction

    // find_delay: slot-aware delay counter value for DFI 3.1 4-phase packing.
    // Returns the value to load into a per-bank delay counter such that
    // the DDR gap between a command in start_slot and a command in end_slot
    // meets or exceeds delay_nCK DDR clock cycles.
    //
    // UberDDR3 uses registered eligibility (counter_d==0 → register → fire next cycle),
    // giving gap = 4*(k+1) + end_slot - start_slot for k >= 0.
    // UberDDR4 fires directly when counter_q <= 1 (no registered pipeline), giving:
    //   k=0 : fire at M+1, gap = 4 + end_slot - start_slot
    //   k>=2: fire at M+k, gap = 4*k + end_slot - start_slot
    //   (k=1 fires at M+1, same as k=0 due to <= 1 check)
    // So for k >= 1 returned by the DDR3-style formula, we add 1 to compensate.
    function integer find_delay(input integer delay_nCK, input integer start_slot, input integer end_slot);
        integer k;
        begin
            k = 0;
            while (((4 - start_slot) + end_slot + 4*k) < delay_nCK)
                k = k + 1;
            if (k > 0) k = k + 1;
            find_delay = k;
        end
    endfunction

    // get_slot: assign each command type to one of 4 DFI slots per controller cycle
    // (DFI 3.1 §3.2). Read/Write slots derived from CL/CWL mod 4; Activate and
    // Precharge fill the remaining slots avoiding collisions.
    function integer get_slot(input [3:0] cmd);
        integer delay;
        reg [2:0] slot_number, read_slot, write_slot;
        reg [2:0] anticipate_activate_slot, anticipate_precharge_slot;
        begin
            // Read slot = (0 - CL_nCK) mod 4
            slot_number = 0;
            delay = CL_nCK;
            while (delay != 0) begin
                slot_number[1:0] = slot_number[1:0] - 1'b1;
                delay = delay - 1;
            end
            read_slot[1:0] = slot_number[1:0];

            // Write slot = (0 - CWL_nCK) mod 4
            slot_number = 0;
            delay = CWL_nCK;
            while (delay != 0) begin
                slot_number[1:0] = slot_number[1:0] - 1'b1;
                delay = delay - 1;
            end
            write_slot[1:0] = slot_number[1:0];

            // Activate slot: back-count tRCD from the higher-latency data slot
            if (CL_nCK > CWL_nCK)
                slot_number[1:0] = read_slot[1:0];
            else
                slot_number[1:0] = write_slot[1:0];
            delay = ps_to_nCK(tRCD_ps);
            while (delay != 0) begin
                slot_number[1:0] = slot_number[1:0] - 1'b1;
                delay = delay - 1;
            end
            anticipate_activate_slot[1:0] = slot_number[1:0];
            // resolve collisions with data slots
            while (anticipate_activate_slot[1:0] == write_slot[1:0] ||
                   anticipate_activate_slot[1:0] == read_slot[1:0])
                anticipate_activate_slot[1:0] = anticipate_activate_slot[1:0] - 1'b1;

            // Precharge slot: first remaining slot
            anticipate_precharge_slot = 0;
            while (anticipate_precharge_slot[1:0] == write_slot[1:0] ||
                   anticipate_precharge_slot[1:0] == read_slot[1:0] ||
                   anticipate_precharge_slot[1:0] == anticipate_activate_slot[1:0])
                anticipate_precharge_slot[1:0] = anticipate_precharge_slot[1:0] - 1'b1;

            case (cmd)
                CMD_RD:  get_slot = $signed({30'b0, read_slot[1:0]});
                CMD_WR:  get_slot = $signed({30'b0, write_slot[1:0]});
                CMD_ACT: get_slot = $signed({30'b0, anticipate_activate_slot[1:0]});
                CMD_PRE: get_slot = $signed({30'b0, anticipate_precharge_slot[1:0]});
                default: get_slot = 0;
            endcase
        end
    endfunction

    // CL_generator: return minimum CAS Latency for the given DDR4 clock period.
    // Supports manual override via CL parameter (nonzero = use directly).
    // One entry per speed bin (JEDEC JESD79-4D Tables 172–173).
    function integer CL_generator(input integer ddr4_clk_period);
        begin
            if (CL != 0)                       CL_generator = $signed({26'b0, CL}); //manual override
            else if (ddr4_clk_period >= 1_500) CL_generator = 9;      //DDR4-1333
            else if (ddr4_clk_period >= 1_250) CL_generator = 10;     //DDR4-1600
            else if (ddr4_clk_period >= 1_071) CL_generator = 13;     //DDR4-1866
            else if (ddr4_clk_period >= 937)   CL_generator = 15;     //DDR4-2133
            else if (ddr4_clk_period >= 833)   CL_generator = 16;     //DDR4-2400
            else if (ddr4_clk_period >= 750)   CL_generator = 18;     //DDR4-2666
            else if (ddr4_clk_period >= 682)   CL_generator = 20;     //DDR4-2933
            else if (ddr4_clk_period >= 625)   CL_generator = 22;     //DDR4-3200
            else                               CL_generator = 22;
        end
    endfunction

    // CWL_generator: CAS Write Latency for 1tCK write preamble (V1).
    function integer CWL_generator(input integer ddr4_clk_period);
        begin
            if (CWL_PARAM != 0)                CWL_generator = $signed({27'b0, CWL_PARAM}); //manual override
            else if (ddr4_clk_period >= 1_500) CWL_generator = 9;      //DDR4-1333
            else if (ddr4_clk_period >= 1_250) CWL_generator = 9;      //DDR4-1600
            else if (ddr4_clk_period >= 1_071) CWL_generator = 10;     //DDR4-1866
            else if (ddr4_clk_period >= 937)   CWL_generator = 11;     //DDR4-2133
            else if (ddr4_clk_period >= 833)   CWL_generator = 12;     //DDR4-2400
            else if (ddr4_clk_period >= 750)   CWL_generator = 14;     //DDR4-2666
            else if (ddr4_clk_period >= 625)   CWL_generator = 16;     //DDR4-2933/3200
            else                               CWL_generator = 16;
        end
    endfunction

    // CL_encoding: JESD79-4D Table 13 — MR0 CAS Latency {A12, A6:A4, A2}
    // CL 9–16: sequential encoding 0–7
    // CL 17–24: non-sequential (18,20,22,24 = enc 8–11; 23,17,19,21 = enc 12–15)
    // CL 25+: A12=1, sequential encoding 16+
    function [4:0] CL_encoding(input [5:0] cl_nck);
        case (cl_nck)
            6'd9:  CL_encoding = 5'b0_000_0; // enc 0
            6'd10: CL_encoding = 5'b0_000_1; // enc 1
            6'd11: CL_encoding = 5'b0_001_0; // enc 2
            6'd12: CL_encoding = 5'b0_001_1; // enc 3
            6'd13: CL_encoding = 5'b0_010_0; // enc 4
            6'd14: CL_encoding = 5'b0_010_1; // enc 5
            6'd15: CL_encoding = 5'b0_011_0; // enc 6
            6'd16: CL_encoding = 5'b0_011_1; // enc 7
            6'd17: CL_encoding = 5'b0_110_1; // enc 13
            6'd18: CL_encoding = 5'b0_100_0; // enc 8
            6'd19: CL_encoding = 5'b0_111_0; // enc 14
            6'd20: CL_encoding = 5'b0_100_1; // enc 9
            6'd21: CL_encoding = 5'b0_111_1; // enc 15
            6'd22: CL_encoding = 5'b0_101_0; // enc 10
            6'd23: CL_encoding = 5'b0_110_0; // enc 12
            6'd24: CL_encoding = 5'b0_101_1; // enc 11
            6'd25: CL_encoding = 5'b1_000_0; // enc 16
            6'd26: CL_encoding = 5'b1_000_1; // enc 17
            6'd27: CL_encoding = 5'b1_001_0; // enc 18
            6'd28: CL_encoding = 5'b1_001_1; // enc 19
            6'd30: CL_encoding = 5'b1_010_0; // enc 20
            6'd32: CL_encoding = 5'b1_010_1; // enc 21
            default: CL_encoding = 5'b0_000_0; // CL=9 fallback
        endcase
    endfunction

    // CWL_encoding: JESD79-4D Table 19 — MR2 A5:A3
    function [2:0] CWL_encoding(input [4:0] cwl_nck);
        case (cwl_nck)
            5'd9:  CWL_encoding = 3'b000;
            5'd10: CWL_encoding = 3'b001;
            5'd11: CWL_encoding = 3'b010;
            5'd12: CWL_encoding = 3'b011;
            5'd14: CWL_encoding = 3'b100;
            5'd16: CWL_encoding = 3'b101;
            5'd18: CWL_encoding = 3'b110;
            5'd20: CWL_encoding = 3'b111;
            default: CWL_encoding = 3'b000;
        endcase
    endfunction

    // WR_RTP_encoding: JESD79-4D Table 13 — MR0 {A13, A11:A9}
    function [3:0] WR_RTP_encoding(input integer wr_nck);
        case (wr_nck)
            10:      WR_RTP_encoding = 4'b0_000;
            12:      WR_RTP_encoding = 4'b0_001;
            14:      WR_RTP_encoding = 4'b0_010;
            16:      WR_RTP_encoding = 4'b0_011;
            18:      WR_RTP_encoding = 4'b0_100;
            20:      WR_RTP_encoding = 4'b0_101;
            24:      WR_RTP_encoding = 4'b0_110;
            22:      WR_RTP_encoding = 4'b0_111;
            26:      WR_RTP_encoding = 4'b1_000;
            default: begin
                //round up to nearest valid WR value
                if      (wr_nck <= 10) WR_RTP_encoding = 4'b0_000;
                else if (wr_nck <= 12) WR_RTP_encoding = 4'b0_001;
                else if (wr_nck <= 14) WR_RTP_encoding = 4'b0_010;
                else if (wr_nck <= 16) WR_RTP_encoding = 4'b0_011;
                else if (wr_nck <= 18) WR_RTP_encoding = 4'b0_100;
                else if (wr_nck <= 20) WR_RTP_encoding = 4'b0_101;
                else if (wr_nck <= 22) WR_RTP_encoding = 4'b0_111;
                else if (wr_nck <= 24) WR_RTP_encoding = 4'b0_110;
                else                   WR_RTP_encoding = 4'b1_000;
            end
        endcase
    endfunction

    // ═══════════════════════
    // §18 — Formal Properties
    // ═══════════════════════
`ifdef FORMAL
    `include "ddr4_controller_formal.vh"
`endif

endmodule
