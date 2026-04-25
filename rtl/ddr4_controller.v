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
    output wire                      o_calib_complete,
    output wire                      o_calib_error
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
                    CMD_ZQCL = 4'b1_110;

    // Packed command word bit-field positions (29 bits, see PLAN §6.4)
    localparam CMD_CS_N     = 28,
               CMD_ACT_N    = 27,
               CMD_RAS_N    = 26, //or A16 when ACT_n=0
               CMD_CAS_N    = 25, //or A15 when ACT_n=0
               CMD_WE_N     = 24, //or A14 when ACT_n=0
               CMD_ODT      = 23,
               CMD_CKE      = 22,
               CMD_RESET_N  = 21,
               CMD_BG_START = 19, //bg[1:0] at [20:19]
               CMD_BA_START = 17; //ba[1:0] at [18:17]
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

    // tRCD/tRP: worst-case speed grade per bin (Table 173)
    localparam tRCD_ps = (DDR4_CLK_PERIOD >= 1_250) ? 13_750 : //DDR4-1600 (L grade)
                         (DDR4_CLK_PERIOD >= 937)   ? 13_130 : //DDR4-1866/2133 (P grade)
                         (DDR4_CLK_PERIOD >= 833)   ? 14_160 : //DDR4-2400 (U grade)
                         (DDR4_CLK_PERIOD >= 750)   ? 13_750 : //DDR4-2666 (T grade)
                                                      13_750;  //DDR4-3200

    localparam tRP_ps  = tRCD_ps; //symmetric for standard grades
    localparam tRC_ps  = tRAS_ps + tRP_ps;
    localparam tWR_ps  = 15_000; //fixed for DDR4
    localparam tRTP_ps = max_fn(DDR4_CLK_PERIOD * 4, 7_500);

    // Bank-group-dependent timing — the key DDR4 addition
    localparam tCCD_L_ps = max_fn(DDR4_CLK_PERIOD * 5,
        (DDR4_CLK_PERIOD >= 1_250) ? 6_250 :  //DDR4-1600
        (DDR4_CLK_PERIOD >= 937)   ? 5_355 :  //DDR4-1866/2133
                                     5_000);   //DDR4-2400+
    localparam tCCD_S_nCK = 4; //always 4nCK across BGs
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

    // MRS / init timing
    localparam tMRD_nCK    = 8;
    localparam tMOD_ps     = max_fn(DDR4_CLK_PERIOD * 24, 15_000);
    localparam tZQinit_nCK = 1024;
    localparam tZQoper_nCK = 512;
    localparam tZQCS_nCK   = 128;

    // DLL lock (speed dependent)
    localparam tDLLK_nCK = (DDR4_CLK_PERIOD >= 1_071) ? 597 :
                           (DDR4_CLK_PERIOD >= 833)   ? 768 : 1024;

    // Refresh (density dependent, tRFC1 — JESD79-4D Table 131)
    localparam tRFC_ps  = (DENSITY == 16) ? 550_000 :
                          (DENSITY == 8)  ? 350_000 :
                          (DENSITY == 4)  ? 260_000 :
                                            160_000;
    localparam tREFI_ps = 7_800_000; //7.8µs, standard temp

    // Write leveling
    localparam tWLMRD_nCK   = 40;
    localparam tWLDQSEN_nCK = 25;

    // Init (shortened for sim)
    localparam POWER_ON_RESET_HIGH_ps = MICRON_SIM ? 10_000 : 200_000_000;
    localparam INITIAL_CKE_LOW_ps     = MICRON_SIM ? 10_000 : 500_000_000;
    localparam tXPR_ps = max_fn(5 * DDR4_CLK_PERIOD, tRFC_ps + 10_000);

    // ═══════════════════════════════════════
    // §4 — Command Slot Assignment
    // ═══════════════════════════════════════
    localparam integer READ_SLOT      = get_slot(CMD_RD);
    localparam integer WRITE_SLOT     = get_slot(CMD_WR);
    localparam integer ACTIVATE_SLOT  = get_slot(CMD_ACT);
    localparam integer PRECHARGE_SLOT = get_slot(CMD_PRE);

    // ═══════════════════════════════════════════════════════════════
    // §5 — Computed Delay Counters (controller clock cycles)
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
    // read-to-write turnaround (global, all banks — ODT needs to switch)
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

    // ROM delay counter width — enough for longest init timer
    localparam DELAY_COUNTER_WIDTH = 20;

    // ════════════════════════════════════════════════════
    // §6 — Mode Register Construction
    // JEDEC JESD79-4D Tables 13–31, Appendix B
    // ════════════════════════════════════════════════════
    localparam[4:0] cl_enc  = CL_encoding(CL_nCK[5:0]);
    localparam[2:0] cwl_enc = CWL_encoding(CWL_nCK[4:0]);
    localparam[3:0] wr_enc  = WR_RTP_encoding(WR_nCK);

    // MR0: BL8, CAS Latency, DLL Reset, Write Recovery (Table 13)
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

    // MR1: DLL, RTT_NOM, output driver, write leveling (Table 16)
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

    // MR2: CWL, RTT_WR (Table 19)
    localparam[13:0] MR2 = {
        1'b0,             //A13: reserved
        1'b0,             //A12: Write CRC = off
        RTT_WR,           //A11,A10:A9
        1'b0,             //A8: reserved
        2'b00,            //A7:A6: LP ASR = manual normal
        cwl_enc,          //A5:A3: CWL
        3'b000            //A2:A0: reserved
    };

    // MR3: MPR (Table 22)
    localparam[13:0] MR3_MPR_DIS = 14'b00_00_000_0_0_0_00_00;
    localparam[13:0] MR3_MPR_EN  = 14'b00_00_000_0_0_1_00_00; //A2=1

    // MR4: preamble, temperature (Table 26) — all defaults for V1
    localparam[13:0] MR4 = 14'b00_0_000_00_0_0_0_000;

    // MR5: DM, DBI, RTT_PARK (Table 28)
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

    // MR6: VrefDQ, tCCD_L (Table 31)
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

    // MRS select — {BG0, BA1, BA0} per JESD79-4D Table 10
    localparam[2:0] MR0_SEL = 3'b000, MR1_SEL = 3'b001, MR2_SEL = 3'b010,
                    MR3_SEL = 3'b011, MR4_SEL = 3'b100, MR5_SEL = 3'b101,
                    MR6_SEL = 3'b110;

    // ═══════════════════════════════════════
    // §7 — Address Mapping
    // ═══════════════════════════════════════
    // COL_LOW = burst-alignment bits removed from WB address (defined in params above)
    // WB_ADDR_BITS = ROW_BITS + BG_BITS + BA_BITS + COL_BITS - COL_LOW (defined above)
    //
    // ADDR_MAPPING=0: {row, bg, ba, col} — legacy sequential
    // ADDR_MAPPING=1: {row, ba, col_hi, bg, col_lo=0} — BG-interleaved (default)
    //   Sequential WB accesses hit different bank groups → exploit tCCD_S over tCCD_L
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

    // ROM / init sequence (logic added in Phase 2)
    reg[DELAY_COUNTER_WIDTH-1:0] delay_counter;
    reg delay_counter_is_zero;
    reg pause_counter;

    // ── Static outputs (Phase 1 stubs) ──
    assign o_wb_ack = 1'b0; // driven by read ACK pipeline in Phase 5
    assign o_dfi_init_start = 1'b0; // driven by ROM controller in Phase 2
    assign o_calib_complete = 1'b0; // driven by training pump in Phase 7
    assign o_calib_error = 1'b0;

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
            o_wb_stall <= 1'b1;
            o_wb_data  <= {WB_DATA_BITS{1'b0}};
            reset_done <= 1'b0;
            bank_status_q <= {NUM_BANKS{1'b0}};
            delay_counter <= {DELAY_COUNTER_WIDTH{1'b0}};
            delay_counter_is_zero <= 1'b1;
            pause_counter <= 1'b0;
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
        end else begin
            // Phase 2: ROM controller + DFI mapping
            // Phase 4: Command scheduler + delay counters
            // Phase 5: Read ACK pipeline + refresh
            // Phase 7: Training command pump
        end
    end

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

    // find_delay: minimum controller cycles between a command in start_slot
    // and a command in end_slot, given a required gap of delay_nCK DDR cycles.
    // The actual DDR gap is: (4 - start_slot) + end_slot + 4*k
    function integer find_delay(input integer delay_nCK, input integer start_slot, input integer end_slot);
        integer k;
        begin
            k = 0;
            while (((4 - start_slot) + end_slot + 4*k) < delay_nCK)
                k = k + 1;
            find_delay = k;
        end
    endfunction

    // get_slot: assign each command type to one of 4 SERDES slots per controller cycle.
    // Read/Write slots are derived from CL/CWL mod 4; Activate and Precharge fill
    // the remaining slots avoiding collisions.
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

    // CL_encoding: 5-bit scattered MR0 CAS Latency field {A12, A6, A5, A4, A2}
    function [4:0] CL_encoding(input [5:0] cl_nck);
        case (cl_nck)
            6'd9:  CL_encoding = 5'b0_000_0;
            6'd10: CL_encoding = 5'b0_001_0;
            6'd11: CL_encoding = 5'b0_010_0;
            6'd12: CL_encoding = 5'b0_011_0;
            6'd13: CL_encoding = 5'b0_100_0;
            6'd14: CL_encoding = 5'b0_101_0;
            6'd15: CL_encoding = 5'b0_110_0;
            6'd16: CL_encoding = 5'b0_111_0;
            6'd17: CL_encoding = 5'b1_000_0;
            6'd18: CL_encoding = 5'b1_001_0;
            6'd19: CL_encoding = 5'b1_010_0;
            6'd20: CL_encoding = 5'b1_011_0;
            6'd21: CL_encoding = 5'b1_100_0;
            6'd22: CL_encoding = 5'b1_101_0;
            6'd23: CL_encoding = 5'b1_110_0;
            6'd24: CL_encoding = 5'b1_111_0;
            6'd25: CL_encoding = 5'b0_000_1;
            6'd26: CL_encoding = 5'b0_001_1;
            6'd27: CL_encoding = 5'b0_010_1;
            6'd28: CL_encoding = 5'b0_011_1;
            6'd30: CL_encoding = 5'b0_100_1;
            6'd32: CL_encoding = 5'b0_101_1;
            default: CL_encoding = 5'b0_000_0; //CL=9 fallback
        endcase
    endfunction

    // CWL_encoding: MR2 A5:A3 (3-bit field)
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

    // WR_RTP_encoding: MR0 {A13, A11, A10, A9}
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
