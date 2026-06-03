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
// Copyright (C) 2026  Angelo Jacobo
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
    // Clock periods in ps
    //   CONTROLLER_CLK_PERIOD = DDR4_CLK_PERIOD * 4 (1/4 rate controller)
    //   DDR4_CLK_PERIOD: 1250ps=DDR4-1600, 1071ps=DDR4-1866, 937ps=DDR4-2133, 833ps=DDR4-2400
    parameter CONTROLLER_CLK_PERIOD = 3_333,
              DDR4_CLK_PERIOD = 833,
    // DDR4 device data width: 4, 8, or 16
    //   4  = x4  (2 chips per byte lane, no DM, BG_BITS=2)
    //   8  = x8  (1 chip per byte lane, DM enabled, BG_BITS=2)
    //   16 = x16 (1 chip = 2 byte lanes, DM enabled, BG_BITS=1)
              DEVICE_WIDTH = 8,
    // Row address width: 14-17 (Refer to JESD79 Table 4-7)
              ROW_BITS = 16,
    // Column address width: 10 
              COL_BITS = 10,
    // Number of 8-bit byte lanes 
              BYTE_LANES = 2,
    // Device density in Gb: 2, 4, 8, or 16
              DENSITY = 8,
    // Set to 1 when simulating with Micron DDR4 model (shortens power on duration)
    parameter[0:0] MICRON_SIM = 0,
    // Address mapping:
    //   0 = sequential {row, bg, ba, col}
    //   1 = BG-interleaved {row, ba, col_hi, bg, col_lo} (recommended, every back-to-back read or write uses tCCD_S)
    parameter[1:0] ADDR_MAPPING = 1,
    // On-die termination during WRITE
    //   RTT_NOM  (MR1 A10:A8): 000=off, 001=RZQ/4, 010=RZQ/2, 011=RZQ/6,
    //                           100=RZQ/1, 101=RZQ/5, 110=RZQ/3, 111=RZQ/7
    //   RTT_WR   (MR2 A11:A9): 000=off, 001=RZQ/2, 010=RZQ/1, 011=Hi-Z,
    //                           100=RZQ/3
    //   RTT_PARK (MR5 A8:A6):  000=off, 001=RZQ/4, 010=RZQ/2, 011=RZQ/6,
    //                           100=RZQ/1, 101=RZQ/5, 110=RZQ/3, 111=RZQ/7
    // Refer to JESD79 Section 4.1: ODT Mode Register and ODT State Table
    parameter[2:0] RTT_NOM  = 3'b001, // DRAM turns ON RTT_NOM if it sees ODT asserted (setting RTT_NOM is enough for single-rank config)
                   RTT_WR   = 3'b000, // The rank that is being written to provide termination regardless of ODT pin status
                   RTT_PARK = 3'b000, //  Default parked value when ODT is low
    // Output driver impedance during READS (MR1 A2:A1): 0=RZQ/7 (34ohm), 1=RZQ/5 (48ohm)
    parameter[0:0] DRIVE_IMP = 0,
    // CAS Latency override (0=auto from DDR4_CLK_PERIOD)
    //   Auto picks worst-case CL that works with ALL speed bins (JESD79-4D Tables 147-150):
    //   DDR4-1600=12, DDR4-1866=14, DDR4-2133=16, DDR4-2400=18
    //   Override with a lower value for faster bins (e.g. CL=16 for DDR4-2400R)
    parameter[5:0] CL = 0,
    // CAS Write Latency override (0=auto from DDR4_CLK_PERIOD)
    //   Auto values from JESD79-4D Table 21 (1tCK write preamble):
    //   DDR4-1600=9, DDR4-1866=10, DDR4-2133=11, DDR4-2400=12
    parameter[4:0] CWL = 0,
    // The next parameters act more like localparams but are here to simplify port declarations 
    parameter SERDES_RATIO = 4, // 4:1 controller
              BA_BITS = 2, // bank address (always 2 for DDR4)
              BG_BITS = (DEVICE_WIDTH == 16) ? 1 : 2, //JESD79-4D Table 4
              DQ_BITS = 8, //always 8 (byte-lane granularity)
              NUM_BG = (1 << BG_BITS),
              NUM_BANKS = NUM_BG * (1 << BA_BITS),
              DFI_DATA_WIDTH = 2 * DQ_BITS * BYTE_LANES, //per DFI phase 
              WB_DATA_BITS = DFI_DATA_WIDTH * SERDES_RATIO,
              WB_SEL_BITS = WB_DATA_BITS / 8,
              COL_LOW = $clog2(SERDES_RATIO * 2), //burst-alignment: BL8 covers 8 columns per write/read
              WB_ADDR_BITS = ROW_BITS + BG_BITS + BA_BITS + COL_BITS - COL_LOW,
              CMD_LEN = 29, //packed command word width
              // DFI training timing (PHY-defined, override if PHY is swapped)
              T_RDLVL_EN  = 4,   // min DFI clks: rdlvl_en -> first READ
              T_RDLVL_RR  = 16,  // min DFI clks between training READs
              T_WRLVL_EN  = 4,   // min DFI clks: wrlvl_en -> first strobe
              T_WRLVL_WW  = 16,  // min DFI clks between strobe pulses
              // DFI training timing (MC-defined, override for longer PHY write/read leveling)
              T_RDLVL_MAX     = 4096, // timeout (DFI clks) for rdlvl_resp
              T_WRLVL_MAX     = 4096, // timeout (DFI clks) for wrlvl_resp
              CALIB_RETRY_MAX = 3     // retries per training phase before failure
) (
    input wire i_controller_clk, // Controller clock with CONTROLLER_CLK_PERIOD
    input wire i_rst_n, // Active-low reset

    // Wishbone B4 Pipelined Interface
    input wire                       i_wb_cyc,   // Bus cycle active (held for entire burst)
    input wire                       i_wb_stb,   // Transfer strobe (qualifies we/addr/data/sel)
    input wire                       i_wb_we,    // Write enable (1=write, 0=read)
    input wire[WB_ADDR_BITS-1:0]     i_wb_addr,  // Byte address (burst-aligned)
    input wire[WB_DATA_BITS-1:0]     i_wb_data,  // Write data from master
    input wire[WB_SEL_BITS-1:0]      i_wb_sel,   // Byte-lane select (1 bit per byte of write data)
    output reg                       o_wb_stall, // Pipeline stall (high when slave cannot accept new request)
    output wire                      o_wb_ack,   // Transfer acknowledge to master
    output reg[WB_DATA_BITS-1:0]     o_wb_data,  // Read data to master

    // DFI 3.1 Control Interface 
    output reg[SERDES_RATIO*17-1:0]             o_dfi_address,    // DRAM address bus (per-phase)
    output reg[SERDES_RATIO*BA_BITS-1:0]        o_dfi_bank,       // DRAM bank address (per-phase)
    output reg[SERDES_RATIO*BG_BITS-1:0]        o_dfi_bg,         // DRAM bank group (per-phase, DDR4)
    output reg[SERDES_RATIO-1:0]                o_dfi_cs_n,       // Chip select (per-phase)
    output reg[SERDES_RATIO-1:0]                o_dfi_act_n,      // Activate (per-phase, DDR4)
    output reg[SERDES_RATIO-1:0]                o_dfi_ras_n,      // Row address strobe / A16 (per-phase)
    output reg[SERDES_RATIO-1:0]                o_dfi_cas_n,      // Column address strobe / A15 (per-phase)
    output reg[SERDES_RATIO-1:0]                o_dfi_we_n,       // Write enable / A14 (per-phase)
    output reg[SERDES_RATIO-1:0]                o_dfi_cke,        // Clock enable (per-phase)
    output reg[SERDES_RATIO-1:0]                o_dfi_odt,        // On-die termination (per-phase)
    output reg[SERDES_RATIO-1:0]                o_dfi_reset_n,    // DRAM reset (per-phase)

    // DFI 3.1 Write Data Interface
    output reg[SERDES_RATIO*DFI_DATA_WIDTH-1:0] o_dfi_wrdata,     // Write data to PHY
    output reg[SERDES_RATIO-1:0]                o_dfi_wrdata_en,  // Write data enable (triggers PHY write path)
    output reg[SERDES_RATIO*(DFI_DATA_WIDTH/8)-1:0] o_dfi_wrdata_mask, // Write data byte mask

    // DFI 3.1 Read Data Interface 
    input wire[SERDES_RATIO*DFI_DATA_WIDTH-1:0] i_dfi_rddata,       // Read data from PHY
    input wire[SERDES_RATIO-1:0]                i_dfi_rddata_valid, // Read data valid (PHY asserts with data)
    output reg[SERDES_RATIO-1:0]                o_dfi_rddata_en,    // Read data enable (MC tells PHY read is expected)

    // DFI 3.1 Status Interface 
    output wire                      o_dfi_init_start,    // MC requests PHY initialization
    input wire                       i_dfi_init_complete, // PHY signals initialization complete

    // DFI 3.1 Training Interface: MC -> PHY
    output reg                       o_dfi_rdlvl_en,      // Read data eye training enable
    output reg                       o_dfi_rdlvl_gate_en, // Read DQS gate training enable
    output reg                       o_dfi_wrlvl_en,      // Write leveling enable
    output reg                       o_dfi_wrlvl_strobe,  // DQS strobe for write leveling
    output reg[3:0]                  o_dfi_lvl_pattern,   // Training pattern selector (DDR4 MPR encoding)
    output reg                       o_dfi_lvl_periodic,  // Periodic vs. initial training flag

    // DFI 3.1 Training Interface: PHY -> MC
    input wire[BYTE_LANES-1:0]       i_dfi_rdlvl_resp,     // Read training done (per byte lane)
    input wire[BYTE_LANES-1:0]       i_dfi_wrlvl_resp,     // Write leveling done (per byte lane)
    input wire                       i_dfi_rdlvl_req,      // PHY requests read data eye training
    input wire                       i_dfi_rdlvl_gate_req, // PHY requests gate training
    input wire                       i_dfi_wrlvl_req,      // PHY requests write leveling

    // Status
    output reg                       o_calib_complete, // All training phases finished successfully
    output reg                       o_calib_error,    // Training failed after retries

    // Debug status (lightweight assigns for CSR readback)
    output wire [3:0]                o_calib_state,
    output wire                      o_stage1_pending,
    output wire                      o_stage2_pending,
    output wire                      o_stage2_we,
    output wire                      o_refresh_idle,
    output wire [NUM_BANKS-1:0]      o_bank_status
);

    // =====================================================================
    // DDR4 Command Encoding
    // JEDEC JESD79-4D Table 35: {ACT_n, RAS_n/A16, CAS_n/A15, WE_n/A14}
    // Each 4-bit code uniquely identifies a DRAM command on the bus.
    // ACT_n=0 distinguishes ACTIVATE from all other commands.
    // =====================================================================
    localparam[3:0] CMD_MRS  = 4'b1_000,
                    CMD_REF  = 4'b1_001,
                    CMD_PRE  = 4'b1_010,
                    CMD_ACT  = 4'b0_000, //ACT_n=0: {RAS,CAS,WE} carry row addr A16:A14
                    CMD_WR   = 4'b1_100,
                    CMD_RD   = 4'b1_101,
                    CMD_NOP  = 4'b1_111,
                    CMD_ZQCL = 4'b1_110,
                    CMD_DES  = 4'b1_111; //same as NOP, cs_n=1 makes it DES

    // The init sequence is driven by a small ROM (steps 0-32). Each ROM
    // entry is a 32-bit instruction word. The upper 5 bits are a control
    // field that selects what CKE, RESET_n, and A10 look like during
    // that step, and whether the lower 20 bits are a delay count.
    // The presets below are the only combinations the ROM uses:
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

    // MRS select -- {BG0, BA1, BA0}
    localparam[2:0] MRS_MR0 = 3'b000, MRS_MR1 = 3'b001, MRS_MR2 = 3'b010,
                    MRS_MR3 = 3'b011, MRS_MR4 = 3'b100, MRS_MR5 = 3'b101,
                    MRS_MR6 = 3'b110;

    // Training command pump FSM states (DFI 3.1 training sequence).
    // The pump walks through three phases in order:
    //   1. Gate training  (GATE_EN -> GATE_READ -> GATE_WAIT -> GATE_EXIT)
    //      PHY learns when read-data-valid window opens.
    //   2. Eye training   (EYE_EN -> EYE_READ -> EYE_WAIT -> EYE_EXIT)
    //      PHY centres the sampling clock within the data eye.
    //   3. Write leveling (WL_EN -> WL_STROBE -> WL_WAIT -> WL_EXIT)
    //      PHY aligns DQS to CK at the DRAM.
    // Each phase retries up to CALIB_RETRY_MAX times on timeout.
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

    // Each DFI phase gets a 29-bit command word (cmd_d[0..3]).
    // These constants name the bit positions so the scheduler can
    // build commands by setting individual fields. The words are
    // unpacked into the flat o_dfi_* output vectors further below.
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
     * DDR4 Timing Parameters
     *
     * Minimum time gaps the DRAM requires between commands.
     * Values are in picoseconds unless suffixed _nCK (DDR4
     * clock cycles). ps_to_nCK() converts ps to DDR4 clocks;
     * The scheduler keeps a countdown counter per bank /
     * bank-group; a command fires only after its counter
     * reaches zero.
     ************************************************************/

    // ----- Latency (JESD79-4D Tables 147-153) -----
    // CL  : CAS Read Latency  -- clocks from READ command to first data out
    // CWL : CAS Write Latency -- clocks from WRITE command to first data in
    localparam integer CL_nCK  = CL_generator(DDR4_CLK_PERIOD);
    localparam integer CWL_nCK = CWL_generator(DDR4_CLK_PERIOD);

    // ----- Row Timing (JESD79-4D Tables 147-153) -----
    // tRAS : ACT-to-PRE -- minimum time a row must stay open
    localparam tRAS_ps = (DDR4_CLK_PERIOD >= 1_250) ? 35_000 : //DDR4-1600
                         (DDR4_CLK_PERIOD >= 1_071) ? 34_000 : //DDR4-1866
                         (DDR4_CLK_PERIOD >= 937)   ? 33_000 : //DDR4-2133
                                                      32_000;  //DDR4-2400+
    // tRCD : ACT-to-READ/WRITE -- row activate to column access delay
    // tRP  : PRE command period -- time to close a row before opening another
    // Derived from CL since CL=nRCD=nRP for all JEDEC speed bins
    // (Tables 147-153). The auto CL values match the worst-case nRCD at
    // each speed grade, so CL*tCK satisfies tRCD for any DDR4 part
    // (including cross-speed scenarios like DDR4-2400 running at 1600).
    localparam tRCD_ps = CL_nCK * DDR4_CLK_PERIOD;
    localparam tRP_ps  = tRCD_ps;
    // tRC  : ACT-to-ACT (same bank) = tRAS + tRP
    localparam tRC_ps  = tRAS_ps + tRP_ps;

    // ----- Column / Read-Write Timing (JESD79-4D Tables 172-173) -----
    // tWR  : Write Recovery -- internal write completion before PRE
    localparam tWR_ps  = 15_000; // always 15ns for DDR4
    localparam integer WR_nCK = ps_to_nCK(tWR_ps);
    // tRTP : READ-to-PRE -- minimum gap from read to precharge
    localparam tRTP_ps = max_fn(DDR4_CLK_PERIOD * 4, 7_500); // max(4nCK, 7.5ns)

    // ----- Bank-Group Timing (JESD79-4D Tables 172-173) -----
    // DDR4 splits timing into "L" (same bank-group, longer) and
    // "S" (different bank-group, shorter).
    //
    // tCCD_L : CAS-to-CAS delay, same bank group
    localparam tCCD_L_ps = max_fn(DDR4_CLK_PERIOD * 5,
        (DDR4_CLK_PERIOD >= 1_250) ? 6_250 :  //DDR4-1600
        (DDR4_CLK_PERIOD >= 937)   ? 5_355 :  //DDR4-1866/2133
                                     5_000);   //DDR4-2400+
    // tCCD_S : CAS-to-CAS delay, different bank group
    localparam tCCD_S_nCK = 4; // always 4nCK
    // tWTR_L : Write-to-Read turnaround, same bank group
    localparam tWTR_L_ps = max_fn(DDR4_CLK_PERIOD * 4, 7_500); // max(4nCK, 7.5ns)
    // tWTR_S : Write-to-Read turnaround, different bank group
    localparam tWTR_S_ps = max_fn(DDR4_CLK_PERIOD * 2, 2_500); // max(2nCK, 2.5ns)

    // ----- ACT-to-ACT / Four-Activate Window (JESD79-4D Tables 172-173) -----
    // Larger pages disturb more sense amps, so JEDEC requires longer spacing.
    localparam PAGE_SIZE = (1 << COL_BITS) * DEVICE_WIDTH / 8; //bytes
    //
    // tRRD_L : ACT-to-ACT delay, same bank group
    localparam tRRD_L_ps = max_fn(DDR4_CLK_PERIOD * 4,
        (PAGE_SIZE >= 2048) ?
            ((DDR4_CLK_PERIOD >= 1_250) ? 7_500 : 6_400) :  //2KB page
        (DDR4_CLK_PERIOD >= 1_250) ? 6_000 :  //1KB or 0.5KB, DDR4-1600
        (DDR4_CLK_PERIOD >= 937)   ? 5_300 :  //1KB or 0.5KB, DDR4-1866/2133
                                     4_900);  //1KB or 0.5KB, DDR4-2400+
    // tRRD_S : ACT-to-ACT delay, different bank group
    localparam tRRD_S_ps = max_fn(DDR4_CLK_PERIOD * 4,
        (PAGE_SIZE >= 2048) ?
            ((DDR4_CLK_PERIOD >= 1_250) ? 6_000 : 5_300) :  //2KB page
        (DDR4_CLK_PERIOD >= 1_250) ? 5_000 : //1KB or 0.5KB, DDR4-1600
        (DDR4_CLK_PERIOD >= 1_071) ? 4_200 : //1KB or 0.5KB, DDR4-1866
        (DDR4_CLK_PERIOD >= 937)   ? 3_700 : //1KB or 0.5KB, DDR4-2133
                                     3_300); //1KB or 0.5KB, DDR4-2400
    // tFAW : Four-Activate Window -- max 4 ACTs within this rolling window
    localparam tFAW_nCK_min = (PAGE_SIZE >= 2048) ? 28 :
                              (PAGE_SIZE >= 1024) ? 20 : 16;
    localparam tFAW_ps = max_fn(DDR4_CLK_PERIOD * tFAW_nCK_min,
        (PAGE_SIZE >= 2048) ?
            ((DDR4_CLK_PERIOD >= 1_250) ? 35_000 : 30_000) :  //2KB
        (PAGE_SIZE >= 1024) ?
            ((DDR4_CLK_PERIOD >= 1_250) ? 25_000 :             //1KB, DDR4-1600
             (DDR4_CLK_PERIOD >= 1_071) ? 23_000 : 21_000) :  //1KB, DDR4-1866 / DDR4-2133+
            ((DDR4_CLK_PERIOD >= 1_250) ? 20_000 :             //0.5KB, DDR4-1600
             (DDR4_CLK_PERIOD >= 1_071) ? 17_000 :             //0.5KB, DDR4-1866
             (DDR4_CLK_PERIOD >= 937)   ? 15_000 :             //0.5KB, DDR4-2133
             (DDR4_CLK_PERIOD >= 833)   ? 13_000 : 10_000));   //0.5KB, DDR4-2400 / DDR4-2666+

    // ----- MRS / Mode Register Timing (JESD79-4D Table 172-173) -----
    // tMRD : MRS-to-MRS command delay
    localparam tMRD_nCK    = 8;  // 8nCK all speed grades
    // tMOD : MRS-to-non-MRS command delay
    localparam tMOD_ps     = max_fn(DDR4_CLK_PERIOD * 24, 15_000); // max(24nCK, 15ns)
    // tZQinit : ZQ calibration long (initial calibration after reset)
    localparam tZQinit_nCK = 1024;

    // ----- DLL Lock (JESD79-4D Tables 172-173) -----
    // tDLLK : DLL locking time after reset or entering self-refresh
    localparam tDLLK_nCK = (DDR4_CLK_PERIOD >= 1_071) ? 597 :  //DDR4-1600/1866
                           (DDR4_CLK_PERIOD >= 833)   ? 768 :  //DDR4-2133/2400
                                                        1024;  //DDR4-2666+

    // ----- Refresh (JESD79-4D Tables 43, 172-173) -----
    // tRFC : Refresh cycle time -- density dependent (larger = more rows to refresh)
    localparam tRFC_ps  = (DENSITY == 16) ? 550_000 : //16Gb
                          (DENSITY == 8)  ? 350_000 : //8Gb
                          (DENSITY == 4)  ? 260_000 : //4Gb
                                            160_000;  //2Gb
    // tREFI : Average periodic refresh interval (normal temp <=85°C)
    localparam tREFI_ps = 7_800_000; // 7.8us (Table 43)

    // ----- Write Leveling (JESD79-4D Tables 172-173) -----
    // tWLMRD : First DQS pulse after entering write-leveling mode
    localparam tWLMRD_nCK = 40;

    // ----- Power-On / Init Sequence (JESD79-4D Figure 7) -----
    // Shortened for simulation when MICRON_SIM=1.
    localparam POWER_ON_RESET_HIGH_ps = MICRON_SIM ? 10_000 : 200_000_000; // tPW_RESET >=200us
    localparam INITIAL_CKE_LOW_ps     = MICRON_SIM ? 10_000 : 500_000_000; // >=500us after RESET_n deassert
    // tXPR : Exit reset to first command (Tables 172-173)
    localparam tXPR_ps = max_fn(5 * DDR4_CLK_PERIOD, tRFC_ps + 10_000); // max(5nCK, tRFC+10ns)

    // =========================================
    // Command Slot Assignment (DFI 3.1)
    //
    // With SERDES_RATIO=4, one controller clock = 4 DDR clocks.
    // The DFI interface carries 4 command "slots" (0-3) per
    // controller cycle, one per DDR clock phase. Each command
    // type is assigned a fixed slot so that e.g. ACT and RD can
    // fire in the same controller cycle on different phases.
    //
    // get_slot() computes slots from CL/CWL mod 4 for RD/WR,
    // then places ACT and PRE in the remaining free slots.
    // When RD and WR land on the same slot (e.g. DDR4-1866),
    // they share it — the controller never issues both at once.
    // =========================================
    localparam [1:0] READ_SLOT      = get_slot(CMD_RD);
    localparam [1:0] WRITE_SLOT     = get_slot(CMD_WR);
    localparam [1:0] ACTIVATE_SLOT  = get_slot(CMD_ACT);
    localparam [1:0] PRECHARGE_SLOT = get_slot(CMD_PRE);

    // =================================================================
    // Computed Delay Counters (controller clock cycles)
    //
    // These are the minimum number of controller cycles the scheduler
    // must wait between two commands (e.g. ACT-> RD, WR -> PRE).
    //
    // find_delay() converts DDR-clock timing requirements into
    // controller cycles, accounting for the slot offset between
    // the two commands. For example, if ACT is in slot 3 and RD
    // is in slot 0, that 1-slot head start reduces the number of
    // full controller cycles needed to satisfy tRCD.
    // =================================================================

    // Per-bank delays
    localparam ACTIVATE_TO_WRITE_DELAY =
        find_delay(ps_to_nCK(tRCD_ps), ACTIVATE_SLOT, WRITE_SLOT);
    localparam ACTIVATE_TO_READ_DELAY =
        find_delay(ps_to_nCK(tRCD_ps), ACTIVATE_SLOT, READ_SLOT); // tRCD (see JESD79-4D Figure 68)
    localparam READ_TO_PRECHARGE_DELAY =
        find_delay(ps_to_nCK(tRTP_ps), READ_SLOT, PRECHARGE_SLOT); // tRTP (see JESD79-4D Figure 112)
    localparam WRITE_TO_PRECHARGE_DELAY =
        find_delay(CWL_nCK + 4 + ps_to_nCK(tWR_ps), WRITE_SLOT, PRECHARGE_SLOT); // WL + (BL/2) + tWR (see JESD79-4D Figure 145)
    localparam PRECHARGE_TO_ACTIVATE_DELAY =
        find_delay(ps_to_nCK(tRP_ps), PRECHARGE_SLOT, ACTIVATE_SLOT); // tRP (see JESD79-4D Figure 67)
    localparam ACTIVATE_TO_PRECHARGE_DELAY =
        find_delay(ps_to_nCK(tRAS_ps), ACTIVATE_SLOT, PRECHARGE_SLOT); // tRAS (see JESD79-4D Figure 67)
    localparam READ_TO_WRITE_DELAY =
        find_delay(CL_nCK + 4 + 2 - CWL_nCK, READ_SLOT, WRITE_SLOT); // CL + (BL/2) + (tRPST+tWPRE) - CWL (see JESD79-4D Figure 98)

    // Bank-group-dependent delays (new for DDR4)
    localparam CAS_TO_CAS_DELAY_SAME_BG =
        find_delay(ps_to_nCK(tCCD_L_ps), READ_SLOT, READ_SLOT); // tCCD_L (see JESD79-4D Figure 69)
    localparam CAS_TO_CAS_DELAY_DIFF_BG =
        find_delay(tCCD_S_nCK, READ_SLOT, READ_SLOT); // tCCD_S (see JESD79-4D Figure 69)
    localparam WRITE_TO_READ_DELAY_SAME_BG =
        find_delay(CWL_nCK + 4 + ps_to_nCK(tWTR_L_ps), WRITE_SLOT, READ_SLOT); // WL + (BL/2) + tWTR_L (see JESD79-4D Figure 74)
    localparam WRITE_TO_READ_DELAY_DIFF_BG =
        find_delay(CWL_nCK + 4 + ps_to_nCK(tWTR_S_ps), WRITE_SLOT, READ_SLOT); // WL + (BL/2) + tWTR_S (see JESD79-4D Figure 73)
    localparam ACTIVATE_TO_ACTIVATE_DELAY_SAME_BG =
        find_delay(ps_to_nCK(tRRD_L_ps), ACTIVATE_SLOT, ACTIVATE_SLOT);  // tRRD_L (see JESD79-4D Figure 71)
    localparam ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG =
        find_delay(ps_to_nCK(tRRD_S_ps), ACTIVATE_SLOT, ACTIVATE_SLOT);  // tRRD_S (see JESD79-4D Figure 71)

    // tFAW in controller cycles
    localparam TFAW_CYCLES = nCK_to_cycles(ps_to_nCK(tFAW_ps)); // tFAW (see JESD79-4D Figure 72)

    // Counter width sizing 
    localparam MAX_PRECHARGE_DELAY = max_fn(max_fn(ACTIVATE_TO_PRECHARGE_DELAY, WRITE_TO_PRECHARGE_DELAY), READ_TO_PRECHARGE_DELAY);
    localparam MAX_ACTIVATE_DELAY  = PRECHARGE_TO_ACTIVATE_DELAY; // tRRD(act-to-act) uses separate rrd_counter
    localparam MAX_WRITE_DELAY     = max_fn(ACTIVATE_TO_WRITE_DELAY, READ_TO_WRITE_DELAY); // tCCD(wr-to-wr) uses separate per-BG counter
    localparam MAX_READ_DELAY      = ACTIVATE_TO_READ_DELAY; // tCCD(rd-to-rd) / tWTR(wr-to-rd) use separate per-BG counters
    localparam MAX_CCD_DELAY       = CAS_TO_CAS_DELAY_SAME_BG;
    localparam MAX_WTR_DELAY       = WRITE_TO_READ_DELAY_SAME_BG;
    localparam MAX_RRD_DELAY       = ACTIVATE_TO_ACTIVATE_DELAY_SAME_BG;

    // -- Read/Write data enable pipeline depths (see ddr4_phy.v) --
    //
    // RDDATA_EN_PIPE_WIDTH sets the width of rddata_en_pipe_q, a
    // shift register that delays o_dfi_rddata_en by exactly the
    // number of controller cycles between issuing a READ and the
    // PHY having valid aligned data ready to capture.
    //
    // READ_DELAY = CL converted to controller cycles
    // +4 = one cycle per pipeline stage on the round-trip path:
    //   +1 controller DFI output reg   (cmd_d -> o_dfi_*)
    //   +1 OSERDESE3 cmd serializer    (DFI -> DDR4 CA pin)
    //   +1 ISERDESE3 data deserializer (DDR4 DQ -> iserdes_dq_q)
    //   +1 prev_iserdes_q register     (holds prior iserdes_dq_q so
    //       PHY can barrel-shift {cur, prev} for bit alignment)
    //
    localparam READ_DELAY = find_delay(CL_nCK, READ_SLOT, READ_SLOT);
    localparam RDDATA_EN_PIPE_WIDTH = READ_DELAY + 4;
    // Write analog: CWL -> controller cycles. No "+4", the write
    // path has no return pipeline; OSERDES DQ/DQS latency is
    // handled by the PHY's wrdata_en_shift register.
    localparam WRITE_DATA_DELAY = find_delay(CWL_nCK, WRITE_SLOT, WRITE_SLOT);

    // ROM delay counter width -- enough for longest init timer
    localparam DELAY_COUNTER_WIDTH = 20;

    // Refresh loop timer -- adjusted so REF-to-REF <= tREFI
    // The ROM loops through addresses 33->34->35->33:
    //   addr 33: PRE ALL, wait tRP
    //   addr 34: REF,     wait tRFC
    //   addr 35: NOP,     wait REFRESH_TREFI_TIMER (computed below)
    // Total period = T_RP + T_RFC + REFRESH_TREFI_TIMER + 3 controller cycles.
    // The -3 accounts for the 3 cycles consumed firing each ROM entry
    // (one cycle each for addr 33, 34, 35) before their delay counters run.
    localparam REFRESH_TREFI_TIMER = tREFI_ps / CONTROLLER_CLK_PERIOD - 3 - ps_to_cycles(tRP_ps) - ps_to_cycles(tRFC_ps);

    // ========================================================
    // Mode Register Construction
    // JEDEC JESD79-4D Tables 13-31
    // These localparams build the 14-bit MR values that the
    // init ROM writes into the DRAM's mode registers.
    // ========================================================
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
        1'b0,             //A13: Rx CTLE (with A6:A5) = vendor default
        1'b0,             //A12: Qoff = enabled
        1'b0,             //A11: TDQS = disabled
        RTT_NOM,          //A10:A8
        1'b0,             //A7: Write Leveling = off
        2'b00,            //A6:A5: Rx CTLE (with A13) = vendor default
        2'b00,            //A4:A3: AL = 0
        1'b0, DRIVE_IMP,  //A2:A1: output driver
        1'b1              //A0: DLL = on
    };
    localparam[13:0] MR1_WL_EN = {
        1'b0,             //A13: Rx CTLE (with A6:A5) = vendor default
        1'b0,             //A12: Qoff = enabled
        1'b0,             //A11: TDQS = disabled
        RTT_NOM,          //A10:A8
        1'b1,             //A7: Write Leveling = on
        2'b00,            //A6:A5: Rx CTLE (with A13) = vendor default
        2'b00,            //A4:A3: AL = 0
        1'b0, DRIVE_IMP,  //A2:A1: output driver
        1'b1              //A0: DLL = on
    };

    // MR2: CWL, RTT_WR (JESD79-4D Table 19)
    localparam[13:0] MR2 = {
        1'b0,             //A13: reserved
        1'b0,             //A12: Write CRC = off
        RTT_WR,           //A11,A10:A9
        1'b0,             //A8: reserved
        2'b11,            //A7:A6: LP ASR = ASR (auto self-refresh)
        cwl_enc,          //A5:A3: CWL
        3'b000            //A2:A0: reserved
    };

    // MR3: MPR, geardown, fine refresh (JESD79-4D Table 22)
    localparam[13:0] MR3_MPR_DIS = {
        1'b0,             //A13: reserved
        2'b00,            //A12:A11: MPR Read Format = serial
        2'b00,            //A10:A9: Write CMD Latency = 4nCK (don't-care: CRC off)
        3'b000,           //A8:A6: Fine Granularity Refresh = normal 1x
        1'b0,             //A5: Temperature sensor readout = off
        1'b0,             //A4: Per DRAM Addressability = off
        1'b0,             //A3: Geardown Mode = 1/2 rate
        1'b0,             //A2: MPR Operation = normal
        2'b00             //A1:A0: MPR page = page 0 (training pattern)
    };
    localparam[13:0] MR3_MPR_EN = {
        1'b0,             //A13: reserved
        2'b00,            //A12:A11: MPR Read Format = serial
        2'b00,            //A10:A9: Write CMD Latency = 4nCK (don't-care: CRC off)
        3'b000,           //A8:A6: Fine Granularity Refresh = normal 1x
        1'b0,             //A5: Temperature sensor readout = off
        1'b0,             //A4: Per DRAM Addressability = off
        1'b0,             //A3: Geardown Mode = 1/2 rate
        1'b1,             //A2: MPR Operation = dataflow from/to MPR
        2'b00             //A1:A0: MPR page = page 0 (training pattern)
    };

    // MR4: preamble, temperature, PPR (JESD79-4D Table 26) -- all defaults
    localparam[13:0] MR4 = {
        1'b0,             //A13: hPPR = off
        1'b0,             //A12: Write Preamble = 1 nCK
        1'b0,             //A11: Read Preamble = 1 nCK
        1'b0,             //A10: Read Preamble Training = off
        1'b0,             //A9: Self Refresh Abort = off
        3'b000,           //A8:A6: CS to CMD/ADDR Latency = disabled
        1'b0,             //A5: sPPR = off
        1'b0,             //A4: Internal Vref Monitor = off
        1'b0,             //A3: Temp Controlled Refresh = off
        1'b0,             //A2: Temp Controlled Refresh Range = normal
        1'b0,             //A1: Max Power Down Mode = off
        1'b0              //A0: MBIST PPR = off
    };

    // MR5: DM, DBI, RTT_PARK (JESD79-4D Table 28)
    localparam[0:0] DM_ENABLED = (DEVICE_WIDTH != 4); //x4 has no DM_n 
    localparam[13:0] MR5 = {
        1'b0,             //A13: reserved
        1'b0,             //A12: Read DBI = off
        1'b0,             //A11: Write DBI = off
        DM_ENABLED,       //A10: Data Mask
        1'b0,             //A9: CA Parity Persistent Error = off
        RTT_PARK,         //A8:A6
        1'b0,             //A5: ODT Input Buffer in Power Down = on
        1'b0,             //A4: C/A Parity Error Status = clear
        1'b0,             //A3: CRC Error Clear = clear
        3'b000            //A2:A0: CA Parity Latency = off
    };

    // MR6: VrefDQ, tCCD_L (JESD79-4D Tables 31-34)
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
        6'b010100         //A5:A0: VrefDQ 73% (step 20, Range 1 -- matches Xilinx MIG default)
    };

    // =========================================
    // Address Mapping
    // =========================================
    // The Wishbone address excludes the lowest COL_LOW column bits
    // (burst-aligned by the SERDES ratio), so the WB address carries
    // COL_USED column bits plus BA, BG, and ROW fields.
    //
    // ADDR_MAPPING selects how these fields are arranged in i_wb_addr:
    //
    //   MAP=0  {row, bg, ba, col}        -- sequential addressing
    //   MAP=1  {row, ba, col, bg}        -- BG-interleaved (default)
    //          BG in the lowest bits means sequential WB addresses
    //          rotate through bank groups, so back-to-back accesses
    //          use tCCD_S (4nCK) instead of tCCD_L (5-8nCK).
    //
    localparam COL_USED = COL_BITS - COL_LOW;

    // =====================================================================
    // Registers and Wires
    // =====================================================================

    reg reset_done;

    // Per-bank delay counters (_q = registered, _d = combinational next-state).
    // Saturating countdown counters: nonzero blocks the command for that bank.
    // Scheduler checks counter_q <= 1 (not == 0) because the combinational
    // decrement makes the value available one cycle early -- saves a bubble.
    reg[$clog2(MAX_PRECHARGE_DELAY):0] delay_before_precharge_counter_q [NUM_BANKS-1:0], delay_before_precharge_counter_d [NUM_BANKS-1:0];
    reg[$clog2(MAX_ACTIVATE_DELAY):0]  delay_before_activate_counter_q  [NUM_BANKS-1:0], delay_before_activate_counter_d  [NUM_BANKS-1:0];
    reg[$clog2(MAX_WRITE_DELAY):0]     delay_before_write_counter_q     [NUM_BANKS-1:0], delay_before_write_counter_d     [NUM_BANKS-1:0];
    reg[$clog2(MAX_READ_DELAY):0]      delay_before_read_counter_q      [NUM_BANKS-1:0], delay_before_read_counter_d      [NUM_BANKS-1:0];
    reg[NUM_BANKS-1:0]                 bank_status_q,                                    bank_status_d;
    reg[ROW_BITS-1:0]                  bank_active_row_q                [NUM_BANKS-1:0], bank_active_row_d                [NUM_BANKS-1:0];

    // Per-bank-group delay counters (new for DDR4).
    // These enforce the "L" vs "S" timing split across bank groups.
    reg[$clog2(MAX_CCD_DELAY):0] ccd_counter_q [NUM_BG-1:0], ccd_counter_d [NUM_BG-1:0];
    reg[$clog2(MAX_WTR_DELAY):0] wtr_counter_q [NUM_BG-1:0], wtr_counter_d [NUM_BG-1:0];
    reg[$clog2(MAX_RRD_DELAY):0] rrd_counter_q [NUM_BG-1:0], rrd_counter_d [NUM_BG-1:0];

    // Packed command slots (internal, decomposed to DFI outputs below)
    reg[CMD_LEN-1:0] cmd_d [SERDES_RATIO-1:0];

    // ROM / init sequence state
    reg[5:0] instruction_address;
    reg[DELAY_COUNTER_WIDTH-1:0] delay_counter;
    reg delay_counter_is_zero;
    reg pause_counter; // held by training pump to freeze ROM advance

    // -- Training pump state (driven by the calibration FSM) --
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

    // Decode current init ROM entry (packed {control, cmd, timer} word)
    assign rom_instruction = read_rom_instruction(instruction_address);
    assign rom_cmd_is_mrs  = (rom_instruction[26:23] == CMD_MRS);
    assign rom_use_timer   = rom_instruction[ROM_USE_TIMER];

    // CKE/RESET_n mux: use ROM value when firing, else hold previous.
    // MRS commands force both high. 
    wire init_firing  = delay_counter_is_zero && !pause_counter;
    wire init_cke     = init_firing ? (rom_cmd_is_mrs ? 1'b1 : rom_instruction[ROM_CKE])     : rom_cke_hold;
    wire init_reset_n = init_firing ? (rom_cmd_is_mrs ? 1'b1 : rom_instruction[ROM_RESET_N]) : rom_reset_n_hold;

    // -- Read/Write data enable pipelines --
    // Shift registers that track when dfi_rddata_en / dfi_wrdata_en
    // should assert. A '1' is loaded at the MSB when a RD/WR command
    // issues, then shifts right each cycle until it falls out at bit 0.
    reg[RDDATA_EN_PIPE_WIDTH-1:0] rddata_en_pipe_q;
    reg[WRITE_DATA_DELAY:0]       wrdata_en_pipe_q;
    reg                           write_ack_q;
    reg                           read_ack_q;

    // -- Static outputs --
    assign o_wb_ack = i_rst_n && reset_done && (write_ack_q || read_ack_q);
    assign o_dfi_init_start = ~reset_done; // request PHY init until ROM completes

    // =====================================================================
    // Address Decode
    // WB address -> {row, bg, ba, col} per ADDR_MAPPING
    // ADDR_MAPPING=0: {row, bg, ba, col} -- sequential
    // ADDR_MAPPING=1: {row, ba, col, bg} -- BG-interleaved (default)
    // =====================================================================
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
            // BG at lowest position -- sequential WB accesses cycle through bank groups,
            // exploiting tCCD_S < tCCD_L for streaming workloads (JESD79-4D)
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

    // -- Stage 1 pipeline registers --
    // Stage 1 accepts a new WB request. It latches the decoded address
    // fields and waits for Stage 2 to become available. This two-stage
    // pipeline lets the controller accept a new WB request while Stage 2
    // is still working through PRE->ACT->RD/WR for the previous one.
    reg                      stage1_pending;
    reg                      stage1_we;
    reg[WB_DATA_BITS-1:0]    stage1_data;
    reg[WB_SEL_BITS-1:0]     stage1_dm;
    reg[COL_BITS-1:0]        stage1_col;
    reg[BA_BITS-1:0]         stage1_ba;
    reg[BG_BITS-1:0]         stage1_bg;
    reg[ROW_BITS-1:0]        stage1_row;
    reg[BG_BITS+BA_BITS-1:0] stage1_bank;
    reg[BG_BITS+BA_BITS-1:0] stage1_next_bank; // anticipation: pre-ACT target
    reg[ROW_BITS-1:0]        stage1_next_row;

    // -- Stage 2 pipeline registers --
    // Stage 2 holds the request currently being scheduled against the
    // bank state machine. It drives PRE/ACT/WR/RD until complete.
    reg                      stage2_pending;
    reg                      stage2_we;
    reg[WB_DATA_BITS-1:0]    stage2_data;
    reg[WB_SEL_BITS-1:0]     stage2_dm;
    // Write data delay pipeline -- mirrors wrdata_en_pipe_q structure
    // (same depth, shift direction, load position) so data and enable
    // stay aligned. Prevents stage2_data overwrite on back-to-back writes.
    reg[WB_DATA_BITS-1:0]    wr_data_pipe_q [WRITE_DATA_DELAY:0];
    reg[WB_SEL_BITS-1:0]     wr_dm_pipe_q   [WRITE_DATA_DELAY:0];
    reg[COL_BITS-1:0]        stage2_col;
    reg[BA_BITS-1:0]         stage2_ba;
    reg[BG_BITS-1:0]         stage2_bg;
    reg[ROW_BITS-1:0]        stage2_row;
    reg[BG_BITS+BA_BITS-1:0] stage2_bank;

    // -- tFAW sliding window -- blocks 5th ACT within the window.
    // Four timestamps record when the last four ACTs happened.
    // If the oldest one hasn't expired, a new ACT is blocked.
    reg[$clog2(TFAW_CYCLES):0] activate_timestamp_q [3:0], activate_timestamp_d [3:0];
    reg[1:0]                   activate_index_q;

    // The refresh loop (ROM addrs 33-35) fires PRE ALL, REF, then loads
    // the tREFI delay and jumps back to addr 33. While that tREFI timer
    // counts down, the ROM sits at addr 33 doing nothing — that is the
    // "refresh_idle" window where the scheduler is free to issue user
    // commands. Once the timer expires, refresh fires again.
    //   refresh_idle   = sitting at REF_START, timer still counting (safe)
    //   refresh_active = init done AND not in that idle window (blocked)
    wire refresh_idle = (instruction_address == ROM_ADDR_REF_START)
                        && !delay_counter_is_zero;
    wire refresh_active = reset_done && !refresh_idle;

    // Is the current ROM instruction a PRECHARGE ALL?
    wire rom_is_prea = (rom_instruction[26:23] == CMD_PRE)
                       && rom_instruction[ROM_A10];

    // Any bank still has a pending write/read that hasn't completed tWR/tRTP?
    // If so, PRE ALL must wait to avoid JEDEC timing violations.
    wire [NUM_BANKS-1:0] precharge_pending_vec;
    genvar gpk;
    generate
        for (gpk = 0; gpk < NUM_BANKS; gpk = gpk + 1) begin : gen_pre_pending
            assign precharge_pending_vec[gpk] = |delay_before_precharge_counter_q[gpk];
        end
    endgenerate
    wire any_precharge_pending = |precharge_pending_vec;

    // ROM PRE ALL gating logic (ROM addrs 19, 30, 33 issue PRE ALL).
    //
    // Problem: during normal operation, the scheduler may have just
    // issued a RD/WR whose tRAS or tWR timer hasn't expired yet.
    // If the refresh loop's PRE ALL fires immediately, it would
    // violate that bank's timing. (During init this doesn't apply
    // because no user commands are in flight, so reset_done gates it.)
    //
    // Solution: rom_prea_hold stalls the ROM when a PRE ALL is next
    // but any bank still has a nonzero precharge counter. This holds
    // off the entire ROM (via rom_firing) until it's safe.
    //
    // rom_prea_hold     = PRE ALL is next, but a bank precharge timer is still running
    // rom_firing        = ROM ready to issue (delay done, not paused, not held)
    // rom_precharge_all = PRE ALL actually fires this cycle
    wire rom_prea_hold = rom_is_prea && any_precharge_pending && reset_done;
    wire rom_firing = delay_counter_is_zero && !pause_counter
                      && (!reset_done || instruction_address >= ROM_ADDR_REF_START)
                      && !rom_prea_hold;
    wire rom_precharge_all = rom_firing && rom_is_prea;

    wire wb_accept = i_wb_cyc && i_wb_stb && !o_wb_stall;

    // -- Scheduler decision flags (set in combinational block, used by sequential) --
    reg cmd_odt;
    reg stage2_update;
    reg sched_precharge;
    reg sched_activate;
    reg sched_write;
    reg sched_read;
    reg sched_anticipate;

    // tFAW check: JEDEC allows at most 4 ACTs in any tFAW window.
    // activate_timestamp_q[3:0] is a circular buffer of 4 countdown
    // timers. Each time an ACT fires, the slot at activate_index_q
    // is loaded with TFAW_CYCLES and the index advances (wraps 0-3).
    // The current index therefore points to the oldest ACT's timer.
    // If that timer is still nonzero, 4 ACTs are already in-flight
    // within the window, so a 5th is blocked.
    wire tfaw_blocked = |activate_timestamp_q[activate_index_q];

    // Next-bank BG extraction for anticipation
    wire[BG_BITS-1:0] stage1_next_bg = stage1_next_bank[BG_BITS+BA_BITS-1:BA_BITS];

    // cmd_d is a fixed 29-bit packed command word where BG always
    // occupies bits [20:19] (2 bits). For x16 devices BG_BITS=1,
    // so we zero-pad to 2 bits to fill the field.
    wire [1:0] stage2_bg_padded = stage2_bg;
    wire [1:0] stage1_next_bg_padded = stage1_next_bg;

    // DDR4 address bus is A[16:0] = 17 bits. The ACT command puts
    // the full row address on these pins. ROW_BITS may be smaller
    // (e.g. 14-16 depending on density), so we zero-pad to 17 bits.
    wire[16:0] stage2_row_padded     = {{(17-ROW_BITS){1'b0}}, stage2_row};
    wire[16:0] stage1_next_row_padded = {{(17-ROW_BITS){1'b0}}, stage1_next_row};

    // =====================================================================
    // Combinational Counter Decrement
    // Saturating decrement: (|counter) is 1 when nonzero, 0 when zero.
    // The scheduler (below) overrides _d values to reload counters
    // whenever it issues a command.
    // =====================================================================
    integer ci;
    always @* begin
        for (ci = 0; ci < NUM_BANKS; ci = ci + 1) begin
            delay_before_precharge_counter_d[ci] = delay_before_precharge_counter_q[ci] - (|delay_before_precharge_counter_q[ci]);
            delay_before_activate_counter_d[ci] = delay_before_activate_counter_q[ci] - (|delay_before_activate_counter_q[ci]);
            delay_before_write_counter_d[ci] = delay_before_write_counter_q[ci] - (|delay_before_write_counter_q[ci]);
            delay_before_read_counter_d[ci] = delay_before_read_counter_q[ci] - (|delay_before_read_counter_q[ci]);
            bank_status_d[ci] = bank_status_q[ci];
            bank_active_row_d[ci] = bank_active_row_q[ci];
        end
        for (ci = 0; ci < NUM_BG; ci = ci + 1) begin
            ccd_counter_d[ci] = ccd_counter_q[ci] - (|ccd_counter_q[ci]);
            wtr_counter_d[ci] = wtr_counter_q[ci] - (|wtr_counter_q[ci]);
            rrd_counter_d[ci] = rrd_counter_q[ci] - (|rrd_counter_q[ci]);
        end
        for (ci = 0; ci < 4; ci = ci + 1) begin
            activate_timestamp_d[ci] = activate_timestamp_q[ci] - (|activate_timestamp_q[ci]);
        end

        // ROM PRE ALL closes all banks (refresh loop addr 33)
        if (rom_precharge_all) begin
            for (ci = 0; ci < NUM_BANKS; ci = ci + 1) begin
                bank_status_d[ci] = 1'b0;
            end
        end

        // =============================================================
        // Stage 2 Command Scheduling + Counter Loading
        // Uses counter_q <= 1 optimization: fire one cycle early
        // because the combinational decrement has already applied.
        // =============================================================
        cmd_odt = 1'b0;
        stage2_update = !stage2_pending;
        sched_precharge = 1'b0;
        sched_activate  = 1'b0;
        sched_write     = 1'b0;
        sched_read      = 1'b0;
        sched_anticipate = 1'b0;

        // Stage 2 scheduling: the request has a target {bank, row, we}.
        // DDR4 requires a bank to be ACTIVATEd on the correct row before
        // any RD/WR.  Three mutually exclusive cases arise:
        //   1. Bank open on WRONG row  -> close it first  (PRECHARGE)
        //   2. Bank closed             -> open target row  (ACTIVATE)
        //   3. Bank open on RIGHT row  -> issue data cmd   (WRITE / READ)
        // Each case loads JEDEC-mandated delay counters so the next
        // command to that bank (or bank group) cannot fire too early.
        if (stage2_pending && refresh_idle) begin

            // ---- Case 1: row miss ----
            // Bank is already active but on a different row than the one
            // we need.  Issue a single-bank PRECHARGE to close it.
            // Guard: delay_before_precharge_counter_q <= 1 ensures tRAS
            // (and tRTP/tWR if a RD/WR loaded it earlier) has elapsed.
            // After PRE, load tRP into the activate counter so the next
            // ACTIVATE to this bank waits the required PRECHARGE-to-
            // ACTIVATE interval before re-opening the bank.
            if (bank_status_q[stage2_bank]
                && (bank_active_row_q[stage2_bank] != stage2_row)
                && (delay_before_precharge_counter_q[stage2_bank] <= 1)) begin
                sched_precharge = 1'b1;
                delay_before_activate_counter_d[stage2_bank] = PRECHARGE_TO_ACTIVATE_DELAY[$clog2(MAX_ACTIVATE_DELAY):0];
                bank_status_d[stage2_bank] = 1'b0;
            end

            // ---- Case 2: bank idle -> ACTIVATE ----
            // Bank is closed.  Open it on the target row.
            // Three guards must ALL be satisfied before we can fire ACT:
            //   (a) delay_before_activate_counter <= 1 : tRP from a prior
            //       PRECHARGE to THIS bank has elapsed.
            //   (b) rrd_counter[bg] <= 1 : tRRD (ACT-to-ACT within this
            //       bank group) has elapsed — JEDEC requires a minimum
            //       interval between consecutive ACTIVATEs.
            //   (c) !tfaw_blocked : no more than 4 ACTIVATEs have occurred
            //       in the last tFAW window.
            else if (!bank_status_q[stage2_bank]
                     && (delay_before_activate_counter_q[stage2_bank] <= 1)
                     && (rrd_counter_q[stage2_bg] <= 1)
                     && !tfaw_blocked) begin
                sched_activate = 1'b1;
                // tRAS: minimum time bank must stay active before it can
                // be PRECHARGEd again (JESD79-4D §4.29).
                delay_before_precharge_counter_d[stage2_bank] = ACTIVATE_TO_PRECHARGE_DELAY[$clog2(MAX_PRECHARGE_DELAY):0];
                // tRCD: minimum ACT-to-RD/WR delay.
                // "Only raise" = use MAX(current, new): if a higher delay
                // is already pending don't overwrite it with a
                // shorter value.  The `<` comparison achieves this —
                // we only load the new value when it is LARGER than what
                // is already in the counter.
                if (delay_before_write_counter_d[stage2_bank] < ACTIVATE_TO_WRITE_DELAY[$clog2(MAX_WRITE_DELAY):0]) begin
                    delay_before_write_counter_d[stage2_bank] = ACTIVATE_TO_WRITE_DELAY[$clog2(MAX_WRITE_DELAY):0];
                end
                if (delay_before_read_counter_d[stage2_bank] < ACTIVATE_TO_READ_DELAY[$clog2(MAX_READ_DELAY):0]) begin
                    delay_before_read_counter_d[stage2_bank] = ACTIVATE_TO_READ_DELAY[$clog2(MAX_READ_DELAY):0];
                end
                bank_status_d[stage2_bank] = 1'b1;
                bank_active_row_d[stage2_bank] = stage2_row;
                // tRRD — Row-to-Row Delay: minimum time between two
                // ACTIVATEs.  DDR4 has TWO tRRD values:
                //   tRRD_L (Long)  — ACT-to-ACT within the SAME bank group
                //   tRRD_S (Short) — ACT-to-ACT to a DIFFERENT bank group
                // Loop over all bank groups (`ci` is the loop iterator).
                // For the bank group that just fired ACT: unconditionally
                // load tRRD_L (same-BG, always the longer value).
                // For every OTHER bank group: only-raise to tRRD_S — load
                // tRRD_S only if larger than the current counter, so a
                // previously loaded tRRD_L is never shortened.
                for (ci = 0; ci < NUM_BG; ci = ci + 1) begin
                    if (ci[BG_BITS-1:0] == stage2_bg) begin
                        rrd_counter_d[ci] = ACTIVATE_TO_ACTIVATE_DELAY_SAME_BG[$clog2(MAX_RRD_DELAY):0];
                    end
                    else if (rrd_counter_d[ci] < ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_RRD_DELAY):0]) begin
                        rrd_counter_d[ci] = ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_RRD_DELAY):0];
                    end
                end
                // Per-bank activate delay (tRRD_S) for every OTHER bank.
                // Why needed when we already have tRRD counters above?
                // The rrd_counter is per-BANK-GROUP (4 counters for 4 BGs).
                // It blocks the next ACT to any bank in that BG. But the
                // per-bank delay_before_activate_counter is per-BANK (16
                // counters). It is checked in the ACT guard (case 2 above)
                // and provides per-bank granularity that tRRD alone cannot:
                // e.g. tRP (PRE→ACT) is loaded here too, verify each bank's 
                // counter individually.  Loading tRRD_S into every other bank's
                // activate counter is belt-and-suspenders: it guarantees
                // tRRD_S compliance even if the BG-level rrd_counter were
                // somehow bypassed.
                for (ci = 0; ci < NUM_BANKS; ci = ci + 1) begin
                    if (ci[BG_BITS+BA_BITS-1:0] != stage2_bank
                        && delay_before_activate_counter_d[ci] < ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_ACTIVATE_DELAY):0])
                        delay_before_activate_counter_d[ci] = ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_ACTIVATE_DELAY):0];
                end
                // tFAW — Four-Activate Window: record this ACT's timestamp
                // in the circular buffer so the sliding-window check can
                // block a 5th ACT within the tFAW interval.
                activate_timestamp_d[activate_index_q] = TFAW_CYCLES[$clog2(TFAW_CYCLES):0];
            end

            // ---- Case 3: row hit -> issue WRITE or READ ----
            // Bank is active and already has the correct row open.
            // No PRE/ACT needed — go straight to the data command.
            else if (bank_status_q[stage2_bank]
                     && (bank_active_row_q[stage2_bank] == stage2_row)) begin

                // WRITE path.
                // Guards: (a) tRCD elapsed (delay_before_write_counter),
                //         (b) tCCD elapsed (ccd_counter per bank group).
                // ODT (On-Die Termination) is enabled for writes — the
                // DRAM switches its termination resistors to transmit mode.
                if (stage2_we
                    && (delay_before_write_counter_q[stage2_bank] <= 1)
                    && (ccd_counter_q[stage2_bg] <= 1)) begin
                    sched_write = 1'b1;
                    cmd_odt = 1'b1;
                    stage2_update = 1'b1;
                    // tWR: minimum WR-to-PRE delay (write recovery time).
                    // Only-raise: don't shorten a pending tRAS.
                    if (delay_before_precharge_counter_d[stage2_bank] < WRITE_TO_PRECHARGE_DELAY[$clog2(MAX_PRECHARGE_DELAY):0])
                        delay_before_precharge_counter_d[stage2_bank] = WRITE_TO_PRECHARGE_DELAY[$clog2(MAX_PRECHARGE_DELAY):0];
                    // Per-BG tCCD (CAS-to-CAS) + tWTR (WR-to-RD turnaround).
                    // Same BG: unconditionally load the LONG delays.
                    // Diff BG: only-raise to SHORT delays.
                    for (ci = 0; ci < NUM_BG; ci = ci + 1) begin
                        if (ci[BG_BITS-1:0] == stage2_bg) begin
                            ccd_counter_d[ci] = CAS_TO_CAS_DELAY_SAME_BG[$clog2(MAX_CCD_DELAY):0];
                            wtr_counter_d[ci] = WRITE_TO_READ_DELAY_SAME_BG[$clog2(MAX_WTR_DELAY):0];
                        end
                        else begin
                            if (ccd_counter_d[ci] < CAS_TO_CAS_DELAY_DIFF_BG[$clog2(MAX_CCD_DELAY):0]) begin
                                ccd_counter_d[ci] = CAS_TO_CAS_DELAY_DIFF_BG[$clog2(MAX_CCD_DELAY):0];
                            end
                            if (wtr_counter_d[ci] < WRITE_TO_READ_DELAY_DIFF_BG[$clog2(MAX_WTR_DELAY):0]) begin
                                wtr_counter_d[ci] = WRITE_TO_READ_DELAY_DIFF_BG[$clog2(MAX_WTR_DELAY):0];
                            end
                        end
                    end
                end

                // READ path.
                // Guards: (a) tRCD elapsed (delay_before_read_counter),
                //         (b) tCCD elapsed (ccd_counter per bank group),
                //         (c) tWTR elapsed (wtr_counter) — must wait for
                //             any prior WRITE's data to clear the bus
                //             before driving a READ.
                // ODT stays off for reads — DRAM is in receive mode.
                else if (!stage2_we
                         && (delay_before_read_counter_q[stage2_bank] <= 1)
                         && (ccd_counter_q[stage2_bg] <= 1)
                         && (wtr_counter_q[stage2_bg] <= 1)) begin
                    sched_read = 1'b1;
                    stage2_update = 1'b1;
                    // tRTP: minimum RD-to-PRE delay (read-to-precharge).
                    // Only-raise: don't shorten a pending tRAS.
                    if (delay_before_precharge_counter_d[stage2_bank] < READ_TO_PRECHARGE_DELAY[$clog2(MAX_PRECHARGE_DELAY):0]) begin
                        delay_before_precharge_counter_d[stage2_bank] = READ_TO_PRECHARGE_DELAY[$clog2(MAX_PRECHARGE_DELAY):0];
                    end
                    // RD→WR turnaround: applies to ALL banks globally.
                    // After a READ, the DQ bus carries read data for
                    // RL + BL/2 clocks.  A WRITE cannot drive the bus
                    // until that window plus ODT settling time has passed.
                    // This is a bus-level (not bank-level) constraint.
                    for (ci = 0; ci < NUM_BANKS; ci = ci + 1) begin
                        if (delay_before_write_counter_d[ci] < READ_TO_WRITE_DELAY[$clog2(MAX_WRITE_DELAY):0]) begin
                            delay_before_write_counter_d[ci] = READ_TO_WRITE_DELAY[$clog2(MAX_WRITE_DELAY):0];
                        end
                    end
                    // Per-BG tCCD: same BG = LONG, diff BG = only-raise SHORT
                    for (ci = 0; ci < NUM_BG; ci = ci + 1) begin
                        if (ci[BG_BITS-1:0] == stage2_bg) begin
                            ccd_counter_d[ci] = CAS_TO_CAS_DELAY_SAME_BG[$clog2(MAX_CCD_DELAY):0];
                        end
                        else if (ccd_counter_d[ci] < CAS_TO_CAS_DELAY_DIFF_BG[$clog2(MAX_CCD_DELAY):0]) begin
                            ccd_counter_d[ci] = CAS_TO_CAS_DELAY_DIFF_BG[$clog2(MAX_CCD_DELAY):0];
                        end
                    end
                end
            end
        end

        // ---- Bank anticipation (speculative ACTIVATE) ----
        // Performance optimization: while Stage 2 is completing a WR/RD
        // (or is idle), peek at Stage 1's NEXT request and pre-open its
        // target bank.  By the time that request reaches Stage 2, tRCD
        // has already elapsed and the WR/RD can fire immediately —
        // hiding the ACT-to-CAS latency behind the current transaction.
        //
        // Guards (all must be true simultaneously):
        //   (a) stage2_update      : Stage 2 just finished a WR/RD (or
        //       is idle), so we have a free command slot this cycle. Q: but in uberddr3 it seems anticipation can also happen when stage2 is still busy with a prior request, 
        //   (b) stage1_pending     : Stage 1 has a queued request.
        //   (c) refresh_idle       : no refresh in progress.
        //   (d) !bank_status_d[next_bank] : target bank is closed.
        //       Uses _d (not _q) so it sees any PRE that fired THIS
        //       cycle in Stage 2 — avoids a 1-cycle stale-state hazard. Q: what does it meant by stale-state  hazard?
        //   (e) delay_before_activate_counter_d == 0 : tRP fully elapsed.
        //       Checks == 0 (not <= 1) because Stage 2 may have ALSO
        //       loaded this counter this cycle (e.g. PRE just fired);
        //       using == 0 ensures no conflict with Stage 2's loads.
        //   (f) rrd_counter_d == 0 : tRRD fully elapsed (same reason).
        //   (g) activate_timestamp_d == 0 : tFAW slot is free.  If
        //       Stage 2 also fired an ACT this cycle, it already wrote
        //       the tFAW timestamp, so this check correctly blocks a
        //       second ACT in the same cycle.
        //
        // Counter loading below mirrors Case 2 (ACTIVATE) exactly:
        // same tRAS, tRCD, tRRD, tFAW loads — because this IS an ACT
        // command, just issued speculatively from a different pipeline
        // stage.
        // Q: WHy does in uberddr3 the antiicipate is both precharge and activate, but here only ativate? isnt this a downgrade? WHY DO IT THIS WAY??
        if (stage2_update && stage1_pending && refresh_idle
            && !bank_status_d[stage1_next_bank]
            && (delay_before_activate_counter_d[stage1_next_bank] == 0)
            && (rrd_counter_d[stage1_next_bg] == 0)
            && (activate_timestamp_d[activate_index_q] == 0)) begin
            sched_anticipate = 1'b1;
            // tRAS: block PRE until minimum active time elapses
            delay_before_precharge_counter_d[stage1_next_bank] = ACTIVATE_TO_PRECHARGE_DELAY[$clog2(MAX_PRECHARGE_DELAY):0];
            // tRCD: block WR/RD until row is fully open (only-raise)
            if (delay_before_write_counter_d[stage1_next_bank] < ACTIVATE_TO_WRITE_DELAY[$clog2(MAX_WRITE_DELAY):0]) begin
                delay_before_write_counter_d[stage1_next_bank] = ACTIVATE_TO_WRITE_DELAY[$clog2(MAX_WRITE_DELAY):0];
            end
            if (delay_before_read_counter_d[stage1_next_bank] < ACTIVATE_TO_READ_DELAY[$clog2(MAX_READ_DELAY):0]) begin
                delay_before_read_counter_d[stage1_next_bank] = ACTIVATE_TO_READ_DELAY[$clog2(MAX_READ_DELAY):0];
            end
            bank_status_d[stage1_next_bank] = 1'b1;
            bank_active_row_d[stage1_next_bank] = stage1_next_row;
            // tRRD (Row-to-Row Delay): after this anticipatory ACT, block
            // the next ACT to respect the JEDEC ACT-to-ACT minimum.
            // Loop over every bank group:
            //   - Same BG as the one we just ACTIVATEd: unconditionally
            //     load tRRD_L (Long), which is the stricter same-BG limit.
            //   - Different BG: only-raise to tRRD_S (Short).  The `<`
            //     guard ensures we never overwrite a longer pending delay
            //     (e.g. a tRRD_L that was loaded by Stage 2's ACT earlier
            //     this cycle).
            for (ci = 0; ci < NUM_BG; ci = ci + 1) begin
                if (ci[BG_BITS-1:0] == stage1_next_bg) begin
                    rrd_counter_d[ci] = ACTIVATE_TO_ACTIVATE_DELAY_SAME_BG[$clog2(MAX_RRD_DELAY):0];
                end
                else if (rrd_counter_d[ci] < ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_RRD_DELAY):0]) begin
                    rrd_counter_d[ci] = ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_RRD_DELAY):0];
                end
            end
            // Per-bank activate delay: only-raise tRRD_S into every
            // OTHER bank's delay_before_activate_counter.  This is
            // redundant with the per-BG rrd_counter above but provides
            // per-bank granularity — the same counter also holds tRP
            // (from PRE), so a single per-bank check in the ACT guard
            // covers both tRP and tRRD_S.  See Case 2 comment for the
            // full rationale on why both per-BG and per-bank counters
            // coexist.
            for (ci = 0; ci < NUM_BANKS; ci = ci + 1) begin
                if (ci[BG_BITS+BA_BITS-1:0] != stage1_next_bank
                    && delay_before_activate_counter_d[ci] < ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_ACTIVATE_DELAY):0]) begin
                    delay_before_activate_counter_d[ci] = ACTIVATE_TO_ACTIVATE_DELAY_DIFF_BG[$clog2(MAX_ACTIVATE_DELAY):0];
                end
            end
            // tFAW (Four-Activate Window): write the tFAW countdown into
            // the next slot of the 4-entry circular timestamp buffer.
            // When all 4 slots are non-zero, tfaw_blocked goes high and
            // no further ACTs can fire until the oldest entry expires.
            activate_timestamp_d[activate_index_q] = TFAW_CYCLES[$clog2(TFAW_CYCLES):0];
        end
    end

    // --- Stall: combinational, reflects current registered state ---
    always @* begin
        o_wb_stall = stage1_pending || !reset_done || refresh_active
                     || !o_calib_complete;
    end

    // -- Main sequential block --
    // Everything here updates on posedge i_controller_clk.
    // Handles reset, ROM controller, scheduler command construction,
    // data enable pipelines, calibration FSM, counter latching,
    // pipeline handoff, and DFI signal mapping.
    integer bank_i;
    always @(posedge i_controller_clk) begin
        if (!i_rst_n) begin
            o_dfi_cs_n     <= {SERDES_RATIO{1'b1}};
            o_dfi_act_n    <= {SERDES_RATIO{1'b1}};
            o_dfi_ras_n    <= {SERDES_RATIO{1'b1}};
            o_dfi_cas_n    <= {SERDES_RATIO{1'b1}};
            o_dfi_we_n     <= {SERDES_RATIO{1'b1}};
            o_dfi_cke      <= {SERDES_RATIO{1'b0}};
            o_dfi_odt      <= {SERDES_RATIO{1'b0}};
            o_dfi_reset_n  <= {SERDES_RATIO{1'b0}};
            o_dfi_address  <= {(SERDES_RATIO*17){1'b0}};
            o_dfi_bank     <= {(SERDES_RATIO*BA_BITS){1'b0}};
            o_dfi_bg       <= {(SERDES_RATIO*BG_BITS){1'b0}};
            o_dfi_wrdata   <= {(SERDES_RATIO*DFI_DATA_WIDTH){1'b0}};
            o_dfi_wrdata_en   <= {SERDES_RATIO{1'b0}};
            o_dfi_wrdata_mask <= {(SERDES_RATIO*(2*BYTE_LANES)){1'b0}};
            o_dfi_rddata_en   <= {SERDES_RATIO{1'b0}};
            o_dfi_rdlvl_en      <= 1'b0;
            o_dfi_rdlvl_gate_en <= 1'b0;
            o_dfi_wrlvl_en      <= 1'b0;
            o_dfi_wrlvl_strobe  <= 1'b0;
            o_dfi_lvl_pattern   <= {SERDES_RATIO{1'b0}};
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
            // =============================================================
            // Command Slot Defaults (NOP/DES)
            // All 4 slots default to DES (cs_n=1, NOP encoding).
            // The ROM controller or scheduler overrides specific slots.
            // =============================================================
            for (bank_i = 0; bank_i < SERDES_RATIO; bank_i = bank_i + 1) begin
                cmd_d[bank_i] <= {
                    1'b1,       //cs_n = 1 (deselected)
                    CMD_NOP,    //{act_n=1, ras_n=1, cas_n=1, we_n=1}
                    cmd_odt,    //odt (broadcast to all slots)
                    reset_done ? 1'b1 : init_cke,     //cke (muxed: rom_instruction on fire, hold on countdown)
                    reset_done ? 1'b1 : init_reset_n, //reset_n (muxed: rom_instruction on fire, hold on countdown)
                    2'b00,      //bg
                    2'b00,      //ba
                    17'b0       //addr
                };
            end

            // =============================================================
            // ROM Controller (implements JESD79-4D init sequence)
            // Walks through the 36-entry ROM: power-on reset, MRS writes,
            // ZQCL, DLL lock, calibration windows, then loops the refresh
            // sequence (addrs 33-35) forever after init completes.
            // =============================================================
            if (!reset_done || instruction_address >= ROM_ADDR_REF_START) begin
                // Delay counter management
                if (!delay_counter_is_zero) begin
                    delay_counter <= delay_counter - 1'b1;
                    delay_counter_is_zero <= (delay_counter == {{(DELAY_COUNTER_WIDTH-1){1'b0}}, 1'b1});
                end else if (!pause_counter && !rom_prea_hold) begin
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
            // =============================================================
            // Scheduler Command Construction
            // Driven by sched_* flags from the combinational scheduler.
            // Each flag builds the appropriate DFI command word and
            // places it in the correct slot. Only fires during the
            // tREFI idle window (between refresh commands).
            // =============================================================
            if (sched_precharge) begin
                cmd_d[PRECHARGE_SLOT] <= {
                    1'b0,           //cs_n = 0
                    CMD_PRE,        //{act_n=1, ras_n=0, cas_n=1, we_n=0}
                    cmd_odt, 1'b1, 1'b1,  //odt, cke=1, reset_n=1
                    stage2_bg_padded,
                    stage2_ba,      //ba
                    7'b0, 1'b0, 9'b0  //A10=0 (single bank precharge)
                };
            end
            if (sched_activate) begin
                cmd_d[ACTIVATE_SLOT] <= {
                    1'b0,           //cs_n = 0
                    1'b0,           //act_n = 0 (ACTIVATE)
                    stage2_row_padded[16],  //ras_n -> A16
                    stage2_row_padded[15],  //cas_n -> A15
                    stage2_row_padded[14],  //we_n  -> A14
                    cmd_odt, 1'b1, 1'b1,
                    stage2_bg_padded,
                    stage2_ba,
                    stage2_row_padded  //addr[16:0] = full row address
                };
            end
            if (sched_write) begin
                cmd_d[WRITE_SLOT] <= {
                    1'b0,           //cs_n = 0
                    CMD_WR,         //{act_n=1, ras_n=1, cas_n=0, we_n=0}
                    cmd_odt, 1'b1, 1'b1,
                    stage2_bg_padded, 
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
                    stage2_bg_padded, 
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
                    stage1_next_bg_padded, 
                    stage1_next_bank[BA_BITS-1:0],
                    stage1_next_row_padded
                };
            end

            // ===========================================================
            // Read/Write Data Enable Pipelines + WB ACK
            // Shift registers track when dfi_rddata_en / dfi_wrdata_en
            // should assert after a READ / WRITE command. WB ACK is
            // generated from write command issue and dfi_rddata_valid.
            // ===========================================================

            // Write data enable shift register
            wrdata_en_pipe_q <= {1'b0, wrdata_en_pipe_q[WRITE_DATA_DELAY:1]};
            if (sched_write)
                wrdata_en_pipe_q[WRITE_DATA_DELAY] <= 1'b1;
            o_dfi_wrdata_en <= {SERDES_RATIO{wrdata_en_pipe_q[0]}};

            // Write data delay pipeline -- mirrors wrdata_en_pipe_q
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

            // Read data enable shift register
            rddata_en_pipe_q <= {1'b0, rddata_en_pipe_q[RDDATA_EN_PIPE_WIDTH-1:1]};
            if (sched_read)
                rddata_en_pipe_q[RDDATA_EN_PIPE_WIDTH-1] <= 1'b1;
            o_dfi_rddata_en <= {SERDES_RATIO{rddata_en_pipe_q[0]}};

            // Read data capture from DFI
            if (|i_dfi_rddata_valid)
                o_wb_data <= i_dfi_rddata;

            // WB ACK generation
            // Write ACK: 1 cycle after WR command issues
            // Read ACK: 1 cycle after dfi_rddata_valid (data latch delay)
            write_ack_q <= sched_write;
            read_ack_q  <= |i_dfi_rddata_valid;

            // ===========================================================
            // Training Command Pump (DFI 3.1 Full Training Mode)
            // MC-side calibration FSM: drives DFI training enables and
            // pumps READ commands / wrlvl strobes during calibration
            // windows opened by the init ROM (addrs 22, 27). The pump
            // takes over cmd_d directly while pause_counter is held.
            // See the training FSM states above.
            // ===========================================================
            begin
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
                                ACTIVATE_TO_READ_DELAY[$clog2(T_WRLVL_WW):0];
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
                            // already released -- wait for init to finish
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

                    default: begin
                        calib_state <= CALIB_ERROR;
                    end
                endcase
            end

            // ===========================================================
            // Counter Latch + Pipeline Handoff + Stage 1 WB Accept
            // ===========================================================

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

            // Stage 1->2 handoff: zero-bubble pipeline
            // stage2_update=1 when Stage 2 idle OR just issued WR/RD (completing).
            // Consume Stage 1's request immediately -- no wasted cycles.
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

            // ===========================================================
            // DFI Signal Mapping (DFI 3.1, 4-phase, 1:4 ratio)
            // Decompose packed cmd_d[] slots into flat DFI output vectors
            // ===========================================================
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

    // ==============================================================
    // Reset/Refresh ROM (JESD79-4D Figure 7)
    // 36 addresses (0-35): init sequence + calibration windows + refresh loop
    // ==============================================================

    function [31:0] rom_timer(input [4:0] ctl, input [3:0] cmd, input integer timer);
        rom_timer = {ctl, cmd, 3'b000, timer[19:0]};
    endfunction

    function [31:0] rom_mrs(input [2:0] mrs_sel, input [13:0] mrs_addr);
        rom_mrs = {2'b00, mrs_addr[10], 2'b11, CMD_MRS, mrs_sel, 6'b0, mrs_addr};
    endfunction

    function [31:0] read_rom_instruction(input [5:0] addr);
        case (addr)
            // -- Power-on reset (JESD79-4D Figure 7) --
            6'd0:  read_rom_instruction = rom_timer(CTL_CKE0_RST0, CMD_NOP, ps_to_cycles(POWER_ON_RESET_HIGH_ps)); // RESET_n=0, CKE=0, wait >=200us
            6'd1:  read_rom_instruction = rom_timer(CTL_CKE0_RST1, CMD_NOP, ps_to_cycles(INITIAL_CKE_LOW_ps));     // RESET_n=1, CKE=0, wait >=500us
            6'd2:  read_rom_instruction = rom_timer(CTL_TIMER,      CMD_DES, ps_to_cycles(tXPR_ps));                // CKE=1, deselect, wait tXPR

            // -- Mode register writes (MR3->MR6->MR5->MR4->MR2->MR1->MR0) --
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

            // -- ZQCL + DLL lock --
            6'd17: read_rom_instruction = rom_timer(CTL_TIMER_A10,  CMD_ZQCL, nCK_to_cycles(tZQinit_nCK)); // ZQCL (A10=1), wait tZQinit
            6'd18: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, nCK_to_cycles(tDLLK_nCK));    // wait tDLLK (DLL lock)
            6'd19: read_rom_instruction = rom_timer(CTL_TIMER_A10,  CMD_PRE, ps_to_cycles(tRP_ps));        // PRE ALL (A10=1), wait tRP

            // -- Read calibration window (MPR mode, JESD79-4D) --
            6'd20: read_rom_instruction = rom_mrs  (MRS_MR3, MR3_MPR_EN);                              // MR3: MPR enable (A2=1)
            6'd21: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, ps_to_cycles(tMOD_ps));    // wait tMOD
            6'd22: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, 0);                        // read leveling trigger (pause_counter gates ROM) (ROM_ADDR_RD_CAL)
            6'd23: read_rom_instruction = rom_mrs  (MRS_MR3, MR3_MPR_DIS);                             // MR3: MPR disable
            6'd24: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, ps_to_cycles(tMOD_ps));    // wait tMOD

            // -- Write leveling window (JESD79-4D) --
            6'd25: read_rom_instruction = rom_mrs  (MRS_MR1, MR1_WL_EN);                               // MR1: write leveling on (A7=1)
            6'd26: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, nCK_to_cycles(tWLMRD_nCK)); // wait tWLMRD
            6'd27: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, 0);                        // write leveling trigger (pause_counter gates ROM) (ROM_ADDR_WL_CAL)
            6'd28: read_rom_instruction = rom_mrs  (MRS_MR1, MR1_WL_DIS);                              // MR1: write leveling off
            6'd29: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, ps_to_cycles(tMOD_ps));    // wait tMOD

            // -- Final refresh + done --
            6'd30: read_rom_instruction = rom_timer(CTL_TIMER_A10,  CMD_PRE, ps_to_cycles(tRP_ps));  // PRE ALL, wait tRP
            6'd31: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_REF, ps_to_cycles(tRFC_ps)); // REF, wait tRFC
            6'd32: read_rom_instruction = rom_timer(CTL_DONE,       CMD_NOP, 0);                     // reset_done=1, init complete (ROM_ADDR_NORMAL)

            // -- Refresh loop (repeats 33->34->35->33) --
            6'd33: read_rom_instruction = rom_timer(CTL_TIMER_A10,  CMD_PRE, ps_to_cycles(tRP_ps));   // PRE ALL, wait tRP (ROM_ADDR_REF_START)
            6'd34: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_REF, ps_to_cycles(tRFC_ps));  // REF, wait tRFC
            6'd35: read_rom_instruction = rom_timer(CTL_TIMER,      CMD_NOP, REFRESH_TREFI_TIMER);    // NOP, wait adjusted tREFI (ROM_ADDR_REF_END)
            default: read_rom_instruction = rom_timer(CTL_TIMER, CMD_NOP, 0);
        endcase
    endfunction

    // ==============================================================
    // Debug $display
    // ==============================================================
`ifndef YOSYS
    initial begin
        $display("==============================================================");
        $display("UberDDR4 Controller Configuration");
        $display("==============================================================");

        $display("-- Device");
        $display("  DEVICE_WIDTH          = x%0d", DEVICE_WIDTH);
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
        $display("  DM_ENABLED            = %0d", DM_ENABLED);
        $display("  NUM_BANKS             = %0d", NUM_BANKS);
        $display("  NUM_BG                = %0d", NUM_BG);

        $display("-- Latency");
        $display("  CL                    = %0d nCK", CL_nCK);
        $display("  CWL                   = %0d nCK", CWL_nCK);
        $display("  WR                    = %0d nCK", WR_nCK);

        $display("-- Core Timing");
        $display("  tRAS                  = %0d ps (%0d nCK)", tRAS_ps, ps_to_nCK(tRAS_ps));
        $display("  tRCD                  = %0d ps (%0d nCK)", tRCD_ps, ps_to_nCK(tRCD_ps));
        $display("  tRP                   = %0d ps (%0d nCK)", tRP_ps, ps_to_nCK(tRP_ps));
        $display("  tRC                   = %0d ps (%0d nCK)", tRC_ps, ps_to_nCK(tRC_ps));
        $display("  tWR                   = %0d ps (%0d nCK)", tWR_ps, ps_to_nCK(tWR_ps));
        $display("  tRTP                  = %0d ps (%0d nCK)", tRTP_ps, ps_to_nCK(tRTP_ps));

        $display("-- Bank-Group Timing");
        $display("  tCCD_L                = %0d ps (%0d nCK)", tCCD_L_ps, ps_to_nCK(tCCD_L_ps));
        $display("  tCCD_S                = %0d nCK", tCCD_S_nCK);
        $display("  tWTR_L                = %0d ps (%0d nCK)", tWTR_L_ps, ps_to_nCK(tWTR_L_ps));
        $display("  tWTR_S                = %0d ps (%0d nCK)", tWTR_S_ps, ps_to_nCK(tWTR_S_ps));
        $display("  tRRD_L                = %0d ps (%0d nCK)", tRRD_L_ps, ps_to_nCK(tRRD_L_ps));
        $display("  tRRD_S                = %0d ps (%0d nCK)", tRRD_S_ps, ps_to_nCK(tRRD_S_ps));
        $display("  tFAW                  = %0d ps (%0d nCK, %0d ctrl)", tFAW_ps, ps_to_nCK(tFAW_ps), TFAW_CYCLES);

        $display("-- Init/MRS Timing");
        $display("  tMRD                  = %0d nCK (%0d ctrl)", tMRD_nCK, nCK_to_cycles(tMRD_nCK));
        $display("  tMOD                  = %0d ps (%0d ctrl)", tMOD_ps, ps_to_cycles(tMOD_ps));
        $display("  tZQinit               = %0d nCK (%0d ctrl)", tZQinit_nCK, nCK_to_cycles(tZQinit_nCK));
        $display("  tDLLK                 = %0d nCK (%0d ctrl)", tDLLK_nCK, nCK_to_cycles(tDLLK_nCK));
        $display("  tXPR                  = %0d ps (%0d ctrl)", tXPR_ps, ps_to_cycles(tXPR_ps));
        $display("  tWLMRD                = %0d nCK (%0d ctrl)", tWLMRD_nCK, nCK_to_cycles(tWLMRD_nCK));
        $display("  POWER_ON_RESET        = %0d ps (%0d ctrl)", POWER_ON_RESET_HIGH_ps, ps_to_cycles(POWER_ON_RESET_HIGH_ps));
        $display("  INITIAL_CKE_LOW       = %0d ps (%0d ctrl)", INITIAL_CKE_LOW_ps, ps_to_cycles(INITIAL_CKE_LOW_ps));

        $display("-- Refresh");
        $display("  tRFC                  = %0d ps (%0d ctrl)", tRFC_ps, ps_to_cycles(tRFC_ps));
        $display("  tREFI                 = %0d ps (%0d ctrl)", tREFI_ps, ps_to_cycles(tREFI_ps));
        $display("  Refresh loop period   = %0d ctrl (%0d ps)",
                 ps_to_cycles(tRP_ps) + ps_to_cycles(tRFC_ps) + REFRESH_TREFI_TIMER + 3,
                 (ps_to_cycles(tRP_ps) + ps_to_cycles(tRFC_ps) + REFRESH_TREFI_TIMER + 3)
                 * CONTROLLER_CLK_PERIOD);

        $display("-- Computed Delays (controller cycles)");
        $display("  ACT->WR               = %0d", ACTIVATE_TO_WRITE_DELAY);
        $display("  ACT->RD               = %0d", ACTIVATE_TO_READ_DELAY);
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

        $display("-- Slot Assignment");
        $display("  READ_SLOT             = %0d", READ_SLOT);
        $display("  WRITE_SLOT            = %0d", WRITE_SLOT);
        $display("  ACTIVATE_SLOT         = %0d", ACTIVATE_SLOT);
        $display("  PRECHARGE_SLOT        = %0d", PRECHARGE_SLOT);

        $display("-- Address Mapping");
        $display("  ADDR_MAPPING          = %0d", ADDR_MAPPING);
        $display("  WB_ADDR_BITS          = %0d", WB_ADDR_BITS);
        $display("  WB_DATA_BITS          = %0d", WB_DATA_BITS);
        $display("  COL_LOW               = %0d", COL_LOW);

        $display("-- Mode Registers");
        $display("  MR0                   = 14'h%04h", MR0);
        $display("  MR1 (WL off)          = 14'h%04h", MR1_WL_DIS);
        $display("  MR2                   = 14'h%04h", MR2);
        $display("  MR3 (MPR off)         = 14'h%04h", MR3_MPR_DIS);
        $display("  MR4                   = 14'h%04h", MR4);
        $display("  MR5                   = 14'h%04h", MR5);
        $display("  MR6                   = 14'h%04h", MR6);

        $display("-- Sim Flags");
        $display("  MICRON_SIM            = %0d", MICRON_SIM);
        $display("==============================================================");
    end
`endif

    // ==============================================================
    // Helper Functions
    // NOTE: Verilog elaboration resolves functions before localparams,
    // so these can be placed at the bottom even though earlier sections call them
    // ==============================================================

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
    // UberDDR3 uses registered eligibility (counter_d==0 -> register -> fire next cycle),
    // giving gap = 4*(k+1) + end_slot - start_slot for k >= 0.
    // UberDDR4 fires directly when counter_q <= 1 (no registered pipeline), giving:
    //   k=0 : fire at M+1, gap = 4 + end_slot - start_slot
    //   k>=2: fire at M+k, gap = 4*k + end_slot - start_slot
    //   (k=1 fires at M+1, same as k=0 due to <= 1 check)
    // So for k >= 1 returned by the DDR3-style formula, we add 1 to compensate.
    function integer find_delay(input integer delay_nCK, input [1:0] start_slot, input [1:0] end_slot);
        integer k;
        begin
            k = 0;
            while (((4 - {30'b0, start_slot}) + {30'b0, end_slot} + 4*k) < delay_nCK)
                k = k + 1;
            if (k > 0) k = k + 1;
            find_delay = k;
        end
    endfunction

    // get_slot: assign each command type to one of 4 DFI slots per controller cycle
    // (DFI 3.1). Read/Write slots derived from CL/CWL mod 4; Activate and
    // Precharge fill the remaining slots avoiding collisions.
    function [1:0] get_slot(input [3:0] cmd);
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
                CMD_RD:  get_slot = read_slot[1:0];
                CMD_WR:  get_slot = write_slot[1:0];
                CMD_ACT: get_slot = anticipate_activate_slot[1:0];
                CMD_PRE: get_slot = anticipate_precharge_slot[1:0];
                default: get_slot = 2'b0;
            endcase
        end
    endfunction

    // CL_generator: return worst-case CAS Latency for the given DDR4 clock period.
    // Picks the highest CL supported by ALL speed bins at each speed grade,
    // so the auto value works with any DDR4 part (JESD79-4D Tables 147-153).
    // Override via CL parameter (nonzero = use directly) for faster bins.
    function integer CL_generator(input integer ddr4_clk_period);
        begin
            if (CL != 0)                       CL_generator = {26'b0, CL}; //manual override
            else if (ddr4_clk_period >= 1_500) CL_generator = 9;      //DDR4-1333
            else if (ddr4_clk_period >= 1_250) CL_generator = 12;     //DDR4-1600 (Table 147: J=10, K=11, L=12)
            else if (ddr4_clk_period >= 1_071) CL_generator = 14;     //DDR4-1866 (Table 148: L=12, M=13, N=14)
            else if (ddr4_clk_period >= 937)   CL_generator = 16;     //DDR4-2133 (Table 149: N=14, P=15, R=16)
            else if (ddr4_clk_period >= 833)   CL_generator = 18;     //DDR4-2400 (Table 150: P=15, R=16, T=17, U=18)
            else if (ddr4_clk_period >= 750)   CL_generator = 20;     //DDR4-2666 (Table 151: T=17, U=18, V=19, W=20)
            else if (ddr4_clk_period >= 682)   CL_generator = 22;     //DDR4-2933
            else if (ddr4_clk_period >= 625)   CL_generator = 24;     //DDR4-3200 (Table 153: W=20, AA=22, AC=24)
            else                               CL_generator = 24;
        end
    endfunction

    // CWL_generator: CAS Write Latency for 1tCK write preamble (JESD79-4D Table 21)
    function integer CWL_generator(input integer ddr4_clk_period);
        begin
            if (CWL != 0)                      CWL_generator = {27'b0, CWL}; //manual override
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

    // CL_encoding: JESD79-4D Table 15 -- MR0 CAS Latency {A12, A6:A4, A2}
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
            6'd30: CL_encoding = 5'b1_010_1; // enc 21
            6'd32: CL_encoding = 5'b1_011_1; // enc 23
            default: CL_encoding = 5'b0_000_0; // CL=9 fallback
        endcase
    endfunction

    // CWL_encoding: JESD79-4D Table 21 -- MR2 A5:A3
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

    // WR_RTP_encoding: JESD79-4D Table 13 -- MR0 {A13, A11:A9}
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

    // ===================================
    // Debug Status Assigns
    // ===================================
    assign o_calib_state    = calib_state;
    assign o_stage1_pending = stage1_pending;
    assign o_stage2_pending = stage2_pending;
    assign o_stage2_we      = stage2_we;
    assign o_refresh_idle   = refresh_idle;
    assign o_bank_status    = bank_status_q;

    // =======================
    // Formal Properties
    // =======================
`ifdef FORMAL
    `include "ddr4_controller_formal.vh"
`endif

endmodule
