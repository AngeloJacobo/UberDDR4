////////////////////////////////////////////////////////////////////////////////
// AXKU3 board-level DDR4 LED bring-up simulation
//
// Instantiates the real axku3_uberddr4 top level, including its Clock Wizard
// instances.  Two x16 Micron DDR4 models represent the AXKU3 board's two
// MT40A512M16LY-062E devices.  The test completes when the design asserts an
// initialization terminal status and therefore changes the LED output.
//
// Compile the Clock Wizard simulation products together with this file:
//   clk_wiz_0/sim/clk_wiz_0.v and clk_wiz_1/sim/clk_wiz_1.v
//
// Configure the Micron model for the 8 Gb x16 part and tCK = 1.250 ns
// (DDR4-1600).
////////////////////////////////////////////////////////////////////////////////

`timescale 1ps / 1ps
`default_nettype none

`ifdef XILINX_SIMULATOR
// XSim requires this transparent bidirectional primitive wrapper for the
// Micron model interface connections (matching the Vivado MIG example TBs).
module short(in1, in1);
    inout wire in1;
endmodule
`endif

module axku3_uberddr4_sim_top;

    import arch_package::*;

    localparam TIMEOUT_PS = 2_000_000_000; // 2 ms: includes real 200 us + 500 us DRAM waits
    localparam TB_DENSITY = _8G;

    // ------------------------------------------------------------------------
    // AXKU3 board clock and pushbutton reset
    // ------------------------------------------------------------------------
    reg sys_clk_p = 1'b0;
    wire sys_clk_n = ~sys_clk_p;
    always #2500 sys_clk_p = ~sys_clk_p; // 200 MHz differential oscillator

    reg rst_n;
    initial begin
        rst_n = 1'b0;
        #200_000;                        // hold pushbutton reset for 200 ns
        rst_n = 1'b1;
    end

    // ------------------------------------------------------------------------
    // Board DDR4 nets
    // ------------------------------------------------------------------------
    wire        ddr4_ck_p, ddr4_ck_n;
    wire        ddr4_reset_n, ddr4_cke, ddr4_cs_n, ddr4_act_n;
    wire [16:0] ddr4_addr;
    wire [1:0]  ddr4_ba;
    wire        ddr4_bg;
    wire        ddr4_odt;
    wire [3:0]  ddr4_dm_n;
    wire [31:0] ddr4_dq;
    wire [3:0]  ddr4_dqs_p, ddr4_dqs_n;
    wire [3:0]  led;

    axku3_uberddr4 u_dut (
        .sys_clk_p     (sys_clk_p),
        .sys_clk_n     (sys_clk_n),
        .rst_n         (rst_n),
        .led           (led),
        .ddr4_ck_p     (ddr4_ck_p),
        .ddr4_ck_n     (ddr4_ck_n),
        .ddr4_reset_n  (ddr4_reset_n),
        .ddr4_cke      (ddr4_cke),
        .ddr4_cs_n     (ddr4_cs_n),
        .ddr4_act_n    (ddr4_act_n),
        .ddr4_addr     (ddr4_addr),
        .ddr4_ba       (ddr4_ba),
        .ddr4_bg       (ddr4_bg),
        .ddr4_odt      (ddr4_odt),
        .ddr4_dm_n     (ddr4_dm_n),
        .ddr4_dq       (ddr4_dq),
        .ddr4_dqs_p    (ddr4_dqs_p),
        .ddr4_dqs_n    (ddr4_dqs_n)
    );

    // ddr4_top is instantiated inside the board wrapper, so override its
    // elaboration-time Micron-model timing mode from this simulation only.
    // A procedural force cannot modify a parameter after elaboration.
    defparam u_dut.u_ddr4_top.MICRON_SIM = 1'b1;

`ifdef SIM_NATIVE_DIAG_ARCHIVE_FAST_WL
    // Reproduce the historical native-PHY application-path run without
    // spending hours in the UNISIM model's write-level sweep.  This diagnostic
    // is never enabled by the normal AXKU3 or regression flows.
    integer archive_fast_wl_lane;
    initial begin : archive_fast_wl
        for (archive_fast_wl_lane = 0;
             archive_fast_wl_lane < 4;
             archive_fast_wl_lane = archive_fast_wl_lane + 1) begin
            wait (u_dut.u_ddr4_top.gen_native_phy.u_phy.u_native.phy_state ==
                      4'd8 &&
                  u_dut.u_ddr4_top.gen_native_phy.u_phy.u_native.train_lane ==
                      archive_fast_wl_lane);
            force u_dut.u_ddr4_top.gen_native_phy.u_phy.u_native.phy_state =
                4'd12;
            repeat (6) @(posedge u_dut.controller_clk);
            release u_dut.u_ddr4_top.gen_native_phy.u_phy.u_native.phy_state;
        end
    end
`endif

`ifdef SIM_NATIVE_DIAG_SHORT_FIRST_READ
    // Directed native-PHY boundary test.  Write address zero once, then move
    // the simulation-only BIST address counter to its terminal value so the
    // first readback arrives without streaming the other 1023 locations.
    // Stop after that read is checked.  Normal AXKU3 simulation never defines
    // this acceleration hook and therefore runs the complete BIST unchanged.
    initial begin : native_short_first_read
        wait (u_dut.u_ddr4_top.u_prober.bist_state == 3'd1);
        // The request already presented on Wishbone is address zero. Make it
        // the final burst write, then do the same for the first burst read.
        // Forcing only write_addr would still stream 1024 reads and generate
        // misleading Micron "unwritten address" warnings for addresses 1+.
        force u_dut.u_ddr4_top.u_prober.gen_bist.write_addr = 10'h3ff;
        @(posedge u_dut.controller_clk);
        #1 release u_dut.u_ddr4_top.u_prober.gen_bist.write_addr;
        wait (u_dut.u_ddr4_top.u_prober.bist_state == 3'd2);
        force u_dut.u_ddr4_top.u_prober.gen_bist.read_addr = 10'h3ff;
        @(posedge u_dut.controller_clk);
        #1 release u_dut.u_ddr4_top.u_prober.gen_bist.read_addr;
        wait ((u_dut.u_ddr4_top.u_prober.correct_count != 0) ||
              (u_dut.u_ddr4_top.u_prober.error_count != 0));
        $display("[%0t] NATIVE_SHORT_FIRST_READ: correct=%0d error=%0d",
                 $realtime,
                 u_dut.u_ddr4_top.u_prober.correct_count,
                 u_dut.u_ddr4_top.u_prober.error_count);
        $finish;
    end
    initial begin : native_short_first_read_timeout
        wait (u_dut.u_ddr4_top.u_prober.bist_state == 3'd2);
        repeat (200) @(posedge u_dut.controller_clk);
        $display("[%0t] NATIVE_SHORT_TIMEOUT: cal_session=%0b outstanding=%0d quiet=%0d cal_req=%0b phy_state=%0d empty=%h rden=%h",
                 $realtime,
                 u_dut.u_ddr4_top.gen_native_phy.u_phy.u_native.calibration_session,
                 u_dut.u_ddr4_top.gen_native_phy.u_phy.u_native.calibration_read_outstanding,
                 u_dut.u_ddr4_top.gen_native_phy.u_phy.u_native.calibration_quiet_count,
                 u_dut.u_ddr4_top.gen_native_phy.u_phy.u_native.calibration_request,
                 u_dut.u_ddr4_top.gen_native_phy.u_phy.u_native.phy_state,
                 u_dut.u_ddr4_top.gen_native_phy.u_phy.u_native.fifo_empty_flat,
                 u_dut.u_ddr4_top.gen_native_phy.u_phy.u_native.phy_rden_ready);
        $finish;
    end
`endif

    // The LED assignment in the DUT is observed as it exists in hardware.
    // These hierarchical references provide the unencoded terminal reason.
    wire init_done   = u_dut.init_done;
    wire init_failed = u_dut.init_failed;

    // ------------------------------------------------------------------------
    // Two MT40A512M16LY-062E x16 DDR4 devices
    // Device 0 owns byte lanes 0/1; device 1 owns byte lanes 2/3.
    // ------------------------------------------------------------------------
    wire model_enable = 1'b1;
    DDR4_if #(.CONFIGURED_DQ_BITS(16)) iDDR4[1:0]();

    genvar device, bit_index;
    generate
        for (device = 0; device < 2; device = device + 1) begin : gen_ddr4
            assign iDDR4[device].CK        = {ddr4_ck_p, ddr4_ck_n};
            assign iDDR4[device].RESET_n   = ddr4_reset_n;
            assign iDDR4[device].CKE       = ddr4_cke;
            assign iDDR4[device].CS_n      = ddr4_cs_n;
            assign iDDR4[device].ACT_n     = ddr4_act_n;
            assign iDDR4[device].RAS_n_A16 = ddr4_addr[16];
            assign iDDR4[device].CAS_n_A15 = ddr4_addr[15];
            assign iDDR4[device].WE_n_A14  = ddr4_addr[14];
            assign iDDR4[device].ADDR      = ddr4_addr[13:0];
            assign iDDR4[device].BA        = ddr4_ba;
            assign iDDR4[device].BG        = {1'b0, ddr4_bg};
            assign iDDR4[device].ODT       = ddr4_odt;
            assign iDDR4[device].ADDR_17   = 1'b0;
            assign iDDR4[device].C         = '0;
            assign iDDR4[device].TEN       = 1'b0;
            assign iDDR4[device].PARITY    = 1'b0;
            assign iDDR4[device].ZQ        = 1'b1;
            assign iDDR4[device].PWR       = 1'b1;
            assign iDDR4[device].VREF_CA   = 1'b1;
            assign iDDR4[device].VREF_DQ   = 1'b1;

            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin : gen_dq_lo
                `ifdef XILINX_SIMULATOR
                short bidi_dq(iDDR4[device].DQ[bit_index], ddr4_dq[(device*2)*8 + bit_index]);
                `else
                tran  bidi_dq(iDDR4[device].DQ[bit_index], ddr4_dq[(device*2)*8 + bit_index]);
                `endif
            end
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin : gen_dq_hi
                `ifdef XILINX_SIMULATOR
                short bidi_dq(iDDR4[device].DQ[8 + bit_index], ddr4_dq[(device*2 + 1)*8 + bit_index]);
                `else
                tran  bidi_dq(iDDR4[device].DQ[8 + bit_index], ddr4_dq[(device*2 + 1)*8 + bit_index]);
                `endif
            end

            `ifdef XILINX_SIMULATOR
            short bidi_dqs_p0(iDDR4[device].DQS_t[0], ddr4_dqs_p[device*2]);
            short bidi_dqs_n0(iDDR4[device].DQS_c[0], ddr4_dqs_n[device*2]);
            short bidi_dqs_p1(iDDR4[device].DQS_t[1], ddr4_dqs_p[device*2 + 1]);
            short bidi_dqs_n1(iDDR4[device].DQS_c[1], ddr4_dqs_n[device*2 + 1]);
            short bidi_dm0   (iDDR4[device].DM_n[0],  ddr4_dm_n[device*2]);
            short bidi_dm1   (iDDR4[device].DM_n[1],  ddr4_dm_n[device*2 + 1]);
            `else
            tran  bidi_dqs_p0(iDDR4[device].DQS_t[0], ddr4_dqs_p[device*2]);
            tran  bidi_dqs_n0(iDDR4[device].DQS_c[0], ddr4_dqs_n[device*2]);
            tran  bidi_dqs_p1(iDDR4[device].DQS_t[1], ddr4_dqs_p[device*2 + 1]);
            tran  bidi_dqs_n1(iDDR4[device].DQS_c[1], ddr4_dqs_n[device*2 + 1]);
            tran  bidi_dm0   (iDDR4[device].DM_n[0],  ddr4_dm_n[device*2]);
            tran  bidi_dm1   (iDDR4[device].DM_n[1],  ddr4_dm_n[device*2 + 1]);
            `endif

            ddr4_model #(
                .CONFIGURED_DQ_BITS (16),
                .CONFIGURED_DENSITY (TB_DENSITY),
                .CONFIGURED_RANKS   (1)
            ) u_ddr4_model (
                .model_enable (model_enable),
                .iDDR4        (iDDR4[device])
            );
        end
    endgenerate

    // Waveform-friendly decode of the actual DDR4 initialization/refresh ROM
    // address supplied to the prober.  This is testbench-only debug state.
    wire [5:0] dbg_rom_instruction_address =
        u_dut.u_ddr4_top.u_prober.i_instruction_address;
    string dbg_rom_phase;
    always @* begin
        case (dbg_rom_instruction_address)
            6'd0:    dbg_rom_phase = "POWER_ON_RESET";
            6'd1:    dbg_rom_phase = "CKE_LOW";
            6'd2:    dbg_rom_phase = "tXPR_WAIT";
            6'd3:    dbg_rom_phase = "MRS MR3";
            6'd4:    dbg_rom_phase = "tMRD_WAIT";
            6'd5:    dbg_rom_phase = "MRS MR6";
            6'd6:    dbg_rom_phase = "tMRD_WAIT";
            6'd7:    dbg_rom_phase = "MRS MR5";
            6'd8:    dbg_rom_phase = "tMRD_WAIT";
            6'd9:    dbg_rom_phase = "MRS MR4";
            6'd10:   dbg_rom_phase = "tMRD_WAIT";
            6'd11:   dbg_rom_phase = "MRS MR2";
            6'd12:   dbg_rom_phase = "tMRD_WAIT";
            6'd13:   dbg_rom_phase = "MRS MR1";
            6'd14:   dbg_rom_phase = "tMRD_WAIT";
            6'd15:   dbg_rom_phase = "MRS MR0";
            6'd16:   dbg_rom_phase = "tMOD_WAIT";
            6'd17:   dbg_rom_phase = "ZQCL";
            6'd18:   dbg_rom_phase = "DLL_LOCK";
            6'd19:   dbg_rom_phase = "PRE_ALL";
            6'd20:   dbg_rom_phase = "MPR_ENABLE";
            6'd21:   dbg_rom_phase = "tMOD_WAIT";
            6'd22:   dbg_rom_phase = "READ_CAL";
            6'd23:   dbg_rom_phase = "MPR_DISABLE";
            6'd24:   dbg_rom_phase = "tMOD_WAIT";
            6'd25:   dbg_rom_phase = "WL_ENABLE";
            6'd26:   dbg_rom_phase = "tWLMRD_WAIT";
            6'd27:   dbg_rom_phase = "WRITE_CAL";
            6'd28:   dbg_rom_phase = "WL_DISABLE";
            6'd29:   dbg_rom_phase = "tMOD_WAIT";
            6'd30:   dbg_rom_phase = "PRE_ALL";
            6'd31:   dbg_rom_phase = "REFRESH";
            6'd32:   dbg_rom_phase = "INIT_DONE";
            6'd33:   dbg_rom_phase = "REF_PRE_ALL";
            6'd34:   dbg_rom_phase = "REF_REFRESH";
            6'd35:   dbg_rom_phase = "REF_IDLE";
            default: dbg_rom_phase = "???";
        endcase
    end

    // ------------------------------------------------------------------------
    // Terminal status and watchdog
    // ------------------------------------------------------------------------
    initial begin
        wait (init_done || init_failed);
        #1_000;
        if (init_done && !init_failed)
            $display("[AXKU3_TB_PASS] init_done=1 led=%b at %0t ps", led, $time);
        else
            $display("[AXKU3_TB_FAIL] init_failed=1 led=%b at %0t ps", led, $time);

`ifdef SIM_NATIVE_RX_DEBUG_CONTINUE_ON_FAIL
        // Diagnostic-only tail: keep the BIST read stream alive long enough
        // to distinguish the first FIFO head from the following native words.
        if (init_failed)
            repeat (32) @(posedge u_dut.controller_clk);
`endif

        // Read the live status signals feeding ddr4_prober directly.  This
        // mirrors the CSR contents without requiring the unused debug WB port.
        begin : csr_dump
            integer lane;
            $display("[%0t] === CSR-equivalent Prober Status ===", $realtime);
            $display("[%0t]   PHY FSM State        : %0d", $realtime, u_dut.u_ddr4_top.u_prober.i_phy_state);
            $display("[%0t]   Calib State          : %0d", $realtime, u_dut.u_ddr4_top.u_prober.i_calib_state);
            $display("[%0t]   Stage1 Pending       : %0b", $realtime, u_dut.u_ddr4_top.u_prober.i_stage1_pending);
            $display("[%0t]   Stage2 Pending       : %0b", $realtime, u_dut.u_ddr4_top.u_prober.i_stage2_pending);
            $display("[%0t]   Stage2 WE            : %0b", $realtime, u_dut.u_ddr4_top.u_prober.i_stage2_we);
            $display("[%0t]   Refresh Idle         : %0b", $realtime, u_dut.u_ddr4_top.u_prober.i_refresh_idle);
            $display("[%0t]   Bank Status          : %016b", $realtime, u_dut.u_ddr4_top.u_prober.i_bank_status);
            $display("[%0t]   Train Fail (gate)    : %0b", $realtime, u_dut.u_ddr4_top.u_prober.i_phy_train_fail[3:0]);
            $display("[%0t]   Train Fail (eye)     : %0b", $realtime, u_dut.u_ddr4_top.u_prober.i_phy_train_fail[7:4]);
            $display("[%0t]   Train Fail (wl)      : %0b", $realtime, u_dut.u_ddr4_top.u_prober.i_phy_train_fail[11:8]);
            $display("[%0t]   Calib Retry Count    : %0d", $realtime, u_dut.u_ddr4_top.u_prober.i_calib_retry_count);
            $display("[%0t]   BIST Correct Count   : %0d", $realtime, u_dut.u_ddr4_top.u_prober.correct_count);
            $display("[%0t]   BIST Error Count     : %0d", $realtime, u_dut.u_ddr4_top.u_prober.error_count);
            $display("[%0t]   BIST FSM State       : %0d", $realtime, u_dut.u_ddr4_top.u_prober.bist_state);
            $display("[%0t]   BIST Busy            : %0b", $realtime, u_dut.u_ddr4_top.u_prober.o_bist_busy);
            $display("[%0t]   BIST Pass            : %0b", $realtime, u_dut.u_ddr4_top.u_prober.bist_pass);
            $display("[%0t]   BIST Fail Sticky     : %0b", $realtime, u_dut.u_ddr4_top.u_prober.bist_fail_sticky);
            $display("[%0t]   Init Done            : %0b", $realtime, init_done);
            $display("[%0t]   Init Failed          : %0b", $realtime, init_failed);
            for (lane = 0; lane < 4; lane = lane + 1) begin
                $display("[%0t]   Lane %0d IDELAY Center : %0d", $realtime, lane, u_dut.u_ddr4_top.u_prober.i_phy_idelay_center[lane*9 +: 9]);
                $display("[%0t]   Lane %0d WL DQS Tap    : %0d", $realtime, lane, u_dut.u_ddr4_top.u_prober.i_phy_wl_tap[lane*9 +: 9]);
                $display("[%0t]   Lane %0d WL DQ Tap     : %0d", $realtime, lane, u_dut.u_ddr4_top.u_prober.i_phy_wl_dq_tap[lane*9 +: 9]);
                $display("[%0t]   Lane %0d DQS Init Tap  : %0d", $realtime, lane, u_dut.u_ddr4_top.u_prober.i_phy_dqs_initial_tap[lane*9 +: 9]);
                $display("[%0t]   Lane %0d Bitslip       : %0d", $realtime, lane, u_dut.u_ddr4_top.u_prober.i_phy_bitslip[lane*4 +: 4]);
                $display("[%0t]   Lane %0d Eye Start     : %0d", $realtime, lane, u_dut.u_ddr4_top.u_prober.i_phy_best_start[lane*9 +: 9]);
                $display("[%0t]   Lane %0d Eye Width     : %0d", $realtime, lane, u_dut.u_ddr4_top.u_prober.i_phy_best_width[lane*9 +: 9]);
                $display("[%0t]   Lane %0d Rd Lat Extra  : %0b", $realtime, lane, u_dut.u_ddr4_top.u_prober.i_phy_rd_lat_extra[lane]);
            end
            $display("[%0t]   EN_VTC               : %0b", $realtime, u_dut.u_ddr4_top.u_prober.i_phy_en_vtc);
            $display("[%0t]   ROM Instruction Addr : %0d", $realtime, u_dut.u_ddr4_top.u_prober.i_instruction_address);
            $display("[%0t]   Pause Counter        : %0b", $realtime, u_dut.u_ddr4_top.u_prober.i_pause_counter);
            $display("[%0t]   Reset Done           : %0b", $realtime, u_dut.u_ddr4_top.u_prober.i_reset_done);
            $display("[%0t]   Pipe Stall           : %0b", $realtime, u_dut.u_ddr4_top.u_prober.i_pipe_stall);
            $display("[%0t] === End CSR-equivalent Status ===", $realtime);
        end
        $finish;
    end

    initial begin
        #TIMEOUT_PS;
        $display("[AXKU3_TB_TIMEOUT] No terminal LED status after %0t ps; led=%b", $time, led);
        $finish;
    end

`ifdef VCD_DUMP
    initial begin
        $dumpfile("axku3_uberddr4_sim_top.vcd");
        $dumpvars(0, axku3_uberddr4_sim_top);
`ifdef SIM_NATIVE_RX_VCD_WINDOW
        // Keep primitive-level diagnostic traces small enough to inspect.
        // This window spans the BIST write-to-read turn and first returns.
        $dumpoff;
`ifdef SIM_NATIVE_WL_AB_VCD_WINDOW
        #25_200_000;
        $dumpon;
        #7_900_000;
        $dumpoff;
`else
`ifdef SIM_VCD_START_TIME
        #(`SIM_VCD_START_TIME);
`else
        #54_800_000;
`endif
        $dumpon;
`ifdef SIM_VCD_END_TIME
        #(`SIM_VCD_END_TIME - `SIM_VCD_START_TIME);
`else
        #2_000_000;
`endif
        $dumpoff;
`endif
`endif
    end
`endif

endmodule

`default_nettype wire
