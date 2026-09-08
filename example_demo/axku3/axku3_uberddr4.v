////////////////////////////////////////////////////////////////////////////////
// AXKU3 DDR4 bring-up top level (build/clock/IP instructions: README.md here)
//
// Board: ALINX AXKU3, XCKU3P-FFVB676-2-I
// DRAM:  two Micron MT40A512M16LY-062E (2 x x16 = 32-bit interface)
//
// Create one Clocking Wizard IP named "clk_wiz_0" before synthesizing this
// design.  The IBUFDS/BUFG below is the ONLY buffer for the board's 200 MHz
// differential oscillator.  Configure the wizard input as single-ended
// "No Buffer", name it clk_in1, and generate both outputs from the same MMCM:
//
//   clk_wiz_0 / ddr4_clk   : 300.000000 MHz, normal Buffer
//                -> quarter-rate controller/DFI clock for the native PHY
//   clk_wiz_0 / ref300_clk : controller_clk / 2 (150.000000 MHz here),
//                            zero-degree phase, normal Buffer
//                -> native BITSLICE register-interface (RIU) clock
//
// UG571 requires the PLLE4 input clock and BITSLICE_CONTROL RIU_CLK to come
// from the same MMCM with the same phase shift when RL_DLY_RNK is used.  Do
// not generate these two clocks with independent Clocking Wizard instances.
//
// The native PHY instantiates one PLLE4/CLKOUTPHY per occupied I/O clock
// region.  Each local PLL multiplies the 300 MHz word clock by four and uses
// VCO_2X to deliver the 2.4 GHz serial BITSLICE clock required for a 1.2 GHz
// DDR4 CK (DDR4-2400).  The high-speed clock remains entirely on
// dedicated XPHY routing and never crosses a frequency-limited global buffer.
////////////////////////////////////////////////////////////////////////////////

`default_nettype none
`timescale 1ps / 1ps

module axku3_uberddr4 (
    // 200 MHz differential oscillator and active-low pushbutton reset
    input  wire        sys_clk_p,
    input  wire        sys_clk_n,
    input  wire        rst_n,

    // Board LEDs: LED0 = initialization passed; all LEDs = initialization failed
    output wire [3:0]  led,

    // Board fan PWM control: active low, held enabled continuously
    output wire        fan_pwm,

    // 32-bit DDR4 interface (two x16 MT40A512M16LY-062E devices)
    output wire        ddr4_ck_p,
    output wire        ddr4_ck_n,
    output wire        ddr4_reset_n,
    output wire        ddr4_cke,
    output wire        ddr4_cs_n,
    output wire        ddr4_act_n,
    output wire [16:0] ddr4_addr,
    output wire [1:0]  ddr4_ba,
    output wire        ddr4_bg,
    output wire        ddr4_odt,
    output wire [3:0]  ddr4_dm_n,
    inout  wire [31:0] ddr4_dq,
    inout  wire [3:0]  ddr4_dqs_p,
    inout  wire [3:0]  ddr4_dqs_n
);

    wire controller_clk;
    wire ref_clk;
    wire locked_0;
    wire sys_clk_ibuf;
    wire sys_clk;

    // Buffer the board clock once.  Do not configure the Clocking Wizard to
    // instantiate an additional IBUF/IBUFDS.
    IBUFDS sys_clk_ibuf_inst (
        .I (sys_clk_p),
        .IB(sys_clk_n),
        .O (sys_clk_ibuf)
    );
    BUFG sys_clk_buf (
        .I(sys_clk_ibuf),
        .O(sys_clk)
    );

    clk_wiz_0 clk_wiz_0_inst
     (
      // Clock out ports
      .ddr4_clk(controller_clk),
      .ref300_clk(ref_clk),
      // Status and control signals
      .reset(~rst_n),
      .locked(locked_0),
     // Clock in ports
      .clk_in1(sys_clk)
     );

    // Keep the controller in reset until the manually-created Clocking Wizard
    // has locked. The pushbutton asynchronously clears this wrapper register;
    // lock is sampled on controller_clk. The core also has synchronous state.
    reg ddr4_rst_n;
    always @(posedge controller_clk, negedge rst_n) begin
        if (!rst_n) begin
            ddr4_rst_n <= 1'b0;
        end else begin
            ddr4_rst_n <= locked_0;
        end
    end
    wire init_done;
    wire init_failed;

    // No external traffic generator is connected. BIST_MODE=2 owns the main
    // Wishbone port during bring-up; all external Wishbone inputs are inactive.
    // Status persists until external or internal recovery reset. Runtime status
    // and recovery counters require ILA; DEBUG_CSR_ENABLE does not remove probes.
    ddr4_top #(
        // Quarter-rate DFI relationship for a 300 MHz controller clock and
        // 1.2 GHz DDR4 CK (DDR4-2400).  Integer picosecond parameters round
        // downward; verify derived timing against the actual memory and clocks.
        // These integers are not an independent proof of every JEDEC minimum.
        .CONTROLLER_CLK_PERIOD(3_332),
        .DDR4_CLK_PERIOD      (833),
        .DEVICE_WIDTH          (16),
        .ROW_BITS              (16),
        .COL_BITS              (10),
        .BYTE_LANES            (4),    // two x16 devices = four byte lanes
        .DENSITY               (8),
        // 0 = component PHY; 1 = native PHY used by this AXKU3 example.
        // Set to 1 after adding the native PHY source files to the project.
        // ddr4_top then applies the native PHY's required DFI timing itself.
        .PHY_IMPL              (1),
        // Native BITSLICE connectivity follows the physical nibble/position
        // encoded by each Bank 66/67 package pin in axku3_uberddr4.xdc.
        // ACMD entries are {nibble[4:0], position[2:0]} for logical
        // {CK,RESET,ODT,CKE,ACT,CS,BG,BA,A}; unused high entries are 8'hff.
        .PHY_ACMD_NIBBLE_COUNT (6),
        .PHY_ACMD_PIN_MAP      ({128'hffff_ffff_ffff_1009_1308_1524_0522_022e,
                                  128'h2304_1928_1a20_2d1b_2c03_182b_2921_252a}),
        // Four bits per logical DQ: {upper_nibble, position[2:0]}.
        // This preserves DQ numbering while connecting each RXTX_BITSLICE to
        // the dedicated BIT_CTRL route of its actual Bank 67 byte position.
        .PHY_DQ_PIN_MAP        (128'h253b_dac4_2ba5_3dc4_5dbc_a342_4ca2_35bd),
        // Bank 66 (ACMD) and Bank 67 (all four data bytes) occupy adjacent
        // I/O clock regions.  UG571 requires a local CLKOUTPHY per region;
        // this is the same two-PLL topology generated by the working MIG.
        .PHY_PLL_COUNT         (2),
        .PHY_ACMD_PLL_MAP      (96'd0),
        .PHY_BYTE_PLL_MAP      (12'h249), // lanes 3:0 all select PLL 1
        .BIST_MODE             (2),
        .BIST_DM_TEST          (0),
        // Board bring-up diagnostic: on the first mismatch, reread the same
        // address 32 times so ILA can distinguish write storage from RX noise.
        .BIST_REREAD_DIAG      (1),
        .DEBUG_CSR_ENABLE      (0)
    ) u_ddr4_top (
        .i_controller_clk(controller_clk),
        // Retained for the common ddr4_top interface. Native mode generates
        // its high-speed clocks locally and does not consume i_ddr4_clk.
        .i_ddr4_clk      (controller_clk),
        .i_ref_clk       (ref_clk),
        .i_rst_n         (ddr4_rst_n),

        .i_wb_cyc        (1'b0),
        .i_wb_stb        (1'b0),
        .i_wb_we         (1'b0),
        .i_wb_addr       (26'b0),
        .i_wb_data       (256'b0),
        .i_wb_sel        (32'b0),
        .o_wb_stall      (),
        .o_wb_ack        (),
        .o_wb_data       (),

        .i_wb_dbg_cyc    (1'b0),
        .i_wb_dbg_stb    (1'b0),
        .i_wb_dbg_we     (1'b0),
        .i_wb_dbg_addr   (4'b0),
        .i_wb_dbg_data   (32'b0),
        .i_wb_dbg_sel    (4'b0),
        .o_wb_dbg_stall  (),
        .o_wb_dbg_ack    (),
        .o_wb_dbg_data   (),

        .o_ddr4_ck_p     (ddr4_ck_p),
        .o_ddr4_ck_n     (ddr4_ck_n),
        .o_ddr4_reset_n  (ddr4_reset_n),
        .o_ddr4_cke      (ddr4_cke),
        .o_ddr4_cs_n     (ddr4_cs_n),
        .o_ddr4_act_n    (ddr4_act_n),
        .o_ddr4_addr     (ddr4_addr),
        .o_ddr4_ba       (ddr4_ba),
        .o_ddr4_bg       (ddr4_bg),
        .o_ddr4_odt      (ddr4_odt),
        .o_ddr4_dm_n     (ddr4_dm_n),
        .io_ddr4_dq      (ddr4_dq),
        .io_ddr4_dqs_p   (ddr4_dqs_p),
        .io_ddr4_dqs_n   (ddr4_dqs_n),
        .o_init_done     (init_done),
        .o_init_failed   (init_failed)
    );

    // The AXKU3 board's fan control input is active low.  A constant low
    // enables the fan continuously; no PWM generator is needed for bring-up.
    assign fan_pwm = 1'b0;

    // Light LED0 if failed, light LED1-3 if passed
    assign led = {{3{init_done}}, init_failed};

endmodule

`default_nettype wire
