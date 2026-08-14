////////////////////////////////////////////////////////////////////////////////
// AXKU3 DDR4 bring-up top level
//
// Board: ALINX AXKU3, XCKU3P-FFVB676-2-I
// DRAM:  two Micron MT40A512M16LY-062E (2 x x16 = 32-bit interface)
//
// Create two Clocking Wizard IPs named "clk_wiz_0" and "clk_wiz_1" before
// synthesizing this design.  The IBUFDS/BUFG below is the ONLY buffer for the
// board's 200 MHz differential oscillator.  Both wizard inputs therefore must
// be configured as single-ended "No Buffer" inputs and use port name clk_in1.
// Configure their outputs as follows:
//
//   clk_wiz_0 / ddr4_clk   : 625 MHz (1.600 ns), No Buffer
//                -> raw MMCM output; buffered below for OSERDESE3/ISERDESE3
//   clk_wiz_1 / ref300_clk : 300 MHz (3.333 ns), normal Buffer
//              -> IDELAYCTRL reference clock
//
// The 156.25 MHz controller/CLKDIV clock is deliberately NOT a wizard output.
// The raw 625 MHz MMCM output drives a BUFGCE and a BUFGCE_DIV /4 IN PARALLEL
// below.  This is the component-mode topology required by OSERDESE3/ISERDESE3:
// CLK and CLKDIV use sibling dedicated global buffers, rather than cascading a
// BUFGCE_DIV from the wizard's buffered output.  Cascading them produces an
// excessive CLK-to-CLKDIV skew and fails the OSERDESE3 max-skew check.
//
// 625 MHz gives tCK = 1.600 ns (1250 MT/s nominal), meeting the JEDEC
// DDR4-1333 maximum tCK and this -2 speed grade's 1.600 ns minimum
// OSERDESE3/ISERDESE3 CLK period.  DDR4-1333 at 666.667 MHz does not meet the
// latter requirement.  Faster operation requires
// a PHY-specific PLLE4 CLKOUTPHY/XIPHY clocking architecture.
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
    wire ddr4_clk;
    wire ddr4_clk_mmcm;
    wire ref_clk;
    wire locked_0;
    wire locked_1;
    wire sys_clk_ibuf;
    wire sys_clk;

    // Share the board clock safely between the two MMCMs.  Do not configure
    // either Clock Wizard to instantiate an additional IBUF/IBUFDS.
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
      .ddr4_clk(ddr4_clk_mmcm),
      // Status and control signals
      .reset(~rst_n),
      .locked(locked_0),
     // Clock in ports
      .clk_in1(sys_clk)
     );

    clk_wiz_1 clk_wiz_1_inst
     (
      // Clock out ports
      .ref300_clk(ref_clk),
      // Status and control signals
      .reset(~rst_n),
      .locked(locked_1),
      // Clock in ports -- configured as single-ended, No Buffer
      .clk_in1(sys_clk)
     );

    // Keep the high-speed CLK and divided CLKDIV as parallel descendants of
    // the same raw MMCM output.  clk_wiz_0's 625 MHz output must be configured
    // as "No Buffer"; inserting its output buffer here would recreate the
    // prohibited BUFGCE -> BUFGCE_DIV clock-buffer cascade.
    BUFGCE #(
        .SIM_DEVICE("ULTRASCALE_PLUS")
    ) ddr4_clk_buf (
        .I (ddr4_clk_mmcm),
        .CE(1'b1),
        .O (ddr4_clk)
    );

    // OSERDESE3/ISERDESE3 require CLKDIV to be a dedicated /4 clock derived
    // from the same MMCM output as CLK.  Do not replace this with another MMCM
    // output or a fabric divider.
    BUFGCE_DIV #(
        .BUFGCE_DIVIDE(4),
        .SIM_DEVICE("ULTRASCALE_PLUS")
    ) controller_clk_buf (
        .I  (ddr4_clk_mmcm),
        .CE (1'b1),
        .CLR(1'b0),
        .O  (controller_clk)
    );

    // Keep the controller in reset until the manually-created Clocking Wizard
    // has locked.  This is an asynchronous assertion path as required by
    // ddr4_top.i_rst_n.
    reg ddr4_rst_n;
    always @(posedge controller_clk, negedge rst_n) begin
        if (!rst_n) begin
            ddr4_rst_n <= 1'b0;
        end else begin
            ddr4_rst_n <= locked_0 && locked_1;
        end
    end
    wire init_done;
    wire init_failed;

    // No external traffic generator is connected.  BIST_MODE=0 keeps the
    // Wishbone port quiescent; all unused Wishbone inputs are tied inactive.
    // Status is sticky inside ddr4_top, so the display persists after training.
    ddr4_top #(
        .CONTROLLER_CLK_PERIOD(6_400), // 156.25 MHz: DDR4_CLK_PERIOD * 4
        .DDR4_CLK_PERIOD      (1_600), // 625 MHz DDR4 clock (tCK = 1.600 ns)
        .DEVICE_WIDTH          (16),
        .ROW_BITS              (16),
        .COL_BITS              (10),
        .BYTE_LANES            (4),    // two x16 devices = four byte lanes
        .DENSITY               (8),
        // Safe for both PHYs and required by the native BITSLICE PHY: its
        // initial CKE transition reaches the pins nine controller clocks
        // later than the steady-state CA command path.
        .TPHY_INIT_LAT         (9),
        .BIST_MODE             (2),
        .BIST_DM_TEST          (0),
        .DEBUG_CSR_ENABLE      (0)
    ) u_ddr4_top (
        .i_controller_clk(controller_clk),
        .i_ddr4_clk      (ddr4_clk),
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
