////////////////////////////////////////////////////////////////////////////////
// UberDDR4 native-PHY build adapter
//
// This adapter gives the native UltraScale/UltraScale+ PHY the same DFI and
// prober-facing ports as rtl/ddr4_phy.v.  It has a distinct module name so
// both PHY implementations may be compiled together and selected by the
// ddr4_top PHY_IMPL parameter.
////////////////////////////////////////////////////////////////////////////////

`default_nettype none
`timescale 1ps / 1ps

module ddr4_phy_native_adapter #(
    parameter CONTROLLER_CLK_PERIOD = 3_333,
              DDR4_CLK_PERIOD       = 833,
              DEVICE_WIDTH          = 8,
              BYTE_LANES            = 2,
    parameter BA_BITS = 2,
              BG_BITS = (DEVICE_WIDTH == 16) ? 1 : 2,
              DQ_BITS = 8,
    parameter SERDES_RATIO = 4,
              DFI_DATA_WIDTH = 2 * DQ_BITS * BYTE_LANES,
    parameter SIM_DEVICE = "ULTRASCALE_PLUS",
              PHY_PROFILE = "GENERIC",
              FIFO_PACE_LANE = (BYTE_LANES > 0) ? BYTE_LANES-1 : 0,
              FIFO_PACE_BIT  = DQ_BITS-1,
              ACMD_NIBBLE_COUNT = 0,
    parameter [255:0] ACMD_PIN_MAP = {256{1'b1}},
    parameter [4*DQ_BITS*BYTE_LANES-1:0] DQ_PIN_MAP =
              {4*DQ_BITS*BYTE_LANES{1'b1}},
              PLL_COUNT = 1,
    parameter [95:0] ACMD_PLL_MAP = 96'd0,
    parameter [3*BYTE_LANES-1:0] BYTE_PLL_MAP =
              {3*BYTE_LANES{1'b0}}
) (
    input wire i_controller_clk, input wire i_ddr4_clk, input wire i_ref_clk,
    input wire i_rst_n,
    input wire [SERDES_RATIO*17-1:0] i_dfi_address,
    input wire [SERDES_RATIO*BA_BITS-1:0] i_dfi_bank,
    input wire [SERDES_RATIO*BG_BITS-1:0] i_dfi_bg,
    input wire [SERDES_RATIO-1:0] i_dfi_cs_n, i_dfi_act_n, i_dfi_ras_n,
    input wire [SERDES_RATIO-1:0] i_dfi_cas_n, i_dfi_we_n, i_dfi_cke,
    input wire [SERDES_RATIO-1:0] i_dfi_odt, i_dfi_reset_n,
    input wire [SERDES_RATIO*DFI_DATA_WIDTH-1:0] i_dfi_wrdata,
    input wire [SERDES_RATIO-1:0] i_dfi_wrdata_en,
    input wire [SERDES_RATIO*(2*BYTE_LANES)-1:0] i_dfi_wrdata_mask,
    output wire [SERDES_RATIO*DFI_DATA_WIDTH-1:0] o_dfi_rddata,
    output wire [SERDES_RATIO-1:0] o_dfi_rddata_valid,
    input wire [SERDES_RATIO-1:0] i_dfi_rddata_en,
    input wire i_dfi_init_start, output wire o_dfi_init_complete,
    input wire i_dfi_rdlvl_en, i_dfi_rdlvl_gate_en, i_dfi_wrlvl_en,
    input wire i_dfi_wrlvl_strobe,
    input wire [SERDES_RATIO-1:0] i_dfi_lvl_pattern,
    input wire i_dfi_lvl_periodic,
    output wire [BYTE_LANES-1:0] o_dfi_rdlvl_resp,
    output wire [BYTE_LANES-1:0] o_dfi_wrlvl_resp,
    output wire o_dfi_rdlvl_req, o_dfi_rdlvl_gate_req, o_dfi_wrlvl_req,
    input wire i_tx_diag_req,
    input wire [7:0] i_tx_diag_dq,
    input wire [8:0] i_tx_diag_tap,
    output wire o_tx_diag_ack,
    output wire o_tx_diag_error,
    output wire [8:0] o_tx_diag_current_tap,
    output wire [8:0] o_tx_diag_previous_tap,
    output wire o_ddr4_ck_p, o_ddr4_ck_n, o_ddr4_reset_n, o_ddr4_cke,
    output wire o_ddr4_cs_n, o_ddr4_act_n,
    output wire [16:0] o_ddr4_addr,
    output wire [BA_BITS-1:0] o_ddr4_ba,
    output wire [BG_BITS-1:0] o_ddr4_bg,
    output wire o_ddr4_odt,
    output wire [BYTE_LANES-1:0] o_ddr4_dm_n,
    inout wire [DQ_BITS*BYTE_LANES-1:0] io_ddr4_dq,
    inout wire [BYTE_LANES-1:0] io_ddr4_dqs_p,
    inout wire [BYTE_LANES-1:0] io_ddr4_dqs_n,
    output wire [3:0] o_phy_state,
    output wire [9*BYTE_LANES-1:0] o_phy_idelay_center,
    output wire [9*BYTE_LANES-1:0] o_phy_wl_tap,
    output wire [4*BYTE_LANES-1:0] o_phy_bitslip,
    output wire [BYTE_LANES-1:0] o_phy_train_fail_gate,
    output wire [BYTE_LANES-1:0] o_phy_train_fail_eye,
    output wire [BYTE_LANES-1:0] o_phy_train_fail_wl,
    output wire [9*BYTE_LANES-1:0] o_phy_best_width,
    output wire [9*BYTE_LANES-1:0] o_phy_best_start,
    output wire [9*BYTE_LANES-1:0] o_phy_wl_dq_tap,
    output wire [9*BYTE_LANES-1:0] o_phy_dqs_initial_tap,
    output wire [BYTE_LANES-1:0] o_phy_rd_lat_extra,
    output wire o_phy_en_vtc
);

    // Keep the component-PHY's simulation/debug hierarchy available at the
    // DFI boundary.  Existing benches intentionally inspect these per-lane
    // training values through u_dut.u_phy without requiring a native-only
    // testbench or any controller/prober changes.
    wire [3:0] phy_state = o_phy_state;
    wire [BYTE_LANES-1:0] rd_lat_extra = o_phy_rd_lat_extra;
    wire [8:0] eye_center_tap [0:BYTE_LANES-1];
    wire [3:0] bitslip_count_q [0:BYTE_LANES-1];
    wire [8:0] wl_tap [0:BYTE_LANES-1];
    wire [8:0] wl_dq_tap [0:BYTE_LANES-1];

    generate
        genvar dbg_lane;
        for (dbg_lane = 0; dbg_lane < BYTE_LANES; dbg_lane = dbg_lane + 1) begin : gen_debug_alias
            assign eye_center_tap[dbg_lane] = o_phy_idelay_center[dbg_lane*9 +: 9];
            assign bitslip_count_q[dbg_lane] = o_phy_bitslip[dbg_lane*4 +: 4];
            assign wl_tap[dbg_lane] = o_phy_wl_tap[dbg_lane*9 +: 9];
            assign wl_dq_tap[dbg_lane] = o_phy_wl_dq_tap[dbg_lane*9 +: 9];
        end
    endgenerate

    ddr4_phy_native #(
        .CONTROLLER_CLK_PERIOD(CONTROLLER_CLK_PERIOD),
        .DDR4_CLK_PERIOD(DDR4_CLK_PERIOD),
        .DEVICE_WIDTH(DEVICE_WIDTH), .BYTE_LANES(BYTE_LANES),
        .BA_BITS(BA_BITS), .BG_BITS(BG_BITS), .DQ_BITS(DQ_BITS),
        .SERDES_RATIO(SERDES_RATIO), .DFI_DATA_WIDTH(DFI_DATA_WIDTH),
        .SIM_DEVICE(SIM_DEVICE), .PHY_PROFILE(PHY_PROFILE),
        .FIFO_PACE_LANE(FIFO_PACE_LANE), .FIFO_PACE_BIT(FIFO_PACE_BIT),
        .ACMD_NIBBLE_COUNT(ACMD_NIBBLE_COUNT),
        .ACMD_PIN_MAP(ACMD_PIN_MAP), .DQ_PIN_MAP(DQ_PIN_MAP),
        .PLL_COUNT(PLL_COUNT), .ACMD_PLL_MAP(ACMD_PLL_MAP),
        .BYTE_PLL_MAP(BYTE_PLL_MAP)
    ) u_native (
        .i_controller_clk(i_controller_clk), .i_ddr4_clk(i_ddr4_clk),
        .i_ref_clk(i_ref_clk), .i_rst_n(i_rst_n),
        .i_dfi_address(i_dfi_address), .i_dfi_bank(i_dfi_bank),
        .i_dfi_bg(i_dfi_bg), .i_dfi_cs_n(i_dfi_cs_n),
        .i_dfi_act_n(i_dfi_act_n), .i_dfi_ras_n(i_dfi_ras_n),
        .i_dfi_cas_n(i_dfi_cas_n), .i_dfi_we_n(i_dfi_we_n),
        .i_dfi_cke(i_dfi_cke), .i_dfi_odt(i_dfi_odt),
        .i_dfi_reset_n(i_dfi_reset_n), .i_dfi_wrdata(i_dfi_wrdata),
        .i_dfi_wrdata_en(i_dfi_wrdata_en),
        .i_dfi_wrdata_mask(i_dfi_wrdata_mask), .o_dfi_rddata(o_dfi_rddata),
        .o_dfi_rddata_valid(o_dfi_rddata_valid),
        .i_dfi_rddata_en(i_dfi_rddata_en),
        .i_dfi_init_start(i_dfi_init_start),
        .o_dfi_init_complete(o_dfi_init_complete),
        .i_dfi_rdlvl_en(i_dfi_rdlvl_en),
        .i_dfi_rdlvl_gate_en(i_dfi_rdlvl_gate_en),
        .i_dfi_wrlvl_en(i_dfi_wrlvl_en),
        .i_dfi_wrlvl_strobe(i_dfi_wrlvl_strobe),
        .i_dfi_lvl_pattern(i_dfi_lvl_pattern),
        .i_dfi_lvl_periodic(i_dfi_lvl_periodic),
        .o_dfi_rdlvl_resp(o_dfi_rdlvl_resp),
        .o_dfi_wrlvl_resp(o_dfi_wrlvl_resp),
        .o_dfi_rdlvl_req(o_dfi_rdlvl_req),
        .o_dfi_rdlvl_gate_req(o_dfi_rdlvl_gate_req),
        .o_dfi_wrlvl_req(o_dfi_wrlvl_req), .o_ddr4_ck_p(o_ddr4_ck_p),
        .i_tx_diag_req(i_tx_diag_req), .i_tx_diag_dq(i_tx_diag_dq),
        .i_tx_diag_tap(i_tx_diag_tap), .o_tx_diag_ack(o_tx_diag_ack),
        .o_tx_diag_error(o_tx_diag_error),
        .o_tx_diag_current_tap(o_tx_diag_current_tap),
        .o_tx_diag_previous_tap(o_tx_diag_previous_tap),
        .o_ddr4_ck_n(o_ddr4_ck_n), .o_ddr4_reset_n(o_ddr4_reset_n),
        .o_ddr4_cke(o_ddr4_cke), .o_ddr4_cs_n(o_ddr4_cs_n),
        .o_ddr4_act_n(o_ddr4_act_n), .o_ddr4_addr(o_ddr4_addr),
        .o_ddr4_ba(o_ddr4_ba), .o_ddr4_bg(o_ddr4_bg),
        .o_ddr4_odt(o_ddr4_odt), .o_ddr4_dm_n(o_ddr4_dm_n),
        .io_ddr4_dq(io_ddr4_dq), .io_ddr4_dqs_p(io_ddr4_dqs_p),
        .io_ddr4_dqs_n(io_ddr4_dqs_n), .o_phy_state(o_phy_state),
        .o_phy_idelay_center(o_phy_idelay_center), .o_phy_wl_tap(o_phy_wl_tap),
        .o_phy_bitslip(o_phy_bitslip),
        .o_phy_train_fail_gate(o_phy_train_fail_gate),
        .o_phy_train_fail_eye(o_phy_train_fail_eye),
        .o_phy_train_fail_wl(o_phy_train_fail_wl),
        .o_phy_best_width(o_phy_best_width), .o_phy_best_start(o_phy_best_start),
        .o_phy_wl_dq_tap(o_phy_wl_dq_tap),
        .o_phy_dqs_initial_tap(o_phy_dqs_initial_tap),
        .o_phy_rd_lat_extra(o_phy_rd_lat_extra), .o_phy_en_vtc(o_phy_en_vtc)
    );
endmodule

`default_nettype wire
