// ddr4_phy_native_byte.v
// One byte lane (8 DQ + 1 DQS pair + 1 DM) for the native-mode DDR4 PHY on
// UltraScale and UltraScale+ devices. The explicit lower/upper-nibble
// structure mirrors the physical BITSLICE layout and is intentionally kept
// visible for placement review against an implemented design.

`timescale 1ps / 1ps
`default_nettype none

module ddr4_phy_native_byte #(
    parameter DQ_BITS     = 8,
    parameter REFCLK_FREQ = 300.0,
    parameter SIM_DEVICE  = "ULTRASCALE_PLUS",
    // One four-bit entry per logical DQ: {upper_nibble, position[2:0]}.
    // Valid data positions are lower/upper 2..5.  All ones selects the
    // canonical simulation map (DQ0..3 lower, DQ4..7 upper).
    parameter [4*DQ_BITS-1:0] DQ_PIN_MAP = {4*DQ_BITS{1'b1}}
)(
    // Clocks
    input  wire        i_pll_clkoutphy,
    input  wire        i_div_clk,
    // Reset
    input  wire        i_bsc_rst,
    input  wire        i_bitslice_rst,
    // Clears the RX deserializer/FIFO pointers while RX_RST_DLY remains low,
    // preserving the trained input-delay value.
    input  wire        i_rx_fifo_rst,
    // BISC status
    output wire        o_dly_rdy,
    output wire        o_vtc_rdy,
    // EN_VTC control
    input  wire        i_bsc_en_vtc,
    input  wire        i_bitslice_en_vtc,
    // TX data (write path)
    input  wire [DQ_BITS*8-1:0] i_tx_dq_data,
    input  wire [7:0]           i_tx_dqs_data,
    input  wire [7:0]           i_tx_dm_data,
    // TX tristate control
    input  wire [3:0]  i_tbyte_dq,
    input  wire [3:0]  i_tbyte_dqs,
    input  wire [3:0]  i_phy_rden,
    // RX data (read path)
    output wire [DQ_BITS*8-1:0] o_rx_dq_data,
    output wire [DQ_BITS-1:0]   o_fifo_empty,
    // Each asynchronous DQ FIFO has its own registered read enable.
    input  wire [DQ_BITS-1:0]   i_fifo_rd_en,
    // Per-byte RIU access to the upper nibble that owns the DQS input and
    // therefore the read-gate delay.  The lower nibble receives the upper
    // nibble's gated PCLK/NCLK through the dedicated inter-nibble path; it
    // must not be selected for the same RIU transaction.
    input  wire [5:0]           i_riu_addr,
    input  wire [15:0]          i_riu_wr_data,
    input  wire                 i_riu_wr_en,
    input  wire                 i_riu_nibble_sel,
    output wire [15:0]          o_riu_rd_data,
    output wire                 o_riu_valid,
    // RX delay control
    input  wire [8:0]  i_rx_cntvaluein,
    input  wire        i_rx_load,
    output wire [8:0]  o_rx_cntvalueout_dq0,
    // TX delay control for DQ
    input  wire [8:0]  i_tx_dq_cntvaluein,
    input  wire        i_tx_dq_load,
    // TX delay control for DQS
    input  wire [8:0]  i_tx_dqs_cntvaluein,
    input  wire        i_tx_dqs_load,
    output wire [8:0]  o_tx_dqs_cntvalueout,
    // DDR4 physical pins
    inout  wire [DQ_BITS-1:0] io_ddr4_dq,
    inout  wire               io_ddr4_dqs_p,
    inout  wire               io_ddr4_dqs_n,
    output wire               o_ddr4_dm_n
);
// ---------------------------------------------------------------------------
// Physical DQ placement within this byte
// ---------------------------------------------------------------------------
function [3:0] dq_map_entry;
    input integer logical_dq;
    begin
        if (&DQ_PIN_MAP) begin
            dq_map_entry[3]   = (logical_dq >= 4);
            dq_map_entry[2:0] = (logical_dq % 4) + 2;
        end else begin
            dq_map_entry = DQ_PIN_MAP[logical_dq*4 +: 4];
        end
    end
endfunction

function integer dq_pin_at;
    input integer upper_nibble;
    input integer position;
    integer logical_dq;
    begin
        dq_pin_at = -1;
        for (logical_dq = 0; logical_dq < DQ_BITS;
             logical_dq = logical_dq + 1)
            if (dq_map_entry(logical_dq) ==
                ((upper_nibble ? 4'h8 : 4'h0) | position))
                dq_pin_at = logical_dq;
    end
endfunction

function integer dq_slot_occupancy;
    input integer upper_nibble;
    input integer position;
    integer logical_dq;
    begin
        dq_slot_occupancy = 0;
        for (logical_dq = 0; logical_dq < DQ_BITS;
             logical_dq = logical_dq + 1)
            if (dq_map_entry(logical_dq) ==
                ((upper_nibble ? 4'h8 : 4'h0) | position))
                dq_slot_occupancy = dq_slot_occupancy + 1;
    end
endfunction

`ifndef SYNTHESIS
integer dq_check_pin;
integer dq_check_pos;
initial begin
    if (DQ_BITS != 8)
        $error("Native byte PHY requires eight DQ bits per byte lane");
    for (dq_check_pin = 0; dq_check_pin < DQ_BITS;
         dq_check_pin = dq_check_pin + 1)
        if (((dq_map_entry(dq_check_pin) & 4'h7) < 4'd2) ||
            ((dq_map_entry(dq_check_pin) & 4'h7) > 4'd5))
            $error("Native PHY DQ%0d has invalid byte map entry 0x%01x",
                   dq_check_pin, dq_map_entry(dq_check_pin));
    for (dq_check_pos = 2; dq_check_pos <= 5;
         dq_check_pos = dq_check_pos + 1) begin
        if (dq_slot_occupancy(0, dq_check_pos) != 1)
            $error("Native PHY lower-nibble position %0d must map exactly one DQ",
                   dq_check_pos);
        if (dq_slot_occupancy(1, dq_check_pos) != 1)
            $error("Native PHY upper-nibble position %0d must map exactly one DQ",
                   dq_check_pos);
    end
end
`endif

// A byte has one physical CLB-to-RIU ingress bus shared by its two nibble
// controls (UG571, RIU_OR topology).  Register a byte-local copy to prevent
// cross-byte fanout onto that dedicated route.  Only the upper nibble is
// selected: it owns DQS and its gate delay/status; the lower nibble receives
// the upper nibble's trained P/N clocks through EN_OTHER_PCLK/NCLK.
(* DONT_TOUCH = "TRUE", SHREG_EXTRACT = "NO" *)
reg [5:0]  riu_addr_q;
(* DONT_TOUCH = "TRUE", SHREG_EXTRACT = "NO" *)
reg [15:0] riu_wr_data_q;
(* DONT_TOUCH = "TRUE", SHREG_EXTRACT = "NO" *)
reg        riu_wr_en_q, riu_nibble_sel_q;

always @(posedge i_div_clk) begin
    if (i_bsc_rst) begin
        riu_addr_q       <= 6'd0;
        riu_wr_data_q    <= 16'd0;
        riu_wr_en_q      <= 1'b0;
        riu_nibble_sel_q <= 1'b0;
    end else begin
        riu_addr_q       <= i_riu_addr;
        riu_wr_data_q    <= i_riu_wr_data;
        riu_wr_en_q      <= i_riu_wr_en;
        riu_nibble_sel_q <= i_riu_nibble_sel;
    end
end
// ---------------------------------------------------------------------------
// Internal wires - BIT_CTRL buses (40-bit each)
// ---------------------------------------------------------------------------
// Lower nibble BITSLICE_CONTROL <-> bitslices
/* verilator lint_off UNUSEDSIGNAL */
wire [39:0] rx_bit_ctrl_out0_low, rx_bit_ctrl_out1_low, rx_bit_ctrl_out2_low;
wire [39:0] rx_bit_ctrl_out3_low, rx_bit_ctrl_out4_low, rx_bit_ctrl_out5_low;
wire [39:0] rx_bit_ctrl_out6_low;
wire [39:0] rx_bit_ctrl_in0_low, rx_bit_ctrl_in1_low, rx_bit_ctrl_in2_low;
wire [39:0] rx_bit_ctrl_in3_low, rx_bit_ctrl_in4_low, rx_bit_ctrl_in5_low;
wire [39:0] rx_bit_ctrl_in6_low;
wire [39:0] tx_bit_ctrl_out0_low, tx_bit_ctrl_out1_low, tx_bit_ctrl_out2_low;
wire [39:0] tx_bit_ctrl_out3_low, tx_bit_ctrl_out4_low, tx_bit_ctrl_out5_low;
wire [39:0] tx_bit_ctrl_out6_low;
wire [39:0] tx_bit_ctrl_in0_low, tx_bit_ctrl_in1_low, tx_bit_ctrl_in2_low;
wire [39:0] tx_bit_ctrl_in3_low, tx_bit_ctrl_in4_low, tx_bit_ctrl_in5_low;
wire [39:0] tx_bit_ctrl_in6_low;
wire [39:0] tx_bit_ctrl_out_tri_low, tx_bit_ctrl_in_tri_low;
// Upper nibble BITSLICE_CONTROL <-> bitslices
wire [39:0] rx_bit_ctrl_out0_upp, rx_bit_ctrl_out1_upp, rx_bit_ctrl_out2_upp;
wire [39:0] rx_bit_ctrl_out3_upp, rx_bit_ctrl_out4_upp, rx_bit_ctrl_out5_upp;
wire [39:0] rx_bit_ctrl_out6_upp;
wire [39:0] rx_bit_ctrl_in0_upp, rx_bit_ctrl_in1_upp, rx_bit_ctrl_in2_upp;
wire [39:0] rx_bit_ctrl_in3_upp, rx_bit_ctrl_in4_upp, rx_bit_ctrl_in5_upp;
wire [39:0] rx_bit_ctrl_in6_upp;
wire [39:0] tx_bit_ctrl_out0_upp, tx_bit_ctrl_out1_upp, tx_bit_ctrl_out2_upp;
wire [39:0] tx_bit_ctrl_out3_upp, tx_bit_ctrl_out4_upp, tx_bit_ctrl_out5_upp;
wire [39:0] tx_bit_ctrl_out6_upp;
wire [39:0] tx_bit_ctrl_in0_upp, tx_bit_ctrl_in1_upp, tx_bit_ctrl_in2_upp;
wire [39:0] tx_bit_ctrl_in3_upp, tx_bit_ctrl_in4_upp, tx_bit_ctrl_in5_upp;
wire [39:0] tx_bit_ctrl_in6_upp;
wire [39:0] tx_bit_ctrl_out_tri_upp, tx_bit_ctrl_in_tri_upp;
/* verilator lint_on UNUSEDSIGNAL */
// Inter-nibble clocking
wire pclk_nibble_out_upp, nclk_nibble_out_upp;
// DLY/VTC ready per nibble
wire dly_rdy_low, dly_rdy_upp;
wire vtc_rdy_low, vtc_rdy_upp;
// Tristate serial outputs (driven by TX_BITSLICE_TRI through BIT_CTRL bus in hardware;
// for simulation with empty stubs, default to output-enabled)
wire tbyte_out_low;
wire tbyte_out_upp;
// DQ IOB wires
wire [DQ_BITS-1:0] dq_to_iob;
wire [DQ_BITS-1:0] dq_from_iob;
wire [DQ_BITS-1:0] dq_t;
// DQS IOB wires
wire dqs_to_iob, dqs_from_iob, dqs_t;
// DM output wire
wire dm_to_obuf;

// Only the DQS-owning upper nibble participates in parent RIU transactions.
// The lower nibble still receives the broadcast address/write controls, but
// its readback is intentionally left unused.
wire [15:0] riu_rd_data_upp;
wire        riu_rd_valid_upp;
// ---------------------------------------------------------------------------
// Status outputs
// ---------------------------------------------------------------------------
assign o_dly_rdy = dly_rdy_low & dly_rdy_upp;
assign o_vtc_rdy = vtc_rdy_low & vtc_rdy_upp;
assign o_riu_rd_data = riu_rd_data_upp;
assign o_riu_valid   = riu_rd_valid_upp;
// ---------------------------------------------------------------------------
// Tie off unused bit positions (position 1 and 6 in lower, position 1 and 6 in upper)
// ---------------------------------------------------------------------------
assign rx_bit_ctrl_in1_low = 40'd0;
assign tx_bit_ctrl_in1_low = 40'd0;
assign rx_bit_ctrl_in6_low = 40'd0;
assign tx_bit_ctrl_in6_low = 40'd0;
assign rx_bit_ctrl_in1_upp = 40'd0;
assign tx_bit_ctrl_in1_upp = 40'd0;
assign rx_bit_ctrl_in6_upp = 40'd0;
assign tx_bit_ctrl_in6_upp = 40'd0;
// ---------------------------------------------------------------------------
// Lower Nibble BITSLICE_CONTROL
// ---------------------------------------------------------------------------
/* verilator lint_off PINCONNECTEMPTY */
/* verilator lint_off PINMISSING */
BITSLICE_CONTROL #(
    // Match the generated UltraScale DDR4 PHY attributes explicitly.  These
    // are not cosmetic: CTRL_CLK selects the RIU clock domain and dynamic
    // ODELAY mode allows the write-leveling loads driven below to reach the
    // native delay elements while BISC continues to track PVT.
    .CTRL_CLK          ("EXTERNAL"),
    .DIV_MODE           ("DIV4"),
    .SERIAL_MODE        ("FALSE"),
    .EN_DYN_ODLY_MODE   ("TRUE"),
    .IDLY_VT_TRACK      ("TRUE"),
    .ODLY_VT_TRACK      ("TRUE"),
    .QDLY_VT_TRACK      ("TRUE"),
    .ROUNDING_FACTOR    (16),
    .RXGATE_EXTEND      ("FALSE"),
    // The generated DDR4 MIG sets both data-byte nibble controls to SHIFT_90.
    // This is the native XiPHY FIFO framing phase; IDELAY then centers DQ
    // within the resulting DQS sampling eye.
    .RX_CLK_PHASE_P     ("SHIFT_90"),
    .RX_CLK_PHASE_N     ("SHIFT_90"),
    .EN_OTHER_PCLK      ("TRUE"),
    .EN_OTHER_NCLK      ("TRUE"),
    .SELF_CALIBRATE     ("ENABLE"),
`ifdef SIM_NATIVE_RX_GATE_DISABLE
    .RX_GATING          ("DISABLE"),
`else
    .RX_GATING          ("ENABLE"),
`endif
    // Match the DDR4 MIG native-PHY setting.  A zero idle count makes the
    // gate detector close/restart inside legal back-to-back BL8 traffic and
    // splits the final burst across an invalid FIFO word.
    .READ_IDLE_COUNT    (31),
    // Match the native MIG topology: TX gating is enabled while the two
    // PHY_WRCS buses remain Low.  They are native clock-select controls,
    // not aliases for DFI wrdata_en; byte output ownership is TBYTE_IN.
    .TX_GATING          ("ENABLE"),
    .REFCLK_SRC         ("PLLCLK"),
    .SIM_DEVICE         (SIM_DEVICE)
) u_bsc_lower (
    .PLL_CLK            (i_pll_clkoutphy),
    .REFCLK             (1'b0),
    .RIU_CLK            (i_div_clk),
    .RST                (i_bsc_rst),
    .EN_VTC             (i_bsc_en_vtc),
    .DLY_RDY            (dly_rdy_low),
    .VTC_RDY            (vtc_rdy_low),
    // UG571 requires an unused inter-byte clock input to be pulled High.
    .CLK_FROM_EXT       (1'b1),
    .PCLK_NIBBLE_IN     (pclk_nibble_out_upp),
    .NCLK_NIBBLE_IN     (nclk_nibble_out_upp),
    .PCLK_NIBBLE_OUT    (),
    .NCLK_NIBBLE_OUT    (),
    .TBYTE_IN           (i_tbyte_dq),
    .PHY_RDEN           (i_phy_rden),
    // Single-rank interfaces use rank 0 implicitly. MIG drives both rank
    // selector buses Low for RANKS=1; leaving them floating corrupts the
    // native gate state in simulation and is illegal in hardware.
    .PHY_RDCS0          (4'b0000),
    .PHY_RDCS1          (4'b0000),
    .PHY_WRCS0          (4'b0000),
    .PHY_WRCS1          (4'b0000),
    // Position 0 (DM - TX only)
    .RX_BIT_CTRL_OUT0   (rx_bit_ctrl_out0_low),
    .TX_BIT_CTRL_OUT0   (tx_bit_ctrl_out0_low),
    .RX_BIT_CTRL_IN0    (rx_bit_ctrl_in0_low),
    .TX_BIT_CTRL_IN0    (tx_bit_ctrl_in0_low),
    // Position 1 (unused)
    .RX_BIT_CTRL_OUT1   (rx_bit_ctrl_out1_low),
    .TX_BIT_CTRL_OUT1   (tx_bit_ctrl_out1_low),
    .RX_BIT_CTRL_IN1    (rx_bit_ctrl_in1_low),
    .TX_BIT_CTRL_IN1    (tx_bit_ctrl_in1_low),
    // Position 2 (DQ[0])
    .RX_BIT_CTRL_OUT2   (rx_bit_ctrl_out2_low),
    .TX_BIT_CTRL_OUT2   (tx_bit_ctrl_out2_low),
    .RX_BIT_CTRL_IN2    (rx_bit_ctrl_in2_low),
    .TX_BIT_CTRL_IN2    (tx_bit_ctrl_in2_low),
    // Position 3 (DQ[1])
    .RX_BIT_CTRL_OUT3   (rx_bit_ctrl_out3_low),
    .TX_BIT_CTRL_OUT3   (tx_bit_ctrl_out3_low),
    .RX_BIT_CTRL_IN3    (rx_bit_ctrl_in3_low),
    .TX_BIT_CTRL_IN3    (tx_bit_ctrl_in3_low),
    // Position 4 (DQ[2])
    .RX_BIT_CTRL_OUT4   (rx_bit_ctrl_out4_low),
    .TX_BIT_CTRL_OUT4   (tx_bit_ctrl_out4_low),
    .RX_BIT_CTRL_IN4    (rx_bit_ctrl_in4_low),
    .TX_BIT_CTRL_IN4    (tx_bit_ctrl_in4_low),
    // Position 5 (DQ[3])
    .RX_BIT_CTRL_OUT5   (rx_bit_ctrl_out5_low),
    .TX_BIT_CTRL_OUT5   (tx_bit_ctrl_out5_low),
    .RX_BIT_CTRL_IN5    (rx_bit_ctrl_in5_low),
    .TX_BIT_CTRL_IN5    (tx_bit_ctrl_in5_low),
    // Position 6 (unused)
    .RX_BIT_CTRL_OUT6   (rx_bit_ctrl_out6_low),
    .TX_BIT_CTRL_OUT6   (tx_bit_ctrl_out6_low),
    .RX_BIT_CTRL_IN6    (rx_bit_ctrl_in6_low),
    .TX_BIT_CTRL_IN6    (tx_bit_ctrl_in6_low),
    // Tristate bus
    .TX_BIT_CTRL_OUT_TRI(tx_bit_ctrl_out_tri_low),
    .TX_BIT_CTRL_IN_TRI (tx_bit_ctrl_in_tri_low),
    .RIU_ADDR           (riu_addr_q),
    .RIU_WR_DATA        (riu_wr_data_q),
    .RIU_WR_EN          (riu_wr_en_q),
    // No RIU transaction targets this nibble.  Its source-clock selection is
    // established by EN_OTHER_PCLK/NCLK and must not be overwritten while
    // training the DQS-owning upper nibble.
    .RIU_NIBBLE_SEL     (1'b0),
    .RIU_RD_DATA        (),
    .RIU_VALID          ()
);
// ---------------------------------------------------------------------------
// Upper Nibble BITSLICE_CONTROL
// ---------------------------------------------------------------------------
BITSLICE_CONTROL #(
    .CTRL_CLK          ("EXTERNAL"),
    .DIV_MODE           ("DIV4"),
    .SERIAL_MODE        ("FALSE"),
    .EN_DYN_ODLY_MODE   ("TRUE"),
    .IDLY_VT_TRACK      ("TRUE"),
    .ODLY_VT_TRACK      ("TRUE"),
    .QDLY_VT_TRACK      ("TRUE"),
    .ROUNDING_FACTOR    (16),
    .RXGATE_EXTEND      ("FALSE"),
    .RX_CLK_PHASE_P     ("SHIFT_90"),
    .RX_CLK_PHASE_N     ("SHIFT_90"),
    .EN_OTHER_PCLK      ("FALSE"),
    .EN_OTHER_NCLK      ("FALSE"),
    .SELF_CALIBRATE     ("ENABLE"),
`ifdef SIM_NATIVE_RX_GATE_DISABLE
    .RX_GATING          ("DISABLE"),
`else
    .RX_GATING          ("ENABLE"),
`endif
    .READ_IDLE_COUNT    (31),
    .TX_GATING          ("ENABLE"),
    .REFCLK_SRC         ("PLLCLK"),
    .SIM_DEVICE         (SIM_DEVICE)
) u_bsc_upper (
    .PLL_CLK            (i_pll_clkoutphy),
    .REFCLK             (1'b0),
    .RIU_CLK            (i_div_clk),
    .RST                (i_bsc_rst),
    .EN_VTC             (i_bsc_en_vtc),
    .DLY_RDY            (dly_rdy_upp),
    .VTC_RDY            (vtc_rdy_upp),
    // No inter-byte clock is used by this generic byte interface.
    .CLK_FROM_EXT       (1'b1),
    .PCLK_NIBBLE_IN     (1'b0),
    .NCLK_NIBBLE_IN     (1'b0),
    .PCLK_NIBBLE_OUT    (pclk_nibble_out_upp),
    .NCLK_NIBBLE_OUT    (nclk_nibble_out_upp),
    .TBYTE_IN           (i_tbyte_dqs),
    .PHY_RDEN           (i_phy_rden),
    .PHY_RDCS0          (4'b0000),
    .PHY_RDCS1          (4'b0000),
    .PHY_WRCS0          (4'b0000),
    .PHY_WRCS1          (4'b0000),
    // Position 0 (DQS)
    .RX_BIT_CTRL_OUT0   (rx_bit_ctrl_out0_upp),
    .TX_BIT_CTRL_OUT0   (tx_bit_ctrl_out0_upp),
    .RX_BIT_CTRL_IN0    (rx_bit_ctrl_in0_upp),
    .TX_BIT_CTRL_IN0    (tx_bit_ctrl_in0_upp),
    // Position 1 (DQS_N - differential, unused bus)
    .RX_BIT_CTRL_OUT1   (rx_bit_ctrl_out1_upp),
    .TX_BIT_CTRL_OUT1   (tx_bit_ctrl_out1_upp),
    .RX_BIT_CTRL_IN1    (rx_bit_ctrl_in1_upp),
    .TX_BIT_CTRL_IN1    (tx_bit_ctrl_in1_upp),
    // Position 2 (DQ[4])
    .RX_BIT_CTRL_OUT2   (rx_bit_ctrl_out2_upp),
    .TX_BIT_CTRL_OUT2   (tx_bit_ctrl_out2_upp),
    .RX_BIT_CTRL_IN2    (rx_bit_ctrl_in2_upp),
    .TX_BIT_CTRL_IN2    (tx_bit_ctrl_in2_upp),
    // Position 3 (DQ[5])
    .RX_BIT_CTRL_OUT3   (rx_bit_ctrl_out3_upp),
    .TX_BIT_CTRL_OUT3   (tx_bit_ctrl_out3_upp),
    .RX_BIT_CTRL_IN3    (rx_bit_ctrl_in3_upp),
    .TX_BIT_CTRL_IN3    (tx_bit_ctrl_in3_upp),
    // Position 4 (DQ[6])
    .RX_BIT_CTRL_OUT4   (rx_bit_ctrl_out4_upp),
    .TX_BIT_CTRL_OUT4   (tx_bit_ctrl_out4_upp),
    .RX_BIT_CTRL_IN4    (rx_bit_ctrl_in4_upp),
    .TX_BIT_CTRL_IN4    (tx_bit_ctrl_in4_upp),
    // Position 5 (DQ[7])
    .RX_BIT_CTRL_OUT5   (rx_bit_ctrl_out5_upp),
    .TX_BIT_CTRL_OUT5   (tx_bit_ctrl_out5_upp),
    .RX_BIT_CTRL_IN5    (rx_bit_ctrl_in5_upp),
    .TX_BIT_CTRL_IN5    (tx_bit_ctrl_in5_upp),
    // Position 6 (unused)
    .RX_BIT_CTRL_OUT6   (rx_bit_ctrl_out6_upp),
    .TX_BIT_CTRL_OUT6   (tx_bit_ctrl_out6_upp),
    .RX_BIT_CTRL_IN6    (rx_bit_ctrl_in6_upp),
    .TX_BIT_CTRL_IN6    (tx_bit_ctrl_in6_upp),
    // Tristate bus
    .TX_BIT_CTRL_OUT_TRI(tx_bit_ctrl_out_tri_upp),
    .TX_BIT_CTRL_IN_TRI (tx_bit_ctrl_in_tri_upp),
    .RIU_ADDR           (riu_addr_q),
    .RIU_WR_DATA        (riu_wr_data_q),
    .RIU_WR_EN          (riu_wr_en_q),
    .RIU_NIBBLE_SEL     (riu_nibble_sel_q),
    .RIU_RD_DATA        (riu_rd_data_upp),
    .RIU_VALID          (riu_rd_valid_upp)
);
// ---------------------------------------------------------------------------
// TX_BITSLICE_TRI - Lower nibble (DQ tristate)
// ---------------------------------------------------------------------------
TX_BITSLICE_TRI #(
    .DATA_WIDTH         (8),
    .OUTPUT_PHASE_90    ("TRUE"),
    .INIT               (1'b1),
    .SIM_DEVICE         (SIM_DEVICE)
) u_tri_lower (
    // TBYTE_IN is serialised by BITSLICE_CONTROL.  As in the generated
    // UltraScale MIG native PHY, this resource is controlled through the
    // BIT_CTRL bus rather than a fabric-clocked TX datapath.
    .CLK                (1'b1),
    .RST                (i_bitslice_rst),
    .RST_DLY            (i_bitslice_rst),
    .CE                 (1'b0),
    .INC                (1'b0),
    .LOAD               (1'b0),
    .CNTVALUEIN         (9'd0),
    .CNTVALUEOUT        (),
    .EN_VTC             (1'b1),
    .TRI_OUT            (tbyte_out_low),
    .BIT_CTRL_IN        (tx_bit_ctrl_out_tri_low),
    .BIT_CTRL_OUT       (tx_bit_ctrl_in_tri_low)
);
// ---------------------------------------------------------------------------
// TX_BITSLICE_TRI - Upper nibble (DQS tristate)
// ---------------------------------------------------------------------------
TX_BITSLICE_TRI #(
    .DATA_WIDTH         (8),
    .OUTPUT_PHASE_90    ("TRUE"),
    .INIT               (1'b1),
    .SIM_DEVICE         (SIM_DEVICE)
) u_tri_upper (
    .CLK                (1'b1),
    .RST                (i_bitslice_rst),
    .RST_DLY            (i_bitslice_rst),
    .CE                 (1'b0),
    .INC                (1'b0),
    .LOAD               (1'b0),
    .CNTVALUEIN         (9'd0),
    .CNTVALUEOUT        (),
    .EN_VTC             (1'b1),
    .TRI_OUT            (tbyte_out_upp),
    .BIT_CTRL_IN        (tx_bit_ctrl_out_tri_upp),
    .BIT_CTRL_OUT       (tx_bit_ctrl_in_tri_upp)
);
// ---------------------------------------------------------------------------
// RXTX_BITSLICE - DM (lower nibble position 0)
// ---------------------------------------------------------------------------
RXTX_BITSLICE #(
    .FIFO_SYNC_MODE     ("FALSE"),
    .NATIVE_ODELAY_BYPASS("FALSE"),
    .RX_UPDATE_MODE     ("ASYNC"),
    .TX_UPDATE_MODE     ("ASYNC"),
    .RX_DATA_TYPE       ("DATA"),
    .RX_DATA_WIDTH      (8),
    .TX_DATA_WIDTH      (8),
    .RX_DELAY_FORMAT    ("COUNT"),
    .TX_DELAY_FORMAT    ("COUNT"),
    .RX_DELAY_TYPE      ("VAR_LOAD"),
    .TX_DELAY_TYPE      ("VAR_LOAD"),
    .RX_DELAY_VALUE     (0),
    .TX_DELAY_VALUE     (0),
    .TX_OUTPUT_PHASE_90 ("FALSE"),
    .RX_REFCLK_FREQUENCY(REFCLK_FREQ),
    .TX_REFCLK_FREQUENCY(REFCLK_FREQ),
    .TBYTE_CTL          ("TBYTE_IN"),
    .INIT               (1'b1),
    .SIM_DEVICE         (SIM_DEVICE)
) u_rxtx_dm (
    .FIFO_RD_CLK        (i_div_clk),
    .FIFO_RD_EN         (1'b0),
    .FIFO_EMPTY         (),
    .RX_RST             (i_rx_fifo_rst),
    .TX_RST             (i_bitslice_rst),
    .RX_RST_DLY         (i_bitslice_rst),
    .TX_RST_DLY         (i_bitslice_rst),
    // Native memory mode obtains the receive clock through BITSLICE_CONTROL.
    // Even this TX-only DM position must not inject CLKDIV into that network;
    // the generated DDR4 PHY ties RX_CLK High on every RXTX_BITSLICE.
    .RX_CLK             (1'b1),
    // TX is clocked through the BITSLICE_CONTROL network.  TX_CLK must be
    // tied High (not CLKDIV): this is the UltraScale native-PHY topology and
    // prevents the serial TX word from being delayed by fabric clocking.
    .TX_CLK             (1'b1),
    .RX_EN_VTC          (i_bitslice_en_vtc),
    .TX_EN_VTC          (i_bitslice_en_vtc),
    .RX_CE              (1'b0),
    .RX_INC             (1'b0),
    .TX_CE              (1'b0),
    .TX_INC             (1'b0),
    .RX_CNTVALUEIN      (9'd0),
    .RX_LOAD            (1'b0),
    .RX_CNTVALUEOUT     (),
    .TX_CNTVALUEIN      (i_tx_dq_cntvaluein),
    .TX_LOAD            (i_tx_dq_load),
    .TX_CNTVALUEOUT     (),
    .D                  (i_tx_dm_data),
    .O                  (dm_to_obuf),
    .T                  (1'b0),
    .TBYTE_IN           (tbyte_out_low),
    .DATAIN             (1'b0),
    .Q                  (),
    .RX_BIT_CTRL_IN     (rx_bit_ctrl_out0_low),
    .RX_BIT_CTRL_OUT    (rx_bit_ctrl_in0_low),
    .TX_BIT_CTRL_IN     (tx_bit_ctrl_out0_low),
    .TX_BIT_CTRL_OUT    (tx_bit_ctrl_in0_low),
    .T_OUT              ()
);
// ---------------------------------------------------------------------------
// RXTX_BITSLICE - DQS (upper nibble position 0)
// ---------------------------------------------------------------------------
RXTX_BITSLICE #(
    .FIFO_SYNC_MODE     ("FALSE"),
    .NATIVE_ODELAY_BYPASS("FALSE"),
    .RX_UPDATE_MODE     ("ASYNC"),
    .TX_UPDATE_MODE     ("ASYNC"),
    .RX_DATA_TYPE       ("DATA_AND_CLOCK"),
    .RX_DATA_WIDTH      (8),
    .TX_DATA_WIDTH      (8),
    .RX_DELAY_FORMAT    ("COUNT"),
    .TX_DELAY_FORMAT    ("COUNT"),
    // DQS is the native source-synchronous receive clock.  Its RX delay is
    // not part of the DQ eye sweep: keep it at the calibrated zero-delay
    // reference exactly as the UltraScale MIG DQS BITSLICE does.  Moving it
    // by an arbitrary startup count changes the receiver gate phase and can
    // truncate a BL8 burst's final beat.
    .RX_DELAY_TYPE      ("FIXED"),
    .TX_DELAY_TYPE      ("VAR_LOAD"),
    .RX_DELAY_VALUE     (0),
    .TX_DELAY_VALUE     (0),
    .TX_OUTPUT_PHASE_90 ("TRUE"),
    .RX_REFCLK_FREQUENCY(REFCLK_FREQ),
    .TX_REFCLK_FREQUENCY(REFCLK_FREQ),
    .TBYTE_CTL          ("TBYTE_IN"),
    .INIT               (1'b1),
    .SIM_DEVICE         (SIM_DEVICE)
) u_rxtx_dqs (
    .FIFO_RD_CLK        (i_div_clk),
    // MIG advances the DATA_AND_CLOCK slice with the same common FIFO read
    // pulse as the byte's DQ slices. Although DQS.Q is unused, this keeps the
    // native gate monitor and byte FIFO word boundary in lockstep.
    .FIFO_RD_EN         (i_fifo_rd_en[0]),
    .FIFO_EMPTY         (),
    .RX_RST             (i_rx_fifo_rst),
    .TX_RST             (i_bitslice_rst),
    .RX_RST_DLY         (i_bitslice_rst),
    .TX_RST_DLY         (i_bitslice_rst),
    .RX_CLK             (1'b1),
    .TX_CLK             (1'b1),
    .RX_EN_VTC          (i_bitslice_en_vtc),
    .TX_EN_VTC          (i_bitslice_en_vtc),
    .RX_CE              (1'b0),
    .RX_INC             (1'b0),
    .TX_CE              (1'b0),
    .TX_INC             (1'b0),
    .RX_CNTVALUEIN      (9'd0),
    .RX_LOAD            (1'b0),
    .RX_CNTVALUEOUT     (),
    .TX_CNTVALUEIN      (i_tx_dqs_cntvaluein),
    .TX_LOAD            (i_tx_dqs_load),
    .TX_CNTVALUEOUT     (o_tx_dqs_cntvalueout),
    .D                  (i_tx_dqs_data),
    .O                  (dqs_to_iob),
    // DATA_AND_CLOCK is the byte's master clock slice. MIG fixes T Low here;
    // byte ownership still comes from the serialized TBYTE_IN path.
    .T                  (1'b0),
    .TBYTE_IN           (tbyte_out_upp),
    .DATAIN             (dqs_from_iob),
    .Q                  (),
    .RX_BIT_CTRL_IN     (rx_bit_ctrl_out0_upp),
    .RX_BIT_CTRL_OUT    (rx_bit_ctrl_in0_upp),
    .TX_BIT_CTRL_IN     (tx_bit_ctrl_out0_upp),
    .TX_BIT_CTRL_OUT    (tx_bit_ctrl_in0_upp),
    .T_OUT              (dqs_t)
);
// ---------------------------------------------------------------------------
// RXTX_BITSLICE - physical lower-nibble positions 2-5
// LOGICAL_DQ inverts DQ_PIN_MAP, preserving the external/DFI bit number.
// ---------------------------------------------------------------------------
wire [8:0] rx_cntvalueout_dq [0:DQ_BITS-1];
assign o_rx_cntvalueout_dq0 = rx_cntvalueout_dq[0];
genvar gi;
generate
for (gi = 0; gi < 4; gi = gi + 1) begin : gen_dq_lower
    localparam integer LOGICAL_DQ = dq_pin_at(0, gi + 2);
    wire [39:0] rx_ctrl_out_w;
    wire [39:0] tx_ctrl_out_w;
    wire [39:0] rx_ctrl_in_w;
    wire [39:0] tx_ctrl_in_w;
    // Mux BIT_CTRL connections per position (2,3,4,5)
    assign rx_ctrl_in_w = (gi == 0) ? rx_bit_ctrl_out2_low :
                          (gi == 1) ? rx_bit_ctrl_out3_low :
                          (gi == 2) ? rx_bit_ctrl_out4_low :
                                      rx_bit_ctrl_out5_low;
    assign tx_ctrl_in_w = (gi == 0) ? tx_bit_ctrl_out2_low :
                          (gi == 1) ? tx_bit_ctrl_out3_low :
                          (gi == 2) ? tx_bit_ctrl_out4_low :
                                      tx_bit_ctrl_out5_low;
    RXTX_BITSLICE #(
        .FIFO_SYNC_MODE     ("FALSE"),
        .NATIVE_ODELAY_BYPASS("FALSE"),
        .RX_UPDATE_MODE     ("ASYNC"),
        .TX_UPDATE_MODE     ("ASYNC"),
        .RX_DATA_TYPE       ("DATA"),
        .RX_DATA_WIDTH      (8),
        .TX_DATA_WIDTH      (8),
        .RX_DELAY_FORMAT    ("COUNT"),
        .TX_DELAY_FORMAT    ("COUNT"),
        .RX_DELAY_TYPE      ("VAR_LOAD"),
        .TX_DELAY_TYPE      ("VAR_LOAD"),
        .RX_DELAY_VALUE     (0),
        .TX_DELAY_VALUE     (0),
        .TX_OUTPUT_PHASE_90 ("FALSE"),
        .RX_REFCLK_FREQUENCY(REFCLK_FREQ),
        .TX_REFCLK_FREQUENCY(REFCLK_FREQ),
        .TBYTE_CTL          ("TBYTE_IN"),
        .INIT               (1'b1),
        .SIM_DEVICE         (SIM_DEVICE)
    ) u_rxtx_dq (
        .FIFO_RD_CLK        (i_div_clk),
        .FIFO_RD_EN         (i_fifo_rd_en[LOGICAL_DQ]),
        .FIFO_EMPTY         (o_fifo_empty[LOGICAL_DQ]),
        .RX_RST             (i_rx_fifo_rst),
        .TX_RST             (i_bitslice_rst),
        .RX_RST_DLY         (i_bitslice_rst),
        .TX_RST_DLY         (i_bitslice_rst),
        // DATA slices are clocked by the byte's DQS through the native
        // BITSLICE_CONTROL network.  RX_CLK must therefore be tied High;
        // i_div_clk is only the mesochronous FIFO read clock.  This matches
        // the UltraScale DDR4 MIG topology and prevents the first DQS sample
        // of a BL8 read from being lost.
        .RX_CLK             (1'b1),
        .TX_CLK             (1'b1),
        .RX_EN_VTC          (i_bitslice_en_vtc),
        .TX_EN_VTC          (i_bitslice_en_vtc),
        .RX_CE              (1'b0),
        .RX_INC             (1'b0),
        .TX_CE              (1'b0),
        .TX_INC             (1'b0),
        .RX_CNTVALUEIN      (i_rx_cntvaluein),
        .RX_LOAD            (i_rx_load),
        .RX_CNTVALUEOUT     (rx_cntvalueout_dq[LOGICAL_DQ]),
        .TX_CNTVALUEIN      (i_tx_dq_cntvaluein),
        .TX_LOAD            (i_tx_dq_load),
        .TX_CNTVALUEOUT     (),
        .D                  (i_tx_dq_data[LOGICAL_DQ*8 +: 8]),
        .O                  (dq_to_iob[LOGICAL_DQ]),
        .T                  (1'b1),
        .TBYTE_IN           (tbyte_out_low),
        .DATAIN             (dq_from_iob[LOGICAL_DQ]),
        .Q                  (o_rx_dq_data[LOGICAL_DQ*8 +: 8]),
        .RX_BIT_CTRL_IN     (rx_ctrl_in_w),
        .RX_BIT_CTRL_OUT    (rx_ctrl_out_w),
        .TX_BIT_CTRL_IN     (tx_ctrl_in_w),
        .TX_BIT_CTRL_OUT    (tx_ctrl_out_w),
        .T_OUT              (dq_t[LOGICAL_DQ])
    );
    if (gi == 0) begin : assign_pos2
        assign rx_bit_ctrl_in2_low = rx_ctrl_out_w;
        assign tx_bit_ctrl_in2_low = tx_ctrl_out_w;
    end else if (gi == 1) begin : assign_pos3
        assign rx_bit_ctrl_in3_low = rx_ctrl_out_w;
        assign tx_bit_ctrl_in3_low = tx_ctrl_out_w;
    end else if (gi == 2) begin : assign_pos4
        assign rx_bit_ctrl_in4_low = rx_ctrl_out_w;
        assign tx_bit_ctrl_in4_low = tx_ctrl_out_w;
    end else begin : assign_pos5
        assign rx_bit_ctrl_in5_low = rx_ctrl_out_w;
        assign tx_bit_ctrl_in5_low = tx_ctrl_out_w;
    end
end
endgenerate
// ---------------------------------------------------------------------------
// RXTX_BITSLICE - physical upper-nibble positions 2-5
// ---------------------------------------------------------------------------
generate
for (gi = 0; gi < 4; gi = gi + 1) begin : gen_dq_upper
    localparam integer LOGICAL_DQ = dq_pin_at(1, gi + 2);
    wire [39:0] rx_ctrl_out_w;
    wire [39:0] tx_ctrl_out_w;
    wire [39:0] rx_ctrl_in_w;
    wire [39:0] tx_ctrl_in_w;
    assign rx_ctrl_in_w = (gi == 0) ? rx_bit_ctrl_out2_upp :
                          (gi == 1) ? rx_bit_ctrl_out3_upp :
                          (gi == 2) ? rx_bit_ctrl_out4_upp :
                                      rx_bit_ctrl_out5_upp;
    assign tx_ctrl_in_w = (gi == 0) ? tx_bit_ctrl_out2_upp :
                          (gi == 1) ? tx_bit_ctrl_out3_upp :
                          (gi == 2) ? tx_bit_ctrl_out4_upp :
                                      tx_bit_ctrl_out5_upp;
    RXTX_BITSLICE #(
        .FIFO_SYNC_MODE     ("FALSE"),
        .NATIVE_ODELAY_BYPASS("FALSE"),
        .RX_UPDATE_MODE     ("ASYNC"),
        .TX_UPDATE_MODE     ("ASYNC"),
        .RX_DATA_TYPE       ("DATA"),
        .RX_DATA_WIDTH      (8),
        .TX_DATA_WIDTH      (8),
        .RX_DELAY_FORMAT    ("COUNT"),
        .TX_DELAY_FORMAT    ("COUNT"),
        .RX_DELAY_TYPE      ("VAR_LOAD"),
        .TX_DELAY_TYPE      ("VAR_LOAD"),
        .RX_DELAY_VALUE     (0),
        .TX_DELAY_VALUE     (0),
        .TX_OUTPUT_PHASE_90 ("FALSE"),
        .RX_REFCLK_FREQUENCY(REFCLK_FREQ),
        .TX_REFCLK_FREQUENCY(REFCLK_FREQ),
        .TBYTE_CTL          ("TBYTE_IN"),
        .INIT               (1'b1),
        .SIM_DEVICE         (SIM_DEVICE)
    ) u_rxtx_dq (
        .FIFO_RD_CLK        (i_div_clk),
        .FIFO_RD_EN         (i_fifo_rd_en[LOGICAL_DQ]),
        .FIFO_EMPTY         (o_fifo_empty[LOGICAL_DQ]),
        .RX_RST             (i_rx_fifo_rst),
        .TX_RST             (i_bitslice_rst),
        .RX_RST_DLY         (i_bitslice_rst),
        .TX_RST_DLY         (i_bitslice_rst),
        // See the lower-nibble DATA slice: DQS, not CLKDIV, clocks native RX.
        .RX_CLK             (1'b1),
        .TX_CLK             (1'b1),
        .RX_EN_VTC          (i_bitslice_en_vtc),
        .TX_EN_VTC          (i_bitslice_en_vtc),
        .RX_CE              (1'b0),
        .RX_INC             (1'b0),
        .TX_CE              (1'b0),
        .TX_INC             (1'b0),
        .RX_CNTVALUEIN      (i_rx_cntvaluein),
        .RX_LOAD            (i_rx_load),
        .RX_CNTVALUEOUT     (rx_cntvalueout_dq[LOGICAL_DQ]),
        .TX_CNTVALUEIN      (i_tx_dq_cntvaluein),
        .TX_LOAD            (i_tx_dq_load),
        .TX_CNTVALUEOUT     (),
        .D                  (i_tx_dq_data[LOGICAL_DQ*8 +: 8]),
        .O                  (dq_to_iob[LOGICAL_DQ]),
        .T                  (1'b1),
        .TBYTE_IN           (tbyte_out_upp),
        .DATAIN             (dq_from_iob[LOGICAL_DQ]),
        .Q                  (o_rx_dq_data[LOGICAL_DQ*8 +: 8]),
        .RX_BIT_CTRL_IN     (rx_ctrl_in_w),
        .RX_BIT_CTRL_OUT    (rx_ctrl_out_w),
        .TX_BIT_CTRL_IN     (tx_ctrl_in_w),
        .TX_BIT_CTRL_OUT    (tx_ctrl_out_w),
        .T_OUT              (dq_t[LOGICAL_DQ])
    );
    if (gi == 0) begin : assign_pos2
        assign rx_bit_ctrl_in2_upp = rx_ctrl_out_w;
        assign tx_bit_ctrl_in2_upp = tx_ctrl_out_w;
    end else if (gi == 1) begin : assign_pos3
        assign rx_bit_ctrl_in3_upp = rx_ctrl_out_w;
        assign tx_bit_ctrl_in3_upp = tx_ctrl_out_w;
    end else if (gi == 2) begin : assign_pos4
        assign rx_bit_ctrl_in4_upp = rx_ctrl_out_w;
        assign tx_bit_ctrl_in4_upp = tx_ctrl_out_w;
    end else begin : assign_pos5
        assign rx_bit_ctrl_in5_upp = rx_ctrl_out_w;
        assign tx_bit_ctrl_in5_upp = tx_ctrl_out_w;
    end
end
endgenerate
// ---------------------------------------------------------------------------
// IOBs - DQ[7:0]
// ---------------------------------------------------------------------------
generate
for (gi = 0; gi < DQ_BITS; gi = gi + 1) begin : gen_iobuf_dq
    IOBUF u_iobuf_dq (
        .O  (dq_from_iob[gi]),
        .IO (io_ddr4_dq[gi]),
        .I  (dq_to_iob[gi]),
        .T  (dq_t[gi])
    );
end
endgenerate
// ---------------------------------------------------------------------------
// IOB - DQS differential pair
// ---------------------------------------------------------------------------
IOBUFDS #(
    // Match the generated DDR4 MIG I/O byte.  The memory's differential
    // termination establishes the idle DQS level during read ownership.
    .DQS_BIAS ("FALSE")
) u_iobufds_dqs (
    .O   (dqs_from_iob),
    .IO  (io_ddr4_dqs_p),
    .IOB (io_ddr4_dqs_n),
    .I   (dqs_to_iob),
    .T   (dqs_t)
);
// ---------------------------------------------------------------------------
// IOB - DM output
// ---------------------------------------------------------------------------
OBUF u_obuf_dm (
    .O (o_ddr4_dm_n),
    .I (dm_to_obuf)
);
/* verilator lint_on PINMISSING */
/* verilator lint_on PINCONNECTEMPTY */
endmodule

`default_nettype wire
