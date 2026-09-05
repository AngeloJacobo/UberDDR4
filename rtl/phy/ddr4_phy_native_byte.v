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
    // RIU has a lower maximum frequency than DIV_CLK on UltraScale(+).
    // Keep it on a dedicated clock (normally DIV_CLK/2, as in the Xilinx
    // DDR4 PHY) and cross the infrequent calibration transactions below.
    input  wire        i_riu_clk,
    // Reset
    input  wire        i_bsc_rst,
    input  wire        i_riu_rst,
    input  wire        i_bitslice_rst,
    // Clears the RX deserializer/FIFO pointers while RX_RST_DLY remains low,
    // preserving the trained input-delay value.
    input  wire        i_rx_fifo_rst,
    // BISC status
    output wire        o_dly_rdy,
    output wire        o_vtc_rdy,
    // BITSLICE_CONTROL and TIME-mode DQ RX EN_VTC controls.  DQS, DM, and DQ
    // TX use TIME/FIXED and maintain EN_VTC High locally; DQ RX lowers this
    // input only while eye training updates its TIME/VAR_LOAD delay.
    input  wire        i_bsc_en_vtc,
    input  wire        i_bitslice_en_vtc,
    // TX data (write path)
    input  wire [DQ_BITS*8-1:0] i_tx_dq_data,
    input  wire [7:0]           i_tx_dqs_data,
    input  wire [7:0]           i_tx_dm_data,
    // TX tristate control
    input  wire [3:0]  i_tbyte_dq,
    input  wire [3:0]  i_tbyte_dqs,
    // Suppress local TX loopback only for normal application writes.  DQ and
    // DQS have separate controls because write leveling must receive DRAM's
    // DQ feedback while the DQS receiver is guarded across pad ownership
    // changes.  Once DQS is driven stably Low, its receiver is enabled for
    // the actual training pulses only.
    input  wire        i_rx_dq_input_disable,
    input  wire        i_rx_dqs_input_disable,
    // UltraScale XiPHY has one BITSLICE_CONTROL per nibble.  Their read-gate
    // enables are trained independently even when the lower nibble imports
    // the DQS clocks from the upper nibble.
    input  wire [3:0]  i_phy_rden_lower,
    input  wire [3:0]  i_phy_rden_upper,
    // RX data (read path)
    output wire [DQ_BITS*8-1:0] o_rx_dq_data,
    output wire [DQ_BITS-1:0]   o_fifo_empty,
    // Fabric-side observation of the DATA_AND_CLOCK slice.  These signals
    // preserve the dedicated DQS IOB-to-BITSLICE route while allowing ILA to
    // distinguish an empty DQS FIFO from a populated byte whose DQ data does
    // not match the expected training pattern.
    output wire                 o_dqs_fifo_empty,
    output wire [7:0]           o_dqs_fifo_data,
    // {upper VTC, lower VTC, upper DLY, lower DLY}, registered on i_div_clk.
    output wire [3:0]           o_dbg_nibble_ready,
    // Each asynchronous DQ FIFO has its own registered read enable.
    input  wire [DQ_BITS-1:0]   i_fifo_rd_en,
    // Per-byte RIU access. Read-gate registers target only the upper nibble
    // that owns DQS. Write-level registers target both nibbles so all eight
    // DQ bits retain the same trained TX phase as DQS.
    input  wire [5:0]           i_riu_addr,
    input  wire [15:0]          i_riu_wr_data,
    input  wire                 i_riu_wr_en,
    input  wire                 i_riu_lower_sel,
    input  wire                 i_riu_upper_sel,
    output reg  [15:0]          o_riu_rd_data,
    output reg                  o_riu_valid,
    // GT_STATUS can assert for less than one DIV_CLK response round trip.
    // Capture it beside RIU_OR and clear it with the same CLR_GATE write that
    // starts each native gate candidate.  The synchronized level therefore
    // belongs to exactly one candidate and cannot be lost by the RIU CDC.
    output wire                 o_riu_gate_status_sticky,
    // RX delay control
    input  wire [8:0]             i_rx_cntvaluein,
    // Eye training first sweeps one common byte offset, then loads the
    // independently measured center of each DQ eye.  The mode input changes
    // only the requested relative offset; BISC Align_Delay remains applied
    // independently below for every physical bit slice.
    input  wire [DQ_BITS*9-1:0]   i_rx_cntvaluein_per_dq,
    input  wire                   i_rx_per_dq_mode,
    input  wire                   i_rx_load,
    output wire [8:0]  o_rx_cntvalueout_dq0,
    // Largest lane-wide relative offset that leaves every DQ at or below the
    // TIME-mode tap limit after adding its individual BISC Align_Delay.
    output reg  [8:0]  o_rx_max_relative_offset,
    // Assert only after every per-bit CNTVALUEOUT has remained unchanged for
    // a qualified interval with RX_EN_VTC Low.  The parent must not issue a
    // TIME/VAR_LOAD update before this handshake is High.
    output wire        o_rx_align_valid,
    // TX delay control for DQS
    input  wire [8:0]  i_tx_dqs_cntvaluein,
    input  wire        i_tx_dqs_load,
    output wire [8:0]  o_tx_dqs_cntvalueout,
    // Per-DQ TX delay control.  LOAD/CNTVALUEIN are retained for the native
    // primitive interface, while the post-failure diagnostic uses the
    // one-hot CE bus and shared INC direction for deterministic single-tap
    // changes with EN_VTC disabled, as specified by UG571.
    input  wire [8:0]            i_tx_dq_cntvaluein,
    input  wire [DQ_BITS-1:0]    i_tx_dq_load,
    input  wire [DQ_BITS-1:0]    i_tx_dq_ce,
    input  wire                  i_tx_dq_inc,
    input  wire                  i_tx_dq_en_vtc,
    output wire [DQ_BITS*9-1:0]  o_tx_dq_cntvalueout,
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
// controls (UG571, RIU_OR topology). The controller and RIU clocks are
// generated by the same MMCM with the same phase shift, as required for
// RL_DLY_RNK operation; their frequencies need not be equal. Write commands use a
// bundled-data toggle handshake: the source payload is held until the parent
// calibration FSM observes the synchronized RIU response. Read controls are
// level-synchronized because the parent holds them while awaiting RIU_VALID.
(* DONT_TOUCH = "TRUE", SHREG_EXTRACT = "NO" *)
reg [5:0]  riu_wr_addr_hold;
(* DONT_TOUCH = "TRUE", SHREG_EXTRACT = "NO" *)
reg [15:0] riu_wr_data_hold;
(* DONT_TOUCH = "TRUE", SHREG_EXTRACT = "NO" *)
reg        riu_wr_lower_hold, riu_wr_upper_hold;
reg        riu_wr_toggle;

always @(posedge i_div_clk) begin
    if (i_bsc_rst) begin
        riu_wr_addr_hold  <= 6'd0;
        riu_wr_data_hold  <= 16'd0;
        riu_wr_lower_hold <= 1'b0;
        riu_wr_upper_hold <= 1'b0;
        riu_wr_toggle     <= 1'b0;
    end else if (i_riu_wr_en) begin
        riu_wr_addr_hold  <= i_riu_addr;
        riu_wr_data_hold  <= i_riu_wr_data;
        riu_wr_lower_hold <= i_riu_lower_sel;
        riu_wr_upper_hold <= i_riu_upper_sel;
        riu_wr_toggle     <= ~riu_wr_toggle;
    end
end

(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg [1:0] riu_wr_toggle_sync;
reg       riu_wr_toggle_seen;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg [5:0] riu_wr_addr_meta, riu_wr_addr_sync;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg [15:0] riu_wr_data_meta, riu_wr_data_sync;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg riu_wr_lower_meta, riu_wr_lower_sync;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg riu_wr_upper_meta, riu_wr_upper_sync;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg [5:0] riu_read_addr_meta, riu_read_addr_sync;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg       riu_read_lower_meta, riu_read_lower_sync;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg       riu_read_upper_meta, riu_read_upper_sync;
(* DONT_TOUCH = "TRUE", SHREG_EXTRACT = "NO" *)
reg [5:0]  riu_addr_q;
(* DONT_TOUCH = "TRUE", SHREG_EXTRACT = "NO" *)
reg [15:0] riu_wr_data_q;
(* DONT_TOUCH = "TRUE", SHREG_EXTRACT = "NO" *)
reg        riu_wr_en_q, riu_lower_sel_q, riu_upper_sel_q;

always @(posedge i_riu_clk) begin
    if (i_riu_rst) begin
        riu_wr_toggle_sync <= 2'b00;
        riu_wr_toggle_seen <= 1'b0;
        riu_wr_addr_meta <= 6'd0;
        riu_wr_addr_sync <= 6'd0;
        riu_wr_data_meta <= 16'd0;
        riu_wr_data_sync <= 16'd0;
        riu_wr_lower_meta <= 1'b0;
        riu_wr_lower_sync <= 1'b0;
        riu_wr_upper_meta <= 1'b0;
        riu_wr_upper_sync <= 1'b0;
        riu_read_addr_meta <= 6'd0;
        riu_read_addr_sync <= 6'd0;
        riu_read_lower_meta <= 1'b0;
        riu_read_lower_sync <= 1'b0;
        riu_read_upper_meta <= 1'b0;
        riu_read_upper_sync <= 1'b0;
        riu_addr_q       <= 6'd0;
        riu_wr_data_q    <= 16'd0;
        riu_wr_en_q      <= 1'b0;
        riu_lower_sel_q  <= 1'b0;
        riu_upper_sel_q  <= 1'b0;
    end else begin
        riu_wr_toggle_sync <= {riu_wr_toggle_sync[0], riu_wr_toggle};
        // The source holds this payload from request until the returned RIU
        // response. Synchronize every bit before consuming it when the
        // independently synchronized request toggle arrives. The required
        // common-MMCM phase relationship makes the bundled-data sampling
        // deterministic.
        riu_wr_addr_meta <= riu_wr_addr_hold;
        riu_wr_addr_sync <= riu_wr_addr_meta;
        riu_wr_data_meta <= riu_wr_data_hold;
        riu_wr_data_sync <= riu_wr_data_meta;
        riu_wr_lower_meta <= riu_wr_lower_hold;
        riu_wr_lower_sync <= riu_wr_lower_meta;
        riu_wr_upper_meta <= riu_wr_upper_hold;
        riu_wr_upper_sync <= riu_wr_upper_meta;
        riu_read_addr_meta <= i_riu_addr;
        riu_read_addr_sync <= riu_read_addr_meta;
        riu_read_lower_meta <= i_riu_lower_sel;
        riu_read_lower_sync <= riu_read_lower_meta;
        riu_read_upper_meta <= i_riu_upper_sel;
        riu_read_upper_sync <= riu_read_upper_meta;

        riu_wr_en_q <= 1'b0;
        if (riu_wr_toggle_sync[1] != riu_wr_toggle_seen) begin
            riu_wr_toggle_seen <= riu_wr_toggle_sync[1];
            riu_addr_q         <= riu_wr_addr_sync;
            riu_wr_data_q      <= riu_wr_data_sync;
            riu_lower_sel_q    <= riu_wr_lower_sync;
            riu_upper_sel_q    <= riu_wr_upper_sync;
            riu_wr_en_q        <= riu_wr_lower_sync | riu_wr_upper_sync;
        end else begin
            riu_addr_q         <= riu_read_addr_sync;
            riu_lower_sel_q    <= riu_read_lower_sync;
            riu_upper_sel_q    <= riu_read_upper_sync;
        end
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
// Dedicated inter-nibble strobe-clock links.  Both directions must be wired
// whenever both BITSLICE_CONTROLs in a byte are instantiated (UG571,
// "Inter-Nibble Clocking").  EN_OTHER_PCLK/NCLK below selects the upper-DQS
// to lower-data direction, but the reverse links must still be present so the
// physical byte has the complete native XiPHY clock topology.  This is also
// the topology generated by the DDR4 MIG.
wire pclk_nibble_out_low, nclk_nibble_out_low;
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
wire [8:0] rx_cntvalueout_dq [0:DQ_BITS-1];
// DQS IOB wires
wire dqs_to_iob, dqs_from_iob, dqs_t;
// DM output wire
wire dm_to_obuf;

// ---------------------------------------------------------------------------
// Per-bit BISC input-alignment preservation
// ---------------------------------------------------------------------------
// In TIME mode BISC inserts a separate Align_Delay into every DQ input path.
// Eye training is lane-wide, but it must move each bit relative to its own
// BISC baseline.  Loading the raw lane tap directly would overwrite that
// baseline (and a value below Align_Delay is explicitly forbidden by UG571).
//
// The parent supplies a relative eye offset.  At the start of every update
// session, after EN_VTC falls, capture the current BISC-maintained total and
// subtract the offset applied by the preceding session.  This also makes a
// later retraining pass safe after voltage/temperature tracking has adjusted
// the physical tap counts.
(* mark_debug = "true" *) reg [DQ_BITS*9-1:0] rx_align_delay_q;
// Remember the relative offset currently installed in every physical DQ.
// A later VTC/BISC baseline refresh must subtract the matching per-bit value;
// using one byte-wide value would fold DQ skew back into Align_Delay.
(* mark_debug = "true" *) reg [DQ_BITS*9-1:0] rx_relative_offset_q;
(* mark_debug = "true" *) reg                 rx_align_valid_q;
reg rx_align_capture_pending_q;
reg [3:0] rx_align_settle_count_q;
reg [DQ_BITS*9-1:0] rx_align_candidate_q;
reg [2:0] rx_align_stable_count_q;
reg rx_align_candidate_valid_q;
integer rx_delay_idx;
wire [8:0] rx_max_align_01;
wire [8:0] rx_max_align_23;
wire [8:0] rx_max_align_45;
wire [8:0] rx_max_align_67;
wire [8:0] rx_max_align_03;
wire [8:0] rx_max_align_47;
wire [8:0] rx_max_align_delay;
wire [DQ_BITS*9-1:0] rx_cntvalueout_sample;

assign o_rx_align_valid = rx_align_valid_q;

// Xilinx's encrypted UNISIM core does not model the analog TIME-mode
// Align_Delay and can leave CNTVALUEOUT unknown.  Normalize only that model
// artifact in simulation; hardware always uses the primitive readback.
genvar gai;
generate
for (gai = 0; gai < DQ_BITS; gai = gai + 1) begin : gen_rx_align_sample
`ifndef SYNTHESIS
    assign rx_cntvalueout_sample[gai*9 +: 9] =
        ((^rx_cntvalueout_dq[gai]) === 1'bx) ?
        9'd0 : rx_cntvalueout_dq[gai];
`else
    assign rx_cntvalueout_sample[gai*9 +: 9] =
        rx_cntvalueout_dq[gai];
`endif
end
endgenerate

function [8:0] rx_total_from_offset;
    input [8:0] align_delay;
    input [8:0] relative_offset;
    reg [9:0] sum;
    begin
        sum = {1'b0, align_delay} + {1'b0, relative_offset};
        rx_total_from_offset = sum[9] ? 9'h1ff : sum[8:0];
    end
endfunction

function [8:0] rx_applied_from_offset;
    input [8:0] align_delay;
    input [8:0] relative_offset;
    begin
        rx_applied_from_offset =
            rx_total_from_offset(align_delay, relative_offset) - align_delay;
    end
endfunction

// A lane-wide eye offset must be representable by every DQ in the byte.  Use
// the largest per-bit Align_Delay to derive the common legal sweep range; this
// prevents a saturated endpoint from appearing to be an artificially wide
// valid eye in the parent calibration FSM.  Keep the eight-way maximum as a
// balanced tree and register its result.  The previous priority-loop inferred
// a long comparator cascade directly into the calibration FSM, unnecessarily
// limiting the native PHY controller clock.  Align_Delay changes only when a
// new EN_VTC-low calibration session begins, and the parent waits for the
// ensuing VTC/RIU settle interval before consuming this limit, so the single
// registered cycle does not change calibration behavior.
assign rx_max_align_01 =
    (rx_align_delay_q[0*9 +: 9] > rx_align_delay_q[1*9 +: 9]) ?
     rx_align_delay_q[0*9 +: 9] : rx_align_delay_q[1*9 +: 9];
assign rx_max_align_23 =
    (rx_align_delay_q[2*9 +: 9] > rx_align_delay_q[3*9 +: 9]) ?
     rx_align_delay_q[2*9 +: 9] : rx_align_delay_q[3*9 +: 9];
assign rx_max_align_45 =
    (rx_align_delay_q[4*9 +: 9] > rx_align_delay_q[5*9 +: 9]) ?
     rx_align_delay_q[4*9 +: 9] : rx_align_delay_q[5*9 +: 9];
assign rx_max_align_67 =
    (rx_align_delay_q[6*9 +: 9] > rx_align_delay_q[7*9 +: 9]) ?
     rx_align_delay_q[6*9 +: 9] : rx_align_delay_q[7*9 +: 9];
assign rx_max_align_03 =
    (rx_max_align_01 > rx_max_align_23) ?
     rx_max_align_01 : rx_max_align_23;
assign rx_max_align_47 =
    (rx_max_align_45 > rx_max_align_67) ?
     rx_max_align_45 : rx_max_align_67;
assign rx_max_align_delay =
    (rx_max_align_03 > rx_max_align_47) ?
     rx_max_align_03 : rx_max_align_47;

always @(posedge i_div_clk) begin
    if (i_bitslice_rst) begin
        rx_align_delay_q           <= {(DQ_BITS*9){1'b0}};
        rx_relative_offset_q       <= {(DQ_BITS*9){1'b0}};
        rx_align_valid_q           <= 1'b0;
        rx_align_capture_pending_q <= 1'b1;
        rx_align_settle_count_q    <= 4'd0;
        rx_align_candidate_q       <= {(DQ_BITS*9){1'b0}};
        rx_align_stable_count_q    <= 3'd0;
        rx_align_candidate_valid_q <= 1'b0;
        o_rx_max_relative_offset   <= 9'h1ff;
    end else begin
        // 9'h1ff - x is exactly the nine-bit one's complement of x.
        o_rx_max_relative_offset <= ~rx_max_align_delay;

        // A High interval lets BISC and VTC maintain the programmed TIME
        // delay. Arm one fresh baseline capture for the next Low interval.
        if (i_bitslice_en_vtc) begin
            rx_align_capture_pending_q <= 1'b1;
            rx_align_settle_count_q    <= 4'd0;
            rx_align_stable_count_q    <= 3'd0;
            rx_align_candidate_valid_q <= 1'b0;
        end

        // UG571's TIME/VAR_LOAD procedure requires EN_VTC Low for at least ten
        // CLK cycles before CNTVALUEOUT is sampled.  The value can still be
        // transitioning on the first Low cycle: sampling it there produced a
        // reset-dependent value near 400 taps even though DELAY_VALUE=0 BISC
        // Align_Delay is specified as 45..65 taps.  Count ten complete DIV_CLK
        // intervals, then require four identical full-byte samples.  The
        // stability test is deliberately value-agnostic: BISC tap values vary
        // with device/process/voltage/temperature, so no board-specific range
        // is used.  The parent observes o_rx_align_valid and cannot issue LOAD
        // while this convergence test is still in progress.
        if (!i_bitslice_en_vtc && rx_align_capture_pending_q &&
            (rx_align_settle_count_q != 4'd10)) begin
            if (rx_align_settle_count_q == 4'd0)
                rx_align_valid_q <= 1'b0;
            rx_align_settle_count_q <= rx_align_settle_count_q + 1'b1;
        end else if (!i_bitslice_en_vtc && rx_align_capture_pending_q) begin
            if (!rx_align_candidate_valid_q ||
                (rx_cntvalueout_sample != rx_align_candidate_q)) begin
                rx_align_candidate_q       <= rx_cntvalueout_sample;
                rx_align_stable_count_q    <= 3'd0;
                rx_align_candidate_valid_q <= 1'b1;
            end else if (rx_align_stable_count_q != 3'd3) begin
                rx_align_stable_count_q <= rx_align_stable_count_q + 1'b1;
            end else begin
                for (rx_delay_idx = 0; rx_delay_idx < DQ_BITS;
                     rx_delay_idx = rx_delay_idx + 1) begin
                    if (rx_align_candidate_q[rx_delay_idx*9 +: 9] >=
                        rx_applied_from_offset(
                            rx_align_delay_q[rx_delay_idx*9 +: 9],
                            rx_relative_offset_q[
                                rx_delay_idx*9 +: 9]))
                        rx_align_delay_q[rx_delay_idx*9 +: 9] <=
                            rx_align_candidate_q[rx_delay_idx*9 +: 9] -
                            rx_applied_from_offset(
                                rx_align_delay_q[rx_delay_idx*9 +: 9],
                                rx_relative_offset_q[
                                    rx_delay_idx*9 +: 9]);
                    else
                        // Defensive recovery for a reset/reconfiguration that
                        // invalidated the remembered offset without asserting
                        // the byte reset. Never synthesize a wrapped baseline.
                        rx_align_delay_q[rx_delay_idx*9 +: 9] <=
                            rx_align_candidate_q[rx_delay_idx*9 +: 9];
                end
                rx_align_valid_q           <= 1'b1;
                rx_align_capture_pending_q <= 1'b0;
                rx_align_settle_count_q    <= 4'd0;
                rx_align_stable_count_q    <= 3'd0;
                rx_align_candidate_valid_q <= 1'b0;
            end
        end

        if (i_rx_load && rx_align_valid_q) begin
            for (rx_delay_idx = 0; rx_delay_idx < DQ_BITS;
                 rx_delay_idx = rx_delay_idx + 1)
                rx_relative_offset_q[rx_delay_idx*9 +: 9] <=
                    i_rx_per_dq_mode ?
                    i_rx_cntvaluein_per_dq[rx_delay_idx*9 +: 9] :
                    i_rx_cntvaluein;
        end
    end
end

// Each nibble has an independent RIU select/readback on the shared byte-local
// ingress bus. The parent can write the write-level register to both in the
// same RIU cycle.
wire [15:0] riu_rd_data_low;
wire        riu_rd_valid_low;
wire [15:0] riu_rd_data_upp;
wire        riu_rd_valid_upp;
wire [15:0] riu_rd_data_raw;
wire        riu_rd_valid_raw;
// ---------------------------------------------------------------------------
// Status outputs
// ---------------------------------------------------------------------------
assign o_dly_rdy = dly_rdy_low & dly_rdy_upp;
assign o_vtc_rdy = vtc_rdy_low & vtc_rdy_upp;

// Both nibble RIU readback paths are dedicated byte-local routes. UG571
// requires them to terminate in the RIU_OR belonging to this physical byte;
// routing either BITSLICE_CONTROL readback directly into fabric is illegal.
// RIU_OR presents the pair as the single byte-wide RIU port used by the
// training sequencer, matching the topology generated by the DDR4 MIG.
(* BOX_TYPE = "PRIMITIVE" *)
RIU_OR #(
    .SIM_DEVICE          (SIM_DEVICE),
    .SIM_VERSION         (2.0)
) u_riu_or (
    .RIU_RD_DATA         (riu_rd_data_raw),
    .RIU_RD_VALID        (riu_rd_valid_raw),
    .RIU_RD_DATA_LOW     (riu_rd_data_low),
    .RIU_RD_DATA_UPP     (riu_rd_data_upp),
    .RIU_RD_VALID_LOW    (riu_rd_valid_low),
    .RIU_RD_VALID_UPP    (riu_rd_valid_upp)
);

// NIBBLE_CTRL0[9] is the live gate-placement result and bit 8 is CLR_GATE.
// The primitive status is observed in the RIU clock domain; forwarding only
// the most recent RIU word to DIV_CLK is insufficient because the status can
// return Low before that word completes the CDC handshake.  Clear the sticky
// bit from the actual upper-nibble CLR_GATE write, then accumulate live
// readback until the next candidate performs another clear.
reg riu_gate_status_sticky;
always @(posedge i_riu_clk) begin
    if (i_riu_rst) begin
        riu_gate_status_sticky <= 1'b0;
    end else if (riu_wr_en_q && riu_upper_sel_q &&
                 (riu_addr_q == 6'h00) && riu_wr_data_q[8]) begin
        riu_gate_status_sticky <= 1'b0;
    end else if (!riu_wr_en_q && riu_upper_sel_q &&
                 (riu_addr_q == 6'h00) && riu_rd_valid_raw &&
                 riu_rd_data_raw[9]) begin
        riu_gate_status_sticky <= 1'b1;
    end
end

(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg [1:0] riu_gate_status_sync;
always @(posedge i_div_clk) begin
    if (i_bsc_rst)
        riu_gate_status_sync <= 2'b00;
    else
        riu_gate_status_sync <=
            {riu_gate_status_sync[0], riu_gate_status_sticky};
end
assign o_riu_gate_status_sticky = riu_gate_status_sync[1];

// Return exactly one address-tagged RIU response for each settled read or
// completed write. RIU_VALID is a port-availability level, not a per-read
// pulse (UG571), so toggling a CDC event on every High cycle can alias an even
// number of source transitions and associate data from the preceding address
// with the current request. Wait three RIU clocks after the effective request
// changes (or a write is launched), then capture one stable readback. The tag
// prevents a late response from an abandoned request being accepted in DIV_CLK.
reg [15:0] riu_response_data_hold;
reg        riu_response_toggle;
reg [5:0]  riu_response_addr_hold;
reg        riu_response_lower_hold, riu_response_upper_hold;
reg [5:0]  riu_response_addr_q;
reg        riu_response_lower_q, riu_response_upper_q;
reg [1:0]  riu_response_settle_q;
reg        riu_response_pending_q;
wire       riu_response_request_changed =
    (riu_addr_q != riu_response_addr_q) ||
    (riu_lower_sel_q != riu_response_lower_q) ||
    (riu_upper_sel_q != riu_response_upper_q);
always @(posedge i_riu_clk) begin
    if (i_riu_rst) begin
        riu_response_data_hold <= 16'd0;
        riu_response_toggle    <= 1'b0;
        riu_response_addr_hold <= 6'd0;
        riu_response_lower_hold <= 1'b0;
        riu_response_upper_hold <= 1'b0;
        riu_response_addr_q    <= 6'd0;
        riu_response_lower_q   <= 1'b0;
        riu_response_upper_q   <= 1'b0;
        riu_response_settle_q  <= 2'd0;
        riu_response_pending_q <= 1'b1;
    end else if (riu_wr_en_q || riu_response_request_changed) begin
        riu_response_addr_q    <= riu_addr_q;
        riu_response_lower_q   <= riu_lower_sel_q;
        riu_response_upper_q   <= riu_upper_sel_q;
        riu_response_settle_q  <= 2'd0;
        riu_response_pending_q <= 1'b1;
    end else if (riu_response_pending_q) begin
        if (riu_response_settle_q != 2'd3) begin
            riu_response_settle_q <= riu_response_settle_q + 1'b1;
        end else if (riu_rd_valid_raw &&
                     (riu_lower_sel_q || riu_upper_sel_q)) begin
            riu_response_data_hold  <= riu_rd_data_raw;
            riu_response_addr_hold  <= riu_addr_q;
            riu_response_lower_hold <= riu_lower_sel_q;
            riu_response_upper_hold <= riu_upper_sel_q;
            riu_response_toggle     <= ~riu_response_toggle;
            riu_response_pending_q  <= 1'b0;
        end
    end
end

(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg [1:0] riu_response_toggle_sync;
reg       riu_response_toggle_seen;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg [15:0] riu_response_data_meta, riu_response_data_sync;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg [5:0] riu_response_addr_meta, riu_response_addr_sync;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg riu_response_lower_meta, riu_response_lower_sync;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *)
reg riu_response_upper_meta, riu_response_upper_sync;
reg [5:0] riu_request_addr_q;
reg       riu_request_lower_q, riu_request_upper_q;
wire      riu_request_changed =
    (i_riu_addr != riu_request_addr_q) ||
    (i_riu_lower_sel != riu_request_lower_q) ||
    (i_riu_upper_sel != riu_request_upper_q);
always @(posedge i_div_clk) begin
    if (i_bsc_rst) begin
        riu_response_toggle_sync <= 2'b00;
        riu_response_toggle_seen <= 1'b0;
        riu_response_data_meta   <= 16'd0;
        riu_response_data_sync   <= 16'd0;
        riu_response_addr_meta   <= 6'd0;
        riu_response_addr_sync   <= 6'd0;
        riu_response_lower_meta  <= 1'b0;
        riu_response_lower_sync  <= 1'b0;
        riu_response_upper_meta  <= 1'b0;
        riu_response_upper_sync  <= 1'b0;
        riu_request_addr_q       <= 6'd0;
        riu_request_lower_q      <= 1'b0;
        riu_request_upper_q      <= 1'b0;
        o_riu_rd_data            <= 16'd0;
        o_riu_valid              <= 1'b0;
    end else begin
        riu_response_toggle_sync <=
            {riu_response_toggle_sync[0], riu_response_toggle};
        riu_response_data_meta <= riu_response_data_hold;
        riu_response_data_sync <= riu_response_data_meta;
        riu_response_addr_meta <= riu_response_addr_hold;
        riu_response_addr_sync <= riu_response_addr_meta;
        riu_response_lower_meta <= riu_response_lower_hold;
        riu_response_lower_sync <= riu_response_lower_meta;
        riu_response_upper_meta <= riu_response_upper_hold;
        riu_response_upper_sync <= riu_response_upper_meta;
        riu_request_addr_q  <= i_riu_addr;
        riu_request_lower_q <= i_riu_lower_sel;
        riu_request_upper_q <= i_riu_upper_sel;

        // Hold completion until the next transaction. Each byte has its own
        // toggle synchronizer, so one byte can legally observe a response one
        // DIV_CLK later than another. Sticky completion lets the parent AND
        // all byte-valid bits without requiring coincident response pulses.
        if (i_riu_wr_en || riu_request_changed)
            o_riu_valid <= 1'b0;
        if (riu_response_toggle_sync[1] !=
            riu_response_toggle_seen) begin
            riu_response_toggle_seen <= riu_response_toggle_sync[1];
            if (!(i_riu_wr_en || riu_request_changed) &&
                (riu_response_addr_sync == i_riu_addr) &&
                (riu_response_lower_sync == i_riu_lower_sel) &&
                (riu_response_upper_sync == i_riu_upper_sel)) begin
                o_riu_rd_data <= riu_response_data_sync;
                o_riu_valid <= 1'b1;
            end
        end
    end
end

// ---------------------------------------------------------------------------
// Per-nibble readiness diagnostics
// ---------------------------------------------------------------------------
// Native BITSLICE DATAIN has an exclusive IOB-to-BITSLICE route and therefore
// must never be tapped by fabric debug logic (Vivado DRC REQP-1922).
// FIFO_WRCLK_OUT also requires scarce clock routing and is deliberately left
// unused. DLY_RDY/VTC_RDY are the nonintrusive status boundary provided by
// BITSLICE_CONTROL, while FIFO_EMPTY/Q and RIU status are observed upstream.
reg [3:0] nibble_ready_q;

always @(posedge i_div_clk) begin
    if (i_bitslice_rst)
        nibble_ready_q <= 4'b0000;
    else
        nibble_ready_q <= {vtc_rdy_upp, vtc_rdy_low,
                           dly_rdy_upp, dly_rdy_low};
end

assign o_dbg_nibble_ready = nibble_ready_q;
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
`ifdef SIM_NATIVE_DIAG_RXGATE_EXTEND
    .RXGATE_EXTEND      ("TRUE"),
`else
    .RXGATE_EXTEND      ("FALSE"),
`endif
    // This generic wrapper places DQS in upper position 0 and imports it into
    // the lower nibble.  SHIFT_90 is the corresponding native FIFO framing
    // phase; the per-DQ RX delay below then locates the eye center.  Unlike the
    // MIG wrapper (which has a different internal DQS placement), SHIFT_0 here
    // produces complete FIFO words with no valid cyclic MPR pattern.
`ifdef SIM_NATIVE_DIAG_RX_SHIFT0
    .RX_CLK_PHASE_P     ("SHIFT_0"),
    .RX_CLK_PHASE_N     ("SHIFT_0"),
`else
    .RX_CLK_PHASE_P     ("SHIFT_90"),
    .RX_CLK_PHASE_N     ("SHIFT_90"),
`endif
    // DQS occupies position 0 of the upper nibble in this generic byte
    // wrapper.  The lower DQ nibble must therefore import that dedicated
    // source-synchronous clock through the inter-nibble links.  The reverse
    // links remain connected but are not selected by the upper control.
    .EN_OTHER_PCLK      ("TRUE"),
    .EN_OTHER_NCLK      ("TRUE"),
    .SELF_CALIBRATE     ("ENABLE"),
    // PHY_RDEN is a command-timed BL8 window, so native DQS gating must be
    // enabled. UG571 permits RX_GATING=DISABLE only when PHY_RDEN is tied
    // permanently High; pulsing it in that mode leaves FIFO framing dependent
    // on the byte's incidental power-up/board phase.
    .RX_GATING          ("ENABLE"),
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
    .RIU_CLK            (i_riu_clk),
    .RST                (i_riu_rst),
    .EN_VTC             (i_bsc_en_vtc),
    .DLY_RDY            (dly_rdy_low),
    .VTC_RDY            (vtc_rdy_low),
    // UG571 requires an unused inter-byte clock input to be pulled High.
    .CLK_FROM_EXT       (1'b1),
    .PCLK_NIBBLE_IN     (pclk_nibble_out_upp),
    .NCLK_NIBBLE_IN     (nclk_nibble_out_upp),
    .PCLK_NIBBLE_OUT    (pclk_nibble_out_low),
    .NCLK_NIBBLE_OUT    (nclk_nibble_out_low),
    .TBYTE_IN           (i_tbyte_dq),
    // Gate each BL8 receive window with the command-timed mask. The parent
    // sweeps RL_DLY while observing fresh FIFO words, then the eye stage
    // verifies the selected window against the exact MPR pattern.
    .PHY_RDEN           (i_phy_rden_lower),
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
    .RIU_NIBBLE_SEL     (riu_lower_sel_q),
    .RIU_RD_DATA        (riu_rd_data_low),
    .RIU_VALID          (riu_rd_valid_low)
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
`ifdef SIM_NATIVE_DIAG_RXGATE_EXTEND
    .RXGATE_EXTEND      ("TRUE"),
`else
    .RXGATE_EXTEND      ("FALSE"),
`endif
`ifdef SIM_NATIVE_DIAG_RX_SHIFT0
    .RX_CLK_PHASE_P     ("SHIFT_0"),
    .RX_CLK_PHASE_N     ("SHIFT_0"),
`else
    .RX_CLK_PHASE_P     ("SHIFT_90"),
    .RX_CLK_PHASE_N     ("SHIFT_90"),
`endif
    .EN_OTHER_PCLK      ("FALSE"),
    .EN_OTHER_NCLK      ("FALSE"),
    .SELF_CALIBRATE     ("ENABLE"),
    // The upper nibble owns DQS and the lower nibble imports its gated
    // PCLK/NCLK. Keep both controls in the same RX-gating mode, matching the
    // native DDR4 MIG byte topology.
    .RX_GATING          ("ENABLE"),
    .READ_IDLE_COUNT    (31),
    .TX_GATING          ("ENABLE"),
    .REFCLK_SRC         ("PLLCLK"),
    .SIM_DEVICE         (SIM_DEVICE)
) u_bsc_upper (
    .PLL_CLK            (i_pll_clkoutphy),
    .REFCLK             (1'b0),
    .RIU_CLK            (i_riu_clk),
    .RST                (i_riu_rst),
    .EN_VTC             (i_bsc_en_vtc),
    .DLY_RDY            (dly_rdy_upp),
    .VTC_RDY            (vtc_rdy_upp),
    // No inter-byte clock is used by this generic byte interface.  The
    // PCLK/NCLK ports below are the dedicated *inter-nibble* return links,
    // not inter-byte clocks; keep them connected even though this upper
    // control sources the DQS and therefore leaves EN_OTHER_* disabled.
    .CLK_FROM_EXT       (1'b1),
    .PCLK_NIBBLE_IN     (pclk_nibble_out_low),
    .NCLK_NIBBLE_IN     (nclk_nibble_out_low),
    .PCLK_NIBBLE_OUT    (pclk_nibble_out_upp),
    .NCLK_NIBBLE_OUT    (nclk_nibble_out_upp),
    .TBYTE_IN           (i_tbyte_dqs),
    .PHY_RDEN           (i_phy_rden_upper),
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
    .RIU_NIBBLE_SEL     (riu_upper_sel_q),
    .RIU_RD_DATA        (riu_rd_data_upp),
    .RIU_VALID          (riu_rd_valid_upp)
);
// ---------------------------------------------------------------------------
// TX_BITSLICE_TRI - Lower nibble (DQ tristate)
// ---------------------------------------------------------------------------
TX_BITSLICE_TRI #(
    .DATA_WIDTH         (8),
    // Match the generated DDR4 MIG byte topology: the shared tristate
    // serializer uses the 90-degree TX phase while DQ/DM use the unshifted
    // phase.  The parent presents TBYTE one complete DIV_CLK word before
    // registered DQ, preserving the preamble, first data UI, and postamble as
    // one coherent physical write window.
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
    // The upper nibble carries DQ[7:4] and DQS.  MIG uses the same 90-degree
    // tristate phase here as on the lower nibble; DQ remains unshifted and DQS
    // separately selects its required 90-degree generated-clock phase.
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
    .ENABLE_PRE_EMPHASIS("TRUE"),
    .RX_UPDATE_MODE     ("ASYNC"),
    .TX_UPDATE_MODE     ("ASYNC"),
    .RX_DATA_TYPE       ("DATA"),
    .RX_DATA_WIDTH      (8),
    .TX_DATA_WIDTH      (8),
    // Match the generated DDR4 MIG for this TX-only slice.  Keeping every
    // slice in the nibble in TIME mode lets BISC calibrate one coherent byte.
    .RX_DELAY_FORMAT    ("TIME"),
    .TX_DELAY_FORMAT    ("TIME"),
    .RX_DELAY_TYPE      ("FIXED"),
    .TX_DELAY_TYPE      ("FIXED"),
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
    .RX_CLK             (1'b1),
    .TX_CLK             (1'b1),
    .RX_EN_VTC          (1'b1),
    .TX_EN_VTC          (1'b1),
    .RX_CE              (1'b0),
    .RX_INC             (1'b0),
    .TX_CE              (1'b0),
    .TX_INC             (1'b0),
    .RX_CNTVALUEIN      (9'd0),
    .RX_LOAD            (1'b0),
    .RX_CNTVALUEOUT     (),
    .TX_CNTVALUEIN      (9'd0),
    .TX_LOAD            (1'b0),
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
    .ENABLE_PRE_EMPHASIS("TRUE"),
    .RX_UPDATE_MODE     ("ASYNC"),
    .TX_UPDATE_MODE     ("ASYNC"),
    .RX_DATA_TYPE       ("DATA_AND_CLOCK"),
    .RX_DATA_WIDTH      (8),
    .TX_DATA_WIDTH      (8),
    // DQS is the native source-synchronous receive clock.  Match the XiPHY
    // configuration used by MIG: TIME/FIXED lets BISC insert and maintain the
    // input Align_Delay between the pad clock path and the native capture
    // registers.  COUNT mode deliberately omits that clock-path alignment and
    // is not reliable across DBC/QBC byte positions or PVT.  Write leveling
    // does not modify this direct delay line; it uses BITSLICE_CONTROL's
    // rank-specific WL_DLY_RNK0 setting for the complete byte instead.
    // UG571 requires RX_DELAY_FORMAT and TX_DELAY_FORMAT to match for BISC.
    .RX_DELAY_FORMAT    ("TIME"),
    .TX_DELAY_FORMAT    ("TIME"),
    .RX_DELAY_TYPE      ("FIXED"),
    .TX_DELAY_TYPE      ("FIXED"),
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
    .FIFO_EMPTY         (o_dqs_fifo_empty),
    .RX_RST             (i_rx_fifo_rst),
    .TX_RST             (i_bitslice_rst),
    .RX_RST_DLY         (i_bitslice_rst),
    .TX_RST_DLY         (i_bitslice_rst),
    // Both direct delay lines are FIXED.  The TX control ports remain wired
    // for a uniform byte-slice interface but are inactive in this mode.
    .RX_CLK             (1'b1),
    .TX_CLK             (1'b1),
    // UG571 requires EN_VTC High for a FIXED delay in TIME mode.  Keep the
    // DQS BISC alignment is maintained independently of the DQ eye-update
    // session, whose TIME/VAR_LOAD RX paths temporarily lower EN_VTC.
    .RX_EN_VTC          (1'b1),
    .TX_EN_VTC          (1'b1),
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
    .Q                  (o_dqs_fifo_data),
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
assign o_rx_cntvalueout_dq0 = rx_cntvalueout_dq[0];
genvar gi;
generate
for (gi = 0; gi < 4; gi = gi + 1) begin : gen_dq_lower
    localparam integer LOGICAL_DQ = dq_pin_at(0, gi + 2);
    wire [39:0] rx_ctrl_out_w;
    wire [39:0] tx_ctrl_out_w;
    wire [39:0] rx_ctrl_in_w;
    wire [39:0] tx_ctrl_in_w;
    wire [8:0] rx_requested_offset_w = i_rx_per_dq_mode ?
        i_rx_cntvaluein_per_dq[LOGICAL_DQ*9 +: 9] :
        i_rx_cntvaluein;
    wire [8:0] rx_cntvaluein_w = rx_total_from_offset(
        rx_align_delay_q[LOGICAL_DQ*9 +: 9], rx_requested_offset_w);
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
        .ENABLE_PRE_EMPHASIS("TRUE"),
        .RX_UPDATE_MODE     ("ASYNC"),
        .TX_UPDATE_MODE     ("ASYNC"),
        .RX_DATA_TYPE       ("DATA"),
        .RX_DATA_WIDTH      (8),
        .TX_DATA_WIDTH      (8),
        // TIME mode lets BISC remove the per-bit clock/data insertion skew.
        // RX remains variable for eye centering.  TX is VAR_LOAD so the
        // board-bring-up diagnostic can measure the physical write eye one DQ
        // at a time; with LOAD inactive it is behaviorally identical to the
        // previous zero-delay FIXED configuration.
        // Match the generated DDR4 MIG transmit relationship: DQ and DM use
        // the unshifted serializer phase, while DQS and the shared TBYTE
        // serializer use the 90-degree phase.  Shifting DQ together with DQS
        // removes the source-synchronous quarter-cycle separation and launches
        // DQ transitions on the DQS sampling edges, which becomes unreliable
        // as tCK is reduced.
        .RX_DELAY_FORMAT    ("TIME"),
        .TX_DELAY_FORMAT    ("TIME"),
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
        // DQS sampling and serial TX clocks come from BITSLICE_CONTROL.
        // RX_CLK captures the TIME/VAR_LOAD update in the controller domain.
        .RX_CLK             (i_div_clk),
        .TX_CLK             (i_div_clk),
        .RX_EN_VTC          (i_bitslice_en_vtc),
        .TX_EN_VTC          (i_tx_dq_en_vtc),
        .RX_CE              (1'b0),
        .RX_INC             (1'b0),
        .TX_CE              (i_tx_dq_ce[LOGICAL_DQ]),
        .TX_INC             (i_tx_dq_inc),
        .RX_CNTVALUEIN      (rx_cntvaluein_w),
        .RX_LOAD            (i_rx_load),
        .RX_CNTVALUEOUT     (rx_cntvalueout_dq[LOGICAL_DQ]),
        .TX_CNTVALUEIN      (i_tx_dq_cntvaluein),
        .TX_LOAD            (i_tx_dq_load[LOGICAL_DQ]),
        .TX_CNTVALUEOUT     (o_tx_dq_cntvalueout[LOGICAL_DQ*9 +: 9]),
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
    wire [8:0] rx_requested_offset_w = i_rx_per_dq_mode ?
        i_rx_cntvaluein_per_dq[LOGICAL_DQ*9 +: 9] :
        i_rx_cntvaluein;
    wire [8:0] rx_cntvaluein_w = rx_total_from_offset(
        rx_align_delay_q[LOGICAL_DQ*9 +: 9], rx_requested_offset_w);
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
        .ENABLE_PRE_EMPHASIS("TRUE"),
        .RX_UPDATE_MODE     ("ASYNC"),
        .TX_UPDATE_MODE     ("ASYNC"),
        .RX_DATA_TYPE       ("DATA"),
        .RX_DATA_WIDTH      (8),
        .TX_DATA_WIDTH      (8),
        .RX_DELAY_FORMAT    ("TIME"),
        .TX_DELAY_FORMAT    ("TIME"),
        .RX_DELAY_TYPE      ("VAR_LOAD"),
        .TX_DELAY_TYPE      ("VAR_LOAD"),
        .RX_DELAY_VALUE     (0),
        .TX_DELAY_VALUE     (0),
        // The upper-nibble DQ bits use the same unshifted phase as lower DQ
        // and DM.  DQS and TBYTE alone select the 90-degree serializer phase.
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
        // See the lower-nibble DATA slice: these clocks capture delay-control
        // LOAD strobes; BITSLICE_CONTROL supplies the actual datapath clocks.
        .RX_CLK             (i_div_clk),
        .TX_CLK             (i_div_clk),
        .RX_EN_VTC          (i_bitslice_en_vtc),
        .TX_EN_VTC          (i_tx_dq_en_vtc),
        .RX_CE              (1'b0),
        .RX_INC             (1'b0),
        .TX_CE              (i_tx_dq_ce[LOGICAL_DQ]),
        .TX_INC             (i_tx_dq_inc),
        .RX_CNTVALUEIN      (rx_cntvaluein_w),
        .RX_LOAD            (i_rx_load),
        .RX_CNTVALUEOUT     (rx_cntvalueout_dq[LOGICAL_DQ]),
        .TX_CNTVALUEIN      (i_tx_dq_cntvaluein),
        .TX_LOAD            (i_tx_dq_load[LOGICAL_DQ]),
        .TX_CNTVALUEOUT     (o_tx_dq_cntvalueout[LOGICAL_DQ*9 +: 9]),
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
    // UltraScale/UltraScale+ native mode uses the E3 buffer so application TX
    // can be blocked from the source-synchronous RX path.  Write leveling
    // keeps the DQ receiver enabled and therefore still receives DRAM
    // feedback.
    IOBUFE3 #(
        .SIM_DEVICE       (SIM_DEVICE),
        .USE_IBUFDISABLE ("TRUE")
    ) u_iobuf_dq (
        .O               (dq_from_iob[gi]),
        .IO              (io_ddr4_dq[gi]),
        .I               (dq_to_iob[gi]),
        .T               (dq_t[gi]),
        .IBUFDISABLE     (i_rx_dq_input_disable),
        .DCITERMDISABLE  (1'b0),
        .OSC             (4'b0000),
        .OSC_EN          (1'b0),
        .VREF            (1'b0)
    );
end
endgenerate
// ---------------------------------------------------------------------------
// IOB - DQS differential pair
// ---------------------------------------------------------------------------
// The E3 differential buffer suppresses locally transmitted application DQS
// before it can clock an ignored word into the native RX FIFO.  Its dedicated
// control also masks write-level DQS pad ownership transitions while leaving
// the DQ receivers active for JEDEC feedback.
IOBUFDSE3 #(
    .DQS_BIAS          ("TRUE"),
    .SIM_DEVICE        (SIM_DEVICE),
    .USE_IBUFDISABLE  ("TRUE")
) u_iobufds_dqs (
    .O                (dqs_from_iob),
    .IO               (io_ddr4_dqs_p),
    .IOB              (io_ddr4_dqs_n),
    .I                (dqs_to_iob),
    .T                (dqs_t),
    .IBUFDISABLE      (i_rx_dqs_input_disable),
    .DCITERMDISABLE   (1'b0),
    .OSC              (4'b0000),
    .OSC_EN           (2'b00)
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
