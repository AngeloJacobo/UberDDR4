////////////////////////////////////////////////////////////////////////////////
// Native PHY primitive-reset sequencer
//
// Sequences PLL lock, BITSLICE_CONTROL/BITSLICE reset release, delay ready,
// VTC ready, TBYTE and PHY read enable. The programmed settle counts are
// fixed; total startup time also depends on the incoming ready/lock signals.
// The controller's separate TPHY_INIT_LAT=9 native guard covers initial CA/CKE
// serialization, not this entire readiness sequence. See docs/ARCHITECTURE.md.
// i_clk is the native controller/DIV clock; reset asserts asynchronously.
////////////////////////////////////////////////////////////////////////////////

`default_nettype none

module ddr4_phy_native_reset (
    input  wire       i_clk,
    input  wire       i_rst_n,
    input  wire       i_pll_locked,
    input  wire       i_dly_rdy,
    input  wire       i_vtc_rdy,
    output reg        o_pll_rst,
    output reg        o_bsc_rst,
    output reg        o_bitslice_rst,
    output reg        o_clkoutphy_en,
    output reg        o_en_vtc,
    output reg        o_tbyte_en,
    output reg        o_phy_rden,
    output reg        o_init_complete,
    output wire [3:0] o_phy_state
);
// State encoding is exposed through o_phy_state for implementation debug.
localparam [3:0] RST_IDLE       = 4'd0,
                 RST_PLL_WAIT   = 4'd1,
                 RST_RELEASE    = 4'd2,
                 RST_CLKOUTPHY  = 4'd3,
                 RST_DLY_WAIT   = 4'd4,
                 RST_EN_VTC     = 4'd5,
                 RST_VTC_WAIT   = 4'd6,
                 RST_TBYTE      = 4'd7,
                 RST_PHY_RDEN   = 4'd8,
                 RST_DONE       = 4'd9;

// Cycle counts are inclusive because each state starts with cnt == 0.
localparam [6:0] BSC_RELEASE_LAST = 7'd63,
                 CLKOUTPHY_LAST   = 7'd15,
                 TBYTE_LAST       = 7'd3,
                 PHY_RDEN_LAST    = 7'd3;

// State and counter registers
reg [3:0] state;
reg [6:0] cnt;

// EN_VTC qualification pipeline. o_en_vtc is generated in this same clock
// domain; two registered observations ensure the primitive control has been
// asserted before VTC_RDY is accepted.
reg en_vtc_q;
reg en_vtc_qq;

assign o_phy_state = state;

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        en_vtc_q  <= 1'b0;
        en_vtc_qq <= 1'b0;
    end else begin
        en_vtc_q  <= o_en_vtc;
        en_vtc_qq <= en_vtc_q;
    end
end

// Restart the complete primitive sequence if PLL lock is lost after it was
// first acquired. This leaves every BITSLICE control in a known safe state.
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        state          <= RST_IDLE;
        cnt            <= 7'd0;
        o_pll_rst      <= 1'b1;
        o_bsc_rst      <= 1'b1;
        o_bitslice_rst <= 1'b1;
        o_clkoutphy_en <= 1'b0;
        o_en_vtc       <= 1'b0;
        o_tbyte_en     <= 1'b0;
        o_phy_rden     <= 1'b0;
        o_init_complete <= 1'b0;
    end else if (state != RST_IDLE && state != RST_PLL_WAIT && !i_pll_locked) begin
        state          <= RST_IDLE;
        cnt            <= 7'd0;
        o_pll_rst      <= 1'b1;
        o_bsc_rst      <= 1'b1;
        o_bitslice_rst <= 1'b1;
        o_clkoutphy_en <= 1'b0;
        o_en_vtc       <= 1'b0;
        o_tbyte_en     <= 1'b0;
        o_phy_rden     <= 1'b0;
        o_init_complete <= 1'b0;
    end else begin
        case (state)
            RST_IDLE: begin
                o_pll_rst      <= 1'b1;
                o_bsc_rst      <= 1'b1;
                o_bitslice_rst <= 1'b1;
                o_clkoutphy_en <= 1'b0;
                o_en_vtc       <= 1'b0;
                o_tbyte_en     <= 1'b0;
                o_phy_rden     <= 1'b0;
                o_init_complete <= 1'b0;
                cnt            <= 7'd0;
                state          <= RST_PLL_WAIT;
            end
            RST_PLL_WAIT: begin
                o_pll_rst <= 1'b0;
                if (i_pll_locked) begin
                    state <= RST_RELEASE;
                    cnt   <= 7'd0;
                end
            end
            RST_RELEASE: begin
                o_bsc_rst      <= 1'b0;
                o_bitslice_rst <= 1'b0;
                if (cnt == BSC_RELEASE_LAST) begin
                    state <= RST_CLKOUTPHY;
                    cnt   <= 7'd0;
                end else begin
                    cnt <= cnt + 7'd1;
                end
            end
            RST_CLKOUTPHY: begin
                o_clkoutphy_en <= 1'b1;
                if (cnt == CLKOUTPHY_LAST) begin
                    state <= RST_DLY_WAIT;
                    cnt   <= 7'd0;
                end else begin
                    cnt <= cnt + 7'd1;
                end
            end
            RST_DLY_WAIT: begin
                if (i_dly_rdy) begin
                    state <= RST_EN_VTC;
                end
            end
            RST_EN_VTC: begin
                o_en_vtc <= 1'b1;
                state    <= RST_VTC_WAIT;
            end
            RST_VTC_WAIT: begin
                if (i_vtc_rdy && en_vtc_qq) begin
                    state <= RST_TBYTE;
                    cnt   <= 7'd0;
                end
            end
            RST_TBYTE: begin
                o_tbyte_en <= 1'b1;
                if (cnt == TBYTE_LAST) begin
                    state <= RST_PHY_RDEN;
                    cnt   <= 7'd0;
                end else begin
                    cnt <= cnt + 7'd1;
                end
            end
            RST_PHY_RDEN: begin
                o_phy_rden <= 1'b1;
                if (cnt == PHY_RDEN_LAST) begin
                    state <= RST_DONE;
                end else begin
                    cnt <= cnt + 7'd1;
                end
            end
            RST_DONE: begin
                o_init_complete <= 1'b1;
            end
            default: begin
                state <= RST_IDLE;
            end
        endcase
    end
end
endmodule

`default_nettype wire
