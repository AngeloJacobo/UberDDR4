// f_addr_decode.v — Independent address decoder for formal verification
// Mirrors the controller's Wishbone address decode as a separate
// implementation. Any bug in the controller's decode is caught because
// the FIFO oracle's reference decode differs.
//
// Engineer: Angelo C. Jacobo
// Copyright (c) 2025, Angelo C. Jacobo
// License: GPL v3
`default_nettype none
`timescale 1ps/1ps

module f_addr_decode #(
    parameter ADDR_MAPPING = 0,
    parameter ROW_BITS  = 15,
    parameter BG_BITS   = 2,
    parameter BA_BITS   = 2,
    parameter COL_BITS  = 10,
    parameter COL_LOW   = 4
)(
    input  wire [ROW_BITS+BG_BITS+BA_BITS+COL_BITS-COL_LOW-1:0] wb_addr,
    output wire [BG_BITS+BA_BITS-1:0] bank,
    output wire [COL_BITS-1:0]        col,
    output wire [ROW_BITS-1:0]        row
);
    localparam COL_HIGH = COL_BITS - COL_LOW;

    generate
        if (ADDR_MAPPING == 0) begin : map0
            // {row, bg, ba, col_upper}
            assign col  = {wb_addr[COL_HIGH-1:0], {COL_LOW{1'b0}}};
            assign bank = wb_addr[COL_HIGH +: BG_BITS+BA_BITS];
            assign row  = wb_addr[COL_HIGH+BG_BITS+BA_BITS +: ROW_BITS];
        end else begin : map1
            // ADDR_MAPPING==1: {row, ba, col_upper, bg} — BG-interleaved
            // bank encoding: {bg, ba} matching controller's wb_bank = {wb_bg, wb_ba}
            assign bank[BG_BITS+BA_BITS-1:BA_BITS]  = wb_addr[BG_BITS-1:0];
            assign col                              = {wb_addr[BG_BITS +: COL_HIGH], {COL_LOW{1'b0}}};
            assign bank[BA_BITS-1:0]                = wb_addr[BG_BITS+COL_HIGH +: BA_BITS];
            assign row                              = wb_addr[BG_BITS+COL_HIGH+BA_BITS +: ROW_BITS];
        end
    endgenerate

endmodule