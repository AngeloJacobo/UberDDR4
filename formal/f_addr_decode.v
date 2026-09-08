// f_addr_decode.v -- Independent address decoder for formal verification
//
// Purpose: provides a second, structurally independent implementation
// of the Wishbone-to-DDR4 address decode. The formal harness feeds the
// same wb_addr into this module and into the controller's pipeline
// registers, then asserts the outputs match under its active guards. This
// catches disagreements with the reference mapping in the constrained model;
// it does not independently validate a board's physical memory geometry.
// wb_addr is a burst-word address; COL_LOW zero bits are appended to col.
// See docs/VERIFICATION.md for task configuration and assumptions.
//
// Supports both address mappings:
//   map0: {row, bg, ba, col_upper}
//   map1: {row, ba, col_upper, bg} (BG-interleaved for throughput)
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
    parameter COL_LOW   = 3
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
            // ADDR_MAPPING==1: {row, ba, col_upper, bg}  --  BG-interleaved
            // bank encoding: {bg, ba} matching controller's wb_bank = {wb_bg, wb_ba}
            assign bank[BG_BITS+BA_BITS-1:BA_BITS]  = wb_addr[BG_BITS-1:0];
            assign col                              = {wb_addr[BG_BITS +: COL_HIGH], {COL_LOW{1'b0}}};
            assign bank[BA_BITS-1:0]                = wb_addr[BG_BITS+COL_HIGH +: BA_BITS];
            assign row                              = wb_addr[BG_BITS+COL_HIGH+BA_BITS +: ROW_BITS];
        end
    endgenerate

endmodule