// mini_fifo.v -- 2-entry FIFO oracle for pipeline formal verification
//
// Purpose: acts as an independent shadow model of the controller's
// 2-stage pipeline. Every WB request accepted by the controller is
// simultaneously written into this FIFO (with address + direction),
// and every scheduler fire (WR/RD) pops the FIFO. The formal harness
// then asserts:
//   - Occupancy match: FIFO full/empty tracks stage1/stage2 pending
//     flags exactly (Prop 5).
//   - Data integrity: FIFO head data matches the pipeline's decoded
//     address fields (Prop 6), cross-checked by f_addr_decode.
//
// If the pipeline ever drops, duplicates, or reorders a request, the
// FIFO can expose the disagreement through harness assertions, subject to
// their guards and assumptions. This helper is used at FIFO_WIDTH=1 (2 entries);
// read_data_next uses logical pointer inversion and is specific to that width.
// See docs/VERIFICATION.md for the scope of the surrounding proof.
//
// Reused from UberDDR3 (ZipCPU pattern).
//
// Engineer: Angelo C. Jacobo
// Copyright (c) 2025, Angelo C. Jacobo
// License: GPL v3
`default_nettype none
`timescale 1ps/1ps

module mini_fifo #(
    parameter FIFO_WIDTH = 1,
    parameter DATA_WIDTH = 8
)(
    input  wire i_clk, i_rst_n,
    input  wire read_fifo, write_fifo,
    output reg  empty, full,
    input  wire [DATA_WIDTH-1:0] write_data,
    output wire [DATA_WIDTH-1:0] read_data,
    output wire [DATA_WIDTH-1:0] read_data_next
);
    reg [FIFO_WIDTH-1:0] write_pointer, read_pointer;
    reg [DATA_WIDTH-1:0] fifo_reg [2**FIFO_WIDTH-1:0];

    initial begin
        write_pointer = 0;
        read_pointer  = 0;
        empty = 1;
        full  = 0;
    end

    always @(posedge i_clk) begin
        if (!i_rst_n) begin
            empty <= 1;
            full  <= 0;
            read_pointer  <= 0;
            write_pointer <= 0;
        end else begin
            if (read_fifo) begin
`ifdef FORMAL
                assert(!empty);
`endif
                if (!write_fifo) full <= 0;
                read_pointer <= read_pointer + 1;
                if (read_pointer + 1'b1 == write_pointer && !write_fifo)
                    empty <= 1;
            end
            if (write_fifo) begin
`ifdef FORMAL
                if (!read_fifo) assert(!full);
`endif
                if (!read_fifo) empty <= 0;
                fifo_reg[write_pointer] <= write_data;
                write_pointer <= write_pointer + 1;
                if (write_pointer + 1'b1 == read_pointer && !read_fifo)
                    full <= 1'b1;
            end
        end
    end

    assign read_data      = fifo_reg[read_pointer];
    assign read_data_next = fifo_reg[!read_pointer];

`ifdef FORMAL
    always @* begin
        if (empty || full)
            assert(write_pointer == read_pointer);
        if (write_pointer == read_pointer)
            assert(empty || full);
        assert(!(empty && full));
    end
`endif

endmodule