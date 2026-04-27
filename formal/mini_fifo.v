// mini_fifo.v — 2-entry FIFO oracle for pipeline formal verification
// Reused from UberDDR3 (ZipCPU pattern). Tracks what enters and exits
// the controller pipeline as an independent reference model.
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