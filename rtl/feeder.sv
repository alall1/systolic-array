module feeder
import pe_pkg::*;
(
    input logic clk,
    input logic rst_n,  // active low

    input logic start,
    input logic [DIM_W-1:0] M,
    input logic [DIM_W-1:0] N,
    input logic [K_W-1:0] K,
    output logic busy,
    output logic done,

    
);

read_sequencer rd_seq (
    .clk(clk),
    .rst_n(rst_n),

    .start(),

    .busy(),
    .done(),

    .M(),
    .N(),
    .K(),

    .rd_en(),
    .rd_col_a(),
    .rd_row_b(),

    .rd_a(),
    .rd_b(),

    .out_a(),
    .out_b()
);

skew_buffer skew_buf (
    .clk(clk),
    .rst_n(rst_n),

    .in_a(),
    .out_a(),

    .in_b(),
    .out_b()
);

endmodule