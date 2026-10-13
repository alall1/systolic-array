module feeder
import pe_pkg::*;
(
    input logic clk,
    input logic rst_n,  // active low

    // job control
    input  logic start,                     // 1-cycle pulse; ignored while busy
    input  logic [DIM_W-1:0] M,             // captured on the start edge
    input  logic [DIM_W-1:0] N,
    input  logic [K_W-1:0] K,
    output logic busy,
    output logic done,                      // last payload issued to skew (not yet delivered)

    // operand buffer read port (buffer is synchronous, 1-cycle read latency)
    output logic rd_en,
    output logic [K_W-1:0] rd_col_a,        // which column of A (= k)
    output logic [K_W-1:0] rd_row_b,        // which row of B    (= k)
    input  logic signed [DATA_WIDTH-1:0] rd_a [0:ARRAY_DIM-1],
    input  logic signed [DATA_WIDTH-1:0] rd_b [0:ARRAY_DIM-1],

    // skewed operand stream to pe_grid
    output a_payload_t out_a [0:ARRAY_DIM-1],
    output b_payload_t out_b [0:ARRAY_DIM-1]
);

// sequencer -> skew_buffer (unskewed payloads)
a_payload_t seq_a [0:ARRAY_DIM-1];
b_payload_t seq_b [0:ARRAY_DIM-1];

read_sequencer rd_seq (
    .clk(clk),
    .rst_n(rst_n),

    .start(start),

    .busy(busy),
    .done(done),

    .M(M),
    .N(N),
    .K(K),

    .rd_en(rd_en),
    .rd_col_a(rd_col_a),
    .rd_row_b(rd_row_b),

    .rd_a(rd_a),
    .rd_b(rd_b),

    .out_a(seq_a),
    .out_b(seq_b)
);

skew_buffer skew_buf (
    .clk(clk),
    .rst_n(rst_n),

    .in_a(seq_a),
    .out_a(out_a),

    .in_b(seq_b),
    .out_b(out_b)
);

endmodule