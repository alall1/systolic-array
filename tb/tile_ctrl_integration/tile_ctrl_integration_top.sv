// Integration harness (testbench only, not product RTL): tile_ctrl drives the
// unchanged feeder + pe_grid, and collector turns the grid's drain into row
// writes of C. The operand buffer and the result memory live in the Python
// testbench; tile_m/tile_n tell the buffer which operand tile to serve.
module tile_ctrl_integration_top
import pe_pkg::*;
#(
    parameter  int MAX_M   = 64,
    parameter  int MAX_N   = 64,
    localparam int M_W     = $clog2(MAX_M + 1),
    localparam int N_W     = $clog2(MAX_N + 1),
    localparam int TILES_M = (MAX_M + ARRAY_DIM - 1) / ARRAY_DIM,
    localparam int TILES_N = (MAX_N + ARRAY_DIM - 1) / ARRAY_DIM,
    localparam int TM_W    = (TILES_M > 1) ? $clog2(TILES_M) : 1,
    localparam int TN_W    = (TILES_N > 1) ? $clog2(TILES_N) : 1,
    localparam int ROW_W   = (MAX_M > 1) ? $clog2(MAX_M) : 1,
    localparam int COL_W   = (MAX_N > 1) ? $clog2(MAX_N) : 1
)
(
    input  logic clk,
    input  logic rst_n,

    // command interface (straight to tile_ctrl)
    input  logic start,
    input  logic [M_W-1:0] M,
    input  logic [N_W-1:0] N,
    input  logic [K_W-1:0] K,
    output logic busy,
    output logic done,
    output logic error,

    // operand buffer port (feeder read port + tile identifiers)
    output logic rd_en,
    output logic [K_W-1:0] rd_col_a,
    output logic [K_W-1:0] rd_row_b,
    input  logic signed [DATA_WIDTH-1:0] rd_a [0:ARRAY_DIM-1],
    input  logic signed [DATA_WIDTH-1:0] rd_b [0:ARRAY_DIM-1],
    output logic [TM_W-1:0] tile_m,
    output logic [TN_W-1:0] tile_n,

    // result memory write port (from collector)
    output logic wr_en,
    output logic [ROW_W-1:0] wr_row,
    output logic [COL_W-1:0] wr_col,
    output logic [ARRAY_DIM-1:0] wr_lane_en,
    output logic signed [ACC_WIDTH-1:0] wr_data [0:ARRAY_DIM-1],

    // observation only (system-level checks)
    output logic feed_start,
    output logic feed_done,
    output logic feeder_busy
);

logic [DIM_W-1:0] feed_M, feed_N;
logic [K_W-1:0] feed_K;
logic capture, shift_en;
logic [ACC_WIDTH-1:0] drain_out [0:ARRAY_DIM-1];
a_payload_t grid_a [0:ARRAY_DIM-1];
b_payload_t grid_b [0:ARRAY_DIM-1];
logic signed [ACC_WIDTH-1:0] grid_acc [0:ARRAY_DIM-1][0:ARRAY_DIM-1];   // unused: results leave via drain

tile_ctrl #(.MAX_M(MAX_M), .MAX_N(MAX_N)) u_ctrl (
    .clk(clk),
    .rst_n(rst_n),
    .start(start),
    .M(M),
    .N(N),
    .K(K),
    .busy(busy),
    .done(done),
    .error(error),
    .feed_start(feed_start),
    .feed_M(feed_M),
    .feed_N(feed_N),
    .feed_K(feed_K),
    .feed_done(feed_done),
    .capture(capture),
    .shift_en(shift_en),
    .tile_m(tile_m),
    .tile_n(tile_n)
);

feeder u_feeder (
    .clk(clk),
    .rst_n(rst_n),
    .start(feed_start),
    .M(feed_M),
    .N(feed_N),
    .K(feed_K),
    .busy(feeder_busy),
    .done(feed_done),
    .rd_en(rd_en),
    .rd_col_a(rd_col_a),
    .rd_row_b(rd_row_b),
    .rd_a(rd_a),
    .rd_b(rd_b),
    .out_a(grid_a),
    .out_b(grid_b)
);

pe_grid u_grid (
    .clk(clk),
    .rst_n(rst_n),
    .a_in(grid_a),
    .b_in(grid_b),
    .capture(capture),
    .shift_en(shift_en),
    .out(grid_acc),
    .drain_out(drain_out)
);

collector #(.MAX_M(MAX_M), .MAX_N(MAX_N)) u_collector (
    .clk(clk),
    .rst_n(rst_n),
    .capture(capture),
    .shift_en(shift_en),
    .tile_m(tile_m),
    .tile_n(tile_n),
    .tile_rows(feed_M),
    .tile_cols(feed_N),
    .drain_out(drain_out),
    .wr_en(wr_en),
    .wr_row(wr_row),
    .wr_col(wr_col),
    .wr_lane_en(wr_lane_en),
    .wr_data(wr_data)
);

endmodule
