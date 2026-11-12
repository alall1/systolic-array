// Integration harness (testbench only, not product RTL): tile_ctrl drives the
// unchanged feeder + pe_grid, collector turns the grid's drain into row writes
// of C, and three lane_ram instances hold A, B and C in the tile-word layout
// (docs/memory_contract.md). All address logic is concatenation. The Python
// testbench plays the host: it loads A/B words before a job and reads C after.
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
    localparam int KI_W    = (MAX_K > 1) ? $clog2(MAX_K) : 1,      // k = 0 .. MAX_K-1
    localparam int A_AW    = TM_W + KI_W,                           // word {tile_m, k}
    localparam int B_AW    = TN_W + KI_W,                           // word {tile_n, k}
    localparam int C_AW    = ROW_W + TN_W                           // word {row, tile_n}
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

    // host: load A and B tile words (only while no job runs)
    input  logic a_we,
    input  logic [A_AW-1:0] a_waddr,
    input  logic [DATA_WIDTH-1:0] a_wdata [0:ARRAY_DIM-1],
    input  logic b_we,
    input  logic [B_AW-1:0] b_waddr,
    input  logic [DATA_WIDTH-1:0] b_wdata [0:ARRAY_DIM-1],

    // host: read C words (from the cycle after done, memory_contract.md §6)
    input  logic c_re,
    input  logic [C_AW-1:0] c_raddr,
    output logic [ACC_WIDTH-1:0] c_rdata [0:ARRAY_DIM-1],

    // observation only (system-level checks)
    output logic feed_start,
    output logic feed_done,
    output logic feeder_busy
);

logic [DIM_W-1:0] feed_M, feed_N;
logic [K_W-1:0] feed_K;
logic capture, shift_en;
logic [TM_W-1:0] tile_m;
logic [TN_W-1:0] tile_n;
logic rd_en;
// K_W holds K = MAX_K; k itself is < MAX_K, so the RAM address takes only KI_W bits (memory_contract §5)
/* verilator lint_off UNUSEDSIGNAL */
logic [K_W-1:0] rd_col_a, rd_row_b;
/* verilator lint_on UNUSEDSIGNAL */
logic signed [DATA_WIDTH-1:0] rd_a [0:ARRAY_DIM-1];
logic signed [DATA_WIDTH-1:0] rd_b [0:ARRAY_DIM-1];
logic [ACC_WIDTH-1:0] drain_out [0:ARRAY_DIM-1];
a_payload_t grid_a [0:ARRAY_DIM-1];
b_payload_t grid_b [0:ARRAY_DIM-1];
/* verilator lint_off UNUSEDSIGNAL */
logic signed [ACC_WIDTH-1:0] grid_acc [0:ARRAY_DIM-1][0:ARRAY_DIM-1];   // unused: results leave via drain
/* verilator lint_on UNUSEDSIGNAL */

logic wr_en;
logic [ROW_W-1:0] wr_row;
logic [TN_W-1:0] wr_tile_n;
logic [ARRAY_DIM-1:0] wr_lane_en;
logic signed [ACC_WIDTH-1:0] wr_data [0:ARRAY_DIM-1];

// raw-bit views for lane_ram (same widths; signedness is only an interpretation)
logic [DATA_WIDTH-1:0] mem_a_rdata [0:ARRAY_DIM-1];
logic [DATA_WIDTH-1:0] mem_b_rdata [0:ARRAY_DIM-1];
logic [ACC_WIDTH-1:0]  mem_c_wdata [0:ARRAY_DIM-1];
always_comb begin
    for (int l = 0; l < ARRAY_DIM; l++) begin
        rd_a[l]        = $signed(mem_a_rdata[l]);
        rd_b[l]        = $signed(mem_b_rdata[l]);
        mem_c_wdata[l] = wr_data[l];
    end
end

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
    .wr_tile_n(wr_tile_n),
    .wr_lane_en(wr_lane_en),
    .wr_data(wr_data)
);

// A: word {tile_m, k}, lane l = A[tile_m*n + l][k] (A stored transposed)
lane_ram #(.LANES(ARRAY_DIM), .WIDTH(DATA_WIDTH), .ADDR_W(A_AW)) u_mem_a (
    .clk(clk),
    .we(a_we),
    .waddr(a_waddr),
    .wlane_en('1),
    .wdata(a_wdata),
    .re(rd_en),
    .raddr({tile_m, rd_col_a[KI_W-1:0]}),
    .rdata(mem_a_rdata)
);

// B: word {tile_n, k}, lane l = B[k][tile_n*n + l]
lane_ram #(.LANES(ARRAY_DIM), .WIDTH(DATA_WIDTH), .ADDR_W(B_AW)) u_mem_b (
    .clk(clk),
    .we(b_we),
    .waddr(b_waddr),
    .wlane_en('1),
    .wdata(b_wdata),
    .re(rd_en),
    .raddr({tile_n, rd_row_b[KI_W-1:0]}),
    .rdata(mem_b_rdata)
);

// C: word {row, tile_n}, lane l = C[row][tile_n*n + l]
lane_ram #(.LANES(ARRAY_DIM), .WIDTH(ACC_WIDTH), .ADDR_W(C_AW)) u_mem_c (
    .clk(clk),
    .we(wr_en),
    .waddr({wr_row, wr_tile_n}),
    .wlane_en(wr_lane_en),
    .wdata(mem_c_wdata),
    .re(c_re),
    .raddr(c_raddr),
    .rdata(c_rdata)
);

endmodule
