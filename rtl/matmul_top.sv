// Top level: the whole matmul accelerator behind one host bus. Pure wiring, no
// logic of its own: host_if (bus decode, registers, status) commands tile_ctrl,
// which tiles C over the grid; feeder streams A/B tile words from two lane_rams
// into pe_grid; collector writes C rows into the third lane_ram, which the host
// reads back through host_if. Address logic is concatenation only (tile-word
// layout). One MAX_M / MAX_N reaches every block that sizes tile indices.
// Contracts: docs/top_contract.md (host side), docs/memory_contract.md §5 (wiring)
module matmul_top
import pe_pkg::*;
#(
    parameter  int MAX_M   = 64,                                    // largest legal job M
    parameter  int MAX_N   = 64,                                    // largest legal job N
    localparam int M_W     = $clog2(MAX_M + 1),
    localparam int N_W     = $clog2(MAX_N + 1),
    localparam int TILES_M = (MAX_M + ARRAY_DIM - 1) / ARRAY_DIM,
    localparam int TILES_N = (MAX_N + ARRAY_DIM - 1) / ARRAY_DIM,
    localparam int TM_W    = (TILES_M > 1) ? $clog2(TILES_M) : 1,
    localparam int TN_W    = (TILES_N > 1) ? $clog2(TILES_N) : 1,
    localparam int ROW_W   = (MAX_M > 1) ? $clog2(MAX_M) : 1,
    localparam int KI_W    = (MAX_K > 1) ? $clog2(MAX_K) : 1,      // k = 0 .. MAX_K-1
    localparam int A_AW    = TM_W + KI_W,                           // A word {tile_m, k}
    localparam int B_AW    = TN_W + KI_W,                           // B word {tile_n, k}
    localparam int C_AW    = ROW_W + TN_W,                          // C word {row, tile_n}
    localparam int LANE_W  = (ARRAY_DIM > 1) ? $clog2(ARRAY_DIM) : 1,
    localparam int WORD_W  = (A_AW > B_AW) ? ((A_AW > C_AW) ? A_AW : C_AW) : ((B_AW > C_AW) ? B_AW : C_AW),
    localparam int HOST_AW = 2 + WORD_W + LANE_W,                   // {region, word, lane}
    localparam int HOST_W  = ACC_WIDTH
)
(
    input  logic clk,
    input  logic rst_n,                                     // async, active low

    // host bus: SRAM-style, always ready, read data the cycle after the read edge
    input  logic host_we,
    input  logic host_re,
    input  logic [HOST_AW-1:0] host_addr,
    input  logic [HOST_W-1:0] host_wdata,
    output logic [HOST_W-1:0] host_rdata
);

// host_if <-> tile_ctrl
logic start, busy, done, error;
logic [M_W-1:0] cmd_M;
logic [N_W-1:0] cmd_N;
logic [K_W-1:0] cmd_K;

// tile_ctrl -> feeder / pe_grid / collector / operand RAMs
logic feed_start, feed_done;
// feeder.busy is intentionally unconsumed: the feeder is idle at every ISSUE (fsm_contract §4.2)
/* verilator lint_off UNUSEDSIGNAL */
logic feeder_busy;
/* verilator lint_on UNUSEDSIGNAL */
logic [DIM_W-1:0] feed_M, feed_N;
logic [K_W-1:0] feed_K;
logic capture, shift_en;
logic [TM_W-1:0] tile_m;                    // tile being fed: operand RAM addresses
logic [TN_W-1:0] tile_n;
logic [TM_W-1:0] cap_tile_m;                // tile being captured: collector (fsm_contract §4.5)
logic [TN_W-1:0] cap_tile_n;
logic [DIM_W-1:0] cap_M, cap_N;

// feeder <-> operand RAMs (rd_col_a / rd_row_b are K_W wide to carry K = MAX_K;
// k itself is < MAX_K, so the RAM address takes only KI_W bits: memory_contract §5)
logic rd_en;
/* verilator lint_off UNUSEDSIGNAL */
logic [K_W-1:0] rd_col_a, rd_row_b;
/* verilator lint_on UNUSEDSIGNAL */
logic signed [DATA_WIDTH-1:0] rd_a [0:ARRAY_DIM-1];
logic signed [DATA_WIDTH-1:0] rd_b [0:ARRAY_DIM-1];

// feeder -> pe_grid -> collector
a_payload_t grid_a [0:ARRAY_DIM-1];
b_payload_t grid_b [0:ARRAY_DIM-1];
logic [ACC_WIDTH-1:0] drain_out [0:ARRAY_DIM-1];
/* verilator lint_off UNUSEDSIGNAL */
logic signed [ACC_WIDTH-1:0] grid_acc [0:ARRAY_DIM-1][0:ARRAY_DIM-1];   // unused: results leave via drain
/* verilator lint_on UNUSEDSIGNAL */

// collector -> C RAM
logic wr_en;
logic [ROW_W-1:0] wr_row;
logic [TN_W-1:0] wr_tile_n;
logic [ARRAY_DIM-1:0] wr_lane_en;
logic signed [ACC_WIDTH-1:0] wr_data [0:ARRAY_DIM-1];

// host_if <-> RAMs
logic a_we, b_we, c_re;
logic [A_AW-1:0] a_waddr;
logic [B_AW-1:0] b_waddr;
logic [C_AW-1:0] c_raddr;
logic [ARRAY_DIM-1:0] a_wlane_en, b_wlane_en;
logic [DATA_WIDTH-1:0] a_wdata [0:ARRAY_DIM-1];
logic [DATA_WIDTH-1:0] b_wdata [0:ARRAY_DIM-1];
logic [ACC_WIDTH-1:0] c_rdata [0:ARRAY_DIM-1];

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

host_if #(.MAX_M(MAX_M), .MAX_N(MAX_N)) u_host_if (
    .clk(clk),
    .rst_n(rst_n),
    .host_we(host_we),
    .host_re(host_re),
    .host_addr(host_addr),
    .host_wdata(host_wdata),
    .host_rdata(host_rdata),
    .start(start),
    .M(cmd_M),
    .N(cmd_N),
    .K(cmd_K),
    .busy(busy),
    .done(done),
    .error(error),
    .a_we(a_we),
    .a_waddr(a_waddr),
    .a_wlane_en(a_wlane_en),
    .a_wdata(a_wdata),
    .b_we(b_we),
    .b_waddr(b_waddr),
    .b_wlane_en(b_wlane_en),
    .b_wdata(b_wdata),
    .c_re(c_re),
    .c_raddr(c_raddr),
    .c_rdata(c_rdata)
);

tile_ctrl #(.MAX_M(MAX_M), .MAX_N(MAX_N)) u_ctrl (
    .clk(clk),
    .rst_n(rst_n),
    .start(start),
    .M(cmd_M),
    .N(cmd_N),
    .K(cmd_K),
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
    .tile_n(tile_n),
    .cap_tile_m(cap_tile_m),
    .cap_tile_n(cap_tile_n),
    .cap_M(cap_M),
    .cap_N(cap_N)
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
    .tile_m(cap_tile_m),
    .tile_n(cap_tile_n),
    .tile_rows(cap_M),
    .tile_cols(cap_N),
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
    .wlane_en(a_wlane_en),
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
    .wlane_en(b_wlane_en),
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
