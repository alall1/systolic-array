// Drain collector: turns pe_grid's drain stream into row writes of C.
// After each capture, drain_out shows grid row n-1, then one row further up per
// shift_en; rows < M_t are written at global row tile_m*n + r, lanes < N_t only.
// Data passes through untouched. Contract: docs/collector_contract.md
module collector
import pe_pkg::*;
#(
    parameter  int MAX_M   = 64,                                    // must equal tile_ctrl's
    parameter  int MAX_N   = 64,                                    // must equal tile_ctrl's
    localparam int TILES_M = (MAX_M + ARRAY_DIM - 1) / ARRAY_DIM,
    localparam int TILES_N = (MAX_N + ARRAY_DIM - 1) / ARRAY_DIM,
    localparam int TM_W    = (TILES_M > 1) ? $clog2(TILES_M) : 1,
    localparam int TN_W    = (TILES_N > 1) ? $clog2(TILES_N) : 1,
    localparam int ROW_W   = (MAX_M > 1) ? $clog2(MAX_M) : 1,      // global row of C
    localparam int COL_W   = (MAX_N > 1) ? $clog2(MAX_N) : 1       // global column of lane 0
)
(
    input  logic clk,
    input  logic rst_n,                                     // async, active low

    // from tile_ctrl
    input  logic capture,                                   // same net as pe_grid.capture
    input  logic shift_en,                                  // same net as pe_grid.shift_en
    input  logic [TM_W-1:0] tile_m,                         // sampled only with capture
    input  logic [TN_W-1:0] tile_n,
    input  logic [DIM_W-1:0] tile_rows,                     // <- tile_ctrl.feed_M (M_t)
    input  logic [DIM_W-1:0] tile_cols,                     // <- tile_ctrl.feed_N (N_t)

    // from pe_grid
    input  logic [ACC_WIDTH-1:0] drain_out [0:ARRAY_DIM-1],

    // result write port: one row of C per wr_en cycle
    output logic wr_en,
    output logic [ROW_W-1:0] wr_row,
    output logic [COL_W-1:0] wr_col,
    output logic [ARRAY_DIM-1:0] wr_lane_en,                // lane j written iff j < N_t
    output logic signed [ACC_WIDTH-1:0] wr_data [0:ARRAY_DIM-1]
);

// same parameter rules as tile_ctrl, which drives the tile inputs
generate
    if (ARRAY_DIM < 2) begin : g_dim_check
        $error("collector requires ARRAY_DIM >= 2");
    end
    if (MAX_M < ARRAY_DIM || MAX_N < ARRAY_DIM) begin : g_max_check
        $error("collector requires MAX_M >= ARRAY_DIM and MAX_N >= ARRAY_DIM");
    end
endgenerate

localparam int R_W = $clog2(ARRAY_DIM);                             // grid row index, 0..ARRAY_DIM-1
localparam logic [R_W-1:0] BOTTOM_ROW = R_W'(ARRAY_DIM - 1);        // first row on drain_out after capture

// no encoded state machine: every combination of these registers is benign
// (wr_en needs active AND fresh), so there is no illegal state to recover from
logic             active, active_n;         // a tile's drain is in progress
logic             fresh, fresh_n;           // the row on drain_out has not been offered yet
logic [R_W-1:0]   row, row_n;               // grid row now on drain_out
logic [TM_W-1:0]  tm, tm_n;                 // tile inputs latched at capture: tile_ctrl
logic [TN_W-1:0]  tn, tn_n;                 // moves them on before row 0 drains
logic [DIM_W-1:0] rows, rows_n;
logic [DIM_W-1:0] cols, cols_n;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        active <= 1'b0;
        fresh  <= 1'b0;
        row    <= '0;
        tm     <= '0;
        tn     <= '0;
        rows   <= '0;
        cols   <= '0;
    end else begin
        active <= active_n;
        fresh  <= fresh_n;
        row    <= row_n;
        tm     <= tm_n;
        tn     <= tn_n;
        rows   <= rows_n;
        cols   <= cols_n;
    end
end

always_comb begin
    // defaults: hold the drain position and latched tile, nothing new to offer
    active_n = active;
    fresh_n  = 1'b0;
    row_n    = row;
    tm_n     = tm;
    tn_n     = tn;
    rows_n   = rows;
    cols_n   = cols;

    if (capture) begin                                  // wins over shift_en, as in pe
        active_n = 1'b1;
        fresh_n  = 1'b1;
        row_n    = BOTTOM_ROW;
        tm_n     = tile_m;
        tn_n     = tile_n;
        rows_n   = tile_rows;
        cols_n   = tile_cols;
    end else if (active && shift_en && row != '0) begin // next row up reaches drain_out
        fresh_n  = 1'b1;
        row_n    = row - R_W'(1);
    end else if (active && fresh && row == '0) begin    // row 0 was offered: drain done
        active_n = 1'b0;
    end
end

// write port: rows >= M_t hold stale accumulators and are skipped; all-zero when idle
always_comb begin
    wr_en  = active && fresh && (DIM_W'(row) < rows);
    wr_row = wr_en ? ROW_W'(int'(tm) * ARRAY_DIM + int'(row)) : '0;
    wr_col = wr_en ? COL_W'(int'(tn) * ARRAY_DIM) : '0;
    for (int j = 0; j < ARRAY_DIM; j++) begin
        wr_lane_en[j] = wr_en && (j < int'(cols));
        wr_data[j]    = wr_en ? $signed(drain_out[j]) : '0;
    end
end

`ifndef SYNTHESIS
// ---------------------------------------------------------------------------
// Assertions: contract §6-7 properties local to this block (Verilator
// --assert). System properties (§7.6-7: each C element written once, last write
// in tile_ctrl's done cycle) are checked by the integration testbench. Disabled
// in reset, so async-reset behaviour is checked by the cocotb monitors.
// ---------------------------------------------------------------------------

// armed one edge after reset release (rst_n stays purely asynchronous)
logic sva_armed;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) sva_armed <= 1'b0;
    else        sva_armed <= 1'b1;
end

// checker-only bookkeeping: capture seen since reset, writes of the current tile
logic sva_captured;
int   sva_writes;
logic [ROW_W-1:0] sva_last_row;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sva_captured <= 1'b0;
        sva_writes   <= 0;
        sva_last_row <= '0;
    end else begin
        if (capture) sva_captured <= 1'b1;
        if (capture)    sva_writes <= 0;
        else if (wr_en) sva_writes <= sva_writes + 1;
        if (wr_en) sva_last_row <= wr_row;
    end
end

logic sva_port_zero, sva_data_through;
always_comb begin
    sva_port_zero    = (wr_row == '0) && (wr_col == '0) && (wr_lane_en == '0);
    sva_data_through = 1'b1;
    for (int j = 0; j < ARRAY_DIM; j++) begin
        sva_port_zero    = sva_port_zero && (wr_data[j] == '0);
        sva_data_through = sva_data_through && (wr_data[j] == $signed(drain_out[j]));
    end
end

// §7.1, 4: writes only after a capture; the idle port is all-zero
a_write_needs_capture: assert property (@(posedge clk) disable iff (!sva_armed)
    wr_en |-> sva_captured);
a_idle_port_is_zero: assert property (@(posedge clk) disable iff (!sva_armed)
    !wr_en |-> sva_port_zero);

// §7.3: every write is a tile row inside C, a thermometer lane mask, data passed through
a_lanes_are_thermometer: assert property (@(posedge clk) disable iff (!sva_armed)
    wr_en |-> (wr_lane_en != '0) && ((wr_lane_en & (wr_lane_en + 1'b1)) == '0));
a_write_inside_c: assert property (@(posedge clk) disable iff (!sva_armed)
    wr_en |-> (int'(wr_row) < MAX_M) && (int'(wr_col) % ARRAY_DIM == 0)
              && (int'(wr_col) + $countones(wr_lane_en) <= MAX_N));
a_data_passes_through: assert property (@(posedge clk) disable iff (!sva_armed)
    wr_en |-> sva_data_through);
a_write_lands_in_tile: assert property (@(posedge clk) disable iff (!sva_armed)
    wr_en |-> (int'(wr_row) == int'(tm) * ARRAY_DIM + int'(row))
              && (int'(wr_col) == int'(tn) * ARRAY_DIM)
              && ($countones(wr_lane_en) == int'(cols)));

// §7.2: rows of one tile are written bottom-up, one apart, M_t of them per full drain
a_rows_bottom_up: assert property (@(posedge clk) disable iff (!sva_armed)
    (wr_en && sva_writes != 0) |-> int'(wr_row) == int'(sva_last_row) - 1);
a_full_drain_writes_m_t_rows: assert property (@(posedge clk) disable iff (!sva_armed)
    ($past(active) && !active && !$past(capture)) |-> sva_writes == int'(rows));

// §7.5 + §4: tile latched only on capture; capture restarts at the bottom row
a_tile_latched_only_on_capture: assert property (@(posedge clk) disable iff (!sva_armed)
    !$stable({tm, tn, rows, cols}) |-> $past(capture));
a_capture_restarts_drain: assert property (@(posedge clk) disable iff (!sva_armed)
    capture |=> active && fresh && (row == BOTTOM_ROW));
a_row_offered_once: assert property (@(posedge clk) disable iff (!sva_armed)
    fresh |-> $past(capture) || ($past(active) && $past(shift_en) && $past(row) != '0));

// §6 input assumption from tile_ctrl: the captured tile lies inside C
a_captured_tile_inside_c: assert property (@(posedge clk) disable iff (!sva_armed)
    capture |-> (tile_rows != '0) && (int'(tile_rows) <= ARRAY_DIM)
                && (tile_cols != '0) && (int'(tile_cols) <= ARRAY_DIM)
                && (int'(tile_m) * ARRAY_DIM + int'(tile_rows) <= MAX_M)
                && (int'(tile_n) * ARRAY_DIM + int'(tile_cols) <= MAX_N));

// §6 out-of-contract inputs: v1 tile_ctrl never produces them. The standalone
// testbench drives them on purpose to check the defined behaviour, and builds
// with COLLECTOR_TB_OUT_OF_CONTRACT; every system build keeps them live.
`ifndef COLLECTOR_TB_OUT_OF_CONTRACT
a_no_capture_mid_drain: assert property (@(posedge clk) disable iff (!sva_armed)
    capture |-> !active);
a_no_capture_with_shift: assert property (@(posedge clk) disable iff (!sva_armed)
    !(capture && shift_en));
a_shift_only_mid_drain: assert property (@(posedge clk) disable iff (!sva_armed)
    (shift_en && !capture) |-> active && (row != '0));
`endif
`endif

endmodule
