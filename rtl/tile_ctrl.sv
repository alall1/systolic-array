// Control FSM: tiles one C[MxN] = A[MxK] * B[KxN] job over the ARRAY_DIM x ARRAY_DIM grid.
// Control signals only; no data, no addresses, no valid/first (read_sequencer owns those).
// Contract: docs/fsm_contract.md
//
// per tile:  ISSUE 1 | FEED K+1 (until feed_done) | FLUSH M_t+N_t-1 | CAPTURE 1 | DRAIN n-1
//            -> K + M_t + N_t + n + 1 cycles; tiles row-major (tile_n inner)
module tile_ctrl
import pe_pkg::*;
#(
    parameter  int MAX_M   = 64,                                    // largest legal job M
    parameter  int MAX_N   = 64,                                    // largest legal job N
    localparam int M_W     = $clog2(MAX_M + 1),
    localparam int N_W     = $clog2(MAX_N + 1),
    localparam int TILES_M = (MAX_M + ARRAY_DIM - 1) / ARRAY_DIM,   // max tile rows
    localparam int TILES_N = (MAX_N + ARRAY_DIM - 1) / ARRAY_DIM,   // max tile cols
    localparam int TM_W    = (TILES_M > 1) ? $clog2(TILES_M) : 1,
    localparam int TN_W    = (TILES_N > 1) ? $clog2(TILES_N) : 1
)
(
    input  logic clk,
    input  logic rst_n,                     // async, active low

    // command interface
    input  logic start,                     // sampled only while busy is low
    input  logic [M_W-1:0] M,               // sampled with an accepted start
    input  logic [N_W-1:0] N,
    input  logic [K_W-1:0] K,
    output logic busy,
    output logic done,                      // one cycle per accepted command
    output logic error,                     // only with done: command rejected

    // toward feeder
    output logic feed_start,                // -> feeder.start
    output logic [DIM_W-1:0] feed_M,        // -> feeder.M (tile rows M_t)
    output logic [DIM_W-1:0] feed_N,        // -> feeder.N (tile cols N_t)
    output logic [K_W-1:0] feed_K,          // -> feeder.K
    input  logic feed_done,                 // <- feeder.done

    // toward pe_grid
    output logic capture,                   // -> pe_grid.capture
    output logic shift_en,                  // -> pe_grid.shift_en

    // toward operand buffer: which output tile is in flight
    output logic [TM_W-1:0] tile_m,
    output logic [TN_W-1:0] tile_n
);

// DRAIN is ARRAY_DIM-1 cycles, so a 1x1 grid has no legal schedule; job limits
// must hold at least one full tile so ARRAY_DIM fits the M/N widths
generate
    if (ARRAY_DIM < 2) begin : g_dim_check
        $error("tile_ctrl requires ARRAY_DIM >= 2");
    end
    if (MAX_M < ARRAY_DIM || MAX_N < ARRAY_DIM) begin : g_max_check
        $error("tile_ctrl requires MAX_M >= ARRAY_DIM and MAX_N >= ARRAY_DIM");
    end
endgenerate

localparam int CNT_W = $clog2(2 * ARRAY_DIM);                       // holds the longest phase, FLUSH <= 2*ARRAY_DIM-1
localparam logic [CNT_W-1:0] DRAIN_LEN = CNT_W'(ARRAY_DIM - 1);     // shifts to bring row 0 to drain_out

typedef enum logic [2:0] {IDLE, ISSUE, FEED, FLUSH, CAPTURE, DRAIN, DONE, REJECT} state_t;
state_t state, state_n;

logic [CNT_W-1:0] cnt, cnt_n;               // cycles left in FLUSH / DRAIN, including this one
logic [TM_W-1:0]  tile_m_n;
logic [TN_W-1:0]  tile_n_n;
logic [M_W-1:0]   m_rem, m_rem_n;           // rows remaining, including the current tile row
logic [N_W-1:0]   n_rem, n_rem_n;           // cols remaining, including the current tile col
logic [N_W-1:0]   N_h, N_h_n;               // job N, reloads n_rem at each new tile row
logic [K_W-1:0]   K_h, K_h_n;

// current tile dims: a compare, not a multiply
logic [DIM_W-1:0] M_t, N_t;
assign M_t = (m_rem > M_W'(ARRAY_DIM)) ? DIM_W'(ARRAY_DIM) : DIM_W'(m_rem);
assign N_t = (n_rem > N_W'(ARRAY_DIM)) ? DIM_W'(ARRAY_DIM) : DIM_W'(n_rem);

// when a MAX_* is 2^W-1 its upper-bound compare is constant true (every representable
// value is legal); it stays so every other parameterization is still range-checked
logic cmd_legal;
/* verilator lint_off CMPCONST */
assign cmd_legal = (M != '0) && (M <= M_W'(MAX_M)) && (N != '0) && (N <= N_W'(MAX_N)) && (K != '0) && (K <= K_W'(MAX_K));
/* verilator lint_on CMPCONST */

// tile outputs come straight from registers: they only move on load and on DRAIN -> ISSUE
assign feed_M = M_t;
assign feed_N = N_t;
assign feed_K = K_h;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state  <= IDLE;
        cnt    <= '0;
        tile_m <= '0;
        tile_n <= '0;
        m_rem  <= '0;
        n_rem  <= '0;
        N_h    <= '0;
        K_h    <= '0;
    end else begin
        state  <= state_n;
        cnt    <= cnt_n;
        tile_m <= tile_m_n;
        tile_n <= tile_n_n;
        m_rem  <= m_rem_n;
        n_rem  <= n_rem_n;
        N_h    <= N_h_n;
        K_h    <= K_h_n;
    end
end

always_comb begin
    // defaults: hold every register, every control output low
    state_n    = state;
    cnt_n      = cnt;
    tile_m_n   = tile_m;
    tile_n_n   = tile_n;
    m_rem_n    = m_rem;
    n_rem_n    = n_rem;
    N_h_n      = N_h;
    K_h_n      = K_h;

    busy       = 1'b0;
    done       = 1'b0;
    error      = 1'b0;
    feed_start = 1'b0;
    capture    = 1'b0;
    shift_en   = 1'b0;

    case (state)
        // idle-like states: busy low, so start is sampled
        IDLE, DONE, REJECT: begin
            done  = (state != IDLE);
            error = (state == REJECT);
            if (!start) begin
                state_n = IDLE;
            end else if (cmd_legal) begin
                state_n  = ISSUE;
                tile_m_n = '0;
                tile_n_n = '0;
                m_rem_n  = M;
                n_rem_n  = N;
                N_h_n    = N;
                K_h_n    = K;
            end else begin
                state_n = REJECT;
            end
        end

        ISSUE: begin
            busy       = 1'b1;
            feed_start = 1'b1;
            state_n    = FEED;
        end

        // feeder streams K payloads; read_sequencer owns the k loop
        FEED: begin
            busy = 1'b1;
            if (feed_done) begin
                state_n = FLUSH;
                cnt_n   = CNT_W'(M_t) + CNT_W'(N_t) - CNT_W'(1);
            end
        end

        // last payload crosses skew + grid; PE(M_t-1, N_t-1) is final in CAPTURE
        FLUSH: begin
            busy = 1'b1;
            if (cnt == CNT_W'(1)) state_n = CAPTURE;
            else                  cnt_n   = cnt - CNT_W'(1);
        end

        CAPTURE: begin
            busy    = 1'b1;
            capture = 1'b1;
            state_n = DRAIN;
            cnt_n   = DRAIN_LEN;
        end

        DRAIN: begin
            busy     = 1'b1;
            shift_en = 1'b1;
            if (cnt != CNT_W'(1)) begin
                cnt_n = cnt - CNT_W'(1);
            end else if (n_rem > N_W'(ARRAY_DIM)) begin         // next tile in this row
                state_n  = ISSUE;
                tile_n_n = tile_n + TN_W'(1);
                n_rem_n  = n_rem - N_W'(ARRAY_DIM);
            end else if (m_rem > M_W'(ARRAY_DIM)) begin         // first tile of the next row
                state_n  = ISSUE;
                tile_m_n = tile_m + TM_W'(1);
                m_rem_n  = m_rem - M_W'(ARRAY_DIM);
                tile_n_n = '0;
                n_rem_n  = N_h;
            end else begin
                state_n  = DONE;
            end
        end

        // recovery for an encoding outside the enum (e.g. after a re-encode + upset)
        default: state_n = IDLE;
    endcase
end

`ifndef SYNTHESIS
// ---------------------------------------------------------------------------
// Assertions: the contract §9 invariants that are local to this block
// (simulate with Verilator --assert). §9.11 (feed_done only in FEED) is a
// system property, checked by the integration testbench instead. Async-reset
// behaviour (outputs idle while rst_n is low) cannot be seen by assertions that
// are disabled in reset; the cocotb ProtocolMonitor and Env.reset check it.
// ---------------------------------------------------------------------------

// armed one edge after reset release, so $past never looks into reset; async-reset
// flop keeps rst_n purely asynchronous (disable iff (!sva_armed) would sample it synchronously)
logic sva_armed;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) sva_armed <= 1'b0;
    else        sva_armed <= 1'b1;
end

// cycles since this tile's ISSUE (0 in ISSUE); checker-only state
int sva_age;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)                 sva_age <= 0;
    else if (state_n == ISSUE)  sva_age <= 0;
    else                        sva_age <= sva_age + 1;
end

int sva_tile_len;               // contract: T_tile = K + M_t + N_t + n + 1
assign sva_tile_len = int'(K_h) + int'(M_t) + int'(N_t) + ARRAY_DIM + 1;

function automatic logic legal_step(state_t from, state_t to);
    case (from)
        IDLE, DONE, REJECT: return to inside {IDLE, ISSUE, REJECT};
        ISSUE:              return to == FEED;
        FEED:               return to inside {FEED, FLUSH};
        FLUSH:              return to inside {FLUSH, CAPTURE};
        CAPTURE:            return to == DRAIN;
        DRAIN:              return to inside {DRAIN, ISSUE, DONE};
        default:            return 1'b0;
    endcase
endfunction

// §9.1-3: command handshake
a_busy_means_working: assert property (@(posedge clk) disable iff (!sva_armed)
    busy == (state inside {ISSUE, FEED, FLUSH, CAPTURE, DRAIN}));
a_done_never_busy: assert property (@(posedge clk) disable iff (!sva_armed)
    !(done && busy));
a_error_only_with_done: assert property (@(posedge clk) disable iff (!sva_armed)
    error |-> done);
a_one_done_per_command: assert property (@(posedge clk) disable iff (!sva_armed)
    (done && $past(done)) |-> $past(start));
a_start_accepted_iff_idle: assert property (@(posedge clk) disable iff (!sva_armed)
    ($past(state) inside {IDLE, DONE, REJECT} && $past(start))
        |-> state == ($past(cmd_legal) ? ISSUE : REJECT));
a_legal_transition: assert property (@(posedge clk) disable iff (!sva_armed)
    legal_step($past(state), state));

// §9.4: control pulses only while working, never capture with shift
a_pulses_only_while_busy: assert property (@(posedge clk) disable iff (!sva_armed)
    (feed_start || capture || shift_en) |-> busy);
a_capture_excludes_shift: assert property (@(posedge clk) disable iff (!sva_armed)
    !(capture && shift_en));

// §9.6: per-tile timing
a_capture_when_last_pe_final: assert property (@(posedge clk) disable iff (!sva_armed)
    capture |-> sva_age == int'(K_h) + int'(M_t) + int'(N_t) + 1);
a_drain_is_n_minus_1_shifts: assert property (@(posedge clk) disable iff (!sva_armed)
    capture |=> shift_en [*ARRAY_DIM-1] ##1 !shift_en);
a_tile_length: assert property (@(posedge clk) disable iff (!sva_armed)
    (state == DRAIN && state_n != DRAIN) |-> sva_age == sva_tile_len - 1);

// §9.7-8: tile indices and dims in range while working
a_tile_index_in_range: assert property (@(posedge clk) disable iff (!sva_armed)
    busy |-> (int'(tile_m) < TILES_M) && (int'(tile_n) < TILES_N) && (m_rem != '0) && (n_rem != '0));
a_tile_dims_in_range: assert property (@(posedge clk) disable iff (!sva_armed)
    busy |-> (feed_M != '0) && (int'(feed_M) <= ARRAY_DIM) && (feed_N != '0) && (int'(feed_N) <= ARRAY_DIM)
             && (feed_K != '0) && (int'(feed_K) <= MAX_K));

// §9.7: tiles visited row-major, each once: next column, or first column of the next row
a_tiles_row_major: assert property (@(posedge clk) disable iff (!sva_armed)
    ($past(state) == DRAIN && state == ISSUE)
        |-> (tile_m == $past(tile_m) && tile_n == $past(tile_n) + TN_W'(1))
         || (tile_m == $past(tile_m) + TM_W'(1) && tile_n == '0));
a_new_row_has_all_cols: assert property (@(posedge clk) disable iff (!sva_armed)
    (state == ISSUE && tile_n == '0) |-> n_rem == N_h);

// §9.9: tile outputs move only on load and on DRAIN -> ISSUE
a_tile_outputs_held: assert property (@(posedge clk) disable iff (!sva_armed)
    !$stable({feed_M, feed_N, feed_K, tile_m, tile_n})
        |-> state == ISSUE && $past(state) inside {IDLE, DONE, REJECT, DRAIN});
`endif

endmodule
