// Control FSM v2: tiles one C[MxN] = A[MxK] * B[KxN] job over the ARRAY_DIM x ARRAY_DIM grid,
// overlapping each tile's capture/drain with the next tile's feed.
// Control signals only; no data, no addresses, no valid/first (read_sequencer owns those).
// Contract: docs/fsm_contract.md (v2)
//
// front FSM : IDLE | ISSUE 1 | FEED K+1 (until feed_done) | WAIT (schedule guard) | ... | TAIL | DONE
// back      : one-tile capture slot -> capture at K+M_t+N_t+1 after the tile's ISSUE,
//             then n-1 shift_en cycles; captures stay >= n+1 apart
// handoff   : the edge that ends a tile's feed_done cycle moves it into the capture
//             slot (cap_*) and advances the feed side to the next tile
// next ISSUE: s_{i+1} = s_i + max(K+2, M_t+N_t+max(K-2,0), n+1+(M_t+N_t)-(M_t'+N_t'))
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

    // toward feeder: the tile being fed
    output logic feed_start,                // -> feeder.start
    output logic [DIM_W-1:0] feed_M,        // -> feeder.M (tile rows M_t)
    output logic [DIM_W-1:0] feed_N,        // -> feeder.N (tile cols N_t)
    output logic [K_W-1:0] feed_K,          // -> feeder.K
    input  logic feed_done,                 // <- feeder.done

    // toward pe_grid (and collector)
    output logic capture,                   // -> pe_grid.capture
    output logic shift_en,                  // -> pe_grid.shift_en

    // toward operand buffer: the tile being fed
    output logic [TM_W-1:0] tile_m,
    output logic [TN_W-1:0] tile_n,

    // toward collector: the tile in the capture slot (valid at every capture)
    output logic [TM_W-1:0] cap_tile_m,
    output logic [TN_W-1:0] cap_tile_n,
    output logic [DIM_W-1:0] cap_M,
    output logic [DIM_W-1:0] cap_N
);

// the drain is ARRAY_DIM-1 shifts, so a 1x1 grid has no legal schedule; job limits
// must hold at least one full tile so ARRAY_DIM fits the M/N widths
generate
    if (ARRAY_DIM < 2) begin : g_dim_check
        $error("tile_ctrl requires ARRAY_DIM >= 2");
    end
    if (MAX_M < ARRAY_DIM || MAX_N < ARRAY_DIM) begin : g_max_check
        $error("tile_ctrl requires MAX_M >= ARRAY_DIM and MAX_N >= ARRAY_DIM");
    end
endgenerate

localparam int CNT_W = $clog2(3 * ARRAY_DIM + 1);                   // holds the largest count, M_t+N_t+n <= 3n
localparam logic [CNT_W-1:0] DRAIN_LEN = CNT_W'(ARRAY_DIM - 1);     // shifts to bring row 0 to drain_out

typedef enum logic [2:0] {IDLE, ISSUE, FEED, WAIT, TAIL, DONE, REJECT} state_t;
state_t state, state_n;

// front: the tile being fed
logic [TM_W-1:0]  tile_m_n;
logic [TN_W-1:0]  tile_n_n;
logic [M_W-1:0]   m_rem, m_rem_n;           // rows remaining, including the current tile row
logic [N_W-1:0]   n_rem, n_rem_n;           // cols remaining, including the current tile col
logic [N_W-1:0]   N_h, N_h_n;               // job N, reloads n_rem at each new tile row
logic [K_W-1:0]   K_h, K_h_n;

// back: capture slot, drain, capture spacing
logic [TM_W-1:0]  cap_tile_m_n;
logic [TN_W-1:0]  cap_tile_n_n;
logic [DIM_W-1:0] cap_M_n, cap_N_n;
logic             slot, slot_n;             // a tile waits in the capture slot
logic [CNT_W-1:0] slot_cnt, slot_cnt_n;     // cycles until its capture (0 = capture now)
logic [CNT_W-1:0] drain, drain_n;           // shift_en cycles left, including this one
logic [CNT_W-1:0] gap, gap_n;               // cycles until the next capture is allowed

// tile dims: a compare, not a multiply. Current tile, and the tile after this edge
logic [DIM_W-1:0] M_t, N_t, M_t_nx, N_t_nx;
assign M_t    = (m_rem   > M_W'(ARRAY_DIM)) ? DIM_W'(ARRAY_DIM) : DIM_W'(m_rem);
assign N_t    = (n_rem   > N_W'(ARRAY_DIM)) ? DIM_W'(ARRAY_DIM) : DIM_W'(n_rem);
assign M_t_nx = (m_rem_n > M_W'(ARRAY_DIM)) ? DIM_W'(ARRAY_DIM) : DIM_W'(m_rem_n);
assign N_t_nx = (n_rem_n > N_W'(ARRAY_DIM)) ? DIM_W'(ARRAY_DIM) : DIM_W'(n_rem_n);

// when a MAX_* is 2^W-1 its upper-bound compare is constant true (every representable
// value is legal); it stays so every other parameterization is still range-checked
logic cmd_legal;
/* verilator lint_off CMPCONST */
assign cmd_legal = (M != '0) && (M <= M_W'(MAX_M)) && (N != '0) && (N <= N_W'(MAX_N)) && (K != '0) && (K <= K_W'(MAX_K));
/* verilator lint_on CMPCONST */

logic more_tiles;                           // another tile follows the one being fed
assign more_tiles = (n_rem > N_W'(ARRAY_DIM)) || (m_rem > M_W'(ARRAY_DIM));

logic handoff;                              // this edge ends the tile's feed_done cycle
assign handoff = (state == FEED) && feed_done;

// outputs: front from the state, back gated by busy (V8)
assign busy       = (state == ISSUE) || (state == FEED) || (state == WAIT) || (state == TAIL);
assign done       = (state == DONE) || (state == REJECT);
assign error      = (state == REJECT);
assign feed_start = (state == ISSUE);
assign capture    = busy && slot && (slot_cnt == '0);
assign shift_en   = busy && (drain != '0);
assign feed_M     = M_t;
assign feed_N     = N_t;
assign feed_K     = K_h;

logic load;                                 // accepted legal start: load tile 0
assign load = ((state == IDLE) || (state == DONE) || (state == REJECT)) && start && cmd_legal;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state      <= IDLE;
        tile_m     <= '0;
        tile_n     <= '0;
        m_rem      <= '0;
        n_rem      <= '0;
        N_h        <= '0;
        K_h        <= '0;
        cap_tile_m <= '0;
        cap_tile_n <= '0;
        cap_M      <= '0;
        cap_N      <= '0;
        slot       <= 1'b0;
        slot_cnt   <= '0;
        drain      <= '0;
        gap        <= '0;
    end else begin
        state      <= state_n;
        tile_m     <= tile_m_n;
        tile_n     <= tile_n_n;
        m_rem      <= m_rem_n;
        n_rem      <= n_rem_n;
        N_h        <= N_h_n;
        K_h        <= K_h_n;
        cap_tile_m <= cap_tile_m_n;
        cap_tile_n <= cap_tile_n_n;
        cap_M      <= cap_M_n;
        cap_N      <= cap_N_n;
        slot       <= slot_n;
        slot_cnt   <= slot_cnt_n;
        drain      <= drain_n;
        gap        <= gap_n;
    end
end

// ---- back sequencer: capture -> drain, countdowns, handoff --------------------
always_comb begin
    slot_n       = slot;
    slot_cnt_n   = slot_cnt;
    drain_n      = drain;
    gap_n        = gap;
    cap_tile_m_n = cap_tile_m;
    cap_tile_n_n = cap_tile_n;
    cap_M_n      = cap_M;
    cap_N_n      = cap_N;

    if (capture) begin
        slot_n  = 1'b0;
        drain_n = DRAIN_LEN;
    end else begin
        if (drain != '0) drain_n = drain - CNT_W'(1);
        if (slot)        slot_cnt_n = slot_cnt - CNT_W'(1);
    end
    if (gap != '0) gap_n = gap - CNT_W'(1);

    // the tile whose feed just ended enters the capture slot; the first cycle
    // after this edge is tau = K+2, its capture is at tau = K+M_t+N_t+1
    if (handoff) begin
        slot_n       = 1'b1;
        slot_cnt_n   = CNT_W'(M_t) + CNT_W'(N_t) - CNT_W'(1);
        gap_n        = CNT_W'(M_t) + CNT_W'(N_t) + CNT_W'(ARRAY_DIM);
        cap_tile_m_n = tile_m;
        cap_tile_n_n = tile_n;
        cap_M_n      = M_t;
        cap_N_n      = N_t;
    end

    // V8: an accepted start clears the back sequencer (cap_* keep their values)
    if (load) begin
        slot_n     = 1'b0;
        slot_cnt_n = '0;
        drain_n    = '0;
        gap_n      = '0;
    end
end

// ---- feed side: load tile 0, or advance on the handoff edge (row-major) --------
always_comb begin
    tile_m_n = tile_m;
    tile_n_n = tile_n;
    m_rem_n  = m_rem;
    n_rem_n  = n_rem;
    N_h_n    = N_h;
    K_h_n    = K_h;

    if (load) begin
        tile_m_n = '0;
        tile_n_n = '0;
        m_rem_n  = M;
        n_rem_n  = N;
        N_h_n    = N;
        K_h_n    = K;
    end else if (handoff && more_tiles) begin
        if (n_rem > N_W'(ARRAY_DIM)) begin              // next tile in this row
            tile_n_n = tile_n + TN_W'(1);
            n_rem_n  = n_rem - N_W'(ARRAY_DIM);
        end else begin                                  // first tile of the next row
            tile_m_n = tile_m + TM_W'(1);
            m_rem_n  = m_rem - M_W'(ARRAY_DIM);
            tile_n_n = '0;
            n_rem_n  = N_h;
        end
    end
end

// ---- ISSUE guard, on the values after this edge (§6.2) -------------------------
//   (b) the pending capture is <= min(3, K+1) cycles after the new ISSUE
//   (c) the new tile's capture lands >= n+1 after the latest capture
// (a) holds by construction: ISSUE is never earlier than the cycle after feed_done
logic [CNT_W-1:0] b_lim;
logic issue_ok;
assign b_lim    = (K_h == K_W'(1)) ? CNT_W'(2) : CNT_W'(3);
assign issue_ok = (!slot_n || (slot_cnt_n <= b_lim))
               && (int'(gap_n) <= int'(K_h) + int'(M_t_nx) + int'(N_t_nx) + 1);

// ---- front FSM -------------------------------------------------------------------
always_comb begin
    state_n = state;
    case (state)
        // idle-like states: busy low, so start is sampled
        IDLE, DONE, REJECT: begin
            if (!start)         state_n = IDLE;
            else if (cmd_legal) state_n = ISSUE;
            else                state_n = REJECT;
        end

        ISSUE: state_n = FEED;

        // feeder streams K payloads; read_sequencer owns the k loop
        FEED: begin
            if (handoff) begin
                if (!more_tiles)   state_n = TAIL;
                else if (issue_ok) state_n = ISSUE;
                else               state_n = WAIT;
            end
        end

        WAIT: if (issue_ok) state_n = ISSUE;

        // every tile is fed: wait for the last tile's last shift
        TAIL: if (shift_en && (drain == CNT_W'(1)) && !slot_n) state_n = DONE;

        // recovery for an encoding outside the enum (e.g. after a re-encode + upset)
        default: state_n = IDLE;
    endcase
end

`ifndef SYNTHESIS
// ---------------------------------------------------------------------------
// Assertions: the contract §9 properties that are local to this block
// (simulate with Verilator --assert). §9.6e (feeder idle at feed_start) and
// §9.12 (feed_done only in FEED) are system properties, checked by the cocotb
// feeder stand-in and the system observer. Async-reset behaviour (outputs idle
// while rst_n is low) cannot be seen by assertions disabled in reset; the
// cocotb ProtocolMonitor and Env.reset check it.
//
// The checker counters below re-derive the timing from the pulses themselves
// (cycles since a feed_start / capture), not from the design's slot/drain/gap
// counters, so a wrong counter cannot also fool its own check.
// ---------------------------------------------------------------------------

// armed one edge after reset release, so $past never looks into reset; async-reset
// flop keeps rst_n purely asynchronous (disable iff (!sva_armed) would sample it synchronously)
logic sva_armed;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) sva_armed <= 1'b0;
    else        sva_armed <= 1'b1;
end

localparam int SVA_FAR = 1 << 20;           // "no capture for a long time"

int  sva_since_issue;                       // cycles since the latest feed_start (1 = the cycle after)
int  sva_since_cap;                         // cycles since the latest capture, across jobs
int  sva_front_age;                         // cycles since the fed tile's feed_start
int  sva_slot_age;                          // cycles since the slot tile's feed_start
logic sva_have_prev;                        // this job has issued a tile already
logic [DIM_W-1:0] sva_prev_M, sva_prev_N;   // dims of the previously issued tile
logic [TM_W+TN_W+2*DIM_W-1:0] sva_front_ids, sva_slot_ids;   // {tile_m, tile_n, M_t, N_t} at feed_start
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sva_since_issue <= 0;
        sva_since_cap   <= SVA_FAR;
        sva_front_age   <= 0;
        sva_slot_age    <= 0;
        sva_have_prev   <= 1'b0;
        sva_prev_M      <= '0;
        sva_prev_N      <= '0;
        sva_front_ids   <= '0;
        sva_slot_ids    <= '0;
    end else begin
        sva_since_issue <= feed_start ? 1 : sva_since_issue + 1;
        sva_since_cap   <= capture ? 1 : ((sva_since_cap < SVA_FAR) ? sva_since_cap + 1 : SVA_FAR);
        sva_front_age   <= feed_start ? 1 : sva_front_age + 1;
        sva_slot_age    <= handoff ? sva_front_age + 1 : sva_slot_age + 1;
        if (load)            sva_have_prev <= 1'b0;
        else if (feed_start) sva_have_prev <= 1'b1;
        if (feed_start) begin
            sva_prev_M    <= feed_M;
            sva_prev_N    <= feed_N;
            sva_front_ids <= {tile_m, tile_n, feed_M, feed_N};
        end
        if (handoff) sva_slot_ids <= sva_front_ids;
    end
end

// §6.2 gap, written out from the contract (not from the guard above)
function automatic int sva_gap(int k, int mt, int nt, int mt2, int nt2);
    int a, b, c, g;
    a = k + 2;
    b = mt + nt + ((k > 2) ? k - 2 : 0);
    c = ARRAY_DIM + 1 + mt + nt - mt2 - nt2;
    g = (a > b) ? a : b;
    return (g > c) ? g : c;
endfunction

function automatic logic legal_step(state_t from, state_t to);
    case (from)
        IDLE, DONE, REJECT: return to inside {IDLE, ISSUE, REJECT};
        ISSUE:              return to == FEED;
        FEED:               return to inside {FEED, ISSUE, WAIT, TAIL};
        WAIT:               return to inside {WAIT, ISSUE};
        TAIL:               return to inside {TAIL, DONE};
        default:            return 1'b0;
    endcase
endfunction

// §9.1-3: command handshake
a_busy_means_working: assert property (@(posedge clk) disable iff (!sva_armed)
    busy == (state inside {ISSUE, FEED, WAIT, TAIL}));
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
a_feed_left_only_on_done: assert property (@(posedge clk) disable iff (!sva_armed)
    ($past(state) == FEED && state != FEED) |-> $past(feed_done));

// §9.4: pulses only while working, never capture with shift
a_pulses_only_while_busy: assert property (@(posedge clk) disable iff (!sva_armed)
    (feed_start || capture || shift_en) |-> busy);
a_capture_excludes_shift: assert property (@(posedge clk) disable iff (!sva_armed)
    !(capture && shift_en));

// §9.6: timing
a_capture_offset: assert property (@(posedge clk) disable iff (!sva_armed)                 // (a)
    capture |-> sva_slot_age == int'(K_h) + int'(cap_M) + int'(cap_N) + 1);
a_drain_is_n_minus_1_shifts: assert property (@(posedge clk) disable iff (!sva_armed)      // (b)
    capture |=> shift_en [*ARRAY_DIM-1] ##1 !shift_en);
a_shift_only_after_capture: assert property (@(posedge clk) disable iff (!sva_armed)
    shift_en |-> (sva_since_cap >= 1) && (sva_since_cap <= ARRAY_DIM - 1));
a_captures_n_plus_1_apart: assert property (@(posedge clk) disable iff (!sva_armed)        // (c)
    capture |-> sva_since_cap >= ARRAY_DIM + 1);
a_issue_gap: assert property (@(posedge clk) disable iff (!sva_armed)                      // (d)
    (feed_start && sva_have_prev)
        |-> sva_since_issue == sva_gap(int'(K_h), int'(sva_prev_M), int'(sva_prev_N),
                                       int'(feed_M), int'(feed_N)));
a_done_n_after_last_capture: assert property (@(posedge clk) disable iff (!sva_armed)
    (state == DONE) |-> sva_since_cap == ARRAY_DIM);
a_capture_slot_never_overrun: assert property (@(posedge clk) disable iff (!sva_armed)     // §5.2
    handoff |-> (!slot || capture));

// §9.7-8: tile indices and dims in range while working; row-major advance
a_tile_index_in_range: assert property (@(posedge clk) disable iff (!sva_armed)
    busy |-> (int'(tile_m) < TILES_M) && (int'(tile_n) < TILES_N) && (m_rem != '0) && (n_rem != '0));
a_tile_dims_in_range: assert property (@(posedge clk) disable iff (!sva_armed)
    busy |-> (feed_M != '0) && (int'(feed_M) <= ARRAY_DIM) && (feed_N != '0) && (int'(feed_N) <= ARRAY_DIM)
             && (feed_K != '0) && (int'(feed_K) <= MAX_K));
a_tiles_row_major: assert property (@(posedge clk) disable iff (!sva_armed)
    ($past(handoff) && $past(more_tiles))
        |-> (tile_m == $past(tile_m) && tile_n == $past(tile_n) + TN_W'(1))
         || (tile_m == $past(tile_m) + TM_W'(1) && tile_n == '0 && n_rem == N_h));

// §9.9: the capture names the tile that was issued
a_cap_names_issued_tile: assert property (@(posedge clk) disable iff (!sva_armed)
    capture |-> {cap_tile_m, cap_tile_n, cap_M, cap_N} == sva_slot_ids);

// §9.10 / §4.6: stability
a_feed_side_held: assert property (@(posedge clk) disable iff (!sva_armed)
    !$stable({feed_M, feed_N, feed_K, tile_m, tile_n})
        |-> (state == ISSUE && $past(state) inside {IDLE, DONE, REJECT})
         || ($past(handoff) && $past(more_tiles)));
a_cap_side_held: assert property (@(posedge clk) disable iff (!sva_armed)
    !$stable({cap_tile_m, cap_tile_n, cap_M, cap_N}) |-> $past(handoff));
a_handoff_loads_fed_tile: assert property (@(posedge clk) disable iff (!sva_armed)
    $past(handoff) |-> {cap_tile_m, cap_tile_n, cap_M, cap_N}
                       == {$past(tile_m), $past(tile_n), $past(feed_M), $past(feed_N)});

// V8: an accepted start empties the back sequencer
a_load_clears_back: assert property (@(posedge clk) disable iff (!sva_armed)
    load |=> !slot && (drain == '0) && (gap == '0));
`endif

endmodule
