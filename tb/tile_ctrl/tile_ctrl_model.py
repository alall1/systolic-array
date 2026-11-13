"""
Cycle-accurate golden model for "tile_ctrl" v2 (drain/feed overlap), plus a
pure schedule helper that predicts the same timing WITHOUT a state machine.

Contract: docs/fsm_contract.md (v2)

step() = one rising clock edge. Inputs passed to step() are the values visible
BEFORE that edge; it returns the POST-edge value of every output.

Two sequencers (contract §5):
  front FSM : IDLE / ISSUE / FEED / WAIT / TAIL / DONE / REJECT
              owns the command interface, feed_start and the feed-side tile
  back      : a one-tile capture slot (capture K+M_t+N_t+1 after the tile's
              ISSUE) and the drain (n-1 shift_en cycles after each capture)
They meet on the handoff edge that ends a tile's feed_done cycle.

The model decides ISSUE with counters (cycles to the pending capture, cycles
until the next capture is allowed). tile_schedule() below uses the contract's
§6.2 gap formula instead, so the two are independent references.
"""

IDLE   = "IDLE"
ISSUE  = "ISSUE"
FEED   = "FEED"
WAIT   = "WAIT"
TAIL   = "TAIL"
DONE   = "DONE"
REJECT = "REJECT"

# same order as the RTL enum, so index == encoding
STATES = (IDLE, ISSUE, FEED, WAIT, TAIL, DONE, REJECT)
BUSY_STATES = frozenset((ISSUE, FEED, WAIT, TAIL))
ACCEPT_STATES = frozenset((IDLE, DONE, REJECT))     # busy low -> start is sampled

FEED_SIDE = ("feed_M", "feed_N", "feed_K", "tile_m", "tile_n")
CAP_SIDE = ("cap_tile_m", "cap_tile_n", "cap_M", "cap_N")


def ceil_div(a, b):
    return -(-a // b)


class TileCtrlModel:
    def __init__(self, array_dim, max_m, max_n, max_k):
        assert array_dim >= 2, "contract requires ARRAY_DIM >= 2 (drain is n-1 shifts)"
        self.n = array_dim
        self.max_m = max_m
        self.max_n = max_n
        self.max_k = max_k
        # port widths: clog2(X+1) == X.bit_length()
        self.m_w = max_m.bit_length()
        self.n_w = max_n.bit_length()
        self.k_w = max_k.bit_length()
        self.reset()

    # ------------------------------------------------------------------ #
    def legal(self, M, N, K):
        return (1 <= M <= self.max_m) and (1 <= N <= self.max_n) and (1 <= K <= self.max_k)

    def _clear_back(self):
        self.slot = False       # a tile waits in the capture slot
        self.slot_cnt = 0       # cycles until its capture (0 = capture this cycle)
        self.drain = 0          # shift_en cycles left, including this one
        self.gap = 0            # cycles until the next capture is allowed (n+1 rule)

    def reset(self):
        """State after rst_n low. Returns the idle output snapshot."""
        self.state = IDLE
        # front: tile being fed
        self.tile_m = 0
        self.tile_n = 0
        self.m_rem = 0          # rows remaining, including the current tile row
        self.n_rem = 0          # cols remaining, including the current tile col
        self.N_h = 0            # job N, reloads n_rem at each new tile row
        self.K_h = 0
        # back: tile in (or last through) the capture slot
        self.cap_tile_m = 0
        self.cap_tile_n = 0
        self.cap_M = 0
        self.cap_N = 0
        self._clear_back()
        return self.outputs()

    def _tile_dims(self):
        return min(self.n, self.m_rem), min(self.n, self.n_rem)

    def outputs(self):
        s = self.state
        busy = s in BUSY_STATES
        M_t, N_t = self._tile_dims()
        return dict(
            state=s,
            busy=busy,
            done=s in (DONE, REJECT),
            error=s == REJECT,
            feed_start=s == ISSUE,
            # V8: back-sequencer pulses are gated by busy
            capture=busy and self.slot and self.slot_cnt == 0,
            shift_en=busy and self.drain > 0,
            feed_M=M_t,
            feed_N=N_t,
            feed_K=self.K_h,
            tile_m=self.tile_m,
            tile_n=self.tile_n,
            cap_tile_m=self.cap_tile_m,
            cap_tile_n=self.cap_tile_n,
            cap_M=self.cap_M,
            cap_N=self.cap_N,
        )

    # ------------------------------------------------------------------ #
    def _load(self, M, N, K):
        self.tile_m = 0
        self.tile_n = 0
        self.m_rem = M
        self.n_rem = N
        self.N_h = N
        self.K_h = K
        self._clear_back()                  # V8

    def _more_tiles(self):
        return self.n_rem > self.n or self.m_rem > self.n

    def _advance(self):
        """Handoff edge with more tiles: next tile, row-major."""
        n = self.n
        if self.n_rem > n:
            self.tile_n += 1
            self.n_rem -= n
        else:
            self.tile_m += 1
            self.m_rem -= n
            self.tile_n = 0
            self.n_rem = self.N_h

    def _handoff(self):
        """Edge that ends the feed_done cycle: tile -> capture slot."""
        assert not self.slot, "capture slot still occupied at handoff (schedule broken)"
        M_t, N_t = self._tile_dims()
        self.cap_tile_m, self.cap_tile_n = self.tile_m, self.tile_n
        self.cap_M, self.cap_N = M_t, N_t
        self.slot = True
        # first post-handoff cycle is tau = K+2, capture at tau = K+M_t+N_t+1
        self.slot_cnt = M_t + N_t - 1
        self.gap = M_t + N_t - 1 + self.n + 1

    def _issue_ok(self):
        """Post-edge back state allows THIS (post-edge) cycle to be the next ISSUE.
        (b) the pending capture is <= min(3, K+1) cycles away;
        (c) the new tile's capture lands >= n+1 after the latest capture."""
        K = self.K_h
        M_t, N_t = self._tile_dims()            # already the next tile
        b_ok = (not self.slot) or self.slot_cnt <= min(3, K + 1)
        c_ok = self.gap <= K + M_t + N_t + 1
        return b_ok and c_ok

    def step(self, start=False, M=0, N=0, K=0, feed_done=False):
        # the DUT only sees the low port-width bits
        M &= (1 << self.m_w) - 1
        N &= (1 << self.n_w) - 1
        K &= (1 << self.k_w) - 1

        pre = self.outputs()
        s = self.state

        # ---- back sequencer: advance on every edge ------------------------
        if pre["capture"]:
            self.slot = False
            self.drain = self.n - 1
        elif self.drain > 0:
            self.drain -= 1
        if self.slot:
            self.slot_cnt -= 1
        if self.gap > 0:
            self.gap -= 1

        # ---- front FSM ------------------------------------------------------
        if s in ACCEPT_STATES:
            if not start:
                nxt = IDLE
            elif self.legal(M, N, K):
                self._load(M, N, K)
                nxt = ISSUE
            else:
                nxt = REJECT
        elif s == ISSUE:
            nxt = FEED
        elif s == FEED:
            nxt = FEED
            if feed_done:
                self._handoff()
                if self._more_tiles():
                    self._advance()
                    nxt = ISSUE if self._issue_ok() else WAIT
                else:
                    nxt = TAIL
        elif s == WAIT:
            nxt = ISSUE if self._issue_ok() else WAIT
        elif s == TAIL:
            # the last shift of the last tile was the pre-edge cycle
            last_shift = pre["shift_en"] and self.drain == 0 and not self.slot
            nxt = DONE if last_shift else TAIL
        else:
            raise AssertionError(f"unknown state {s}")

        self.state = nxt
        return self.outputs()


# ---------------------------------------------------------------------- #
# Pure schedule helpers: no state machine, only the contract's §6.2 rule.
# ---------------------------------------------------------------------- #

def tile_gap(K, n, M_t, N_t, M_t2, N_t2):
    """§6.2: ISSUE-to-ISSUE gap from a M_t x N_t tile to a M_t2 x N_t2 tile.
    Returns (G, terms) with terms = dict(a=, b=, c=)."""
    terms = dict(a=K + 2,
                 b=M_t + N_t + max(K - 2, 0),
                 c=n + 1 + (M_t + N_t) - (M_t2 + N_t2))
    return max(terms.values()), terms


def tile_schedule(M, N, K, n):
    """Expected per-tile schedule of one legal job, in job-relative cycles t
    (t = 0 is tile 0's ISSUE).

    Returns dict(tiles=[...], latency=L) where each tile is
      tile_m, tile_n, M_t, N_t,
      issue    : t of its ISSUE cycle (feed_start high)
      feed_done: t of its feed_done cycle (handoff edge at its end)
      capture  : t of its capture cycle
      shift_en : the n-1 cycles with shift_en high
      gap      : cycles to the next tile's ISSUE (None for the last tile)
      terms    : the three §6.2 terms of that gap (None for the last tile)
    and latency is the t at which done is high.
    """
    assert M >= 1 and N >= 1 and K >= 1 and n >= 2
    dims = [(tm, tn, min(n, M - tm * n), min(n, N - tn * n))
            for tm in range(ceil_div(M, n)) for tn in range(ceil_div(N, n))]
    tiles = []
    t = 0
    for i, (tm, tn, M_t, N_t) in enumerate(dims):
        capture = t + K + M_t + N_t + 1
        if i + 1 < len(dims):
            gap, terms = tile_gap(K, n, M_t, N_t, *dims[i + 1][2:])
        else:
            gap, terms = None, None
        tiles.append(dict(tile_m=tm, tile_n=tn, M_t=M_t, N_t=N_t,
                          issue=t, feed_done=t + K + 1, capture=capture,
                          shift_en=list(range(capture + 1, capture + n)),
                          gap=gap, terms=terms))
        if gap is not None:
            t += gap
    return dict(tiles=tiles, latency=tiles[-1]["capture"] + n)


def job_latency(M, N, K, n):
    """Accepted start edge -> done visible (t of the done cycle), §6.2."""
    return tile_schedule(M, N, K, n)["latency"]
