"""
Cycle-accurate golden model for "tile_ctrl" (control FSM), plus a pure
schedule helper that predicts the same timing WITHOUT a state machine.

Contract: docs/fsm_contract.md

step() = one rising clock edge. Inputs passed to step() are the values visible
BEFORE that edge; it returns the POST-edge value of every output.

Job-relative time t: t = 0 is the cycle right after the accepted start edge
(tile 0's ISSUE). Per tile:

  ISSUE 1 | FEED K+1 (ends on feed_done) | FLUSH M_t+N_t-1 | CAPTURE 1 | DRAIN n-1
  T_tile = K + M_t + N_t + n + 1

Job latency (accepted start edge -> done visible):
  L = T_m*T_n*(K+n+1) + T_n*M + T_m*N,   T_m = ceil(M/n), T_n = ceil(N/n)
"""

IDLE    = "IDLE"
ISSUE   = "ISSUE"
FEED    = "FEED"
FLUSH   = "FLUSH"
CAPTURE = "CAPTURE"
DRAIN   = "DRAIN"
DONE    = "DONE"
REJECT  = "REJECT"

# same order as the RTL enum, so index == encoding
STATES = (IDLE, ISSUE, FEED, FLUSH, CAPTURE, DRAIN, DONE, REJECT)
BUSY_STATES = frozenset((ISSUE, FEED, FLUSH, CAPTURE, DRAIN))
ACCEPT_STATES = frozenset((IDLE, DONE, REJECT))     # busy low -> start is sampled


def ceil_div(a, b):
    return -(-a // b)


class TileCtrlModel:
    def __init__(self, array_dim, max_m, max_n, max_k):
        assert array_dim >= 2, "contract requires ARRAY_DIM >= 2 (DRAIN is n-1 cycles)"
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

    def reset(self):
        """State after rst_n low. Returns the idle output snapshot."""
        self.state = IDLE
        self.cnt = 0
        self.tile_m = 0
        self.tile_n = 0
        self.m_rem = 0      # rows remaining, including the current tile row
        self.n_rem = 0      # cols remaining, including the current tile col
        self.N_h = 0        # job N, reloads n_rem at each new tile row
        self.K_h = 0
        return self.outputs()

    def _tile_dims(self):
        return min(self.n, self.m_rem), min(self.n, self.n_rem)

    def outputs(self):
        s = self.state
        M_t, N_t = self._tile_dims()
        return dict(
            state=s,
            busy=s in BUSY_STATES,
            done=s in (DONE, REJECT),
            error=s == REJECT,
            feed_start=s == ISSUE,
            capture=s == CAPTURE,
            shift_en=s == DRAIN,
            feed_M=M_t,
            feed_N=N_t,
            feed_K=self.K_h,
            tile_m=self.tile_m,
            tile_n=self.tile_n,
        )

    # ------------------------------------------------------------------ #
    def _load(self, M, N, K):
        self.tile_m = 0
        self.tile_n = 0
        self.m_rem = M
        self.n_rem = N
        self.N_h = N
        self.K_h = K

    def _advance(self):
        """DRAIN's last cycle: next tile (row-major) or end of job."""
        n = self.n
        if self.n_rem > n:
            self.tile_n += 1
            self.n_rem -= n
            return ISSUE
        if self.m_rem > n:
            self.tile_m += 1
            self.m_rem -= n
            self.tile_n = 0
            self.n_rem = self.N_h
            return ISSUE
        return DONE

    def step(self, start=False, M=0, N=0, K=0, feed_done=False):
        # the DUT only sees the low port-width bits
        M &= (1 << self.m_w) - 1
        N &= (1 << self.n_w) - 1
        K &= (1 << self.k_w) - 1

        s = self.state
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
                M_t, N_t = self._tile_dims()
                self.cnt = M_t + N_t - 1        # FLUSH cycles, >= 1
                nxt = FLUSH
        elif s == FLUSH:
            nxt = FLUSH
            if self.cnt == 1:
                nxt = CAPTURE
            else:
                self.cnt -= 1
        elif s == CAPTURE:
            self.cnt = self.n - 1               # DRAIN cycles, >= 1
            nxt = DRAIN
        elif s == DRAIN:
            nxt = DRAIN
            if self.cnt == 1:
                nxt = self._advance()
            else:
                self.cnt -= 1
        else:
            raise AssertionError(f"unknown state {s}")

        self.state = nxt
        return self.outputs()


# ---------------------------------------------------------------------- #
# Pure schedule helpers: no state machine, only the contract's formulas.
# ---------------------------------------------------------------------- #

def tile_schedule(M, N, K, n):
    """Expected per-tile schedule of one legal job, in job-relative cycles t.

    Returns dict(tiles=[...], latency=L) where each tile is
      tile_m, tile_n, M_t, N_t,
      issue     : t of the ISSUE cycle (feed_start high)
      capture   : t of the CAPTURE cycle
      shift_en  : list of t with shift_en high (n-1 consecutive cycles)
      length    : T_tile
    and latency is the t at which done is high.
    """
    assert M >= 1 and N >= 1 and K >= 1 and n >= 2
    tiles = []
    t = 0
    for tm in range(ceil_div(M, n)):
        M_t = min(n, M - tm * n)
        for tn in range(ceil_div(N, n)):
            N_t = min(n, N - tn * n)
            length = K + M_t + N_t + n + 1
            capture = t + K + M_t + N_t + 1
            tiles.append(dict(
                tile_m=tm, tile_n=tn, M_t=M_t, N_t=N_t,
                issue=t,
                capture=capture,
                shift_en=list(range(capture + 1, capture + n)),
                length=length,
            ))
            t += length
    return dict(tiles=tiles, latency=t)


def job_latency(M, N, K, n):
    """Closed form of tile_schedule(...)['latency']."""
    T_m, T_n = ceil_div(M, n), ceil_div(N, n)
    return T_m * T_n * (K + n + 1) + T_n * M + T_m * N
