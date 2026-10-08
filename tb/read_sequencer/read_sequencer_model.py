"""
Cycle-accurate golden model for read_sequencer.

Timing contract (must match RTL):
  - start edge latches config, issues k=0's address.
  - A read issued at cycle T returns data at T+1 (sync buffer).
  - So the first data cycle after start is a FILL cycle: no valid payload.
  - Valid payloads for k = 0..K-1 emerge on the K cycles following the fill.

step() convention:
  Call step() once per data cycle, starting with the first edge AFTER the
  start edge. Returns:
    None                               -> no valid payload this cycle
    (a_vec, b_vec, first)              -> payload visible this cycle, where
        a_vec[lane] = data on A lane, or None if lane invalid (lane >= M)
        b_vec[lane] = data on B lane, or None if lane invalid (lane >= N)
        first       = bool, True iff this is k==0's data
"""


class ReadSequencerModel:
    def __init__(self, array_dim, M, N, K, buf_a_fn, buf_b_fn):
        self.n = array_dim
        self.M = M
        self.N = N
        self.K = K
        self.buf_a = buf_a_fn   # (col, lane) -> data word
        self.buf_b = buf_b_fn   # (row, lane) -> data word

        # internal cycle counter over data cycles since start edge.
        # data_cycle 0 is the FILL cycle (k=0 address issued, data not back yet).
        # data_cycle d (d >= 1) carries k = d-1's data.
        self._dc = 1

    def step(self):
        """Advance one data cycle; return payload-or-None for THIS cycle."""
        dc = self._dc
        self._dc += 1

        k = dc - 1   # which k's data is visible this cycle

        # fill cycle, or past the end of the feed -> nothing valid
        if k < 0 or k >= self.K:
            return None

        a_vec = [None] * self.n
        b_vec = [None] * self.n
        for lane in range(self.n):
            if lane < self.M:
                a_vec[lane] = self.buf_a(k, lane)
            if lane < self.N:
                b_vec[lane] = self.buf_b(k, lane)

        first = (k == 0)
        return (a_vec, b_vec, first)

    def is_last_payload(self):
        """True iff the payload returned by the MOST RECENT step() was k==K-1's.
        Call right after step(). Used to check done alignment."""
        k = (self._dc - 1) - 1   # _dc already advanced; last step's k
        return k == self.K - 1

    def reset(self):
        self._dc = 0