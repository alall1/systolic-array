"""
Cycle-accurate model of "feeder" (read_sequencer + skew_buffer).

Composes the two validated models without modifying them.

step() = one rising clock edge; returns the POST-edge value of every output.
Edge 0 is the edge on which `start` is sampled.

Timing contract (matches the RTL):
  busy, rd_en      : high for edges 0 .. K-1
  rd_col_a/rd_row_b: == e while rd_en (don't-care otherwise -> None)
  done             : high only at edge K (busy is already low)
  payload k        : leaves the sequencer at edge k+1, is captured by the skew
                     buffer at edge k+2, and appears on lane i at edge k+2+i
  start is accepted iff busy was low before the edge; the earliest legal
  back-to-back start is edge K+1.

Output lane encoding (None = lane invalid, data/first are don't-care in RTL):
  out_a[i] -> None | (data, first)
  out_b[i] -> None | (data,)
The skew model's reset state (int 0) decodes to None, i.e. valid=0.
"""

from read_sequencer_model import ReadSequencerModel
from skew_buffer_model import SkewBufferModel   # adjust if your filename differs


class FeederModel:
    def __init__(self, array_dim, buf_a_fn, buf_b_fn):
        self.n = array_dim
        self.buf_a_fn = buf_a_fn   # (k, lane) -> data word
        self.buf_b_fn = buf_b_fn   # (k, lane) -> data word
        self.skew = SkewBufferModel(array_dim)
        self.reset()

    # ------------------------------------------------------------------ #
    @staticmethod
    def _decode(vec):
        return [x if isinstance(x, tuple) else None for x in vec]

    def reset(self):
        """Model state after >=1 clock edge with rst_n low. Returns the idle
        output snapshot so a testbench can check the DUT against it."""
        n = self.n
        self.skew.reset()
        self.seq = None
        self.K = 0
        self._e = 0
        self.busy = False
        self._pend_a = [None] * n
        self._pend_b = [None] * n
        a, b = self.skew.out()
        return dict(busy=False, done=False, rd_en=False,
                    rd_col_a=None, rd_row_b=None,
                    out_a=self._decode(a), out_b=self._decode(b))

    # ------------------------------------------------------------------ #
    def step(self, start=False, M=0, N=0, K=0):
        n = self.n
        busy_prev = self.busy

        # edge: skew buffer captures what the sequencer presented last cycle
        sk_a, sk_b = self.skew.step(self._pend_a, self._pend_b)

        # edge: job control (start only accepted if busy was low)
        if start and not busy_prev:
            assert 1 <= M <= n and 1 <= N <= n and K >= 1, \
                f"illegal job M={M} N={N} K={K}"
            self.seq = ReadSequencerModel(n, M, N, K, self.buf_a_fn, self.buf_b_fn)
            self.K = K
            self._e = 0
        elif self.seq is not None:
            self._e += 1
            if self._e > self.K:        # past the done cycle -> job over
                self.seq = None

        # post-edge control outputs
        in_job = self.seq is not None
        busy   = in_job and self._e < self.K
        done   = in_job and self._e == self.K
        rd_en  = busy
        addr   = self._e if busy else None

        # payload the sequencer presents this cycle (captured at the NEXT edge)
        pend_a = [None] * n
        pend_b = [None] * n
        if in_job and 1 <= self._e <= self.K:
            payload = self.seq.step()           # first call -> k=0 at edge 1
            assert payload is not None
            a_raw, b_raw, first = payload
            assert done == self.seq.is_last_payload()
            for lane in range(n):
                if a_raw[lane] is not None:
                    pend_a[lane] = (a_raw[lane], bool(first))
                if b_raw[lane] is not None:
                    pend_b[lane] = (b_raw[lane],)
        self._pend_a, self._pend_b = pend_a, pend_b
        self.busy = busy

        return dict(busy=busy, done=done, rd_en=rd_en,
                    rd_col_a=addr, rd_row_b=addr,
                    out_a=self._decode(sk_a), out_b=self._decode(sk_b))


# ---------------------------------------------------------------------- #
# Pure-Python self-check, no RTL involved:  python feeder_model.py
# Invariant: a[i][k] and b[k][j] must reach PE(i,j) on the same cycle
# (A hops east 1 cycle/PE, B hops south 1 cycle/PE), plus the timing contract.
# ---------------------------------------------------------------------- #
if __name__ == "__main__":
    import random
    rng = random.Random(0)
    n = 4
    for (M, N, K) in [(4, 4, 4), (1, 1, 1), (3, 2, 5), (4, 1, 7), (2, 4, 1), (1, 1, 6)]:
        A = [[rng.randint(-128, 127) for _ in range(n)] for _ in range(K)]  # A[k][lane]
        B = [[rng.randint(-128, 127) for _ in range(n)] for _ in range(K)]  # B[k][lane]
        m = FeederModel(n, lambda k, l: A[k][l], lambda k, l: B[k][l])

        ta = [[] for _ in range(n)]
        tb = [[] for _ in range(n)]
        for e in range(K + 2 * n + 4):
            o = m.step(start=(e == 0), M=M, N=N, K=K)
            assert o["busy"] == (e < K) and o["rd_en"] == (e < K)
            assert o["done"] == (e == K)
            if e < K:
                assert o["rd_col_a"] == e and o["rd_row_b"] == e
            for i, x in enumerate(o["out_a"]):
                if x is not None:
                    ta[i].append((e, x[0], x[1]))
            for j, x in enumerate(o["out_b"]):
                if x is not None:
                    tb[j].append((e, x[0]))

        for i in range(n):
            assert len(ta[i]) == (K if i < M else 0)
            assert len(tb[i]) == (K if i < N else 0)
        for i in range(M):
            for j in range(N):
                for k in range(K):
                    assert ta[i][k][0] == k + 2 + i
                    assert tb[j][k][0] == k + 2 + j
                    assert ta[i][k][0] + j == tb[j][k][0] + i      # same-cycle at PE(i,j)
                    assert ta[i][k][1] == A[k][i] and tb[j][k][1] == B[k][j]
                    assert ta[i][k][2] == (k == 0)
    print("feeder_model self-check passed")