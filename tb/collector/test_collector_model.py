"""
Standalone pytest for CollectorModel (no RTL):  pytest test_collector_model.py

Independent references checked against the model:
  1. expected_tile_writes(): the contract's §5.1 rule, no register model
  2. a hand-written shadow chain (ShadowChain) for drain_out, so data
     pass-through and row order are checked against what pe_grid would show
  3. the full system in closed loop: TileCtrlModel + FeederModel + PEModel grid
     (tile_ctrl's validated Harness) + CollectorModel + a result memory. C must
     equal numpy A @ B, every element written exactly once, nothing outside
     M x N written, and the job's last write in tile_ctrl's DONE cycle.
"""

import os
import random

import numpy as np
import pytest

from collector_model import CollectorModel, expected_tile_writes
from shadow_chain_model import ShadowChain
from test_tile_ctrl_model import Harness

SEED = int(os.environ.get("SEED", "20261104"))
MAX_M = 64
MAX_N = 64
ACC_WIDTH = 32
IDLE_OUT = dict(wr_en=False, wr_row=0, wr_col=0, wr_lane_en=0)


def idle(o, n):
    return {k: o[k] for k in IDLE_OUT} == IDLE_OUT and o["wr_data"] == [0] * n

def rand_tile(rng, n, lo=-(1 << 31), hi=(1 << 31) - 1):
    return [[rng.randint(lo, hi) for _ in range(n)] for _ in range(n)]


class Bench:
    """Collector + shadow chain stepped together; records every write."""
    def __init__(self, n):
        self.n = n
        self.col = CollectorModel(n, MAX_M, MAX_N, ACC_WIDTH)
        self.chain = ShadowChain(n)
        self.t = 0
        self.writes = []        # (t, out dict)

    def tick(self, capture=False, shift_en=False, tile=(0, 0, 0, 0), acc=None):
        d = self.chain.step(capture, shift_en, acc)
        o = self.col.step(capture, shift_en, *tile, drain_out=d)
        self.t += 1
        if o["wr_en"]:
            self.writes.append((self.t, o))
        else:
            assert idle(o, self.n), f"t={self.t}: write port not idle without wr_en: {o}"
        return o

    def v1_drain(self, tile, acc, junk_rng=None):
        """capture, then n-1 shift_en cycles, then 1 idle cycle (row 0's slot).
        Tile inputs carry junk after the capture cycle when junk_rng is given."""
        n = self.n
        t_cap = self.t + 1                      # post-edge index of the capture edge
        self.tick(capture=True, tile=tile, acc=acc)
        for _ in range(n - 1):
            junk = tile if junk_rng is None else tuple(junk_rng.randrange(0, 1 << 3) for _ in range(4))
            self.tick(shift_en=True, tile=junk)
        self.tick()
        return t_cap


def check_tile(bench, t_cap, tile, acc, writes):
    tm, tn, rows, cols = tile
    n = bench.n
    exp = expected_tile_writes(tm, tn, rows, cols, n)
    assert len(writes) == len(exp), f"{len(writes)} writes, expected {len(exp)}"
    for (t, o), (off, r, row, col, lanes) in zip(writes, exp):
        assert t == t_cap + off, f"grid row {r} written at t={t}, expected {t_cap + off}"
        assert (o["wr_row"], o["wr_col"], o["wr_lane_en"]) == (row, col, lanes)
        assert o["wr_data"] == acc[r], f"row {r} data"

# --------------------------------------------------------------------------- #
# 1. reset / idle
# --------------------------------------------------------------------------- #

def test_reset_and_idle():
    m = CollectorModel(4, MAX_M, MAX_N, ACC_WIDTH)
    assert idle(m.reset(), 4)
    for _ in range(5):
        assert idle(m.step(shift_en=True, tile_m=3, tile_rows=4, tile_cols=4,
                           drain_out=[7, 7, 7, 7]), 4), "shift_en while idle must be ignored"


def test_expected_tile_writes_examples():
    """docs/collector_contract.md §5.1 worked example: n=4, tile (1,0), 2x3."""
    assert expected_tile_writes(1, 0, 2, 3, 4) == [(2, 1, 5, 0, 0b0111), (3, 0, 4, 0, 0b0111)]

# --------------------------------------------------------------------------- #
# 2. one tile, v1 timing: every shape, data pass-through, signedness
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("n", [2, 3, 4, 5])
def test_single_tile_all_shapes(n):
    rng = random.Random(SEED + n)
    b = Bench(n)
    for rows in range(1, n + 1):
        for cols in range(1, n + 1):
            tile = (rng.randrange(0, 4), rng.randrange(0, 4), rows, cols)
            acc = rand_tile(rng, n)
            b.writes = []
            t_cap = b.v1_drain(tile, acc)
            check_tile(b, t_cap, tile, acc, b.writes)
            for _ in range(3):
                b.tick()
            assert not b.col.active


def test_extreme_values_pass_through():
    """Signed reinterpretation of the unsigned drain_out bits, both extremes."""
    n = 3
    b = Bench(n)
    acc = [[-(1 << 31), (1 << 31) - 1, -1]] * n
    unsigned = [[v & 0xFFFFFFFF for v in r] for r in acc]
    t_cap = b.v1_drain((0, 0, n, n), unsigned)
    check_tile(b, t_cap, (0, 0, n, n), acc, b.writes)


@pytest.mark.parametrize("n", [2, 4, 5])
def test_tile_inputs_latched_at_capture(n):
    """Junk on tile_m/tile_n/rows/cols after capture (as on the real next-ISSUE
    cycle) must not change where or how much is written."""
    rng = random.Random(SEED + 10 + n)
    b = Bench(n)
    for _ in range(20):
        tile = (rng.randrange(0, 4), rng.randrange(0, 4), rng.randint(1, n), rng.randint(1, n))
        acc = rand_tile(rng, n)
        b.writes = []
        t_cap = b.v1_drain(tile, acc, junk_rng=rng)
        check_tile(b, t_cap, tile, acc, b.writes)

# --------------------------------------------------------------------------- #
# 3. D2 robustness: shift_en stalls (not produced by v1 tile_ctrl)
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("n", [2, 4, 5])
def test_stalled_drain_offers_each_row_once(n):
    """Random idle cycles between shifts: each kept row is written exactly once,
    on the first cycle it is on drain_out, with the right data."""
    rng = random.Random(SEED + 20 + n)
    b = Bench(n)
    for _ in range(20):
        tile = (rng.randrange(0, 4), rng.randrange(0, 4), rng.randint(1, n), rng.randint(1, n))
        acc = rand_tile(rng, n)
        b.writes = []
        b.tick(capture=True, tile=tile, acc=acc)
        first_seen = {n - 1: b.t}
        for r in range(n - 2, -1, -1):
            for _ in range(rng.randint(0, 3)):
                b.tick()
            b.tick(shift_en=True)
            first_seen[r] = b.t
        for _ in range(rng.randint(1, 4)):
            b.tick()
        tm, tn, rows, cols = tile
        assert [(t, o["wr_row"]) for t, o in b.writes] == \
               [(first_seen[r], tm * n + r) for r in range(rows - 1, -1, -1)]
        assert all(o["wr_data"] == acc[o["wr_row"] - tm * n] for _, o in b.writes)
        assert not b.col.active

# --------------------------------------------------------------------------- #
# 4. out-of-contract inputs (§6)
# --------------------------------------------------------------------------- #

def test_capture_during_drain_restarts():
    n = 4
    rng = random.Random(SEED + 30)
    b = Bench(n)
    t1, t2 = (0, 0, n, n), (2, 1, 3, 2)
    acc1, acc2 = rand_tile(rng, n), rand_tile(rng, n)
    b.tick(capture=True, tile=t1, acc=acc1)
    b.tick(shift_en=True)
    b.writes = []
    t_cap = b.t + 1
    b.tick(capture=True, tile=t2, acc=acc2)             # mid-drain: restart on tile 2
    for _ in range(n - 1):
        b.tick(shift_en=True)
    b.tick()
    check_tile(b, t_cap, t2, acc2, b.writes)


def test_capture_beats_shift_en():
    n = 3
    rng = random.Random(SEED + 31)
    b = Bench(n)
    tile, acc = (1, 1, n, n), rand_tile(rng, n)
    t_cap = b.t + 1
    b.tick(capture=True, shift_en=True, tile=tile, acc=acc)   # chain: capture wins too
    for _ in range(n - 1):
        b.tick(shift_en=True)
    b.tick()
    check_tile(b, t_cap, tile, acc, b.writes)


def test_extra_shift_after_row0_ignored():
    n = 3
    rng = random.Random(SEED + 32)
    b = Bench(n)
    tile, acc = (0, 2, 2, 2), rand_tile(rng, n)
    t_cap = b.t + 1
    b.tick(capture=True, tile=tile, acc=acc)
    for _ in range(n + 3):                              # n+3 shifts instead of n-1
        b.tick(shift_en=True)
    check_tile(b, t_cap, tile, acc, b.writes)
    assert not b.col.active


@pytest.mark.parametrize("cut", [0, 1, 2, 3])
def test_reset_mid_drain(cut):
    n = 4
    rng = random.Random(SEED + 40 + cut)
    b = Bench(n)
    b.tick(capture=True, tile=(1, 1, n, n), acc=rand_tile(rng, n))
    for _ in range(cut):
        b.tick(shift_en=True)
    assert idle(b.col.reset(), n)
    b.writes = []
    for _ in range(n + 2):
        b.tick(shift_en=True)
    assert b.writes == [], "write after reset without a new capture"

# --------------------------------------------------------------------------- #
# 5. end to end: TileCtrlModel + FeederModel + PE grid + collector + memory
# --------------------------------------------------------------------------- #

class System:
    """tile_ctrl's validated closed-loop Harness (FSM + feeder + PE grid) with
    the collector consuming the grid's drain and writing a result memory."""
    def __init__(self, n):
        self.n = n
        self.h = Harness(n, with_grid=True)
        self.col = CollectorModel(n, MAX_M, MAX_N, ACC_WIDTH)

    def run_job(self, A, B):
        n, h = self.n, self.h
        A, B = np.asarray(A), np.asarray(B)
        M, K = A.shape
        N = B.shape[1]
        h.set_operands(A, B)
        mem = {}
        writes_per_capture = []
        t = 0
        o = h.o
        h.tick(start=True, cfg=(M, N, K))
        while True:
            # collector edge with the same PRE-edge controls the grid just used
            w = self.col.step(o["capture"], o["shift_en"], o["tile_m"], o["tile_n"],
                              o["feed_M"], o["feed_N"], drain_out=h.grid.drain_out())
            if o["capture"]:
                writes_per_capture.append(0)
            if w["wr_en"]:
                writes_per_capture[-1] += 1
                for j in range(n):
                    if w["wr_lane_en"] >> j & 1:
                        key = (w["wr_row"], w["wr_col"] + j)
                        assert key not in mem, f"C{key} written twice"
                        mem[key] = w["wr_data"][j]
                last_write = t
            if h.trace[-1]["done"]:
                break
            o = h.trace[-1]
            h.tick()
            t += 1
        return mem, M, N, t, last_write, writes_per_capture


@pytest.mark.parametrize("n", [2, 3, 4, 5])
def test_system_tiled_matmul(n):
    """Back-to-back jobs on one grid (stale accumulators in edge tiles are live),
    directed edge shapes plus random ones."""
    rng = random.Random(SEED + 100 + n)
    print(f"SEED={SEED + 100 + n}")
    s = System(n)
    shapes = [(n, n, n), (1, 1, 1), (2 * n + 1, n - 1 or 1, 3), (1, 3 * n, 2),
              (3 * n, 1, 1), (n + 1, 2 * n + 1, 4)]
    shapes += [(rng.randint(1, 3 * n + 1), rng.randint(1, 3 * n + 1), rng.randint(1, 7))
               for _ in range(12)]
    for (M, N, K) in shapes:
        A = [[rng.randint(-8, 7) for _ in range(K)] for _ in range(M)]
        B = [[rng.randint(-8, 7) for _ in range(N)] for _ in range(K)]
        mem, M, N, done_t, last_write, per_cap = s.run_job(A, B)
        assert set(mem) == {(i, j) for i in range(M) for j in range(N)}, "written region != M x N"
        C = np.array([[mem[(i, j)] for j in range(N)] for i in range(M)])
        assert np.array_equal(C, np.asarray(A) @ np.asarray(B)), f"M={M} N={N} K={K}"
        assert last_write == done_t, f"last write t={last_write}, done t={done_t}"
        assert sum(per_cap) == -(-N // n) * M, "writes per job != T_n * M"
