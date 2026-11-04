"""
Standalone pytest for TileCtrlModel (no RTL):  pytest test_tile_ctrl_model.py

Three independent references are checked against the model:
  1. tile_schedule() / job_latency(): contract formulas, no state machine
  2. FeederModel in closed loop: feed_done comes from the validated feeder
     model, which also asserts every feeder job the FSM issues is legal
  3. a grid of PEModels: the whole tiled matmul is run end to end (feeder ->
     grid -> capture -> drain) and the drained tiles are compared to numpy.
     This proves the CAPTURE cycle (zero slack) and the drain window are right.

Every harness run also checks the per-cycle protocol properties (contract §9).
"""

import os
import random

import numpy as np
import pytest

from feeder_model import FeederModel
from pe_model import PEModel
from tile_ctrl_model import (
    TileCtrlModel, tile_schedule, job_latency, ceil_div,
    IDLE, ISSUE, FEED, FLUSH, CAPTURE, DRAIN, DONE, REJECT,
    BUSY_STATES, ACCEPT_STATES,
)

SEED = int(os.environ.get("SEED", "20261103"))
MAX_M = 64
MAX_N = 64
MAX_K = 1024
DATA_WIDTH = 8
ACC_WIDTH = 32

# --------------------------------------------------------------------------- #
# grid of PEModels: same wiring as pe_grid.sv
# --------------------------------------------------------------------------- #

class PEGridModel:
    def __init__(self, n):
        self.n = n
        self.pe = [[PEModel(DATA_WIDTH, ACC_WIDTH) for _ in range(n)] for _ in range(n)]

    def drain_out(self):
        return [self.pe[self.n - 1][j].out_shadow_signed for j in range(self.n)]

    def step(self, a_in, b_in, capture, shift_en):
        """a_in[i]: None | (data, first); b_in[j]: None | (data,) -- FeederModel lane format."""
        n = self.n
        ins = [[None] * n for _ in range(n)]
        for i in range(n):          # gather every PE's pre-edge inputs first
            for j in range(n):
                if j == 0:
                    a = a_in[i]
                    a_d, a_v, f = (a[0], 1, int(a[1])) if a is not None else (0, 0, 0)
                else:
                    p = self.pe[i][j - 1]
                    a_d, a_v, f = p.out_a, p.out_a_valid, p.out_first
                if i == 0:
                    b = b_in[j]
                    b_d, b_v = (b[0], 1) if b is not None else (0, 0)
                    sh = 0
                else:
                    p = self.pe[i - 1][j]
                    b_d, b_v, sh = p.out_b, p.out_b_valid, p.out_shadow
                ins[i][j] = (a_d, b_d, a_v, b_v, f, sh)
        for i in range(n):
            for j in range(n):
                a_d, b_d, a_v, b_v, f, sh = ins[i][j]
                self.pe[i][j].step(a_d, b_d, a_v, b_v, in_first=f,
                                   capture=int(capture), shift_en=int(shift_en), in_shadow=sh)

# --------------------------------------------------------------------------- #
# closed-loop harness: FSM model + FeederModel (+ optional grid), lockstep
# --------------------------------------------------------------------------- #

class Harness:
    def __init__(self, n, with_grid=False, max_m=MAX_M, max_n=MAX_N, max_k=MAX_K):
        self.n = n
        self.fsm = TileCtrlModel(n, max_m, max_n, max_k)
        self.fdr = FeederModel(n, self._buf_a, self._buf_b)
        self.grid = PEGridModel(n) if with_grid else None
        self.A = None
        self.B = None
        self.C = None
        self.reset()

    # operand buffer stub: tile ids select the operand tile (contract §4.4)
    def _buf_a(self, k, lane):
        return 0 if self.A is None else int(self.A[self._tile[0] * self.n + lane][k])

    def _buf_b(self, k, lane):
        return 0 if self.B is None else int(self.B[k][self._tile[1] * self.n + lane])

    def reset(self):
        self.o = self.fsm.reset()
        self.fo = self.fdr.reset()
        self.collect = None
        self.trace = []

    def set_operands(self, A, B):
        """Only call while idle (like OperandMemory.randomize in test_feeder)."""
        self.A, self.B = np.asarray(A), np.asarray(B)
        self.C = np.zeros((self.A.shape[0], self.B.shape[1]), dtype=object)

    def tick(self, start=False, cfg=(0, 0, 0), spurious_done=False):
        """One edge for every block, each fed the others' PRE-edge outputs."""
        o, fo = self.o, self.fo
        M, N, K = cfg

        # feeder must be idle whenever the FSM issues (contract §6.1)
        if o["feed_start"]:
            assert not fo["busy"], "feed_start while feeder busy"

        # contract §9.11: the real feeder's done only ever arrives in FEED
        assert not fo["done"] or o["state"] == FEED, f"feed_done in {o['state']}"

        feed_done = fo["done"] or (spurious_done and o["state"] != FEED)
        new_o = self.fsm.step(start=start, M=M, N=N, K=K, feed_done=feed_done)

        self._tile = (o["tile_m"], o["tile_n"])
        new_fo = self.fdr.step(start=o["feed_start"], M=o["feed_M"], N=o["feed_N"], K=o["feed_K"])

        if self.grid is not None:
            self.grid.step(fo["out_a"], fo["out_b"], o["capture"], o["shift_en"])
            self._collect(o)

        check_cycle(o, new_o, self.n, accepted=start and o["state"] in ACCEPT_STATES)
        self.o, self.fo = new_o, new_fo
        self.trace.append(new_o)
        return new_o

    def _collect(self, o):
        """drain_out shows row n-1 post capture-edge, then one row up per shift."""
        if o["capture"]:
            assert self.collect is None
            self.collect = dict(tm=o["tile_m"], tn=o["tile_n"], M_t=o["feed_M"],
                                N_t=o["feed_N"], r=self.n - 1, rows={})
        c = self.collect
        if c is None:
            return
        c["rows"][c["r"]] = self.grid.drain_out()
        c["r"] -= 1
        if c["r"] < 0:
            for i in range(c["M_t"]):
                for j in range(c["N_t"]):
                    self.C[c["tm"] * self.n + i][c["tn"] * self.n + j] = c["rows"][i][j]
            self.collect = None

    def run_job(self, M, N, K, extra=0, **kw):
        """Accepted start + ticks until done, then `extra` idle ticks.
        Returns the job's trace with t=0 at the ISSUE cycle."""
        t0 = len(self.trace)
        self.tick(start=True, cfg=(M, N, K), **kw)
        guard = job_latency(M, N, K, self.n) + 5
        while not self.trace[-1]["done"]:
            assert len(self.trace) - t0 < guard, "job did not finish"
            self.tick(**kw)
        for _ in range(extra):
            self.tick(**kw)
        return self.trace[t0:]


# --------------------------------------------------------------------------- #
# property checks
# --------------------------------------------------------------------------- #

HELD = ("feed_M", "feed_N", "feed_K", "tile_m", "tile_n")

def check_cycle(prev, cur, n, accepted):
    """Contract §9 properties 1-4, 9 between two consecutive post-edge snapshots.
    `accepted`: a start was sampled on this edge while busy was low."""
    s = cur["state"]
    assert cur["busy"] == (s in BUSY_STATES)
    assert not (cur["done"] and cur["busy"])
    # one done per command: back-to-back done only if this edge took a new (rejected) command
    assert not (prev["done"] and cur["done"]) or accepted
    assert not cur["error"] or cur["done"]
    if not cur["busy"]:
        assert not (cur["feed_start"] or cur["capture"] or cur["shift_en"])
    assert not (cur["capture"] and cur["shift_en"])
    if cur["busy"]:
        assert 1 <= cur["feed_M"] <= n and 1 <= cur["feed_N"] <= n and cur["feed_K"] >= 1
    if any(prev[k] != cur[k] for k in HELD):
        assert s == ISSUE and prev["state"] in (ACCEPT_STATES | {DRAIN}), \
            f"held output changed on {prev['state']}->{s}"


def extract(trace):
    """Observed schedule of one job trace (t=0 = ISSUE) in tile_schedule's format."""
    tiles = []
    done_t = None
    for t, o in enumerate(trace):
        if o["feed_start"]:
            tiles.append(dict(tile_m=o["tile_m"], tile_n=o["tile_n"],
                              M_t=o["feed_M"], N_t=o["feed_N"], issue=t, shift_en=[]))
        if o["capture"]:
            tiles[-1]["capture"] = t
        if o["shift_en"]:
            tiles[-1]["shift_en"].append(t)
        if o["done"] and done_t is None:
            done_t = t
    for a, b in zip(tiles, tiles[1:] + [dict(issue=done_t)]):
        a["length"] = b["issue"] - a["issue"]
    return dict(tiles=tiles, latency=done_t)


def check_job(trace, M, N, K, n):
    exp = tile_schedule(M, N, K, n)
    got = extract(trace)
    assert got == exp, f"M={M} N={N} K={K} n={n}\n got={got}\n exp={exp}"
    assert got["latency"] == job_latency(M, N, K, n)
    assert all(o["feed_K"] == K for o in trace[:-1])
    done = [t for t, o in enumerate(trace) if o["done"]]
    assert done == [got["latency"]], "exactly one done, at t = L"
    assert not trace[got["latency"]]["error"]


# --------------------------------------------------------------------------- #
# 0. the pure helpers agree with each other and with the contract examples
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("n", [2, 3, 4, 5, 8])
def test_schedule_closed_form(n):
    for M in range(1, 3 * n + 2):
        for N in range(1, 3 * n + 2):
            for K in (1, 2, 7):
                s = tile_schedule(M, N, K, n)
                assert s["latency"] == job_latency(M, N, K, n)
                assert len(s["tiles"]) == ceil_div(M, n) * ceil_div(N, n)
                assert sum(t["M_t"] for t in s["tiles"] if t["tile_n"] == 0) == M
                assert sum(t["N_t"] for t in s["tiles"] if t["tile_m"] == 0) == N


def test_contract_worked_examples():
    """docs/fsm_contract.md §6.1 and §6.2, literally."""
    s = tile_schedule(2, 3, 3, 4)
    assert s["latency"] == 13 and s["tiles"][0]["capture"] == 9
    assert s["tiles"][0]["shift_en"] == [10, 11, 12]

    s = tile_schedule(6, 5, 3, 4)
    assert s["latency"] == 54
    assert [(t["tile_m"], t["tile_n"], t["M_t"], t["N_t"], t["length"], t["issue"])
            for t in s["tiles"]] == [(0, 0, 4, 4, 16, 0), (0, 1, 4, 1, 13, 16),
                                     (1, 0, 2, 4, 14, 29), (1, 1, 2, 1, 11, 43)]

# --------------------------------------------------------------------------- #
# 1. reset and single tile
# --------------------------------------------------------------------------- #

def test_reset_outputs():
    m = TileCtrlModel(4, MAX_M, MAX_N, MAX_K)
    o = m.reset()
    assert o["state"] == IDLE
    assert all(not o[k] for k in ("busy", "done", "error", "feed_start", "capture", "shift_en"))
    assert all(o[k] == 0 for k in HELD)
    # idles with no start
    for _ in range(5):
        assert m.step() == o


def test_single_tile_state_sequence():
    """§6.1 waveform: n=4, K=3, M_t=2, N_t=3."""
    h = Harness(4)
    tr = h.run_job(2, 3, 3, extra=1)
    states = [o["state"] for o in tr]
    assert states == ([ISSUE] + [FEED] * 4 + [FLUSH] * 4 + [CAPTURE] + [DRAIN] * 3
                      + [DONE, IDLE])
    check_job(tr, 2, 3, 3, 4)


@pytest.mark.parametrize("n", [2, 3, 4, 5])
def test_single_tile_all_shapes(n):
    """Every M_t, N_t in [1, n] incl. full tile and 1x1, several K."""
    h = Harness(n)
    for M in range(1, n + 1):
        for N in range(1, n + 1):
            for K in (1, 2, 5):
                check_job(h.run_job(M, N, K), M, N, K, n)

# --------------------------------------------------------------------------- #
# 2. multi-tile and edge tiles
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("n", [2, 3, 4, 5])
def test_multi_tile_and_edge_tiles(n):
    """Exact multiples, one-over, one-under, rectangular, skinny."""
    h = Harness(n)
    shapes = [(2 * n, 2 * n), (2 * n + 1, n), (n, 2 * n + 1), (2 * n - 1, 3 * n - 1),
              (1, 3 * n), (3 * n, 1), (n + 1, n + 1)]
    for (M, N) in shapes:
        for K in (1, 4):
            check_job(h.run_job(M, N, K), M, N, K, n)


def test_boundary_dims():
    """Largest legal M, N and K are accepted and run to the formula."""
    n = 4
    h = Harness(n)
    check_job(h.run_job(MAX_M, MAX_N, 1), MAX_M, MAX_N, 1, n)
    check_job(h.run_job(1, 1, MAX_K), 1, 1, MAX_K, n)
    check_job(h.run_job(MAX_M, 1, 3), MAX_M, 1, 3, n)

# --------------------------------------------------------------------------- #
# 3. degenerate / illegal commands
# --------------------------------------------------------------------------- #

def illegal_cfgs(m):
    top_m, top_n, top_k = (1 << m.m_w) - 1, (1 << m.n_w) - 1, (1 << m.k_w) - 1
    return [(0, 1, 1), (1, 0, 1), (1, 1, 0), (0, 0, 0),
            (MAX_M + 1, 1, 1), (1, MAX_N + 1, 1), (1, 1, MAX_K + 1),
            (top_m, 1, 1), (1, top_n, 1), (1, 1, top_k), (top_m, top_n, top_k)]


def test_illegal_commands_rejected():
    """REJECT for one cycle: done & error at t=0, busy never rises, nothing
    issued, held outputs untouched (a legal job runs first so they are nonzero)."""
    h = Harness(4)
    for cfg in illegal_cfgs(h.fsm):
        h.run_job(5, 6, 3, extra=2)
        before = {k: h.o[k] for k in HELD}
        o = h.tick(start=True, cfg=cfg)
        assert o["state"] == REJECT and o["done"] and o["error"] and not o["busy"], cfg
        assert {k: o[k] for k in HELD} == before
        for _ in range(3):
            o = h.tick()
            assert o["state"] == IDLE and not o["done"] and not o["feed_start"]


def test_reject_then_restart():
    """REJECT accepts start like IDLE: illegal -> REJECT again, legal -> ISSUE."""
    h = Harness(4)
    h.tick(start=True, cfg=(0, 1, 1))
    assert h.tick(start=True, cfg=(1, 1, 0))["state"] == REJECT
    t0 = len(h.trace)
    h.tick(start=True, cfg=(3, 7, 2))
    assert h.trace[t0]["state"] == ISSUE
    while not h.trace[-1]["done"]:
        h.tick()
    check_job(h.trace[t0:], 3, 7, 2, 4)


def test_input_masking():
    """Ports are M_W/N_W/K_W bits; bits above them are invisible to the DUT."""
    m = TileCtrlModel(4, MAX_M, MAX_N, MAX_K)
    o = m.step(start=True, M=5 + (1 << m.m_w), N=3, K=2)
    assert o["state"] == ISSUE and o["feed_M"] == 4

# --------------------------------------------------------------------------- #
# 4. start while busy, back-to-back, held start
# --------------------------------------------------------------------------- #

def test_start_while_busy_ignored():
    """start (different legal cfg) on every cycle t=1..L+1 of a job. Busy
    cycles must not perturb the job; the DONE-cycle start (t=L+1 edge) is accepted."""
    n = 4
    M1, N1, K1 = 6, 5, 3
    M2, N2, K2 = 3, 9, 2
    L = job_latency(M1, N1, K1, n)
    for e in range(1, L + 2):
        h = Harness(n)
        h.tick(start=True, cfg=(M1, N1, K1))
        for t in range(1, L + 2):
            h.tick(start=(t == e), cfg=(M2, N2, K2) if t == e else (0, 0, 0))
        check_job(h.trace[:L + 1], M1, N1, K1, n)
        accepted = e == L + 1
        assert (h.trace[L + 1]["state"] == ISSUE) == accepted, f"e={e}"
        if accepted:
            while not h.trace[-1]["done"]:
                h.tick()
            check_job(h.trace[L + 1:], M2, N2, K2, n)


def test_back_to_back_jobs():
    """Next start in the DONE cycle, different dims each time: config reloads."""
    n = 4
    h = Harness(n)
    jobs = [(6, 5, 3), (1, 1, 1), (8, 8, 4), (3, 9, 1), (4, 4, 2)]
    t0 = 0
    for (M, N, K) in jobs:
        h.tick(start=True, cfg=(M, N, K))       # issued while previous job's DONE is visible
        while not h.trace[-1]["done"]:
            h.tick()
        check_job(h.trace[t0:], M, N, K, n)
        t0 = len(h.trace)


def test_start_held_high():
    """Contract §8: start held high launches a new job every L+1 cycles."""
    n = 3
    h = Harness(n)
    M, N, K = 4, 2, 2
    L = job_latency(M, N, K, n)
    for _ in range(3 * (L + 1)):
        h.tick(start=True, cfg=(M, N, K))
    for j in range(3):
        check_job(h.trace[j * (L + 1):(j + 1) * (L + 1)], M, N, K, n)

# --------------------------------------------------------------------------- #
# 5. robustness: spurious feed_done, reset mid-run
# --------------------------------------------------------------------------- #

def test_feed_done_outside_feed_ignored():
    n = 4
    ref = Harness(n).run_job(7, 6, 3, extra=3)
    h = Harness(n)
    h.tick(spurious_done=True)                  # in IDLE too
    got = h.run_job(7, 6, 3, extra=3, spurious_done=True)
    assert got == ref


@pytest.mark.parametrize("cut", [0, 1, 4, 9, 17, 30])
def test_reset_mid_run(cut):
    """Reset at any point: idle snapshot, everything zero, next job is clean."""
    n = 4
    h = Harness(n)
    h.tick(start=True, cfg=(6, 5, 3))
    for _ in range(cut):
        h.tick()
    h.reset()
    assert h.o == TileCtrlModel(n, MAX_M, MAX_N, MAX_K).reset()
    for _ in range(n + 3):
        assert h.tick()["state"] == IDLE
    check_job(h.run_job(5, 2, 2), 5, 2, 2, n)

# --------------------------------------------------------------------------- #
# 6. random closed-loop schedule check
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("n", [2, 3, 4, 5])
def test_random_jobs(n):
    rng = random.Random(SEED + n)
    print(f"SEED={SEED + n}")
    h = Harness(n)
    for _ in range(60):
        M = rng.randint(1, 4 * n)
        N = rng.randint(1, 4 * n)
        K = rng.randint(1, 9)
        for _ in range(rng.randint(0, 3)):
            h.tick()
        check_job(h.run_job(M, N, K), M, N, K, n)

# --------------------------------------------------------------------------- #
# 7. end to end: tiled matmul through FeederModel + PEModel grid vs numpy
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("n", [2, 3, 4, 5])
def test_tiled_matmul_end_to_end(n):
    """Consecutive jobs on ONE grid (no reset between), so stale accumulators
    in edge tiles are exercised; only the M_t x N_t region is collected."""
    rng = np.random.default_rng(SEED + 100 + n)
    print(f"SEED={SEED + 100 + n}")
    h = Harness(n, with_grid=True)
    shapes = [(n, n, n), (2 * n + 1, n - 1, 3), (1, 3 * n, 2), (3 * n, 1, 1)]
    shapes += [tuple(int(x) for x in rng.integers(1, 3 * n + 1, size=2)) + (int(rng.integers(1, 8)),)
               for _ in range(8)]
    for (M, N, K) in shapes:
        A = rng.integers(-8, 8, size=(M, K))
        B = rng.integers(-8, 8, size=(K, N))
        h.set_operands(A, B)
        h.run_job(M, N, K, extra=int(rng.integers(0, 3)))
        assert h.collect is None, "drain window still open after done"
        exp = A @ B
        assert np.array_equal(h.C.astype(np.int64), exp), \
            f"M={M} N={N} K={K}\n got=\n{h.C}\n exp=\n{exp}"
