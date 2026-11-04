"""
cocotb testbench for "tile_ctrl" module (control FSM)
Contract: docs/fsm_contract.md

Lockstep: every cycle the SAME stimulus drives the DUT and TileCtrlModel, then
every output (and the state register) is compared after the edge (Env.tick).
feed_done comes from a FeederModel closed around the DUT's own feed_* outputs,
so the DUT runs against the validated feeder timing.

Independent of the model, three concurrent monitors sample the DUT once per
cycle (FallingEdge, so inputs and outputs are both stable):
  ProtocolMonitor     - contract §9 props 1-4, 9, legal transitions, one done per command
  TileScheduleMonitor - props 5-8, 10: tile order/bounds/dims, capture/shift_en
                        timing, cycles-per-tile formula, job latency
  CoverageMonitor     - states, transitions, dim classes, corner-case events
Coverage accumulates over the whole run; test_coverage_closure (last) fails on
any hole, so it is only meaningful for a full run (not with TESTCASE=...).

Optional env vars: SEED (default 12345); LOCKSTEP=0 disables the DUT-vs-model
compare so only the monitors judge the DUT (shows they stand on their own);
MONITORS=0 does not start the monitors (mutation attribution only; coverage
closure is skipped then).
"""

import os
import random
from collections import Counter

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer

from feeder_model import FeederModel
from tile_ctrl_model import (
    TileCtrlModel, tile_schedule, job_latency, ceil_div,
    STATES, IDLE, ISSUE, FEED, FLUSH, CAPTURE, DRAIN, DONE, REJECT,
    BUSY_STATES, ACCEPT_STATES,
)

CLK_PERIOD_NS   = 10
NUM_RANDOM_JOBS = 80
MAX_TEST_K      = 12

OUTPUTS = ("busy", "done", "error", "feed_start", "capture", "shift_en",
           "feed_M", "feed_N", "feed_K", "tile_m", "tile_n")
INPUTS = ("rst_n", "start", "M", "N", "K", "feed_done")
HELD = ("feed_M", "feed_N", "feed_K", "tile_m", "tile_n")

# contract §5 state diagram (reset -> IDLE from anywhere is handled separately)
LEGAL_TRANSITIONS = {
    (IDLE, IDLE), (IDLE, ISSUE), (IDLE, REJECT),
    (ISSUE, FEED),
    (FEED, FEED), (FEED, FLUSH),
    (FLUSH, FLUSH), (FLUSH, CAPTURE),
    (CAPTURE, DRAIN),
    (DRAIN, DRAIN), (DRAIN, ISSUE), (DRAIN, DONE),
    (DONE, IDLE), (DONE, ISSUE), (DONE, REJECT),
    (REJECT, IDLE), (REJECT, ISSUE), (REJECT, REJECT),
}

# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

def clog2(x):
    return (x - 1).bit_length()

class Params:
    def __init__(self):
        self.n     = int(os.environ["ARRAY_DIM"])
        self.max_k = int(os.environ["MAX_K"])
        self.max_m = int(os.environ["MAX_M"])
        self.max_n = int(os.environ["MAX_N"])
        self.seed  = int(os.environ.get("SEED", "12345"))
        self.lockstep = os.environ.get("LOCKSTEP", "1") != "0"
        self.monitors = os.environ.get("MONITORS", "1") != "0"
        self.m_w   = self.max_m.bit_length()        # == clog2(MAX_M + 1)
        self.n_w   = self.max_n.bit_length()
        self.k_w   = self.max_k.bit_length()
        self.dim_w = self.n.bit_length()
        self.tm_w  = max(1, clog2(ceil_div(self.max_m, self.n)))
        self.tn_w  = max(1, clog2(ceil_div(self.max_n, self.n)))

    def over_limit_representable(self):
        return (self.max_m + 1 < (1 << self.m_w) or self.max_n + 1 < (1 << self.n_w)
                or self.max_k + 1 < (1 << self.k_w))

    def check_supported(self):
        """The directed shapes go up to 3n+1 rows/cols and K = n+1; smaller limits
        cannot even be driven on the ports, so refuse them up front."""
        need = 3 * self.n + 1
        assert self.max_m >= need and self.max_n >= need and self.max_k >= self.n + 1, (
            f"testbench needs MAX_M, MAX_N >= 3*ARRAY_DIM+1 = {need} and MAX_K >= ARRAY_DIM+1; "
            f"got MAX_M={self.max_m} MAX_N={self.max_n} MAX_K={self.max_k}")

    def legal(self, M, N, K):
        return 1 <= M <= self.max_m and 1 <= N <= self.max_n and 1 <= K <= self.max_k


def read(sig, name):
    """int value, or raise on X/Z (no control output may ever be unknown)."""
    try:
        return int(sig.value)
    except ValueError:
        raise AssertionError(f"{name} is X/Z: {sig.value}")


def snapshot(dut):
    """Every port plus the state register, as plain ints / state names."""
    s = {name: read(getattr(dut, name), name) for name in INPUTS + OUTPUTS}
    code = read(dut.state, "state")
    assert code < len(STATES), f"state encoding {code} out of range"
    s["state"] = STATES[code]
    return s

# --------------------------------------------------------------------------- #
# coverage (module level: accumulates over every test in the run)
# --------------------------------------------------------------------------- #

class Coverage:
    def __init__(self):
        self.states = Counter()
        self.transitions = Counter()
        self.classes = Counter()
        self.events = Counter()

    def required_transitions(self, n):
        req = set(LEGAL_TRANSITIONS)
        if n == 2:
            req.discard((DRAIN, DRAIN))     # DRAIN is n-1 = 1 cycle
        return req

    REQUIRED_CLASSES = {
        "single_tile", "multi_tile", "M_edge_tile", "N_edge_tile", "M_and_N_edge",
        "M_exact_multiple", "N_exact_multiple", "tile_1x1", "K_1", "K_gt_n",
        "M_max", "N_max", "K_max",
    }
    REQUIRED_EVENTS = {
        "reject_zero_dim", "reject_over_limit", "start_while_busy", "back_to_back",
        "restart_from_reject", "reset_mid_job", "spurious_feed_done", "start_held",
    }

    def required_events(self, p):
        req = set(self.REQUIRED_EVENTS)
        if not p.over_limit_representable():
            req.discard("reject_over_limit")    # every MAX_* is 2^W-1
        return req

    def holes(self, p):
        return (sorted(map(str, self.required_transitions(p.n) - set(self.transitions))),
                sorted(self.REQUIRED_CLASSES - set(self.classes)),
                sorted(self.required_events(p) - set(self.events)))

    def report(self, log, n):
        log.info("---- functional coverage (cumulative) ----")
        log.info("states      : " + ", ".join(f"{s}={self.states[s]}" for s in STATES))
        req = self.required_transitions(n)
        log.info(f"transitions : {len(req & set(self.transitions))}/{len(req)} required hit")
        for (a, b) in sorted(req, key=lambda t: (STATES.index(t[0]), STATES.index(t[1]))):
            log.info(f"    {a:>7} -> {b:<7} {self.transitions[(a, b)]}")
        log.info("dim classes : " + ", ".join(f"{c}={self.classes[c]}" for c in sorted(self.REQUIRED_CLASSES)))
        log.info("events      : " + ", ".join(f"{e}={self.events[e]}" for e in sorted(self.REQUIRED_EVENTS)))

COVERAGE = Coverage()

# --------------------------------------------------------------------------- #
# monitors: concurrent, sample once per cycle at the falling edge
# --------------------------------------------------------------------------- #

class _Monitor:
    def __init__(self, dut, p):
        self.dut = dut
        self.p = p
        self.errors = []
        self.prev = None
        self.cyc = 0

    def fail(self, msg):
        """Record and raise: an exception in a forked task fails the running test."""
        err = f"[{type(self).__name__} cycle {self.cyc}] {msg}"
        self.errors.append(err)
        raise AssertionError(err)

    async def run(self):
        while True:
            await FallingEdge(self.dut.clk)
            self.cyc += 1
            try:
                cur = snapshot(self.dut)
            except AssertionError as e:
                self.fail(str(e))
            if not cur["rst_n"]:
                self.on_reset(self.prev, cur)
                self.prev = None
                continue
            self.sample(self.prev, cur)
            self.prev = cur

    def on_reset(self, prev, cur):
        pass

    @staticmethod
    def accepted(prev):
        """A start sampled on the edge after `prev` was taken (busy low)."""
        return prev is not None and prev["start"] and prev["state"] in ACCEPT_STATES


class ProtocolMonitor(_Monitor):
    def on_reset(self, prev, cur):
        """Async reset (§3): while rst_n is low everything is already idle,
        whether or not a clock edge has happened since it went low."""
        if cur["state"] != IDLE or any(cur[k] for k in OUTPUTS):
            self.fail(f"rst_n low but not idle: state={cur['state']} "
                      + ", ".join(f"{k}={cur[k]}" for k in OUTPUTS if cur[k]))

    def sample(self, prev, cur):
        p, s = self.p, cur["state"]
        if cur["busy"] != (s in BUSY_STATES):
            self.fail(f"busy={cur['busy']} in {s}")
        if cur["done"] and cur["busy"]:
            self.fail("done with busy")
        if cur["error"] and not cur["done"]:
            self.fail("error without done")
        if not cur["busy"] and (cur["feed_start"] or cur["capture"] or cur["shift_en"]):
            self.fail(f"control pulse while idle in {s}: {cur}")
        if cur["capture"] and cur["shift_en"]:
            self.fail("capture and shift_en together")
        if (cur["feed_start"], cur["capture"], cur["shift_en"]) != (s == ISSUE, s == CAPTURE, s == DRAIN):
            self.fail(f"control output outside its state {s}: {cur}")
        if (cur["done"], cur["error"]) != (s in (DONE, REJECT), s == REJECT):
            self.fail(f"done/error outside DONE/REJECT in {s}")
        if cur["busy"] and not (1 <= cur["feed_M"] <= p.n and 1 <= cur["feed_N"] <= p.n
                                and 1 <= cur["feed_K"] <= p.max_k):
            self.fail(f"tile dims out of range: M_t={cur['feed_M']} N_t={cur['feed_N']} K={cur['feed_K']}")
        if prev is None:
            return
        acc = self.accepted(prev)
        if (prev["state"], s) not in LEGAL_TRANSITIONS:
            self.fail(f"illegal transition {prev['state']} -> {s}")
        if acc:
            want = ISSUE if p.legal(prev["M"], prev["N"], prev["K"]) else REJECT
            if s != want:
                self.fail(f"start M={prev['M']} N={prev['N']} K={prev['K']} in {prev['state']} -> {s}, want {want}")
        elif prev["state"] in ACCEPT_STATES and s != IDLE:
            self.fail(f"left {prev['state']} without start -> {s}")
        if prev["done"] and cur["done"] and not acc:
            self.fail("done held two cycles for one command")
        if any(prev[k] != cur[k] for k in HELD):
            if not (s == ISSUE and prev["state"] in (ACCEPT_STATES | {DRAIN})):
                self.fail(f"held output changed on {prev['state']} -> {s}: "
                          + ", ".join(f"{k} {prev[k]}->{cur[k]}" for k in HELD if prev[k] != cur[k]))


class TileScheduleMonitor(_Monitor):
    """Checks each job against tile_schedule() (no state machine) and the
    cycles-per-tile formula written out inline."""

    def __init__(self, dut, p):
        super().__init__(dut, p)
        self.job = None
        self.jobs_checked = 0

    def on_reset(self, prev, cur):
        self.job = None

    def sample(self, prev, cur):
        p = self.p
        if self.accepted(prev) and p.legal(prev["M"], prev["N"], prev["K"]):
            M, N, K = prev["M"], prev["N"], prev["K"]
            self.job = dict(M=M, N=N, K=K, t0=self.cyc, exp=tile_schedule(M, N, K, p.n),
                            idx=-1, issue=None, dims=None, shifts=0)
        job = self.job
        if job is None:
            if cur["done"] and not cur["error"]:
                self.fail("done (no error) without a job in flight")
            return
        t = self.cyc - job["t0"]
        M, N, K = job["M"], job["N"], job["K"]
        tiles = job["exp"]["tiles"]

        if cur["busy"] and not (cur["tile_m"] < ceil_div(M, p.n) and cur["tile_n"] < ceil_div(N, p.n)):
            self.fail(f"tile index out of bounds: ({cur['tile_m']},{cur['tile_n']}) for M={M} N={N}")

        if cur["feed_start"] or cur["done"]:
            if job["issue"] is not None:        # close the previous tile: cycles-per-tile formula
                M_t, N_t = job["dims"]
                if t - job["issue"] != K + M_t + N_t + p.n + 1:
                    self.fail(f"tile {job['idx']} lasted {t - job['issue']} cycles, "
                              f"formula K+M_t+N_t+n+1 = {K + M_t + N_t + p.n + 1}")
        if cur["feed_start"]:
            job["idx"] += 1
            if job["idx"] >= len(tiles):
                self.fail(f"extra tile issued (expected {len(tiles)})")
                self.job = None
                return
            e = tiles[job["idx"]]
            got = (t, cur["tile_m"], cur["tile_n"], cur["feed_M"], cur["feed_N"], cur["feed_K"])
            want = (e["issue"], e["tile_m"], e["tile_n"], e["M_t"], e["N_t"], K)
            if got != want:
                self.fail(f"tile issue (t, tm, tn, M_t, N_t, K) got {got} want {want}")
            job["issue"], job["dims"] = t, (cur["feed_M"], cur["feed_N"])
        e = tiles[job["idx"]] if 0 <= job["idx"] < len(tiles) else None
        if cur["capture"] and (e is None or t != e["capture"]):
            self.fail(f"capture at t={t}, want {None if e is None else e['capture']}")
        if cur["shift_en"]:
            job["shifts"] += 1
            if e is None or t not in e["shift_en"]:
                self.fail(f"shift_en at t={t}, want {None if e is None else e['shift_en']}")
        if e is not None and t in e["shift_en"] and not cur["shift_en"]:
            self.fail(f"shift_en missing at t={t}")
        if cur["done"]:
            if cur["error"]:
                self.fail("error on a legal job")
            if t != job["exp"]["latency"] or t != job_latency(M, N, K, p.n):
                self.fail(f"done at t={t}, want L={job['exp']['latency']}")
            if job["idx"] != len(tiles) - 1:
                self.fail(f"done after {job['idx'] + 1} tiles, want {len(tiles)}")
            if job["shifts"] != len(tiles) * (p.n - 1):
                self.fail(f"{job['shifts']} shift_en cycles, want {len(tiles) * (p.n - 1)}")
            self.jobs_checked += 1
            self.job = None


class CoverageMonitor(_Monitor):
    def __init__(self, dut, p):
        super().__init__(dut, p)
        self.pprev = None       # snapshot before prev (for the held-start event)

    def on_reset(self, prev, cur):
        self.pprev = None
        if prev is not None and prev["busy"]:
            COVERAGE.events["reset_mid_job"] += 1

    def sample(self, prev, cur):
        pprev, self.pprev = self.pprev, prev
        p, cov = self.p, COVERAGE
        cov.states[cur["state"]] += 1
        if cur["feed_done"] and cur["state"] != FEED:
            cov.events["spurious_feed_done"] += 1
        if prev is None:
            return
        cov.transitions[(prev["state"], cur["state"])] += 1
        if prev["start"] and prev["busy"]:
            cov.events["start_while_busy"] += 1
        if not self.accepted(prev):
            return
        M, N, K = prev["M"], prev["N"], prev["K"]
        if prev["state"] == DONE:
            cov.events["back_to_back"] += 1
        if prev["state"] == REJECT and cur["state"] == ISSUE:
            cov.events["restart_from_reject"] += 1
        if prev["state"] in (DONE, REJECT) and pprev is not None and pprev["start"]:
            cov.events["start_held"] += 1
        if not p.legal(M, N, K):
            cov.events["reject_zero_dim" if 0 in (M, N, K) else "reject_over_limit"] += 1
            return
        n = p.n
        cls = cov.classes
        cls["single_tile" if (M <= n and N <= n) else "multi_tile"] += 1
        if M > n and M % n:
            cls["M_edge_tile"] += 1
        if N > n and N % n:
            cls["N_edge_tile"] += 1
        if M > n and M % n and N > n and N % n:
            cls["M_and_N_edge"] += 1
        if M > n and M % n == 0:
            cls["M_exact_multiple"] += 1
        if N > n and N % n == 0:
            cls["N_exact_multiple"] += 1
        if M % n == 1 and N % n == 1:       # some tile is 1x1
            cls["tile_1x1"] += 1
        if K == 1:
            cls["K_1"] += 1
        if K > n:
            cls["K_gt_n"] += 1
        if M == p.max_m:
            cls["M_max"] += 1
        if N == p.max_n:
            cls["N_max"] += 1
        if K == p.max_k:
            cls["K_max"] += 1

# --------------------------------------------------------------------------- #
# environment: driver + model + feeder stand-in + scoreboard, lockstep
# --------------------------------------------------------------------------- #

class Env:
    def __init__(self, dut, p):
        self.dut = dut
        self.p = p
        self.rng = random.Random(p.seed)
        self.model = TileCtrlModel(p.n, p.max_m, p.max_n, p.max_k)
        self.fdr = FeederModel(p.n, lambda k, l: 0, lambda k, l: 0)
        self.fo = self.fdr.reset()
        self.cycle = 0
        self.monitors = [ProtocolMonitor(dut, p), TileScheduleMonitor(dut, p), CoverageMonitor(dut, p)]

    def junk_cfg(self):
        p = self.p
        return (self.rng.randrange(0, 1 << p.m_w),
                self.rng.randrange(0, 1 << p.n_w),
                self.rng.randrange(0, 1 << p.k_w))

    async def tick(self, start=False, cfg=None, spurious_done=False):
        """Drive inputs for the coming edge, wait for it, step model and feeder
        stand-in with the same inputs, compare every output. When start is low,
        M/N/K carry junk: the FSM must only look at them on an accepted start."""
        dut = self.dut
        if cfg is None:
            assert not start, "start requires an explicit cfg"
            cfg = self.junk_cfg()
        M, N, K = cfg

        # feeder stand-in is driven by the DUT's own (pre-edge) feed_* outputs
        f = {k: read(getattr(dut, k), k) for k in ("feed_start", "feed_M", "feed_N", "feed_K")}
        feed_done = bool(self.fo["done"]) or (spurious_done and self.model.state != FEED)

        dut.start.value = int(start)
        dut.M.value = M
        dut.N.value = N
        dut.K.value = K
        dut.feed_done.value = int(feed_done)

        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")

        if f["feed_start"]:
            assert not self.fo["busy"], f"cycle {self.cycle}: feed_start while feeder busy"
        self.fo = self.fdr.step(start=bool(f["feed_start"]), M=f["feed_M"], N=f["feed_N"], K=f["feed_K"])
        exp = self.model.step(start=bool(start), M=M, N=N, K=K, feed_done=feed_done)
        self.cycle += 1
        self.check(exp)
        return exp

    def check(self, exp):
        if not self.p.lockstep:
            return
        got = snapshot(self.dut)
        tag = f"cycle {self.cycle}"
        assert got["state"] == exp["state"], f"{tag}: state dut={got['state']} exp={exp['state']}"
        for k in OUTPUTS:
            assert got[k] == int(exp[k]), f"{tag} [{exp['state']}]: {k} dut={got[k]} exp={int(exp[k])}"

    async def run_job(self, M, N, K, spurious_done=False):
        """Accepted start, then tick until done (bounded). Returns job latency."""
        await self.tick(start=True, cfg=(M, N, K), spurious_done=spurious_done)
        t = 0
        bound = job_latency(M, N, K, self.p.n) + 5 if self.p.legal(M, N, K) else 1
        while not self.model.outputs()["done"]:
            assert t < bound, f"job M={M} N={N} K={K} did not finish in {bound} cycles"
            await self.tick(spurious_done=spurious_done)
            t += 1
        return t

    async def idle(self, cycles, **kw):
        for _ in range(cycles):
            await self.tick(**kw)

    async def reset(self, cycles=2):
        """Asynchronous reset mid-cycle: outputs must drop BEFORE any clock edge,
        then match the model's reset snapshot. Released off-edge."""
        dut = self.dut
        await Timer(2, unit="ns")                 # mid-cycle, away from any edge
        dut.rst_n.value = 0
        dut.start.value = 0
        dut.feed_done.value = 0
        await Timer(1, unit="ns")
        exp = self.model.reset()
        self.fo = self.fdr.reset()
        self.check(exp)                           # async: already idle, no edge yet
        for _ in range(cycles):
            await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        self.check(exp)
        dut.rst_n.value = 1

    async def finish(self):
        await FallingEdge(self.dut.clk)           # let monitors see the last cycle
        COVERAGE.report(self.dut._log, self.p.n)
        self.dut._log.info(f"TileScheduleMonitor fully checked {self.monitors[1].jobs_checked} job(s)")
        errors = [e for m in self.monitors for e in m.errors]
        assert not errors, f"{len(errors)} monitor violation(s):\n" + "\n".join(errors[:20])


async def make_env(dut):
    p = Params()
    p.check_supported()
    dut._log.info(f"SEED={p.seed} LOCKSTEP={int(p.lockstep)} MONITORS={int(p.monitors)} ARRAY_DIM={p.n} MAX_M={p.max_m} MAX_N={p.max_n} MAX_K={p.max_k}")
    # stale-build guard: port widths must match what the testbench computes
    for name, w in (("M", p.m_w), ("N", p.n_w), ("K", p.k_w), ("feed_M", p.dim_w),
                    ("feed_N", p.dim_w), ("feed_K", p.k_w), ("tile_m", p.tm_w), ("tile_n", p.tn_w)):
        assert len(getattr(dut, name)) == w, f"{name} is {len(getattr(dut, name))} bits, expected {w}"
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    dut.rst_n.value = 0
    dut.start.value = 0
    dut.M.value = 0
    dut.N.value = 0
    dut.K.value = 0
    dut.feed_done.value = 0
    for _ in range(2):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.rst_n.value = 1
    env = Env(dut, p)
    if p.monitors:
        for m in env.monitors:
            cocotb.start_soon(m.run())
    return env

# --------------------------------------------------------------------------- #
# directed tests
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_reset(dut):
    """Reset state: IDLE, every output 0; junk inputs with start low keep it idle."""
    env = await make_env(dut)
    env.check(env.model.outputs())
    await env.idle(10)
    await env.finish()


@cocotb.test()
async def test_single_tile(dut):
    """Every M_t, N_t in [1, n] (full tile and 1x1 included), K in {1, 3, n+1}."""
    env = await make_env(dut)
    n = env.p.n
    for M in range(1, n + 1):
        for N in range(1, n + 1):
            for K in (1, 3, n + 1):
                await env.run_job(M, N, K)
    await env.finish()


@cocotb.test()
async def test_multi_tile(dut):
    """Exact multiples of n in M and/or N."""
    env = await make_env(dut)
    n = env.p.n
    for (M, N) in [(2 * n, 2 * n), (3 * n, n), (n, 3 * n), (2 * n, 3 * n)]:
        for K in (1, 4):
            await env.run_job(M, N, K)
    await env.finish()


@cocotb.test()
async def test_edge_tiles(dut):
    """M and/or N not a multiple of n: last tile row/col has M_t/N_t < n."""
    env = await make_env(dut)
    n = env.p.n
    for (M, N) in [(n + 1, n), (n, n + 1), (2 * n + 1, 2 * n - 1), (2 * n - 1, 3 * n + 1),
                   (1, 3 * n + 1), (3 * n + 1, 1), (n + 1, n + 1)]:
        for K in (1, 3):
            await env.run_job(M, N, K)
    await env.finish()


@cocotb.test()
async def test_boundary_dims(dut):
    """Largest legal M, N, K are accepted and follow the formula."""
    env = await make_env(dut)
    p = env.p
    await env.run_job(p.max_m, p.max_n, 1)
    await env.run_job(1, 1, p.max_k)
    await env.run_job(p.max_m, 1, 2)
    await env.run_job(1, p.max_n, 2)
    await env.finish()


@cocotb.test()
async def test_illegal_commands(dut):
    """Zero and over-limit dims: REJECT, done & error for one cycle, nothing
    issued, held outputs untouched (a legal job runs first so they are nonzero)."""
    env = await make_env(dut)
    p = env.p
    top_m, top_n, top_k = (1 << p.m_w) - 1, (1 << p.n_w) - 1, (1 << p.k_w) - 1
    cfgs = [(0, 1, 1), (1, 0, 1), (1, 1, 0), (0, 0, 0),
            (p.max_m + 1, 1, 1), (1, p.max_n + 1, 1), (1, 1, p.max_k + 1),
            (top_m, 1, 1), (1, top_n, 1), (1, 1, top_k), (top_m, top_n, top_k)]
    # a MAX_* of 2^W-1 makes MAX_*+1 unrepresentable on the port; top_* still runs
    cfgs = [c for c in cfgs if c[0] <= top_m and c[1] <= top_n and c[2] <= top_k]
    cfgs = [c for c in cfgs if not p.legal(*c)]
    for cfg in cfgs:
        await env.run_job(p.n + 2, 2 * p.n - 1, 3)
        await env.idle(2)
        exp = await env.tick(start=True, cfg=cfg)
        assert exp["state"] == REJECT
        await env.idle(3)
    await env.finish()


@cocotb.test()
async def test_reject_then_restart(dut):
    """REJECT samples start like IDLE: illegal -> REJECT again, legal -> ISSUE.
    DONE does too: an illegal start in the DONE cycle goes straight to REJECT."""
    env = await make_env(dut)
    await env.tick(start=True, cfg=(0, 1, 1))
    await env.tick(start=True, cfg=(1, 1, 0))
    await env.run_job(3, 2 * env.p.n + 1, 2)
    exp = await env.tick(start=True, cfg=(0, 0, 0))     # DONE -> REJECT
    assert exp["state"] == REJECT
    await env.idle(2)
    await env.finish()


@cocotb.test()
async def test_back_to_back(dut):
    """Next start lands in the DONE cycle, different dims each time."""
    env = await make_env(dut)
    n = env.p.n
    for (M, N, K) in [(2 * n + 1, n - 1, 3), (1, 1, 1), (2 * n, 2 * n, 4),
                      (n - 1, 3 * n, 1), (n, n, 2)]:
        await env.run_job(M, N, K)          # no idle gap between jobs
    await env.finish()


@cocotb.test()
async def test_start_held_high(dut):
    """Contract §8: start held high (same cfg) relaunches from the DONE cycle."""
    env = await make_env(dut)
    n = env.p.n
    cfg = (n + 1, 2, 2)
    for _ in range(3 * (job_latency(*cfg, n) + 1)):
        await env.tick(start=True, cfg=cfg)
    await env.finish()


@cocotb.test()
async def test_start_while_busy(dut):
    """start (different legal cfg) on each cycle t=1..L+1 of a job, fresh reset
    per offset. Busy cycles must be ignored; the DONE-cycle start is accepted."""
    env = await make_env(dut)
    n = env.p.n
    job1 = (n + 2, n + 1, 3)
    job2 = (n - 1, 2 * n + 1, 2)
    L = job_latency(*job1, n)
    for e in range(1, L + 2):
        await env.reset()
        await env.tick(start=True, cfg=job1)
        for t in range(1, L + 2):
            await env.tick(start=(t == e), cfg=job2 if t == e else None)
        assert (env.model.state == ISSUE) == (e == L + 1), f"e={e}"
    await env.finish()


@cocotb.test()
async def test_spurious_feed_done(dut):
    """feed_done high in every non-FEED cycle must not change anything."""
    env = await make_env(dut)
    n = env.p.n
    await env.idle(3, spurious_done=True)
    await env.run_job(2 * n + 1, n + 2, 3, spurious_done=True)
    await env.idle(3, spurious_done=True)
    await env.finish()


@cocotb.test()
async def test_reset_mid_run(dut):
    """Async reset at every cycle offset of a multi-tile job (so in every state),
    then a clean job afterwards."""
    env = await make_env(dut)
    n = env.p.n
    job = (n + 1, n + 2, 2)
    for cut in range(0, job_latency(*job, n) + 1):
        await env.tick(start=True, cfg=job)
        await env.idle(cut)
        await env.reset()
        await env.idle(2)
    await env.run_job(2, 2 * n + 1, 3)
    await env.finish()

# --------------------------------------------------------------------------- #
# constrained random
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_random(dut):
    """Random dims (mostly legal, some illegal), K, idle gaps (incl. 0 =
    back-to-back), start pokes while busy, spurious feed_done, rare resets.
    Reproduce with SEED=<n>."""
    env = await make_env(dut)
    p, rng = env.p, env.rng
    n = p.n

    def rand_cfg():
        if rng.random() < 0.1:
            return env.junk_cfg()               # often illegal
        return (rng.randint(1, min(p.max_m, 4 * n + 1)),
                rng.randint(1, min(p.max_n, 4 * n + 1)),
                rng.randint(1, min(MAX_TEST_K, p.max_k)))

    for _ in range(NUM_RANDOM_JOBS):
        await env.idle(rng.choice([0, 0, 1, 2, 3]))
        await env.tick(start=True, cfg=rand_cfg())
        while not env.model.outputs()["done"]:
            poke = rng.random() < 0.05
            await env.tick(start=poke, cfg=rand_cfg() if poke else None,
                           spurious_done=rng.random() < 0.2)
            if rng.random() < 0.002:
                await env.reset()
                break
    await env.idle(2)
    await env.finish()

# --------------------------------------------------------------------------- #
# coverage closure (must stay the LAST test)
# --------------------------------------------------------------------------- #

@cocotb.test(skip=os.environ.get("MONITORS", "1") == "0")
async def test_coverage_closure(dut):
    """Every required transition, dim class and event was hit by the run."""
    p = Params()
    COVERAGE.report(dut._log, p.n)
    tr, cl, ev = COVERAGE.holes(p)
    assert not (tr or cl or ev), f"coverage holes:\n transitions {tr}\n classes {cl}\n events {ev}"
