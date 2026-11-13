"""
cocotb testbench for "tile_ctrl" v2 (control FSM with drain/feed overlap)
Contract: docs/fsm_contract.md (v2)

Lockstep: every cycle the SAME stimulus drives the DUT and TileCtrlModel, then
every output (and the front-FSM state register) is compared after the edge
(Env.tick). feed_done comes from a FeederModel closed around the DUT's own
feed_* outputs, so the DUT runs against the validated feeder timing.

Independent of the model, three concurrent monitors sample the DUT once per
cycle (FallingEdge, so inputs and outputs are both stable):
  ProtocolMonitor     - §9 props 1-4, 10: busy/done/error, pulses only while busy,
                        capture never with shift_en, legal front transitions,
                        the §4.6 stability rule for the feed-side and cap_* outputs
  TileScheduleMonitor - props 5-9, 11: every feed_start / capture / shift_en / done
                        cycle against tile_schedule(), plus the same timing re-derived
                        inline from observed values only: capture K+M_t+N_t+1 after
                        its feed_start, cap_* naming the tile issued, ISSUE gaps from
                        the §6.2 terms, captures >= n+1 apart (across jobs too),
                        done n cycles after the last capture
  CoverageMonitor     - front states and transitions, dim classes, which §6.2 term
                        set each gap, overlap events, corner-case events
Coverage accumulates over the whole run; test_coverage_closure (last) fails on
any hole, so it is only meaningful for a full run (not with TESTCASE=...).
Required transitions and gap terms are those the §6.2 schedule can reach at
this ARRAY_DIM over the shapes test_all_small_jobs drives.

Optional env vars: SEED (default 12345); LOCKSTEP=0 disables the DUT-vs-model
compare so only the monitors judge the DUT; MONITORS=0 does not start the
monitors (mutation attribution only; coverage closure is skipped then).
"""

import os
import random
from collections import Counter, deque

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer

from feeder_model import FeederModel
from tile_ctrl_model import (
    TileCtrlModel, tile_schedule, job_latency, ceil_div,
    STATES, IDLE, ISSUE, FEED, WAIT, TAIL, DONE, REJECT,
    BUSY_STATES, ACCEPT_STATES, FEED_SIDE, CAP_SIDE,
)

CLK_PERIOD_NS   = 10
NUM_RANDOM_JOBS = 80
MAX_TEST_K      = 12
SMALL_K         = (1, 2, 3, 7)      # test_all_small_jobs: every binding-term mix

OUTPUTS = ("busy", "done", "error", "feed_start", "capture", "shift_en") + FEED_SIDE + CAP_SIDE
INPUTS = ("rst_n", "start", "M", "N", "K", "feed_done")

# contract §5.1 state diagram (reset -> IDLE from anywhere is handled separately)
LEGAL_TRANSITIONS = {
    (IDLE, IDLE), (IDLE, ISSUE), (IDLE, REJECT),
    (ISSUE, FEED),
    (FEED, FEED), (FEED, ISSUE), (FEED, WAIT), (FEED, TAIL),
    (WAIT, WAIT), (WAIT, ISSUE),
    (TAIL, TAIL), (TAIL, DONE),
    (DONE, IDLE), (DONE, ISSUE), (DONE, REJECT),
    (REJECT, IDLE), (REJECT, ISSUE), (REJECT, REJECT),
}

# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

def clog2(x):
    return (x - 1).bit_length()


def gap_terms(K, n, M_t, N_t, M_t2, N_t2):
    """§6.2, written out again here (independent of tile_ctrl_model.tile_gap)."""
    return dict(a=K + 2, b=M_t + N_t + max(K - 2, 0), c=n + 1 + M_t + N_t - M_t2 - N_t2)


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
        """The directed shapes go up to 3n+1 rows/cols and K = max(n+1, 7); smaller
        limits cannot even be driven on the ports, so refuse them up front."""
        need = 3 * self.n + 1
        assert self.max_m >= need and self.max_n >= need and self.max_k >= max(self.n + 1, max(SMALL_K)), (
            f"testbench needs MAX_M, MAX_N >= 3*ARRAY_DIM+1 = {need} and MAX_K >= max(ARRAY_DIM+1, "
            f"{max(SMALL_K)}); got MAX_M={self.max_m} MAX_N={self.max_n} MAX_K={self.max_k}")

    def legal(self, M, N, K):
        return 1 <= M <= self.max_m and 1 <= N <= self.max_n and 1 <= K <= self.max_k

    def small_jobs(self):
        n = self.n
        return [(M, N, K) for M in range(1, 2 * n + 2) for N in range(1, 2 * n + 2) for K in SMALL_K]


def read(sig, name):
    """int value, or raise on X/Z (no control output may ever be unknown)."""
    try:
        return int(sig.value)
    except ValueError:
        raise AssertionError(f"{name} is X/Z: {sig.value}")


def snapshot(dut):
    """Every port plus the front-FSM state register, as plain ints / state names."""
    s = {name: read(getattr(dut, name), name) for name in INPUTS + OUTPUTS}
    code = read(dut.state, "state")
    assert code < len(STATES), f"state encoding {code} out of range"
    s["state"] = STATES[code]
    return s

# --------------------------------------------------------------------------- #
# coverage (module level: accumulates over every test in the run)
# --------------------------------------------------------------------------- #

def reachable(p):
    """Front transitions and §6.2 gap terms the contract schedule produces at
    this n over the shapes test_all_small_jobs drives. Pure schedule, no model."""
    trans, terms = set(), set()
    for (M, N, K) in p.small_jobs():
        tiles = tile_schedule(M, N, K, p.n)["tiles"]
        for t in tiles[:-1]:
            g = t["gap"]
            trans.add((FEED, ISSUE) if g == K + 2 else (FEED, WAIT))
            if g >= K + 4:
                trans.add((WAIT, WAIT))         # WAIT lasts g - (K+2) cycles
            if g >= K + 3:
                trans.add((WAIT, ISSUE))
            terms |= {k for k, v in t["terms"].items() if v == g}
    return trans, terms


class Coverage:
    def __init__(self):
        self.states = Counter()
        self.transitions = Counter()
        self.classes = Counter()
        self.events = Counter()

    def required_transitions(self, p):
        trans, _ = reachable(p)
        optional = {(FEED, ISSUE), (WAIT, WAIT), (WAIT, ISSUE), (FEED, WAIT)}
        return (LEGAL_TRANSITIONS - optional) | trans

    REQUIRED_CLASSES = {
        "single_tile", "multi_tile", "M_edge_tile", "N_edge_tile", "M_and_N_edge",
        "M_exact_multiple", "N_exact_multiple", "tile_1x1", "K_1", "K_2", "K_gt_n",
        "M_max", "N_max", "K_max",
    }
    REQUIRED_EVENTS = {
        "reject_zero_dim", "reject_over_limit", "start_while_busy", "back_to_back",
        "restart_from_reject", "reset_mid_job", "reset_in_wait", "reset_in_tail",
        "reset_capture_pending", "spurious_feed_done", "start_held",
        # overlap: the reasons v2 exists
        "issue_capture_pending",    # feed_start while an earlier tile is not yet captured
        "capture_during_feed",      # capture while the next tile is in FEED
        "shift_during_feed",        # a drain shift while the next tile is in FEED
        "capture_on_handoff",       # capture in a feed_done cycle (slot freed and refilled on one edge)
    }

    def required_events(self, p):
        req = set(self.REQUIRED_EVENTS)
        if not p.over_limit_representable():
            req.discard("reject_over_limit")    # every MAX_* is 2^W-1
        _, terms = reachable(p)
        req |= {f"gap_{t}" for t in terms}
        return req

    def holes(self, p):
        return (sorted(map(str, self.required_transitions(p) - set(self.transitions))),
                sorted(self.REQUIRED_CLASSES - set(self.classes)),
                sorted(self.required_events(p) - set(self.events)))

    def report(self, log, p):
        log.info("---- functional coverage (cumulative) ----")
        log.info("states      : " + ", ".join(f"{s}={self.states[s]}" for s in STATES))
        req = self.required_transitions(p)
        log.info(f"transitions : {len(req & set(self.transitions))}/{len(req)} required hit")
        for (a, b) in sorted(LEGAL_TRANSITIONS, key=lambda t: (STATES.index(t[0]), STATES.index(t[1]))):
            log.info(f"    {a:>7} -> {b:<7} {self.transitions[(a, b)]}{'' if (a, b) in req else '  (not reachable at this n)'}")
        log.info("dim classes : " + ", ".join(f"{c}={self.classes[c]}" for c in sorted(self.REQUIRED_CLASSES)))
        log.info("events      : " + ", ".join(f"{e}={self.events[e]}" for e in sorted(self.required_events(p))))

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
        if cur["feed_start"] != (s == ISSUE):
            self.fail(f"feed_start={cur['feed_start']} in {s}")
        if (cur["done"], cur["error"]) != (s in (DONE, REJECT), s == REJECT):
            self.fail(f"done/error outside DONE/REJECT in {s}")
        if cur["busy"] and not (1 <= cur["feed_M"] <= p.n and 1 <= cur["feed_N"] <= p.n
                                and 1 <= cur["feed_K"] <= p.max_k):
            self.fail(f"tile dims out of range: M_t={cur['feed_M']} N_t={cur['feed_N']} K={cur['feed_K']}")
        if prev is None:
            return
        ps = prev["state"]
        acc = self.accepted(prev)
        if (ps, s) not in LEGAL_TRANSITIONS:
            self.fail(f"illegal transition {ps} -> {s}")
        if acc:
            want = ISSUE if p.legal(prev["M"], prev["N"], prev["K"]) else REJECT
            if s != want:
                self.fail(f"start M={prev['M']} N={prev['N']} K={prev['K']} in {ps} -> {s}, want {want}")
        elif ps in ACCEPT_STATES and s != IDLE:
            self.fail(f"left {ps} without start -> {s}")
        if prev["done"] and cur["done"] and not acc:
            self.fail("done held two cycles for one command")
        if ps == FEED and s != FEED and not prev["feed_done"]:
            self.fail(f"left FEED -> {s} without feed_done")
        if ps == FEED and s == FEED and prev["feed_done"]:
            self.fail("feed_done in FEED but no handoff")
        # §4.6 stability
        load = ps in ACCEPT_STATES and s == ISSUE
        advance = ps == FEED and s in (ISSUE, WAIT)
        handoff = ps == FEED and s in (ISSUE, WAIT, TAIL)
        moved = [k for k in FEED_SIDE if prev[k] != cur[k]]
        if moved and not (load or advance):
            self.fail(f"feed-side output changed on {ps} -> {s}: "
                      + ", ".join(f"{k} {prev[k]}->{cur[k]}" for k in moved))
        moved = [k for k in CAP_SIDE if prev[k] != cur[k]]
        if moved and not handoff:
            self.fail(f"cap_* changed on {ps} -> {s}: " + ", ".join(f"{k} {prev[k]}->{cur[k]}" for k in moved))
        if handoff:
            # cap_* take the feed-side values of the feed_done cycle
            got = tuple(cur[k] for k in CAP_SIDE)
            want = (prev["tile_m"], prev["tile_n"], prev["feed_M"], prev["feed_N"])
            if got != want:
                self.fail(f"handoff loaded cap_*={got}, feed side was {want}")


class TileScheduleMonitor(_Monitor):
    """Each job against tile_schedule() (no state machine), and the same timing
    re-derived inline from the DUT's own observed values."""

    def __init__(self, dut, p):
        super().__init__(dut, p)
        self.job = None
        self.jobs_checked = 0
        self.last_cap = None        # cycle of the latest capture, across jobs

    def on_reset(self, prev, cur):
        self.job = None
        self.last_cap = None

    def sample(self, prev, cur):
        p, n = self.p, self.p.n
        if self.accepted(prev) and p.legal(prev["M"], prev["N"], prev["K"]):
            if self.job is not None:
                self.fail("new job accepted while one is in flight")
            M, N, K = prev["M"], prev["N"], prev["K"]
            exp = tile_schedule(M, N, K, n)
            ts = exp["tiles"]
            self.job = dict(
                M=M, N=N, K=K, t0=self.cyc, L=exp["latency"],
                issues={t["issue"]: t for t in ts},
                caps={t["capture"]: t for t in ts},
                shifts={x for t in ts for x in t["shift_en"]},
                n_tiles=len(ts),
                issued=[], pending=deque(), captured=0, shifts_seen=0)
        job = self.job

        if cur["capture"]:
            if self.last_cap is not None and self.cyc - self.last_cap < n + 1:
                self.fail(f"capture {self.cyc - self.last_cap} cycles after the previous one, want >= n+1 = {n + 1}")
            self.last_cap = self.cyc

        if job is None:
            if cur["done"] and not cur["error"]:
                self.fail("done (no error) without a job in flight")
            if cur["capture"] or cur["shift_en"] or cur["feed_start"]:
                self.fail("control pulse without a job in flight")
            return

        t = self.cyc - job["t0"]
        M, N, K = job["M"], job["N"], job["K"]
        if cur["busy"] and not (cur["tile_m"] < ceil_div(M, n) and cur["tile_n"] < ceil_div(N, n)):
            self.fail(f"tile index out of bounds: ({cur['tile_m']},{cur['tile_n']}) for M={M} N={N}")

        # ---- feed_start: against the schedule, then the gap re-derived inline
        e = job["issues"].get(t)
        if cur["feed_start"] != (e is not None):
            self.fail(f"feed_start={cur['feed_start']} at t={t}, schedule issues at {sorted(job['issues'])}")
        if cur["feed_start"]:
            ids = (cur["tile_m"], cur["tile_n"], cur["feed_M"], cur["feed_N"])
            if ids != (e["tile_m"], e["tile_n"], e["M_t"], e["N_t"]) or cur["feed_K"] != K:
                self.fail(f"tile issued {ids} K={cur['feed_K']}, want {e}")
            if job["issued"]:
                t_prev, prev_ids = job["issued"][-1]
                terms = gap_terms(K, n, prev_ids[2], prev_ids[3], ids[2], ids[3])
                if t - t_prev != max(terms.values()):
                    self.fail(f"ISSUE gap {t - t_prev}, §6.2 terms {terms}")
                COVERAGE.events.update(f"gap_{k}" for k, v in terms.items() if v == t - t_prev)
                if job["pending"]:
                    COVERAGE.events["issue_capture_pending"] += 1
            job["issued"].append((t, ids))
            job["pending"].append((t, ids))

        # ---- capture: schedule, inline offset from its own feed_start, cap_* identity
        e = job["caps"].get(t)
        if cur["capture"] != (e is not None):
            self.fail(f"capture={cur['capture']} at t={t}, schedule captures at {sorted(job['caps'])}")
        if cur["capture"]:
            if not job["pending"]:
                self.fail("capture with no issued tile pending")
            t_iss, ids = job["pending"].popleft()
            if t - t_iss != K + ids[2] + ids[3] + 1:
                self.fail(f"capture {t - t_iss} cycles after its feed_start, want K+M_t+N_t+1 = {K + ids[2] + ids[3] + 1}")
            got = tuple(cur[k] for k in CAP_SIDE)
            if got != ids:
                self.fail(f"cap_* = {got} at capture, tile issued as {ids}")
            job["captured"] += 1
            if cur["state"] == FEED:
                COVERAGE.events["capture_during_feed"] += 1
                if cur["feed_done"]:
                    COVERAGE.events["capture_on_handoff"] += 1

        # ---- shift_en: exactly the n-1 cycles after each capture
        want_shift = t in job["shifts"]
        if cur["shift_en"] != want_shift:
            self.fail(f"shift_en={cur['shift_en']} at t={t}, want {int(want_shift)}")
        if cur["shift_en"]:
            job["shifts_seen"] += 1
            if cur["state"] == FEED:
                COVERAGE.events["shift_during_feed"] += 1

        # ---- done
        if cur["done"]:
            if cur["error"]:
                self.fail("error on a legal job")
            if t != job["L"] or t != job_latency(M, N, K, n):
                self.fail(f"done at t={t}, want L={job['L']}")
            if self.last_cap is None or self.cyc - self.last_cap != n:
                self.fail(f"done {None if self.last_cap is None else self.cyc - self.last_cap} cycles after the last capture, want n = {n}")
            if len(job["issued"]) != job["n_tiles"] or job["captured"] != job["n_tiles"]:
                self.fail(f"done after {len(job['issued'])} issues / {job['captured']} captures, want {job['n_tiles']}")
            if job["shifts_seen"] != job["n_tiles"] * (n - 1):
                self.fail(f"{job['shifts_seen']} shift_en cycles, want {job['n_tiles'] * (n - 1)}")
            self.jobs_checked += 1
            self.job = None
        elif t > job["L"]:
            self.fail(f"no done by t={t}, L={job['L']}")


class CoverageMonitor(_Monitor):
    def __init__(self, dut, p):
        super().__init__(dut, p)
        self.pprev = None       # snapshot before prev (for the held-start event)
        self.pending = 0        # issued tiles not yet captured (for reset coverage)

    def on_reset(self, prev, cur):
        self.pprev = None
        if prev is not None and prev["busy"]:
            COVERAGE.events["reset_mid_job"] += 1
            if prev["state"] == WAIT:
                COVERAGE.events["reset_in_wait"] += 1
            if prev["state"] == TAIL:
                COVERAGE.events["reset_in_tail"] += 1
            if self.pending:
                COVERAGE.events["reset_capture_pending"] += 1
        self.pending = 0

    def sample(self, prev, cur):
        pprev, self.pprev = self.pprev, prev
        p, cov = self.p, COVERAGE
        cov.states[cur["state"]] += 1
        self.pending += int(cur["feed_start"]) - int(cur["capture"])
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
        if K == 2:
            cls["K_2"] += 1
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
            # §9.6e: never in a feeder busy or done cycle
            assert not self.fo["busy"] and not self.fo["done"], \
                f"cycle {self.cycle}: feed_start while feeder busy/done"
            if not (1 <= f["feed_M"] <= self.p.n and 1 <= f["feed_N"] <= self.p.n and f["feed_K"] >= 1):
                raise AssertionError(f"cycle {self.cycle}: illegal feeder job {f}")
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
        COVERAGE.report(self.dut._log, self.p)
        self.dut._log.info(f"TileScheduleMonitor fully checked {self.monitors[1].jobs_checked} job(s)")
        errors = [e for m in self.monitors for e in m.errors]
        assert not errors, f"{len(errors)} monitor violation(s):\n" + "\n".join(errors[:20])


async def make_env(dut):
    p = Params()
    p.check_supported()
    dut._log.info(f"SEED={p.seed} LOCKSTEP={int(p.lockstep)} MONITORS={int(p.monitors)} ARRAY_DIM={p.n} MAX_M={p.max_m} MAX_N={p.max_n} MAX_K={p.max_k}")
    # stale-build guard: port widths must match what the testbench computes
    for name, w in (("M", p.m_w), ("N", p.n_w), ("K", p.k_w), ("feed_M", p.dim_w),
                    ("feed_N", p.dim_w), ("feed_K", p.k_w), ("tile_m", p.tm_w), ("tile_n", p.tn_w),
                    ("cap_tile_m", p.tm_w), ("cap_tile_n", p.tn_w), ("cap_M", p.dim_w), ("cap_N", p.dim_w)):
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
    """Every M_t, N_t in [1, n] (full tile and 1x1 included), K in {1, 3, n+1}:
    FEED -> TAIL -> DONE, v1 latency."""
    env = await make_env(dut)
    n = env.p.n
    for M in range(1, n + 1):
        for N in range(1, n + 1):
            for K in (1, 3, n + 1):
                await env.run_job(M, N, K)
    await env.finish()


@cocotb.test()
async def test_contract_example(dut):
    """docs/fsm_contract.md §6.2 example shape (6 x 5, K = 3), here at this n,
    plus full-tile rows where term (b) binds at K = 1, 2 and > 2."""
    env = await make_env(dut)
    n = env.p.n
    await env.run_job(6, 5, 3)
    for K in (1, 2, 3, n + 1):
        await env.run_job(n, 3 * n, K)
    await env.finish()


@cocotb.test()
async def test_multi_tile(dut):
    """Exact multiples of n in M and/or N."""
    env = await make_env(dut)
    n = env.p.n
    for (M, N) in [(2 * n, 2 * n), (3 * n, n), (n, 3 * n), (2 * n, 3 * n)]:
        for K in (1, 2, 4):
            await env.run_job(M, N, K)
    await env.finish()


@cocotb.test()
async def test_edge_tiles(dut):
    """M and/or N not a multiple of n: shrinking tiles, where term (c) binds."""
    env = await make_env(dut)
    n = env.p.n
    for (M, N) in [(n + 1, n), (n, n + 1), (2 * n + 1, 2 * n - 1), (2 * n - 1, 3 * n + 1),
                   (1, 3 * n + 1), (3 * n + 1, 1), (n + 1, n + 1)]:
        for K in (1, 3):
            await env.run_job(M, N, K)
    await env.finish()


@cocotb.test()
async def test_all_small_jobs(dut):
    """Every M, N in [1, 2n+1] for K in {1, 2, 3, 7}, back to back: every mix of
    binding §6.2 terms this n can produce (the coverage targets come from here)."""
    env = await make_env(dut)
    for (M, N, K) in env.p.small_jobs():
        await env.run_job(M, N, K)
    await env.finish()


@cocotb.test()
async def test_boundary_dims(dut):
    """Largest legal M, N, K are accepted and follow the schedule."""
    env = await make_env(dut)
    p = env.p
    await env.run_job(p.max_m, p.max_n, 1)
    await env.run_job(1, 1, p.max_k)
    await env.run_job(p.max_m, 1, 2)
    await env.run_job(1, p.max_n, 2)
    await env.run_job(p.n + 1, p.n + 2, p.max_k)
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
    """Next start lands in the DONE cycle, different dims each time; captures
    stay n+1 apart across the job boundary (TileScheduleMonitor)."""
    env = await make_env(dut)
    n = env.p.n
    for (M, N, K) in [(2 * n + 1, n - 1, 3), (1, 1, 1), (2 * n, 2 * n, 4),
                      (n - 1, 3 * n, 1), (n, n, 2), (3 * n, 2, 1)]:
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
    per offset. Busy cycles (incl. WAIT and TAIL) must be ignored; the
    DONE-cycle start is accepted."""
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
    """feed_done high in every non-FEED cycle (ISSUE, WAIT, TAIL, idle) must not
    change anything."""
    env = await make_env(dut)
    n = env.p.n
    await env.idle(3, spurious_done=True)
    await env.run_job(2 * n + 1, n + 2, 3, spurious_done=True)
    await env.run_job(n, 3 * n, 1, spurious_done=True)
    await env.idle(3, spurious_done=True)
    await env.finish()


@cocotb.test()
async def test_reset_mid_run(dut):
    """Async reset at every cycle offset of a multi-tile job (so in every state,
    with a capture pending and mid-drain), then a clean job afterwards. K = 1
    too, because at n = 2 only K = 1 jobs ever enter WAIT."""
    env = await make_env(dut)
    n = env.p.n
    for job in ((n + 1, n + 2, 2), (n + 1, n + 2, 1)):
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
    """Every required transition, dim class, gap term and event was hit by the run."""
    p = Params()
    COVERAGE.report(dut._log, p)
    tr, cl, ev = COVERAGE.holes(p)
    assert not (tr or cl or ev), f"coverage holes:\n transitions {tr}\n classes {cl}\n events {ev}"
