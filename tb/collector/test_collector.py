"""
cocotb testbench for "collector" module (drain -> row writes of C)
Contract: docs/collector_contract.md

Lockstep: every cycle the SAME control inputs drive the DUT and CollectorModel.
drain_out is driven right after each edge from a ShadowChain stand-in (as
pe_grid's shadow registers would change), then every write-port output is
compared (Env.tick).

Stimulus comes from two sources:
  - raw control sequences (directed + random): stalls, mid-drain captures,
    capture with shift_en, junk on the tile inputs between captures
  - the real FSM schedule: TileCtrlModel + FeederModel in closed loop, so
    tile_m/tile_n/rows/cols move on the row-0 cycle exactly as in the system

Independent of the model, concurrent monitors sample once per cycle (FallingEdge):
  WritePortMonitor - contract §7.1, 3, 4: idle outputs zero, thermometer lane
                     mask, bounds, wr_data == drain_out, no write before a capture,
                     idle during reset
  TileWriteMonitor - §7.2, 5: per capture, re-derives from the observed capture /
                     shift_en cycles exactly which grid row must be written on
                     which cycle, at which row/col, with which mask
  CoverageMonitor  - tile shapes and drain-pattern events
Coverage accumulates over the whole run; test_coverage_closure (last) fails on
any hole, so it is only meaningful for a full run (not with TESTCASE=...).

Optional env vars: SEED (default 12345); LOCKSTEP=0 disables the DUT-vs-model
compare; MONITORS=0 does not start the monitors (mutation attribution only).
"""

import os
import random
from collections import Counter

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer

from collector_model import CollectorModel, expected_tile_writes
from shadow_chain_model import ShadowChain
from feeder_model import FeederModel
from tile_ctrl_model import TileCtrlModel
from utils import to_signed

CLK_PERIOD_NS = 10
NUM_RANDOM_CYCLES = 4000

TILE_IN = ("tile_m", "tile_n", "tile_rows", "tile_cols")
WR_SCALAR = ("wr_en", "wr_row", "wr_tile_n", "wr_lane_en")

# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

def clog2(x):
    return (x - 1).bit_length()

def ceil_div(a, b):
    return -(-a // b)

class Params:
    def __init__(self):
        self.n         = int(os.environ["ARRAY_DIM"])
        self.acc_width = int(os.environ["ACC_WIDTH"])
        self.max_m     = int(os.environ["MAX_M"])
        self.max_n     = int(os.environ["MAX_N"])
        self.seed      = int(os.environ.get("SEED", "12345"))
        self.lockstep  = os.environ.get("LOCKSTEP", "1") != "0"
        self.monitors  = os.environ.get("MONITORS", "1") != "0"
        self.tiles_m   = ceil_div(self.max_m, self.n)
        self.tiles_n   = ceil_div(self.max_n, self.n)
        self.tm_w      = max(1, clog2(self.tiles_m))
        self.tn_w      = max(1, clog2(self.tiles_n))
        self.dim_w     = self.n.bit_length()
        self.row_w     = max(1, clog2(self.max_m))
        self.acc_mask  = (1 << self.acc_width) - 1


def read(sig, name):
    """int value, or raise on X/Z."""
    try:
        return int(sig.value)
    except ValueError:
        raise AssertionError(f"{name} is X/Z: {sig.value}")


def snapshot(dut, p):
    s = {k: read(getattr(dut, k), k) for k in ("rst_n", "capture", "shift_en") + TILE_IN + WR_SCALAR}
    s["drain_out"] = [read(dut.drain_out[j], f"drain_out[{j}]") for j in range(p.n)]
    s["wr_data"] = [to_signed(read(dut.wr_data[j], f"wr_data[{j}]"), p.acc_width) for j in range(p.n)]
    return s

# --------------------------------------------------------------------------- #
# coverage (module level: accumulates over every test in the run)
# --------------------------------------------------------------------------- #

class Coverage:
    REQUIRED_SHAPES = {"rows_1", "rows_n", "rows_mid", "cols_1", "cols_n", "cols_mid",
                       "tile_1x1", "tile_full"}
    REQUIRED_EVENTS = {"v1_drain", "fsm_driven_drain", "tile_inputs_move_mid_drain",
                       "stall_mid_drain", "capture_mid_drain", "capture_with_shift",
                       "shift_while_idle", "extra_shift_after_row0", "reset_mid_drain",
                       "negative_data_written", "extreme_data_written", "multi_tile_row_col"}

    def __init__(self):
        self.shapes = Counter()
        self.events = Counter()

    def holes(self, p):
        req_shapes = set(self.REQUIRED_SHAPES)
        if p.n == 2:
            req_shapes -= {"rows_mid", "cols_mid"}      # no size strictly between 1 and 2
        req_events = set(self.REQUIRED_EVENTS)
        if p.tiles_m == 1 or p.tiles_n == 1:
            req_events.discard("multi_tile_row_col")    # MAX_M / MAX_N fit in one tile
        return (sorted(req_shapes - set(self.shapes)), sorted(req_events - set(self.events)))

    def report(self, log):
        log.info("---- functional coverage (cumulative) ----")
        log.info("tile shapes : " + ", ".join(f"{k}={self.shapes[k]}" for k in sorted(self.REQUIRED_SHAPES)))
        log.info("events      : " + ", ".join(f"{k}={self.events[k]}" for k in sorted(self.REQUIRED_EVENTS)))

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
                cur = snapshot(self.dut, self.p)
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


class WritePortMonitor(_Monitor):
    def __init__(self, dut, p):
        super().__init__(dut, p)
        self.captured = False       # a capture has been sampled since reset

    def on_reset(self, prev, cur):
        self.captured = False
        if any(cur[k] for k in WR_SCALAR) or any(cur["wr_data"]):
            self.fail(f"rst_n low but write port not idle: {[(k, cur[k]) for k in WR_SCALAR]}")

    def sample(self, prev, cur):
        p, n = self.p, self.p.n
        if not cur["wr_en"]:
            if any(cur[k] for k in WR_SCALAR) or any(cur["wr_data"]):
                self.fail("write port not all-zero without wr_en: "
                          + ", ".join(f"{k}={cur[k]}" for k in WR_SCALAR))
        else:
            if not self.captured:
                self.fail("wr_en before any capture since reset")
            lanes = cur["wr_lane_en"]
            if lanes == 0 or lanes & (lanes + 1) or lanes >= (1 << n):
                self.fail(f"wr_lane_en {lanes:0{n}b} is not a non-empty thermometer code")
            width = bin(lanes).count("1")
            if not cur["wr_row"] < p.max_m:
                self.fail(f"wr_row {cur['wr_row']} >= MAX_M {p.max_m}")
            if cur["wr_tile_n"] >= p.tiles_n or cur["wr_tile_n"] * n + width > p.max_n:
                self.fail(f"wr_tile_n {cur['wr_tile_n']} (+{width} lanes) not a tile column inside MAX_N")
            exp = [to_signed(d, p.acc_width) for d in cur["drain_out"]]
            if cur["wr_data"] != exp:
                self.fail(f"wr_data {cur['wr_data']} != drain_out {exp}")
        if cur["capture"]:
            self.captured = True    # takes effect from the next cycle (sampled on the coming edge)


class TileWriteMonitor(_Monitor):
    """Per capture: from the observed capture/shift_en cycles, row r is first on
    drain_out at v[n-1] = capture+1 and v[r-1] = (cycle of the next sampled shift) + 1.
    A write of row r is due exactly at v[r] iff r < rows; nothing else may be written."""

    def __init__(self, dut, p):
        super().__init__(dut, p)
        self.rec = None
        self.tiles_checked = 0

    def on_reset(self, prev, cur):
        self._close()

    def _close(self):
        rec, self.rec = self.rec, None
        if rec is None:
            return
        self.tiles_checked += 1

    def sample(self, prev, cur):
        n = self.p.n
        rec = self.rec
        if rec is not None:
            # write check for THIS cycle against the row due now
            due = rec["due"].pop(self.cyc, None)
            if cur["wr_en"]:
                tm, tn, rows, cols = rec["tile"]
                if due is None:
                    self.fail(f"unexpected write row {cur['wr_row']} (tile {rec['tile']}, no row due)")
                want = (tm * n + due, tn, (1 << cols) - 1)
                got = (cur["wr_row"], cur["wr_tile_n"], cur["wr_lane_en"])
                if got != want:
                    self.fail(f"write (row, col, lanes) got {got} want {want} for grid row {due}")
            elif due is not None:
                self.fail(f"missing write of grid row {due} (tile {rec['tile']})")
            # the row now visible advances on the coming edge if shift_en is sampled
            if cur["shift_en"] and not cur["capture"] and rec["row"] > 0:
                rec["row"] -= 1
                if rec["row"] < rec["tile"][2]:
                    rec["due"][self.cyc + 1] = rec["row"]
        elif cur["wr_en"]:
            self.fail(f"write row {cur['wr_row']} with no capture open")
        if cur["capture"]:
            self._close()
            tile = tuple(cur[k] for k in TILE_IN)
            self.rec = dict(tile=tile, row=n - 1, due={})
            if n - 1 < tile[2]:
                self.rec["due"][self.cyc + 1] = n - 1


class CoverageMonitor(_Monitor):
    def __init__(self, dut, p):
        super().__init__(dut, p)
        self.open = None            # dict(t0, shifts, latched, stalled, moved) while a drain is incomplete
        self.seen_tm = set()
        self.seen_tn = set()

    def on_reset(self, prev, cur):
        if self.open is not None:
            COVERAGE.events["reset_mid_drain"] += 1
        self.open = None

    def sample(self, prev, cur):
        n, cov = self.p.n, COVERAGE
        o = self.open
        if cur["wr_en"]:
            if any(v < 0 for v in cur["wr_data"]):
                cov.events["negative_data_written"] += 1
            top = (1 << (self.p.acc_width - 1))
            if any(v in (-top, top - 1) for v in cur["wr_data"]):
                cov.events["extreme_data_written"] += 1
        if o is not None and not cur["capture"]:
            if tuple(cur[k] for k in TILE_IN) != o["latched"]:
                o["moved"] = True
            if cur["shift_en"]:
                o["shifts"] += 1
                if o["shifts"] > n - 1:
                    cov.events["extra_shift_after_row0"] += 1
            elif o["shifts"] < n - 1:
                o["stalled"] = True
            if o["shifts"] >= n - 1 and not cur["shift_en"]:
                if not o["stalled"]:
                    cov.events["v1_drain"] += 1
                if o["moved"]:
                    cov.events["tile_inputs_move_mid_drain"] += 1
                if o["stalled"]:
                    cov.events["stall_mid_drain"] += 1
                self.open = None
        elif o is None and cur["shift_en"] and not cur["capture"]:
            cov.events["shift_while_idle"] += 1
        if cur["capture"]:
            if self.open is not None:
                cov.events["capture_mid_drain"] += 1
            if cur["shift_en"]:
                cov.events["capture_with_shift"] += 1
            tm, tn, rows, cols = (cur[k] for k in TILE_IN)
            cov.shapes["rows_1" if rows == 1 else "rows_n" if rows == n else "rows_mid"] += 1
            cov.shapes["cols_1" if cols == 1 else "cols_n" if cols == n else "cols_mid"] += 1
            if rows == 1 and cols == 1:
                cov.shapes["tile_1x1"] += 1
            if rows == n and cols == n:
                cov.shapes["tile_full"] += 1
            self.seen_tm.add(tm)
            self.seen_tn.add(tn)
            if len(self.seen_tm) > 1 and len(self.seen_tn) > 1:
                cov.events["multi_tile_row_col"] += 1
            self.open = dict(shifts=0, latched=(tm, tn, rows, cols), stalled=False, moved=False)

# --------------------------------------------------------------------------- #
# stimulus: the real FSM schedule (tile_ctrl model closed with the feeder model)
# --------------------------------------------------------------------------- #

class FsmSource:
    def __init__(self, p):
        self.fsm = TileCtrlModel(p.n, p.max_m, p.max_n, 1024)
        self.fdr = FeederModel(p.n, lambda k, l: 0, lambda k, l: 0)
        self.o = self.fsm.reset()
        self.fo = self.fdr.reset()

    def controls(self):
        """tile_ctrl's outputs now = the collector's inputs for the coming edge."""
        o = self.o
        return o["capture"], o["shift_en"], (o["tile_m"], o["tile_n"], o["feed_M"], o["feed_N"])

    def advance(self, start=False, cfg=(0, 0, 0)):
        o, fo = self.o, self.fo
        self.fo = self.fdr.step(start=o["feed_start"], M=o["feed_M"], N=o["feed_N"], K=o["feed_K"])
        self.o = self.fsm.step(start=start, M=cfg[0], N=cfg[1], K=cfg[2], feed_done=fo["done"])

# --------------------------------------------------------------------------- #
# environment: driver + model + shadow chain + scoreboard, lockstep
# --------------------------------------------------------------------------- #

class Env:
    def __init__(self, dut, p):
        self.dut = dut
        self.p = p
        self.rng = random.Random(p.seed)
        self.model = CollectorModel(p.n, p.max_m, p.max_n, p.acc_width)
        self.chain = ShadowChain(p.n)
        self.src = FsmSource(p)
        self.cycle = 0
        self.monitors = [WritePortMonitor(dut, p), TileWriteMonitor(dut, p), CoverageMonitor(dut, p)]

    def junk_tile(self):
        p = self.p
        return (self.rng.randrange(0, 1 << p.tm_w), self.rng.randrange(0, 1 << p.tn_w),
                self.rng.randrange(0, 1 << p.dim_w), self.rng.randrange(0, 1 << p.dim_w))

    def legal_tile(self):
        """A tile tile_ctrl could produce: indices in range, dims fit inside MAX_M / MAX_N."""
        p, rng = self.p, self.rng
        tm = rng.randrange(0, p.tiles_m)
        tn = rng.randrange(0, p.tiles_n)
        return (tm, tn, rng.randint(1, min(p.n, p.max_m - tm * p.n)),
                rng.randint(1, min(p.n, p.max_n - tn * p.n)))

    def rand_acc(self):
        p, rng = self.p, self.rng
        top = 1 << (p.acc_width - 1)
        pick = lambda: rng.choice([top, top - 1, p.acc_mask]) if rng.random() < 0.1 else rng.getrandbits(p.acc_width)
        return [[pick() for _ in range(p.n)] for _ in range(p.n)]

    async def tick(self, capture=False, shift_en=False, tile=None, acc=None):
        """Drive controls for the coming edge (junk tile inputs unless given),
        cross it, drive the shadow chain's post-edge drain_out, step the model
        with the same values, compare the write port."""
        dut = self.dut
        if tile is None:
            tile = self.junk_tile()
        dut.capture.value = int(capture)
        dut.shift_en.value = int(shift_en)
        for k, v in zip(TILE_IN, tile):
            getattr(dut, k).value = v

        await RisingEdge(dut.clk)
        if capture and acc is None:
            acc = self.rand_acc()
        d = self.chain.step(capture, shift_en, acc)
        for j, v in enumerate(d):
            dut.drain_out[j].value = v
        await Timer(1, unit="ns")

        exp = self.model.step(capture, shift_en, *tile, drain_out=d)
        self.cycle += 1
        self.check(exp)
        return exp

    def check(self, exp):
        if not self.p.lockstep:
            return
        got = snapshot(self.dut, self.p)
        tag = f"cycle {self.cycle}"
        for k in WR_SCALAR:
            assert got[k] == int(exp[k]), f"{tag}: {k} dut={got[k]} exp={int(exp[k])}"
        assert got["wr_data"] == exp["wr_data"], f"{tag}: wr_data dut={got['wr_data']} exp={exp['wr_data']}"

    async def v1_drain(self, tile, acc=None):
        """capture, n-1 shifts, then row 0's cycle: exactly what tile_ctrl does."""
        await self.tick(capture=True, tile=tile, acc=acc)
        for _ in range(self.p.n - 1):
            await self.tick(shift_en=True)
        await self.tick()

    async def run_fsm_job(self, M, N, K):
        """Collector driven by the real FSM schedule for one job (start -> done)."""
        start = True
        while True:
            capture, shift_en, tile = self.src.controls()
            if capture:
                COVERAGE.events["fsm_driven_drain"] += 1
            await self.tick(capture, shift_en, tile)
            self.src.advance(start=start, cfg=(M, N, K))
            start = False
            if self.src.o["done"]:
                return

    async def idle(self, cycles):
        for _ in range(cycles):
            await self.tick()

    async def reset(self, cycles=2):
        """Asynchronous reset mid-cycle: the write port must be idle BEFORE any
        clock edge, then match the model's reset snapshot. Released off-edge.
        The shadow chain (pe_grid) shares rst_n in the system, so it resets too."""
        dut = self.dut
        await Timer(2, unit="ns")
        dut.rst_n.value = 0
        dut.capture.value = 0
        dut.shift_en.value = 0
        await Timer(1, unit="ns")
        exp = self.model.reset()
        self.check(exp)                           # async: already idle, no edge yet
        for _ in range(cycles):
            await RisingEdge(dut.clk)
        for j, v in enumerate(self.chain.reset()):
            dut.drain_out[j].value = v
        await Timer(1, unit="ns")
        self.check(self.model.outputs(self.chain.drain_out()))
        dut.rst_n.value = 1

    async def finish(self):
        await FallingEdge(self.dut.clk)
        COVERAGE.report(self.dut._log)
        self.dut._log.info(f"TileWriteMonitor checked {self.monitors[1].tiles_checked} tile drain(s)")
        errors = [e for m in self.monitors for e in m.errors]
        assert not errors, f"{len(errors)} monitor violation(s):\n" + "\n".join(errors[:20])


async def make_env(dut):
    p = Params()
    dut._log.info(f"SEED={p.seed} LOCKSTEP={int(p.lockstep)} MONITORS={int(p.monitors)} "
                  f"ARRAY_DIM={p.n} ACC_WIDTH={p.acc_width} MAX_M={p.max_m} MAX_N={p.max_n}")
    # stale-build guard: port widths must match what the testbench computes
    for name, w in (("tile_m", p.tm_w), ("tile_n", p.tn_w), ("tile_rows", p.dim_w), ("tile_cols", p.dim_w),
                    ("wr_row", p.row_w), ("wr_tile_n", p.tn_w), ("wr_lane_en", p.n)):
        assert len(getattr(dut, name)) == w, f"{name} is {len(getattr(dut, name))} bits, expected {w}"
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    dut.rst_n.value = 0
    dut.capture.value = 0
    dut.shift_en.value = 0
    for k in TILE_IN:
        getattr(dut, k).value = 0
    for j in range(p.n):
        dut.drain_out[j].value = 0
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
    """Reset: write port idle; shift_en and junk tile inputs alone never write."""
    env = await make_env(dut)
    env.check(env.model.outputs([0] * env.p.n))
    for _ in range(10):
        await env.tick(shift_en=env.rng.random() < 0.5)
    await env.finish()


@cocotb.test()
async def test_single_tile_all_shapes(dut):
    """Every M_t x N_t in [1, n]^2 with v1 drain timing; junk on the tile inputs
    after each capture, so the latch is exercised on every tile."""
    env = await make_env(dut)
    p = env.p
    for rows in range(1, p.n + 1):
        for cols in range(1, p.n + 1):
            tm = env.rng.randint(0, (p.max_m - rows) // p.n)    # tm*n + rows <= MAX_M, as from tile_ctrl
            tn = env.rng.randint(0, (p.max_n - cols) // p.n)
            await env.v1_drain((tm, tn, rows, cols))
            await env.idle(env.rng.randint(0, 2))
    await env.finish()


@cocotb.test()
async def test_signed_extremes(dut):
    """drain_out bits are reinterpreted as signed: INT_MIN, INT_MAX, -1 pass through."""
    env = await make_env(dut)
    p = env.p
    top = 1 << (p.acc_width - 1)
    acc = [[(top, top - 1, p.acc_mask)[(i + j) % 3] for j in range(p.n)] for i in range(p.n)]
    await env.v1_drain((0, 0, p.n, p.n), acc)
    await env.finish()


@cocotb.test()
async def test_fsm_driven_jobs(dut):
    """The real tile_ctrl schedule, back-to-back jobs incl. edge tiles: tile
    inputs move on every row-0 cycle, so a collector that does not latch fails."""
    env = await make_env(dut)
    n = env.p.n
    for (M, N, K) in [(n, n, 2), (1, 1, 1), (2 * n + 1, n - 1, 3), (1, 3 * n, 1),
                      (3 * n, 1, 2), (n + 1, 2 * n + 1, 1), (2 * n, 2 * n, 1)]:
        await env.run_fsm_job(M, N, K)
    await env.idle(3)
    await env.finish()


@cocotb.test()
async def test_stalled_drain(dut):
    """Idle cycles between shifts (D2): each kept row written once, on the first
    cycle it is on drain_out."""
    env = await make_env(dut)
    for _ in range(25):
        await env.tick(capture=True, tile=env.legal_tile())
        for _ in range(env.p.n - 1):
            await env.idle(env.rng.randint(0, 3))
            await env.tick(shift_en=True)
        await env.idle(env.rng.randint(1, 3))
    await env.finish()


@cocotb.test()
async def test_capture_during_drain(dut):
    """A new capture at every point of a running drain restarts on the new tile."""
    env = await make_env(dut)
    for cut in range(env.p.n + 1):
        await env.tick(capture=True, tile=env.legal_tile())
        for _ in range(cut):
            await env.tick(shift_en=True)
        await env.v1_drain(env.legal_tile())
        await env.idle(2)
    await env.finish()


@cocotb.test()
async def test_capture_with_shift(dut):
    """capture and shift_en in the same cycle: capture wins (mirrors pe)."""
    env = await make_env(dut)
    for _ in range(5):
        await env.tick(capture=True, shift_en=True, tile=env.legal_tile())
        for _ in range(env.p.n - 1):
            await env.tick(shift_en=True)
        await env.idle(2)
    await env.finish()


@cocotb.test()
async def test_idle_and_extra_shifts(dut):
    """shift_en while idle, and shifts beyond n-1, are ignored."""
    env = await make_env(dut)
    for _ in range(5):
        await env.tick(shift_en=True)
    for _ in range(5):
        await env.tick(capture=True, tile=env.legal_tile())
        for _ in range(env.p.n + 3):
            await env.tick(shift_en=True)
    await env.finish()


@cocotb.test()
async def test_reset_mid_drain(dut):
    """Async reset at every drain offset: idle before any edge, no write after,
    then a clean drain."""
    env = await make_env(dut)
    for cut in range(env.p.n + 1):
        await env.tick(capture=True, tile=env.legal_tile())
        for _ in range(cut):
            await env.tick(shift_en=True)
        await env.reset()
        for _ in range(env.p.n + 1):
            await env.tick(shift_en=True)
        await env.v1_drain(env.legal_tile())
    await env.finish()

# --------------------------------------------------------------------------- #
# constrained random
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_random(dut):
    """Random captures (legal tiles), random shift_en, junk tile inputs between
    captures, FSM-driven jobs in between, rare resets. Reproduce with SEED=<n>."""
    env = await make_env(dut)
    rng, n = env.rng, env.p.n
    while env.cycle < NUM_RANDOM_CYCLES:
        r = rng.random()
        if r < 0.03:
            await env.run_fsm_job(rng.randint(1, 3 * n + 1), rng.randint(1, 3 * n + 1), rng.randint(1, 4))
        elif r < 0.004 + 0.03:
            await env.reset()
        else:
            cap = rng.random() < 0.06
            await env.tick(capture=cap, shift_en=rng.random() < 0.75,
                           tile=env.legal_tile() if cap else None)
    await env.idle(n + 1)
    await env.finish()

# --------------------------------------------------------------------------- #
# coverage closure (must stay the LAST test)
# --------------------------------------------------------------------------- #

@cocotb.test(skip=os.environ.get("MONITORS", "1") == "0")
async def test_coverage_closure(dut):
    """Every required tile shape and drain event was hit by the run."""
    p = Params()
    COVERAGE.report(dut._log)
    shapes, events = COVERAGE.holes(p)
    assert not (shapes or events), f"coverage holes:\n shapes {shapes}\n events {events}"
