"""
cocotb testbench for "lane_ram" module (the A, B and C tile-word memories)
Contract: docs/memory_contract.md §4, §7.1-4

Lockstep: every cycle the SAME inputs drive the DUT and LaneRamModel, then rdata
is compared on every lane the model knows (lanes never written, and rdata
before the first read, are don't-care by contract). Inputs a port is not using
carry junk (waddr/wdata/wlane_en while we is low, raddr while re is low).

Independent of the model, concurrent monitors sample once per cycle (FallingEdge):
  ReadScoreboard - own shadow memory from the observed write traffic; predicts
                   every rdata (read-first collisions, holds) and checks known lanes
  HoldMonitor    - rdata (every lane, known or not) never changes across an edge
                   where re was low
  CoverageMonitor- lane-mask classes, collisions, holds, address/data extremes, streams
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

from lane_ram_model import LaneRamModel
from tile_layout import operand_addr, pack_a_tiles, ceil_div

CLK_PERIOD_NS = 10
NUM_RANDOM_CYCLES = 6000

# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

class Params:
    def __init__(self):
        self.lanes    = int(os.environ["LANES"])
        self.width    = int(os.environ["WIDTH"])
        self.addr_w   = int(os.environ["ADDR_W"])
        self.seed     = int(os.environ.get("SEED", "12345"))
        self.lockstep = os.environ.get("LOCKSTEP", "1") != "0"
        self.monitors = os.environ.get("MONITORS", "1") != "0"
        self.depth    = 1 << self.addr_w
        self.dmask    = (1 << self.width) - 1
        self.all_lanes = (1 << self.lanes) - 1


def read(sig, name):
    try:
        return int(sig.value)
    except ValueError:
        raise AssertionError(f"{name} is X/Z: {sig.value}")


def snapshot(dut, p):
    s = {k: read(getattr(dut, k), k) for k in ("we", "waddr", "wlane_en", "re", "raddr")}
    s["wdata"] = [read(dut.wdata[l], f"wdata[{l}]") for l in range(p.lanes)]
    s["rdata"] = [read(dut.rdata[l], f"rdata[{l}]") for l in range(p.lanes)]
    return s

# --------------------------------------------------------------------------- #
# coverage (module level: accumulates over every test in the run)
# --------------------------------------------------------------------------- #

class Coverage:
    REQUIRED = {"mask_all", "mask_none", "mask_single", "mask_partial",
                "collision_same_addr", "read_and_write_diff_addr", "read_never_written",
                "rewrite_word", "hold_3_cycles", "back_to_back_reads", "stream_8_reads",
                "write_addr_0", "write_addr_max", "read_addr_0", "read_addr_max",
                "data_all_ones", "data_zero"}

    def __init__(self):
        self.hits = Counter()

    def holes(self, p):
        req = set(self.REQUIRED)
        if p.lanes < 2:
            req.discard("mask_single")          # the only one-lane mask is "all lanes"
        if p.lanes < 3:
            req.discard("mask_partial")         # nothing between one lane and all lanes
        return sorted(req - set(self.hits))

    def report(self, log):
        log.info("---- functional coverage (cumulative) ----")
        log.info(", ".join(f"{k}={self.hits[k]}" for k in sorted(self.REQUIRED)))

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
            self.sample(self.prev, cur)
            self.prev = cur


class ReadScoreboard(_Monitor):
    """A snapshot's inputs are what the coming edge samples; its rdata is what
    the previous edge produced."""
    def __init__(self, dut, p):
        super().__init__(dut, p)
        self.shadow = {}
        self.exp = [None] * p.lanes         # rdata the last edge should have produced
        self.reads_checked = 0

    def sample(self, prev, cur):
        for l, (e, g) in enumerate(zip(self.exp, cur["rdata"])):
            if e is not None and e != g:
                self.fail(f"rdata[{l}] = {g:#x}, expected {e:#x} (lanes: got {cur['rdata']}, want {self.exp})")
        if cur["re"]:
            self.exp = list(self.shadow.get(cur["raddr"], [None] * self.p.lanes))   # read-first
            self.reads_checked += 1
        if cur["we"]:
            word = list(self.shadow.get(cur["waddr"], [None] * self.p.lanes))
            for l in range(self.p.lanes):
                if cur["wlane_en"] >> l & 1:
                    word[l] = cur["wdata"][l]
            self.shadow[cur["waddr"]] = word


class HoldMonitor(_Monitor):
    def sample(self, prev, cur):
        if prev is not None and not prev["re"] and cur["rdata"] != prev["rdata"]:
            self.fail(f"rdata changed across an edge with re low: {prev['rdata']} -> {cur['rdata']}")


class CoverageMonitor(_Monitor):
    def __init__(self, dut, p):
        super().__init__(dut, p)
        self.written = set()
        self.hold = 0
        self.run_len = 0

    def sample(self, prev, cur):
        p, h = self.p, COVERAGE.hits
        if cur["we"]:
            m = cur["wlane_en"]
            h["mask_all" if m == p.all_lanes else "mask_none" if m == 0
              else "mask_single" if bin(m).count("1") == 1 else "mask_partial"] += 1
            if m and cur["waddr"] in self.written:
                h["rewrite_word"] += 1
            if m:
                self.written.add(cur["waddr"])
            if cur["waddr"] == 0:
                h["write_addr_0"] += 1
            if cur["waddr"] == p.depth - 1:
                h["write_addr_max"] += 1
            if m and any(cur["wdata"][l] == p.dmask for l in range(p.lanes) if m >> l & 1):
                h["data_all_ones"] += 1
            if m and any(cur["wdata"][l] == 0 for l in range(p.lanes) if m >> l & 1):
                h["data_zero"] += 1
        if cur["re"]:
            if cur["we"]:
                h["collision_same_addr" if cur["raddr"] == cur["waddr"] else "read_and_write_diff_addr"] += 1
            if cur["raddr"] not in self.written:
                h["read_never_written"] += 1
            if cur["raddr"] == 0:
                h["read_addr_0"] += 1
            if cur["raddr"] == p.depth - 1:
                h["read_addr_max"] += 1
            if prev is not None and prev["re"]:
                h["back_to_back_reads"] += 1
            self.run_len = self.run_len + 1 if (prev is not None and prev["re"]
                                                and cur["raddr"] == (prev["raddr"] + 1) % p.depth) else 1
            if self.run_len == 8:
                h["stream_8_reads"] += 1
            self.hold = 0
        else:
            self.hold += 1
            if self.hold == 3:
                h["hold_3_cycles"] += 1

# --------------------------------------------------------------------------- #
# environment: driver + model + scoreboard, lockstep
# --------------------------------------------------------------------------- #

class Env:
    def __init__(self, dut, p):
        self.dut = dut
        self.p = p
        self.rng = random.Random(p.seed)
        self.model = LaneRamModel(p.lanes, p.width, p.addr_w)
        self.cycle = 0
        self.monitors = [ReadScoreboard(dut, p), HoldMonitor(dut, p), CoverageMonitor(dut, p)]

    def rand_word(self):
        rng, p = self.rng, self.p
        pick = lambda: rng.choice([0, p.dmask]) if rng.random() < 0.1 else rng.getrandbits(p.width)
        return [pick() for _ in range(p.lanes)]

    async def tick(self, we=False, waddr=None, wlane_en=None, wdata=None,
                   re=False, raddr=None):
        """Drive one edge's inputs (junk on whatever is unused), cross the edge,
        step the model with the same values, compare rdata on known lanes."""
        dut, p, rng = self.dut, self.p, self.rng
        if waddr is None:
            waddr = rng.randrange(p.depth)
        if wlane_en is None:
            wlane_en = rng.randrange(1 << p.lanes) if not we else p.all_lanes
        if wdata is None:
            wdata = self.rand_word()
        if raddr is None:
            raddr = rng.randrange(p.depth)
        dut.we.value = int(we)
        dut.waddr.value = waddr
        dut.wlane_en.value = wlane_en
        for l in range(p.lanes):
            dut.wdata[l].value = wdata[l]
        dut.re.value = int(re)
        dut.raddr.value = raddr

        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")

        exp = self.model.step(we, waddr, wlane_en, wdata, re, raddr)
        self.cycle += 1
        self.check(exp)
        return exp

    def check(self, exp):
        if not self.p.lockstep:
            return
        got = [read(self.dut.rdata[l], f"rdata[{l}]") for l in range(self.p.lanes)]
        for l, (e, g) in enumerate(zip(exp, got)):
            assert e is None or e == g, f"cycle {self.cycle}: rdata[{l}] dut={g:#x} exp={e:#x}"

    async def write(self, addr, data, lanes=None):
        return await self.tick(we=True, waddr=addr, wdata=data,
                               wlane_en=self.p.all_lanes if lanes is None else lanes)

    async def read(self, addr):
        return await self.tick(re=True, raddr=addr)

    async def idle(self, cycles):
        for _ in range(cycles):
            await self.tick()

    async def finish(self):
        await FallingEdge(self.dut.clk)
        COVERAGE.report(self.dut._log)
        self.dut._log.info(f"ReadScoreboard checked {self.monitors[0].reads_checked} read(s)")
        errors = [e for m in self.monitors for e in m.errors]
        assert not errors, f"{len(errors)} monitor violation(s):\n" + "\n".join(errors[:20])


async def make_env(dut):
    p = Params()
    dut._log.info(f"SEED={p.seed} LOCKSTEP={int(p.lockstep)} MONITORS={int(p.monitors)} "
                  f"LANES={p.lanes} WIDTH={p.width} ADDR_W={p.addr_w}")
    for name, w in (("waddr", p.addr_w), ("raddr", p.addr_w), ("wlane_en", p.lanes)):
        assert len(getattr(dut, name)) == w, f"{name} is {len(getattr(dut, name))} bits, expected {w}"
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    dut.we.value = 0
    dut.re.value = 0
    dut.waddr.value = 0
    dut.raddr.value = 0
    dut.wlane_en.value = 0
    for l in range(p.lanes):
        dut.wdata[l].value = 0
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    env = Env(dut, p)
    if p.monitors:
        for m in env.monitors:
            cocotb.start_soon(m.run())
    return env

# --------------------------------------------------------------------------- #
# directed tests
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_write_then_read(dut):
    """Write a word, read it back one cycle later; unused inputs carry junk."""
    env = await make_env(dut)
    for _ in range(20):
        a, w = env.rng.randrange(env.p.depth), env.rand_word()
        await env.write(a, w)
        got = await env.read(a)
        assert got == [v & env.p.dmask for v in w]
    await env.finish()


@cocotb.test()
async def test_lane_masks(dut):
    """Every write mask: masked lanes take the new value, the others keep theirs."""
    env = await make_env(dut)
    p = env.p
    for mask in range(1 << p.lanes):
        a = mask % p.depth
        base, new = env.rand_word(), env.rand_word()
        await env.write(a, base)
        await env.write(a, new, lanes=mask)
        got = await env.read(a)
        assert got == [(new[l] if mask >> l & 1 else base[l]) for l in range(p.lanes)], f"mask {mask:0{p.lanes}b}"
    await env.finish()


@cocotb.test()
async def test_read_first_collision(dut):
    """Read and write the same word on the same edge: the read sees the old word."""
    env = await make_env(dut)
    for _ in range(10):
        a, old, new = env.rng.randrange(env.p.depth), env.rand_word(), env.rand_word()
        await env.write(a, old)
        got = await env.tick(we=True, waddr=a, wlane_en=env.p.all_lanes, wdata=new, re=True, raddr=a)
        assert got == old
        assert await env.read(a) == new
    await env.finish()


@cocotb.test()
async def test_read_hold(dut):
    """rdata holds while re is low, even when the word it came from is rewritten."""
    env = await make_env(dut)
    for _ in range(10):
        a, w = env.rng.randrange(env.p.depth), env.rand_word()
        await env.write(a, w)
        await env.read(a)
        for _ in range(env.rng.randint(3, 6)):
            got = await env.tick(we=True, waddr=a, wlane_en=env.p.all_lanes, wdata=env.rand_word())
            assert got == w
    await env.finish()


@cocotb.test()
async def test_address_and_data_extremes(dut):
    """First and last word; all-zero and all-ones lanes."""
    env = await make_env(dut)
    p = env.p
    for a in (0, p.depth - 1):
        for w in ([0] * p.lanes, [p.dmask] * p.lanes, [p.dmask if l % 2 else 0 for l in range(p.lanes)]):
            await env.write(a, w)
            assert await env.read(a) == w
    await env.finish()


@cocotb.test()
async def test_read_never_written(dut):
    """Reading a word that was never written is legal (don't-care data, no error)."""
    env = await make_env(dut)
    for a in range(env.p.depth):
        await env.read(a)
    await env.finish()


@cocotb.test()
async def test_operand_stream(dut):
    """Use as the A memory: load tile words {tile, k} for a random A, then read
    them back-to-back one per cycle, as the feeder does, and check each lane."""
    env = await make_env(dut)
    p, rng = env.p, env.rng
    ki_w = max(1, p.addr_w - 2)                  # up to 4 tile slots in this RAM
    n = p.lanes
    for _ in range(4):
        K = rng.randint(min(8, 1 << ki_w), 1 << ki_w)
        M = rng.randint(1, min(4, 1 << (p.addr_w - ki_w)) * n)
        A = [[rng.getrandbits(p.width) for _ in range(K)] for _ in range(M)]
        words = pack_a_tiles(A, n, ki_w, pad=lambda l: rng.getrandbits(p.width))
        for addr, w in words.items():
            await env.write(addr, w)
        for tm in range(ceil_div(M, n)):
            # one read per cycle; the word for k is on rdata right after the edge that sampled read k
            for k in range(K):
                got = await env.read(operand_addr(tm, k, ki_w))
                for l in range(n):
                    if tm * n + l < M:
                        assert got[l] == A[tm * n + l][k], f"tile {tm} k {k} lane {l}"
    await env.finish()

# --------------------------------------------------------------------------- #
# constrained random
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_random(dut):
    """Random reads, writes, masks and collisions over a small address space.
    Reproduce with SEED=<n>."""
    env = await make_env(dut)
    rng, p = env.rng, env.p
    while env.cycle < NUM_RANDOM_CYCLES:
        we, re = rng.random() < 0.5, rng.random() < 0.5
        wa = rng.randrange(p.depth)
        ra = wa if (we and re and rng.random() < 0.2) else rng.randrange(p.depth)
        await env.tick(we=we, waddr=wa, wlane_en=rng.randrange(1 << p.lanes), re=re, raddr=ra)
    await env.finish()

# --------------------------------------------------------------------------- #
# coverage closure (must stay the LAST test)
# --------------------------------------------------------------------------- #

@cocotb.test(skip=os.environ.get("MONITORS", "1") == "0")
async def test_coverage_closure(dut):
    """Every required coverage item was hit by the run."""
    COVERAGE.report(dut._log)
    holes = COVERAGE.holes(Params())
    assert not holes, f"coverage holes: {holes}"
