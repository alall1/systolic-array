"""
cocotb testbench for "host_if" (bus decode, registers, status, start pulse, C read mux)
Contract: docs/top_contract.md §3-8

Closed loop: TileCtrlModel + FeederModel take the DUT's own start/M/N/K and
drive busy/done/error back, so every busy window, done cycle and rejected
command has the real schedule's timing. A C-RAM stand-in answers c_re/c_raddr
with registered words (lane_ram timing). Unused bus fields carry junk.

Lockstep every cycle against HostIfModel:
  before the edge: the same-cycle decode (A/B write strobes, C read strobe)
  after the edge : start, M, N, K, and host_rdata after every read

Independent of the model, concurrent monitors sample once per cycle (FallingEdge):
  RegisterScoreboard - own shadow of DIM registers and status bits; predicts
                       every read's data, the start pulse and M/N/K (§8.1-3, 5)
  DecodeMonitor      - for each cycle's bus inputs: exactly the right A/B write
                       and C read strobes, one-hot lane enables (§8.4, 6)
  CoverageMonitor    - registers, start rules, sticky causes, decode, unmapped space
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

from host_map import (HostMap, REGION_REGS, REGION_A, REGION_B, REGION_C, REG_CTRL,
                      REG_STATUS, REG_DIM_M, REG_DIM_N, REG_DIM_K, NUM_REGS, CTRL_START,
                      STATUS_BUSY, STATUS_DONE, STATUS_CMD_ERROR, STATUS_HOST_ERROR)
from host_if_model import HostIfModel
from feeder_model import FeederModel
from tile_ctrl_model import TileCtrlModel, job_latency

CLK_PERIOD_NS = 10
NUM_RANDOM_CYCLES = 5000

# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

class Params:
    def __init__(self):
        self.n          = int(os.environ["ARRAY_DIM"])
        self.data_width = int(os.environ["DATA_WIDTH"])
        self.acc_width  = int(os.environ["ACC_WIDTH"])
        self.max_k      = int(os.environ["MAX_K"])
        self.max_m      = int(os.environ["MAX_M"])
        self.max_n      = int(os.environ["MAX_N"])
        self.seed       = int(os.environ.get("SEED", "12345"))
        self.lockstep   = os.environ.get("LOCKSTEP", "1") != "0"
        self.monitors   = os.environ.get("MONITORS", "1") != "0"
        self.map = HostMap(self.n, self.max_m, self.max_n, self.max_k, self.data_width, self.acc_width)


def read(sig, name):
    try:
        return int(sig.value)
    except ValueError:
        raise AssertionError(f"{name} is X/Z: {sig.value}")


SCALARS_IN = ("rst_n", "host_we", "host_re", "host_addr", "host_wdata", "busy", "done", "error")
SCALARS_OUT = ("host_rdata", "start", "M", "N", "K", "a_we", "a_waddr", "a_wlane_en",
               "b_we", "b_waddr", "b_wlane_en", "c_re", "c_raddr")
DECODE = ("a_we", "a_waddr", "a_wlane_en", "b_we", "b_waddr", "b_wlane_en", "c_re", "c_raddr")


def snapshot(dut, n):
    s = {k: read(getattr(dut, k), k) for k in SCALARS_IN + SCALARS_OUT}
    s["a_wdata"] = [read(dut.a_wdata[l], f"a_wdata[{l}]") for l in range(n)]
    s["b_wdata"] = [read(dut.b_wdata[l], f"b_wdata[{l}]") for l in range(n)]
    s["c_rdata"] = [read(dut.c_rdata[l], f"c_rdata[{l}]") for l in range(n)]
    return s


def mapped(m, region, word, lane):
    """Independent of HostIfModel: §4 table."""
    if region == REGION_REGS:
        return lane == 0 and word < NUM_REGS
    width = {REGION_A: m.a_aw, REGION_B: m.b_aw, REGION_C: m.c_aw}[region]
    return lane < m.n and word < (1 << width)

# --------------------------------------------------------------------------- #
# coverage (module level: accumulates over every test in the run)
# --------------------------------------------------------------------------- #

class Coverage:
    REQUIRED = {"dim_write", "dim_read", "ctrl_read", "status_read", "status_write_ignored",
                "start_taken", "start_ignored_busy", "start_ignored_pending", "clear_beats_set",
                "done_seen_live", "cmd_error_set", "host_error_set", "a_write_every_lane",
                "b_write_every_lane", "c_read_every_lane", "unmapped_reg_lane", "unmapped_reg_index",
                "unmapped_lane_ge_n", "unmapped_high_word", "read_ab_region", "write_c_region",
                "we_and_re_together", "reset_mid_job"}

    def __init__(self):
        self.hits = Counter()
        self.lanes = {"a": set(), "b": set(), "c": set()}

    def holes(self, p):
        m = p.map
        req = set(self.REQUIRED)
        if m.n == (1 << m.lane_w):
            req.discard("unmapped_lane_ge_n")           # every lane code is a real lane
        if min(m.a_aw, m.b_aw, m.c_aw) == m.word_w:
            req.discard("unmapped_high_word")
        for k in ("a", "b", "c"):
            if len(self.lanes[k]) == m.n:
                self.hits[f"{k}_{'read' if k == 'c' else 'write'}_every_lane"] += 1
        return sorted(req - set(self.hits))

    def report(self, log):
        log.info("---- functional coverage (cumulative) ----")
        log.info(", ".join(f"{k}={self.hits[k]}" for k in sorted(self.REQUIRED)))
        log.info("lanes: " + ", ".join(f"{k}={sorted(v)}" for k, v in self.lanes.items()))

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
                cur = snapshot(self.dut, self.p.n)
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


class RegisterScoreboard(_Monitor):
    """A snapshot's bus inputs are what the coming edge samples; its outputs
    (start, M/N/K, host_rdata) are what the previous edge produced."""
    def __init__(self, dut, p):
        super().__init__(dut, p)
        self._clear()
        self.reads_checked = 0

    def _clear(self):
        self.dims = {REG_DIM_M: 0, REG_DIM_N: 0, REG_DIM_K: 0}
        self.done_s = self.cmd_s = self.host_s = False
        self.exp_start = False
        self.exp_read = None            # None | ("val", v) | ("c", lane)

    def on_reset(self, prev, cur):
        self._clear()
        if cur["start"] or cur["M"] or cur["N"] or cur["K"]:
            self.fail(f"rst_n low but start/M/N/K = {cur['start']}/{cur['M']}/{cur['N']}/{cur['K']}")

    def sample(self, prev, cur):
        m = self.p.map
        # outputs produced by the previous edge
        if bool(cur["start"]) != self.exp_start:
            self.fail(f"start = {cur['start']}, expected {int(self.exp_start)}")
        got_dims = (cur["M"], cur["N"], cur["K"])
        want_dims = (self.dims[REG_DIM_M], self.dims[REG_DIM_N], self.dims[REG_DIM_K])
        if got_dims != want_dims:
            self.fail(f"M/N/K = {got_dims}, registers hold {want_dims}")
        if self.exp_read is not None:
            kind, v = self.exp_read
            want = v if kind == "val" else cur["c_rdata"][v]
            if cur["host_rdata"] != want:
                self.fail(f"host_rdata = {cur['host_rdata']:#x}, expected {want:#x} ({self.exp_read})")
            self.reads_checked += 1

        # what the coming edge does
        region, word, lane = m.fields(cur["host_addr"])
        ok = mapped(m, region, word, lane)
        busy_now = bool(cur["busy"] or cur["start"])        # job running, or a start pending
        if cur["host_re"]:
            if ok and region == REGION_C:
                self.exp_read = ("c", lane)
            elif ok and region == REGION_REGS:
                if word == REG_STATUS:
                    v = ((STATUS_BUSY if busy_now else 0)
                         | (STATUS_DONE if self.done_s or cur["done"] else 0)
                         | (STATUS_CMD_ERROR if self.cmd_s or (cur["done"] and cur["error"]) else 0)
                         | (STATUS_HOST_ERROR if self.host_s else 0))
                else:
                    v = self.dims.get(word, 0)
                self.exp_read = ("val", v)
            else:
                self.exp_read = ("val", 0)
        else:
            self.exp_read = None
        reg_w = cur["host_we"] and ok and region == REGION_REGS
        taken = reg_w and word == REG_CTRL and bool(cur["host_wdata"] & CTRL_START) and not busy_now
        if taken:
            self.done_s = self.cmd_s = self.host_s = False
        else:
            self.done_s |= bool(cur["done"])
            self.cmd_s |= bool(cur["done"] and cur["error"])
            self.host_s |= bool(cur["host_we"] and ok and region in (REGION_A, REGION_B) and busy_now)
        self.exp_start = taken
        if reg_w and word in self.dims:
            w = {REG_DIM_M: m.m_w, REG_DIM_N: m.n_w, REG_DIM_K: m.k_w}[word]
            self.dims[word] = cur["host_wdata"] & ((1 << w) - 1)


class DecodeMonitor(_Monitor):
    def sample(self, prev, cur):
        m = self.p.map
        region, word, lane = m.fields(cur["host_addr"])
        ok = mapped(m, region, word, lane)
        busy_now = bool(cur["busy"] or cur["start"])
        dmask = (1 << m.data_width) - 1
        for r, pre in ((REGION_A, "a"), (REGION_B, "b")):
            want = bool(cur["host_we"] and ok and region == r and not busy_now)
            if bool(cur[f"{pre}_we"]) != want:
                self.fail(f"{pre}_we = {cur[pre + '_we']}, expected {int(want)} (addr {cur['host_addr']:#x}, busy {busy_now})")
            if want:
                if cur[f"{pre}_waddr"] != word or cur[f"{pre}_wlane_en"] != 1 << lane:
                    self.fail(f"{pre} write word/lanes {cur[pre + '_waddr']}/{cur[pre + '_wlane_en']:b}, "
                              f"expected {word}/{1 << lane:b}")
                if cur[f"{pre}_wdata"][lane] != cur["host_wdata"] & dmask:
                    self.fail(f"{pre}_wdata[{lane}] = {cur[pre + '_wdata'][lane]:#x}, expected {cur['host_wdata'] & dmask:#x}")
        want_c = bool(cur["host_re"] and ok and region == REGION_C)
        if bool(cur["c_re"]) != want_c:
            self.fail(f"c_re = {cur['c_re']}, expected {int(want_c)}")
        if want_c and cur["c_raddr"] != word:
            self.fail(f"c_raddr = {cur['c_raddr']}, expected {word}")


class CoverageMonitor(_Monitor):
    def on_reset(self, prev, cur):
        if prev is not None and (prev["busy"] or prev["start"]):
            COVERAGE.hits["reset_mid_job"] += 1

    def sample(self, prev, cur):
        m, h = self.p.map, COVERAGE.hits
        region, word, lane = m.fields(cur["host_addr"])
        ok = mapped(m, region, word, lane)
        busy_now = bool(cur["busy"] or cur["start"])
        we, re = cur["host_we"], cur["host_re"]
        if we and re:
            h["we_and_re_together"] += 1
        if not ok and (we or re):
            if region == REGION_REGS:
                h["unmapped_reg_lane" if lane else "unmapped_reg_index"] += 1
            elif lane >= m.n:
                h["unmapped_lane_ge_n"] += 1
            else:
                h["unmapped_high_word"] += 1
        if not ok:
            return
        if region == REGION_REGS:
            if we and word in (REG_DIM_M, REG_DIM_N, REG_DIM_K):
                h["dim_write"] += 1
            if re and word in (REG_DIM_M, REG_DIM_N, REG_DIM_K):
                h["dim_read"] += 1
            if re and word == REG_CTRL:
                h["ctrl_read"] += 1
            if re and word == REG_STATUS:
                h["status_read"] += 1
                if cur["done"]:
                    h["done_seen_live"] += 1
            if we and word == REG_STATUS:
                h["status_write_ignored"] += 1
            if we and word == REG_CTRL and cur["host_wdata"] & CTRL_START:
                if not busy_now:
                    h["start_taken"] += 1
                    if cur["done"]:
                        h["clear_beats_set"] += 1
                else:
                    h["start_ignored_pending" if cur["start"] else "start_ignored_busy"] += 1
        elif region in (REGION_A, REGION_B):
            if we:
                if busy_now:
                    h["host_error_set"] += 1
                else:
                    COVERAGE.lanes["a" if region == REGION_A else "b"].add(lane)
            if re:
                h["read_ab_region"] += 1
        else:
            if re:
                COVERAGE.lanes["c"].add(lane)
            if we:
                h["write_c_region"] += 1
        if cur["done"] and cur["error"]:
            h["cmd_error_set"] += 1

# --------------------------------------------------------------------------- #
# environment: driver + model + FSM / C-RAM stand-ins + scoreboard, lockstep
# --------------------------------------------------------------------------- #

class FsmSource:
    """tile_ctrl's model closed with the feeder model; driven by the DUT's start/M/N/K."""
    def __init__(self, p):
        self.p = p
        self.reset()

    def reset(self):
        p = self.p
        self.fsm = TileCtrlModel(p.n, p.max_m, p.max_n, p.max_k)
        self.fdr = FeederModel(p.n, lambda k, l: 0, lambda k, l: 0)
        self.o = self.fsm.reset()
        self.fo = self.fdr.reset()

    def advance(self, start, M, N, K):
        o, fo = self.o, self.fo
        self.fo = self.fdr.step(start=o["feed_start"], M=o["feed_M"], N=o["feed_N"], K=o["feed_K"])
        self.o = self.fsm.step(start=start, M=M, N=N, K=K, feed_done=fo["done"])


class CRam:
    """Read-only stand-in for the C lane_ram's read port: registered, held while c_re is low."""
    def __init__(self, p, rng):
        self.p, self.rng = p, rng
        self.words = {}
        self.rdata = [0] * p.n

    def word(self, addr):
        if addr not in self.words:
            self.words[addr] = [self.rng.getrandbits(self.p.acc_width) for _ in range(self.p.n)]
        return self.words[addr]

    def edge(self, re, raddr):
        if re:
            self.rdata = list(self.word(raddr))
        return self.rdata


class Env:
    def __init__(self, dut, p):
        self.dut = dut
        self.p = p
        self.m = p.map
        self.rng = random.Random(p.seed)
        self.model = HostIfModel(p.map)
        self.src = FsmSource(p)
        self.cram = CRam(p, random.Random(p.seed + 1))
        self.cycle = 0
        self.monitors = [RegisterScoreboard(dut, p), DecodeMonitor(dut, p), CoverageMonitor(dut, p)]

    async def tick(self, we=False, re=False, addr=None, wdata=None):
        """One edge: drive the bus (junk on unused fields) and busy/done/error from
        the FSM source; check the same-cycle decode; cross the edge; drive the C
        RAM's post-edge read data; step model + FSM; check registered outputs."""
        dut, m, rng = self.dut, self.m, self.rng
        if addr is None:
            addr = rng.randrange(1 << m.addr_w)
        if wdata is None:
            wdata = rng.getrandbits(m.host_w)
        o = self.src.o
        dut.host_we.value = int(we)
        dut.host_re.value = int(re)
        dut.host_addr.value = addr
        dut.host_wdata.value = wdata
        dut.busy.value = int(o["busy"])
        dut.done.value = int(o["done"])
        dut.error.value = int(o["error"])
        await Timer(1, unit="ns")

        d = self.model.decode(we, re, addr, wdata, busy=o["busy"])
        if self.p.lockstep:
            self.check_decode(d)
        pre = {k: read(getattr(dut, k), k) for k in ("start", "M", "N", "K", "c_re", "c_raddr")}

        await RisingEdge(dut.clk)
        c_rdata = self.cram.edge(pre["c_re"], pre["c_raddr"])
        for l, v in enumerate(c_rdata):
            dut.c_rdata[l].value = v
        await Timer(1, unit="ns")

        exp = self.model.step(we, re, addr, wdata, busy=o["busy"], done=o["done"],
                              error=o["error"], c_rdata=c_rdata)
        self.src.advance(bool(pre["start"]), pre["M"], pre["N"], pre["K"])
        self.cycle += 1
        if self.p.lockstep:
            self.check_registered(exp)
        return read(dut.host_rdata, "host_rdata")

    def check_decode(self, d):
        dut, tag = self.dut, f"cycle {self.cycle} (pre-edge)"
        for k in DECODE:
            got = read(getattr(dut, k), k)
            assert got == int(d[k]), f"{tag}: {k} dut={got} exp={int(d[k])}"
        for pre in ("a", "b"):
            if d[f"{pre}_we"]:
                lane = d[f"{pre}_wlane_en"].bit_length() - 1
                got = read(getattr(dut, f"{pre}_wdata")[lane], f"{pre}_wdata")
                assert got == d[f"{pre}_wdata"][lane], f"{tag}: {pre}_wdata[{lane}] dut={got:#x} exp={d[pre + '_wdata'][lane]:#x}"

    def check_registered(self, exp):
        dut, tag = self.dut, f"cycle {self.cycle}"
        for k in ("start", "M", "N", "K"):
            got = read(getattr(dut, k), k)
            assert got == int(exp[k]), f"{tag}: {k} dut={got} exp={int(exp[k])}"
        if exp["host_rdata"] is not None:
            got = read(dut.host_rdata, "host_rdata")
            assert got == exp["host_rdata"], f"{tag}: host_rdata dut={got:#x} exp={exp['host_rdata']:#x}"

    # host-side helpers ------------------------------------------------------
    async def write(self, addr, value):
        await self.tick(we=True, addr=addr, wdata=value & ((1 << self.m.host_w) - 1))

    async def read(self, addr):
        return await self.tick(re=True, addr=addr)

    async def idle(self, cycles):
        for _ in range(cycles):
            await self.tick()

    async def start_job(self, M, N, K):
        m = self.m
        await self.write(m.reg(REG_DIM_M), M)
        await self.write(m.reg(REG_DIM_N), N)
        await self.write(m.reg(REG_DIM_K), K)
        await self.write(m.reg(REG_CTRL), CTRL_START)

    async def poll(self, limit):
        for _ in range(limit):
            s = await self.read(self.m.reg(REG_STATUS))
            assert s & (STATUS_BUSY | STATUS_DONE), f"STATUS {s:#x} looks idle while a job is outstanding"
            if s & STATUS_DONE:
                return s
        raise AssertionError("no DONE")

    async def reset(self, cycles=2):
        """Async reset mid-cycle: registered outputs idle before any edge."""
        dut = self.dut
        await Timer(2, unit="ns")
        dut.rst_n.value = 0
        dut.host_we.value = 0
        dut.host_re.value = 0
        await Timer(1, unit="ns")
        for k in ("start", "M", "N", "K"):
            assert read(getattr(dut, k), k) == 0, f"{k} not 0 right after async reset"
        self.model.reset()
        self.src.reset()
        dut.busy.value = dut.done.value = dut.error.value = 0
        for _ in range(cycles):
            await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        dut.rst_n.value = 1

    async def finish(self):
        await FallingEdge(self.dut.clk)
        COVERAGE.report(self.dut._log)
        self.dut._log.info(f"RegisterScoreboard checked {self.monitors[0].reads_checked} read(s)")
        errors = [e for m in self.monitors for e in m.errors]
        assert not errors, f"{len(errors)} monitor violation(s):\n" + "\n".join(errors[:20])


async def make_env(dut):
    p = Params()
    m = p.map
    dut._log.info(f"SEED={p.seed} LOCKSTEP={int(p.lockstep)} MONITORS={int(p.monitors)} ARRAY_DIM={p.n} "
                  f"MAX_M={p.max_m} MAX_N={p.max_n} MAX_K={p.max_k} HOST_AW={m.addr_w}")
    for name, w in (("host_addr", m.addr_w), ("host_wdata", m.host_w), ("host_rdata", m.host_w),
                    ("M", m.m_w), ("N", m.n_w), ("K", m.k_w), ("a_waddr", m.a_aw), ("b_waddr", m.b_aw),
                    ("c_raddr", m.c_aw), ("a_wlane_en", m.n)):
        assert len(getattr(dut, name)) == w, f"{name} is {len(getattr(dut, name))} bits, expected {w}"
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    dut.rst_n.value = 0
    for k in ("host_we", "host_re", "host_addr", "host_wdata", "busy", "done", "error"):
        getattr(dut, k).value = 0
    for l in range(p.n):
        dut.c_rdata[l].value = 0
    for _ in range(2):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.rst_n.value = 1
    env = Env(dut, p)
    if p.monitors:
        for mon in env.monitors:
            cocotb.start_soon(mon.run())
    return env

# --------------------------------------------------------------------------- #
# directed tests
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_reset_and_registers(dut):
    """Reset values; DIM registers masked to their widths, read back 1 cycle
    later; CTRL reads 0; STATUS writes ignored; read-first on the same edge."""
    env = await make_env(dut)
    m = env.m
    assert await env.read(m.reg(REG_STATUS)) == 0
    for reg, w in ((REG_DIM_M, m.m_w), (REG_DIM_N, m.n_w), (REG_DIM_K, m.k_w)):
        await env.write(m.reg(reg), 0xFFFF_FFFF)
        assert await env.read(m.reg(reg)) == (1 << w) - 1
        assert await env.tick(we=True, re=True, addr=m.reg(reg), wdata=3) == (1 << w) - 1   # old value
        assert await env.read(m.reg(reg)) == 3
    assert await env.read(m.reg(REG_CTRL)) == 0
    await env.write(m.reg(REG_STATUS), 0xF)
    assert await env.read(m.reg(REG_STATUS)) == 0
    await env.finish()


@cocotb.test()
async def test_jobs_through_the_bus(dut):
    """Start real-schedule jobs and poll: BUSY or DONE on every poll, DONE seen
    exactly L + 2 cycles after the START write."""
    env = await make_env(dut)
    n = env.p.n
    for (M, N, K) in [(n, n, 2), (1, 1, 1), (2 * n + 1, n - 1, 3), (n + 2, 2 * n, 1)]:
        await env.start_job(M, N, K)
        t0 = env.cycle
        s = await env.poll(job_latency(M, N, K, n) + 10)
        assert s == STATUS_DONE and env.cycle - t0 == job_latency(M, N, K, n) + 2
        await env.idle(env.rng.randint(0, 3))
    await env.finish()


@cocotb.test()
async def test_illegal_command(dut):
    """Illegal dims: DONE | CMD_ERROR with no idle-looking gap; next START clears."""
    env = await make_env(dut)
    for cfg in [(0, 1, 1), (1, 0, 1), (1, 1, 0), (env.p.max_m + 1, 1, 1)]:
        await env.start_job(*cfg)
        assert await env.poll(10) == STATUS_DONE | STATUS_CMD_ERROR
    await env.start_job(2, 2, 2)
    assert await env.read(env.m.reg(REG_STATUS)) == STATUS_BUSY
    await env.poll(40)
    await env.finish()


@cocotb.test()
async def test_start_rules(dut):
    """START ignored while pending or busy; taken in tile_ctrl's done cycle,
    where its clear beats the same-edge done."""
    env = await make_env(dut)
    m, n = env.m, env.p.n
    M, N, K = n + 1, n, 2
    await env.start_job(M, N, K)
    await env.write(m.reg(REG_CTRL), CTRL_START)                    # pending: ignored
    for _ in range(3):
        await env.write(m.reg(REG_CTRL), CTRL_START)                # busy: ignored
    while not env.src.o["done"]:                                    # run to tile_ctrl's done cycle
        await env.tick()
    await env.write(m.reg(REG_CTRL), CTRL_START)                    # taken in the done cycle
    assert await env.read(m.reg(REG_STATUS)) == STATUS_BUSY, "clear beats the same-edge done"
    await env.poll(job_latency(M, N, K, n) + 10)
    await env.finish()


@cocotb.test()
async def test_ab_writes(dut):
    """Every lane of A and B; writes while a job runs are dropped and set HOST_ERROR."""
    env = await make_env(dut)
    m, rng = env.m, env.rng
    for region in (REGION_A, REGION_B):
        width = m.a_aw if region == REGION_A else m.b_aw
        for lane in range(m.n):
            await env.write(m.addr(region, rng.randrange(1 << width), lane), rng.getrandbits(m.host_w))
    await env.start_job(env.p.n, env.p.n, 3)
    for region in (REGION_A, REGION_B):
        await env.write(m.addr(region, 1, 0), 7)                     # dropped
    s = await env.poll(60)
    assert s == STATUS_DONE | STATUS_HOST_ERROR
    await env.finish()


@cocotb.test()
async def test_c_reads(dut):
    """Every lane of C words: data is the C RAM's word, lane-selected, 1 cycle later."""
    env = await make_env(dut)
    m, rng = env.m, env.rng
    for _ in range(20):
        word = rng.randrange(1 << m.c_aw)
        for lane in range(m.n):
            got = await env.read(m.addr(REGION_C, word, lane))
            assert got == env.cram.word(word)[lane]
    await env.finish()


@cocotb.test()
async def test_unmapped_and_odd_accesses(dut):
    """Unmapped reads return 0 and writes do nothing; A/B reads return 0; C writes ignored."""
    env = await make_env(dut)
    m = env.m
    odd = [m.addr(REGION_REGS, 0, 1), m.addr(REGION_REGS, NUM_REGS, 0),
           m.addr(REGION_REGS, (1 << m.word_w) - 1, 0), m.addr(REGION_A, 0, 0), m.addr(REGION_B, 0, 0)]
    if m.n < (1 << m.lane_w):
        odd += [m.addr(r, 0, m.n) for r in (REGION_A, REGION_B, REGION_C)]
    for r, w in ((REGION_A, m.a_aw), (REGION_B, m.b_aw), (REGION_C, m.c_aw)):
        if w < m.word_w:
            odd.append(m.addr(r, 1 << w, 0))
    for a in odd:
        await env.write(a, 0xFFFF_FFFF)
        assert await env.read(a) == 0, hex(a)
    await env.write(m.addr(REGION_C, 0, 0), 0x1234)                   # C is not host-writable
    assert await env.read(m.reg(REG_STATUS)) == 0, "odd accesses set no error"
    await env.finish()


@cocotb.test()
async def test_reset_mid_job(dut):
    """Async reset during a job: registers and status back to 0, next job clean."""
    env = await make_env(dut)
    n = env.p.n
    for cut in (0, 2, 7):
        await env.write(env.m.reg(REG_DIM_M), 3)
        await env.start_job(n + 1, n, 3)
        await env.idle(cut)
        await env.reset()
        assert await env.read(env.m.reg(REG_STATUS)) == 0
        assert await env.read(env.m.reg(REG_DIM_M)) == 0
    await env.start_job(2, 2, 2)
    await env.poll(40)
    await env.finish()

# --------------------------------------------------------------------------- #
# constrained random
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_random(dut):
    """Random bus traffic over the whole map (mapped and unmapped), random START
    with legal and illegal dims, while real-schedule jobs run. Reproduce with SEED=<n>."""
    env = await make_env(dut)
    m, rng, n = env.m, env.rng, env.p.n

    def rand_addr():
        r = rng.random()
        if r < 0.25:
            return m.reg(rng.choice([REG_STATUS, REG_STATUS, REG_DIM_M, REG_DIM_N, REG_DIM_K, REG_CTRL]))
        if r < 0.85:
            region = rng.choice([REGION_A, REGION_B, REGION_C])
            width = {REGION_A: m.a_aw, REGION_B: m.b_aw, REGION_C: m.c_aw}[region]
            return m.addr(region, rng.randrange(1 << width), rng.randrange(m.n))
        return rng.randrange(1 << m.addr_w)             # anything, often unmapped

    while env.cycle < NUM_RANDOM_CYCLES:
        r = rng.random()
        if r < 0.03:
            await env.write(m.reg(REG_DIM_M), rng.randint(0, 2 * n + 1))
            await env.write(m.reg(REG_DIM_N), rng.randint(1, 2 * n + 1))
            await env.write(m.reg(REG_DIM_K), rng.randint(1, 4))
            await env.write(m.reg(REG_CTRL), CTRL_START)
        elif r < 0.45:
            await env.tick()
        else:
            a = rand_addr()
            we, re = rng.random() < 0.5, rng.random() < 0.6
            await env.tick(we=we, re=re, addr=a)
    await env.idle(3)
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
