"""
cocotb system test for "matmul_top", driven ONLY through the host bus
(docs/top_contract.md). The testbench is a host program: it loads A and B
element by element, writes DIM_M/N/K and CTRL.START, polls STATUS and reads C
element by element. Golden model: numpy A @ B. Jobs run back-to-back with no
reset between them, so stale accumulators and stale memory words are live.

Observers peek into the hierarchy (no extra ports) for the system properties
carried over from the former tb/tile_ctrl_integration/ harness:
  - fsm_contract §9.11: feed_done only in FEED; feed_start never while the feeder is busy
  - collector_contract §7.6-7: every C element written exactly once, nothing
    outside M x N or outside a job, last write in tile_ctrl's done cycle
  - memory_contract §7.6: the feeder only reads A / B words the host loaded
and the ones the bus adds (top_contract §6, §8.7-9):
  - every STATUS poll after START shows BUSY or DONE; DONE seen L + 2 after START
  - the host's first C read of a job is sampled strictly after the job's last C write
  - illegal dims -> DONE | CMD_ERROR, nothing written; START / A / B writes during
    a job are ignored, set HOST_ERROR and leave the result unchanged
Every block's SVA is live in this build (SVA=1), including the collector's
out-of-contract-input assertions.

Optional env vars: SEED (default 12345)
"""

import os
import random

import cocotb
import numpy as np
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer

from utils import to_signed, golden_matmul
from host_map import (HostMap, REGION_A, REGION_B, REG_CTRL, REG_STATUS, REG_DIM_M,
                      REG_DIM_N, REG_DIM_K, CTRL_START, STATUS_BUSY, STATUS_DONE,
                      STATUS_CMD_ERROR, STATUS_HOST_ERROR)
from tile_ctrl_model import job_latency

CLK_PERIOD_NS = 10
NUM_RANDOM_JOBS = 20
STATE_FEED = 2          # tile_ctrl state_t encoding: IDLE, ISSUE, FEED, ...


class Params:
    def __init__(self):
        self.n = int(os.environ["ARRAY_DIM"])
        self.data_width = int(os.environ["DATA_WIDTH"])
        self.acc_width = int(os.environ["ACC_WIDTH"])
        self.max_m = int(os.environ["MAX_M"])
        self.max_n = int(os.environ["MAX_N"])
        self.max_k = int(os.environ["MAX_K"])
        self.seed = int(os.environ.get("SEED", "12345"))
        self.map = HostMap(self.n, self.max_m, self.max_n, self.max_k, self.data_width, self.acc_width)


class System:
    def __init__(self, dut, p):
        self.dut = dut
        self.p = p
        self.m = p.map
        self.rng = random.Random(p.seed)
        self.cycle = 0
        self.errors = []
        self.loaded_a = set()           # A / B words with at least one lane written by the host
        self.loaded_b = set()
        self.writes = None              # {(row, col)} written by the collector in the job in flight
        self.last_c_write = None        # cycle of the job's last C RAM write
        self.first_c_read = None        # cycle of the host's first C RAM read after the job
        self.done_cycle = None

    # ------------------------------------------------------------------ #
    # observers (sample at the falling edge: inputs for the coming edge)
    # ------------------------------------------------------------------ #
    async def observer(self):
        dut, m = self.dut, self.m
        col, ctrl = dut.u_collector, dut.u_ctrl
        while True:
            await FallingEdge(dut.clk)
            if not int(dut.rst_n.value):
                continue
            # §9.11 and feeder handshake
            if int(dut.feed_done.value) and int(ctrl.state.value) != STATE_FEED:
                self.errors.append("feed_done outside FEED")
            if int(dut.feed_start.value) and int(dut.feeder_busy.value):
                self.errors.append("feed_start while feeder busy")
            if int(ctrl.done.value):
                self.done_cycle = self.cycle
            # host A / B writes that land (taken by the RAM on this edge)
            if int(dut.u_mem_a.we.value):
                self.loaded_a.add(int(dut.u_mem_a.waddr.value))
            if int(dut.u_mem_b.we.value):
                self.loaded_b.add(int(dut.u_mem_b.waddr.value))
            # feeder reads only loaded words
            if int(dut.u_mem_a.re.value):
                a, b = int(dut.u_mem_a.raddr.value), int(dut.u_mem_b.raddr.value)
                if a not in self.loaded_a:
                    self.errors.append(f"A read of unloaded word {a:#x}")
                if b not in self.loaded_b:
                    self.errors.append(f"B read of unloaded word {b:#x}")
            # collector writes into C
            if int(col.wr_en.value):
                if self.writes is None:
                    self.errors.append("collector wrote outside a job")
                else:
                    row, base, lanes = int(col.wr_row.value), int(col.wr_tile_n.value) * m.n, int(col.wr_lane_en.value)
                    for j in range(m.n):
                        if lanes >> j & 1:
                            if (row, base + j) in self.writes:
                                self.errors.append(f"C{(row, base + j)} written twice")
                            self.writes.add((row, base + j))
                    self.last_c_write = self.cycle
            if int(dut.u_mem_c.re.value) and self.first_c_read is None:
                self.first_c_read = self.cycle

    # ------------------------------------------------------------------ #
    # bus-functional host
    # ------------------------------------------------------------------ #
    async def edge(self, we=False, re=False, addr=0, wdata=0):
        dut = self.dut
        dut.host_we.value = int(we)
        dut.host_re.value = int(re)
        dut.host_addr.value = addr
        dut.host_wdata.value = wdata & ((1 << self.m.host_w) - 1)
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        self.cycle += 1
        dut.host_we.value = 0
        dut.host_re.value = 0
        return int(dut.host_rdata.value)

    async def write(self, addr, value):
        await self.edge(we=True, addr=addr, wdata=value)

    async def read(self, addr):
        return await self.edge(re=True, addr=addr)

    async def load(self, A, B):
        m = self.m
        for r in range(A.shape[0]):
            for k in range(A.shape[1]):
                await self.write(m.a_elem(r, k), int(A[r][k]))
        for k in range(B.shape[0]):
            for c in range(B.shape[1]):
                await self.write(m.b_elem(k, c), int(B[k][c]))

    async def start(self, M, N, K):
        m = self.m
        await self.write(m.reg(REG_DIM_M), M)
        await self.write(m.reg(REG_DIM_N), N)
        await self.write(m.reg(REG_DIM_K), K)
        self.writes, self.last_c_write, self.first_c_read, self.done_cycle = set(), None, None, None
        await self.write(m.reg(REG_CTRL), CTRL_START)
        return self.cycle

    async def poll(self, limit, during=None):
        """Poll STATUS until DONE; every read must show BUSY or DONE.
        `during(i)` may issue extra bus operations between polls."""
        for i in range(limit):
            s = await self.read(self.m.reg(REG_STATUS))
            assert s & (STATUS_BUSY | STATUS_DONE), f"STATUS {s:#x} looks idle while a job is outstanding"
            if s & STATUS_DONE:
                return s
            if during is not None:
                await during(i)
        raise AssertionError("no DONE")

    async def read_c(self, M, N):
        C = np.zeros((M, N), dtype=object)
        for r in range(M):
            for c in range(N):
                C[r][c] = to_signed(await self.read(self.m.c_elem(r, c)), self.p.acc_width)
        return C

    # ------------------------------------------------------------------ #
    async def run_job(self, A, B, during=None, expect=STATUS_DONE):
        n = self.p.n
        A, B = np.asarray(A), np.asarray(B)
        M, K = A.shape
        N = B.shape[1]
        await self.load(A, B)
        t0 = await self.start(M, N, K)
        L = job_latency(M, N, K, n)
        status = await self.poll(L + 10, during)
        seen = self.cycle
        writes, self.writes = self.writes, None
        C = await self.read_c(M, N)
        assert not self.errors, "\n".join(self.errors[:10])
        assert status == expect, f"STATUS {status:#x}, expected {expect:#x}"
        outside = sorted(k for k in writes if not (k[0] < M and k[1] < N))
        assert not outside, f"M={M} N={N} K={K}: written outside C: {outside[:8]}"
        missing = [(i, j) for i in range(M) for j in range(N) if (i, j) not in writes]
        exp = golden_matmul(A, B)
        assert not missing and np.array_equal(C.astype(np.int64), exp), \
            f"M={M} N={N} K={K} (missing {missing[:8]})\n got=\n{C}\n exp=\n{exp}"
        assert self.last_c_write == self.done_cycle, "last C write in tile_ctrl's done cycle"
        assert self.first_c_read > self.last_c_write, \
            f"first C read (cycle {self.first_c_read}) not after the last C write (cycle {self.last_c_write})"
        if during is None:
            assert seen - t0 == L + 2, f"DONE seen {seen - t0} cycles after START, contract L+2 = {L + 2}"
        self.dut._log.info(f"M={M:2} N={N:2} K={K:2}: {-(-M // n) * -(-N // n)} tiles, L={L}, "
                           f"DONE seen at L+{seen - t0 - L}, C == A@B")
        return C


async def make_system(dut):
    p = Params()
    m = p.map
    dut._log.info(f"SEED={p.seed} ARRAY_DIM={p.n} MAX_M={p.max_m} MAX_N={p.max_n} MAX_K={p.max_k} HOST_AW={m.addr_w}")
    assert len(dut.host_addr) == m.addr_w and len(dut.host_rdata) == m.host_w
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    dut.rst_n.value = 0
    dut.host_we.value = dut.host_re.value = 0
    dut.host_addr.value = dut.host_wdata.value = 0
    for _ in range(2):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.rst_n.value = 1
    s = System(dut, p)
    cocotb.start_soon(s.observer())
    return s


def rand_ops(rng, M, N, K):
    A = np.array([[rng.randint(-128, 127) for _ in range(K)] for _ in range(M)])
    B = np.array([[rng.randint(-128, 127) for _ in range(N)] for _ in range(K)])
    return A, B

# --------------------------------------------------------------------------- #
# tests
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_directed_matmuls(dut):
    """Single full tile, 1x1, edge tiles in M / N / both, skinny, K=1, multi-tile."""
    s = await make_system(dut)
    n = s.p.n
    for (M, N, K) in [(n, n, n), (1, 1, 1), (2 * n + 1, n - 1, 3), (1, 3 * n, 2),
                      (3 * n, 1, 1), (n + 1, 2 * n + 1, 5), (2 * n, 2 * n, 2)]:
        await s.run_job(*rand_ops(s.rng, M, N, K))


@cocotb.test()
async def test_random_matmuls(dut):
    """Random shapes back-to-back, full-range operands, no reset between jobs."""
    s = await make_system(dut)
    n, rng = s.p.n, s.rng
    for _ in range(NUM_RANDOM_JOBS):
        M = rng.randint(1, min(s.p.max_m, 3 * n + 2))
        N = rng.randint(1, min(s.p.max_n, 3 * n + 2))
        K = rng.randint(1, 9)
        await s.run_job(*rand_ops(rng, M, N, K))
        for _ in range(rng.randint(0, 2)):
            await s.edge()


@cocotb.test()
async def test_illegal_dims(dut):
    """Illegal dims through the bus: DONE | CMD_ERROR, nothing written; a legal
    job afterwards is clean."""
    s = await make_system(dut)
    m = s.m
    for cfg in [(0, 2, 2), (2, 0, 2), (2, 2, 0), (s.p.max_m + 1, 2, 2)]:
        await s.start(*cfg)
        assert await s.poll(10) == STATUS_DONE | STATUS_CMD_ERROR
        assert not s.writes and not s.errors
        s.writes = None
    await s.run_job(*rand_ops(s.rng, s.p.n + 1, 2, 3))


@cocotb.test()
async def test_busy_rules(dut):
    """During a job: a second START and A / B writes (to elements the job reads)
    are ignored, HOST_ERROR is set, and C is exactly the loaded A @ B."""
    s = await make_system(dut)
    m, n = s.m, s.p.n
    M, N, K = 2 * n + 1, n + 2, 4
    A, B = rand_ops(s.rng, M, N, K)

    async def interfere(i):
        # only early polls: a START that lands in the done cycle (BUSY low) is correctly
        # TAKEN and starts a new job, which is not what this test is about
        if i < 3:
            await s.write(m.reg(REG_CTRL), CTRL_START)
            await s.write(m.a_elem(i % M, i % K), 99)
            await s.write(m.b_elem(i % K, i % N), -99)

    await s.run_job(A, B, during=interfere, expect=STATUS_DONE | STATUS_HOST_ERROR)
    await s.run_job(A, B)                                   # START clears HOST_ERROR


@cocotb.test()
async def test_reset_mid_job(dut):
    """Async reset during a job: STATUS and DIM registers back to 0; a full job
    afterwards (reload, start, poll, read) is correct."""
    s = await make_system(dut)
    m, n = s.m, s.p.n
    for cut in (0, 3, 20):
        A, B = rand_ops(s.rng, n + 2, n + 1, 3)
        await s.load(A, B)
        await s.start(n + 2, n + 1, 3)
        for _ in range(cut):
            await s.edge()
        await Timer(2, unit="ns")
        dut.rst_n.value = 0
        await Timer(1, unit="ns")
        for _ in range(2):
            await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        dut.rst_n.value = 1
        s.writes = None
        assert await s.read(m.reg(REG_STATUS)) == 0
        assert await s.read(m.reg(REG_DIM_M)) == 0
        await s.run_job(*rand_ops(s.rng, 2 * n, n, 2))
