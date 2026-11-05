"""
cocotb system smoke test: tile_ctrl drives the UNCHANGED feeder + pe_grid, and
collector turns the grid's drain into row writes of C.

The testbench plays the two blocks that do not exist yet:
  - operand buffer: 1-cycle registered read, serves A[tile_m*n + lane][k] and
    B[k][tile_n*n + lane] for the address + tile ids presented one cycle earlier;
    junk otherwise (and on lanes >= M_t / >= N_t) so valid masking is exercised
  - result memory: applies collector's row writes (wr_row, wr_col + lane j for
    each enabled lane) and nothing else; it does no placement of its own

Golden model: numpy A @ B. Jobs run back-to-back on ONE grid with no reset, so
stale accumulators in inactive PEs of edge tiles are part of the test.

System-level checks:
  - fsm_contract §9.11 (feed_done only in FEED), feed_start never while the
    feeder is busy, job latency == L
  - collector_contract §7.6-7: every C element written exactly once, nothing
    outside M x N or outside a job, last write in tile_ctrl's done cycle
  - the collector's out-of-contract-input assertions are live in this build,
    so tile_ctrl is also checked to never capture mid-drain or shift idle

Optional env vars: SEED (default 12345)
"""

import os
import random

import cocotb
import numpy as np
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer
from cocotb.utils import get_sim_time

from utils import to_signed, golden_matmul

CLK_PERIOD_NS = 10
NUM_RANDOM_JOBS = 25
STATE_FEED = 2          # tile_ctrl state_t encoding: IDLE, ISSUE, FEED, ...


def job_latency(M, N, K, n):
    """Contract §6.2 closed form."""
    T_m, T_n = -(-M // n), -(-N // n)
    return T_m * T_n * (K + n + 1) + T_n * M + T_m * N


class Params:
    def __init__(self):
        self.data_width = int(os.environ["DATA_WIDTH"])
        self.acc_width = int(os.environ["ACC_WIDTH"])
        self.n = int(os.environ["ARRAY_DIM"])
        self.max_m = int(os.environ["MAX_M"])
        self.max_n = int(os.environ["MAX_N"])
        self.seed = int(os.environ.get("SEED", "12345"))


class System:
    def __init__(self, dut, p):
        self.dut = dut
        self.p = p
        self.rng = random.Random(p.seed)
        self.A = None
        self.B = None
        self.mem = None             # {(row, col): value} for the job in flight, None between jobs
        self.last_write_ns = None
        self.errors = []
        self.rows_written = 0

    # ------------------------------------------------------------------ #
    async def operand_buffer(self):
        dut, n, w = self.dut, self.p.n, self.p.data_width
        junk = random.Random(self.p.seed + 1)
        captured = None
        while True:
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")
            for lane in range(n):
                a = b = None
                if captured is not None:
                    col, row, tm, tn = captured
                    r, c = tm * n + lane, tn * n + lane
                    if r < self.A.shape[0]:
                        a = int(self.A[r][col])
                    if c < self.B.shape[1]:
                        b = int(self.B[row][c])
                dut.rd_a[lane].value = (a if a is not None else junk.getrandbits(w) - (1 << (w - 1))) & ((1 << w) - 1)
                dut.rd_b[lane].value = (b if b is not None else junk.getrandbits(w) - (1 << (w - 1))) & ((1 << w) - 1)
            if int(dut.rd_en.value):
                captured = (int(dut.rd_col_a.value), int(dut.rd_row_b.value),
                            int(dut.tile_m.value), int(dut.tile_n.value))
            else:
                captured = None

    async def result_memory(self):
        """Dumb row-write memory: commits what the write port shows each cycle."""
        dut, n, aw = self.dut, self.p.n, self.p.acc_width
        while True:
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")
            if not int(dut.wr_en.value):
                continue
            if self.mem is None:
                self.errors.append("collector wrote outside a job")
                continue
            row, col, lanes = int(dut.wr_row.value), int(dut.wr_col.value), int(dut.wr_lane_en.value)
            for j in range(n):
                if lanes >> j & 1:
                    key = (row, col + j)
                    if key in self.mem:
                        self.errors.append(f"C{key} written twice")
                    self.mem[key] = to_signed(int(dut.wr_data[j].value), aw)
            self.rows_written += 1
            self.last_write_ns = get_sim_time(unit="ns")

    async def system_monitor(self):
        dut = self.dut
        while True:
            await FallingEdge(dut.clk)
            if not int(dut.rst_n.value):
                continue
            state = int(dut.u_ctrl.state.value)
            if int(dut.feed_done.value) and state != STATE_FEED:
                self.errors.append(f"feed_done in tile_ctrl state {state} (contract §9.11)")
            if int(dut.feed_start.value) and int(dut.feeder_busy.value):
                self.errors.append("feed_start while feeder busy")

    # ------------------------------------------------------------------ #
    async def run_job(self, A, B):
        dut, n = self.dut, self.p.n
        A, B = np.asarray(A), np.asarray(B)
        M, K = A.shape
        N = B.shape[1]
        self.A, self.B = A, B
        self.mem = {}
        dut.start.value = 1
        dut.M.value, dut.N.value, dut.K.value = M, N, K
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        dut.start.value = 0
        L = job_latency(M, N, K, n)
        for t in range(L + 5):
            if int(dut.done.value):
                break
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")
        else:
            raise AssertionError(f"M={M} N={N} K={K}: no done within {L + 5} cycles")
        done_ns = get_sim_time(unit="ns")
        await Timer(1, unit="ns")       # the last row is written in this same timestep: let the memory run first
        mem, self.mem = self.mem, None
        assert not self.errors, "\n".join(self.errors[:10])
        outside = sorted(k for k in mem if not (k[0] < M and k[1] < N))
        assert not outside, f"M={M} N={N} K={K}: written outside C: {outside[:8]}"
        missing = [(i, j) for i in range(M) for j in range(N) if (i, j) not in mem]
        C = np.array([[mem.get((i, j), 0) for j in range(N)] for i in range(M)], dtype=object)
        exp = golden_matmul(A, B)       # data first: the result matters more than the timing
        assert not missing and np.array_equal(C.astype(np.int64), exp), \
            f"M={M} N={N} K={K} (missing {missing[:8]})\n got=\n{C}\n exp=\n{exp}"
        assert self.last_write_ns == done_ns, \
            f"M={M} N={N} K={K}: last write at {self.last_write_ns} ns, done at {done_ns} ns"
        assert t == L, f"M={M} N={N} K={K}: done at t={t}, contract L={L}"
        assert not int(dut.error.value)
        dut._log.info(f"M={M:2} N={N:2} K={K:2}: {-(-M // n) * -(-N // n)} tiles, {L} cycles, C == A@B")


async def make_system(dut):
    p = Params()
    dut._log.info(f"SEED={p.seed} ARRAY_DIM={p.n} DATA_WIDTH={p.data_width} ACC_WIDTH={p.acc_width}")
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    dut.rst_n.value = 0
    dut.start.value = 0
    dut.M.value = dut.N.value = dut.K.value = 0
    for lane in range(p.n):
        dut.rd_a[lane].value = 0
        dut.rd_b[lane].value = 0
    for _ in range(2):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.rst_n.value = 1
    s = System(dut, p)
    s.A, s.B = np.zeros((1, 1), dtype=int), np.zeros((1, 1), dtype=int)
    for coro in (s.operand_buffer(), s.result_memory(), s.system_monitor()):
        cocotb.start_soon(coro)
    return s


def rand_ops(rng, M, N, K):
    A = np.array([[rng.randint(-8, 7) for _ in range(K)] for _ in range(M)])
    B = np.array([[rng.randint(-8, 7) for _ in range(N)] for _ in range(K)])
    return A, B


@cocotb.test()
async def test_directed_matmuls(dut):
    """Single full tile, edge tiles in M / N / both, skinny, K=1."""
    s = await make_system(dut)
    n = s.p.n
    for (M, N, K) in [(n, n, n), (1, 1, 1), (2 * n + 1, n - 1, 3), (1, 3 * n, 2),
                      (3 * n, 1, 1), (n + 1, 2 * n + 1, 5), (2 * n, 2 * n, 2)]:
        await s.run_job(*rand_ops(s.rng, M, N, K))
    assert s.rows_written > 0


@cocotb.test()
async def test_random_matmuls(dut):
    """Random shapes back-to-back on one grid, no reset between jobs."""
    s = await make_system(dut)
    n, rng = s.p.n, s.rng
    for _ in range(NUM_RANDOM_JOBS):
        M = rng.randint(1, min(s.p.max_m, 3 * n + 2))
        N = rng.randint(1, min(s.p.max_n, 3 * n + 2))
        K = rng.randint(1, 9)
        await s.run_job(*rand_ops(rng, M, N, K))
        for _ in range(rng.randint(0, 2)):
            await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
