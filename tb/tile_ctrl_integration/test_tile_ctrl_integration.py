"""
cocotb system smoke test: tile_ctrl drives the UNCHANGED feeder + pe_grid,
collector turns the grid's drain into row writes of C, and three lane_ram
instances hold A, B and C in the tile-word layout (docs/memory_contract.md).

The testbench plays only the host:
  - before each job it loads A (transposed) and B tile words through the A/B
    write ports, padding lanes past M / N with junk so valid masking is exercised
  - after done it reads C back through the C read port, starting the cycle
    after done (memory_contract.md §6)

Golden model: numpy A @ B. Jobs run back-to-back on ONE grid with no reset, so
stale accumulators in inactive PEs of edge tiles are part of the test.

System-level checks (observers peek into the hierarchy; no extra ports):
  - fsm_contract §9.11 (feed_done only in FEED), feed_start never while the
    feeder is busy, job latency == L
  - collector_contract §7.6-7: every C element written exactly once, nothing
    outside M x N or outside a job, last write in tile_ctrl's done cycle
  - memory_contract §7.6: the feeder only ever reads A / B words the host loaded
  - memory_contract §6 / §7.8: a C read issued in the done cycle of the word
    written in that cycle returns the old word (directed test)
  - the collector's out-of-contract-input assertions and every block's SVA are
    live in this build

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
from tile_layout import (k_index_width, tile_index_width, pack_a_tiles, pack_b_tiles,
                         c_addr, c_words, unpack_c)

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
        self.max_k = int(os.environ["MAX_K"])
        self.seed = int(os.environ.get("SEED", "12345"))
        self.ki_w = k_index_width(self.max_k)
        self.tn_w = tile_index_width(self.max_n, self.n)


class System:
    def __init__(self, dut, p):
        self.dut = dut
        self.p = p
        self.rng = random.Random(p.seed)
        self.writes = None          # {(row, col)} written by the collector in the job in flight
        self.last_write_ns = None
        self.loaded_a = set()
        self.loaded_b = set()
        self.errors = []
        self.rows_written = 0

    # ------------------------------------------------------------------ #
    # observers
    # ------------------------------------------------------------------ #
    async def write_observer(self):
        """Watches the collector's write port into the C RAM (collector_contract §7.6-7)."""
        dut, n = self.dut, self.p.n
        col = dut.u_collector
        while True:
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")
            if not int(col.wr_en.value):
                continue
            if self.writes is None:
                self.errors.append("collector wrote outside a job")
                continue
            row, base, lanes = int(col.wr_row.value), int(col.wr_tile_n.value) * n, int(col.wr_lane_en.value)
            for j in range(n):
                if lanes >> j & 1:
                    if (row, base + j) in self.writes:
                        self.errors.append(f"C{(row, base + j)} written twice")
                    self.writes.add((row, base + j))
            self.rows_written += 1
            self.last_write_ns = get_sim_time(unit="ns")

    async def operand_read_observer(self):
        """The feeder only reads words the host loaded (memory_contract §7.6)."""
        dut = self.dut
        while True:
            await FallingEdge(dut.clk)
            if int(dut.u_mem_a.re.value):
                a, b = int(dut.u_mem_a.raddr.value), int(dut.u_mem_b.raddr.value)
                if a not in self.loaded_a:
                    self.errors.append(f"A read of unloaded word {a:#x}")
                if b not in self.loaded_b:
                    self.errors.append(f"B read of unloaded word {b:#x}")

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
    # host
    # ------------------------------------------------------------------ #
    async def load(self, A, B):
        """Write A / B tile words, one A and one B word per cycle (no job running)."""
        dut, p, rng = self.dut, self.p, self.rng
        dmask = (1 << p.data_width) - 1
        junk = lambda lane: rng.getrandbits(p.data_width)
        wa = sorted(pack_a_tiles(A.tolist(), p.n, p.ki_w, pad=junk).items())
        wb = sorted(pack_b_tiles(B.tolist(), p.n, p.ki_w, pad=junk).items())
        self.loaded_a, self.loaded_b = {a for a, _ in wa}, {a for a, _ in wb}
        for i in range(max(len(wa), len(wb))):
            for (we, addr, data, ws) in ((dut.a_we, dut.a_waddr, dut.a_wdata, wa),
                                         (dut.b_we, dut.b_waddr, dut.b_wdata, wb)):
                we.value = int(i < len(ws))
                if i < len(ws):
                    addr.value = ws[i][0]
                    for l in range(p.n):
                        data[l].value = ws[i][1][l] & dmask
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")
        dut.a_we.value = 0
        dut.b_we.value = 0

    async def read_c_word(self, addr):
        """One host read: address before an edge, data after it (1-cycle latency)."""
        dut = self.dut
        dut.c_re.value = 1
        dut.c_raddr.value = addr
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        dut.c_re.value = 0
        return [to_signed(int(dut.c_rdata[l].value), self.p.acc_width) for l in range(self.p.n)]

    async def read_c(self, M, N):
        words = {}
        for addr in c_words(M, N, self.p.n, self.p.tn_w):
            words[addr] = await self.read_c_word(addr)
        return np.array(unpack_c(lambda a: words[a], M, N, self.p.n, self.p.tn_w), dtype=object)

    # ------------------------------------------------------------------ #
    async def run_job(self, A, B, read_in_done_cycle=None):
        """Load, start, wait for done, then read C from the cycle after done.
        read_in_done_cycle: issue that C read IN the done cycle and return its data."""
        dut, n = self.dut, self.p.n
        A, B = np.asarray(A), np.asarray(B)
        M, K = A.shape
        N = B.shape[1]
        await self.load(A, B)
        self.writes = set()
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
        await Timer(1, unit="ns")       # the last row is written in this same timestep: let the observer run first
        writes, self.writes = self.writes, None
        if read_in_done_cycle is not None:
            early = await self.read_c_word(read_in_done_cycle)      # issued IN the done cycle (§6 race)
        else:
            early = None
            await RisingEdge(dut.clk)   # host registers done: reads start in cycle L+1 (§6)
            await Timer(1, unit="ns")
        C = await self.read_c(M, N)
        assert not self.errors, "\n".join(self.errors[:10])
        outside = sorted(k for k in writes if not (k[0] < M and k[1] < N))
        assert not outside, f"M={M} N={N} K={K}: written outside C: {outside[:8]}"
        missing = [(i, j) for i in range(M) for j in range(N) if (i, j) not in writes]
        exp = golden_matmul(A, B)       # data first: the result matters more than the timing
        assert not missing and np.array_equal(C.astype(np.int64), exp), \
            f"M={M} N={N} K={K} (missing {missing[:8]})\n got=\n{C}\n exp=\n{exp}"
        assert self.last_write_ns == done_ns, \
            f"M={M} N={N} K={K}: last write at {self.last_write_ns} ns, done at {done_ns} ns"
        assert t == L, f"M={M} N={N} K={K}: done at t={t}, contract L={L}"
        assert not int(dut.error.value)
        dut._log.info(f"M={M:2} N={N:2} K={K:2}: {-(-M // n) * -(-N // n)} tiles, {L} cycles, C == A@B")
        return C, early


async def make_system(dut):
    p = Params()
    dut._log.info(f"SEED={p.seed} ARRAY_DIM={p.n} DATA_WIDTH={p.data_width} ACC_WIDTH={p.acc_width}")
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    dut.rst_n.value = 0
    dut.start.value = 0
    dut.M.value = dut.N.value = dut.K.value = 0
    dut.a_we.value = dut.b_we.value = dut.c_re.value = 0
    dut.a_waddr.value = dut.b_waddr.value = dut.c_raddr.value = 0
    for lane in range(p.n):
        dut.a_wdata[lane].value = 0
        dut.b_wdata[lane].value = 0
    for _ in range(2):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.rst_n.value = 1
    s = System(dut, p)
    for coro in (s.write_observer(), s.operand_read_observer(), s.system_monitor()):
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
async def test_read_in_done_cycle_returns_old_word(dut):
    """memory_contract §6: the job's last write is committed on the edge that ends
    the done cycle, so a C read issued IN the done cycle (same edge) returns the
    old word. Two jobs of the same shape: the early read must see job 1's value."""
    s = await make_system(dut)
    n, p = s.p.n, s.p
    M, N, K = 2 * n + 1, n + 2, 3
    T_m, T_n = -(-M // n), -(-N // n)
    # last tile is (T_m-1, T_n-1); its row 0 is the job's last write
    last = c_addr((T_m - 1) * n, T_n - 1, p.tn_w)
    C1, _ = await s.run_job(*rand_ops(s.rng, M, N, K))
    A2, B2 = rand_ops(s.rng, M, N, K)
    while np.array_equal(golden_matmul(A2, B2)[(T_m - 1) * n], C1[(T_m - 1) * n].astype(np.int64)):
        A2, B2 = rand_ops(s.rng, M, N, K)
    C2, early = await s.run_job(A2, B2, read_in_done_cycle=last)
    lanes = range(N - (T_n - 1) * n)
    row = (T_m - 1) * n
    assert [early[l] for l in lanes] == [C1[row][(T_n - 1) * n + l] for l in lanes], "done-cycle read: old word"
    assert [C2[row][(T_n - 1) * n + l] for l in lanes] != [early[l] for l in lanes], "later read: new word"


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
