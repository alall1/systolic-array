"""
cocotb testbench for "pe_grid" module

Run:  
    make
    make WAVES=1  (dump waves for GTKWave)

Golden model: numpy A @ B.
Driver: applies diagonal skew from build_skew_schedule, then waits on "out_ready" signal and reads acc grid.
"""

import random
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer, First, Edge

import numpy as np

from utils import to_signed, to_unsigned, pack_a, pack_b, golden_matmul, build_skew_schedule

CLK_PERIOD_NS = 10

RECT_SHAPES = [
    (3, 5, 4),   # fully asymmetric
    (1, 5, 4),   # M=1
    (3, 5, 1),   # N=1
    (3, 1, 4),   # K=1
    (1, 1, 1),   # degenerate
]

# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

def get_params(dut):
    data_width = int(os.environ["DATA_WIDTH"])
    acc_width  = int(os.environ["ACC_WIDTH"])
    array_dim = int(os.environ["ARRAY_DIM"])

    return data_width, acc_width, array_dim

async def start_clock(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())

async def reset_dut(dut, P, cycles=2):
    """Assert active-low reset for `cycles`, then release on an edge."""
    dut.rst_n.value = 0
    for i in range(P):
        dut.a_in[i].value = 0
        dut.b_in[i].value = 0
    for _ in range(cycles):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")

async def wait_out_ready(dut, timeout_cycles):
    """Wait for out_ready, but fail if it doesn't assert in time"""
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        if int(dut.out_ready.value) == 1:
            return True
    return False

def read_out_grid(dut, acc_width, P, M=None, N=None):
    """Read the acc grid as signed ints. Defaults to the full PxP array; pass M, N to read only the top-left MxN sub-region."""
    if M is None:
        M = P
    if N is None:
        N = P
    grid = np.zeros((M, N), dtype=object)
    for i in range(M):
        for j in range(N):
            grid[i][j] = to_signed(int(dut.out[i][j].value), acc_width)
    return grid

async def drive_schedule(dut, A, B, P, data_width, first=False):
    """Drive one A@B (MxK * KxN) through a PxP grid using the unified skew schedule with per-direction valids. Returns n_cycles driven."""
    a_data, a_valid, a_first, b_data, b_valid, n_cycles = build_skew_schedule(A, B, P)

    for t in range(n_cycles):
        for i in range(P):
            dut.a_in[i].value = pack_a(a_data[i][t], a_valid[i][t], a_first[i][t], data_width)
            dut.b_in[i].value = pack_b(b_data[i][t], b_valid[i][t], data_width)
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")

    for i in range(P):
        dut.a_in[i].value = 0
        dut.b_in[i].value = 0

    return n_cycles

async def run_matmul(dut, A, B, data_width, acc_width, P, first=False):
    """Square convenience wrapper: drive NxN * NxN, wait out_ready, read grid."""
    n_cycles = await drive_schedule(dut, A, B, P, data_width, first=first)
    ok = await wait_out_ready(dut, timeout_cycles=2 * n_cycles + 10)
    assert ok, (
        f"out_ready never asserted within timeout for P={P}; "
        f"check the grid's out_ready counter/FSM")
    return read_out_grid(dut, acc_width, P)

def check_grid(got, expected, ctx=""):
    prefix = f"[{ctx}] " if ctx else ""
    if not np.array_equal(got, expected):
        raise AssertionError(
            f"{prefix}matmul mismatch\ngot=\n{got}\nexpected=\n{expected}")

def inactive_pes(M, N, P):
    """PEs that must stay idle: dead rows (i>=M) UNION dead cols (j>=N)."""
    return {(i, j) for i in range(P) for j in range(P) if (i >= M or j >= N)}

class AccEnMonitor:
    """Samples every PE's in_valid once per cycle for the whole run, recording any inactive PE ever enabled. Forked coroutine so no cycle is missed."""

    def __init__(self, dut, P):
        self.dut = dut
        self.P = P
        self.ever_enabled = set()
        self._stop = False

    async def run(self):
        while not self._stop:
            await RisingEdge(self.dut.clk)
            await Timer(1, unit="ns")
            for i in range(self.P):
                for j in range(self.P):
                    if int(get_pe_valid(self.dut, i, j).value) == 1:
                        self.ever_enabled.add((i, j))

    def stop(self):
        self._stop = True

def get_pe_valid(dut, i, j):
    """Handle to PE(i,j)'s in_valid to check if it is computing a MAC"""
    return dut.row_loop[i].col_loop[j].pe.in_valid

# --------------------------------------------------------------------------- #
# tests
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_zeros(dut):
    """All-zero operands -> all-zero result"""
    data_width, acc_width, P = get_params(dut)
    await start_clock(dut)
    await reset_dut(dut, P, data_width)

    A = np.zeros((P, P), dtype=int)
    B = np.zeros((P, P), dtype=int)
    got = await run_matmul(dut, A, B, data_width, acc_width, P)
    check_grid(got, golden_matmul(A, B), ctx="zeros")

@cocotb.test()
async def test_identity(dut):
    """A @ I == A: a simple, readable first case"""
    data_width, acc_width, P = get_params(dut)
    await start_clock(dut)
    await reset_dut(dut, P, data_width)

    lo = -(1 << (data_width - 1))
    hi = (1 << (data_width - 1)) - 1
    A = np.random.randint(lo, hi + 1, size=(P, P))   # fits in DATA_WIDTH
    B = np.eye(P, dtype=int)
    got = await run_matmul(dut, A, B, data_width, acc_width, P)
    check_grid(got, golden_matmul(A, B), ctx="identity")

@cocotb.test()
async def test_random_matmul(dut):
    """Randomized A, B across many trials; NOT for consecutive matmuls--fresh reset per trial. Grid-size square matmuls only"""
    data_width, acc_width, P = get_params(dut)
    await start_clock(dut)

    hi = (1 << (data_width - 1)) - 1
    # Keep operands modest so N-term sums stay well inside ACC_WIDTH.
    bound = max(2, min(hi, 1 << (data_width // 2)))

    for trial in range(30):
        await reset_dut(dut, P, data_width)
        A = np.random.randint(-bound, bound, size=(P, P))
        B = np.random.randint(-bound, bound, size=(P, P))
        got = await run_matmul(dut, A, B, data_width, acc_width, P)
        check_grid(got, golden_matmul(A, B), ctx=f"rand[{trial}]")

@cocotb.test()
async def test_rectangular_matmul(dut):
    """
    Rectangular MxK * KxN, fresh reset per shape. Two checks per shape:
      (1) active MxN sub-region equals numpy A@B
      (2) inactive PEs (i>=M OR j>=N) never raise in_valid (continuous sample)
    """
    data_width, acc_width, P = get_params(dut)
    await start_clock(dut)

    for (M, K, N) in RECT_SHAPES:
        if M > P or N > P:
            continue
        await reset_dut(dut, P, data_width)

        # fresh monitor per shape so violations don't bleed across shapes
        monitor = AccEnMonitor(dut, P)
        mon_task = cocotb.start_soon(monitor.run())

        rng = np.random.default_rng(hash((M, K, N)) & 0xFFFF)
        A = rng.integers(-4, 4, size=(M, K))
        B = rng.integers(-4, 4, size=(K, N))

        n_cycles = await drive_schedule(dut, A, B, P, data_width)
        ok = await wait_out_ready(dut, timeout_cycles=2 * n_cycles + 10)
        assert ok, f"out_ready never asserted for M={M},K={K},N={N}"

        # let the monitor catch trailing enables, then stop it
        for _ in range(2):
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")
        monitor.stop()

        # (1) correctness on the active sub-region
        got = read_out_grid(dut, acc_width, P, M, N)
        exp = golden_matmul(A, B)
        if not np.array_equal(got, exp):
            raise AssertionError(
                f"[M={M},K={K},N={N}] active MxN mismatch\n"
                f"got=\n{got}\nexpected=\n{exp}")

        # (2) inactive PEs stayed idle
        violations = sorted(inactive_pes(M, N, P) & monitor.ever_enabled)
        assert not violations, f"[M={M},K={K},N={N}] inactive PEs raised in_valid (should be idle): {violations}"

@cocotb.test()
async def test_consecutive_matmul(dut):
    """
    Consecutive matmuls of RANDOM shapes (square or rectangular), NO reset between them. 
    Checks per matmul: 
    (1) active MxN sub-region equals A@B,
    (2) inactive PEs (i>=M OR j>=N) never raised in_valid during that matmul.
    """
    data_width, acc_width, P = get_params(dut)
    await start_clock(dut)
    await reset_dut(dut, P, data_width)   # ONE reset at the very start only

    monitor = AccEnMonitor(dut, P)
    cocotb.start_soon(monitor.run())

    rng = np.random.default_rng(20260829)
    for trial in range(30):
        M = int(rng.integers(1, P + 1))
        N = int(rng.integers(1, P + 1))
        K = int(rng.integers(1, 7))
        A = rng.integers(-4, 4, size=(M, K))
        B = rng.integers(-4, 4, size=(K, N))

        # snapshot enables seen so far; anything new is from THIS matmul
        before = set(monitor.ever_enabled)

        n_cycles = await drive_schedule(dut, A, B, P, data_width, first=True)
        ok = await wait_out_ready(dut, timeout_cycles=2 * n_cycles + 10)
        assert ok, f"out_ready never asserted for trial {trial} (M={M},K={K},N={N})"

        # let the monitor catch trailing enables before diffing
        for _ in range(2):
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")

        # (1) active sub-region correctness
        got = read_out_grid(dut, acc_width, P, M, N)
        exp = golden_matmul(A, B)
        if not np.array_equal(got, exp):
            raise AssertionError(f"consecutive[{trial}] M={M},K={K},N={N} mismatch got={got.tolist()} expected={exp.tolist()}")

        # (2) no inactive PE was enabled during this matmul
        enabled_this = monitor.ever_enabled - before
        violations = sorted(inactive_pes(M, N, P) & enabled_this)
        assert not violations, f"consecutive[{trial}] M={M},K={K},N={N} inactive PEs raised in_valid: {violations}"
