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

def get_params():
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

async def drain_out(dut, acc_width, P, M=None, N=None):
    if M is None:
        M = P
    if N is None:
        N = P

    grid = np.zeros((P, P), dtype=object)

    # capture current acc values into shadow registers
    dut.capture.value = 1
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.capture.value = 0

    # shift out P rows; drain_out is combinational (equal to bottom row shadow registers), so sample before the edge
    dut.shift_en.value = 1

    for i in reversed(range(P)):
        await Timer(1, unit="ns")
        for j in range(P):
            grid[i][j] = to_signed(int(dut.drain_out[j].value), acc_width)
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.shift_en.value = 0

    return grid[0:M, 0:N].copy()

async def drive_schedule(dut, A, B, P, data_width):
    """Drive one A@B (MxK * KxN) through a PxP grid using the unified skew schedule with per-direction valids. Returns n_cycles driven."""
    a_data, a_valid, a_first, b_data, b_valid, n_cycles = build_skew_schedule(A, B, P)

    for t in range(n_cycles):
        for i in range(P):
            dut.a_in[i].value = pack_a(a_data[i][t], a_valid[i][t], a_first[i][t], data_width)
            dut.b_in[i].value = pack_b(b_data[i][t], b_valid[i][t], data_width)
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")

async def run_matmul(dut, A, B, data_width, acc_width, P):
    """Square convenience wrapper: drive NxN * NxN, wait out_ready, read grid."""
    M = A.shape[0]
    N = B.shape[1]
    await drive_schedule(dut, A, B, P, data_width)
    return read_out_grid(dut, acc_width, P, M, N)

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
    data_width, acc_width, P = get_params()
    await start_clock(dut)
    await reset_dut(dut, P)

    A = np.zeros((P, P), dtype=int)
    B = np.zeros((P, P), dtype=int)
    got = await run_matmul(dut, A, B, data_width, acc_width, P)
    check_grid(got, golden_matmul(A, B), ctx="zeros")

@cocotb.test()
async def test_square(dut):
    """
    Randomized A, B across many trials; fresh reset per trial. Grid-size square matmuls only
    Checks:
        1. acc[i][j] matrix
        2. reconstructed drain_out matrix
    """
    data_width, acc_width, P = get_params()
    await start_clock(dut)

    hi = (1 << (data_width - 1)) - 1
    # Keep operands modest so N-term sums stay well inside ACC_WIDTH.
    bound = max(2, min(hi, 1 << (data_width // 2)))

    for trial in range(50):
        await reset_dut(dut, P)
        A = np.random.randint(-bound, bound, size=(P, P))
        B = np.random.randint(-bound, bound, size=(P, P))
        exp = golden_matmul(A, B)

        g_out = await run_matmul(dut, A, B, data_width, acc_width, P)
        d_out = await drain_out(dut, acc_width, P)
        check_grid(g_out, exp, ctx=f"rand_g_out[{trial}]")
        check_grid(d_out, exp, ctx=f"rand_d_out[{trial}]")

@cocotb.test()
async def test_rectangular(dut):
    """
    Rectangular MxK * KxN, fresh reset per shape. Two checks per shape:
        1. active MxN sub-region equals numpy A@B
        2. inactive PEs (i>=M OR j>=N) never raise in_valid (continuous sample)
    """
    data_width, acc_width, P = get_params()
    await start_clock(dut)

    for (M, K, N) in RECT_SHAPES:
        if M > P or N > P:
            continue
        await reset_dut(dut, P)

        # fresh monitor per shape so violations don't bleed across shapes
        monitor = AccEnMonitor(dut, P)
        cocotb.start_soon(monitor.run())

        rng = np.random.default_rng(hash((M, K, N)) & 0xFFFF)
        A = rng.integers(-4, 4, size=(M, K))
        B = rng.integers(-4, 4, size=(K, N))

        await drive_schedule(dut, A, B, P, data_width)

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
async def test_consecutive_square(dut):
    """
    Consecutive square matmuls, NO reset between them
    Checks:
        1. acc[i][j] matrix
        2. reconstructed drain_out matrix
    """
    data_width, acc_width, P = get_params()
    await start_clock(dut)
    await reset_dut(dut, P)   # ONE reset at the very start only

    hi = (1 << (data_width - 1)) - 1
    # Keep operands modest so N-term sums stay well inside ACC_WIDTH.
    bound = max(2, min(hi, 1 << (data_width // 2)))
    
    for trial in range(30):
        A = np.random.randint(-bound, bound, size=(P, P))
        B = np.random.randint(-bound, bound, size=(P, P))

        exp = golden_matmul(A, B)
        
        g_out = await run_matmul(dut, A, B, data_width, acc_width, P)
        d_out = await drain_out(dut, acc_width, P)
        
        check_grid(g_out, exp, ctx=f"rand_g_out[{trial}]")
        check_grid(d_out, exp, ctx=f"rand_d_out[{trial}]")

@cocotb.test()
async def test_consecutive_random(dut):
    """
    Consecutive matmuls of RANDOM shapes (square or rectangular), NO reset between them. 
    Checks:
        1. acc[i][j] matrix
        2. reconstructed drain_out matrix
    """
    data_width, acc_width, P = get_params()
    await start_clock(dut)
    await reset_dut(dut, P)   # ONE reset at the very start only

    monitor = AccEnMonitor(dut, P)
    cocotb.start_soon(monitor.run())

    rng = np.random.default_rng(20260829)
    for trial in range(30):
        M = int(rng.integers(1, P + 1))
        N = int(rng.integers(1, P + 1))
        K = int(rng.integers(1, 7))
        A = rng.integers(-4, 4, size=(M, K))
        B = rng.integers(-4, 4, size=(K, N))

        exp = golden_matmul(A, B)

        g_out = await run_matmul(dut, A, B, data_width, acc_width, P)
        d_out = await drain_out(dut, acc_width, P, M, N)

        check_grid(g_out, exp, ctx=f"rand_g_out[{trial}]")
        check_grid(d_out, exp, ctx=f"rand_d_out[{trial}]")


# --------------------------------------------------------------------------- #
# optional tests for debugging readability
# --------------------------------------------------------------------------- #
# @cocotb.test()
# async def test_identity(dut):
#     """A @ I == A: a simple, readable first case"""
#     data_width, acc_width, P = get_params()
#     await start_clock(dut)
#     await reset_dut(dut, P)

#     lo = -(1 << (data_width - 1))
#     hi = (1 << (data_width - 1)) - 1
#     A = np.random.randint(lo, hi + 1, size=(P, P))   # fits in DATA_WIDTH
#     B = np.eye(P, dtype=int)
#     got = await run_matmul(dut, A, B, data_width, acc_width, P)
#     check_grid(got, golden_matmul(A, B), ctx="identity")

# @cocotb.test()
# async def test_identity_drain(dut):
#     """A @ I == A: a simple, readable first case"""
#     data_width, acc_width, P = get_params()
#     await start_clock(dut)
#     await reset_dut(dut, P)

#     lo = -(1 << (data_width - 1))
#     hi = (1 << (data_width - 1)) - 1
#     A = np.random.randint(lo, hi + 1, size=(P, P))   # fits in DATA_WIDTH
#     B = np.eye(P, dtype=int)
#     got = await run_matmul(dut, A, B, data_width, acc_width, P)
#     drain = await drain_out(dut, acc_width, P)
#     exp = golden_matmul(A, B)
#     check_grid(got, exp, ctx="identity_drain_grid")
#     check_grid(drain, exp, ctx="identity_drain_out")