"""
cocotb testbench for "pe_grid" module

Run:  
    make
    make WAVES=1  (dump waves for GTKWave)

Golden model: numpy A @ B.
Driver: applies diagonal skew from build_skew_schedule, then waits on "out_ready" signal and reads acc grid.
"""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer, First, Edge

import numpy as np

from utils import to_signed, to_unsigned, golden_matmul, build_skew_schedule

CLK_PERIOD_NS = 10

# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

def get_params(dut):
    return (int(dut.DATA_WIDTH.value), int(dut.ACC_WIDTH.value), int(dut.N.value))

async def start_clock(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())

async def reset_dut(dut, N, data_width, cycles=2):
    dut.rst_n.value = 0
    dut.inp_valid.value = 0
    dut.inp_first.value = 0
    for i in range(N):
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

def read_out_grid(dut, N, acc_width):
    """Read the NxN accumulator grid as signed ints"""
    grid = np.zeros((N, N), dtype=object)
    for i in range(N):
        for j in range(N):
            grid[i][j] = to_signed(int(dut.out[i][j].value), acc_width)
    return grid

async def run_matmul(dut, A, B, data_width, acc_width, N):
    """Drive one A@B through the grid and return the read-out acc grid"""
    a_in, b_in, n_cycles = build_skew_schedule(A, B)

    # Drive the skewed streams; inp_valid high whole time to not screw up matmuls
    dut.inp_valid.value = 1
    for t in range(n_cycles):
        dut.inp_first.value = 1 if t == 0 else 0    # inp_first is high only for the very first cycle of the matmul
        for i in range(N):
            dut.a_in[i].value = to_unsigned(a_in[i][t], data_width)
        for j in range(N):
            dut.b_in[j].value = to_unsigned(b_in[j][t], data_width)
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")

    # Feed zeros but keep valid high while waiting for out_ready to assert.
    for i in range(N):
        dut.a_in[i].value = 0
        dut.b_in[i].value = 0

    # Generous timeout: schedule length plus slack.
    got_out_ready = await wait_out_ready(dut, timeout_cycles=2 * n_cycles + 10)
    assert got_out_ready, (
        f"out_ready never asserted within timeout for N={N}; "
        f"check the grid's out_ready counter/FSM")

    return read_out_grid(dut, N, acc_width)

def check_grid(got, expected, ctx=""):
    prefix = f"[{ctx}] " if ctx else ""
    if not np.array_equal(got, expected):
        raise AssertionError(
            f"{prefix}matmul mismatch\ngot=\n{got}\nexpected=\n{expected}")

# --------------------------------------------------------------------------- #
# tests
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_identity(dut):
    """A @ I == A: a simple, readable first case"""
    data_width, acc_width, N = get_params(dut)
    await start_clock(dut)
    await reset_dut(dut, N, data_width)

    A = np.arange(1, N * N + 1).reshape(N, N)
    B = np.eye(N, dtype=int)
    got = await run_matmul(dut, A, B, data_width, acc_width, N)
    check_grid(got, golden_matmul(A, B), ctx="identity")

@cocotb.test()
async def test_single_matmul(dut):
    """A fixed small case verifiable by hand"""
    data_width, acc_width, N = get_params(dut)
    await start_clock(dut)
    await reset_dut(dut, N, data_width)

    A = (np.arange(N * N).reshape(N, N) % 5) + 1
    B = ((np.arange(N * N).reshape(N, N) * 2) % 7) - 3
    got = await run_matmul(dut, A, B, data_width, acc_width, N)
    check_grid(got, golden_matmul(A, B), ctx="small_known")

@cocotb.test()
async def test_multiple_matmul(dut):
    """Randomized A, B across many trials; fresh reset per trial to clear accumulator between matmuls, since no back-to-back matmuls yet"""
    data_width, acc_width, N = get_params(dut)
    await start_clock(dut)

    hi = (1 << (data_width - 1)) - 1
    # Keep operands modest so N-term sums stay well inside ACC_WIDTH.
    bound = max(2, min(hi, 1 << (data_width // 2)))

    for trial in range(30):
        # await reset_dut(dut, N, data_width)
        A = np.random.randint(-bound, bound, size=(N, N))
        B = np.random.randint(-bound, bound, size=(N, N))
        got = await run_matmul(dut, A, B, data_width, acc_width, N)
        check_grid(got, golden_matmul(A, B), ctx=f"rand[{trial}]")

@cocotb.test()
async def test_zeros(dut):
    """All-zero operands -> all-zero result"""
    data_width, acc_width, N = get_params(dut)
    await start_clock(dut)
    await reset_dut(dut, N, data_width)

    A = np.zeros((N, N), dtype=int)
    B = np.zeros((N, N), dtype=int)
    got = await run_matmul(dut, A, B, data_width, acc_width, N)
    check_grid(got, golden_matmul(A, B), ctx="zeros")