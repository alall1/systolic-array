"""
cocotb testbench for "skew_buffer" module
"""

import random
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from utils import build_skew_schedule, to_signed, to_unsigned, pack_a, unpack_a, pack_b, unpack_b
from skew_buffer_model import SkewBufferModel

CLK_PERIOD_NS = 10

# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

def get_params():
    data_width = int(os.environ["DATA_WIDTH"])
    array_dim  = int(os.environ["ARRAY_DIM"])
    return data_width, array_dim

async def start_clock(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())

async def reset_dut(dut, array_dim: int, cycles: int = 2):
    """Assert active-low reset for `cycles`, then release on an edge"""
    dut.rst_n.value = 0
    for i in range(array_dim):
        dut.in_a[i].value = 0
        dut.in_b[i].value = 0
    for _ in range(cycles):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.rst_n.value = 1

async def step(dut, model: SkewBufferModel, in_a: list[int], in_b: list[int], data_width: int, array_dim: int):
    """Drive one set of inputs and cross one clock edge"""
    for i in range(array_dim):
        dut.in_a[i].value = pack_a(in_a[i], 0, 0, data_width)
        dut.in_b[i].value = pack_b(in_b[i], 0, data_width)

    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")  # settle past the edge before anyone samples

    exp_a, exp_b = model.step(in_a, in_b)

    out_a, out_b = [], []

    for i in range(array_dim):
        a_data, _, _ = unpack_a(dut.out_a[i].value, data_width)
        b_data, _ = unpack_b(dut.out_b[i].value, data_width)
        out_a.append(a_data)
        out_b.append(b_data)

    assert out_a == exp_a, (f"{prefix}out_a: dut={out_a} exp={exp_a}")
    assert out_b == exp_b, (f"{prefix}out_b: dut={out_b} exp={exp_b}")

# --------------------------------------------------------------------------- #
# tests
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_skew(dut):
    """Random input values, checking for proper skew"""
    data_width, array_dim = get_params()
    model = SkewBufferModel(array_dim)

    await start_clock(dut)
    await reset_dut(dut, array_dim)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    model.reset()

    lo = -(1 << (data_width - 1))
    hi = (1 << (data_width - 1)) - 1

    for _ in range(50):
        in_a = [random.randint(lo, hi) for _ in range(array_dim)]
        in_b = [random.randint(lo, hi) for _ in range(array_dim)]
        await step(dut, model, in_a, in_b, data_width, array_dim)

    for _ in range(array_dim):
        zeros = [0] * array_dim
        await step(dut, model, zeros, zeros, data_width, array_dim)