"""
cocotb testbench for "pe" module

Run:  
    make
    make WAVES=1  (dump waves for GTKWave)

Notes on timing
---------------
The DUT registers everything on the rising edge, so an input presented in the window before edge N shows up on the outputs after edge N. The pattern used
throughout: drive inputs, `await RisingEdge(clk)` to cross the edge, then a tiny settle delay before sampling so we read post-edge values, not a race.
"""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from pe_model import PEModel
from utils import to_signed, to_unsigned

CLK_PERIOD_NS = 10

# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

def get_params(dut):
    """Read DATA_WIDTH / ACC_WIDTH from the elaborated DUT."""
    data_width = int(dut.DATA_WIDTH.value)
    acc_width = int(dut.ACC_WIDTH.value)
    return data_width, acc_width

async def start_clock(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())

async def reset_dut(dut, cycles: int = 2):
    """Assert active-low reset for `cycles`, then release on an edge."""
    dut.rst_n.value = 0
    dut.in_a.value = 0
    dut.in_b.value = 0
    dut.in_a_valid.value = 0
    dut.in_b_valid.value = 0
    for _ in range(cycles):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.rst_n.value = 1

async def step(dut, in_a: int, in_b: int, in_a_valid: int, in_b_valid: int, in_first: int, data_width: int):
    """Drive one set of inputs and cross one clock edge

    Inputs are written as unsigned bit patterns so negative operands are fed in correctly regardless of the port's signedness.
    """
    dut.in_a.value = to_unsigned(in_a, data_width)
    dut.in_b.value = to_unsigned(in_b, data_width)
    dut.in_a_valid.value = in_a_valid
    dut.in_b_valid.value = in_b_valid
    dut.in_first.value = in_first
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")  # settle past the edge before anyone samples

def check(dut, model: PEModel, data_width: int, acc_width: int, ctx: str = ""):
    """Compare every DUT output against the model."""
    dut_out_a = int(dut.out_a.value)
    dut_out_b = int(dut.out_b.value)
    dut_acc = int(dut.acc.value) & ((1 << acc_width) - 1)
    dut_a_valid = int(dut.out_a_valid.value)
    dut_b_valid = int(dut.out_b_valid.value)
    dut_first = int(dut.out_first.value)

    prefix = f"[{ctx}] " if ctx else ""
    assert dut_out_a == model.out_a, (
        f"{prefix}out_a: dut={dut_out_a} exp={model.out_a}")
    assert dut_out_b == model.out_b, (
        f"{prefix}out_b: dut={dut_out_b} exp={model.out_b}")
    assert dut_a_valid == model.out_a_valid, (
        f"{prefix}out_a_valid: dut={dut_a_valid} exp={model.out_a_valid}")
    assert dut_b_valid == model.out_b_valid, (
            f"{prefix}out_b_valid: dut={dut_b_valid} exp={model.out_b_valid}")
    assert dut_first == model.out_first, (
            f"{prefix}out_first: dut={dut_first} exp={model.out_first}")
    assert dut_acc == model.acc, (
        f"{prefix}acc: dut={to_signed(dut_acc, acc_width)} "
        f"exp={model.acc_signed} (raw dut={dut_acc} exp={model.acc})")


# --------------------------------------------------------------------------- #
# tests
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_reset(dut):
    """After reset, every output is 0"""
    # data_width, acc_width = get_params(dut)
    await start_clock(dut)
    await reset_dut(dut)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")

    assert int(dut.out_a.value) == 0
    assert int(dut.out_b.value) == 0
    assert int(dut.acc.value) == 0
    assert int(dut.out_a_valid.value) == 0
    assert int(dut.out_b_valid.value) == 0
    assert int(dut.out_first.value) == 0

@cocotb.test()
async def test_single_mac(dut):
    """One valid cycle: acc becomes a*b, operands appear on out_a/out_b"""
    data_width, acc_width = get_params(dut)
    model = PEModel(data_width, acc_width)

    await start_clock(dut)
    await reset_dut(dut)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    model.reset()

    a, b = 7, -3
    await step(dut, a, b, in_a_valid=1, in_b_valid=1, in_first=0, data_width=data_width)
    model.step(a, b, in_a_valid=1, in_b_valid=1)
    check(dut, model, data_width, acc_width, ctx="single_mac")

@cocotb.test()
async def test_accumulation(dut):
    """A dense stream of valid cycles accumulates correctly"""
    data_width, acc_width = get_params(dut)
    model = PEModel(data_width, acc_width)

    await start_clock(dut)
    await reset_dut(dut)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    model.reset()

    lo = -(1 << (data_width - 1))
    hi = (1 << (data_width - 1)) - 1

    for i in range(20):
        a = random.randint(lo, hi)
        b = random.randint(lo, hi)
        await step(dut, a, b, in_a_valid = 1, in_b_valid=1, in_first=0, data_width=data_width)
        model.step(a, b, in_a_valid = 1, in_b_valid=1)
        check(dut, model, data_width, acc_width, ctx=f"accum[{i}]")

@cocotb.test()
async def test_operand_propagation(dut):
    """out_a/out_b are the previous cycle's inputs; out_valid tracks in_a_valid = 1, in_b_valid delayed one cycle"""
    data_width, acc_width = get_params(dut)
    model = PEModel(data_width, acc_width)

    await start_clock(dut)
    await reset_dut(dut)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    model.reset()

    vectors = [(1, 2), (3, 4), (-5, 6), (7, -8), (0, 0)]
    for i, (a, b) in enumerate(vectors):
        await step(dut, a, b, in_a_valid = 1, in_b_valid=1, in_first=0, data_width=data_width)
        model.step(a, b, in_a_valid = 1, in_b_valid=1)
        check(dut, model, data_width, acc_width, ctx=f"prop[{i}]")

# TODO: test if only 1 valid makes a bubble
@cocotb.test()
async def test_bubble_holds_acc(dut):
    """
    Accumulate, insert an in_a_valid = 1, in_b_valid=0 bubble, then resume. Across the bubble:
      - acc must HOLD its value
      - out_a/out_b/out_valid drop to 0
    Then accumulation resumes from the held value.
    """
    data_width, acc_width = get_params(dut)
    model = PEModel(data_width, acc_width)

    await start_clock(dut)
    await reset_dut(dut)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    model.reset()

    # phase 1: accumulate a few real terms
    for (a, b) in [(2, 3), (4, 5), (-1, 6)]:
        await step(dut, a, b, in_a_valid = 0, in_b_valid=0, in_first=0, data_width=data_width)
        model.step(a, b, in_a_valid = 0, in_b_valid=0)
        check(dut, model, data_width, acc_width, ctx="bubble/pre")

    acc_before = model.acc

    # phase 2: a bubble — acc must not change (in_a_valid = 0 and in_b_valid = 0)
    await step(dut, 99, 99, in_a_valid = 0, in_b_valid=0, in_first=0, data_width=data_width)
    model.step(99, 99, in_a_valid = 0, in_b_valid=0)
    check(dut, model, data_width, acc_width, ctx="bubble/both zero")
    assert model.acc == acc_before, "model sanity: acc should hold on bubble"

    # phase 3: a bubble — acc must not change (in_a_valid = 1 and in_b_valid = 0)
    await step(dut, 99, 99, in_a_valid = 1, in_b_valid=0, in_first=0, data_width=data_width)
    model.step(99, 99, in_a_valid = 1, in_b_valid=0)
    check(dut, model, data_width, acc_width, ctx="bubble/one zero")
    assert model.acc == acc_before, "model sanity: acc should hold on bubble"

    # phase 4: resume; picks up from the held value
    for (a, b) in [(7, 2), (3, 3)]:
        await step(dut, a, b, in_a_valid = 1, in_b_valid=1, in_first=0, data_width=data_width)
        model.step(a, b, in_a_valid = 1, in_b_valid=1)
        check(dut, model, data_width, acc_width, ctx="bubble/post")

@cocotb.test()
async def test_in_first(dut):
    """A 'in_first' input sets accumulator equal to current multiply product"""
    data_width, acc_width = get_params(dut)
    model = PEModel(data_width, acc_width)

    await start_clock(dut)
    await reset_dut(dut)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    model.reset()

    lo = -(1 << (data_width - 1))
    hi = (1 << (data_width - 1)) - 1

    for i in range(5):
        a = random.randint(lo, hi)
        b = random.randint(lo, hi)
        await step(dut, a, b, in_a_valid = 1, in_b_valid=1, in_first=0, data_width=data_width)
        model.step(a, b, in_a_valid = 1, in_b_valid=1)
    
    await step(dut, a, b, in_a_valid = 1, in_b_valid=1, in_first=1, data_width=data_width)
    model.step(a, b, in_a_valid = 1, in_b_valid=1, in_first=1)

    check(dut, model, data_width, acc_width, ctx=f"accum[{i}]")

@cocotb.test()
async def test_random_regression(dut):
    """Randomized valid/bubble mix"""
    data_width, acc_width = get_params(dut)
    model = PEModel(data_width, acc_width)

    await start_clock(dut)
    await reset_dut(dut)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    model.reset()

    lo = -(1 << (data_width - 1))
    hi = (1 << (data_width - 1)) - 1

    for i in range(500):
        a = random.randint(lo, hi)
        b = random.randint(lo, hi)
        a_v = 1 if random.random() < 0.8 else 0
        b_v = 1 if random.random() < 0.8 else 0
        f = 1 if random.random() < 0.8 else 0
        await step(dut, a, b, in_a_valid = a_v, in_b_valid=b_v, in_first=f, data_width=data_width)
        model.step(a, b, in_a_valid = a_v, in_b_valid=b_v, in_first=f)
        check(dut, model, data_width, acc_width, ctx=f"rand[{i}]")