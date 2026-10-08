"""
cocotb testbench for "read_sequencer" module
"""

import random
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from utils import unpack_a, unpack_b
from read_sequencer_model import ReadSequencerModel

CLK_PERIOD_NS = 10

# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

def get_params():
    data_width = int(os.environ["DATA_WIDTH"])
    array_dim  = int(os.environ["ARRAY_DIM"])
    return data_width, array_dim

def mask(v, data_width):
    """Reduce to raw data_width bits so signed/unsigned interpretation can't
    cause a false mismatch — the sequencer is a pure conduit for bits."""
    return v & ((1 << data_width) - 1)

async def start_clock(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())

async def reset_dut(dut, cycles: int = 2):
    """Assert active-low reset for `cycles`, then release off-edge."""
    dut.rst_n.value   = 0
    dut.start.value   = 0
    dut.M.value       = 0
    dut.N.value       = 0
    dut.K.value       = 0
    for _ in range(cycles):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.rst_n.value = 1

# --------------------------------------------------------------------------- #
# X-safe field accessors.
# Read valid/first straight from the binstr by bit position, so an X in the
# data field (invalid lanes drive 'x) can never crash the read. binstr is
# MSB-first, so LSB index i lives at binstr[-(i+1)].
#   a_payload_t [data : valid : first], widths [DATA_WIDTH : 1 : 1]
#     -> first = bit 0, valid = bit 1
#   b_payload_t [data : valid],         widths [DATA_WIDTH : 1]
#     -> valid = bit 0
# --------------------------------------------------------------------------- #

def _bit(binstr, i):
    """Bit i (LSB=0) of a cocotb binstr as 0/1, or None if x/z."""
    ch = binstr[-(i + 1)]
    return int(ch) if ch in "01" else None

def _valid_a(dut, lane):
    return _bit(str(dut.out_a[lane].value), 1)

def _first_a(dut, lane):
    return _bit(str(dut.out_a[lane].value), 0)

def _valid_b(dut, lane):
    return _bit(str(dut.out_b[lane].value), 0)

# --------------------------------------------------------------------------- #
# stub buffer: models the sync buffer contract.
# Captures rd_col_a / rd_row_b at an edge, returns rd_a / rd_b one edge later.
# Content is a KNOWN function of (index, lane) so the checker can verify both
# that the right column/row was requested AND that lane order is preserved.
# --------------------------------------------------------------------------- #

def buf_a(col, lane, array_dim):
    return (col * array_dim + lane) & 0xFF

def buf_b(row, lane, array_dim):
    return (0x80 + row * array_dim + lane) & 0xFF

async def stub_buffer(dut, array_dim):
    """Runs forever: 1-cycle registered read. Address at T -> data at T+1."""
    while True:
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")  # sample address issued this cycle (post-edge)
        rd_en = int(dut.rd_en.value)
        col   = int(dut.rd_col_a.value)
        row   = int(dut.rd_row_b.value)
        if rd_en:
            for lane in range(array_dim):
                dut.rd_a[lane].value = buf_a(col, lane, array_dim)
                dut.rd_b[lane].value = buf_b(row, lane, array_dim)
        # if not issuing, leave previous data (registered memory holds)

# --------------------------------------------------------------------------- #
# driver + checker for one feed
# --------------------------------------------------------------------------- #

async def run_feed(dut, data_width, array_dim, M, N, K):
    """Pulse start with M/N/K, then observe and check the whole feed."""
    dut.M.value     = M
    dut.N.value     = N
    dut.K.value     = K
    dut.start.value = 1
    await RisingEdge(dut.clk)   # this edge latches config, issues k=0 addr
    await Timer(1, unit="ns")
    dut.start.value = 0

    model = ReadSequencerModel(array_dim, M, N, K,
                               buf_a_fn=lambda c, l: buf_a(c, l, array_dim),
                               buf_b_fn=lambda r, l: buf_b(r, l, array_dim))

    done_seen = 0
    # feed is K cycles of data, arriving 1 cycle after each address.
    # observe K+2 cycles to catch done alignment and the tail.
    for cycle in range(K + 2):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")

        exp  = model.step()        # payload visible THIS data cycle, or None
        done = int(dut.done.value)

        if exp is None:
            # fill cycle or past end: every lane invalid, done low.
            for lane in range(array_dim):
                assert _valid_a(dut, lane) == 0, \
                    f"cycle {cycle} A lane {lane}: expected invalid (no payload)"
                assert _valid_b(dut, lane) == 0, \
                    f"cycle {cycle} B lane {lane}: expected invalid (no payload)"
            assert done == 0, f"cycle {cycle}: done asserted on empty cycle"
            continue

        exp_a, exp_b, exp_first = exp

        for lane in range(array_dim):
            va = _valid_a(dut, lane)
            vb = _valid_b(dut, lane)

            # A lane
            if exp_a[lane] is not None:
                assert va == 1, f"cycle {cycle} A lane {lane}: expected valid"
                da, _, fa = unpack_a(dut.out_a[lane].value, data_width)
                assert mask(da, data_width) == mask(exp_a[lane], data_width), \
                    f"cycle {cycle} A lane {lane}: dut={da} exp={exp_a[lane]}"
                assert fa == (1 if exp_first else 0), \
                    f"cycle {cycle} A lane {lane}: first dut={fa} exp={int(exp_first)}"
            else:
                assert va == 0, f"cycle {cycle} A lane {lane}: expected invalid"

            # B lane (no first field)
            if exp_b[lane] is not None:
                assert vb == 1, f"cycle {cycle} B lane {lane}: expected valid"
                db, _ = unpack_b(dut.out_b[lane].value, data_width)
                assert mask(db, data_width) == mask(exp_b[lane], data_width), \
                    f"cycle {cycle} B lane {lane}: dut={db} exp={exp_b[lane]}"
            else:
                assert vb == 0, f"cycle {cycle} B lane {lane}: expected invalid"

        # all valid A lanes must agree on first
        valid_firsts = [_first_a(dut, lane) for lane in range(array_dim)
                        if _valid_a(dut, lane) == 1]
        assert len(set(valid_firsts)) <= 1, \
            f"cycle {cycle}: valid lanes disagree on first: {valid_firsts}"

        # done must coincide with the last valid payload (k==K-1), and be low
        # on every other payload cycle.
        if model.is_last_payload():
            assert done == 1, f"cycle {cycle}: expected done on last payload"
            done_seen += 1
        else:
            assert done == 0, f"cycle {cycle}: done on non-last payload cycle"

    assert done_seen == 1, f"done asserted on {done_seen} payload cycles, expected 1"

# --------------------------------------------------------------------------- #
# tests
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_full_size(dut):
    """M=N=K=ARRAY_DIM — baseline (no invalid lanes; proves latency + done only)."""
    data_width, array_dim = get_params()
    await start_clock(dut)
    await reset_dut(dut)
    cocotb.start_soon(stub_buffer(dut, array_dim))
    await run_feed(dut, data_width, array_dim, array_dim, array_dim, array_dim)

@cocotb.test()
async def test_sub_n(dut):
    """M=N=K=3 on a larger grid — exercises lane<M / lane<N masking + X lanes."""
    data_width, array_dim = get_params()
    if array_dim < 4:
        return
    await start_clock(dut)
    await reset_dut(dut)
    cocotb.start_soon(stub_buffer(dut, array_dim))
    await run_feed(dut, data_width, array_dim, 3, 3, 3)

@cocotb.test()
async def test_non_square(dut):
    """M != N — A and B mask differently."""
    data_width, array_dim = get_params()
    if array_dim < 4:
        return
    await start_clock(dut)
    await reset_dut(dut)
    cocotb.start_soon(stub_buffer(dut, array_dim))
    await run_feed(dut, data_width, array_dim, 2, 4, 3)

@cocotb.test()
async def test_k1(dut):
    """K=1 — last true on the first issued cycle; done/edge stress."""
    data_width, array_dim = get_params()
    await start_clock(dut)
    await reset_dut(dut)
    cocotb.start_soon(stub_buffer(dut, array_dim))
    await run_feed(dut, data_width, array_dim, array_dim, array_dim, 1)

@cocotb.test()
async def test_back_to_back(dut):
    """Two feeds, different dims — config re-latches, no leakage."""
    data_width, array_dim = get_params()
    await start_clock(dut)
    await reset_dut(dut)
    cocotb.start_soon(stub_buffer(dut, array_dim))
    await run_feed(dut, data_width, array_dim, array_dim, array_dim, array_dim)
    await RisingEdge(dut.clk)   # let busy drop before restarting
    await run_feed(dut, data_width, array_dim, 2, 3, 2)