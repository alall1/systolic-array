"""
Standalone pytest for HostIfModel and the host address map (no RTL):
    pytest test_host_if_model.py

  1. register block, sticky status, start pulse, decode, read path, unmapped
     space: one test per rule of docs/top_contract.md §4-8
  2. host_map element addresses agree with the tile-word layout
  3. the closed loop through the bus only: HostIfModel + TileCtrlModel +
     FeederModel + PE grid (tile_ctrl's Harness) + CollectorModel + three
     LaneRamModels. A Python host that only issues bus operations (one per
     cycle) loads A/B, starts, polls STATUS and reads C, which must equal numpy
     A @ B. Also: illegal dims, start while busy, A/B writes while busy, and
     start-to-DONE-seen latency == L + 3 (§6).
"""

import os
import random

import numpy as np
import pytest

from host_map import (HostMap, REGION_A, REGION_B, REGION_C, REG_CTRL, REG_STATUS,
                      REG_DIM_M, REG_DIM_N, REG_DIM_K, CTRL_START, STATUS_BUSY,
                      STATUS_DONE, STATUS_CMD_ERROR, STATUS_HOST_ERROR)
from host_if_model import HostIfModel
from tile_layout import operand_addr, c_addr, ceil_div
from lane_ram_model import LaneRamModel
from collector_model import CollectorModel
from tile_ctrl_model import job_latency
from test_tile_ctrl_model import Harness
from utils import to_signed

SEED = int(os.environ.get("SEED", "20261112"))
MAX_M = MAX_N = 64
MAX_K = 1024
DATA_WIDTH, ACC_WIDTH = 8, 32


def hmap(n=5):
    return HostMap(n, MAX_M, MAX_N, MAX_K, DATA_WIDTH, ACC_WIDTH)

# --------------------------------------------------------------------------- #
# 1. host_if rules
# --------------------------------------------------------------------------- #

def test_reset():
    h = HostIfModel(hmap())
    o = h.reset()
    assert (o["start"], o["M"], o["N"], o["K"]) == (False, 0, 0, 0)
    assert h.step(re=True, addr=h.m.reg(REG_STATUS))["host_rdata"] == 0


def test_dim_registers_and_read_first():
    m = hmap()
    h = HostIfModel(m)
    h.step(we=True, addr=m.reg(REG_DIM_M), wdata=0xFFFF_FF06)          # masked to M_W bits
    assert h.step(re=True, addr=m.reg(REG_DIM_M))["host_rdata"] == 0xFFFF_FF06 & ((1 << m.m_w) - 1)
    o = h.step(we=True, re=True, addr=m.reg(REG_DIM_K), wdata=9)       # same edge: read returns old
    assert o["host_rdata"] == 0 and o["K"] == 9
    assert h.step(re=True, addr=m.reg(REG_DIM_K))["host_rdata"] == 9
    assert h.step()["host_rdata"] is None                              # no read: don't-care


def test_ctrl_start_is_one_cycle_pulse():
    m = hmap()
    h = HostIfModel(m)
    assert h.step(we=True, addr=m.reg(REG_CTRL), wdata=CTRL_START)["start"]
    assert not h.step()["start"]
    assert not h.step(we=True, addr=m.reg(REG_CTRL), wdata=0)["start"]   # bit 0 clear: no start
    assert h.step(re=True, addr=m.reg(REG_CTRL))["host_rdata"] == 0     # CTRL reads 0


def test_status_semantics():
    m = hmap()
    h = HostIfModel(m)
    rd = lambda **kw: h.step(re=True, addr=m.reg(REG_STATUS), **kw)["host_rdata"]
    start = lambda **kw: h.step(we=True, addr=m.reg(REG_CTRL), wdata=CTRL_START, **kw)
    assert rd(done=True, error=True) == STATUS_DONE | STATUS_CMD_ERROR      # live: the done cycle itself
    assert rd() == STATUS_DONE | STATUS_CMD_ERROR                          # ...and sticky afterwards
    assert rd(busy=True) & STATUS_BUSY                                      # BUSY is live
    assert not start(busy=True)["start"], "START while busy is ignored"
    assert rd() & STATUS_DONE, "...and clears nothing"
    assert start(done=True)["start"], "START taken while not busy, even in a done cycle"
    assert not start()["start"], "a second START in the next cycle (one pending) is ignored"
    h.reset()
    h.step(done=True)
    start(done=True)
    assert rd() == STATUS_BUSY, "pending start reads BUSY; sticky bits cleared (clear beats the same-edge done)"
    assert rd(busy=True) == STATUS_BUSY
    h.step(we=True, addr=m.addr(REGION_A, 1, 0), busy=True)                # A write while busy
    assert rd(done=True) == STATUS_DONE | STATUS_HOST_ERROR
    assert rd() == STATUS_DONE | STATUS_HOST_ERROR


def test_ab_write_decode_and_busy_gating():
    m = hmap()
    h = HostIfModel(m)
    for region, pre in ((REGION_A, "a"), (REGION_B, "b")):
        for lane in range(m.n):
            d = h.decode(we=True, addr=m.addr(region, 77, lane), wdata=0x1F3)
            assert d[f"{pre}_we"] and d[f"{pre}_waddr"] == 77 and d[f"{pre}_wlane_en"] == 1 << lane
            assert d[f"{pre}_wdata"][lane] == 0xF3
        d = h.decode(we=True, addr=m.addr(region, 5, 1), busy=True)
        assert not d[f"{pre}_we"], "A/B write while busy is dropped"
        h.step(we=True, addr=m.addr(region, 5, 1), busy=True)
        assert h.step(re=True, addr=m.reg(REG_STATUS))["host_rdata"] & STATUS_HOST_ERROR
        h.reset()
        h.step(we=True, addr=m.reg(REG_CTRL), wdata=CTRL_START)              # a pending start counts as busy
        assert not h.decode(we=True, addr=m.addr(region, 5, 1))[f"{pre}_we"]
        h.reset()


def test_c_read_path():
    m = hmap()
    h = HostIfModel(m)
    word = c_addr(17, 3, m.tn_w)
    data = [10, 20, 30, 40, 50]
    for lane in range(m.n):
        d = h.decode(re=True, addr=m.addr(REGION_C, word, lane))
        assert d["c_re"] and d["c_raddr"] == word
        assert h.step(re=True, addr=m.addr(REGION_C, word, lane), c_rdata=data)["host_rdata"] == data[lane]
    assert not h.decode(we=True, addr=m.addr(REGION_C, word, 0))["c_re"], "C is not host-writable"


def test_unmapped_space():
    m = hmap()                                          # n = 5, LANE_W = 3: lanes 5..7 unmapped
    h = HostIfModel(m)
    unmapped = [m.addr(0, 0, 1), m.addr(0, 5, 0), m.addr(0, (1 << m.word_w) - 1, 0),
                m.addr(REGION_A, 3, 5), m.addr(REGION_B, 3, 7), m.addr(REGION_C, 1 << m.c_aw, 0)]
    for a in unmapped:
        d = h.decode(we=True, re=True, addr=a, wdata=0xFF)
        assert not (d["a_we"] or d["b_we"] or d["c_re"]), hex(a)
        assert h.step(re=True, addr=a)["host_rdata"] == 0, hex(a)
    for a in (m.addr(REGION_A, 3, 0), m.addr(REGION_B, 3, 0)):
        assert h.step(re=True, addr=a)["host_rdata"] == 0, "A/B are not host-readable"
    assert h.step(re=True, addr=m.reg(REG_STATUS))["host_rdata"] == 0, "unmapped writes set no error"

# --------------------------------------------------------------------------- #
# 2. address map
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("n", [2, 3, 5, 8])
def test_element_addresses_follow_tile_layout(n):
    m = HostMap(n, MAX_M, MAX_N, MAX_K, DATA_WIDTH, ACC_WIDTH)
    for r in range(0, MAX_M, 7):
        for k in range(0, 40, 9):
            assert m.fields(m.a_elem(r, k)) == (REGION_A, operand_addr(r // n, k, m.ki_w), r % n)
            assert m.fields(m.b_elem(k, r)) == (REGION_B, operand_addr(r // n, k, m.ki_w), r % n)
            assert m.fields(m.c_elem(r, k % MAX_N)) == (REGION_C, c_addr(r, (k % MAX_N) // n, m.tn_w), (k % MAX_N) % n)
    assert (m.word_w, m.addr_w) == ((14, 19) if n == 5 else (m.word_w, m.addr_w))

# --------------------------------------------------------------------------- #
# 3. closed loop through the bus only
# --------------------------------------------------------------------------- #

class BusHarness(Harness):
    """tile_ctrl's Harness with the operand buffer served by the A / B RAM models."""
    def __init__(self, n, ram_a, ram_b, ki_w):
        super().__init__(n, with_grid=True, max_m=MAX_M, max_n=MAX_N, max_k=MAX_K)
        self.ram_a, self.ram_b, self.ki_w = ram_a, ram_b, ki_w
        self.C = np.zeros((MAX_M, MAX_N), dtype=object)     # Harness's own drain stand-in (unused)

    def _buf_a(self, k, lane):
        return to_signed(self.ram_a.peek(operand_addr(self._tile[0], k, self.ki_w))[lane], DATA_WIDTH)

    def _buf_b(self, k, lane):
        return to_signed(self.ram_b.peek(operand_addr(self._tile[1], k, self.ki_w))[lane], DATA_WIDTH)


class BusSystem:
    """Every block model, edge by edge, each fed the others' PRE-edge outputs."""
    def __init__(self, n):
        m = self.m = HostMap(n, MAX_M, MAX_N, MAX_K, DATA_WIDTH, ACC_WIDTH)
        self.hi = HostIfModel(m)
        self.ram_a = LaneRamModel(n, DATA_WIDTH, m.a_aw)
        self.ram_b = LaneRamModel(n, DATA_WIDTH, m.b_aw)
        self.ram_c = LaneRamModel(n, ACC_WIDTH, m.c_aw)
        self.h = BusHarness(n, self.ram_a, self.ram_b, m.ki_w)
        self.col = CollectorModel(n, MAX_M, MAX_N, ACC_WIDTH)
        self.hi_out = self.hi.outputs()
        self.w = self.col.outputs([0] * n)
        self.cycle = 0
        self.done_cycles = []               # absolute cycles in which tile_ctrl.done is high

    def cycle_op(self, we=False, re=False, addr=0, wdata=0):
        """One edge of the whole system with one host bus operation; returns host_rdata."""
        o, hi_out, w = self.h.o, self.hi_out, self.w
        d = self.hi.decode(we, re, addr, wdata, busy=o["busy"])
        self.ram_a.step(d["a_we"], d["a_waddr"], d["a_wlane_en"], d["a_wdata"])
        self.ram_b.step(d["b_we"], d["b_waddr"], d["b_wlane_en"], d["b_wdata"])
        c_rdata = self.ram_c.step(w["wr_en"], c_addr(w["wr_row"], w["wr_tile_n"], self.m.tn_w),
                                  w["wr_lane_en"], w["wr_data"], d["c_re"], d["c_raddr"])
        self.hi_out = self.hi.step(we, re, addr, wdata, busy=o["busy"], done=o["done"],
                                   error=o["error"], c_rdata=[v or 0 for v in c_rdata])
        self.h.tick(start=hi_out["start"], cfg=(hi_out["M"], hi_out["N"], hi_out["K"]))
        self.w = self.col.step(o["capture"], o["shift_en"], o["cap_tile_m"], o["cap_tile_n"],
                               o["cap_M"], o["cap_N"], drain_out=self.h.grid.drain_out())
        self.cycle += 1
        if self.h.o["done"]:
            self.done_cycles.append(self.cycle)
        return self.hi_out["host_rdata"]

    # host software: bus operations only -----------------------------------
    def write(self, addr, value):
        self.cycle_op(we=True, addr=addr, wdata=value & ((1 << self.m.host_w) - 1))

    def read(self, addr):
        return self.cycle_op(re=True, addr=addr)

    def load(self, A, B):
        m = self.m
        for r in range(len(A)):
            for k in range(len(A[0])):
                self.write(m.a_elem(r, k), A[r][k])
        for k in range(len(B)):
            for c in range(len(B[0])):
                self.write(m.b_elem(k, c), B[k][c])

    def start(self, M, N, K):
        m = self.m
        self.write(m.reg(REG_DIM_M), M)
        self.write(m.reg(REG_DIM_N), N)
        self.write(m.reg(REG_DIM_K), K)
        self.write(m.reg(REG_CTRL), CTRL_START)
        return self.cycle                   # cycle count after the START edge

    def poll(self, limit):
        """Poll from the cycle right after START: every read shows BUSY or DONE
        (never an idle-looking gap), until DONE."""
        for _ in range(limit):
            s = self.read(self.m.reg(REG_STATUS))
            assert s & (STATUS_BUSY | STATUS_DONE), f"STATUS {s:#x} looks idle while a job is outstanding"
            if s & STATUS_DONE:
                return s
        raise AssertionError("no DONE")

    def read_c(self, M, N):
        return [[to_signed(self.read(self.m.c_elem(r, c)), ACC_WIDTH) for c in range(N)] for r in range(M)]

    def run(self, A, B):
        M, K, N = len(A), len(A[0]), len(B[0])
        self.load(A, B)
        t_start = self.start(M, N, K)
        s = self.poll(job_latency(M, N, K, self.m.n) + 10)
        seen = self.cycle
        return s, t_start, seen, self.read_c(M, N)


def rand_mat(rng, r, c):
    return [[rng.randint(-8, 7) for _ in range(c)] for _ in range(r)]


@pytest.mark.parametrize("n", [2, 3, 5])
def test_matmul_through_the_bus(n):
    rng = random.Random(SEED + n)
    print(f"SEED={SEED + n}")
    s = BusSystem(n)
    shapes = [(n, n, n), (1, 1, 1), (2 * n + 1, max(1, n - 1), 3), (1, 3 * n, 2), (3 * n, 1, 1)]
    shapes += [(rng.randint(1, 3 * n + 1), rng.randint(1, 3 * n + 1), rng.randint(1, 6)) for _ in range(8)]
    for (M, N, K) in shapes:
        A, B = rand_mat(rng, M, K), rand_mat(rng, K, N)
        status, t_start, seen, C = s.run(A, B)
        assert status == STATUS_DONE, f"status {status:#x}"
        assert np.array_equal(np.array(C), np.array(A) @ np.array(B)), f"M={M} N={N} K={K}"
        # §6: done is high in job cycle L = absolute cycle t_start + 1 + L; polling sees it at L + 2
        L = job_latency(M, N, K, n)
        assert s.done_cycles[-1] == t_start + 1 + L
        assert seen - t_start == L + 2, f"start-to-DONE-seen {seen - t_start}, contract L+2 = {L + 2}"


def test_illegal_dims_set_cmd_error():
    s = BusSystem(4)
    s.start(0, 3, 3)
    assert s.poll(10) == STATUS_DONE | STATUS_CMD_ERROR
    s.start(MAX_M + 1, 3, 3)
    assert s.poll(10) == STATUS_DONE | STATUS_CMD_ERROR


def test_start_while_busy_and_writes_while_busy():
    """A second START and A/B writes during a job are ignored; HOST_ERROR is set,
    and the job's result is exactly what the loaded operands give."""
    n, rng = 4, random.Random(SEED + 50)
    s = BusSystem(n)
    M, N, K = 2 * n + 1, n + 2, 4
    A, B = rand_mat(rng, M, K), rand_mat(rng, K, N)
    s.load(A, B)
    s.start(M, N, K)
    for _ in range(5):
        s.write(s.m.reg(REG_CTRL), CTRL_START)                          # ignored (busy)
        s.write(s.m.a_elem(0, 0), 99)                                   # ignored (busy)
        s.write(s.m.b_elem(0, 0), -99)
    status = s.poll(job_latency(M, N, K, n) + 10)
    assert status == STATUS_DONE | STATUS_HOST_ERROR
    assert np.array_equal(np.array(s.read_c(M, N)), np.array(A) @ np.array(B))
    s.start(M, N, K)                                                    # taken: sticky bits clear
    assert s.read(s.m.reg(REG_STATUS)) == STATUS_BUSY
