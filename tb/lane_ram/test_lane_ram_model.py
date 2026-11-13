"""
Standalone pytest for LaneRamModel and the tile-word layout (no RTL):
    pytest test_lane_ram_model.py

  1. LaneRamModel port behaviour (memory_contract.md §4, §7.1-4), including a
     random sequence against a naive reference written separately here
  2. tile_layout helpers: every lane of every word holds what §3 says
  3. the closed loop with the real layout: TileCtrlModel + FeederModel + PE grid
     (tile_ctrl's Harness) reading A and B from tile-word memories addressed only
     by {tile, k}, CollectorModel v1.1 writing C into a LaneRamModel at
     {row, tile_n}, and C read back through the read port from the cycle after
     done (§6), equal to numpy A @ B. Includes the §6 read-in-done-cycle race.
"""

import os
import random

import numpy as np
import pytest

from lane_ram_model import LaneRamModel
from tile_layout import (k_index_width, tile_index_width, operand_addr, c_addr,
                         pack_a_tiles, pack_b_tiles, c_words, unpack_c, ceil_div)
from collector_model import CollectorModel
from utils import to_signed
from test_tile_ctrl_model import Harness

SEED = int(os.environ.get("SEED", "20261112"))

# --------------------------------------------------------------------------- #
# 1. LaneRamModel
# --------------------------------------------------------------------------- #

def test_unknown_until_written_and_read():
    m = LaneRamModel(4, 8, 6)
    assert m.step() == [None] * 4, "rdata before any read is don't-care"
    assert m.step(re=True, raddr=5) == [None] * 4, "word never written is don't-care"


def test_one_cycle_read_and_hold():
    m = LaneRamModel(3, 8, 4)
    m.step(we=True, waddr=2, wlane_en=0b111, wdata=[1, 2, 3])
    assert m.step(re=True, raddr=2) == [1, 2, 3]
    m.step(we=True, waddr=2, wlane_en=0b111, wdata=[7, 8, 9])
    assert m.step() == [1, 2, 3], "rdata holds while re is low, even if the word changes"
    assert m.step(re=True, raddr=2) == [7, 8, 9]


def test_lane_enables():
    m = LaneRamModel(4, 8, 4)
    m.step(we=True, waddr=1, wlane_en=0b1111, wdata=[1, 2, 3, 4])
    m.step(we=True, waddr=1, wlane_en=0b0101, wdata=[9, 9, 9, 9])
    assert m.step(re=True, raddr=1) == [9, 2, 9, 4]
    m.step(we=True, waddr=3, wlane_en=0b0010, wdata=[5, 6, 7, 8])
    assert m.step(re=True, raddr=3) == [None, 6, None, None]


def test_read_first_collision():
    m = LaneRamModel(2, 8, 3)
    m.step(we=True, waddr=4, wlane_en=0b11, wdata=[1, 1])
    assert m.step(we=True, waddr=4, wlane_en=0b11, wdata=[2, 2], re=True, raddr=4) == [1, 1]
    assert m.step(re=True, raddr=4) == [2, 2]


def test_address_and_data_masking():
    m = LaneRamModel(2, 4, 3)
    m.step(we=True, waddr=8 + 1, wlane_en=0b11, wdata=[0x1F, -1])      # addr 9 -> 1, data -> 4 bits
    assert m.step(re=True, raddr=1) == [0xF, 0xF]


def test_random_against_naive_reference():
    rng = random.Random(SEED)
    lanes, width, aw = 5, 8, 4
    m = LaneRamModel(lanes, width, aw)
    ref = [[None] * lanes for _ in range(1 << aw)]
    rdata = [None] * lanes
    for _ in range(5000):
        we, re = rng.random() < 0.5, rng.random() < 0.5
        wa, ra = rng.randrange(1 << aw), rng.randrange(1 << aw)
        en = rng.randrange(1 << lanes)
        wd = [rng.randrange(1 << width) for _ in range(lanes)]
        got = m.step(we, wa, en, wd, re, ra)
        if re:
            rdata = list(ref[ra])
        if we:
            for l in range(lanes):
                if en >> l & 1:
                    ref[wa][l] = wd[l]
        assert got == rdata

# --------------------------------------------------------------------------- #
# 2. layout helpers
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize("n", [2, 3, 5])
def test_pack_layout(n):
    rng = random.Random(SEED + n)
    ki_w = k_index_width(16)
    for _ in range(10):
        M, N, K = rng.randint(1, 3 * n + 1), rng.randint(1, 3 * n + 1), rng.randint(1, 16)
        A = [[rng.randint(-8, 7) for _ in range(K)] for _ in range(M)]
        B = [[rng.randint(-8, 7) for _ in range(N)] for _ in range(K)]
        ma = pack_a_tiles(A, n, ki_w, pad=lambda l: 1000 + l)
        mb = pack_b_tiles(B, n, ki_w, pad=lambda l: 2000 + l)
        assert len(ma) == ceil_div(M, n) * K and len(mb) == ceil_div(N, n) * K
        for tm in range(ceil_div(M, n)):
            for k in range(K):
                for l in range(n):
                    r = tm * n + l
                    assert ma[operand_addr(tm, k, ki_w)][l] == (A[r][k] if r < M else 1000 + l)
        for tn in range(ceil_div(N, n)):
            for k in range(K):
                for l in range(n):
                    c = tn * n + l
                    assert mb[operand_addr(tn, k, ki_w)][l] == (B[k][c] if c < N else 2000 + l)


def test_concatenated_widths_match_contract():
    """docs/memory_contract.md §3 table, default parameters (n=5, 64/64/1024)."""
    n, ki_w = 5, k_index_width(1024)
    tm_w, tn_w, row_w = tile_index_width(64, n), tile_index_width(64, n), (64 - 1).bit_length()
    assert (tm_w + ki_w, tn_w + ki_w, row_w + tn_w) == (14, 14, 10)

# --------------------------------------------------------------------------- #
# 3. closed loop with the real layout
# --------------------------------------------------------------------------- #

MAX_M = MAX_N = 64
MAX_K = 1024
ACC_WIDTH = 32


class MemHarness(Harness):
    """tile_ctrl's closed-loop Harness, operand buffer replaced by tile-word
    memories addressed only by {tile, k} (memory_contract.md §5)."""
    def __init__(self, n):
        super().__init__(n, with_grid=True, max_m=MAX_M, max_n=MAX_N, max_k=MAX_K)
        self.ki_w = k_index_width(MAX_K)
        self.mem_a = {}
        self.mem_b = {}
        self.reads_a = set()
        self.reads_b = set()

    def _buf_a(self, k, lane):
        addr = operand_addr(self._tile[0], k, self.ki_w)
        assert addr in self.mem_a, f"A read of a word the host did not load: {addr:#x}"
        self.reads_a.add(addr)
        return self.mem_a[addr][lane]

    def _buf_b(self, k, lane):
        addr = operand_addr(self._tile[1], k, self.ki_w)
        assert addr in self.mem_b, f"B read of a word the host did not load: {addr:#x}"
        self.reads_b.add(addr)
        return self.mem_b[addr][lane]


class MemSystem:
    def __init__(self, n):
        self.n = n
        self.h = MemHarness(n)
        self.col = CollectorModel(n, MAX_M, MAX_N, ACC_WIDTH)
        self.tn_w = tile_index_width(MAX_N, n)
        self.ram_c = LaneRamModel(n, ACC_WIDTH, (MAX_M - 1).bit_length() + self.tn_w)
        self.w = self.col.outputs([0] * n)          # write port as visible now

    def _c_edge(self, re=False, raddr=0):
        """One edge of the C RAM with the write port as visible before it."""
        w = self.w
        return self.ram_c.step(we=w["wr_en"], waddr=c_addr(w["wr_row"], w["wr_tile_n"], self.tn_w),
                               wlane_en=w["wr_lane_en"], wdata=w["wr_data"], re=re, raddr=raddr)

    def _idle_edge(self, re=False, raddr=0):
        """An edge after done: the whole system keeps clocking, so the collector
        is stepped too (idle controls), not frozen in its last state."""
        r = self._c_edge(re, raddr)
        self.w = self.col.step(drain_out=[0] * self.n)
        return r

    def run_job(self, A, B, rng):
        n, h = self.n, self.h
        M, K, N = len(A), len(A[0]), len(B[0])
        junk = lambda l: rng.randint(-128, 127)
        h.mem_a = pack_a_tiles(A, n, h.ki_w, pad=junk)
        h.mem_b = pack_b_tiles(B, n, h.ki_w, pad=junk)
        h.reads_a, h.reads_b = set(), set()
        h.C = np.zeros((M, N), dtype=object)        # Harness's own Python drain stand-in (unused here)
        writes = set()
        o = h.o
        h.tick(start=True, cfg=(M, N, K))
        while True:
            self._c_edge()                          # commits the write visible before this edge
            self.w = self.col.step(o["capture"], o["shift_en"], o["cap_tile_m"], o["cap_tile_n"],
                                   o["cap_M"], o["cap_N"], drain_out=h.grid.drain_out())
            if self.w["wr_en"]:
                a = c_addr(self.w["wr_row"], self.w["wr_tile_n"], self.tn_w)
                assert a in c_words(M, N, n, self.tn_w), "write outside C"
                assert self.w["wr_tile_n"] * n + bin(self.w["wr_lane_en"]).count("1") <= N, "padding lane written"
                writes.add(a)
            if h.trace[-1]["done"]:
                break
            o = h.trace[-1]
            h.tick()
        last = c_addr(self.w["wr_row"], self.w["wr_tile_n"], self.tn_w)
        assert self.w["wr_en"], "the job's last write is in the done cycle (collector_contract.md §5.3)"
        old = self.ram_c.peek(last)
        # §6: a read issued IN the done cycle races the last write to the same edge -> old word
        assert self._idle_edge(re=True, raddr=last) == old
        assert not self.w["wr_en"], "collector idle after the done cycle"
        # host reads from the cycle after done: one word per cycle, 1-cycle latency;
        # the RAM holds raw bits, the host reinterprets them as signed accumulators
        def read_word(addr):
            self._idle_edge(re=True, raddr=addr)
            return [None if v is None else to_signed(v, ACC_WIDTH) for v in self._idle_edge()]
        C = unpack_c(read_word, M, N, n, self.tn_w)
        assert writes == set(c_words(M, N, n, self.tn_w)), "every C word written"
        assert h.reads_a == set(operand_addr(t, k, h.ki_w) for t in range(ceil_div(M, n)) for k in range(K))
        assert h.reads_b == set(operand_addr(t, k, h.ki_w) for t in range(ceil_div(N, n)) for k in range(K))
        return C


@pytest.mark.parametrize("n", [2, 3, 4, 5])
def test_tiled_matmul_through_tile_word_memories(n):
    rng = random.Random(SEED + 100 + n)
    print(f"SEED={SEED + 100 + n}")
    s = MemSystem(n)
    shapes = [(n, n, n), (1, 1, 1), (2 * n + 1, max(1, n - 1), 3), (1, 3 * n, 2), (3 * n, 1, 1)]
    shapes += [(rng.randint(1, 3 * n + 1), rng.randint(1, 3 * n + 1), rng.randint(1, 6)) for _ in range(10)]
    for (M, N, K) in shapes:
        A = [[rng.randint(-8, 7) for _ in range(K)] for _ in range(M)]
        B = [[rng.randint(-8, 7) for _ in range(N)] for _ in range(K)]
        C = s.run_job(A, B, rng)
        assert np.array_equal(np.array(C, dtype=np.int64), np.array(A) @ np.array(B)), f"M={M} N={N} K={K}"
