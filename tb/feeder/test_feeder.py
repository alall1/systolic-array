"""
cocotb testbench for "feeder" module (read_sequencer + skew_buffer)

Lockstep: every cycle the SAME stimulus drives the DUT and FeederModel, then
every feeder output is compared after the edge. Stimulus, model step and check
happen in one coroutine (Env.tick) so there is no ordering race between them.

Optional env vars: SEED (default 12345), MAX_K (default 1024)
"""

import os
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from utils import unpack_a, unpack_b
from feeder_model import FeederModel

CLK_PERIOD_NS   = 10
NUM_RANDOM_JOBS = 40
MAX_TEST_K      = 12
MEM_DEPTH       = 64

# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #

def get_params():
    data_width = int(os.environ["DATA_WIDTH"])
    array_dim  = int(os.environ["ARRAY_DIM"])
    max_k      = int(os.environ.get("MAX_K", "1024"))
    return data_width, array_dim, max_k

def mask(v, data_width):
    """Reduce to raw data_width bits so signed/unsigned interpretation can't
    cause a false mismatch."""
    return v & ((1 << data_width) - 1)

async def start_clock(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())

async def reset_dut(dut, array_dim, cycles: int = 2):
    """Assert active-low reset for `cycles`, then release off-edge."""
    dut.rst_n.value = 0
    dut.start.value = 0
    dut.M.value     = 0
    dut.N.value     = 0
    dut.K.value     = 0
    for lane in range(array_dim):
        dut.rd_a[lane].value = 0
        dut.rd_b[lane].value = 0
    for _ in range(cycles):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.rst_n.value = 1

# --------------------------------------------------------------------------- #
# X-safe field accessors (invalid lanes carry 'x in data/first; valid never may)
#   a_payload_t [data : valid : first] -> first = bit 0, valid = bit 1
#   b_payload_t [data : valid]         -> valid = bit 0
# --------------------------------------------------------------------------- #

def _bit(binstr, i):
    """Bit i (LSB=0) of a cocotb binstr as 0/1, or None if x/z."""
    ch = binstr[-(i + 1)]
    return int(ch) if ch in "01" else None

def _valid_a(dut, lane):
    return _bit(str(dut.out_a[lane].value), 1)

def _valid_b(dut, lane):
    return _bit(str(dut.out_b[lane].value), 0)

# --------------------------------------------------------------------------- #
# operand memory: random content over ALL lanes (including lanes >= M / >= N,
# so lane masking is actually tested) and over MEM_DEPTH k-rows.
# --------------------------------------------------------------------------- #

class OperandMemory:
    def __init__(self, rng, depth, array_dim, data_width):
        self.rng = rng
        self.depth = depth
        self.n = array_dim
        self.w = data_width
        self.randomize()

    def randomize(self):
        lo, hi = -(1 << (self.w - 1)), (1 << (self.w - 1)) - 1
        self.A = [[self.rng.randint(lo, hi) for _ in range(self.n)] for _ in range(self.depth)]
        self.B = [[self.rng.randint(lo, hi) for _ in range(self.n)] for _ in range(self.depth)]

    def a(self, k, lane):
        return self.A[k][lane]

    def b(self, k, lane):
        return self.B[k][lane]

# --------------------------------------------------------------------------- #
# stub buffer: REAL 1-cycle registered read.
#   address presented post-edge e  ->  data driven post-edge e+1
# Data is only ever changed 1ns after an edge, so the DUT never sees it change
# mid-cycle. When nothing was requested, the read port is driven with junk to
# prove the sequencer's valid gating ignores it.
# --------------------------------------------------------------------------- #

async def stub_buffer(env):
    dut, n, w = env.dut, env.array_dim, env.data_width
    junk_rng = random.Random(env.seed + 1)   # separate stream: stimulus stays reproducible
    captured = None
    while True:
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")

        # drive data for the address captured one cycle ago
        if captured is not None:
            col, row = captured
            assert col < env.mem.depth and row < env.mem.depth, \
                f"read address out of range: col={col} row={row}"
            for lane in range(n):
                dut.rd_a[lane].value = mask(env.mem.A[col][lane], w)
                dut.rd_b[lane].value = mask(env.mem.B[row][lane], w)
        else:
            for lane in range(n):
                dut.rd_a[lane].value = junk_rng.getrandbits(w)
                dut.rd_b[lane].value = junk_rng.getrandbits(w)

        # capture the address presented this cycle
        if int(dut.rd_en.value):
            captured = (int(dut.rd_col_a.value), int(dut.rd_row_b.value))
        else:
            captured = None

# --------------------------------------------------------------------------- #
# environment: driver + model + scoreboard in one lockstep coroutine
# --------------------------------------------------------------------------- #

class Env:
    def __init__(self, dut, rng, seed, data_width, array_dim, max_k):
        self.dut = dut
        self.rng = rng
        self.seed = seed
        self.data_width = data_width
        self.array_dim = array_dim
        self.dim_w = array_dim.bit_length()   # == clog2(ARRAY_DIM + 1)
        self.k_w = max_k.bit_length()         # == clog2(MAX_K + 1)
        self.mem = OperandMemory(rng, MEM_DEPTH, array_dim, data_width)
        self.model = FeederModel(array_dim, self.mem.a, self.mem.b)
        self.cycle = 0
        self.reset_counts()

    def reset_counts(self):
        self.n_a_valid = 0
        self.n_b_valid = 0
        self.n_done = 0

    def junk_cfg(self):
        return (self.rng.randrange(0, 1 << self.dim_w),
                self.rng.randrange(0, 1 << self.dim_w),
                self.rng.randrange(0, 1 << self.k_w))

    async def tick(self, start=False, cfg=None):
        """Drive inputs for the coming edge, wait for it, step the model with the
        same inputs, compare all outputs. When start is low, M/N/K are driven
        with junk: the sequencer must only look at them on the start edge."""
        dut = self.dut
        if cfg is None:
            assert not start, "start requires an explicit cfg"
            cfg = self.junk_cfg()
        M, N, K = cfg
        dut.start.value = 1 if start else 0
        dut.M.value = M
        dut.N.value = N
        dut.K.value = K

        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")

        exp = self.model.step(start=bool(start), M=M, N=N, K=K)
        self.cycle += 1
        self.check(exp)

    async def reset(self, cycles=2):
        """Mid-run reset: model.reset() snapshot must match the DUT while reset
        is still asserted (>=1 edge so the sync-reset skew buffer has cleared)."""
        dut = self.dut
        dut.rst_n.value = 0
        dut.start.value = 0
        for _ in range(cycles):
            await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        self.check(self.model.reset())
        dut.rst_n.value = 1

    def check(self, exp):
        dut, n, w = self.dut, self.array_dim, self.data_width
        tag = f"cycle {self.cycle}"

        busy  = int(dut.busy.value)
        done  = int(dut.done.value)
        rd_en = int(dut.rd_en.value)
        assert busy  == int(exp["busy"]),  f"{tag}: busy dut={busy} exp={int(exp['busy'])}"
        assert done  == int(exp["done"]),  f"{tag}: done dut={done} exp={int(exp['done'])}"
        assert rd_en == int(exp["rd_en"]), f"{tag}: rd_en dut={rd_en} exp={int(exp['rd_en'])}"
        if done:
            self.n_done += 1

        if exp["rd_en"]:
            col = int(dut.rd_col_a.value)
            row = int(dut.rd_row_b.value)
            assert col == exp["rd_col_a"], f"{tag}: rd_col_a dut={col} exp={exp['rd_col_a']}"
            assert row == exp["rd_row_b"], f"{tag}: rd_row_b dut={row} exp={exp['rd_row_b']}"

        for lane in range(n):
            # ---- A lane: (data, first) when valid
            va = _valid_a(dut, lane)
            assert va is not None, f"{tag}: out_a[{lane}].valid is X/Z"
            e_a = exp["out_a"][lane]
            assert va == (1 if e_a is not None else 0), \
                f"{tag} A lane {lane}: valid dut={va} exp={int(e_a is not None)}"
            if e_a is not None:
                da, _, fa = unpack_a(dut.out_a[lane].value, w)
                assert mask(da, w) == mask(e_a[0], w), \
                    f"{tag} A lane {lane}: data dut={da} exp={e_a[0]}"
                assert fa == (1 if e_a[1] else 0), \
                    f"{tag} A lane {lane}: first dut={fa} exp={int(e_a[1])}"
                self.n_a_valid += 1

            # ---- B lane: (data,) when valid (no first field)
            vb = _valid_b(dut, lane)
            assert vb is not None, f"{tag}: out_b[{lane}].valid is X/Z"
            e_b = exp["out_b"][lane]
            assert vb == (1 if e_b is not None else 0), \
                f"{tag} B lane {lane}: valid dut={vb} exp={int(e_b is not None)}"
            if e_b is not None:
                db, _ = unpack_b(dut.out_b[lane].value, w)
                assert mask(db, w) == mask(e_b[0], w), \
                    f"{tag} B lane {lane}: data dut={db} exp={e_b[0]}"
                self.n_b_valid += 1

    def drain_cycles(self, K):
        # last payload (k=K-1) is captured at edge K+1 and leaves lane i at K+1+i
        return K + 2 * self.array_dim + 4

    async def run_job(self, M, N, K):
        """One job, then drain until every payload has left the skew buffer.
        Beyond the per-cycle lockstep compare, assert the DUT delivered exactly
        M*K A-beats, N*K B-beats and one done (guards against vacuous passes)."""
        assert K <= MEM_DEPTH
        self.reset_counts()
        await self.tick(start=True, cfg=(M, N, K))
        for _ in range(self.drain_cycles(K)):
            await self.tick()
        assert self.n_a_valid == M * K, f"A valid beats {self.n_a_valid}, expected {M * K}"
        assert self.n_b_valid == N * K, f"B valid beats {self.n_b_valid}, expected {N * K}"
        assert self.n_done == 1, f"done pulses {self.n_done}, expected 1"


async def make_env(dut):
    data_width, array_dim, max_k = get_params()
    seed = int(os.environ.get("SEED", "12345"))
    dut._log.info(f"SEED={seed} DATA_WIDTH={data_width} ARRAY_DIM={array_dim} MAX_K={max_k}")
    await start_clock(dut)
    await reset_dut(dut, array_dim)
    env = Env(dut, random.Random(seed), seed, data_width, array_dim, max_k)
    cocotb.start_soon(stub_buffer(env))
    return env

# --------------------------------------------------------------------------- #
# tests
# --------------------------------------------------------------------------- #

@cocotb.test()
async def test_full_size(dut):
    """M=N=K=ARRAY_DIM, then K > ARRAY_DIM: baseline latency, skew and done."""
    env = await make_env(dut)
    n = env.array_dim
    await env.run_job(n, n, n)
    await env.run_job(n, n, min(n + 3, MAX_TEST_K))

@cocotb.test()
async def test_sub_array(dut):
    """M=N=ARRAY_DIM-1: lane masking on both operands."""
    env = await make_env(dut)
    n = env.array_dim
    s = max(1, n - 1)
    await env.run_job(s, s, 3)

@cocotb.test()
async def test_non_square(dut):
    """M != N in both directions: A and B mask differently."""
    env = await make_env(dut)
    n = env.array_dim
    h = max(1, n // 2)
    await env.run_job(h, n, 3)
    await env.run_job(n, h, 3)

@cocotb.test()
async def test_k1(dut):
    """K=1: last payload is also the first; done/first/busy edge stress."""
    env = await make_env(dut)
    n = env.array_dim
    await env.run_job(n, n, 1)
    await env.run_job(1, 1, 1)

@cocotb.test()
async def test_single_pe(dut):
    """M=N=1: only lane 0 of each operand is ever valid."""
    env = await make_env(dut)
    await env.run_job(1, 1, 5)

@cocotb.test()
async def test_back_to_back_min_gap(dut):
    """Second start at the earliest accepted edge (K+1), different dims:
    config re-latches, no leakage between jobs in the skew buffer."""
    env = await make_env(dut)
    n = env.array_dim
    M1, N1, K1 = n, n, 4
    M2, N2, K2 = max(1, n - 1), max(1, n // 2), 3
    env.reset_counts()
    await env.tick(start=True, cfg=(M1, N1, K1))          # edge 0
    for _ in range(K1):                                   # edges 1..K1
        await env.tick()
    await env.tick(start=True, cfg=(M2, N2, K2))          # edge K1+1
    for _ in range(env.drain_cycles(K2) + n):
        await env.tick()
    assert env.n_a_valid == M1 * K1 + M2 * K2
    assert env.n_b_valid == N1 * K1 + N2 * K2
    assert env.n_done == 2

@cocotb.test()
async def test_start_while_busy(dut):
    """start pulsed on every edge 1..K+1 of a running job with a DIFFERENT config.
    Edges 1..K (busy high) must be ignored; edge K+1 is accepted."""
    env = await make_env(dut)
    n = env.array_dim
    M1, N1, K1 = n, n, 4
    M2, N2, K2 = max(1, n - 1), max(1, n - 1), K1 + 1
    for e in range(1, K1 + 2):
        env.reset_counts()
        await env.tick(start=True, cfg=(M1, N1, K1))      # edge 0
        for _ in range(e - 1):                            # edges 1..e-1
            await env.tick()
        await env.tick(start=True, cfg=(M2, N2, K2))      # edge e
        for _ in range(K1 + K2 + 2 * n + 6):
            await env.tick()
        accepted = (e == K1 + 1)
        assert env.n_a_valid == M1 * K1 + (M2 * K2 if accepted else 0), f"e={e}"
        assert env.n_b_valid == N1 * K1 + (N2 * K2 if accepted else 0), f"e={e}"
        assert env.n_done == (2 if accepted else 1), f"e={e}"

@cocotb.test()
async def test_reset_mid_feed(dut):
    """Reset while payloads are in flight in the skew buffer: everything drops
    to idle, nothing stale leaks out, and a following job is clean."""
    env = await make_env(dut)
    n = env.array_dim
    await env.tick(start=True, cfg=(n, n, 8))
    for _ in range(4):
        await env.tick()
    await env.reset()
    for _ in range(n + 3):          # must stay idle: model expects no valid lanes
        await env.tick()
    await env.run_job(n, n, 3)

@cocotb.test()
async def test_random_jobs(dut):
    """Random content, dims, K and idle gaps. Re-run with SEED=<n> to explore."""
    env = await make_env(dut)
    n = env.array_dim
    for _ in range(NUM_RANDOM_JOBS):
        env.mem.randomize()         # safe: DUT is idle and drained here
        M = env.rng.randint(1, n)
        N = env.rng.randint(1, n)
        K = env.rng.randint(1, MAX_TEST_K)
        for _ in range(env.rng.randint(0, 3)):
            await env.tick()
        await env.run_job(M, N, K)