# `collector` — drain collector contract (v1)

Status: **implemented (v1.1)** in `rtl/collector.sv`, with D1–D4 as recommended, verified per §10. v1.1 replaced `wr_col` (= tile_n·n) with `wr_tile_n` (`memory_contract.md` D3): the result memory addresses C words as `{row, tile_n}`, and recovering `tile_n` from `wr_col` would need a divide by n. Model, testbench, assertions, sweep and mutation campaign were all redone for v1.1.

**`tile_ctrl` v2 (overlapped tiles, `fsm_contract.md` v2):** no change to `rtl/collector.sv`. Its four tile inputs are now driven from `tile_ctrl`'s `cap_tile_m`, `cap_tile_n`, `cap_M`, `cap_N` (the tile being captured) instead of the feed-side outputs, and captures come ≥ n + 1 cycles apart, so every input stays inside §6's contract. §3.1, §5 and §6 below are updated for v2.

Timing is derived from the RTL as of commit `7406e0b` (`pe_grid`, `tile_ctrl`)
and from `docs/fsm_contract.md` §6.1. It was cross-checked against
`TileCtrlModel` + `FeederModel`. No existing module changes, `tile_ctrl` included.

---

## 1. Scope

After each output tile, `tile_ctrl` pulses `capture` (every PE copies its
accumulator into its shadow register) and then `shift_en` for n − 1 cycles
(the shadow chain shifts down; `pe_grid.drain_out` shows one grid row per
cycle, bottom row first). Today nothing consumes `drain_out`. The integration
test does it with a Python stand-in.

`collector` turns that drain stream into **row writes of the result matrix C**:

- it keeps only the M_t × N_t region of each tile (rows ≥ M_t and columns ≥ N_t
  hold stale accumulators from earlier tiles, see `fsm_contract.md` §7);
- it places each kept row at its global position in C, using the tile
  identifiers `tile_ctrl` already exports;
- it passes accumulator data through untouched (no arithmetic on data).

It does **not** own the result memory (a write port leaves the block; the
memory is outside it, as a Python model in v1), and it does not talk to the host.

### What it deliberately does not own

| Concern | Owner |
|---|---|
| When to capture, how many shifts | `tile_ctrl` (back sequencer: `capture`, then n − 1 `shift_en`) |
| The shadow registers, the shift chain, `drain_out` | `pe_grid` / `pe` |
| Which tile is being captured, its dims M_t × N_t | `tile_ctrl` (`cap_tile_m`, `cap_tile_n`, `cap_M`, `cap_N`) |
| Storing C | result memory (external) |

This answers `fsm_contract.md` **Q3**: the FSM needs **no** extra drain-row
signal. Everything the collector needs is already on `capture`, `shift_en` and
the tile outputs, provided it latches the tile outputs at `capture` (§5.2).

## 2. Parameters, clocking, reset

| Name | Source | Default | Meaning |
|---|---|---|---|
| `ARRAY_DIM` (n) | `pe_pkg` | — | Grid size, ≥ 2 (same rule as `tile_ctrl`; elaboration `$error`) |
| `ACC_WIDTH` | `pe_pkg` | 32 | Accumulator / result width |
| `DIM_W` | `pe_pkg` | `$clog2(n+1)` | Width of M_t, N_t |
| `MAX_M`, `MAX_N` | module parameters | 64 | **Must equal `tile_ctrl`'s** (they set the tile-index widths); ≥ ARRAY_DIM (elaboration `$error`) |
| `TM_W`, `TN_W` | localparam | same formulas as `tile_ctrl` | Tile-index widths |
| `ROW_W` | localparam | `max(1, $clog2(MAX_M))` | Global row index of C, 0 … MAX_M − 1 |
| `R_W` | localparam | `$clog2(n)` | Row-within-tile counter, 0 … n − 1 |

Single clock `clk`. `rst_n` is asynchronous active-low, matching `tile_ctrl`
and `pe`. Reset state: idle, every output 0.

## 3. Port list

### 3.1 From `tile_ctrl`

| Port | Dir | Width | Driven by | Contract |
|---|---|---|---|---|
| `capture` | in | 1 | `tile_ctrl.capture` (same net as `pe_grid.capture`) | Starts a tile's drain; latches the four tile inputs below |
| `shift_en` | in | 1 | `tile_ctrl.shift_en` (same net as `pe_grid.shift_en`) | Each sampled `shift_en` brings the next row up to `drain_out` |
| `tile_m` | in | `TM_W` | `tile_ctrl.cap_tile_m` (v1: `tile_m`) | Sampled **only** with `capture` |
| `tile_n` | in | `TN_W` | `tile_ctrl.cap_tile_n` (v1: `tile_n`) | Sampled only with `capture` |
| `tile_rows` | in | `DIM_W` | `tile_ctrl.cap_M` (M_t; v1: `feed_M`) | Sampled only with `capture` |
| `tile_cols` | in | `DIM_W` | `tile_ctrl.cap_N` (N_t; v1: `feed_N`) | Sampled only with `capture` |

### 3.2 From `pe_grid`

| Port | Dir | Width | Driven by | Contract |
|---|---|---|---|---|
| `drain_out` | in | `ACC_WIDTH` × n | `pe_grid.drain_out` | Bottom row of the shadow chain, combinational in `pe_grid` |

### 3.3 Result write port (to the result memory)

| Port | Dir | Width | Contract |
|---|---|---|---|
| `wr_en` | out | 1 | One row of C is written at the end of this cycle |
| `wr_row` | out | `ROW_W` | Global row of C: `tile_m·n + r` |
| `wr_tile_n` | out | `TN_W` | Tile column: lane j is global column `wr_tile_n·n + j` (v1.1; v1 had `wr_col = tile_n·n`) |
| `wr_lane_en` | out | n | Lane j is written iff `j < N_t` (a thermometer code) |
| `wr_data` | out | signed `ACC_WIDTH` × n | `wr_data[j]` → `C[wr_row][wr_tile_n·n + j]`; = `drain_out[j]` |

The memory is assumed always ready and writes on the rising edge that ends a
`wr_en` cycle. Lanes with `wr_lane_en[j] = 0` must not be written. Their data
is don't-care (in practice stale accumulator bits). All write-port outputs are
0 when `wr_en` is 0, so idle cycles are fully deterministic for the lockstep
compare.

`pe_grid.drain_out` is declared unsigned while `pe_grid.out` is signed. The bits
are identical; the collector declares `wr_data` signed (§9).

## 4. Behaviour

Registers: `active`, `row` (`R_W` bits, the grid row now on `drain_out`),
`fresh` (that row has not been offered for writing yet), and the latched
`tm`, `tn`, `rows`, `cols`.

On each rising edge, in priority order:

1. `capture` sampled → `active ← 1`, `row ← n − 1`, `fresh ← 1`, latch
   `tile_m`, `tile_n`, `tile_rows`, `tile_cols`.
2. else, if `active` and `shift_en` sampled and `row ≠ 0` → `row ← row − 1`, `fresh ← 1`.
3. else, if `active` and `fresh` and `row = 0` → `active ← 0` (row 0 has just been offered).
4. else → `fresh ← 0`.

Outputs (combinational from those registers and `drain_out`):

```
wr_en      = active && fresh && (row < rows)
wr_row     = wr_en ? tm·n + row : 0
wr_tile_n  = wr_en ? tn         : 0
wr_lane_en = wr_en ? (1 << cols) − 1 : 0
wr_data[j] = wr_en ? drain_out[j] : 0
```

The multiply by n is by a constant (adds and shifts in synthesis).

Two choices in this are worth your review (§8, D1–D4). The headline one: the
row counter **advances on `shift_en`**, not on a free-running count from
`capture`. It follows the signal that actually moves the data, so it stays
correct if a later version stalls or overlaps the drain. Under v1 timing the
two are cycle-identical.

## 5. Cycle-accurate timing

### 5.1 One tile

Using `fsm_contract.md` §6.1's tile-relative τ, with CAPTURE at
τ_c = K + M_t + N_t + 1. `pe_grid` and `collector` both sample `capture` on the
edge into τ_c + 1:

| τ | `tile_ctrl` | `drain_out` shows | collector `row` | `wr_en` |
|---|---|---|---|---|
| τ_c | `capture` | (previous tile's shadow) | — | 0 |
| τ_c + 1 … τ_c + n − 1 | `shift_en` | grid row n − 1 − (τ − τ_c − 1) | same | iff row < M_t |
| τ_c + n | DONE if last tile (v1: next ISSUE or DONE) | grid row 0 | 0 | 1 |
| τ_c + n + 1 | | | idle | 0 |

So each tile produces **exactly M_t writes**, bottom-up (row M_t − 1 first), in
the **last M_t cycles** of the drain window, τ_c + n − M_t + 1 … τ_c + n.
The rows ≥ M_t that come out first are dropped.

Worked example (n = 4, K = 3, M_t = 2, N_t = 3, tile (1, 0); `tile_ctrl` τ_c = 9):

```
τ          9    10    11    12    13   | 14
tile_ctrl CAP   DRN   DRN   DRN   ISS/DONE
drain_out  -    r3    r2    r1    r0
wr_en      .    .     .     1     1
wr_row               (skip)(skip) 5     4      (tile_m·4 + r)
wr_tile_n                         0     0
wr_lane_en                        0111  0111   (N_t = 3)
```

### 5.2 Why the tile inputs must be latched at `capture`

Under v1, row 0 is on `drain_out` at τ_c + n, which is the **next tile's ISSUE cycle**,
and `tile_ctrl` has already advanced `tile_m`, `tile_n`, `feed_M` and `feed_N` on
the edge into it. Under v2 the inputs come from `cap_*`, which move on the next
tile's handoff edge, and that can fall inside this tile's drain (`fsm_contract.md`
§6.2 chart: `cap_tile` moves at t = 14 and 20 while rows are still draining). So the
latch is needed under both versions. The v1 finding: Checked against `TileCtrlModel` for M = 6, N = 5, K = 3,
n = 4: at every non-final row-0 cycle the tile outputs already name the next
tile, e.g. capture of tile (0,0) 4×4 → row 0 cycle shows tile (0,1) 4×1. Reading
them live would put row 0 of every tile in the wrong place with the wrong
lane mask.

### 5.3 A whole job

- Writes per job = Σ_tiles M_t = T_n · M (each tile row of C is written once per tile column).
- Every element C[i][j], i < M, j < N, is written **exactly once**. Nothing outside M × N is written.
- The last write of a job happens in `tile_ctrl`'s **DONE cycle** (the last tile's row-0 cycle, τ_c + n = L). It is committed by the edge that ends the DONE cycle. So **`tile_ctrl.done` already means "all of C is written by the end of this cycle"**, and no extra completion signal is needed (§8, D3).
- A tile's drain always ends before the next `capture`: v2 keeps captures ≥ n + 1 cycles apart (`fsm_contract.md` §6.2 term c, V3), so the next capture comes at least one cycle after the row-0 cycle. (v1 was serialized: ≥ K + M_t' + N_t' + 2 cycles.)

## 6. Illegal / out-of-contract inputs

None of these can happen with `tile_ctrl` (v1 or v2). Each is defined anyway, so
the RTL has no undefined behaviour, and each is flagged by an assertion.

| Situation | Behaviour | Why |
|---|---|---|
| `capture` while a drain is active | Restart: latch the new tile, `row ← n − 1` | Mirrors `pe_grid`: the same `capture` overwrites every shadow register, so the old tile's remaining rows are gone |
| `capture` and `shift_en` in the same cycle | `capture` wins | Mirrors `pe`, where `capture` has priority over `shift_en` |
| `shift_en` while idle, or after row 0 | Ignored | No row to advance |
| `rst_n` low | Asynchronously idle, every output 0 | |

**Input assumption (from `tile_ctrl`, not re-checked by the RTL):** at every
`capture`, 1 ≤ `tile_rows`, `tile_cols` ≤ n, `tile_m·n + tile_rows ≤ MAX_M` and
`tile_n·n + tile_cols ≤ MAX_N`. These are `tile_ctrl`'s §9.7–8 properties.
They are what keep `wr_row` inside `ROW_W` and every written column inside `MAX_N`. The cocotb
testbench only drives captures that satisfy them; Phase 5 adds an assertion.

## 7. Properties (spec for the monitors and SVA)

1. `wr_en` only while `active`; never in reset; never without a preceding `capture`.
2. Per `capture`: exactly `rows` writes, with `wr_row` = tm·n + rows − 1, …, tm·n + 0 in that order, on consecutive drain cycles.
3. Every write: `wr_tile_n = tn`, `wr_lane_en = (1 << cols) − 1`, `wr_data = drain_out`, and `wr_row < MAX_M`, `wr_tile_n·n + cols ≤ MAX_N`.
4. Write-port outputs are all 0 whenever `wr_en` is 0.
5. The latched tile values change only on `capture`.
6. (System) Each C element is written exactly once per job, nothing outside M × N is written, and C equals A @ B.
7. (System) The job's last write is in the cycle `tile_ctrl.done` is high.

## 8. Design decisions for review

| | Decision | Recommended | Alternative and trade-off |
|---|---|---|---|
| D1 | Write-port shape | **Row write**: global `wr_row`, tile column `wr_tile_n` (v1.1), n-lane `wr_data` + `wr_lane_en` | Emit `tile_m`/`tile_n`/row-in-tile and let the memory compute the address. That pushes arithmetic into the memory, against the README's "memory is kept dumb" |
| D2 | What advances the row | **`shift_en`** (+ a `fresh` flag so each row is offered once) | Free-running count of n cycles from `capture`. Same v1 timing, one fewer input, but silently wrong as soon as the drain stalls or overlaps |
| D3 | Write port registered? | **No**: outputs are simple logic on registers (`drain_out` comes straight from the shadow flops). Zero added latency, and `tile_ctrl.done` stays the job-complete signal | Register the write port (+1 cycle). Cleaner for timing closure, but then the last write lands after `done`, and the top level needs a "results committed" signal |
| D4 | `capture` during a drain | **Restart** (mirror the grid) | Ignore the new capture. That would write the new tile's shadow data under the old tile's coordinates |

## 9. Known simplifications

1. **No backpressure on the write port**: the result memory must accept one full row per cycle. This ties into the README's valid/ready item.
2. **The write port is n lanes wide** (one grid row per cycle), so the memory needs an n-lane row write with per-lane enables.
3. **Rows arrive bottom-up within a tile**, and tiles arrive in `tile_ctrl`'s row-major order; C is not written in address order.
4. **`drain_out` is reinterpreted as signed.** `pe_grid` declares it unsigned; the bits are the same.
5. **Masked lanes carry stale data** on `wr_data`; only `wr_lane_en` protects them.
6. **`MAX_M` / `MAX_N` must be set identically on `tile_ctrl` and `collector`.** Nothing checks this inside either module; the system top should pass one value to both.
7. **No error output.** Out-of-contract inputs (§6) are defined and asserted, but not reported on a port.
8. **The out-of-contract-input assertions are off in the standalone testbench.** It drives those inputs on purpose, so `tb/collector/Makefile` defines `COLLECTOR_TB_OUT_OF_CONTRACT`. The system build (`tb/matmul_top/`) keeps them live, which also checks that `tile_ctrl` never produces them.
9. **The RTL assertions do not cover reset.** They are disabled while reset is asserted (via a checker-only `sva_armed` flop), so async-reset behaviour is checked by the cocotb `WritePortMonitor` and `Env.reset`.

## 10. Verification plan (same flow as `tile_ctrl`)

| Phase | Deliverable |
|---|---|
| 2 | `tb/collector/collector_model.py`: `CollectorModel`, cycle-steppable like `TileCtrlModel`. `test_collector_model.py`: pytest, including a closed loop of `TileCtrlModel` + `FeederModel` + a PEModel grid + `CollectorModel` + a result-memory model that must reproduce A @ B with each element written exactly once |
| 3 | Stub `rtl/collector.sv` + `tb/collector/test_collector.py`: lockstep against the model, monitors for §7.1–5, coverage (tile shapes, edge tiles, out-of-contract cases), shown failing first |
| 4 | Real `rtl/collector.sv`, green |
| 5 | Lint, SVA, a mutation campaign. The system test (then `tb/tile_ctrl_integration/`, now `tb/matmul_top/`) gets the RTL collector plus a Python result memory in place of the Python collector, and checks §7.6–7 |

## 11. Open questions

- **C1.** *Resolved* by `docs/top_contract.md` (T5): the host sees `STATUS.DONE` (sticky OR live), never `tile_ctrl.done` directly; polling makes the first C read come after the last C write by construction.
- **C2.** When overlapped tiles arrive (`fsm_contract.md` Q1), the next `capture` may come while the collector is still draining. D2 keeps the collector correct only if the shadow drain completes first, which the FSM contract already requires (next capture ≥ n cycles after the previous one). Confirm that requirement survives the overlap design.
- **C3.** Should the result memory be part of the repo (an RTL dummy memory, README future work) or stay a Python model until the top module?
