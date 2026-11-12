# Operand buffer + result memory contract (v1)

Status: **implemented (v1)**: `rtl/lane_ram.sv` (×3 in `tb/tile_ctrl_integration/`), D1–D6 as recommended, collector v1.1 (D3). Verified per §9.

Derived from the RTL as of commit `1702f7c` (`feeder`, `tile_ctrl`,
`collector`) and from the Python stand-ins in `tb/tile_ctrl_integration/`,
which these memories replace.

---

## 1. Scope

The system test still has two Python stand-ins:

- an **operand buffer** that answers the feeder's read port: 1-cycle registered
  read, lane i of A = `A[tile_m·n + i][k]`, lane j of B = `B[k][tile_n·n + j]`;
- a **result memory** that applies the collector's row writes.

This contract replaces both with RTL. Following the README ("memory is kept
dumb"), the memories do **no arithmetic**. Every address is a **concatenation
of indices that already exist on a port**, so all the address logic is wiring.
That works only because the data are stored in a **tile-word layout** (§3):
one memory word holds the n lanes that move together in one cycle.

### What it deliberately does not own

| Concern | Owner |
|---|---|
| k loop, `rd_en`, `rd_col_a` / `rd_row_b` (= k) | `read_sequencer` |
| Which tile is in flight (`tile_m`, `tile_n`) | `tile_ctrl` |
| Which C row, which tile column, which lanes to write | `collector` |
| Putting A, B into the tile-word layout; reading C back | the host (Python in v1; the top module later, §10 M1) |

## 2. Decisions for review

| | Decision | Recommended | Alternatives and trade-off |
|---|---|---|---|
| **D1** | How A is stored | **Transposed, in tile words**: word (tile_m, k) holds `A[tile_m·n + i][k]` for i = 0 … n − 1 | (a) Row-major A: the n lanes come from n different rows, so it needs n separate banks, one per lane. (b) A transposed but packed with a job-dependent stride (`k·T_m + tile_m`): denser, but needs a multiplier. |
| **D2** | Word address | **Concatenation**: `{tile, k}` for A and B, `{row, tile_n}` for C | Exact packing (`tile·MAX_K + k`): no wasted words, but a constant multiply. Concatenation wastes the gap between `TILES_M` and the next power of two (13 → 16 tile slots by default) |
| **D3** | What the collector sends as the column | **Change `collector`: replace `wr_col` (= tile_n·n) with `wr_tile_n` (= tile_n)**. Collector contract v1.1 | (a) Keep `wr_col` and let the memory divide by n: not a shift unless n is a power of two (the default is 5). (b) Keep `wr_col` and add `wr_tile_n`: a redundant port that nothing in the system consumes. (c) Take `tile_n` from `tile_ctrl`: wrong, because `tile_ctrl` has moved on before row 0 is written (`collector_contract.md` §5.2) |
| **D4** | One RAM module, or one per role | **One generic `lane_ram`**, instantiated three times (A, B, C). The layout lives in the address wiring | Purpose-built `operand_mem` / `result_mem` with tile ports. More names, the same logic, three things to verify instead of one |
| **D5** | Read-during-write to the same word | **Read-first** (returns the old word) | Write-first needs a bypass mux, and true dual-port block RAMs often leave a cross-port collision undefined |
| **D6** | Reset | **None**: contents and the read register are not reset (block-RAM style) | Resetting the read register costs a reset net on the read data path, and nothing needs it: the feeder ignores data on invalid lanes, and the host only reads words it waited for |

D3 is the only decision that touches existing, verified RTL. It is a
one-port change, but it needs the collector's model, testbench, assertions and
mutation runs redone. That is planned in §9.

## 3. Layout

n = `ARRAY_DIM`. The tile indices and widths are the same as in `tile_ctrl`
(`TM_W`, `TN_W`). `KI_W = max(1, $clog2(MAX_K))` holds k = 0 … MAX_K − 1.

| Memory | Word address | Lane l of the word holds | Lane width | Address width | Words (default n = 5, MAX_M = MAX_N = 64, MAX_K = 1024) |
|---|---|---|---|---|---|
| A | `{tile_m, k}` | `A[tile_m·n + l][k]` | `DATA_WIDTH` | `TM_W + KI_W` | 2^14 = 16 384 |
| B | `{tile_n, k}` | `B[k][tile_n·n + l]` | `DATA_WIDTH` | `TN_W + KI_W` | 2^14 = 16 384 |
| C | `{row, tile_n}` | `C[row][tile_n·n + l]` | `ACC_WIDTH` | `ROW_W + TN_W` | 2^10 = 1 024 |

- **The tile index is the high field of the A and B address**, so each tile's K
  words are contiguous. A loader (or a future DMA) streams one tile at a time.
- **Padding:**
  - Lanes past the edge of the matrix (`tile_m·n + l ≥ M`, or `tile_n·n + l ≥ N`) are
    ordinary lanes of a word. Every operand read stays in bounds without
    clamping. Whatever the host leaves there is ignored, because
    `read_sequencer` drives `valid = 0` on those lanes.
  - C words for lanes ≥ N_t are never written: the collector's `wr_lane_en` masks them.
- With default parameters A and B are 80 KiB each and C is 20 KiB. That's fine
  for simulation; a synthesis target would set `MAX_K`, `MAX_M` and `MAX_N` to fit its RAM.

## 4. `lane_ram` ports

```
module lane_ram #(parameter int LANES, parameter int WIDTH, parameter int ADDR_W)
```

| Port | Dir | Width | Contract |
|---|---|---|---|
| `clk` | in | 1 | |
| `we` | in | 1 | Write on this edge |
| `waddr` | in | `ADDR_W` | |
| `wlane_en` | in | `LANES` | Lane l is written iff set; the other lanes keep their old value |
| `wdata` | in | `WIDTH` × `LANES` | |
| `re` | in | 1 | Read on this edge |
| `raddr` | in | `ADDR_W` | |
| `rdata` | out | `WIDTH` × `LANES` | The word at `raddr` as of **before** this edge (read-first, D5). Registered: valid the cycle after `re`, and held while `re` is low |

There is no reset and no error output. `rdata` before the first read, and any word
never written, are don't-care (D6).

## 5. Wiring (all of the address logic)

| Instance | Write port | Read port |
|---|---|---|
| `u_mem_a` (LANES = n, WIDTH = DATA_WIDTH) | host | `re = feeder.rd_en`, `raddr = {tile_ctrl.tile_m, feeder.rd_col_a[KI_W-1:0]}`, `rdata → feeder.rd_a` |
| `u_mem_b` (LANES = n, WIDTH = DATA_WIDTH) | host | `re = feeder.rd_en`, `raddr = {tile_ctrl.tile_n, feeder.rd_row_b[KI_W-1:0]}`, `rdata → feeder.rd_b` |
| `u_mem_c` (LANES = n, WIDTH = ACC_WIDTH) | `we = collector.wr_en`, `waddr = {collector.wr_row, collector.wr_tile_n}`, `wlane_en = collector.wr_lane_en`, `wdata = collector.wr_data` | host |

Truncating `rd_col_a` to `KI_W` bits is safe: `read_sequencer` only issues
k < K ≤ MAX_K. Reading `tile_m`/`tile_n` live from `tile_ctrl` is also safe for
operands: they are stable from ISSUE through DRAIN (`fsm_contract.md` §4.5),
which covers every read of the tile.

Signedness: `rd_a`/`rd_b` are signed and `wr_data` is signed, while `lane_ram`
stores raw bits. Each connection is the same width, so the bits pass through unchanged.

## 6. Timing

**Operand reads.** These match the stand-in exactly. `read_sequencer` presents
`rd_en` with k after edge e; `lane_ram` samples it on edge e + 1; the data is on
`rd_a`/`rd_b` after edge e + 1, which is when `read_sequencer` expects it (its
model's FILL cycle).

**Result writes.** The collector shows a row write after edge e, and `lane_ram`
commits it on edge e + 1. The job's last write is in `tile_ctrl`'s DONE cycle
(post-edge L, `collector_contract.md` §5.3), so it is committed on edge L + 1.

**Reading C back. New finding, a rule for the host.** With read-first RAM
(D5), a host read sampled on edge L + 1, i.e. issued in the DONE cycle itself,
races the last write to the same edge and returns the **old** word for the
tile's row 0. So: **the host may issue C reads from cycle L + 1 on**, one cycle
after it sees `done`. A host that registers `done` before acting meets this
automatically. Only one word can be stale: the job's last row write.

## 7. Properties

`lane_ram` alone:

1. A read returns the word as last written before the edge it is sampled on, lane by lane.
2. A write with `wlane_en` lane l clear leaves lane l unchanged.
3. A read and a write to the same address on the same edge return the old word (read-first).
4. `rdata` changes only on an edge with `re` high.

System (the system test, replacing both stand-ins):

5. After `done`, reading C words `{row, t}` for row < M, t < ⌈N/n⌉ gives `A @ B` on lanes with `t·n + l < N`.
6. The feeder never reads an address outside the words the host loaded for this job's tiles (k < K, tile < T_m / T_n).
7. C words for rows ≥ M or tile columns ≥ ⌈N/n⌉ are never written; lanes ≥ N_t of the edge tile column are never written.
8. A read issued in the DONE cycle of a word written in that cycle returns the old word (D5 behaviour, documented, directed test).

The §6 host rule is easy to break. The first version of the system test's own
host issued its first C read in the DONE cycle and read row 0 of a single-tile
job back as stale. The host now registers `done` before reading, and
`test_read_in_done_cycle_returns_old_word` pins the behaviour down.

## 8. Known simplifications

1. **The host transposes A** into tile words before loading (D1). That's free in Python; in hardware it would be the loader's job.
2. **Power-of-two address padding** (D2) wastes memory when `TILES_M`, `TILES_N` or `MAX_K` aren't powers of two.
3. **No reset and no ECC/parity** (D6).
4. **No back-pressure.** The RAMs accept a read and a write every cycle, as the feeder and collector assume (no valid/ready yet).
5. **The host interface is minimal and test-only**: raw `we/waddr/wdata` and `re/raddr/rdata` per memory, driven by Python. A real host protocol is the top module's job (M1).
6. **One job's operands at a time.** No double buffering, so the host must not write A or B while a job runs. This is a precondition, not checked.
7. **`lane_ram`'s assertions arm from an `initial` block.** There is no reset, so a simulation-only flag (`sva_armed`, inside `ifndef SYNTHESIS`) is set by a plain `always` after the first edge. `always_ff` forbids the second driver, and the flag never reaches synthesis.

## 9. Verification plan (same flow as `tile_ctrl` and `collector`)

| Phase | Deliverable |
|---|---|
| 2 | `tb/lane_ram/lane_ram_model.py` (`LaneRamModel`) with pytest. Plus the layout helpers `pack_a_tiles`, `pack_b_tiles`, `unpack_c` (Python, reused by the system test), validated standalone: pack → system model → unpack == A @ B, using the existing closed-loop FSM + feeder + PE-grid + collector models |
| 2b | **Collector v1.1 (D3):** model, contract and testbench switch `wr_col` → `wr_tile_n`. Red against the current collector RTL, then green after the port change. Rerun its sweep and mutation campaign |
| 3 | Stub `rtl/lane_ram.sv` + `tb/lane_ram/test_lane_ram.py`: lockstep against the model, monitors for §7.1–4, coverage (lane masks, collisions, `re` held low, address extremes), shown failing first |
| 4 | Real `rtl/lane_ram.sv`, green, lint clean |
| 5 | SVA, a mutation campaign, and the system test (`tb/tile_ctrl_integration/`) with three `lane_ram` instances replacing both Python stand-ins. The host loads tile words and reads C after `done`, checking §7.5–8 |

## 10. Open questions

- **M1.** The host interface for the top module: separate raw RAM ports (as here), one shared bus, or an AXI-lite-style register/memory map? Decide with the top module.
- **M2.** Should A and B share one RAM with a select bit, halving the instances? That costs read bandwidth: two words are needed every cycle.
- **M3.** Is exact packing (D2 alternative) worth a constant multiplier once a real RAM size matters?
- **M4.** Double buffering (README future work) would add a bank-select bit to the A and B addresses. The concatenation scheme extends naturally (`{bank, tile, k}`).
