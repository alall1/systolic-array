# Top module + host interface contract (v1)

Status: **implemented (v1)**: `rtl/host_if.sv` (unit-verified in `tb/host_if/`) and `rtl/matmul_top.sv` (system-verified through the bus in `tb/matmul_top/`). T1–T7 as recommended, with the Phase 2 STATUS refinement (§5, §6). T7 done: the old harness `tb/tile_ctrl_integration/` has been removed; `tb/matmul_top/` carries all of its checks. Unchanged by `tile_ctrl` v2 apart from L (`fsm_contract.md` §6.2) and the collector's tile inputs, which `matmul_top` now takes from `cap_*` (`fsm_contract.md` §4.5); the system test passes at n = 2, 4, 5 with the new L + 2 bus latency.

Derived from the RTL as of commit `9c07093` and from
`tb/tile_ctrl_integration/tile_ctrl_integration_top.sv` (since removed, T7), which already wired
the whole datapath. This contract settles `memory_contract.md` **M1** (the host
interface) and `collector_contract.md` **C1** (what the host sees as "done").

---

## 1. Scope

Today the system has no host-facing interface. The integration harness exposes
raw ports per memory plus `tile_ctrl`'s command pins, and Python drives them.
v1 adds:

- **`host_if`**: the only new logic. It holds the address decode, a five-entry
  register block, sticky status bits, the start pulse, and the read-data mux.
- **`matmul_top`**: pure wiring. `host_if` + `tile_ctrl` + `feeder` + `pe_grid`
  + `collector` + three `lane_ram`, connected exactly as the integration
  harness does today (`memory_contract.md` §5).

A host then runs a matmul using nothing but one bus:

1. write the A and B elements;
2. write `DIM_M`, `DIM_N`, `DIM_K`, then `CTRL.START`;
3. poll `STATUS` until `DONE`;
4. read the C elements.

### What it deliberately does not own

| Concern | Owner |
|---|---|
| Tiling, the job schedule, rejecting illegal dims | `tile_ctrl` |
| Operand streaming, skew, MACs, drain | `feeder`, `pe_grid` |
| Where each C row goes | `collector` |
| Tile-word layout and address concatenation | `memory_contract.md` §3/§5. The host addresses words and lanes in that layout (§4) |

## 2. Decisions for review

| | Decision | Recommended | Alternatives and trade-off |
|---|---|---|---|
| **T1** | Bus protocol | **Synchronous SRAM-style**: `host_we`, `host_re`, `host_addr`, `host_wdata`, `host_rdata`. Always ready, read data exactly 1 cycle later. | (a) AXI4-Lite: the industry register-bus standard, but five handshaked channels to verify. Better as a later thin adapter around this bus. (b) Keep raw per-memory ports: no new logic, but it isn't an interface a host could use. |
| **T2** | Bus data width | **One element per transfer**, `HOST_W = ACC_WIDTH` (32). A/B writes use the low `DATA_WIDTH` bits and a **one-hot lane enable** (`lane_ram` already has per-lane enables, so no read-modify-write). C reads pick one lane through a registered lane index. | A word-wide bus (n × ACC_WIDTH = 160 bits by default): n× fewer transfers, but very wide ports, and the width differs between A/B and C. |
| **T3** | Address map | **Concatenation `{region, word, lane}`**, so `host_if` does no arithmetic. The tile-word layout is visible to the host: host software computes (tile, k, lane) from (row, col), which costs nothing in software. | Element-addressed row-major map (`A_BASE + r·K + k`): friendlier, but needs a divider and multiplier to reach the tile-word layout, against the README's "memory is kept dumb". |
| **T4** | Where the new logic lives | **A separate `host_if` module**, unit-verified like every other block. `matmul_top` stays pure wiring and is checked by system tests. | Put the decode and registers inline in `matmul_top`. Fewer files, but the only new logic would then be verifiable only through the whole system. |
| **T5** | How the host learns "done" | **`STATUS.DONE` / `CMD_ERROR` read as sticky OR live** (set by `tile_ctrl.done` / `error`, cleared when a `START` write is taken). `BUSY` = job running OR start pending. Polling then satisfies the memory contract's read-after-done rule **by construction** (§6). | Expose `tile_ctrl.done` as a pin or interrupt. It's a 1-cycle pulse, so a host would have to catch it, and the §6 hazard returns. |
| **T6** | A/B writes while a job runs | **Ignored, and they set a sticky `STATUS.HOST_ERROR`**. This turns `memory_contract.md` simplification 6 (unchecked precondition) into enforced behaviour. | Leave it as an unchecked precondition: a stray write silently corrupts the running job. |
| **T7** | Test harness | **`tb/matmul_top/` supersedes `tb/tile_ctrl_integration/`**: the same system checks, driven through the bus only. The old harness is deleted once every one of its checks is ported. **Deleting committed files needs your OK at that point.** | Keep both: two system benches to maintain, the old one driving pins that no longer exist on the real top. |

## 3. Parameters and ports

### 3.1 Parameters

`matmul_top` has `MAX_M` and `MAX_N` as module parameters and passes the same
value to `tile_ctrl`, `collector` and `host_if`. That closes
`collector_contract.md` simplification 6 (equal limits were previously an
unchecked convention). `ARRAY_DIM`, `DATA_WIDTH`, `ACC_WIDTH` and `MAX_K` come
from `pe_pkg` as everywhere else.

| Localparam | Value (default n = 5, MAX_M = MAX_N = 64, MAX_K = 1024) |
|---|---|
| `LANE_W = max(1, $clog2(n))` | 3 |
| `A_AW = TM_W + KI_W`, `B_AW = TN_W + KI_W`, `C_AW = ROW_W + TN_W` | 14, 14, 10 (`memory_contract.md` §3) |
| `WORD_W = max(A_AW, B_AW, C_AW)` | 14 |
| `HOST_AW = 2 + WORD_W + LANE_W` | 19 |
| `HOST_W = ACC_WIDTH` | 32 |

### 3.2 `matmul_top` ports

| Port | Dir | Width | Contract |
|---|---|---|---|
| `clk` | in | 1 | |
| `rst_n` | in | 1 | Async active-low, as every other block |
| `host_we` | in | 1 | Write `host_wdata` to `host_addr` on this edge |
| `host_re` | in | 1 | Read `host_addr` on this edge |
| `host_addr` | in | `HOST_AW` | `{region[1:0], word[WORD_W-1:0], lane[LANE_W-1:0]}` (§4) |
| `host_wdata` | in | `HOST_W` | |
| `host_rdata` | out | `HOST_W` | Valid the cycle after a `host_re` edge. Don't-care in other cycles |

`host_we` and `host_re` may both be high (same address). The read returns the
value as it was before the edge (read-first, as in `lane_ram`).

### 3.3 `host_if` ports

Same as the host side of `matmul_top`, plus:

| Port | Dir | Width | To / from |
|---|---|---|---|
| `start`, `M`, `N`, `K` | out | 1, `M_W`, `N_W`, `K_W` | `tile_ctrl` command inputs |
| `busy`, `done`, `error` | in | 1 each | `tile_ctrl` command outputs |
| `a_we`, `a_waddr`, `a_wlane_en`, `a_wdata` | out | 1, `A_AW`, n, `DATA_WIDTH` × n | `u_mem_a` write port |
| `b_we`, `b_waddr`, `b_wlane_en`, `b_wdata` | out | same, `B_AW` | `u_mem_b` write port |
| `c_re`, `c_raddr` | out | 1, `C_AW` | `u_mem_c` read port |
| `c_rdata` | in | `ACC_WIDTH` × n | `u_mem_c` read data |

## 4. Address map

| `region` | Space | `word` field | `lane` field | Write | Read |
|---|---|---|---|---|---|
| `00` | registers | register index (§5) | must be 0 | register write | register read |
| `01` | A | `{tile_m, k}` (A_AW bits; higher bits must be 0) | lane l < n | element `A[tile_m·n + l][k]` | **not readable**: returns 0 |
| `10` | B | `{tile_n, k}` (B_AW bits) | lane l < n | element `B[k][tile_n·n + l]` | returns 0 |
| `11` | C | `{row, tile_n}` (C_AW bits) | lane l < n | ignored (only the collector writes C) | element `C[row][tile_n·n + l]` |

**Unmapped** means a nonzero lane in region `00`, a lane ≥ n, `word` bits above the region's
width, or an unused register index. Unmapped writes are ignored, and unmapped
reads return 0. Neither sets an error. That's no aliasing and no surprises, at
the cost of one compare per field.

A and B are not readable because each RAM's single read port belongs to the
feeder. A host-side read mux would contend with it mid-job (known simplification 2).

## 5. Registers

| Index | Name | Access | Bits | Reset | Behaviour |
|---|---|---|---|---|---|
| 0 | `CTRL` | W | bit 0 `START` | — | Writing 1 while `BUSY` is low is **taken**: `tile_ctrl.start` is driven for exactly one cycle, with M, N, K from `DIM_*`, and `tile_ctrl` is certain to accept it on the next edge (nothing else can make it busy in between). Writing 1 while `BUSY` is high is ignored. Reads return 0 |
| 1 | `STATUS` | R | bit 0 `BUSY`, bit 1 `DONE`, bit 2 `CMD_ERROR`, bit 3 `HOST_ERROR` | 0 | `BUSY` = `tile_ctrl.busy` OR a taken start not yet accepted. `DONE` = sticky OR `tile_ctrl.done` (live). `CMD_ERROR` = sticky OR `done & error`. `HOST_ERROR` = sticky, set by an A/B write while `BUSY` (T6). The sticky bits clear on the edge a `START` write is taken; that clear wins over a set on the same edge (that `done` belongs to the previous job). Writes ignored |
| 2 | `DIM_M` | R/W | `M_W` | 0 | Sampled by `tile_ctrl` only on an accepted start |
| 3 | `DIM_N` | R/W | `N_W` | 0 | |
| 4 | `DIM_K` | R/W | `K_W` | 0 | |

Writing `DIM_*` during a job is allowed and harmless, because `tile_ctrl`
latched its config at start. A `START` while `BUSY` is ignored by `host_if`
itself (no pulse, no clear), which matches `tile_ctrl`'s own rule
(`fsm_contract.md` §8).

## 6. Timing

All relative to edges. A host operation sampled on edge e:

| Operation | Effect |
|---|---|
| Write `CTRL.START` on e (not `BUSY`) | Sticky bits clear on e; `start` high in the cycle after e (`BUSY` reads 1 from then); `tile_ctrl` accepts on e + 1. Job cycle t = 0 is the cycle after e + 1 |
| `tile_ctrl.done` high in job cycle L (`fsm_contract.md` §6.2) | `STATUS.DONE` reads 1 from cycle L itself (live), and stays 1 (sticky) |
| Read on e | `host_rdata` valid in the cycle after e. Registers are sampled on e; C goes through `lane_ram`'s 1-cycle read plus a lane mux on a registered lane index |
| A/B write on e | committed on e (when not busy) |

**Read-after-done is safe by construction.** The job's last C write is committed
on the edge that ends cycle L (`memory_contract.md` §6). A STATUS read sampled
on that same edge is the earliest that can see `DONE`, and its data reaches the
host in cycle L + 1. The host's first C read is therefore sampled on edge L + 2
or later, one edge after the last write. The race the memory contract warns
about can't happen through the bus.

**No idle-looking gap.** From the cycle after a taken `START` until `DONE`,
every STATUS read shows `BUSY` or `DONE`: the pending start covers the cycle
before `tile_ctrl` accepts, and live `DONE` covers the cycle before the sticky
bit (which matters for an illegal command, where `busy` never rises).

**Job latency through the bus** (`START` write edge to the first STATUS read
data showing `DONE`) is **L + 2** cycles: 1 for the start pulse, then L job
cycles, then 1 for the status read.

*Phase 2 finding (why §5/§6 changed).* The first version cleared the sticky bits
on the edge `tile_ctrl` *accepted* the start and made `DONE` sticky-only. In
the closed-loop model, a host polling right after `START` read STATUS on the
accept edge, got the **previous** job's `DONE` (read-first), and read stale C.
There was also a one-cycle all-zero STATUS after an illegal command. Clearing
on the `START` write, counting a pending start as `BUSY`, and showing `DONE`
live remove both without any host-side rule.

## 7. Illegal / degenerate inputs

| Situation | Behaviour |
|---|---|
| `START` with an illegal M/N/K (0 or over limit) | `tile_ctrl` rejects it: `DONE` = 1, `CMD_ERROR` = 1, no tiles run |
| `START` while `BUSY` (job running or start pending) | Ignored: no pulse, sticky bits unchanged |
| A/B write while `BUSY` | Ignored; `HOST_ERROR` = 1; the running job's result is **unaffected** |
| Write to C, write to `STATUS`, read of A/B, any unmapped address | Ignored / reads 0 |
| `host_we` and `host_re` together | Both happen; the read returns the old value |
| `rst_n` low | Registers and sticky bits 0, `tile_ctrl` idle. Memory contents are **not** reset (`memory_contract.md` D6) |

## 8. Properties

`host_if` alone:

1. `start` is high for exactly one cycle per `CTRL.START` write taken while not `BUSY`, and never otherwise.
2. `M`/`N`/`K` always equal the `DIM_*` registers.
3. The sticky bits are set only by their causes, and cleared only on a taken `START` (or reset); a taken `START` wins over a same-edge set. `BUSY`/`DONE`/`CMD_ERROR` read as defined in §5.
4. An A/B write produces exactly one memory write, with a one-hot lane enable at the decoded word and lane, iff the address is mapped and not busy.
5. Every read returns the addressed value one cycle later; unmapped reads return 0.
6. Nothing writes the C RAM from the host side; `c_re` is high only for a mapped C read.

System (`matmul_top`, driven only through the bus):

7. Load → start → poll → read gives `C == A @ B` for every shape, back-to-back, with no reset between jobs.
8. Every collector-side check from the integration harness still holds: each C element written once, the feeder reads only loaded words, `feed_done` only in FEED, and job latency L.
9. Illegal dims, start-while-busy and write-while-busy behave as §7. A write while busy does not change the result.

## 9. Known simplifications

1. **No handshake** (T1): the host must accept read data exactly one cycle later. An AXI4-Lite adapter is future work.
2. **A and B are write-only from the host**, so a host can't read back what it loaded.
3. **One element per transfer** (T2): loading a K×n tile takes K·n cycles. Fine for a smoke-testable v1; bandwidth is a later concern.
4. **The host must know the tile-word layout** (T3): n, TM_W/TN_W and KI_W. There is no config/ID register yet (open question H2).
5. **Polling only**, no interrupt pin.
6. **Memories are not cleared by reset**, so a C word never written by the current job holds an old value. The host only reads words inside M × N, which this job always writes.

## 10. Verification plan (same flow as the other blocks)

| Phase | Deliverable |
|---|---|
| 2 | `tb/host_if/host_if_model.py` (`HostIfModel`) with pytest, plus a pure address-map helper (`host_addr(region, word, lane)`) shared with the system test |
| 3 | Stub `rtl/host_if.sv` + cocotb. Lockstep against the model, monitors for §8.1–6 (start pulse, sticky set/clear causes, one-hot decode, read latency, unmapped behaviour), coverage, shown failing first |
| 4 | Real `rtl/host_if.sv`, green, lint clean |
| 5 | SVA and a mutation campaign. `rtl/matmul_top.sv` (wiring only) plus `tb/matmul_top/`: a Python bus-functional host and the §8.7–9 system tests, porting every check from `tb/tile_ctrl_integration/`. Then retire the old harness (T7, with your OK) |

Lint baseline: the upstream fixes on `refs/heads/claude/magical-mendeleev-24a667`
remove the four remaining whole-system warnings (`skew_buffer` reset style,
`pe_grid` dead code, `pe_pkg::ADDR_W`, `feeder.sv` newline). Until that branch is
merged, a `matmul_top` lint will still show them.

## 11. Open questions

- **H1.** An AXI4-Lite adapter in front of the SRAM-style bus: when, and should it be AXI4-Lite or AXI-Stream for the operand/result data?
- **H2.** A read-only `CONFIG` register (n, widths, limits) so host software can discover the layout instead of hard-coding it?
- **H3.** An interrupt (`irq = DONE`) for hosts that don't want to poll?
- **H4.** Word-wide burst loads (T2 alternative) once load time dominates a job.
