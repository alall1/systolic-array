# Systolic Array Project
### Anuj Lall

A hardware matrix-multiplication accelerator built on a systolic array of processing elements (PEs). Each PE does a multiply-accumulate; operands stream through the grid diagonally skewed, and results accumulate in place (output-stationary). Larger matrices are tiled over the output dimensions M and N by the control FSM (`tile_ctrl`); tiling over the K reduction dimension is planned. Memory is kept dumb — all intelligence lives in the feeder, address generator, and control FSM.

Next steps: K-tiling (accumulate across passes so K can exceed `MAX_K`; needs a `read_sequencer` change), then valid/ready handshakes

### Current progress
- [x] PE module complete and tested
- [x] PE grid module complete and tested
	- [x] consecutive matmuls
	- [x] MxK * KxN matmul support (PEs outside active range, MxN, are completely inactive)
	- [x] double-buffer broadcast capture
- [x] feeder module complete and tested
	- [x] skew_buffer module
	- [x] read_sequencer module
- [x] control FSM module (`tile_ctrl`) complete and tested
	- [x] M/N output tiling, including partial edge tiles (M or N not a multiple of the grid size)
	- [x] contract written first: [docs/fsm_contract.md](docs/fsm_contract.md)
	- [x] Python golden model + pure schedule helper, validated standalone with pytest
	- [x] cocotb lockstep testbench, concurrent protocol/schedule monitors, functional coverage closure
	- [x] SVA assertions in the RTL, lint clean, mutation-tested (10 planted bugs, all caught)
	- [x] integration smoke test: `tile_ctrl` + feeder + pe_grid computes tiled matmuls equal to numpy
	- [x] v2: each tile's capture/drain overlaps the next tile's feed (27–40% fewer cycles for K ≤ n; full tiles K + 2n − 2 cycles instead of K + 3n + 1), same contract-first flow: probe, contract, model, red bench, RTL, SVA, mutation-tested (18 planted bugs, all caught)
- [x] collector module complete and tested
	- [x] contract written first: [docs/collector_contract.md](docs/collector_contract.md)
	- [x] Python golden model, validated standalone, incl. a closed-loop tiled matmul through the FSM, feeder and PE-grid models
	- [x] cocotb lockstep testbench, concurrent write-port/tile-write monitors, functional coverage closure
	- [x] SVA assertions in the RTL, lint clean, mutation-tested
	- [x] system smoke test: `tile_ctrl` + feeder + pe_grid + `collector` writes C equal to numpy, each element exactly once
- [x] operand + result memories (`lane_ram`, used three times) complete and tested
	- [x] contract written first: [docs/memory_contract.md](docs/memory_contract.md)
	- [x] tile-word layout: every address is a concatenation of existing indices, no address arithmetic
	- [x] Python golden model + layout helpers, validated with a closed-loop tiled matmul through all the other models
	- [x] cocotb lockstep testbench, read scoreboard + hold monitor, functional coverage closure
	- [x] SVA assertions in the RTL, lint clean, mutation-tested
	- [x] system test: every byte of A, B and C goes through RTL; Python only plays the host
- [x] top module (`matmul_top`) + host interface (`host_if`) complete and tested
	- [x] contract written first: [docs/top_contract.md](docs/top_contract.md)
	- [x] SRAM-style host bus: one element per transfer, address `{region, word, lane}`, registers CTRL / STATUS / DIM_M/N/K
	- [x] `host_if`: Python golden model (incl. a closed loop of every block model driven only through the bus), cocotb lockstep testbench, register scoreboard + decode monitor, coverage closure, SVA, mutation-tested
	- [x] `matmul_top` is pure wiring; its system test drives only the bus: load, start, poll, read C == numpy, back-to-back jobs, illegal dims, interference during a job, reset mid-job

### Notes
- this design is *output-stationary*, meaning two matrices (e.g. weights and activations) stream in at the same time and the output matrix is "stationary", staying in each PE. 
- cycles to complete a matmul: K + (M-1) + (N-1) = M + K + N - 2
	- K = accumulation depth / contraction dimension; A (M,K) * B (K, N) = C (M, N); the number of multiply-accumulates each PE does in a single matmul
	- however, for nxn * nxn matmul, first PE (top-left-most PE) finishes after n cycles; sits idle until last PE (bottom-right-most PE) finishes 2n - 2 cycles later unless the feeder starts streaming the values for the next matmul after N cycles.
- for consecutive matmuls propagate signal 'first' through the array, which "clears" each accumulator by appending just the product in the multiply-accumulate flow; doesn't waste a cycle like propagating an 'acc_clear' signal would, because instead of zeroing the accumulators 'first' instead appends the first multiply-accumulate product of the next matmul for each PE.
	- implementation below fixed by having payloads instead of separate data/first/valid propagation; "first" now rides with the first element of matrix A, clearing the array
	- having the following code wouldn't work, because the in_first_bus can't have multiple drivers (is a `logic` bus):
	```
	always_comb begin
		for (int i = 0; i < N; i++) begin
			for (int j = 0; j < N; j++) begin
				in_first_bus[i][j+1] = out_first_bus[i][j];
				in_first_bus[i+1][j] = out_first_bus[i][j];
			end
		end
	end
	```
	- instead, have only the top row of the grid propagate right, and the rest propagate down, as follows:
	```
	always_comb begin
		for (int i = 0; i < N; i++) begin
			for (int j = 0; j < N; j++) begin
				if (i == 0) in_first_bus[i][j+1] = out_first_bus[i][j];     // only top row "first" signas propagate left; the rest propagate downward
				in_first_bus[i+1][j] = out_first_bus[i][j];
			end
		end
	end
	```
	- visualization of 'first' signal propagation (for a single matmul):

		<img width="619" height="472" alt="first_propagation" src="https://github.com/user-attachments/assets/9b519eac-2b44-44d8-88f8-042022de8af4" />

- for rectangular matmuls (MxK * KxN), a regular systolic array setup with all PEs completing a MAC every cycle would work (note that M <= n and N <= n for an nxn array); nothing would be added to the accumulators of the unused PEs, since 0 * anything = 0. However, this introduces two problems:
	1. energy; a zero-multiply still uses the multiplier and accumulator every cycle, even if it is adding nothing to the accumulated sum. On real silicon, MAC is the dominant dynamic-power consumer. Feeding zeros through unused PEs for a small matmul burns power for guaranteed-zero results
	2. the feeder would need to feed in zeroes to the rows/columns of the array that don't exist in A and B; correctness relies on the feeder being perfect with injecting exactly zero into every unused position at every cycle. Any garbage values that leak in corrupts a PE which you then need to remember to ignore.
- my solution to this is two have two valid signals propagating alongside the inputs: a_valid and b_valid. With every element of matrix A and matrix B, a_valid and b_valid propagate through the array with them. Each PE has an internal signal, in_valid, that decides whether they perform a MAC during a cycle or not; in_valid is only high if BOTH a_valid and b_valid are high--a_valid propagates rightwards with the matrix A values and b_valid propagates downwards with the matrix B values, so if and only if both operands coming into a PE from matrices A and B are real, then the PE will MAC.

- reading the output matrix by reaching into each PE's accumulator is something the testbench can do, but is impractical in real hardware because of wiring and I/O. 
	- if every PE exposes its accumulator as an output that leaves the nxn array, you need n<sup>2</sup> result buses routed from the interior of the array all the way to the edge, or to n<sup>2</sup> output ports. This creates a few problems:
		1. routing congestion: PEs in the middle of the array are physically surrounded by other PEs; to read one of them directly, its accumulator bus needs to route over or through the region occupied by the other PEs to reach the edges. For all n<sup>2</sup> PEs, you have n<sup>2</sup> buses that are acc bits wide crossing the array. Wire area grows as n<sup>2</sup> times acc-width, and competes for the area the PEs need for their own connections, causing interior routing congestion.
		2. I/O port count: if the module exposes n<sup>2</sup> accumulator ports as output, that is n<sup>2</sup> * acc-width output pins; for larger arrays, it creates many output pins (n = 16, 32-bit acc -> 8192 output pins)
		3. timing: a bus routed from the middle of the array to the edge is long, so it may be hard to close timing on. Neighbor-to-neighbor drain wires are short and pipelined, so they run at full clock. Wires directly from the interior would force a slower clock or add pipeline stages on the long buses, which would just be a worse version of the shift-chain.
	- the drain solution: have a shift-chain that drains PEs from neighbor-to-neighbor, rightwards or downwards (chose downwards for this design). The accumulated sum drains downward each cycle, completely draining after n cycles in an nxn array. However, draining creates an extra phase, meaning worst case matmuls would finish every K + 2n - 2 (compute phase) + n (drain phase) cycles.
		- instead of having completely separate compute and drain phases, they can be overlapped; while the current matmul is computing, the previous matmul is simultaneously draining. This can be done with double-buffering; adding a separate "shadow acc" to each PE that copies the value of acc when "capture" is asserted. Overlapping the phases increases throughput (NOT LATENCY, each matmul still takes 4n - 2 cycles to compute + drain) 
		- the implementation: broadcasting a "capture" signal; after K + 2n - 2 cycles, when the last PE finishes computing, broadcast a single "capture" signal to all PEs; all accumulated sums are captured to the shadow buffers and the array is free to start computing again. However,capture can only be asserted once the last PE finishes its MACs, and the other PEs sit idle. Matmuls are completed (and drained) every K + 2n - 2 cycles.
  				- visualization of broadcast-capture double-buffer setup:  

					<img width="616.5" height="398.25" alt="broadcast-capture" src="https://github.com/user-attachments/assets/3b7d31c1-04bb-4bba-99f6-e767c00f46ce" />

- for the feeder module, I've decided to split it into two submodules: skew_buffer and read_sequencer. The role of the feeder module is to read a flat, compressed matrix from memory (eventually from a buffer) and feed the skewed schedule to the pe_grid module.
	- read_sequencer: reads operands from memory/buffer; each cycle, one A-element per row (essentially a column per cycle) and one B-element per column (essentially a row per cycle) in flat order; not skewed yet. Decodes layout of compressed matrices in the buffer/memory; owns layout decode and offset arithmetic. Also injects "valid" and "first" signals into the a_payload_t and b_payload_t types before passing along to skew_buffer.
	- skew_buffer: triangular bank of shift-register chains; lane *k* delayed by *k* cycles. (n * (n - 1)) / 2 slots per operand side. Delay is the storage; a payload sits in the chain during its delay. Completely opaque to payload contents, only delaying each payload as one unit without touching anything inside. Input is flat payload lanes from read_sequencer, output is the skewed payload lanes into each grid edge.

- the control FSM, `tile_ctrl`, only emits control signals: it never touches data, addresses, or the valid/first bits (read_sequencer owns the k loop, the read addresses, and valid/first). Full contract: [docs/fsm_contract.md](docs/fsm_contract.md).
	- v2 runs two sequencers. The **front** FSM issues tiles: ISSUE (start the feeder) → FEED (until the feeder's `done`) → WAIT (until the schedule allows the next tile) → … → TAIL (last tile fed) → DONE. The **back** sequencer holds one tile in a capture slot, pulses `capture` (copy every accumulator into its shadow register), then `shift_en` n − 1 times to drain it. A tile moves from front to back on the edge that ends its `feed_done` cycle, so the next tile's feed overlaps this tile's flush, capture and drain. Illegal commands (a zero dimension, or one over its limit) go to REJECT, which pulses `done` and `error` together.
	- each tile still captures K + M<sub>t</sub> + N<sub>t</sub> + 1 cycles after its ISSUE, exactly when the last active PE's accumulator becomes final (zero slack: one cycle early corrupts the bottom-right element). The next ISSUE comes max(K + 2, M<sub>t</sub> + N<sub>t</sub> + max(K − 2, 0), n + 1 + (M<sub>t</sub> + N<sub>t</sub>) − (M<sub>t</sub>' + N<sub>t</sub>')) cycles later: the feeder must be free; the capture must happen before the next tile's first operand overwrites PE(0,0), three cycles after its ISSUE; and captures stay n + 1 apart so a drain finishes before the next capture. Job latency is the sum of those gaps plus the last tile's K + M<sub>t</sub> + N<sub>t</sub> + n + 1.
	- the overlap is limited by the broadcast `capture`: all PEs capture in one cycle, so a tile waits for its far-corner PE while the next tile already needs PE(0,0). A capture that ripples through the grid with the data would go further, but changes `pe` / `pe_grid`.
	- because the feed side has moved on by the time a tile is captured, `tile_ctrl` has separate outputs (`cap_tile_m`, `cap_tile_n`, `cap_M`, `cap_N`) naming the tile being captured; the collector takes those, the operand memories keep `tile_m` / `tile_n`.
	- edge tiles need nothing special from the FSM: it passes the smaller M<sub>t</sub>/N<sub>t</sub> to the feeder, and the inactive PEs stay idle exactly as before. They do keep stale accumulators from earlier tiles, so the collector must only keep the M<sub>t</sub> × N<sub>t</sub> region of each drained tile.
- the collector turns the grid's drain into row writes of C: after each `capture`, `drain_out` shows grid row n-1, then one row further up per `shift_en`. Rows ≥ M<sub>t</sub> hold stale accumulators and are skipped; each kept row is written at global row `tile_m·n + r`, column `tile_n·n`, with lane enables for the N<sub>t</sub> valid columns. Full contract: [docs/collector_contract.md](docs/collector_contract.md).
	- it must latch its tile inputs at `capture`: they come from `tile_ctrl`'s `cap_*` outputs, which already move to the next tile partway through this tile's drain. A collector that reads them live writes the last rows of every tile into the next tile's place (the system test catches this as double writes).
	- it follows `shift_en` rather than counting cycles after `capture`, so it stays correct if a later version stalls or overlaps the drain.
	- the write port is not registered, so the last write of a job lands in `tile_ctrl`'s `done` cycle: `done` already means "all of C is written by the end of this cycle".
- the memories store data in a **tile-word layout**: one word holds the n lanes that move together in one cycle, so every address is a concatenation of indices that already exist on some port. A is stored **transposed** (word `{tile_m, k}` holds `A[tile_m·n + l][k]`), so the n rows the feeder needs each cycle come from one wide read instead of n banks; B is word `{tile_n, k}`; C is word `{row, tile_n}`. Full contract: [docs/memory_contract.md](docs/memory_contract.md).
	- one generic `lane_ram` (per-lane write enables, registered read-first read, no reset) is instantiated three times; the layout lives entirely in the address wiring.
	- the collector sends the tile column `wr_tile_n` rather than the element column, because recovering `tile_n` from `tile_n·n` would need a divide by n.
	- the host must start reading C the cycle **after** `done`: the job's last row is committed on the edge that ends the `done` cycle, and a read on that same edge returns the old word (the system test's own host got this wrong the first time).
- the host sees one SRAM-style bus (always ready, read data one cycle later). Each transfer is one element; the address is `{region, word, lane}` with region 0 = registers, 1 = A, 2 = B, 3 = C, and word/lane in the tile-word layout, so host software does the `/ n` and `% n`, never the hardware. A job is: write A and B elements, write `DIM_M`, `DIM_N`, `DIM_K`, write `CTRL.START`, poll `STATUS` until `DONE`, read C elements. Full contract: [docs/top_contract.md](docs/top_contract.md).
	- `STATUS.BUSY` also covers a start that has been written but not yet accepted, and `DONE` / `CMD_ERROR` read as sticky OR live. The first design cleared the sticky bits one cycle later, on the accept edge; the closed-loop model caught a host polling right after `START` reading the **previous** job's `DONE` and then stale C.
	- because `DONE` is only visible once the last C row is being committed, a polling host can never read C too early: the read-after-done race of the memories is impossible through the bus.
	- A / B writes during a job are dropped and set `STATUS.HOST_ERROR`; the running job's result is unaffected.

### Running the tests
Each module has its own cocotb directory under `tb/` (Verilator + cocotb, from the repo's venv):
```
cd tb/tile_ctrl && make               # FSM: lockstep + monitors + coverage (SVA on)
cd tb/tile_ctrl && make model         # FSM golden model, plain pytest
cd tb/collector && make              # collector: lockstep + monitors + coverage (SVA on)
cd tb/collector && make model        # collector golden model, plain pytest
cd tb/lane_ram && make               # lane_ram: lockstep + monitors + coverage (SVA on)
cd tb/lane_ram && make model         # lane_ram model + tile layout, plain pytest
cd tb/host_if && make                # host_if: lockstep + monitors + coverage (SVA on)
cd tb/host_if && make model          # host_if model + closed loop through the bus, plain pytest
cd tb/matmul_top && make             # whole accelerator, driven only through the host bus, vs numpy
```
Useful knobs: `ARRAY_DIM=<n>`, `SEED=<n>`, `WAVES=1` (GTKWave dump), `SVA=0`, and for `tb/tile_ctrl`, `LOCKSTEP=0` (monitors judge alone).

### Known issues (found while building the FSM, not yet fixed)
- read_sequencer hangs on K = 0 (`busy` stays high until reset); `tile_ctrl` rejects K = 0 so it never reaches the feeder.
- skew_buffer uses a synchronous reset while every other module is asynchronous (Verilator `SYNCASYNCNET` lint warning).
- skew_buffer delays lane *k* by *k* + 1 cycles (the relative skew is still correct) and declares n × n registers per side rather than the triangular count.
- small cleanups: unused `count`/`FINISH` in pe_grid, unused `ADDR_W` in pe_pkg, `drain_out` is unsigned while `out` is signed, missing newline at the end of `feeder.sv`.

### Future work

**K-tiling** Accumulate partial sums across passes so K can exceed `MAX_K`. Needs an upstream change: read_sequencer always asserts `first` on k = 0, which overwrites the accumulators every pass.

**ripple capture** Overlapped tiles (done, `tile_ctrl` v2) are limited by the broadcast `capture`. Capturing each PE as its own result becomes final, in a wave that follows the data, would bring the tile period near K + 1. It changes `pe` and `pe_grid`.

**valid/ready handshake interfaces** Explicit transfer contract on feeder→array and array→output: a transfer happens only when `valid && ready`. `valid` must not depend combinationally on `ready`; once high, data stays stable until the transfer completes. Backpressure propagates — a stalled output freezes the array with no dropped results. Optionally AXI-Stream naming, deliberately not full AXI4-MM.

**Double buffering / ping-pong***
Overlap load and compute so the array never idles on a load.

```
        ┌──────────┐  read    ┌───────────┐
        │ Buffer A │─────────▶│  PE grid  │
        └──────────┘          └───────────┘
        ┌──────────┐  write (loading next tile)
 mem ──▶│ Buffer B │
        └──────────┘
   swap when current compute drains AND next load completes
```

read A, load B <-> read B, load A (ping pong)