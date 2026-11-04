# Systolic Array Project
### Anuj Lall

A hardware matrix-multiplication accelerator built on a systolic array of processing elements (PEs). Each PE does a multiply-accumulate; operands stream through the grid diagonally skewed, and results accumulate in place (output-stationary). Larger matrices are tiled over the output dimensions M and N by the control FSM (`tile_ctrl`); tiling over the K reduction dimension is planned. Memory is kept dumb — all intelligence lives in the feeder, address generator, and control FSM.

Next steps: collector module (drains each output tile from `drain_out` into the result matrix), then the operand buffer and top module

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
- [ ] collector module complete and tested
- [ ] top module complete and tested

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
	- per output tile: ISSUE (start the feeder) → FEED (wait for the feeder's `done`) → FLUSH (wait for the last operands to cross the skew buffer and grid) → CAPTURE (copy every accumulator into its shadow register) → DRAIN (shift the shadow registers out) → next tile or DONE. Illegal commands (a zero dimension, or one over its limit) go to REJECT, which pulses `done` and `error` together.
	- cycles per tile = K + M<sub>t</sub> + N<sub>t</sub> + n + 1, where M<sub>t</sub>, N<sub>t</sub> are the tile's rows/cols (≤ n). Job latency = T<sub>m</sub>·T<sub>n</sub>·(K + n + 1) + T<sub>n</sub>·M + T<sub>m</sub>·N, with T<sub>m</sub> = ⌈M/n⌉ and T<sub>n</sub> = ⌈N/n⌉ tiles.
	- CAPTURE lands in exactly the cycle the last active PE's accumulator becomes final (zero slack): capturing one cycle earlier corrupts only the bottom-right element of each tile, which both the Python model and the RTL integration test confirm.
	- edge tiles need nothing special from the FSM: it passes the smaller M<sub>t</sub>/N<sub>t</sub> to the feeder, and the inactive PEs stay idle exactly as before. They do keep stale accumulators from earlier tiles, so the collector must only keep the M<sub>t</sub> × N<sub>t</sub> region of each drained tile.
	- v1 is deliberately serialized (the next tile waits for the previous drain); overlapping them is listed below.

### Running the tests
Each module has its own cocotb directory under `tb/` (Verilator + cocotb, from the repo's venv):
```
cd tb/tile_ctrl && make               # FSM: lockstep + monitors + coverage (SVA on)
cd tb/tile_ctrl && make model         # FSM golden model, plain pytest
cd tb/tile_ctrl_integration && make   # tile_ctrl + feeder + pe_grid vs numpy
```
Useful knobs: `ARRAY_DIM=<n>`, `SEED=<n>`, `WAVES=1` (GTKWave dump), `SVA=0`, and for `tb/tile_ctrl`, `LOCKSTEP=0` (monitors judge alone).

### Known issues (found while building the FSM, not yet fixed)
- read_sequencer hangs on K = 0 (`busy` stays high until reset); `tile_ctrl` rejects K = 0 so it never reaches the feeder.
- skew_buffer uses a synchronous reset while every other module is asynchronous (Verilator `SYNCASYNCNET` lint warning).
- skew_buffer delays lane *k* by *k* + 1 cycles (the relative skew is still correct) and declares n × n registers per side rather than the triangular count.
- small cleanups: unused `count`/`FINISH` in pe_grid, unused `ADDR_W` in pe_pkg, `drain_out` is unsigned while `out` is signed, missing newline at the end of `feeder.sv`.

### Future work

**collector** Consumes `drain_out` after each `capture` (grid row n-1 first, one row further up per cycle), keeps only the M<sub>t</sub> × N<sub>t</sub> region, and places it using `tile_m`/`tile_n`. Today this exists only as a Python stand-in in the integration test.

**dummy memory / operand buffer** Flat, row-major memories (`address = base + row*stride + col`) for inputs and output. The feeder already produces the skewed operand streams; what is missing is a buffer that serves the operand tile named by `tile_m`/`tile_n` with a 1-cycle read (also a Python stand-in today).

**K-tiling** Accumulate partial sums across passes so K can exceed `MAX_K`. Needs an upstream change: read_sequencer always asserts `first` on k = 0, which overwrites the accumulators every pass.

**overlapped tiles** Let tile *i*'s capture/drain run during tile *i+1*'s feed, which the shadow-register double buffer was built for. The legal capture window is derived in the contract; this would cut the per-tile cost to roughly max(K + 1, K + M<sub>t</sub> + N<sub>t</sub> − 2, n) cycles.

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