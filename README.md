# Systolic Array Project
### Anuj Lall

A hardware matrix-multiplication accelerator built on a systolic array of processing elements (PEs). Each PE does a multiply-accumulate; operands stream through the grid diagonally skewed, and results accumulate in place (output-stationary). Larger matrices will be handled by tiling over M, N, and the K reduction dimension. Memory is kept dumb — all intelligence lives in the feeder, address generator, and control FSM.

Next steps: implement skewed propagating capture; overlapping compute + drain

### Current progress
- [x] PE module complete and tested
- [x] PE grid module complete and tested
	- [x] consecutive matmuls
	- [x] MxK * KxN matmul support (PEs outside active range, MxN, are completely inactive)
	- [ ] double-buffer draining
		- [x] broadcast capture
		- [ ] skewed propagating capture
		- [ ] triple buffer propagating capture
- [ ] feeder module complete and tested
- [ ] control FSM module complete and tested
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
		- instead of having completely separate compute and drain phases, they can be overlapped; while the current matmul is computing, the previous matmul is simultaneously draining. This can be done with double-buffering; adding a separate "shadow acc" to each PE that copies the value of acc when "capture" is asserted. There are 3 setups, scaling in complexity but scaling throughput (NOT LATENCY, each matmul still takes 4n - 2 cycles to compute + drain in all 3 setups)
			1. broadcast-capture: after K + 2n - 2 cycles, when the last PE finishes computing, broadcast a single "capture" signal to all PEs; all accumulated sums are captured to the shadow buffers and the array is free to start computing again. However,capture can only be asserted once the last PE finishes its MACs, and the other PEs sit idle. Matmuls are completed (and drained) every K + 2n - 2 cycles.
  				- visualization of broadcast-capture double-buffer setup:  

					<img width="616.5" height="398.25" alt="broadcast-capture" src="https://github.com/user-attachments/assets/3b7d31c1-04bb-4bba-99f6-e767c00f46ce" />

			2. skewed, propagating capture: (2n? cycles between matmuls)
			3. triple-buffer propagating capture: (n? cycles between matmuls)

### Future work

**dummy memory + feeder / address generator** Flat, row-major memories (`address = base + row*stride + col`) for inputs and output. The real work is the feeder that walks tiles from flat memory and produces the skewed operand streams — the memory itself is trivial.

**control FSM with tiling loops** Nested loops over output tiles (M, N) with an inner K-reduction. Emits control signals only (`first_tile`, `load_en`, `drain_en`, addresses) and never touches data. Asserts `first_tile` on `k == 0`, skewed to the array diagonal. Keep the loop-nest counters separate from the array's internal cycle counter.

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