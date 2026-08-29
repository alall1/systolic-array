# Systolic Array Project
### Anuj Lall

Current progress
- [x] PE module complete and tested
- [x] PE grid module complete and tested
- [ ] feeder module complete and tested
- [ ] control FSM module complete and tested
- [ ] collector module complete and tested
- [ ] top module complete and tested

Notes
- cycles to complete a matmul: K + 2(N-1)
	- K = accumulation depth / contraction dimension; A (M,K) * B (K, N) = C (M, N); the number of multiply-accumulates each PE does in a single matmul
	- however, for NxN * NxN matmul, first PE (top-left-most PE) finishes after 2(N-1) cycles; sits idle until last PE (bottom-right-most PE) finishes K cycles later
		- to reduce PE idle time, read the accumulators in the same cascading way that operands propagate through the grid; read acc[0][0] at 2(N-1), then acc[0][1] and acc[1][0] at 2(N-1) + 1, then read acc[0][2], acc[2][0], and acc[1][1] at 2(N-1) + 2, etc. This way, PEs can be freed up to start on the next matmul instead of idly waiting for their output to be read; also brings up the question of draining output
 - for consecutive matmuls propagate signal 'first' through the array, which "clears" each accumulator by appending just the product in the multiply-accumulate flow; doesn't waste a cycle like propagating an 'acc_clear' signal would, because instead of zeroing the accumulators 'first' instead appends the first multiply-accumulate product of the next matmul for each PE.
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


