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

