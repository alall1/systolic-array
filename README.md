# Systolic Array Project
### Anuj Lall

Current progress
- [x] PE module complete and tested
- [ ] PE grid module complete and tested
- [ ] feeder module complete and tested
- [ ] control FSM module complete and tested
- [ ] collector module complete and tested
- [ ] top module complete and tested

Notes
- cycles to complete a matmul: K + 2(N-1)
	- K = accumulation depth / contraction dimension; A (M,K) * B (K, N) = C (M, N); the number of multiply-accumulates each PE does in a single matmul
