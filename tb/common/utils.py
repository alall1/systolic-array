"""
Shared helpers for the systolic array testbenches

functions that live here:
  1. to_signed - interpret the low 'width' bits of 'value' as two's complement; used to convert n-bit representations into 32-bit Python int
  2. to_unsigned - interpret the low 'width' bits of 'value' as unsigned int; used to convert 32-bit Python int to n-bit representation
  3. golden_matmul - the independent reference for matmuls. Just numpy.
  4. build_skew_schedule - converts matrices A, B into the per-edge, per-cycle input streams the grid needs, plus the cycle count to drain. 
    - This is the timing spec the cocotb driver follows.
"""

import numpy as np

def to_signed(value: int, width: int) -> int:
    """Interpret the low 'width' bits of 'value' as two's complement; used to convert n-bit representations into 32-bit Python int"""
    value &= (1 << width) - 1
    sign_bit = 1 << (width - 1)
    return (value - (1 << width)) if (value & sign_bit) else value

def to_unsigned(value: int, width: int) -> int:
    """Interpret the low 'width' bits of 'value' as unsigned int; used to convert 32-bit Python int to n-bit representation"""
    return value & ((1 << width) - 1)

def golden_matmul(A, B):
    """The golden model for matmuls"""
    return np.asarray(A) @ np.asarray(B)


def build_skew_schedule(A, B):
    """
    Return (a_in, b_in, n_cycles).

    a_in[i][t] = value to drive into the LEFT edge of grid row i at cycle t.
    b_in[j][t] = value to drive into the TOP  edge of grid col j at cycle t.
    n_cycles   = number of clock cycles to run so the array fully fills and drains; after this many active cycles every acc[i][j] holds the final out[i][j].

    Positions not fed by real data are 0 (safe: zero multiplies to zero -> no accumulate)
    """
    A = np.asarray(A)
    B = np.asarray(B)
    N = A.shape[0]
    assert A.shape == (N, N) and B.shape == (N, N), "square NxN only (for now)"

    # A value A[i][k] must reach PE(i,j) at the right time for every j; the
    # edge injection is delayed by the row index i. The last useful value
    # enters at cycle (N-1)+(N-1) and needs another N-1 to propagate to the
    # far corner, so 3N is a safe upper bound. We use the tight value below.
    n_cycles = 3 * N

    a_in = [[0] * n_cycles for _ in range(N)]
    b_in = [[0] * n_cycles for _ in range(N)]

    for i in range(N):
        for k in range(N):
            a_in[i][i + k] = int(A[i][k])   # row i, delayed by i
    for j in range(N):
        for k in range(N):
            b_in[j][j + k] = int(B[k][j])   # col j, delayed by j

    return a_in, b_in, n_cycles