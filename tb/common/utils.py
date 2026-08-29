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

def build_skew_schedule(A, B, P=None):
    """Rectangular skew schedule with per-direction valids. Subsumes the old
    square builder: pass square A, B (P defaults to N) for identical data
    streams, plus valids.

    Returns a_dat, a_val, b_dat, b_val, n_cycles -- each indexed [edge][cycle].
    Rows i>=M and cols j>=N are all zero with valid low.
    """
    A = np.asarray(A)
    B = np.asarray(B)
    M, K = A.shape
    K2, N = B.shape
    assert K == K2, f"inner dims must match: A is {A.shape}, B is {B.shape}"

    if P is None:
        P = max(M, N)
    assert M <= P and N <= P, f"array {P}x{P} too small for {M}x{K} * {K}x{N}"

    n_cycles = (P - 1) + (K - 1) + (P - 1) + 1 + 2  # far-corner drain + slack

    a_dat = [[0] * n_cycles for _ in range(P)]
    a_val = [[0] * n_cycles for _ in range(P)]
    b_dat = [[0] * n_cycles for _ in range(P)]
    b_val = [[0] * n_cycles for _ in range(P)]

    for i in range(M):
        for k in range(K):
            a_dat[i][i + k] = int(A[i][k])
            a_val[i][i + k] = 1
    for j in range(N):
        for k in range(K):
            b_dat[j][j + k] = int(B[k][j])
            b_val[j][j + k] = 1

    return a_dat, a_val, b_dat, b_val, n_cycles