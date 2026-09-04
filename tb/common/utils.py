"""
Shared helpers for the systolic array testbenches

functions that live here:
  1. to_signed - interpret the low 'width' bits of 'value' as two's complement; used to convert n-bit representations into 32-bit Python int
  2. to_unsigned - interpret the low 'width' bits of 'value' as unsigned int; used to convert 32-bit Python int to n-bit representation
  3. golden_matmul - the independent reference for matmuls. Just numpy.
  4. pack_a - packing inputs into the a_payload_t type (data, valid, and first)
  5. unpack_a - slicing the a_payload_t type into data, valid, and first
  6. pack_b - packing inputs into the b_payload_t type (data, valid)
  7. unpack_b - slicing the b_payload_t type into data and valid
  8. build_skew_schedule - converts matrices A, B into the per-edge, per-cycle input streams the grid needs, plus the cycle count to drain. 
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

def pack_a(data, valid, first, data_width):
    """Packing inputs into the a_payload_t type (data, valid, first)"""
    data_u = to_unsigned(data, data_width)
    return (data_u << 2) | ((valid & 1) << 1) | (first & 1)

def unpack_a(word, data_width):
    """Slicing the a_payload_t type into data, valid, and first"""
    word  = int(word)
    first = word & 1
    valid = (word >> 1) & 1
    data  = to_signed((word >> 2) & ((1 << data_width) - 1), data_width)
    return data, valid, first

def pack_b(data, valid, data_width):
    """Packing inputs into the b_payload_t type (data, valid)"""
    data_u = to_unsigned(data, data_width)
    return (data_u << 1) | (valid & 1)

def unpack_b(word, data_width):
    """Slicing the b_payload_t type into data and first"""
    word  = int(word)
    valid = word & 1
    data  = to_signed((word >> 1) & ((1 << data_width) - 1), data_width)
    return data, valid

def build_skew_schedule(A, B, P=None):
    """
    Rectangular skew schedule with per-direction valids plus "first" signal propagating with A payload
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

    n_cycles = (P - 1) + (K - 1) + (P - 1) + 1  # 3N - 2 cycles -> acc[n-1][n-1] just finished

    a_data = [[0] * n_cycles for _ in range(P)]
    a_valid = [[0] * n_cycles for _ in range(P)]
    a_first = [[0] * n_cycles for _ in range(P)]

    b_data = [[0] * n_cycles for _ in range(P)]
    b_valid = [[0] * n_cycles for _ in range(P)]

    for i in range(M):
        for k in range(K):
            c = i + k
            a_data[i][c] = int(A[i][k])
            a_valid[i][c] = 1
            if k == 0:
                a_first[i][c] = 1
    for j in range(N):
        for k in range(K):
            c = j + k
            b_data[j][c] = int(B[k][j])
            b_valid[j][c] = 1

    return a_data, a_valid, a_first, b_data, b_valid, n_cycles