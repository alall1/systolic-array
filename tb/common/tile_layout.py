"""
Tile-word layout of the operand and result memories (docs/memory_contract.md §3).
One memory word holds the n lanes that move together in one cycle; every word
address is a concatenation of indices, so the RTL needs no address arithmetic.

  A (transposed): word {tile_m, k}  lane l = A[tile_m*n + l][k]
  B             : word {tile_n, k}  lane l = B[k][tile_n*n + l]
  C             : word {row, tile_n} lane l = C[row][tile_n*n + l]

Padding lanes past the edge of A / B (rows >= M, cols >= N) are filled by
`pad(lane)`: the hardware never uses them (valid = 0), so tests fill them with
junk to prove it.
"""


def clog2(x):
    return (x - 1).bit_length()


def ceil_div(a, b):
    return -(-a // b)


def k_index_width(max_k):
    """KI_W: holds k = 0 .. MAX_K-1."""
    return max(1, clog2(max_k))


def tile_index_width(max_dim, n):
    """TM_W / TN_W, same formula as tile_ctrl."""
    return max(1, clog2(ceil_div(max_dim, n)))


def operand_addr(tile, k, ki_w):
    return (tile << ki_w) | k


def c_addr(row, tile_n, tn_w):
    return (row << tn_w) | tile_n


def pack_a_tiles(A, n, ki_w, pad=lambda lane: 0):
    """{addr: [n lanes]} for A (M x K), stored transposed in tile words."""
    M, K = len(A), len(A[0])
    return {operand_addr(tm, k, ki_w): [int(A[tm * n + l][k]) if tm * n + l < M else pad(l)
                                        for l in range(n)]
            for tm in range(ceil_div(M, n)) for k in range(K)}


def pack_b_tiles(B, n, ki_w, pad=lambda lane: 0):
    """{addr: [n lanes]} for B (K x N) in tile words."""
    K, N = len(B), len(B[0])
    return {operand_addr(tn, k, ki_w): [int(B[k][tn * n + l]) if tn * n + l < N else pad(l)
                                        for l in range(n)]
            for tn in range(ceil_div(N, n)) for k in range(K)}


def c_words(M, N, n, tn_w):
    """Every C word address that holds part of an M x N result."""
    return [c_addr(r, t, tn_w) for r in range(M) for t in range(ceil_div(N, n))]


def unpack_c(read_word, M, N, n, tn_w):
    """C as a list of rows, from read_word(addr) -> [n lanes]. Lanes past N are ignored."""
    C = [[None] * N for _ in range(M)]
    for r in range(M):
        for t in range(ceil_div(N, n)):
            word = read_word(c_addr(r, t, tn_w))
            for l in range(n):
                if t * n + l < N:
                    C[r][t * n + l] = word[l]
    return C
