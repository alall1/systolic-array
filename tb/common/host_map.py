"""
Host address map of matmul_top (docs/top_contract.md §3-5).

  host_addr = {region[1:0], word[WORD_W-1:0], lane[LANE_W-1:0]}

  region 0: registers   word = register index, lane must be 0
  region 1: A           word = {tile_m, k}    lane l -> A[tile_m*n + l][k]
  region 2: B           word = {tile_n, k}    lane l -> B[k][tile_n*n + l]
  region 3: C           word = {row, tile_n}  lane l -> C[row][tile_n*n + l]

Pure helpers: they compute addresses the way host software would (the
division by n happens here, in "software", never in the RTL).
"""

from tile_layout import clog2, k_index_width, tile_index_width, operand_addr, c_addr

REGION_REGS, REGION_A, REGION_B, REGION_C = 0, 1, 2, 3

REG_CTRL, REG_STATUS, REG_DIM_M, REG_DIM_N, REG_DIM_K = 0, 1, 2, 3, 4
NUM_REGS = 5

CTRL_START = 1 << 0
STATUS_BUSY, STATUS_DONE, STATUS_CMD_ERROR, STATUS_HOST_ERROR = 1 << 0, 1 << 1, 1 << 2, 1 << 3


class HostMap:
    def __init__(self, n, max_m, max_n, max_k, data_width, acc_width):
        self.n = n
        self.max_m, self.max_n, self.max_k = max_m, max_n, max_k
        self.data_width = data_width
        self.host_w = acc_width
        self.lane_w = max(1, clog2(n))
        self.ki_w = k_index_width(max_k)
        self.tm_w = tile_index_width(max_m, n)
        self.tn_w = tile_index_width(max_n, n)
        self.row_w = max(1, clog2(max_m))
        self.m_w = max_m.bit_length()           # clog2(MAX+1): tile_ctrl's port widths
        self.n_w = max_n.bit_length()
        self.k_w = max_k.bit_length()
        self.a_aw = self.tm_w + self.ki_w
        self.b_aw = self.tn_w + self.ki_w
        self.c_aw = self.row_w + self.tn_w
        self.word_w = max(self.a_aw, self.b_aw, self.c_aw)
        self.addr_w = 2 + self.word_w + self.lane_w

    # raw fields ------------------------------------------------------------
    def addr(self, region, word, lane=0):
        assert 0 <= region < 4 and 0 <= word < (1 << self.word_w) and 0 <= lane < (1 << self.lane_w)
        return (region << (self.word_w + self.lane_w)) | (word << self.lane_w) | lane

    def fields(self, addr):
        lane = addr & ((1 << self.lane_w) - 1)
        word = (addr >> self.lane_w) & ((1 << self.word_w) - 1)
        region = addr >> (self.word_w + self.lane_w)
        return region, word, lane

    def reg(self, index):
        return self.addr(REGION_REGS, index, 0)

    # element addresses, as host software computes them ---------------------
    def a_elem(self, r, k):
        return self.addr(REGION_A, operand_addr(r // self.n, k, self.ki_w), r % self.n)

    def b_elem(self, k, c):
        return self.addr(REGION_B, operand_addr(c // self.n, k, self.ki_w), c % self.n)

    def c_elem(self, r, c):
        return self.addr(REGION_C, c_addr(r, c // self.n, self.tn_w), c % self.n)
