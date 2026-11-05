"""
Cycle-accurate golden model for "collector" (drain -> row writes of C), plus a
pure helper that predicts one tile's writes WITHOUT the register model.

Contract: docs/collector_contract.md

step() = one rising clock edge. The control inputs (capture, shift_en, tile_*)
are the values visible BEFORE that edge. drain_out is the value visible AFTER
it: pe_grid.drain_out is combinational from the shadow registers, which update
on the same edge, and the write port passes it straight through. step()
returns the POST-edge value of every output.
"""

from utils import to_signed


class CollectorModel:
    def __init__(self, array_dim, max_m, max_n, acc_width):
        assert array_dim >= 2
        self.n = array_dim
        self.acc_width = acc_width
        # tile-index widths, same formulas as tile_ctrl
        tiles_m = -(-max_m // array_dim)
        tiles_n = -(-max_n // array_dim)
        self.tm_w = max(1, (tiles_m - 1).bit_length())
        self.tn_w = max(1, (tiles_n - 1).bit_length())
        self.dim_w = array_dim.bit_length()
        self.reset()

    def reset(self):
        """State after rst_n low. Returns the idle output snapshot."""
        self.active = False
        self.row = 0            # grid row now on drain_out
        self.fresh = False      # that row has not been offered for writing yet
        self.tm = 0
        self.tn = 0
        self.rows = 0           # latched M_t
        self.cols = 0           # latched N_t
        return self.outputs([0] * self.n)

    def outputs(self, drain_out):
        n = self.n
        wr_en = self.active and self.fresh and self.row < self.rows
        if not wr_en:
            return dict(wr_en=False, wr_row=0, wr_col=0, wr_lane_en=0, wr_data=[0] * n)
        return dict(
            wr_en=True,
            wr_row=self.tm * n + self.row,
            wr_col=self.tn * n,
            wr_lane_en=(1 << self.cols) - 1,
            wr_data=[to_signed(int(d), self.acc_width) for d in drain_out],
        )

    def step(self, capture=False, shift_en=False, tile_m=0, tile_n=0,
             tile_rows=0, tile_cols=0, drain_out=None):
        if drain_out is None:
            drain_out = [0] * self.n
        if capture:                                     # wins over shift_en (mirrors pe)
            self.active = True
            self.row = self.n - 1
            self.fresh = True
            # the DUT only sees the low port-width bits
            self.tm = tile_m & ((1 << self.tm_w) - 1)
            self.tn = tile_n & ((1 << self.tn_w) - 1)
            self.rows = tile_rows & ((1 << self.dim_w) - 1)
            self.cols = tile_cols & ((1 << self.dim_w) - 1)
        elif self.active and shift_en and self.row != 0:
            self.row -= 1
            self.fresh = True
        elif self.active and self.fresh and self.row == 0:
            self.active = False                         # row 0 was offered last cycle
            self.fresh = False
        else:
            self.fresh = False
        return self.outputs(drain_out)


# ---------------------------------------------------------------------- #
# Pure helper: no register model, only the contract's §5.1 rule.
# ---------------------------------------------------------------------- #

def expected_tile_writes(tile_m, tile_n, rows, cols, n):
    """Writes of one tile under v1 drain timing (shift_en on every cycle after
    capture). Offset 0 is the cycle right after the capture edge (grid row
    n-1 on drain_out); grid row r is on drain_out at offset n-1-r.

    Returns [(offset, grid_row, wr_row, wr_col, wr_lane_en)], bottom-up.
    """
    assert 1 <= rows <= n and 1 <= cols <= n
    return [(n - 1 - r, r, tile_m * n + r, tile_n * n, (1 << cols) - 1)
            for r in range(rows - 1, -1, -1)]
