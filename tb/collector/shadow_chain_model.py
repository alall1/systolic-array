"""
Stand-in for pe_grid's shadow chain, as the collector sees it: capture loads
every row, each shift_en moves the chain down one row (zeros enter at the top),
drain_out is the bottom row. Same priority as pe: capture wins over shift_en.

step() = one rising clock edge; returns drain_out as visible AFTER it.
"""


class ShadowChain:
    def __init__(self, n):
        self.n = n
        self.sh = [[0] * n for _ in range(n)]

    def reset(self):
        self.sh = [[0] * self.n for _ in range(self.n)]
        return self.drain_out()

    def drain_out(self):
        return list(self.sh[self.n - 1])

    def step(self, capture, shift_en, acc=None):
        if capture:
            self.sh = [list(r) for r in acc]
        elif shift_en:
            self.sh = [[0] * self.n] + self.sh[:-1]
        return self.drain_out()
