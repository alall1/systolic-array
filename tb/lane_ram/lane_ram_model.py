"""
Cycle-accurate golden model for "lane_ram": one write port with per-lane
enables, one registered read port, read-first, no reset.

Contract: docs/memory_contract.md §4

step() = one rising clock edge. Inputs are the values visible BEFORE that edge;
it returns rdata as visible AFTER it. Unknown values (rdata before the first
read, lanes of a word never written) are None: the contract makes them
don't-care, so a testbench compares only lanes the model knows.
"""


class LaneRamModel:
    def __init__(self, lanes, width, addr_w):
        self.lanes = lanes
        self.width = width
        self.addr_w = addr_w
        self.mem = {}                       # addr -> [value or None] * lanes
        self.rdata = [None] * lanes

    def peek(self, addr):
        return list(self.mem.get(addr & ((1 << self.addr_w) - 1), [None] * self.lanes))

    def step(self, we=False, waddr=0, wlane_en=0, wdata=None,
             re=False, raddr=0):
        amask = (1 << self.addr_w) - 1
        dmask = (1 << self.width) - 1
        if re:                              # read-first: sample before this edge's write
            self.rdata = self.peek(raddr)
        if we:
            word = self.peek(waddr)
            for l in range(self.lanes):
                if wlane_en >> l & 1:
                    word[l] = wdata[l] & dmask
            self.mem[waddr & amask] = word
        return list(self.rdata)
