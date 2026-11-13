"""
Cycle-accurate golden model for "host_if" (bus decode, registers, sticky status,
start pulse, C read mux). Contract: docs/top_contract.md §3-8.

Two halves, matching the RTL:
  decode(...)  pure, same-cycle: the memory-port outputs are combinational from
               the bus inputs (a write is committed by lane_ram on the edge the
               host presents it; a C read is sampled on that edge)
  step(...)    one rising edge: registers, sticky bits, start pulse, read data.
               Bus inputs and busy/done/error are the values BEFORE the edge;
               c_rdata is the C RAM's read data AFTER it (lane_ram's registered
               output, muxed by host_if with a registered lane index).
               Returns the post-edge outputs; host_rdata is None when the edge
               carried no read (don't-care by contract).
"""

from host_map import (REGION_REGS, REGION_A, REGION_B, REGION_C, REG_CTRL, REG_STATUS,
                      REG_DIM_M, REG_DIM_N, REG_DIM_K, NUM_REGS, CTRL_START,
                      STATUS_BUSY, STATUS_DONE, STATUS_CMD_ERROR, STATUS_HOST_ERROR)


class HostIfModel:
    def __init__(self, hmap):
        self.m = hmap
        self.reset()

    def reset(self):
        self.dim = {REG_DIM_M: 0, REG_DIM_N: 0, REG_DIM_K: 0}
        self.start_q = False
        self.done_s = False
        self.cmd_error_s = False
        self.host_error_s = False
        self.read_q = None          # what the last edge read: ("val", v) | ("c", lane) | None
        return self.outputs()

    # ------------------------------------------------------------------ #
    def _mapped(self, region, word, lane):
        m = self.m
        if region == REGION_REGS:
            return lane == 0 and word < NUM_REGS
        if lane >= m.n:
            return False
        width = {REGION_A: m.a_aw, REGION_B: m.b_aw, REGION_C: m.c_aw}[region]
        return word < (1 << width)

    def decode(self, we=False, re=False, addr=0, wdata=0, busy=False):
        """Same-cycle memory-port outputs (all zero when idle). busy = tile_ctrl.busy;
        a pending start counts as busy too (STATUS.BUSY)."""
        m = self.m
        busy = busy or self.start_q
        region, word, lane = m.fields(addr)
        mapped = self._mapped(region, word, lane)
        out = dict(a_we=False, a_waddr=0, a_wlane_en=0, a_wdata=[0] * m.n,
                   b_we=False, b_waddr=0, b_wlane_en=0, b_wdata=[0] * m.n,
                   c_re=False, c_raddr=0)
        dmask = (1 << m.data_width) - 1
        for r, pre in ((REGION_A, "a"), (REGION_B, "b")):
            if we and mapped and region == r and not busy:
                out[f"{pre}_we"] = True
                out[f"{pre}_waddr"] = word
                out[f"{pre}_wlane_en"] = 1 << lane
                out[f"{pre}_wdata"] = [wdata & dmask] * m.n   # replicated; only the enabled lane is written
        if re and mapped and region == REGION_C:
            out["c_re"] = True
            out["c_raddr"] = word
        return out

    def status(self, busy, done=False, error=False):
        """BUSY: job running or a start pending. DONE / CMD_ERROR: sticky OR live,
        so the done cycle itself is already visible (no idle-looking gap)."""
        return ((STATUS_BUSY if busy or self.start_q else 0)
                | (STATUS_DONE if self.done_s or done else 0)
                | (STATUS_CMD_ERROR if self.cmd_error_s or (done and error) else 0)
                | (STATUS_HOST_ERROR if self.host_error_s else 0))

    def outputs(self, c_rdata=None):
        m = self.m
        rdata = None
        if self.read_q is not None:
            kind, v = self.read_q
            rdata = v if kind == "val" else (c_rdata[v] & ((1 << m.host_w) - 1))
        return dict(start=self.start_q, M=self.dim[REG_DIM_M], N=self.dim[REG_DIM_N],
                    K=self.dim[REG_DIM_K], host_rdata=rdata)

    # ------------------------------------------------------------------ #
    def step(self, we=False, re=False, addr=0, wdata=0, busy=False, done=False,
             error=False, c_rdata=None):
        m = self.m
        region, word, lane = m.fields(addr)
        mapped = self._mapped(region, word, lane)

        # read: sampled before this edge's write (read-first)
        if not re:
            self.read_q = None
        elif mapped and region == REGION_C:
            self.read_q = ("c", lane)
        elif mapped and region == REGION_REGS:
            v = {REG_CTRL: 0, REG_STATUS: self.status(busy, done, error)}.get(word, self.dim.get(word))
            self.read_q = ("val", v)
        else:
            self.read_q = ("val", 0)            # A/B are not readable; unmapped reads 0

        # a START write is taken only while not BUSY (job idle, no start pending); then
        # tile_ctrl is certain to accept it on the next edge, because nothing else can
        # make it busy in between. Taking it clears the sticky bits, winning over a
        # same-edge set (that done belongs to the previous job).
        busy_eff = busy or self.start_q
        reg_write = we and mapped and region == REGION_REGS
        start_taken = reg_write and word == REG_CTRL and bool(wdata & CTRL_START) and not busy_eff
        ab_write_busy = we and mapped and region in (REGION_A, REGION_B) and busy_eff
        if start_taken:
            self.done_s = self.cmd_error_s = self.host_error_s = False
        else:
            self.done_s |= done
            self.cmd_error_s |= done and error
            self.host_error_s |= ab_write_busy

        # register writes
        self.start_q = start_taken
        if reg_write and word in self.dim:
            w = {REG_DIM_M: m.m_w, REG_DIM_N: m.n_w, REG_DIM_K: m.k_w}[word]
            self.dim[word] = wdata & ((1 << w) - 1)
        return self.outputs(c_rdata)
