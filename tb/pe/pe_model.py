"""
Golden model for "pe" module

Intended behavior:
  - reset       -> all state 0
  - bubble      -> out_a/out_b/out_valid = 0, acc HOLDS
  - active      -> register operands, out_valid = 1, acc += a*b
"""

from utils import to_signed, to_unsigned

class PEModel:
    """
    Call `step()` once per rising clock edge with the inputs that are present
    before that edge. It returns the outputs that become visible after the
    edge — matching the DUT's one-cycle register latency.
    """

    def __init__(self, data_width: int = 8, acc_width: int = 32):
        self.data_width = data_width
        self.acc_width = acc_width
        self.acc_mask = (1 << acc_width) - 1
        # Registered outputs (post-edge view)
        self.out_a = 0
        self.out_b = 0
        self.acc = 0
        self.out_valid = 0
        self.out_first = 0

    def reset(self):
        self.out_a = 0
        self.out_b = 0
        self.acc = 0
        self.out_valid = 0
        self.out_first = 0

    def step(self, in_a: int, in_b: int, in_valid: int, in_first: int = 0, rst_n: int = 1):
        if not rst_n:
            self.reset()
        elif in_valid:
            self.out_a = to_unsigned(in_a, self.data_width)
            self.out_b = to_unsigned(in_b, self.data_width)
            a = to_signed(in_a, self.data_width)
            b = to_signed(in_b, self.data_width)
            mult_result = a * b
            self.acc = (mult_result & self.acc_mask) if in_first else ((self.acc + mult_result) & self.acc_mask)
            self.out_valid = 1
            self.out_first = in_first
        else:
            # bubble: operands drop to 0, valid drops, accumulator holds
            self.out_a = 0
            self.out_b = 0
            self.out_valid = 0
            self.out_first = 0

    @property
    def acc_signed(self) -> int:
        return to_signed(self.acc, self.acc_width)