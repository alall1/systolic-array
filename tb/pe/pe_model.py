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
        self.out_a_valid = 0
        self.out_b_valid = 0
        self.out_first = 0
        self.out_shadow = 0

    def reset(self):
        self.acc = 0
        self.out_a_valid = 0
        self.out_b_valid = 0
        self.out_first = 0
        self.out_shadow = 0

    def step(self, in_a: int, in_b: int, in_a_valid: int, in_b_valid: int, in_first: int = 0, capture: int = 0, shift_en: int = 0, in_shadow: int = 0, rst_n: int = 1):
        if not rst_n:
            self.reset()
        else:
            if capture:
                self.out_shadow = self.acc & self.acc_mask
            elif shift_en:
                self.out_shadow = in_shadow & self.acc_mask

            self.out_a = to_signed(in_a, self.data_width)
            self.out_b = to_signed(in_b, self.data_width)
            a = to_signed(in_a, self.data_width)
            b = to_signed(in_b, self.data_width)
                            
            self.out_a_valid = in_a_valid
            self.out_b_valid = in_b_valid
            self.out_first = in_first      

            mult_result = a * b

            if in_a_valid and in_b_valid:
                self.acc = (mult_result & self.acc_mask) if in_first else ((self.acc + mult_result) & self.acc_mask)         

    @property
    def acc_signed(self) -> int:
        return to_signed(self.acc, self.acc_width)

    @property
    def out_shadow_signed(self) -> int:
        return to_signed(self.out_shadow, self.acc_width)