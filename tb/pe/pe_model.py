"""
Golden model for "pe" module

Intended behavior:
  - reset       -> all state 0
  - bubble      -> out_a/out_b/out_valid = 0, acc HOLDS
  - active      -> register operands, out_valid = 1, acc += a*b
"""

def to_signed(value: int, width: int) -> int:
    """Interpret the low 'width' bits of 'value' as two's complement; used to convert n-bit representations into 32-bit Python int"""
    value &= (1 << width) - 1
    sign_bit = 1 << (width - 1)
    return (value - (1 << width)) if (value & sign_bit) else value

def to_unsigned(value: int, width: int) -> int:
    """Interpret the low 'width' bits of 'value' as unsigned int; used to convert 32-bit Python int to n-bit representation"""
    return value & ((1 << width) - 1)

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

    def reset(self):
        self.out_a = 0
        self.out_b = 0
        self.acc = 0
        self.out_valid = 0

    def step(self, in_a: int, in_b: int, in_valid: int, rst_n: int = 1):
        if not rst_n:
            self.reset()
        elif in_valid:
            self.out_a = to_unsigned(in_a, self.data_width)
            self.out_b = to_unsigned(in_b, self.data_width)
            a = to_signed(in_a, self.data_width)
            b = to_signed(in_b, self.data_width)
            self.acc = (self.acc + a * b) & self.acc_mask
            self.out_valid = 1
        else:
            # bubble: operands drop to 0, valid drops, accumulator holds
            self.out_a = 0
            self.out_b = 0
            self.out_valid = 0

    @property
    def acc_signed(self) -> int:
        return to_signed(self.acc, self.acc_width)