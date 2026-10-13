class SkewBufferModel:
    """
    Cycle-accurate model of skew_buffer, two lanes (A and B).

    Row i output = input fed i steps before the freshest; latency i+1.
    """

    def __init__(self, array_dim):
        self.array_dim = array_dim
        # sr[i][d] = lane row i's value d shifts ago. [0] is freshest.
        self.a = [[0] * array_dim for _ in range(array_dim)]
        self.b = [[0] * array_dim for _ in range(array_dim)]

    def reset(self):
        self.a = [[0] * self.array_dim for _ in range(self.array_dim)]
        self.b = [[0] * self.array_dim for _ in range(self.array_dim)]

    def out(self):
        a_out = [self.a[i][i] for i in range(self.array_dim)]
        b_out = [self.b[i][i] for i in range(self.array_dim)]
        return a_out, b_out

    def step(self, a_vec, b_vec):
        """One clock edge. a_vec, b_vec: lists of n ints. Returns (a_out, b_out), each a list of n ints, post-edge."""
        assert len(a_vec) == self.array_dim and len(b_vec) == self.array_dim

        for lane, vec in ((self.a, a_vec), (self.b, b_vec)):
            for i in range(self.array_dim):
                for d in range(self.array_dim - 1, 0, -1):
                    lane[i][d] = lane[i][d - 1]
                lane[i][0] = vec[i]

        return self.out()

# testing model
# x = 5
# a = [1, 1, 1, 1, 1]
# b = [1, 1, 1, 1, 1]
# zero = [0, 0, 0, 0, 0]
# model = SkewBufferModel(x)
# model.reset()
# print(model.step(a, b))
# print(model.step(zero, zero))
# print(model.step(zero, zero))
# print(model.step(zero, zero))
# print(model.step(zero, zero))
