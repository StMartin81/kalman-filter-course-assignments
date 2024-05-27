import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options["FIG_SIZE"] = [8, 8]
options["CONSTANT_SPEED"] = True
weight = 0.5


class KalmanFilterToy:
    def __init__(self):
        self.v = 0
        self.prev_x = 0
        self.prev_t = 0

    def predict(self, t):
        prediction = self.prev_x + (self.v * (t - self.prev_t))
        return prediction

    def measure_and_update(self, x, t):
        v = (x - self.prev_x) / (t - self.prev_t)
        self.v += (v - self.v) * weight
        self.prev_x = x
        self.prev_t = t
        return


sim_run(options, KalmanFilterToy)
