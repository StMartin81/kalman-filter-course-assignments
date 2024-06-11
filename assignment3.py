import math
import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options["FIG_SIZE"] = [8, 8]

options["DRIVE_IN_CIRCLE"] = True
# If False, measurements will be x,y.
# If True, measurements will be x,y, and current angle of the car.
# Required if you want to pass the driving in circle.
options["MEASURE_ANGLE"] = True
options["RECIEVE_INPUTS"] = False


class KalmanFilter:
    def __init__(self):
        # Initial State
        self.x = np.matrix([[0.0], [0.0], [0.0], [0.0],])

        # Uncertainity Matrix
        self.P = np.matrix(
            [
                [1000.0, 0.0, 0.0, 0.0],
                [0.0, 1000.0, 0.0, 0.0],
                [0.0, 0.0, 1000.0, 0.0],
                [0.0, 0.0, 0.0, 1000.0],
            ]
        )

        # Next State Function
        self.F = np.matrix(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        # Measurement Function
        self.H = np.matrix(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        # Measurement Uncertainty
        self.R = np.matrix(
            [
                [0.1, 0.0, 0.0],
                [0.0, 0.1, 0.0],
                [0.0, 0.0, 0.1],
            ]
        )

        # Identity Matrix
        self.I = np.matrix(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    def predict(self, dt):
        self.F[0, 2] = dt * math.cos(self.x[3])
        self.F[1, 3] = dt * math.sin(self.x[3])
        self.x = self.F * self.x
        self.P = self.F * self.P * np.transpose(self.F)
        return

    def measure_and_update(self, measurements, dt):
        Z = np.matrix(measurements) # [x, y, omega]
        y = np.transpose(Z) - self.H * self.x # Abweichung zum gemessenen Wert
        S = self.H * self.P * np.transpose(self.H) + self.R
        K = self.P * np.transpose(self.H) * np.linalg.inv(S)
        self.x = self.x + K * y
        self.P = (self.I - K * self.H) * self.P
        self.P += 0.03 * self.I
        return [self.x[0], self.x[1]]

    def recieve_inputs(self, u_steer, u_pedal):
        return


sim_run(options, KalmanFilter)
