from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

STATE_SIZE = 9
MEASUREMENT_SIZE = 3
DIMENSIONS = 3

# TEMPORARY COMMENTS
# Init_pos = [x,y,z]
# Uncertainty = [P,Q,R]
# - P is the estimate uncertainty
# - Q is the process noise uncertainty
# - R is the measurement uncertainty


class RoverKalmanFilter(KalmanFilter):
    def __init__(self, dt, init_pos=[0, 0, 0], uncertainty=[0., 0., 0.]):
        super().__init__(dim_x=STATE_SIZE, dim_z=MEASUREMENT_SIZE)

        # Initial State
        self.x = np.array(init_pos).reshape(3, -1)
        self.x = np.pad(self.x, [(0, 6), (0, 0)])

        # State Transition Matrix
        first_order = np.diag([1] * 9).astype(np.double)
        second_order = np.diag([1] * 6).astype(np.double) * dt
        second_order = np.pad(second_order, [(0, 3), (3, 0)])
        third_order = np.diag([1] * 3).astype(np.double) * 0.5 * dt**2
        third_order = np.pad(third_order, [(0, 6), (6, 0)])
        self.F = first_order + second_order + third_order

        # Observation Matrix
        obs_matrix = np.eye(3).astype(np.double)
        obs_matrix = np.pad(obs_matrix, [(0, 0), (6, 0)])
        self.H = obs_matrix

        # TODO# BETTER EXPLANATION AND VALUEs
        self.setEstimateUncertainty(uncertainty[0])
        self.setProcessNoiseUncertainty(uncertainty[1])
        self.setMeasurementUncertainty(uncertainty[2])

    def setEstimateUncertainty(self, uncertainty):
        self.P = np.eye(STATE_SIZE).astype(np.double) * uncertainty

    def setProcessNoiseUncertainty(self, uncertainty):
        self.Q = Q_discrete_white_noise(3, block_size=DIMENSIONS, order_by_dim=False, var=uncertainty)

    def setMeasurementUncertainty(self, uncertainty):
        self.R = self.R = np.eye(DIMENSIONS).astype(np.double) * uncertainty

    def getEstimatedPos(self):
        return self.x[0:2].reshape(1, 2)[0]

    def getEstimatedVel(self):
        return self.x[3:5].reshape(1, 2)[0]

    def getEstimatedAcc(self):
        return self.x[6:8].reshape(1, 2)[0]

    def displayState(self):
        print("STATE")
        print("X,Y: ", list(self.x[0:2].reshape(1, 2))[0])
        print("Vx, Vy: ", list(self.x[3:5].reshape(1, 2))[0])
        print("Ax, Ay: ", list(self.x[6:8].reshape(1, 2))[0])
