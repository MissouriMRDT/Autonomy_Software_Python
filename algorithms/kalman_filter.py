from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
import math

STATE_SIZE = 9
MEASUREMENT_SIZE = 3
DIMENSIONS = 3
ORDER_OF_MOTION = 3


class MovementHistory(object):
    def __init__(self, label):
        self.pos_hist = []
        self.vel_hist = []
        self.acc_hist = []

    def saveState(self, x):
        pass

    def savePos(self, pos):
        self.pos_hist.append(pos)

    def saveVel(self, vel):
        self.vel_hist.append(vel)

    def saveAcc(self, acc):
        self.acc_hist.append(acc)

    def displayDim(self, ax, dim, label):
        data = []
        if dim == 0:
            data = np.array(self.pos_hist)
        elif dim == 1:
            data = np.array(self.vel_hist)
        else:
            data = np.array(self.acc_hist)
        ax.plot(data[:, 0], data[:, 1], label=label)

    def saveDataOverTimePlot(self):
        pass

    def logData(self):
        pass


class ImuData:
    # orient: {roll, pitch, heading} - degrees
    # loc_acc: {ax, ay, az}
    def __init__(self, orient, loc_acc):
        self.roll = orient[0]
        self.pitch = orient[1]
        self.heading = orient[2]

        self.loc_ax = loc_acc[0]
        self.loc_ay = loc_acc[1]
        self.loc_az = loc_acc[2]

    def getGlobalAccelerations(self):
        def sinDeg(theta):
            return math.sin(math.radians(theta))
        
        def cosDeg(theta):
            return math.cos(math.radians(theta))

        glo_ax  = self.loc_ax * cosDeg(self.heading) * cosDeg(self.pitch)
        glo_ax += self.loc_ay * sinDeg(self.heading) * cosDeg(self.roll)
        glo_ax += self.loc_az * sinDeg(self.pitch) * cosDeg(self.roll)

        glo_ay  = self.loc_ax * sinDeg(self.heading) * cosDeg(self.pitch)
        glo_ay += self.loc_ay * cosDeg(self.heading) * cosDeg(self.roll)
        glo_ay += self.loc_az * cosDeg(self.pitch) * sinDeg(self.roll)

        glo_az  = self.loc_ax * cosDeg(self.heading) * sinDeg(self.pitch)
        glo_az += self.loc_ay * cosDeg(self.heading) * sinDeg(self.roll)
        glo_az += self.loc_az * cosDeg(self.pitch) * cosDeg(self.roll)

        return np.array([
            [glo_ax],
            [glo_ay],
            [glo_az]
        ])


# TEMPORARY COMMENTS
# Init_pos = [x,y,z]
# Uncertainty = [P,Q,R]
# - P is the estimate uncertainty
# - Q is the process noise uncertainty
# - R is the measurement uncertainty


class RoverKalmanFilter(KalmanFilter):
    def __init__(self, dt, init_pos=[0, 0, 0], uncertainty=[0., 0., 0.], save_history=False):
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

        self.uncertainty_scalars = uncertainty

        # TODO# BETTER EXPLANATION AND VALUEs
        self.setEstimateUncertainty(uncertainty[0])
        self.setProcessNoiseUncertainty(uncertainty[1])
        self.setMeasurementUncertainty(uncertainty[2])

        self.save_history = save_history
        if self.save_history:
            self.history = MovementHistory(self.getLabel())

    def update(self, imu_data: ImuData):
        self.predict()
        z = imu_data.getGlobalAccelerations()
        super().update(z)

        if self.save_history:
            self.history.saveState(self.x)

    def setEstimateUncertainty(self, uncertainty):
        self.uncertainty_scalars[0] = uncertainty
        self.P = np.eye(STATE_SIZE).astype(np.double) * uncertainty

    def setProcessNoiseUncertainty(self, uncertainty):
        self.uncertainty_scalars[1] = uncertainty
        self.Q = Q_discrete_white_noise(3, block_size=DIMENSIONS, order_by_dim=False, var=uncertainty)

    def setMeasurementUncertainty(self, uncertainty):
        self.uncertainty_scalars[2] = uncertainty
        self.R = self.R = np.eye(DIMENSIONS).astype(np.double) * uncertainty

    def getEstimatedPos(self):
        return self._getDimState(0)

    def getEstimatedVel(self):
        return self._getDimState(1)

    def getEstimatedAcc(self):
        return self._getDimState(2)

    def getLabel(self):
        letters = ['P', 'Q', 'R']
        label = ""
        for letter, val in zip(letters, self.uncertainty_scalars):
            label += letter + ": " + str(val) + ' '
        return label

    def displayState(self):
        print("STATE")
        print("X,  Y,  Z : ", self._getDimState(0, as_list=True))
        print("Vx, Vy, Vz: ", self._getDimState(1, as_list=True))
        print("Ax, Ay, Az: ", self._getDimState(2, as_list=True))

    def _getDimState(self, order, as_list=False):
        state = self.x[order * DIMENSIONS: (order + 1) * DIMENSIONS].reshape(1, DIMENSIONS)[0]
        return list(state) if as_list else state
