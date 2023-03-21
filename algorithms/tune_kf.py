# THIS IS A TEMPORY FILE MEANT SOLELY FOR CALIBRATING THE KALMAN FILTERS
import kalman_filter

TIMESTEP = 0.1


class RKFManager(object):
    def __init__(self, init_pos=[0, 0, 0], uncertainties=[0, 0, 0]):
        self.rkf = kalman_filter.RoverKalmanFilter(TIMESTEP, init_pos, uncertainties, save_history=True)


class MotionData:
    def __init__(self, orient, local_accel, gps):
        self.orientation = orient
        self.local_accel = local_accel
        self.gps = gps


P = [100, 1000, 10000]
Q = [0.01, 0.1, 0.1]
R = [0.01, 0.01, 0.05]

rkf_managers = []

loc_history = kalman_filter.MovementHistory("Truth")


def initialize_filters(init_pos):
    global rkf_managers
    for uncertainties in zip(P, Q, R):
        rkf_manager = RKFManager(init_pos, uncertainties)
        rkf_managers.append(rkf_manager)


def send_data(mot_data: MotionData):
    imu_data = kalman_filter.ImuData(mot_data.orientation, mot_data.local_accel)

    loc_history.savePos(mot_data.gps)

    for rkf_manager in rkf_managers:
        rkf_manager.rkf.update(imu_data)
