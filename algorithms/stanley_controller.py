#
# Mars Rover Design Team
# stanley_controller.py
#
# Created on March 13, 2023
# Updated on March 13, 2023
#
"""
Path tracking simulation with Stanley steering control and PID speed control.

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
import numpy as np
import matplotlib.pyplot as plt
import time

k = 0.5  # control gain
Kp = 1.0  # speed proportional gain
L = 1.0  # [m] Wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle


class State(object):
    """
    Class representing the state of a vehicle. Uses a basic bicycle model to estimate the
    vehicle position given an acceleration and steering angle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        # Initialize class variables.
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.time_since_last_step = time.time()

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        # Constrain the steering angle to the given limits.
        delta = np.clip(delta, -max_steer, max_steer)

        # Calculate dt.
        dt = time.time() - self.time_since_last_step

        # Calculate estimated position using bicycle model.
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt

        # Update time step.
        self.time_since_last_step = time.time()


def pid_control(target, current):
    """
    Simple proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    # Multiply error py proportional constant.
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    # Find the closest index in the path to where the robot.
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    # Don't go over end of path.
    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # Theta_e corrects the heading error.
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # Theta_d corrects the cross track error.
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control.
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float) Must be in radians.
    :return: (float) Angle in radian in [-pi, pi]
    """
    # Clamp angle
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position.
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index.
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector.
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle
