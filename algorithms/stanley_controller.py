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
import math

k = 0.6  # control gain
Kp = 0.1  # speed proportional gain
L = 1.0  # [m] Wheel base of vehicle
max_steer = np.radians(10.0)  # [rad] max steering angle


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

    def update_bicycle(self, acceleration, delta):
        """
        Update the state of the vehicle with a simple bicycle model. This is used to guess the position
        of the robot given acceleration and heading change and will likely not be accurate at all to real life.
        Only use this for testing and simulating outputs.

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

    def update(self, x, y, yaw):
        """
        Manually update the position and heading of the robot. Velocity is automatically
        determined using the time and distance covered since last update.

        :param x: (float) The x position of the robot.
        :param y: (float) The y position of the robot.
        :param yaw: (float) The heading of the robot.
        """
        # Calcluate the velocity of the robot.
        self.v = math.sqrt(math.pow((x - self.x), 2) + math.pow((y - self.y), 2)) / (
            time.time() - self.time_since_last_step
        )

        # Store the new x, y, and yaw positions.
        self.x = x
        self.y = y
        self.yaw = yaw

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
    :param cyaw: ([float]) Expects yaw to oriented like a unit circle.
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
    theta_d = math.atan2(k * error_front_axle, state.v)
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


def calculate_yaws_from_path(cx, cy, start_angle=0.0, radians=True):
    """
    Computes the appropiate absolute yaw from a given set of Xs and Ys. These should
    produce a sensible path and the two lists should be the same length.
    All yaw angle are calulated in radians by default.

    :param cx: The list of x points within the path.
    :param cy: The list of y points within the path.
    :param start_angle: The initial angle for the first point.
    :param radians: Whether of not to use radians for the angle. (On by default)
    :return: ([yaws]) An array containing the yaw angles for each point.
    """
    # Create instance variables.
    yaws = []

    # Check if both lists are equal size.
    if len(cx) == len(cy):
        # Loop through points. The zip function returns iterables for a sublist starting at 0:-1 and a sublist starting at 1:end for each list x and y.
        for i, x, y, x_next, y_next in zip(range(len(cx) - 1), cx[:-1], cy[:-1], cx[1:], cy[1:]):
            # Basic trig to find angle between two points.
            angle = math.atan2((y_next - y), (x_next - x))

            # Check if we are converting to degrees.
            if not radians:
                angle = np.rad2deg(angle)

            # Append angle to yaws list.
            yaws.append(angle)

        # Copy second to last angle to last point since the last point doesn't have a point after it to find angle from.
        yaws.append(yaws[-1])

    return yaws
