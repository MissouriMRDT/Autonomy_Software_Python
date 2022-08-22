#
# Mars Rover Design Team
# heading_hold.py
#
# Created on Oct 20, 2018
# Updated on Aug 21, 2022
#

import logging

import interfaces

from algorithms.clamp import clamp

from algorithms.pid_controller import PIDcontroller

pid = PIDcontroller(Kp=3, Ki=0.25, Kd=0, wraparound=360)


def get_motor_power_from_heading(speed, goal_heading):
    """
    Derives motor power for (left, right) from the goal heading
    Uses a PID loop to adjust the turn rate to match goal heading

    :param speed: speed to driver rover in -1000, 1000
    :param goal_heading: target heading (degrees) for the rover to drive in
    :return: left and right motor speeds for rover in range of -1000 to 1000 (left, right)
    """

    logger = logging.getLogger(__name__)

    heading_correction = pid.update(goal_heading, interfaces.nav_board.heading())
    heading_correction = clamp(heading_correction, -180, 180)
    logger.debug(f"Heading: {heading_correction}, Speed: {speed}")
    return interfaces.drive_board.calculate_move(speed, heading_correction)
