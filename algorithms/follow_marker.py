#
# Mars Rover Design Team
# ar_tag.py
#
# Created on Aug 29, 2020
# Updated on Aug 21, 2022
#

import logging

import interfaces
import algorithms.heading_hold as hh


def drive_through_gate():
    """
    Drive through a gate composed of two tags

    :return: None
    """

    # TODO: Will attempt to drive through a gate composed of two AR Tags
    pass


def drive_to_marker(speed, angle):
    """
    Returns drive speeds necessary to stay on course towards a single marker

    :param speed: the desired drive speed
    :param angle: the angle towards the given AR marker
    :return: speed of left and right motors (left, right)
    """

    logger = logging.getLogger(__name__)

    goal_heading = interfaces.nav_board.heading() + angle
    left, right = hh.get_motor_power_from_heading(speed, goal_heading)

    logger.debug(f"Driving at: {left}, {right} to marker")
    return left, right
