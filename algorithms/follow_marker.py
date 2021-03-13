import interfaces
import logging
import algorithms.heading_hold as hh


def drive_through_gate():
    # TODO: Will attempt to drive through a gate composed of two AR Tags
    pass


def drive_to_marker(speed, distance, angle):
    """
    Calculates the angle and distance of the AR marker with given center pixels.
    Then returns drive speeds necessary to stay on course towards it.

    Parameters:
    -----------
        speed - the desired drive speed
        center - the center pixels of the AR Tag detected (x, y)

    Returns:
    -----------
        left - the desired speed for left motors
        right - the desired speed for right motors
        distance - the distance the AR marker is away from the rover
    """
    logger = logging.getLogger(__name__)

    goal_heading = interfaces.nav_board.heading() + angle
    left, right = hh.get_motor_power_from_heading(speed, goal_heading)

    logger.debug(f"Driving at: {left}, {right} to marker")
    return (left, right), distance
