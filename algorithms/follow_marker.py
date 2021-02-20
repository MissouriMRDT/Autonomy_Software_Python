import interfaces
import math
import logging
import core
import interfaces
import algorithms.heading_hold as hh


def drive_through_gate():
    # TODO: Will attempt to drive through a gate composed of two AR Tags
    pass


def drive_to_marker(speed, center):
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

    # Center coordinates
    cX, cY = center
    
    # Depth image is at half resolution
    cX = int(cX / 2)
    cY = int(cY / 2)

    print(cX, cY)
    
    # Grab the distance from the depth map
    distance = core.vision.camera_handler.grab_depth_data()[cY][cX]
    distance /= 1000
    
    # H FOV = 85, WIDTH = 640
    angle_per_pixel = 85 / 640
    angle = (cX - (640 / 2)) * angle_per_pixel

    logger.info(f"Distance to marker: {distance}")
    logger.info(f"Angle to marker: {angle}")

    goal_heading = interfaces.nav_board.heading() + angle
    left, right = hh.get_motor_power_from_heading(speed, goal_heading)

    logger.debug(f"Driving at: {left}, {right} to marker")
    return (left, right), distance
