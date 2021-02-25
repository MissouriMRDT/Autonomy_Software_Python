from numpy.core.numeric import NaN
import interfaces
import math
import logging
import core
import algorithms.heading_hold as hh
import itertools
import numpy as np


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

    # Grab the distance from the depth map
    distance = NaN  # core.vision.camera_handler.grab_depth_data()[cY][cX]

    # Find some permutations we can use in case of noisy data
    coordinates = [0, 1, -1]
    perm = list(itertools.permutations(coordinates, 2))

    # Grab the distance from the depth map, iterating over pixels if the distance is not finite
    index = 0
    
    while (not np.isfinite(distance)):
        if index < len(perm):
            distance = core.vision.camera_handler.grab_depth_data()[cY + perm[index][1]][cX + perm[index][0]]
            index += 1
        else:
            index = 0


    #print(index)
    #print(np.isfinite(distance))

    # Grab the distance from the depth map
    if type(core.vision.camera_handler).__name__ == "ZedHandler":
        # ZED units are currently in millimeters
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
