from math import dist
from algorithms import obstacle_detector
import core
import algorithms
import cv2
import asyncio
import sys

this = sys.modules[__name__]
this.detect = False
this.angle = None
this.distance = None


async def async_obstacle_detector():
    """
    Async function to find obstacles
    """
    while True:
        reg_img = core.vision.camera_handler.grab_regular()
        depth_matrix = core.vision.camera_handler.grab_depth_data()

        mask, lower = algorithms.obstacle_detector.get_floor_mask(
            reg_img, int(reg_img.shape[1] / 2), int(reg_img.shape[0] / 2)
        )

        depth_matrix = cv2.bitwise_and(depth_matrix, depth_matrix, mask=mask)

        obstacle = algorithms.obstacle_detector.detect_obstacle(depth_matrix, 1, 4)
        reg_img = cv2.resize(reg_img, (int(1280 / 2), int(720 / 2)))

        if obstacle != []:
            # Track the obstacle in the depth matrix
            angle, distance, _ = algorithms.obstacle_detector.track_obstacle(depth_matrix, obstacle, False)
            # Update the current obstacle info
            this.detect = True
            this.angle = angle
            this.distance = distance
        else:
            # Update the current obstacle info
            this.detect = False
            this.angle = None
            this.distance = None

        await asyncio.sleep(1 / 30)


def is_obstacle():
    """
    Returns whether there is an obstacle

    Returns:
    -------------
        detect (bool) - whether or not something was detected
    """
    return this.detect


def get_angle():
    return this.angle


def get_distance():
    return this.distance
