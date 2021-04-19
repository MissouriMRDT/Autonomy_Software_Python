from collections import namedtuple
from math import dist
import cv2
import logging
from numpy.core.numeric import NaN
import itertools
import core
import numpy as np

tag_cascade = cv2.CascadeClassifier("resources/tag_detection/cascade30.xml")

Tag = namedtuple("Tag", ["cX", "cY", "distance", "angle"])


def detect_ar_tag(reg_img):
    """
    Detects an AR Tag in the provided color image.

    Parameters:
    -----------
        reg_img - the provided image we are looking at to find an ar tag

    Returns:
    --------
        tags - a list of Tags (named tuple) that contain the (cX, cY, distance, angle) of the detected AR tags
        reg_img - the image with detected AR Tags drawn on top of it
    """

    gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)
    tags_coords = tag_cascade.detectMultiScale(gray, 1.3, 5)

    tags = []
    for (x, y, w, h) in tags_coords:
        reg_img = cv2.rectangle(reg_img, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Calculate the center points of the AR Tag
        cX = x + (w / 2)
        cY = y + (h / 2)

        # Find the distance/angle of said center pixels
        distance, angle = track_ar_tag((cX, cY))
        tags.append(Tag(cX, cY, distance, angle))

    return tags, reg_img


def track_ar_tag(center):
    """
    Track the distance and angle of the AR Tag from the perspective of the Rover.

    Parameters:
    -----------
        center - the X, Y of the center pixels of the AR Tag

    Returns:
    --------
        distance - the distance in meters to the AR Tag
        angle - the angle in degrees from the rover to the AR Tag. Left is negative,
        right is positive
    """

    logger = logging.getLogger(__name__)

    # Center coordinates
    cX, cY = center

    # Depth image is at half resolution
    cX = int(cX / 2)
    cY = int(cY / 2)

    # Grab the distance from the depth map
    distance = NaN

    # Find some permutations we can use in case of noisy data
    coordinates = [0, 1, -1, 2, -2]
    perm = list(itertools.permutations(coordinates, 2))

    # Grab the distance from the depth map, iterating over pixels if the distance is not finite
    index = 0

    while not np.isfinite(distance):
        if index < len(perm):
            distance = core.vision.camera_handler.grab_depth_data()[cY + perm[index][1]][cX + perm[index][0]]
            index += 1
        else:
            index = 0

    # Grab the distance from the depth map
    if type(core.vision.camera_handler).__name__ == "ZedHandler":
        # ZED units are currently in millimeters
        distance /= 1000

    # H FOV = 85
    img_res_x, img_res_y = core.vision.camera_handler.get_depth_res()

    angle_per_pixel = 85 / img_res_x
    pixel_offset = cX - (img_res_x / 2)
    angle = pixel_offset * angle_per_pixel

    logger.info(f"Distance to marker: {distance}")
    logger.info(f"Angle to marker: {angle}")
    return distance, angle
