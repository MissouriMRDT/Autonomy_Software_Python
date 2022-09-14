#
# Mars Rover Design Team
# ar_tag.py
#
# Created on Oct 22, 2020
# Updated on Aug 21, 2022
#

import cv2
import logging
import itertools
import numpy as np

import core

from numpy.core.numeric import NaN
from collections import namedtuple

tag_cascade = cv2.CascadeClassifier("resources/tag_detection/cascade30.xml")

Tag = namedtuple("Tag", ["cX", "cY", "distance", "angle"])


def detect_ar_tag(reg_img):
    """
    Detects an AR Tag in the provided color image

    :param reg_img: color image to locate ar tags in
    :return: tags, reg_img
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

    :param center: the X, Y of the center pixels of the AR Tag
    :return: distance (meters), angle (left is negative, right is positive)
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
    coordinates = [0, 1, -1, 2, -2, 3, -3, 4, -4]
    perm = list(itertools.permutations(coordinates, 2))

    # Grab the distance from the depth map, iterating over pixels if the distance is not finite
    index = 0

    while not np.isfinite(distance) and index < len(perm):
        if index < len(perm):
            distance = core.vision.camera_handler.grab_depth_data()[cY + perm[index][1]][cX + perm[index][0]]
            index += 1

    # Vision system reports depth in mm, we want in meters
    distance /= 1000

    # Grab the camera parameters
    img_res_x, img_res_y = core.vision.camera_handler.get_depth_res()
    hfov = core.vision.camera_handler.get_hfov()

    # Calculate the angle of the object using camera params
    angle_per_pixel = hfov / img_res_x
    pixel_offset = cX - (img_res_x / 2)
    angle = pixel_offset * angle_per_pixel

    logger.info(f"Distance to marker: {distance} at pixel ({cX}, {cY})")
    logger.info(f"Angle to marker: {angle}")
    return distance, angle
