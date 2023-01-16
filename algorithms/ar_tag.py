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
import interfaces
from core.constants import FRAMES_DETECTED

import core

from numpy.core.numeric import NaN
from collections import namedtuple

tag_cascade = cv2.CascadeClassifier("resources/tag_detection/cascade30.xml")

class Tag:
    def __init__(self, tag, gps, center):
        """
        Creates an object of Tag
        Parameters:
        -----------
            tag - the id that was found
            gps - the coordinates of the tag (lat, long)
            corner - the four corner points of the aruco tag for distance calculations
        Returns:
        --------
            None
        """

        self.id = tag
        self.lat, self.long = gps
        self.cX, self.cY = center
        self.detected = 1
        self.empty = 0
        self.distance = 0
        self.angle = 0

    def check_tag(self, tag, gps, corner, index):
        """
        Checks to see if the tag that was spotted has already been added to the array.
        Parameters:
        -----------
            tag - the id that is being checked
            gps - the coordinates of the tag (lat, long)
            corner - the four corner points of the aruco tag for distance calculations
            index - the index in the tag list
        Returns:
        --------
            bool - True if detected, False if not detected
        """

        if self.id == tag:
            self.tag_spotted(gps, corner, index)
            return True
        else:
            return False

    def tag_spotted(self, gps, corner, index):
        """
        Increments the detected variable and updates GPS
        Parameters:
        -----------
            gps - the coordinates of the tag (lat, long)
            corner - the four corner points of the aruco tag for distance calculations
        Returns:
        --------
            None
        """
        self.detected += 1
        self.lat, self.long = gps

        x1 = corner[index][0][0][0]  # top left x coord
        y1 = corner[index][0][0][1]  # top left y coord
        x2 = corner[index][0][1][0]  # top right x coord
        y2 = corner[index][0][3][1]  # bottom left y coord

        # Calculate the center points of the AR Tag
        self.cX = (x1 + x2) / 2
        self.cY = (y1 + y2) / 2

        if self.detected >= FRAMES_DETECTED:
            self.distance, self.angle = track_ar_tag((self.cX, self.cY))

    def reset_spotted(self):
        """
        Resets the tags detected counter
        Parameters:
        -----------
            None
        Returns:
        --------
            None
        """

        self.detected = 0

    def print(self):
        """
        Prints the id and the number of times it was detected
        Parameters:
        -----------
            None
        Returns:
        --------
            None
        """

        print(output.format(id=self.id, detected=self.detected))


detected_tags = []
blank_frames = 0
output = "Ids: {id:02d}   |   Detected: {detected:02d}"


def get_gps():
    """
    Finds the GPS Coordinates for the Aruco Tag
    Parameters:
    -----------
        None
    Returns:
    --------
        latitude - the latitude of the tag
        longitude - the longitude of the tag
    """

    latitude = interfaces.nav_board.location()[0]
    longitude = interfaces.nav_board.location()[1]

    return latitude, longitude


def add_tag(tag, corner, index):
    """
    Creates a new Object of Tag in the detected_tags list
    Parameters:
    -----------
        tag - the id of the aruco tag
        corner - the four corner points of the aruco tag for distance calculations
        index - the index in the tag list
    Returns:
    --------
        None
    """

    latitude, longitude = get_gps()

    x1 = corner[index][0][0][0]  # top left x coord
    y1 = corner[index][0][0][1]  # top left y coord
    x2 = corner[index][0][1][0]  # top right x coord
    y2 = corner[index][0][3][1]  # bottom left y coord

    # Calculate the center points of the AR Tag
    cX = (x1 + x2) / 2
    cY = (y1 + y2) / 2

    if tag <= 5:
        detected_tags.append(Tag(tag, (latitude, longitude), (cX, cY)))


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
    # Center coordinates
    cX, cY = center

    # Grab the camera parameters
    img_res_x, img_res_y = core.vision.camera_handler.get_reg_res()

    # Scale ar tag value between image resolutions.
    depth_cX = int((cX * (core.vision.camera_handler.depth_res_x)) / (img_res_x))
    depth_cY = int((cY * (core.vision.camera_handler.depth_res_y)) / (img_res_y))

    # Grab the distance from the depth map
    distance = NaN

    # Find some permutations we can use in case of noisy data
    coordinates = [0, 1, -1, 2, -2, 3, -3, 4, -4]
    perm = list(itertools.permutations(coordinates, 2))

    # Grab the distance from the depth map, iterating over pixels if the distance is not finite
    index = 0

    while not np.isfinite(distance) and index < len(perm):
        if index < len(perm):
            distance = core.vision.camera_handler.grab_depth_data()[depth_cY + perm[index][1]][
                depth_cX + perm[index][0]
            ]
            # If distance is equal to or greater than 40000 (max zed and sim range), then set distance to NaN.
            if distance >= 40000:
                distance = NaN
            index += 1

    # Vision system reports depth in mm, we want in meters
    distance /= 1000

    hfov = core.vision.camera_handler.get_hfov()

    # Calculate the angle of the object using camera params
    angle_per_pixel = hfov / img_res_x
    pixel_offset = cX - (img_res_x / 2)
    angle = pixel_offset * angle_per_pixel

    return distance, angle
