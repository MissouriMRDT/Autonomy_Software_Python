#
# Mars Rover Design Team
# ar_tag.py
#
# Created on Oct 22, 2020
# Updated on Sep 19, 2022
#

import cv2
from cv2 import aruco
import logging
from numpy.core.numeric import NaN
import itertools
import core
import numpy as np
import interfaces
from core.constants import FRAMES_DETECTED


class Tag:
    def __init__(self, tag, gps, center):
        """
        Creates an object of Tag

        :param tag: the id that was found
        :param gps: the coordinates of the tag (lat, long)
        :param center: the center points of the tag
        :return: None
        """

        self.id = tag
        self.lat, self.long = gps
        self.cX, self.cY = center
        self.detected = 1
        self.empty = 0
        self.distance, self.angle = (NaN, NaN)

    def tag_spotted(self, gps):
        """
        Increments the detected variable and updates the GPS

        :param gps: the coordinates of the tag (lat, long)
        :return: None
        """

        self.detected += 1
        self.lat, self.long = gps

        if self.detected >= FRAMES_DETECTED:
            self.distance, self.angle = track_ar_tag((self.cX, self.cY))

    def tag_not_spotted(self):
        """
        Increments empty and checks if the frame is a blank frame

        :params: None
        :return: None
        """

        self.empty += 1
        # if the frame is blank clear it's empty count and detected count
        if self.empty >= FRAMES_DETECTED:
            self.empty = 0
            self.detected = 0

    def print(self):
        """
        Prints the id and the number of times it was detected
        """

        print(output.format(id=self.id, detected=self.detected))

    def is_same_tag(self, tag) -> bool:
        """
           Returns if the given tag and this tag are the same
        """
        if self.id == tag:
            return True
        else:
            return False


detected_tags = []
blank_frames = 0
output = "Ids: {id:02d}   |   Detected: {detected:02d}"


def get_gps():
    """
    Finds the GPS Coordinates for the Aruco Tag

    :return: latitude - the latitude of the tag
             longitude - the longitude of the tag
    """

    latitude = interfaces.nav_board.location()[0]
    longitude = interfaces.nav_board.location()[1]

    return latitude, longitude


def add_tag(tag, corner):
    """
    Creates a new Object of Tag in the detected_tags list

    :param tag: the id of the aruco tag
    :param corner: the four corner points of the aruco tag for distance calculations
    :return: None
    """

    latitude, longitude = get_gps()

    x1 = corner[3][0]  # top left x coord
    y1 = corner[3][1]  # top left y coord
    x2 = corner[2][0]  # top right x coord
    y2 = corner[0][1]  # bottom left y coord

    # Calculate the center points of the AR Tag
    cX = (x1 + x2) / 2
    cY = (y1 + y2) / 2

    detected_tags.append(Tag(tag, (latitude, longitude), (cX, cY)))


def detect_ar_tag(reg_img):
    """
    Detects an AR Tag in the provided color image.

    :param reg_img: the provided image we are looking at to find an ar tag
    :return: tags - a list of Tags (class) that contain the (id, gps, cX and cY) of the detected AR tags
             reg_img - the image with detected AR Tags drawn on top of it
    """
    # Frame Adjustments
    grayscale_img = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    parameters.markerBorderBits = 1
    parameters.errorCorrectionRate = 1

    # Capture Tags
    (corners, ids, rejectedImgPoints) = aruco.detectMarkers(grayscale_img, aruco_dict, parameters=parameters)

    if ids is not None:
        # Image with borders drawn around ArucoTags
        reg_img = aruco.drawDetectedMarkers(reg_img, corners)
        corners_stripped = corners[0][0].tolist()

        # Changes the list of ids from 2-dim to 1-dim
        tagIdsInFrame = []
        for i in ids:
            tagIdsInFrame.append(i[0])

        # Goes through each previously detected tag
        # and checks if it was spotted in the most recent frame
        for detected_tag in detected_tags:
            if detected_tag.id in tagIdsInFrame:
                i = tagIdsInFrame.index(detected_tag.id)
                tagIdsInFrame.pop(i)
                corners_stripped.pop(i)
                detected_tag.tag_spotted(get_gps())
            else:
                detected_tag.tag_not_spotted()

        # Add all tags to the detected_tags list that haven't been detected
        for i, tagIdInFrame in enumerate(tagIdsInFrame):
            add_tag(tagIdInFrame, corners_stripped)

    else:
        # No tags were identified in the frame
        for detected_tag in detected_tags:
            detected_tag.tag_not_spotted()
    return detected_tags, reg_img


def track_ar_tag(center):
    """
    Track the distance and angle of the AR Tag from the perspective of the Rover.
    :param center: the X, Y of the center pixels of the AR Tag
    :return: distance - the distance in meters to the AR Tag
             angle - the angle in degrees from the rover to the AR Tag. Left is negative, right is positive
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
