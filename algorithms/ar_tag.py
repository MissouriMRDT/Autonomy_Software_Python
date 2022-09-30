#
# Mars Rover Design Team
# ar_tag.py
#
# Created on Oct 22, 2020
# Updated on Aug 21, 2022
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
        self.distance, self.angle = NaN

    def check_tag(self, tag, gps):
        """
        Checks to see if the tag that was spotted has already been added to the array.

        :param tag: the id that is being checked
        :param gps: the coordinates of the tag (lat, long)
        :return: bool - True if detected, False if not detected
        """

        if self.id == tag:
            self.tag_spotted(gps)
            return True
        else:
            return False

    def tag_spotted(self, gps):
        """
        Increments the detected variable and updates GPS

        :param gps: the coordinates of the tag (lat, long)
        :return: None
        """

        self.detected += 1
        self.lat, self.long = gps

        if self.detected >= FRAMES_DETECTED:
            self.distance, self.angle = track_ar_tag((self.cX, self.cY))

    def reset_spotted(self):
        """
        Resets the tags detected counter
        """

        self.detected = 0

    def print(self):
        """
        Prints the id and the number of times it was detected
        """

        print(output.format(id=self.id, detected=self.detected))


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

    x1 = corner[0][0][0][0]  # top left x coord
    y1 = corner[0][0][0][1]  # top left y coord
    x2 = corner[0][0][1][0]  # top right x coord
    y2 = corner[0][0][3][1]  # bottom left y coord

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
    gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    parameters.markerBoarderBits = 1
    parameters.errorCorrectionRate = 1

    # Capture Tags
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Add Tags to Tag Class Object
    if ids is not None:
        reg_img = aruco.drawDetectedMarkers(reg_img, corners)

        # Create a list of all ids seen in frame
        frameTags = []
        for i in ids:
            for j in i:
                frameTags.append(j)

        # Loops through ids found and checks adds them to the list accordingly.
        for i in ids:
            for j in i:
                if len(detected_tags) == 0:
                    add_tag(j, corners)
                else:
                    found = False
                    for t in detected_tags:
                        if t.checkTag(j, get_gps()):
                            found = True
                    if not found:
                        add_tag(j, corners)

        # Loops through ids found on current leg and checks if they were in current frame.
        for t in detected_tags:
            if t.id not in frameTags:
                t.empty += 1

    # Checks for Blank Frames and resets counter if >= FRAMES_DETECTED
    else:
        for t in detected_tags:
            t.empty += 1
            # if t.empty >= FRAMES_DETECTED:
            #     t.resetSpotted()

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
