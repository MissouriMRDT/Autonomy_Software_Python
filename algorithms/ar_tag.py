#
# Created on Oct 22, 2020
# Updated on Dec 1, 2022
#

import cv2
import core
import logging
import interfaces
import itertools
import numpy as np
import core.constants
from cv2 import aruco
from numpy.core.numeric import NaN
from typing import Tuple, List


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
        self.distance, self.angle = (NaN, NaN)

    def tag_spotted(self, gps, detected_center):
        """
        Increments the detected variable and updates the GPS
        :param gps: the coordinates of the tag (lat, long)
        :return: None
        """

        self.detected += 1
        self.lat, self.long = gps
        self.cX, self.cY = detected_center

        if self.detected >= core.constants.ARUCO_FRAMES_DETECTED:
            self.distance, self.angle = track_ar_tag((self.cX, self.cY))

    def print(self):
        """
        Prints the id and the number of times it was detected
        """

        print(output.format(id=self.id, detected=self.detected))


class ArucoTagCorners:
    """
    This class stores the four corners for each detected
    aruco tag
    """

    Coordinates = Tuple[float, float]

    def __init__(self, corners):
        self.corners = corners

    # The Aruco Tag Corners are sent in a counter-clockwise
    # fashion such that it starts in the bottom-left and
    # works its way to the top left
    def bottom_left(self) -> Coordinates:
        return tuple(self.corners[0])

    def bottom_right(self) -> Coordinates:
        return tuple(self.corners[1])

    def top_right(self) -> Coordinates:
        return tuple(self.corners[2])

    def top_left(self) -> Coordinates:
        return tuple(self.corners[3])

    def location_of_center(self) -> Coordinates:
        cX = (self.top_left()[0] + self.top_right()[0]) / 2
        cY = (self.top_left()[1] + self.bottom_left()[1]) / 2
        return cX, cY

    def __str__(self):
        return f"BL: {self.bottom_left()}\nBR: {self.bottom_right()}\n" f"TR: {self.top_right()}\nTL: {self.top_left()}"

    def __repr__(self):
        return self.__str__()


class ArucoTagDetector(object):
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters()
        self.params.markerBorderBits = core.constants.ARUCO_MARKER_BORDER_BITS
        self.params.errorCorrectionRate = core.constants.ARUCO_ERROR_CORRECTION_RATE
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.params)

    def detectMarkers(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        return corners, ids


tag_detector = ArucoTagDetector()
detected_tags = []
blank_frames = 0
output = "Ids: {id:02d}   |   Detected: {detected:02d}"


def add_tag(tag, tag_corners):
    """
    Creates a new Object of Tag in the detected_tags list
    :param tag: the id of the aruco tag
    :param corner: the four corner points of the aruco tag for distance calculations
    :return: None
    """
    latitude, longitude = get_gps()

    c_x, c_y = tag_corners.location_of_center()

    detected_tags.append(Tag(tag, (latitude, longitude), (c_x, c_y)))


def get_gps():
    """
    Finds the GPS Coordinates for the Aruco Tag
    :return: latitude - the latitude of the tag
             longitude - the longitude of the tag
    """

    latitude = interfaces.nav_board.location()[0]
    longitude = interfaces.nav_board.location()[1]

    return latitude, longitude


def detect_ar_tag(image):
    "Searching"
    """
    Detects an AR Tag in the provided color image.
    :param reg_img: the provided image we are looking at to find an ar tag
    :return: tags - a list of Tags (class) that contain the (id, gps, cX and cY) of the detected AR tags
             reg_img - the image with detected AR Tags drawn on top of it
    """

    # Get the location of AR tags
    tags_corners, tag_ids_in_frame = tag_detector.detectMarkers(image)
    tags_in_image = [ArucoTagCorners(tag_corners[0]) for tag_corners in tags_corners]

    if tag_ids_in_frame is not None:
        # Changes the list of ids from 2-dim to 1-dim
        tag_ids_in_frame = [id[0] for id in tag_ids_in_frame]
        detected_tags_ids = [tag.id for tag in detected_tags]
        for tag, id in zip(tags_in_image, tag_ids_in_frame):
            try:
                detected_tag = detected_tags[detected_tags_ids.index(id)]
                detected_tag.tag_spotted(get_gps(), tag.location_of_center())
            except ValueError as e:
                print(e)
                add_tag(id, tag)
    return detected_tags, image


def track_ar_tag(center):
    """
    Track the distance and angle of the AR Tag from the perspective of the Rover.
    :param center: the X, Y of the center pixels of the AR Tag
    :return: distance - the distance in meters to the AR Tag
             angle - the angle in degrees from the rover to the AR Tag. Left is negative, right is positive
    """

    # Center coordinates
    c_x, c_y = center

    # Depth image is at half resolution
    depth_res_x, depth_res_y = core.vision.camera_handler.get_depth_res()
    img_res_x, img_res_y = core.vision.camera_handler.get_reg_res()
    c_x = int((c_x * depth_res_x) / img_res_x)
    c_y = int((c_y * depth_res_y) / img_res_y)

    # Grab the distance from the depth map
    distance = NaN

    # Find some permutations we can use in case of noisy data
    coordinates = [0, 1, -1, 2, -2, 3, -3, 4, -4]
    perm = list(itertools.permutations(coordinates, 2))
    # Grab the distance from the depth map, iterating over pixels if the distance is not finite
    index = 0

    while not np.isfinite(distance) and index < len(perm):
        distance = float(core.vision.camera_handler.grab_depth_data()[c_y + perm[index][1]][c_x + perm[index][0]])
        index += 1

    # Vision system reports depth in mm, we want in meters
    distance /= 1000
    # Grab the camera parameters
    img_res_x, img_res_y = core.vision.camera_handler.get_depth_res()
    hfov = core.vision.camera_handler.get_hfov()

    # Calculate the angle of the object using camera params
    angle_per_pixel = hfov / img_res_x
    pixel_offset = c_x - (img_res_x / 2)
    angle = pixel_offset * angle_per_pixel

    return distance, angle
