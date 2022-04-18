from asyncio.log import logger
from collections import namedtuple
from math import dist
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
    def __init__(self, id, gps, center):
        """
        Creates an object of Tag
        Parameters:
        -----------
            id - the tag that was found
            gps - the coordinates of the tag found
            center - the center points of the tag
        Returns:
        --------
            None
        """

        self.id = id
        self.lat, self.long = gps
        self.cX, self.cY = center
        self.detected = 1
        self.distance, self.angle = NaN

    def checkTag(self, id, gps):
        """
        Checks to see if the tag that was spotted has already been added to the array.
        Parameters:
        -----------
            id - the tag that is being checked
            gps - the coordinates of the tag being checked
        Returns:
        --------
            bool - True if detected, False if not detected
        """
        
        if id == self.id:
            self.tagSpotted(gps)
            return True
        else:
            return False

    def tagSpotted(self, gps):
        """
        Increments the detected variable and updates GPS, distance, and angle
        Parameters:
        -----------
            gps - the coordinates of the tag
        Returns:
        --------
            None
        """

        self.detected += 1
        self.gps = gps

        if self.detected >= FRAMES_DETECTED:
            self.distance, self.angle = track_ar_tag((self.cX, self.cY))

    def resetSpotted(self):
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

        print(output.format(id = self.id, detected = self.detected))

tag_cascade = cv2.CascadeClassifier("resources/tag_detection/cascade30.xml") # What is this line used for? Can it be removed?
detected_tags = []
blank_frames = 0
output = "Ids: {id:02d}   |   Detected: {detected:02d}"

def getGPS():
    """
    Finds the GPS Cordinates for the Aruco Tag
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

def addTag(id_list, corner_list):
    """
    Creates a new Object of Tag in the detected_tags list
    Parameters:
    -----------
        id_list - the id of the aruco tag
        corner_list - the four corner points of the aruco tag for distance calculations
    Returns:
    --------
        None
    """

    latitude, longitude = getGPS()

    x1 = corner_list[0][0][0][0]  # top left x coord
    y1 = corner_list[0][0][0][1]  # top left y coord
    x2 = corner_list[0][0][1][0]  # top right x coord
    y2 = corner_list[0][0][3][1]  # bottom left y coord

    # Calculate the center points of the AR Tag
    cX = (x1 + x2) / 2
    cY = (y1 + y2) / 2

    detected_tags.append(Tag(id, (latitude, longitude), (cX, cY)))

def detect_ar_tag(reg_img):
    """
    Detects an AR Tag in the provided color image.
    Parameters:
    -----------
        reg_img - the provided image we are looking at to find an ar tag
    Returns:
    --------
        tags - a list of Tags (class) that contain the (id, gps, cX and cY) of the detected AR tags
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

        # Loops through ids found and checks adds them to the list accordingly.
        for i in ids:
            for j in i:
                if len(detected_tags) == 0:
                    addTag(j, corners)
                else:
                    found = False
                    for t in detected_tags:
                        if t.checkTag(j, getGPS()):
                            found = True
                    if not found:
                        addTag(j, corners)
    
    # Checks for Blank Frames and resets counter if >= FRAMES_DETECTED
    else:
        blank_frames += 1
        if blank_frames >= FRAMES_DETECTED:
            for t in detected_tags:
                t.resetSpotted()

    return detected_tags, reg_img

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
