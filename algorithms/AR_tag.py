from collections import namedtuple
from math import dist
import cv2
import logging
from numpy.core.numeric import NaN
import itertools
import core
import numpy as np
from cv2 import aruco

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

    tags = []

    # print(frame.shape) #480x640
    # Our operations on the frame come here
    gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    # print(parameters)

    """    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        """
    # lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # It's working.
    # my problem was that the cellphone put black all around it. The algorithm
    # depends very much upon finding rectangular black blobs

    if corners:
        reg_img = aruco.drawDetectedMarkers(reg_img, corners)

    """
    if ids is not None:
        tags.clear()
        for array in ids:
            tags.append(array[0])
    """

    # print(rejectedImgPoints)
    # Display the resulting frame
    # cv2.imshow('frame',reg_img)

    # tag 1
    if ids is not None and len(ids) > 0:
        x1 = corners[0][0][0][0]  # top left x coord
        y1 = corners[0][0][0][1]  # top left y coord
        x2 = corners[0][0][1][0]  # top right x coord
        y2 = corners[0][0][3][1]  # bottom left y coord

        # Calculate the center points of the AR Tag
        cX = (x1 + x2) / 2
        cY = (y1 + y2) / 2

        # Find the distance/angle of said center pixels
        distance, angle = track_ar_tag((cX, cY))
        tags.append(Tag(cX, cY, distance, angle))

    # tag 2
    if ids is not None and len(ids) > 1:
        x1b = corners[1][0][0][0]  # top left x coord
        y1b = corners[1][0][0][1]  # top left y coord
        x2b = corners[1][0][1][0]  # top right x coord
        y2b = corners[1][0][3][1]  # bottom left y coord

        cXb = (x1b + x2b) / 2
        cYb = (y1b + y2b) / 2

        distance, angle = track_ar_tag((cXb, cYb))
        tags.append(Tag(cXb, cYb, distance, angle))

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
