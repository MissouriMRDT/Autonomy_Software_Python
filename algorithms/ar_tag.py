import cv2
from numpy import NaN
import itertools
import core
import numpy as np
import interfaces
import core.constants


class Tag:
    def __init__(self, tag, gps, center):
        """
        Creates an object of Tag.

        :param tag: The id that was found.
        :param gps: The coordinates of the tag (lat, long).
        :param corner: The four corner points of the aruco tag for distance calculations.

        :return: None
        """
        self.id = tag
        self.lat, self.long = gps
        self.cX, self.cY = center
        self.detected = 1
        self.empty = 0
        self.distance = 0
        self.angle = 0
        self.output = "Ids: {id:02d}   |   Detected: {detected:02d}"

    def check_tag(self, tag, gps, corner, index):
        """
        Checks to see if the tag that was spotted has already been added to the array.

        :param tag: The id that is being checked.
        :param gps: The coordinates of the tag (lat, long).
        :param corner: The four corner points of the aruco tag for distance calculations.
        :param index: The index in the tag list.

        :return: True if detected, False if not detected
        """
        if self.id == tag:
            self.tag_spotted(gps, corner, index)
            return True

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

        if self.detected >= core.constants.ARUCO_FRAMES_DETECTED:
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

        :return: The output string.
        """
        return self.output.format(id=self.id, detected=self.detected)


# TODO: ThESE ARE BAD CONSTANTS MAKE THESE FUNCTION A CLASS!
detected_tags = []
blank_frames = 0
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
parameters.markerBorderBits = core.constants.ARUCO_MARKER_BORDER_BITS
parameters.errorCorrectionRate = core.constants.ARUCO_ERROR_CORRECTION_RATE
detector = cv2.aruco.ArucoDetector(dictionary, parameters)


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
    Detects an AR Tag in the provided color image.
    Parameters:
    -----------
        reg_img - the provided image we are looking at to find an ar tag
    Returns:
    --------
        tags - a list of Tags (class) that contain the (id, gps, cX and cY) of the detected AR tags
        reg_img - the image with detected AR Tags drawn on top of it
    """

    # Convert frame to grayscale for easier detection.
    gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)

    # Capture Tags
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

    # Add Tags to Tag Class Object
    if ids is not None:
        # reg_img = cv2.aruco.drawDetectedMarkers(reg_img, corners)
        index_counter = 0

        # Create a list of all ids seen in frame
        frameTags = []
        for i in ids:
            for j in i:
                frameTags.append(j)

        # Loops through ids found and checks adds them to the list accordingly.
        for i in frameTags:
            if len(detected_tags) == 0:
                add_tag(i, corners, index_counter)
            else:
                found = False
                for t in detected_tags:
                    if t.check_tag(i, get_gps(), corners, index_counter):
                        found = True
                if not found:
                    add_tag(i, corners, index_counter)

            index_counter += 1

        # Loops through ids found on current leg and checks if they were in current frame.
        for t in detected_tags:
            if t.id not in frameTags:
                t.empty += 1

    # Checks for Blank Frames and resets counter if >= ARUCO_core.constants.ARUCO_FRAMES_DETECTED
    else:
        for t in detected_tags:
            t.empty += 1
            # if t.empty >= ARUCO_core.constants.ARUCO_FRAMES_DETECTED:
            #     t.resetSpotted()

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
    # Center coordinates
    cX, cY = center

    # Grab the camera parameters
    img_res_x, img_res_y = core.vision.camera_handler.get_reg_res()

    # Scale ar tag value between image resolutions.
    depth_cX = int((cX * (core.vision.camera_handler.depth_res_x)) / (img_res_x))
    depth_cY = int((cY * (core.vision.camera_handler.depth_res_y)) / (img_res_y))

    # Grab the distance from the depth map
    distance = 40000

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
            # If distance is equal to or greater than 40000 (max zed and sim range), then set distance to 40 meters.
            if distance >= 40000:
                distance = 40000
            index += 1

    # Vision system reports depth in mm, we want in meters
    distance /= 1000

    hfov = core.vision.camera_handler.get_hfov()

    # Calculate the angle of the object using camera params
    angle_per_pixel = hfov / img_res_x
    pixel_offset = cX - (img_res_x / 2)
    angle = pixel_offset * angle_per_pixel

    return distance, angle
