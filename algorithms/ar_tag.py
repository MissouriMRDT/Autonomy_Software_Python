import cv2
from numpy import NaN
import itertools
import core
import math
import numpy as np
import interfaces
import core.constants
from geopy.distance import VincentyDistance
from geopy import Point


class Tag:
    def __init__(self, id, center):
        """
        Creates an object of Tag.

        :param id: The id that was found.
        :param gps: The coordinates of the tag (lat, long).
        :param corner: The four corner points of the aruco tag for distance calculations.

        :return: None
        """
        self.id = id
        self.cX, self.cY = center
        self.times_detected = 1
        self.distance = 0
        self.angle = 0

        # Get tag info.
        self.refresh(center)

    def refresh(self, center):
        """
        Updates tag info.

        :param: None

        :return: Nothing
        """
        # Update tag center.
        self.cX, self.cY = center

        # Grab the camera parameters
        img_res_x, img_res_y = core.vision.camera_handler.get_reg_res()

        # Scale ar tag value between image resolutions.
        depth_cX = int((self.cX * (core.vision.camera_handler.depth_res_x)) / (img_res_x))
        depth_cY = int((self.cY * (core.vision.camera_handler.depth_res_y)) / (img_res_y))

        # Grab the distance from the depth map
        self.distance = NaN

        # Find some permutations we can use in case of noisy data
        coordinates = [0, 1, -1, 2, -2, 3, -3, 4, -4]
        perm = list(itertools.permutations(coordinates, 2))

        # Grab the distance from the depth map, iterating over pixels if the distance is not finite
        index = 0

        while not np.isfinite(self.distance) and index < len(perm):
            if index < len(perm):
                self.distance = core.vision.camera_handler.grab_depth_data()[depth_cY + perm[index][1]][
                    depth_cX + perm[index][0]
                ]
                # If distance is equal to or greater than 40000 (max zed and sim range), then set distance to 40 meters.
                if self.distance >= 40000:
                    self.distance = 40001
                index += 1

        # Vision system reports depth in mm, we want in meters
        self.distance /= 1000

        hfov = core.vision.camera_handler.get_hfov()

        # Calculate the angle of the object using camera params
        angle_per_pixel = hfov / img_res_x
        pixel_offset = self.cX - (img_res_x / 2)
        self.angle = pixel_offset * angle_per_pixel

        # Calculate absolute heading of the obstacle relative to the rover.
        heading = (interfaces.nav_board.heading() + self.angle) % 360
        rover_location = interfaces.nav_board.location()


class ArucoARTagDetector:
    """
    Class for detecting, organizing, filtering, and updating information about
    AR tags in a sequence of images.
    """

    def __init__(self) -> None:
        """
        Create class member varaibles and objects.
        """
        self.tag_list = []

        # Setup aruco detection object.
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        parameters.markerBorderBits = core.constants.ARUCO_MARKER_BORDER_BITS
        parameters.errorCorrectionRate = core.constants.ARUCO_ERROR_CORRECTION_RATE
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    def add_tag(self, tag_id, corners) -> None:
        """
        Creates a new Object of Tag and adds in to the tag list.

        :param id: The aruco tag id number.
        :param corners: The four corners of the tag.

        :return: None
        """
        # Unpack corner points.
        x1 = corners[0][0][0]  # top left x coord
        y1 = corners[0][0][1]  # top left y coord
        x2 = corners[0][1][0]  # top right x coord
        y2 = corners[0][3][1]  # bottom left y coord

        # Calculate the center points of the AR Tag
        cX = (x1 + x2) / 2
        cY = (y1 + y2) / 2

        # Make sure tag_id is valid.
        if tag_id <= 5:
            # Check if a tag with the same ID is already in the list.
            if tag_id in [tag.id for tag in self.tag_list]:
                # Don't add duplicate tag, just increment times_detected.
                # Find the tag with matching id.
                for tag in self.tag_list:
                    if tag.id == tag_id:
                        # Increment number of times tag has been seen.
                        tag.times_detected += 1
                        # Refresh tag distance and angle.
                        tag.refresh((cX, cY))
            else:
                self.tag_list.append(Tag(tag_id, (cX, cY)))

    def detect_ar_tag(self, reg_img):
        """
        Detects an AR Tags in the provided color image. Adds them to the classes tag list after creating
        a Tag object from each of them. Before adding the tag it checks if a tag with the same ID already exists
        in the list. If it does, then the time_detected counter for the Tag object already in the list is incremented.
        This is better than trying to compare relative tag locations, which is inaccurate. This method only works because
        we will never see two of the same tags next to eachother in comp.

        :param reg_img: The provided image we are looking at to find ar tags.

        :return: None
        """

        # Convert frame to grayscale for easier detection.
        gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)

        # Capture Tags
        detected_corners, detected_ids, _ = self.detector.detectMarkers(gray)

        # Make sure at least one tag is detected.
        if detected_corners is not None and detected_ids is not None:
            # Loop through all detected tags and add them to list.
            for tag_id, tag_corners in zip(detected_ids, detected_corners):
                # Unpack.
                tag_id = tag_id[0]
                # Add tag, this method doesn't care about filtering. It just adds all raw tag data.
                self.add_tag(tag_id, tag_corners)
        # If no tags are detected, then decrement the times_detected from all current tags in the list.
        else:
            for tag in self.tag_list:
                # Decrement detection counter.
                if str(core.states.state_machine.state) != "ApproachingGate":
                    tag.times_detected -= 1
                # Check if times_detected has hit zero.
                if tag.times_detected <= 0:
                    # Remove from list.
                    self.tag_list.remove(tag)

    def filter_ar_tags(self, angle_range=180, distance_range=40, valid_id_range=[1, 2, 3, 4, 5]):
        """
        This method filters out bad AR tags from the list given a user defined set of parameters.

        :param angle_range: The +- range that a tag must be in to be considered valid.
        :param distance_range: The max distance that a tag can be before being considered invalid.
        :param id_range: A list of valid Aruco tag id numbers.

        :return: None
        """
        # Create instance variables.
        temp_list = []

        # Loop through tag list.
        for tag in self.tag_list:
            # Check if tag's ID is in valid id list.
            if tag.id in valid_id_range:
                # Check if tag distance is valid.
                if tag.distance > 0 and tag.distance < distance_range:
                    # Check if tag angle is valid.
                    if math.fabs(tag.angle) < angle_range:
                        # Add tag to temp_list.
                        temp_list.append(tag)

        # Set class member tag list to newly created temp list.
        self.tag_list = temp_list

    def track_ar_tags(self, reg_img):
        """
        This method draws tag info onto the given image.

        :param reg_img: The color image to draw overlay onto.

        :return track_img: The new image with tag overlay.
        """
        # Loop through tag array.
        for tag in self.tag_list:
            # Draw a red dot over each detected (and maybe filtered) tag.
            reg_img = cv2.circle(reg_img, (int(tag.cX), int(tag.cY)), radius=0, color=(0, 0, 255), thickness=-1)
            # Draw tag info.
            reg_img = cv2.putText(
                reg_img,
                f"ID:{tag.id}\nDIST:{tag.distance}\nANG:{tag.angle}\nDET:{tag.times_detected}",
                (int(tag.cX), int(tag.cY)),
                fontFace=cv2.FONT_HERSHEY_COMPLEX,
                fontScale=1,
                color=(0, 0, 255),
                thickness=1,
            )

        return reg_img

    def get_tags(self):
        """
        Getter method for returning class tag list.

        :params: Nothing

        :return tag_list: The current class tag list.
        """
        return self.tag_list

    def clear_tags(self):
        """
        Method for clearing tag list.

        :params: None

        :return: Nothing
        """
        # Clear tag list.
        self.tag_list.clear()
