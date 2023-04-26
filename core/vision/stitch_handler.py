import time
import cv2
import numpy
import os.path
import math
import argparse
import json
import logging
from pathlib import Path
import numpy as np


class StitchHandler:
    def __init__(self, right_camera_json, left_camera_json, camera_right_source, camera_left_source):
        """
        Stitches the feed of the 2 180 degree cameras together
        """
        # Create member variables.
        self.homo_matrix = None

        # Gets the distortion matrix for the right then left cameras
        with open(left_camera_json) as json_file:
            data = json.load(json_file)
            self.camera_left_mtx = np.array(data["camera_matrix"])
            self.camera_left_dist = np.array(data["distortion"])
        with open(right_camera_json) as json_file:
            data = json.load(json_file)
            self.camera_right_mtx = np.array(data["camera_matrix"])
            self.camera_right_dist = np.array(data["distortion"])

        # Makes the camera caputre object
        self.cap_left = cv2.VideoCapture(int(camera_left_source))
        self.cap_right = cv2.VideoCapture(int(camera_right_source))

        # Scale the camera matrix for each camera. This should allow the resolution to change without much issue.
        ret, img = self.cap_left.read()
        if ret:
            h, w = img.shape[:2]
            camera_left_mtx_scaled, roi = cv2.getOptimalNewCameraMatrix(
                self.camera_left_mtx, self.camera_left_dist, (w, h), 1, (w, h)
            )
        else:
            print("Failed to open first camera.")
        ret, img = self.cap_right.read()
        if ret:
            h, w = img.shape[:2]
            camera_right_mtx_scaled, roi = cv2.getOptimalNewCameraMatrix(
                self.camera_right_mtx, self.camera_right_dist, (w, h), 1, (w, h)
            )
        else:
            print("Failed to open second camera.")

        # This section of code determines how much of the image needs to be cutoff to
        # remove the black borders from undistortion.

        # Grab initial camera images.
        ret, img_left = self.cap_left.read()
        ret, img_right = self.cap_right.read()

        # Undistort the images from both cameras using the provided camera matrix values.
        self.camera_left_img = cv2.undistort(
            img_left, self.camera_left_mtx, self.camera_left_dist, None, camera_left_mtx_scaled
        )
        self.camera_right_img = cv2.undistort(
            img_right, self.camera_right_mtx, self.camera_right_dist, None, camera_right_mtx_scaled
        )

        # Loop through both images.
        self.image_crops = []
        for img in (self.camera_left_img, self.camera_right_img):
            # Get image dimensions.
            h, w = img.shape[0], img.shape[1]
            # Get middle row and column from image.
            middle_row = img[h // 2]
            middle_column = img[:, w // 2]
            # Loop through row and find black borders.
            x_min, x_max = 0, w
            for i in range(w // 2):
                # Check min row.
                if (middle_row[(w // 2) - i] == [0, 0, 0]).all() and x_min == 0:
                    x_min = (w // 2) - i
                # Check max row.
                if (middle_row[(w // 2) + i] == [0, 0, 0]).all() and x_max == w:
                    x_max = (w // 2) + i
            # Loop through column and find black borders.
            y_min, y_max = 0, h
            for i in range(h // 2):
                # Check min row.
                if (middle_column[(h // 2) - i] == [0, 0, 0]).all() and y_min == 0:
                    y_min = (h // 2) - i
                # Check max row.
                if (middle_column[(h // 2) + i] == [0, 0, 0]).all() and y_max == h:
                    y_max = (h // 2) + i

            # Append x and y limits to list.
            self.image_crops.append([x_min + 10, x_max - 10, y_min + 10, y_max - 10])

        print("[INFO] Cropping images to (removes black borders): ", self.image_crops)

    def stitch(self, images, ratio=0.75, reproj_thresh=4.0):
        """
        This function takes in an number of different camera images and,
        if a homography matrix hasn't been generated yet, it will attempt to
        calculate one. Upon successfully created an homography matrix, the camera
        images will be warped/bent to become stitched together according to the homography.

        Parameters:
        -----------
            images - A list of image images to stitch together.
            ratio - The ratio to multiply the difference between the matches by.
            reproj_thresh - The pixel thresh to give the OpenCV match_keypoints function.

        Returns:
        --------
            result - The stitched image.
        """
        # Pull out images from list.
        image_a, image_b = images[0], images[1]

        # If the saved homography matrix is None, then we need to apply keypoint matching to construct it.
        if self.homo_matrix is None:
            # Detect keypoints and extract.
            (keypoints_a, features_a) = self.detect_and_extract(image_a)
            (keypoints_b, features_b) = self.detect_and_extract(image_b)

            # Match features between the two images
            matched_keypoints = self.calculate_homography(
                keypoints_a, keypoints_b, features_a, features_b, ratio, reproj_thresh
            )

            # If the match is None, then there aren't enough matched keypoints to create a panorama.
            if matched_keypoints is None:
                return None

            # Save the homography matrix.
            self.homo_matrix = matched_keypoints[1]

        # Apply a perspective transform to stitch the images together using the saved homography matrix.
        output_shape = (image_a.shape[1] + image_b.shape[1], image_a.shape[0] + image_a.shape[1])
        result = cv2.warpPerspective(image_a, self.homo_matrix, output_shape)
        result[0 : image_b.shape[0], 0 : image_b.shape[1]] = image_b

        # Return the stitched image
        return result

    def detect_and_extract(self, image):
        """
        This function takes in an image and will run some OpenCV alogrithms to
        extract the features from the image. These features are hard edges,
        big points, or any other notable geometry.

        Parameters:
        -----------
            image - The image to extract features on.

        Returns:
        --------
            (keypoints, features) - The points and lines of the image.
        """
        # Detect and extract features from the image. (DoG keypoint detector and SIFT feature extractor)
        descriptor = cv2.SIFT_create()
        (keypoints, features) = descriptor.detectAndCompute(image, None)

        # Convert the keypoints from KeyPoint objects to numpy arrays.
        keypoints = np.float32([keypoint.pt for keypoint in keypoints])

        # Return a tuple of keypoints and features.
        return (keypoints, features)

    def calculate_homography(self, keypoints_a, keypoints_b, features_a, features_b, ratio=0.75, reproj_thresh=4.0):
        """
        This method takes in the keypoints and features from two cameras,
        and uses the bruteforce method of RANSAC to allign the two images.

        Parameters:
        -----------
            keypoints_a - Extracted keypoints from the first image.
            keypoints_b - Extracted keypoints from the second image.
            features_a - Extracted features from the first image.
            features_b - Extracted features from the second image.
            ratio - The ratio to multiply the difference between the matches by.
            reproj_thresh - The pixel thresh to give the OpenCV match_keypoints function.

        Returns:
        --------
            matches - An array of matched features.
            homography matrix - The calculated homography matrix
            status - The return status of the findHomography OpenCV function.

            (Returns None if homography failed.)
        """
        # Compute the raw matches and initialize the list of actual matches.
        matcher = cv2.DescriptorMatcher_create("BruteForce")
        raw_matches = matcher.knnMatch(features_a, features_b, k=2)
        matches = []

        for raw_match in raw_matches:
            # Ensure the distance is within a certain ratio of each other. (i.e. Lowe's ratio test)
            if len(raw_match) == 2 and raw_match[0].distance < raw_match[1].distance * ratio:
                matches.append((raw_match[0].trainIdx, raw_match[0].queryIdx))

        # Computing a homography requires at least 4 matches.
        if len(matches) > 4:
            # Construct the two sets of points.
            points_a = np.float32([keypoints_a[i] for (_, i) in matches])
            points_b = np.float32([keypoints_b[i] for (i, _) in matches])

            # Compute the homography between the two sets of points.
            (homography_matrix, status) = cv2.findHomography(points_a, points_b, cv2.RANSAC, reproj_thresh)

            # Return the matches, homography matrix and status of each matched point.
            return (matches, homography_matrix, status)

        # No homography could be computed.
        return None

    def grab_stitched(self):
        """
        opens the camera

        Parameters:
        -----------
            none

        Returns:
        --------
            nothing
            opens the result of the camera
        """
        # Crop images.
        cropped_images = []
        for crop, image in zip(self.image_crops, [self.camera_left_img, self.camera_right_img]):
            cropped_images.append(image[crop[2] : crop[3], crop[0] : crop[1]].copy())

        stitched_image = self.stitch(cropped_images, ratio=0.75, reproj_thresh=2.0)
        return stitched_image

    def close_camera(self):
        """
        closes the camera

        Parameters:
        -----------
            none

        Returns:
        --------
            nothing
            closes the camera window
        """
        self.cap_left.release()
        self.cap_right.release()
        cv2.destroyAllWindows()
        return
