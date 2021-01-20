import numpy as np
import cv2
import core
import math


def detect_obstacle(depth_data):
    """
    Detects an obstacle in the corresponding depth map. This works by filtering the depth map
    into distinct segments of depth and then finding contours in that data. The contour with
    the biggest area is then used as the obstacle if it meets a certain size requirement

    Parameters:
        depth_data (zed depth map)

    Returns:
        blob - the contour with greatest area, or [] if there were none of sufficent size
    """

    depth_matrix = depth_data.get_data()
    width = int(1280 / 2)
    height = int(720 / 2)
    maskDepth = np.zeros([height, width], np.uint8)

    # Filter z-depth in chunks, this will make it easier to find approximate blobs
    # We map slices to a value, making finding contours a little easier (contours require)
    conditions = [
        (depth_matrix < 1.5) & (depth_matrix >= 1),
        (depth_matrix < 1) & (depth_matrix >= 0.5),
        (depth_matrix < 0.5) & (depth_matrix > 0),
        (depth_matrix >= 1.5) | (depth_matrix <= 0),
    ]

    # Map our varying conditions to numbers in matrix, contours of like values will be easier to find now
    mapped_values = [3, 2, 1, 0]
    maskDepth = np.select(conditions, mapped_values).astype(np.uint8)

    # Cut off a bottom chunk of the image. This is usually floor/small obstacles and throws off the detector
    for i in range(1, 99):
        maskDepth[height - i] = [0] * width

    contours, hierarchy = cv2.findContours(maskDepth, 2, 1)

    # Only proceed if at least one blob is found.
    if not contours:
        return []

    # Choose the largest blob
    blob = max(contours, key=cv2.contourArea)

    if cv2.contourArea(blob) < core.MIN_OBSTACLE_PIXEL_AREA:
        return []

    return blob


def track_obstacle(depth_data, obstacle, annotate=False, reg_img=None):
    """
    Tracks the provided contour, returning angle, distance and center and also optionally
    annotates the provided image with the info and outlined obstacle

    Parameters:
        depth_data (zed depth map)
        obstacle - the contour detected as an obstacle
        annotate (bool) - whether or not to also annotate the provided image with the contour/centroid/etc
        reg_img - the color image from the ZED

    Returns:
        angle - the angle of the obstacle in relation to the left ZED camera
        distance - the distance of the center of the obstacle from the ZED
        center (x, y) - the coordinates (pixels) of the center of the obstacle

    """

    # Find center of contour and mark it on image
    M = cv2.moments(obstacle)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    # Grab point cloud and calculate the angle
    pc = core.vision.camera_handler.grab_point_cloud()
    point = pc.get_value(cX, cY)[1]

    # The angle is the arc tan of opposing side (x offset) divided by the adjacent (z offset)
    # This will give us the angle between the left lens of the ZED and the obstacle
    angle = round(math.degrees(math.atan2(point[0], point[2])), 2)

    # Distance is the corresponding value in the depth map of the center of the obstacle
    distance = round(depth_data.get_value(cY, cX)[1], 2)

    # Draw the obstacle if annotate is true
    if annotate:
        cv2.putText(
            reg_img,
            f"Obstacle Detected at {angle, round(depth_data.get_value(cY ,cX)[1], 2)}",
            (cX - 20, cY - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
        )
        cv2.circle(reg_img, (cX, cY), 7, (255, 255, 255), -1)
        cv2.drawContours(reg_img, [obstacle], -1, (0, 255, 0), 3)

    return angle, distance, (cX, cY)
