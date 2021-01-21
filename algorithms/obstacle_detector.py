import numpy as np
import cv2
import core
import math
import heapq

STEP_SIZE = 0.35


def detect_obstacle(depth_data, min_depth, max_depth):
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

    # Depth segments between min and max with step
    li = np.arange(min_depth, max_depth, core.DEPTH_STEP_SIZE)
    max_li = []

    # Cut off a bottom chunk of the image. This is usually floor/small obstacles and throws off the detector
    for i in range(1, int(height / 4)):
        depth_matrix[height - i] = [0] * width

    # Only pick the NUM_DEPTH_SEGMENTS busisest segments to run on, for performance reasons
    for depth in li:
        max_li.append(
            len(
                depth_matrix[
                    (depth_matrix < depth + core.DEPTH_STEP_SIZE)
                    & (depth_matrix > depth)
                ]
                * (1 / (min_depth - depth))
            )
        )

    max_li = heapq.nlargest(3, zip(max_li, li))

    # Sort starting closest (distance wise) first
    max_li.sort(reverse=False, key=lambda x: x[1])

    # For each step selected, run contour detection looking for blobs at that depth
    for (score, depth) in max_li:
        maskDepth = np.where(
            (depth_matrix < depth + core.DEPTH_STEP_SIZE) & (depth_matrix > depth), 1, 0
        )

        # Find any contours
        contours, hierarchy = cv2.findContours(maskDepth, 2, cv2.CHAIN_APPROX_NONE)

        # Check if there are contours to be detected at this depth
        if contours != []:
            # choose the largest blob at this depth
            blob = max(contours, key=cv2.contourArea)

            # return the blob if it meets the size requiremetns
            if cv2.contourArea(blob) >= core.MIN_OBSTACLE_PIXEL_AREA:
                return blob

    return []


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
    # This will give us the angle between the left lens of the ZED and the obstacle on the
    # x plane
    angle = round(math.degrees(math.atan2(point[0], point[2])), 2)

    # Distance is the corresponding value in the depth map of the center of the obstacle
    distance = round(depth_data.get_value(cY, cX)[1], 2)

    # Draw the obstacle if annotate is true
    if annotate:
        cv2.putText(
            reg_img,
            f"Obstacle Detected at {angle, round(depth_data.get_value(cY ,cX)[1], 2)}",
            (cX - 100, cY - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
        )
        cv2.circle(reg_img, (cX, cY), 7, (255, 255, 255), -1)
        cv2.drawContours(reg_img, obstacle, -1, (0, 255, 0), 3)

    return angle, distance, (cX, cY)
