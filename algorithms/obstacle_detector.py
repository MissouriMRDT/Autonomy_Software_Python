import algorithms
import numpy as np
import cv2
import core
import math
import heapq


def get_floor_mask(reg_img, dimX, dimY):
    """
    Returns a cv2 mask that indentifies the floor in the provided reg_img of dimensions dimX, dimY

    This can then be used to remove the floor from a corresponding depth map/color image.

    Parameters:
        reg_img - the color image where we will perform floor detection
        dimX - width in pixels of desire mask (size of image to be applied to)
        dimY - height in pixels of desire mask (size of image to be applied to)
    Returns:
        mask - the mask of the floor
    """
    # Perform various blur operations on the image to enhance accuracy of color segmentation
    test_img = cv2.resize(reg_img.copy(), (dimX, dimY))
    test_img = cv2.blur(test_img, (5, 5))
    test_img = cv2.medianBlur(test_img, 5)
    test_img = cv2.GaussianBlur(test_img, (5, 5), 0)

    test_img = cv2.cvtColor(test_img, cv2.COLOR_BGR2HSV)

    lower_portion = test_img[int((14 / 15) * dimY) :]
    smallest = lower_portion.min(axis=(0, 1))
    largest = lower_portion.max(axis=(0, 1))

    # Get a mask of the possible range of colors of the floor
    mask = cv2.inRange(test_img, smallest, largest)

    # Invert the mask so we select everything BUT the floor
    mask = cv2.bitwise_not(mask)

    return mask, lower_portion


1


def detect_obstacle(depth_matrix, min_depth, max_depth):
    """
    Detects an obstacle in the corresponding depth map. This works by filtering the depth map
    into distinct segments of depth and then finding contours in that data. The contour with
    the biggest area is then used as the obstacle if it meets a certain size requirement.

    Currently we look at the NUM_DEPTH_SEGMENTS busiest segments (most points) and check in order
    of closeness whether or not they have

    Parameters:
        depth_data (zed depth map)

    Returns:
        blob - the contour with greatest area, or [] if there were none of sufficent size
    """
    width = core.vision.camera_handler.depth_res_x
    height = core.vision.camera_handler.depth_res_y

    maskDepth = np.zeros([height, width], np.uint8)

    # Depth segments between min and max with step
    li = np.arange(min_depth, max_depth, core.DEPTH_STEP_SIZE)
    max_li = []

    # Cut off a bottom chunk of the image. This is usually floor/small obstacles and throws off the detector
    # for i in range(1, int(height / 3)):
    #    depth_matrix[height - i] = [0] * width

    # Only pick the NUM_DEPTH_SEGMENTS busisest segments to run on, for performance reasons
    for depth in li:
        max_li.append(
            len(
                depth_matrix[(depth_matrix < depth + core.DEPTH_STEP_SIZE) & (depth_matrix > depth)]
                * (1 / ((depth - min_depth) + 1))
            )
        )

    max_li = heapq.nlargest(core.NUM_DEPTH_SEGMENTS, zip(max_li, li))

    # Sort starting closest (distance wise) first
    max_li.sort(reverse=False, key=lambda x: x[1])

    # For each step selected, run contour detection looking for blobs at that depth
    for (score, depth) in max_li:
        maskDepth = np.where((depth_matrix < depth + core.DEPTH_STEP_SIZE) & (depth_matrix > depth), 1, 0)

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


def compute_precise_angle(cX, cY):
    """
    Computes the precise angle of the centroid
    """
    # Grab point cloud and calculate the angle
    pc = core.vision.camera_handler.grab_point_cloud()
    point = []

    # If the center coordinate of the obstacle happens to have no data
    # look at some neighboring points
    permutations = [
        [cX, cY],
        [cX + 1, cY],
        [cX, cY + 1],
        [cX + 1, cY + 1],
        [cX - 1, cY],
        [cX, cY - 1],
        [cX - 1, cY - 1],
    ]

    # loop through the permuations and check if those values are NaN
    for index in range(len(permutations)):
        point = pc.get_value(permutations[index][0], permutations[index][1])[1]
        if not math.isnan(point[0]):
            cX = permutations[index][0]
            cY = permutations[index][1]
            break

    # The angle is the arc tan of opposing side (x offset) divided by the adjacent (z offset)
    # This will give us the angle between the left lens of the ZED and the obstacle on the
    # x plane
    angle = round(math.degrees(math.atan2(point[0] + core.ZED_X_OFFSET, point[2])), 2)

    return angle


def track_obstacle(depth_data, obstacle, precise_angle=False, annotate=False, reg_img=None, rect=False):
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

    if precise_angle:
        angle = compute_precise_angle(cX, cY)
    else:
        # Return 15 degrees right or left depending on which side of the rover the object was
        angle = 15 if cX > (1280 / 2) else -15

    # Distance is the corresponding value in the depth map of the center of the obstacle
    distance = round(depth_data[cY][cX], 2)

    # Draw the obstacle if annotate is true
    if annotate:
        if rect:
            rect = cv2.boundingRect(obstacle)
            x, y, w, h = rect
            cv2.rectangle(reg_img, (x - 10, y - 10), (x + w + 10, y + h + 10), (255, 0, 0), 2)
        else:
            cv2.drawContours(reg_img, obstacle, -1, (0, 255, 0), 3)
        cv2.putText(
            reg_img,
            f"Obstacle Detected at {angle, round(depth_data[cY][cX], 2)}",
            (cX - 100, cY - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
        )
        cv2.circle(reg_img, (cX, cY), 7, (255, 255, 255), -1)

    return angle, distance, (cX, cY)