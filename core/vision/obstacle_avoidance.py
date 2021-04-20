import logging
import core
import algorithms
import cv2
import asyncio

# Dict to hold the obstacle info
obstacle_dict = {"detected": False, "angle": None, "distance": None}


async def async_obstacle_detector():
    """
    Async function to find obstacles
    """
    while True:
        reg_img = core.vision.camera_handler.grab_regular()
        depth_matrix = core.vision.camera_handler.grab_depth_data()

        mask, lower = algorithms.obstacle_detector.get_floor_mask(
            reg_img, int(reg_img.shape[1] / 2), int(reg_img.shape[0] / 2)
        )

        depth_matrix = cv2.bitwise_and(depth_matrix, depth_matrix, mask=mask)
        obstacle = algorithms.obstacle_detector.detect_obstacle(depth_matrix, 1, 4)

        # Resize the image so it matches the dimensions of the depth data
        depth_img_x, depth_img_y = core.vision.camera_handler.get_depth_res()
        reg_img = cv2.resize(reg_img, (depth_img_x, depth_img_y))

        if obstacle != []:
            # Track the obstacle in the depth matrix
            angle, distance, _ = algorithms.obstacle_detector.track_obstacle(
                depth_matrix, obstacle, False, True, reg_img
            )
            # Update the current obstacle info
            obstacle_dict["detected"] = True
            obstacle_dict["angle"] = angle
            obstacle_dict["distance"] = distance
        else:
            # Update the current obstacle info
            obstacle_dict["detected"] = False
            obstacle_dict["angle"] = None
            obstacle_dict["distance"] = None

        core.vision.feed_handler.handle_frame("obstacle", reg_img)
        await asyncio.sleep(1 / core.vision.camera_handler.get_fps())


def is_obstacle():
    """
    Returns whether there is an obstacle

    Returns:
    -------------
        detect (bool) - whether or not something was detected
    """
    return obstacle_dict["detected"]


def get_angle():
    return obstacle_dict["angle"]


def get_distance():
    return obstacle_dict["distance"]
