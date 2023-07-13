#
# Mars Rover Design Team
# obstacle_avoidance.py
#
# Created on Feb 15, 2021
# Updated on Aug 21, 2022
#

import logging
import asyncio
import core
import cv2
import algorithms
import core.constants
import os
import traceback

# Dict to hold the obstacle info
obstacle_dict = {
    "detected": False,
    "angle": None,
    "distance": None,
    "object_summary": None,
    "inference_time": -1,
    "obstacle_list": None,
}


async def async_obstacle_detector():
    """
    Async function to find obstacles
    """
    # Setup logging.
    logger = logging.getLogger(__name__)

    # Declare detection objects
    ObstacleIgnorance = algorithms.obstacle_detector.YOLOObstacleDetector(
        weights=os.path.dirname(__file__) + "/../../resources/yolo_models/2022-0601/weights/best.pt",
        model_image_size=640,
        classes=core.vision.YOLO_CLASSES,
    )

    while True:
        # Create instance variables.
        object_summary = ""
        inference_time = -1

        # Maybe this is bad practice but it's helpful.
        try:
            # Get regular image from camera.
            reg_img = core.vision.camera_handler.grab_regular()

            # Detect obstacles.
            ObstacleIgnorance.detect_obstacles(
                reg_img, core.constants.DETECTION_MODEL_CONF, core.constants.DETECTION_MODEL_IOU
            )

            # Track a specific obstacle. (closest one)
            angle, distance, object_summary, inference_time, object_locations = ObstacleIgnorance.track_obstacle(
                reg_img
            )

            # If obstacle has been detected store its info.
            if distance > -1:
                # Update the current obstacle info
                obstacle_dict["detected"] = True
                obstacle_dict["angle"] = angle
                obstacle_dict["distance"] = distance / 1000
                obstacle_dict["object_summary"] = object_summary
                obstacle_dict["inference_time"] = inference_time
                obstacle_dict["obstacle_list"] = object_locations
            else:
                # Update the current obstacle info
                obstacle_dict["detected"] = False
                obstacle_dict["angle"] = None
                obstacle_dict["distance"] = None
                obstacle_dict["object_summary"] = object_summary
                obstacle_dict["inference_time"] = inference_time
                obstacle_dict["obstacle_list"] = None

            # Give frame with detections overlay to feed handler.
            core.vision.feed_handler.handle_frame("obstacle", reg_img)

            # Show detections window if DISPLAY constant is set.
            if core.constants.DISPLAY_TEST_MODE:
                # Open window for detections viewing.
                cv2.imshow("Obstacle Detections", cv2.resize(reg_img.copy(), (640, 480)))
                # Must call waitkey or window won't display.
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    # If user tries to quit window, print instruction on how to properly disable.
                    logging.warning(msg="To disable window output, edit core.vision.constants file.")

            # Print detected objects for user.
            if obstacle_dict["detected"]:
                logger.info(
                    f"Object tracked at a distance of {obstacle_dict['distance']} meters and {obstacle_dict['angle']} degrees from camera center!\nTotal Objects Detected: {object_summary}Done. ({inference_time:.3f}s)"
                )
        except Exception:
            # Because we are using async functions, they don't print out helpful tracebacks. We must do this instead.
            logger.critical(traceback.format_exc())
        # Must await async process or the code will pause here.
        await asyncio.sleep(core.EVENT_LOOP_DELAY)


def is_obstacle():
    """
    Returns whether there is an obstacle

    :return: detect (bool) - whether something was detected
    """
    return obstacle_dict["detected"]


def get_angle():
    """
    Returns angle

    :return: angle of obstacle
    """
    return obstacle_dict["angle"]


def get_distance():
    """
    Returns distance

    :return: distance of obstacle
    """
    return obstacle_dict["distance"]


def get_object_summary():
    """
    Returns a string containing a summary of all the detected objects.

    :returns summary: The summary string.
    """
    return obstacle_dict["object_summary"]


def get_inference_time():
    """
    Returns yolo model inference time.

    :returns inference_time: The YOLO models inference time.
    """
    return obstacle_dict["inference_time"]


def get_obstacle_locations():
    """
    Returns a list of object locations, could be useful.

    :returns object_locations: A list containing the 3d world coordinates of the objects in meters from the camera center point.
    """
    return obstacle_dict["obstacle_list"]
