#
# Mars Rover Design Team
# obstacle_avoidance.py
#
# Created on Feb 15, 2021
# Updated on Aug 21, 2022
#

import logging
import core
import algorithms
import core.constants
import os

# Dict to hold the obstacle info
obstacle_dict = {"detected": False, "angle": None, "distance": None, "object_summary": None, "inference_time": -1, "obstacle_list": None}


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
        min_confidence=0.4,
        classes=core.vision.YOLO_CLASSES,
    )

    # Create new feed for obstacle detection viewing.
    # core.vision.feed_handler.add_feed(3, "obstacle", stream_video=core.vision.STREAM_FLAG)

    while True:
        # Create instance variables.
        object_summary = ""
        inference_time = -1

        # Get regular image from camera.
        reg_img = core.vision.camera_handler.grab_regular()

        # Detect obstacles.
        objects, pred = ObstacleIgnorance.detect_obstacles(reg_img)

        # Track a specific obstacle. (closest one)
        angle, distance, object_summary, inference_time, object_locations = ObstacleIgnorance.track_obstacle(reg_img)

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
