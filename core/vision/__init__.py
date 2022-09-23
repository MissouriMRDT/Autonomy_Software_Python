#
# Mars Rover Design Team
# __init__.py
#
# Created on Jan 09, 2021
# Updated on Aug 21, 2022
#

from core.vision.feed_handler import FeedHandler
import core.vision.obstacle_avoidance as obstacle_avoidance
import sys
import core.vision.ar_tag_detector as ar_tag_detector
from core.vision.cameras.camera import Camera

# Camera Handler, used to setup camera and grab frames/point cloud data
camera_handler: Camera = Camera()

# Feed Handler, used to stream/save videos
feed_handler = FeedHandler()

# Flag to indicate whether we are streaming
STREAM_FLAG = True


def setup(type="ZED", stream="Y"):
    """
    Sets up the vision system and camera/feed handlers

    :param type: (str) Currently supports "ZED" and "SIM", specifies the type of camera to init
    :param stream:
    """
    global camera_handler, STREAM_FLAG
    if type == "ZED":
        from core.vision.cameras.zed import ZedCam

        camera_handler = ZedCam()
        camera_handler.start()

    elif type == "SIM":
        from core.vision.cameras.sim_cam import SimCam

        camera_handler = SimCam()
        camera_handler.start()
    elif type == "WEBCAM":
        from core.vision.cameras.web_cam import WebCam

        camera_handler = WebCam()
        camera_handler.start()
    else:
        # TODO: Initialize a regular webcam here
        pass

    # Flag to enable whether we are streaming feeds
    if stream == "Y":
        STREAM_FLAG = True
    else:
        STREAM_FLAG = False


def close(type="ZED"):
    """
    Closes any handlers initialized in the vision subsystem

    :param type: (str) Currently supports "ZED" and "SIM", specifies the type of camera to init
    """
    if type == "ZED" or type == "SIM":
        this.camera_handler.close()
    else:
        pass
