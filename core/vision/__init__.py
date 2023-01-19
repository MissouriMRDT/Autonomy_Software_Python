#
# Mars Rover Design Team
# __init__.py
#
# Created on Jan 09, 2021
# Updated on Aug 21, 2022
#

from core.vision.feed_handler import FeedHandler
from core.vision.camera import Camera
import core.vision.obstacle_avoidance as obstacle_avoidance
import sys
import core.vision.ar_tag_detector as ar_tag_detector

# reference to self
this = sys.modules[__name__]

# Camera Handler, used to setup camera and grab frames/point cloud data
camera_handler: Camera = Camera()

# Feed Handler, used to stream/save videos
feed_handler = FeedHandler()

# Flag to indicate whether we are streaming
STREAM_FLAG = True
AVOIDANCE_FLAG = False
YOLO_CLASSES = None


def setup(type="ZED", stream="Y", avoidance="DISABLE", yolo_classes=None):
    """
    Sets up the vision system and camera/feed handlers

    :param type: (str) Currently supports "ZED" and "SIM", specifies the type of camera to init
    :param stream:
    """
    if type == "ZED":
        from core.vision.zed_handler import ZedHandler

        this.camera_handler = ZedHandler()
        this.camera_handler.start()
    elif type == "SIM":
        from core.vision.sim_cam_handler import SimCamHandler

        this.camera_handler = SimCamHandler()
        this.camera_handler.start()
    else:
        # TODO: Initialize a regular webcam here
        pass

    # Flag to enable whether we are streaming feeds
    if stream == "Y":
        this.STREAM_FLAG = True
    else:
        this.STREAM_FLAG = False

     # Flag to enable whether or not we are doing obstacle avoidance.
    if avoidance == "ENABLE":
        this.AVOIDANCE_FLAG = True
    else:
        this.AVOIDANCE_FLAG = False

    # Store yolo_classes list.
    this.YOLO_CLASSES = yolo_classes


def close(type="ZED"):
    """
    Closes any handlers initialized in the vision subsystem

    :param type: (str) Currently supports "ZED" and "SIM", specifies the type of camera to init
    """
    if type == "ZED" or type == "SIM":
        this.camera_handler.close()
    else:
        pass
