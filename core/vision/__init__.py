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

# Flag to indicate whether or not we are streaming
STREAM_FLAG = True


def setup(type="ZED", stream="Y", stitching="DISABLE", right_camera_path=1, left_camera_path=2):
    """
    Sets up the vision system and camera/feed handlers

    Parameters:
        type (str) - Currently supports "ZED" and "SIM", specifies the type of camera to init
    """
    if type == "ZED":
        from core.vision.zed_handler import ZedHandler

        this.camera_handler = ZedHandler()
        this.camera_handler.start()
    elif type == "SIM":
        from core.vision.sim_cam_handler import SimCamHandler

        this.camera_handler = SimCamHandler()
        this.camera_handler.start()

    # Flag to enable whether or not we are streaming feeds
    if stream == "Y":
        this.STREAM_FLAG = True
    else:
        this.STREAM_FLAG = False

    # Flag for camera stitching
    if stitching == "ENABLE":
        from core.vision.stitch_handler import StitchHandler

        right_camera_json = os.path.dirname(__file__) + "/../../resources/camera_calibration/right_camera.json"
        left_camera_json = os.path.dirname(__file__) + "/../../resources/camera_calibration/left_camera.json"
        this.stitch_handler = StitchHandler(right_camera_json, left_camera_json, right_camera_path, left_camera_path)


def close(type="ZED"):
    """
    Closes any handlers initialized in the vision subsystem
    """
    if type == "ZED" or type == "SIM":
        this.camera_handler.close()
    else:
        pass
