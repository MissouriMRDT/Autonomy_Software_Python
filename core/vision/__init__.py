import core.vision.feed_handler

# Camera Handler, used to setup camera and grab frames/point cloud data
camera_handler: None


def setup(type="ZED"):
    """
    Sets up the vision system and camera/feed handlers

    Parameters:
        type (str) - Currently supports "ZED" and "SIM", specifies the type of camera to init
    """
    if type == "ZED":
        from core.vision.zed_handler import ZedHandler

        core.vision.camera_handler = ZedHandler()
        core.vision.camera_handler.start()
    elif type == "SIM":
        from core.vision.sim_cam_handler import SimCamHandler

        core.vision.camera_handler = SimCamHandler()
        core.vision.camera_handler.start()
    else:
        # TODO: Initialize a regular webcam here
        pass


def close(type="ZED"):
    """
    Closes any handlers initialized in the vision subsystem
    """
    if type == "ZED" or type == "SIM":
        core.vision.camera_handler.close()
    else:
        pass