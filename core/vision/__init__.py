import core.vision.feed_handler

# Camera Handler, used to setup camera and grab frames/point cloud data
camera_handler: None


def setup(type="ZED"):
    """
    Sets up the vision system and camera/feed handlers

    Parameters:
        type (str) - Currently only supports "ZED", specifies the type of camera to init
    """
    if type == "ZED":
        import core.vision.zed_handler

        core.vision.camera_handler = core.vision.ZedHandler()
        core.vision.camera_handler.start()
    else:
        # TODO: Initialize a regular webcam here
        pass


def close(type="ZED"):
    """
    Closes any handlers initialized in the vision subsystem
    """
    if type == "ZED":
        core.vision.camera_handler.close()
    else:
        pass