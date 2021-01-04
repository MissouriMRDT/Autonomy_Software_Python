import vision.feed_handler

# Camera Handler, used to setup camera and grab frames/point cloud data
camera_handler: None


def setup(type="ZED"):
    """
    Sets up the vision system and camera/feed handlers

    Parameters:
        type (str) - Currently only supports "ZED", specifies the type of camera to init
    """
    if type == "ZED":
        import vision.zed_handler

        vision.camera_handler = vision.ZedHandler()
        vision.camera_handler.start()
    else:
        # TODO: Initialize a regular webcam here
        pass
