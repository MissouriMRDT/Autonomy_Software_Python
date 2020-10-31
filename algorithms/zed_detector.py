# using  a zed 2 camera and the ZED SDK for obstacle detection
# need to import pyzed.sl as sl
import time
import numpy as np


class ZEDDetector:

    def __init__(self):
        self.working = True  # will be false if cannot open camera
        self.image = sl.Mat()  # will be a multi-dimensional array of the most updated camera image
        self.depth = sl.Mat()  # will be a multi-dimensional array representing image depth

        # create camera object
        self.zed = sl.camera

        # Create InitParameters object, set parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # hd720 video mode
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        # x increases to the right, y increases down, z increases the farther away you are
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_DOWN
        init_params.sdk_verbose = True

        # open camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.working = False
        else:
            self.runtime_parameters = sl.RuntimeParameters()
            self.runtime_parameters.sensing_mode = sl.SENSING_MODE_STANDARD  # options can be explored here
            # set depth confidence parameters
            self.runtime_parameters.confidence_threshold = 100
            self.runtime_parameters.textureness_confidence_threshold = 100

    def update_image(self, timeout=100) -> bool:
        """
        update the image, timeout is ms of how long to wait before giving up
        :param timeout:
        :return:
        """
        if not self.working:
            return

        start_time_ms = time.time() * 1000
        current_time_ms = start_time_ms
        while current_time_ms - start_time_ms < timeout:
            current_time_ms = time.time() * 1000
            # a new image is available if grab() returns success
            if self.zed.grab(self.runtime_parameters) == s1.ERROR_CODE_SUCCESS:
                # get the left image from the zed camera
                self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
                # get the depth map
                self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
                self.depth = self.depth.get_data()  # creates numpy array
                return True  # successfully updated the image and depth

        return False  # failed to update the image and depth before the timeout

    def close(self):
        self.zed.close()
