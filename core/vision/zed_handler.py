import time
from typing import Tuple
import pyzed.sl as sl
import logging
from core.vision import feed_handler
from core.vision import Camera
import threading
import time


class ZedHandler(Camera):
    def __init__(self):
        """
        Sets up the ZED camera with the specified parameters
        """

        # Create a ZED camera object
        self.zed = sl.Camera()
        self.feed_handler = feed_handler
        self.logger = logging.getLogger(__name__)

        # Define the camera resolutions
        self.depth_res_x = 640
        self.depth_res_y = 360
        self.reg_res_x = 1280
        self.reg_res_y = 720
        self.hfov = 85

        # Define the desired runtime FPS
        self.fps = 30

        # Set configuration parameters
        self.input_type = sl.InputType()
        self.init = sl.InitParameters(input_t=self.input_type)
        self.init.camera_resolution = sl.RESOLUTION.HD720
        self.init.depth_mode = sl.DEPTH_MODE.ULTRA
        self.init.coordinate_units = sl.UNIT.MILLIMETER
        self.init.camera_fps = self.fps
        self.init.depth_minimum_distance = 1

        # Open the camera
        err = self.zed.open(self.init)
        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            self.zed.close()
            exit(1)

        # Add the desired feeds to be recorded (not streamed)
        self.feed_handler.add_feed(10, "regular", save_video=True, stream_video=False)
        self.feed_handler.add_feed(11, "depth", save_video=True, stream_video=False)

        # Create initial frames
        self.reg_img = None
        self.depth_img = None

        # Create thread to constantly grab frames, and pass them to other processes to stream/save
        self._stop = threading.Event()

        self.thread = threading.Thread(target=self.frame_grabber, args=())

    # Should this be a generator or a thread? Generator might help cuz I could schedule this in the ASYNC calls
    def frame_grabber(self):
        """
        Function to be executed as a thread, grabs latest depth/regular images
        from ZED and then passes them to the respective feed handlers
        """

        # Prepare new image size to retrieve half-resolution images
        self.image_size = self.zed.get_camera_information().camera_resolution

        self.depth_size = self.zed.get_camera_information().camera_resolution
        self.depth_size.width = self.depth_size.width / 2
        self.depth_size.height = self.depth_size.height / 2

        # Declare your sl.Mat matrices
        image_zed = sl.Mat(self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)
        depth_image_zed = sl.Mat(self.depth_size.width, self.depth_size.height, sl.MAT_TYPE.U8_C4)
        self.zed.enable_positional_tracking()

        # Set runtime parameters after opening the camera
        runtime = sl.RuntimeParameters()
        runtime.sensing_mode = sl.SENSING_MODE.STANDARD
        runtime.confidence_threshold = 50
        runtime.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA

        while not self._stop.is_set():
            err = self.zed.grab(runtime)
            if err == sl.ERROR_CODE.SUCCESS:
                # Grab images, and grab the data as opencv/numpy matrix
                self.zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, self.image_size)
                self.reg_img = image_zed.get_data()
                self.zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, self.depth_size)
                self.depth_img = depth_image_zed.get_data()

                # Now let the feed_handler stream/save the frames
                self.feed_handler.handle_frame("regular", self.reg_img)
                self.feed_handler.handle_frame("depth", self.depth_img)
                time.sleep(1 / self.fps)

    def grab_regular(self):
        """
        Returns the latest regular frame captured from the ZED
        """
        return self.reg_img

    def grab_depth(self):
        """
        Returns the latest depth frame captured from the ZED
        """
        return self.depth_img

    def grab_depth_data(self):
        self.depth_map = sl.Mat()
        self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH, sl.MEM.CPU, self.depth_size)  # Retrieve depth
        return self.depth_map.get_data()

    def grab_point_cloud(self):
        """
        Returns 3D point cloud data captured with ZED
        """
        point_cloud = sl.Mat()
        self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ, sl.MEM.CPU, self.depth_size)
        return point_cloud

    def get_floor(self):
        """
        Returns the estimated floor plane (ZED SDK)
        """
        plane = sl.Plane()  # detected plane
        reset_tracking_floor_frame = sl.Transform()
        find_plane_status = self.zed.find_floor_plane(plane, reset_tracking_floor_frame)
        # mesh = plane.extract_mesh()
        return plane, find_plane_status

    def get_info(self):
        """
        Returns information regarding ZED camera calibration parameters
        """
        info = self.zed.get_camera_information().calibration_parameters
        return info

    def get_pose(self, pose):
        """
        Returns the estimated pose of the ZED camera
        """
        tracking_state = self.zed.get_position(pose)
        return tracking_state

    def start(self):
        """
        Starts up the frame grabber thread, which constantly polls the ZED camera
        for new frames
        """
        self.thread.start()
        self.logger.info("Starting ZED capture")

    def close(self):
        """
        Closes the zed camera as well as feed handler
        """
        self.logger.info("Calling close")
        # Set the threading event so we kill the thread
        self._stop.set()
        # Wait for the thread to join
        self.thread.join()

        # Now close the ZED camera
        self.logger.info("Calling zed close")
        self.zed.close()

        # Close the feed handler as well
        self.feed_handler.close()

        self.logger.info("Closing ZED capture")