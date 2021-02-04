import pyzed.sl as sl
import logging
from core.vision import feed_handler
import threading
import time


class ZedHandler:
    def __init__(self):
        """
        Sets up the ZED camera with the specified parameters
        """

        # Create a ZED camera object
        self.zed = sl.Camera()
        self.feed_handler = feed_handler
        self.logger = logging.getLogger(__name__)

        # Set configuration parameters
        self.input_type = sl.InputType()
        self.init = sl.InitParameters(input_t=self.input_type)
        self.init.camera_resolution = sl.RESOLUTION.HD720
        self.init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        self.init.coordinate_units = sl.UNIT.MILLIMETER
        self.init.camera_fps = 30

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
        image_size = self.zed.get_camera_information().camera_resolution
        image_size.width = image_size.width / 2
        image_size.height = image_size.height / 2

        # Declare your sl.Mat matrices
        image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
        depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

        # Set runtime parameters after opening the camera
        runtime = sl.RuntimeParameters()
        runtime.sensing_mode = sl.SENSING_MODE.STANDARD

        while not self._stop.is_set():
            err = self.zed.grab(runtime)
            if err == sl.ERROR_CODE.SUCCESS:
                # Grab images, and grab the data as opencv/numpy matrix
                self.zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
                self.reg_img = image_zed.get_data()
                self.zed.retrieve_image(
                    depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, image_size
                )
                self.depth_img = depth_image_zed.get_data()

                # Now let the feed_handler stream/save the frames
                self.feed_handler.handle_frame("regular", self.reg_img)
                self.feed_handler.handle_frame("depth", self.depth_img)

                # Sleep for desired frame rate, reduces usage
                time.sleep(1 / self.init.camera_fps)

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
        # Set the threading event so we kill the thread
        self._stop.set()
        # Wait for the thread to join
        self.thread.join()

        # Now close the ZED camera
        self.zed.close()

        # Close the feed handler as well
        self.feed_handler.close()

        self.logger.info("Closing ZED capture")

    def grab_point_cloud(self):
        """
        Returns 3D point cloud data captured with ZED
        """
        pass
