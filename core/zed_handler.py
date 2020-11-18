import pyzed.sl as sl
import core.feed_handler
import threading


class ZedHandler:

    def __init__(self):
        # Create a ZED camera object
        self.zed = sl.Camera()
        self.feed_handler = core.feed_handler.FeedHandler()

        # Set configuration parameters
        self.input_type = sl.InputType()
        self.init = sl.InitParameters(input_t=self.input_type)
        self.init.camera_resolution = sl.RESOLUTION.HD720
        self.init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        self.init.coordinate_units = sl.UNIT.MILLIMETER

        # Open the camera
        err = self.zed.open(self.init)
        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            self.zed.close()
            exit(1)

        # Add the desired feeds
        self.feed_handler.add_feed(2, "regular")
        self.feed_handler.add_feed(3, "depth")

        # Create initial frames
        self.reg_img = None
        self.depth_img = None

        # Create thread to constantly grab frames, and pass them to other processes to stream/save
        self.thread = threading.Thread(target=self.frame_grabber, args=())

    def frame_grabber(self):
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

        while True:
            err = self.zed.grab(runtime)
            if err == sl.ERROR_CODE.SUCCESS:
                # Grab images, and grab the data as opencv/numpy matrix
                self.zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
                self.reg_img = image_zed.get_data()
                self.zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)
                self.depth_img = depth_image_zed.get_data()

                # Now let the feed_handler stream/save the frames
                self.feed_handler.handle_frame("regular", self.reg_img)
                self.feed_handler.handle_frame("depth", self.depth_img)

    def grab_regular(self):
        return self.reg_img

    def grab_depth(self):
        return self.depth_img

    def start(self):
        self.thread.start()

    def close(self):
        self.zed.close()

    def grab_point_cloud(self):
        pass
