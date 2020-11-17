import pyzed.sl as sl


class ZedHandler:

    def __init__(self):
        # Create a ZED camera object
        self.zed = sl.Camera()

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

    def grab_frames(self):
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
                self.zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
                reg_img = image_zed.get_data()
                self.zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)
                depth_img = depth_image_zed.get_data()
                return reg_img, depth_img

    def close(self):
        self.zed.close()

    def grab_point_cloud(self):
        pass
