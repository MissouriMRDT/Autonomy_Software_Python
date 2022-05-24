from typing import Tuple


class Camera:
    def __init__(self):
        # Resolutions
        self.reg_res_x = 0
        self.reg_res_y = 0
        self.depth_res_x = 0
        self.depth_res_y = 0
        self.point_cloud_res_x = 0
        self.point_cloud_res_y = 0

        # Other params
        self.hfov = 0

        # Desired FPS
        self.fps = 0

    def start(self):
        raise NotImplementedError("Start not implemented for this camera type")

    def stop(self):
        raise NotImplementedError("Stop not implemented for this camera type")

    def frame_grabber(self):
        raise NotImplementedError("Frame_grabber not implemented for this camera type")

    def grab_regular(self):
        raise NotImplementedError("Grab_regular not implemented for this camera type")

    def grab_depth(self):
        raise NotImplementedError("Grab_depth not implemented for this camera type")

    def grab_depth_data(self):
        raise NotImplementedError("Grab_depth_data not implemented for this camera type")

    def grab_point_cloud(self):
        raise NotImplementedError("Grab_point_cloud not implemented for this camera type")

    def get_reg_res(self) -> Tuple[int, int]:
        """
        Returns the resolution for the regular images

        Returns:
        --------
            reg_res_x - the resolution of the width of the image
            reg_res_y - the resolution of the height of the image
        """
        return self.reg_res_x, self.reg_res_y

    def get_depth_res(self) -> Tuple[int, int]:
        """
        Returns the resolution for the depth images

        Returns:
        --------
            reg_res_x - the resolution of the width of the image
            reg_res_y - the resolution of the height of the image
        """
        return self.depth_res_x, self.depth_res_y

    def get_cloud_res(self) -> Tuple[int, int]:
        """
        Returns the resolution for the point cloud.

        Returns:
        --------
            reg_res_x - the resolution of the width of the image
            reg_res_y - the resolution of the height of the image
        """
        return self.point_cloud_res_x, self.point_cloud_res_y

    def get_hfov(self) -> int:
        return self.hfov

    def get_fps(self) -> int:
        return self.fps
