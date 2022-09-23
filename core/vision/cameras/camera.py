#
# Mars Rover Design Team
# camera.py
#
# Created on Apr 16, 2021
# Updated on Aug 21, 2022
#

from typing import Tuple


class Camera:
    def __init__(self):
        # Resolutions
        self.reg_res_x = 0
        self.reg_res_y = 0
        self.depth_res_x = 0
        self.depth_res_y = 0

        # Other params
        self.hfov = 0

        # Desired FPS
        self.fps = 0

    def start(self):
        raise NotImplementedError(f"Start not implemented for {self.__class__.__name__} camera type")

    def stop(self):
        raise NotImplementedError(f"Stop not implemented for {self.__class__.__name__} camera type")

    def close(self):
        raise NotImplementedError(f"Close not implemented for {self.__class__.__name__} camera type")

    def frame_grabber(self):
        raise NotImplementedError(f"Frame_grabber not implemented for {self.__class__.__name__} camera type")

    def grab_regular(self):
        raise NotImplementedError(f"Grab_regular not implemented for {self.__class__.__name__} camera type")
        return None

    def grab_depth(self):
        raise NotImplementedError(f"Grab_depth not implemented for {self.__class__.__name__} camera type")
        return None

    def grab_depth_data(self):
        raise NotImplementedError(f"Grab_depth_data not implemented for {self.__class__.__name__} camera type")
        return None

    def grab_point_cloud(self):
        """
        Returns 3D point cloud data captured with ZED
        """
        raise NotImplementedError(f"Grab_point_cloud not implemented for {self.__class__.__name__} camera type")
        return None

    def get_floor(self):
        """
        Returns the estimated floor plane (ZED SDK)
        """
        raise NotImplementedError(f"Get_floor not implemented for {self.__class__.__name__} camera type")
        return None

    def get_info(self):
        """
        Returns information regarding ZED camera calibration parameters
        """
        raise NotImplementedError(f"Get_info not implemented for {self.__class__.__name__} camera type")
        return None

    def get_pose(self, pose):
        """
        Returns the estimated pose of the ZED camera
        """
        raise NotImplementedError(f"Get_pose not implemented for {self.__class__.__name__} camera type")
        return None

    def get_reg_res(self) -> Tuple[int, int]:
        """
        Returns the resolution for the regular images

        :return: reg_res_x - the resolution of the width of the image
                 reg_res_y - the resolution of the height of the image
        """
        return self.reg_res_x, self.reg_res_y

    def get_depth_res(self) -> Tuple[int, int]:
        """
        Returns the resolution for the depth images

        :return: reg_res_x - the resolution of the width of the image
                 reg_res_y - the resolution of the height of the image
        """
        return self.depth_res_x, self.depth_res_y

    def get_hfov(self) -> int:
        return self.hfov

    def get_fps(self) -> int:
        return self.fps

    def __str__(self) -> str:
        return self.__class__.__name__

    def __repr__(self) -> str:
        return self.__class__.__name__
