import time
import core.vision
import logging
import cv2
import numpy as np
import pyzed.sl as sl


def main() -> None:
    """
    Main function for video stream script, tests streaming/recording camera footage
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    # Give the system a second to set everything up, start reading in frames
    time.sleep(1)

    while True:
        # Test grabbing the latest camera frames
        reg_img = core.vision.camera_handler.grab_regular()
        plane = core.vision.camera_handler.get_floor()
        # contours, hierarchy = cv2.findContours(reg_img, 2, cv2.CHAIN_APPROX_NONE)
        # print(plane.get_bounds())

        cam_params = core.vision.camera_handler.get_info()
        K = np.array(
            [
                [cam_params.left_cam.fx, 0, cam_params.left_cam.cx],
                [0, cam_params.left_cam.fy, cam_params.left_cam.cy],
                [0, 0, 1],
            ]
        )

        # Attempting to get camera matrix
        pose = sl.Pose()
        core.vision.camera_handler.get_pose(pose)
        R = pose.get_rotation_matrix().r.T
        t = pose.get_translation().get()
        world2cam = np.hstack((R, np.dot(-R, t).reshape(3, -1)))
        print(world2cam)
        print(K)
        P = np.dot(K, world2cam)  # camera matrix

        points = []
        bounds = plane.get_bounds()

        print(bounds)
        for i in bounds:
            x = abs((i[0] * P[0][0]) / i[2] + P[0][2])
            y = abs((i[1] * P[1][1]) / i[2] + P[1][2])
            points.append([x, y])
            # print(i)

        print(points)
        points = np.array(points).astype(np.int32)
        print(points)
        points = points.reshape((-1, 1, 2))

        cv2.polylines(reg_img, [points], True, (0, 255, 255))

        # if mesh.get_boundaries != []:
        #    cv2.drawContours(reg_img, mesh.get_boundaries(), -1, (0, 255, 0), 3)
        # Display the camera frames we just grabbed (should show us if potential issues occur)
        cv2.imshow("reg", reg_img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    # Run main()
    main()
