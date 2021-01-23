import time
import core.vision
import logging
import cv2
import numpy as np
import pyzed.sl as sl
import algorithms

DISPLAY = False

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
        depth_img = core.vision.camera_handler.grab_depth()

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

        pose = sl.Pose()
        tracking_state = core.vision.camera_handler.get_pose(pose)
        if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
            plane, find_plane_status = core.vision.camera_handler.get_floor()
            if find_plane_status == sl.ERROR_CODE.SUCCESS:
                # Attempting to get camera matrix
                # R = pose.get_rotation_matrix().r.T
                # t = pose.get_translation().get()
                # world2cam = np.hstack((R, np.dot(-R, t).reshape(3, -1)))
                # print(world2cam)
                # print(K)
                # P = K * pose.pose_data()  # camera matrix

                points = []
                bounds = plane.get_bounds()

                #print(bounds)
                for i in bounds:
                    x = ((i[0] * K[0][0]) / i[2]) + K[0][2]
                    y = ((i[1] * K[1][1]) / i[2]) + K[1][2]
                    points.append([x, y])
                    # print(i)

                #print(points)
                points = np.array(points).astype(np.int32)
                #print(points)
                for i in range(len(points)):
                    for j in range(len(points[i])):
                        points[i][j] = points[i][j]/2            
                # points = points.reshape((-1, 1, 2))
                depth_data = core.vision.camera_handler.grab_depth_data()
                depth_matrix = depth_data.get_data()

                cv2.fillPoly(depth_matrix, [points], 0)

                obstacle = algorithms.obstacle_detector.detect_obstacle(depth_matrix, 1, 5)
                # print(obstacle)
                reg_img = cv2.resize(reg_img, (int(1280 / 2), int(720 / 2)))
                cv2.fillPoly(reg_img, [points], 255)

                if obstacle != []:
                    angle, distance, _ = algorithms.obstacle_detector.track_obstacle(
                        depth_data, obstacle, True, reg_img
                    )


        # if mesh.get_boundaries != []:
        #    cv2.drawContours(reg_img, mesh.get_boundaries(), -1, (0, 255, 0), 3)
        # Display the camera frames we just grabbed (should show us if potential issues occur)

        if DISPLAY == True:
            cv2.imshow("reg", reg_img)
            
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            core.vision.camera_handler.feed_handler.handle_frame("regular", reg_img)
            #time.sleep(1 / 30)



if __name__ == "__main__":
    # Run main()
    main()
