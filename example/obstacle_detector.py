import cv2
import core
import numpy as np
import math
import algorithms
import time

# Define depth img/data so we can use them for OpenCV window callbacks
depth_matrix = None

DISPLAY = True


def click_print_depth(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(depth_matrix[y][x])


def main():
    global depth_img, depth_matrix

    # Enable callbacks on depth window, can be used to display the depth at pixel clicked on
    if DISPLAY:
        cv2.namedWindow("reg")
        cv2.setMouseCallback("reg", click_print_depth)
    else:
        core.vision.feed_handler.add_feed(2, "regular", stream_video=core.vision.STREAM_FLAG)

    while True:
        # Get point cloud from ZED camera.
        reg_img = core.vision.camera_handler.grab_regular()
        # depth_img = core.vision.camera_handler.grab_depth()
        depth_matrix = core.vision.camera_handler.grab_depth_data()
        mask, lower = algorithms.obstacle_detector.get_floor_mask(
            reg_img, int(depth_matrix.shape[1]), int(depth_matrix.shape[0])
        )

        depth_matrix = cv2.bitwise_and(depth_matrix, depth_matrix, mask=mask)
        obstacle = algorithms.obstacle_detector.detect_obstacle(depth_matrix, 1, 5)

        # Resize the image so it matches the dimensions of the depth data
        depth_img_x, depth_img_y = core.vision.camera_handler.get_depth_res()
        reg_img = cv2.resize(reg_img, (depth_img_x, depth_img_y))

        if obstacle != []:
            # Track the obstacle in the depth matrix
            angle, distance, _ = algorithms.obstacle_detector.track_obstacle(
                depth_matrix, obstacle, True, reg_img, False
            )

        # Display the camera frames we just grabbed (should show us if potential issues occur)
        if DISPLAY:
            cv2.imshow("reg", reg_img)
            cv2.imshow("mask", mask)
            cv2.imshow("lower", lower)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            core.vision.feed_handler.handle_frame("regular", reg_img)
            time.sleep(1 / 30)


if __name__ == "__main__":
    main()
