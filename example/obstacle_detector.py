import cv2
import core
import numpy as np
import math
import algorithms
import time

# Define depth img/data so we can use them for OpenCV window callbacks
depth_img = None
depth_data = None


def click_print_depth(event, x, y, flags, param):
    global depth_img, depth_data
    if event == cv2.EVENT_LBUTTONDOWN:
        print(depth_img[y][x])
        print(depth_data.get_data()[y][x])


def main():
    global depth_img, depth_data

    # Enable callbacks on depth window, can be used to display the depth at pixel clicked on
    cv2.namedWindow("depth")
    cv2.setMouseCallback("depth", click_print_depth)

    while True:
        depth_img = core.vision.camera_handler.grab_depth()
        reg_img = core.vision.camera_handler.grab_regular()

        depth_data = core.vision.camera_handler.grab_depth_data()

        obstacle = algorithms.obstacle_detector.detect_obstacle(depth_data, 1, 3)

        reg_img = cv2.resize(reg_img, (int(1280 / 2), int(720 / 2)))

        if obstacle != []:
            angle, distance, _ = algorithms.obstacle_detector.track_obstacle(
                depth_data, obstacle, True, reg_img
            )

        # core.vision.camera_handler.feed_handler.handle_frame("regular", reg_img)
        # time.sleep(1 / 30)

        # Display the camera frames we just grabbed (should show us if potential issues occur)
        cv2.imshow("depth", depth_img)
        cv2.imshow("reg", reg_img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    main()
