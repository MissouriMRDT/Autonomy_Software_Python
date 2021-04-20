import cv2
import core
import numpy as np
import math
import algorithms
import time

# Define depth img/data so we can use them for OpenCV window callbacks
depth_img = None
depth_data = None

DISPLAY = True


def click_print_depth(event, x, y, flags, param):
    global depth_img, depth_data
    if event == cv2.EVENT_LBUTTONDOWN:
        print(depth_img[y][x])
        print(depth_data.get_data()[y][x])


def main():
    global depth_img, depth_data

    # Enable callbacks on depth window, can be used to display the depth at pixel clicked on
    if DISPLAY:
        cv2.namedWindow("depth")
        cv2.setMouseCallback("depth", click_print_depth)
    else:
        core.vision.feed_handler.add_feed(2, "regular", stream_video=core.vision.STREAM_FLAG)

    while True:
        depth_img = core.vision.camera_handler.grab_depth()
        reg_img = core.vision.camera_handler.grab_regular()

        depth_data = core.vision.camera_handler.grab_depth_data()
        depth_data = depth_data.get_data()
        obstacle = algorithms.obstacle_detector.detect_obstacle(depth_data, 1, 3)

        width, height = core.vision.camera_handler.get_reg_res()
        reg_img = cv2.resize(reg_img, (int(width / 2), int(height / 2)))

        if obstacle != []:
            angle, distance, _ = algorithms.obstacle_detector.track_obstacle(depth_data, obstacle, True, reg_img)

        # Display the camera frames we just grabbed (should show us if potential issues occur)
        if DISPLAY:
            cv2.imshow("depth", depth_img)
            cv2.imshow("reg", reg_img)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            core.vision.feed_handler.handle_frame("regular", reg_img)
            time.sleep(1 / 30)


if __name__ == "__main__":
    main()
