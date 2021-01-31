import time
import core.vision
import logging
import cv2
import algorithms
import numpy as np
import pyzed.sl as sl

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
        depth_data = core.vision.camera_handler.grab_depth_data()
        depth_matrix = depth_data.get_data()

        #
        mask, lower = algorithms.obstacle_detector.get_floor_mask(
            reg_img, int(reg_img.shape[1] / 2), int(reg_img.shape[0] / 2)
        )

        depth_matrix = cv2.bitwise_and(depth_matrix, depth_matrix, mask=mask)

        obstacle = algorithms.obstacle_detector.detect_obstacle(depth_matrix, 1, 3)
        # print(obstacle)
        reg_img = cv2.resize(reg_img, (int(1280 / 2), int(720 / 2)))
        test_img = cv2.bitwise_and(reg_img, reg_img, mask=mask)

        if obstacle != []:
            angle, distance, _ = algorithms.obstacle_detector.track_obstacle(
                depth_data, obstacle, True, reg_img
            )

        if DISPLAY == True:
             # Display the camera frames we just grabbed (should show us if potential issues occur)
            cv2.imshow("reg", reg_img)
            cv2.imshow("test", test_img)
            cv2.imshow("low", lower)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            core.vision.camera_handler.feed_handler.handle_frame("regular", reg_img)
            #time.sleep(1 / 30)

if __name__ == "__main__":
    # Run main()
    main()
