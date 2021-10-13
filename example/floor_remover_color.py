import time
import core.vision
import logging
import cv2
import algorithms
import numpy as np
import pyzed.sl as sl

DISPLAY = True


def main() -> None:
    """
    Test script for removing the floor from an image
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    # Give the system a second to set everything up, start reading in frames
    time.sleep(1)

    while True:
        # Test grabbing the latest camera frames
        reg_img = core.vision.camera_handler.grab_regular()
        depth_matrix = core.vision.camera_handler.grab_depth_data()

        # Grab the mask for the floor
        mask, lower = algorithms.obstacle_detector.get_floor_mask(
            reg_img, int(reg_img.shape[1] / 2), int(reg_img.shape[0] / 2)
        )

        # Remove the floor from the depth data
        depth_matrix = cv2.bitwise_and(depth_matrix, depth_matrix, mask=mask)

        obstacle = algorithms.obstacle_detector.detect_obstacle(depth_matrix, 1, 4)

        width, height = core.vision.camera_handler.get_depth_res()
        reg_img = cv2.resize(reg_img, (width, height))

        test_img = cv2.bitwise_and(reg_img, reg_img, mask=mask)

        if obstacle != []:
            print("Obstacle")
            angle, distance, _ = algorithms.obstacle_detector.track_obstacle(depth_matrix, obstacle, True, reg_img)

        if DISPLAY == True:
            # Display the camera frames we just grabbed (should show us if potential issues occur)
            cv2.imshow("reg", reg_img)
            cv2.imshow("test", test_img)
            cv2.imshow("low", lower)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            core.vision.feed_handler.handle_frame("regular", reg_img)
            time.sleep(1 / 30)


if __name__ == "__main__":
    # Run main()
    main()
