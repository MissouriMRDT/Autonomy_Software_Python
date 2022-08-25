#
# Mars Rover Design Team
# floor_pathfinder.py
#
# Created on Feb 25, 2021
# Updated on Aug 21, 2022
#

import time
import core.vision
import logging
import cv2
import numpy as np

DISPLAY = True


def main() -> None:
    """
    Tests some basic bob detection with OpenCV
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    # Give the system a second to set everything up, start reading in frames
    time.sleep(1)
    while True:
        # Test grabbing the latest camera frames
        reg_img = core.vision.camera_handler.grab_regular()
        # depth_img = core.vision.camera_handler.grab_depth()
        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create()

        # Detect blobs.
        keypoints = detector.detect(reg_img)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(
            reg_img,
            keypoints,
            np.array([]),
            (0, 0, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )

        if DISPLAY == True:
            # Display the camera frames we just grabbed (should show us if potential issues occur)
            cv2.imshow("reg", im_with_keypoints)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break


if __name__ == "__main__":
    # Run main()
    main()
