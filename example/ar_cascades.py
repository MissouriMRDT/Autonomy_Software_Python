import algorithms
import core.vision
import logging
import cv2
from cv2 import aruco
import numpy as np
import time


def main() -> None:
    """
    Main function for video stream script, tests streaming/recording zed footage
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")
    core.vision.feed_handler.add_feed(2, "artag")

    while True:
        reg_img = core.vision.camera_handler.grab_regular()

        # Detect some AR Tags
        tags, reg_img = algorithms.AR_tag.detect_ar_tag(reg_img)

        core.vision.feed_handler.handle_frame("artag", reg_img)
        time.sleep(1 / 30)


if __name__ == "__main__":
    # Run main()
    main()
