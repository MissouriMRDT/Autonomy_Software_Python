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
    core.vision.camera_handler.feed_handler.add_feed(2, "artag")

    while True:
        reg_img = core.vision.camera_handler.grab_regular()
        tag_cascade = cv2.CascadeClassifier("algorithms/cascade30.xml")
        gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)

        tags = tag_cascade.detectMultiScale(gray, 1.05, 5)

        for (x, y, w, h) in tags:
            print(x, y, w, h)
            reg_img = cv2.rectangle(reg_img, (x, y), (x + w, y + h), (255, 0, 0), 2)

        core.vision.camera_handler.feed_handler.handle_frame("artag", reg_img)
        time.sleep(1/30)

if __name__ == "__main__":
    # Run main()
    main()
