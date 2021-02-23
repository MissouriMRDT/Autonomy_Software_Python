import core.vision
import logging
import cv2
from cv2 import aruco
import numpy as np

# define an empty custom dictionary with
aruco_dict = aruco.custom_dictionary(2, 5, 1)
# add empty bytesList array to fill with 3 markers later
aruco_dict.bytesList = np.empty(shape=(2, 4, 4), dtype=np.uint8)

# add custom markers as defined by the ALVAR library
mybits = np.array(
    [
        [1, 1, 0, 1, 1],
        [1, 1, 0, 1, 1],
        [1, 0, 1, 0, 1],
        [0, 1, 1, 1, 0],
        [
            0,
            1,
            1,
            1,
            0,
        ],
    ],
    dtype=np.uint8,
)
aruco_dict.bytesList[0] = aruco.Dictionary_getByteListFromBits(mybits)

mybits_1 = np.array(
    [
        [1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1],
        [1, 0, 1, 0, 1],
        [1, 1, 0, 1, 1],
        [
            1,
            1,
            0,
            1,
            1,
        ],
    ],
    dtype=np.uint8,
)
aruco_dict.bytesList[1] = aruco.Dictionary_getByteListFromBits(mybits_1)


def main() -> None:
    """
    Main function for video stream script, tests streaming/recording zed footage
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    while True:
        reg_img = core.vision.camera_handler.grab_regular().copy()
        tag_cascade = cv2.CascadeClassifier("algorithms/cascade30.xml")
        gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)

        tags = tag_cascade.detectMultiScale(gray, 1.05, 5)

        for (x, y, w, h) in tags:
            reg_img = cv2.rectangle(reg_img, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # core.vision.camera_handler.feed_handler.handle_frame("regular", reg_img)
        cv2.imshow("reg", reg_img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    # Run main()
    main()
