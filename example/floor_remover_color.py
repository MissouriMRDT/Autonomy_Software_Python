import time
import core.vision
import logging
import cv2
import numpy as np
import pyzed.sl as sl


def main() -> None:
    """
    Main function for video stream script, tests streaming/recording camera footage
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    # Give the system a second to set everything up, start reading in frames
    time.sleep(1)
    test_img = None
    height = 480

    cap = cv2.VideoCapture("algorithms/obj.avi")
    cv2.namedWindow("res")
    cv2.namedWindow("third")
    cv2.namedWindow("reg")
    time.sleep(5)
    while True:
        # Test grabbing the latest camera frames
        # reg_img = core.vision.camera_handler.grab_regular()
        ret, reg_img = cap.read()
        print(reg_img.shape)
        test_img = cv2.blur(reg_img.copy(), (5, 5))
        test_img = cv2.medianBlur(test_img, 5)
        test_img = cv2.GaussianBlur(test_img, (5, 5), 0)

        test_img = cv2.cvtColor(test_img, cv2.COLOR_BGR2HSV)
        lower_third = test_img[int((9 / 10) * height) :]
        print(int((9 / 10) * height))
        smallest = lower_third.min(axis=(0, 1))
        largest = lower_third.max(axis=(0, 1))

        # Display the camera frames we just grabbed (should show us if potential issues occur)
        # cv2.imshow("depth", reg_img)
        mask = cv2.inRange(test_img, smallest, largest)
        mask = cv2.bitwise_not(mask)
        res = cv2.bitwise_and(test_img, test_img, mask=mask)
        cv2.imshow("res", res)
        cv2.imshow("third", lower_third)
        cv2.imshow("reg", reg_img)

        if cv2.waitKey(100) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    # Run main()
    main()
