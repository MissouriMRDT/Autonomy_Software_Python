import core
import vision
import logging
import time
import cv2


def main() -> None:
    '''
    Main function for video stream script, tests streaming/recording zed footage
    '''
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    while True:
        reg_img = vision.zed_handler.grab_regular()
        depth_img = vision.zed_handler.grab_depth()
        cv2.imshow('reg',reg_img)
        cv2.imshow('depth',depth_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break

if __name__ == "__main__":
    # Run main()
    main()
