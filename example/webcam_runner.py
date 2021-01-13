import time
import core.vision
import logging
import cv2


def main() -> None:
    """
    Main function for video stream script, tests streaming/recording camera footage
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    # Give the system a second to set everything up, start reading in frames
    time.sleep(5)

    while True:
        # Test grabbing the latest camera frames
        reg_img = core.vision.camera_handler.grab_regular()

        # Display the camera frames we just grabbed (should show us if potential issues occur)
        cv2.imshow("reg", reg_img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    # Run main()
    main()
