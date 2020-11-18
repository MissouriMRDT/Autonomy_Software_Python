import core
import logging
import time
import cv2


def main() -> None:
    '''
    Main function for video stream script, tests streaming/recording zed footage
    '''
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    core.zed = core.ZedHandler()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        # Close ZED capture
        core.zed.close()

if __name__ == "__main__":
    # Run main()
    main()
