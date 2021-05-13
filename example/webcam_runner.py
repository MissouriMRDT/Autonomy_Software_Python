import time
import core.vision
import logging


def main() -> None:
    """
    Main function for video stream script, tests streaming/recording camera footage
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    # Give the system a second to set everything up, start reading in frames
    time.sleep(1)
    core.vision.feed_handler.add_feed(2, "artag")

    while True:
        # Test grabbing the latest camera frames
        reg_img = core.vision.camera_handler.grab_regular()

        core.vision.feed_handler.handle_frame("artag", reg_img)


if __name__ == "__main__":
    # Run main()
    main()
