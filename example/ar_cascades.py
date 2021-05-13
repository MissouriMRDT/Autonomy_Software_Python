import algorithms
import core.vision
import logging
import time


def main() -> None:
    """
    Tests ar tag detection using cascades
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")
    core.vision.feed_handler.add_feed(2, "artag")

    while True:
        reg_img = core.vision.camera_handler.grab_regular()

        # Detect some AR Tags
        tags, reg_img = algorithms.AR_tag.detect_ar_tag(reg_img)

        core.vision.feed_handler.handle_frame("artag", reg_img)

        # Sleep so we only process around when we expect a new frame
        time.sleep(1 / 30)


if __name__ == "__main__":
    # Run main()
    main()
