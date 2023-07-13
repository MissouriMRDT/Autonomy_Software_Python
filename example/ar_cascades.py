#
# Mars Rover Design Team
# ar_cascades.py
#
# Created on Jan 16, 2021
# Updated on Aug 21, 2022
#

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
        tags, reg_img = algorithms.ar_tag.detect_ar_tag(reg_img)

        core.vision.feed_handler.handle_frame("artag", reg_img)


if __name__ == "__main__":
    # Run main()
    main()
