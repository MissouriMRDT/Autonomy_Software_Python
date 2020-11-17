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
    core.feed_handler = core.FeedHandler()

    # Add video streams to handler
    core.feed_handler.add_feed(2, "regular")
    core.feed_handler.add_feed(3, "depth")

    core.zed = core.ZedHandler()

    try:
        while True:
            reg, depth = core.zed.grab_frames()
            core.feed_handler.handle_frame("regular", reg)
            core.feed_handler.handle_frame("depth", depth)

    except KeyboardInterrupt:
        # Close all open video streams
        core.feed_handler.close()
        # Close  ZED capture
        core.zed.close()

if __name__ == "__main__":
    # Run main()
    main()
