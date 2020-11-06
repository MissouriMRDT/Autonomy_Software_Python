import core
import logging
import pyzed.sl as sl
import time

def main() -> None:
    '''
    Main function for example script, tests geomath code
    '''
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    # Add video streams to handler
    core.video_handler = core.VideoHandler()

    core.video_handler.add_video(2, "regular")
    core.video_handler.add_video(3, "depth")

    try:
        while True:
            core.video_handler.handle_frame("regular", img)
    except KeyboardInterrupt:
        core.video_handler.close()

if __name__ == "__main__":
    # Run main()
    main()
