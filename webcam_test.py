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

    core.video_handler.add_video(sl.VIEW.DEPTH, sl.MAT_TYPE.U8_C4, 2, "regular")
    core.video_handler.add_video(sl.VIEW.LEFT, sl.MAT_TYPE.U8_C4, 3, "depth")
    #core.video_handler.start()

    try:
        while True:
            core.video_handler.update()
    except KeyboardInterrupt:
        core.video_handler.close()

if __name__ == "__main__":
    # Run main()
    main()
