import asyncio
import core
import algorithms
import logging
import traceback
from core.constants import ARUCO_FRAMES_DETECTED

# Dict to hold the obstacle info
ar_tags = []


async def async_ar_tag_detector():
    """
    Async function to find obstacles.
    """
    # Setup logger for function.
    logger = logging.getLogger(__name__)

    # Declare detection objects.
    TagDetector = algorithms.ar_tag.ArucoARTagDetector()

    while True:
        # Maybe this is bad practice but it's helpful.
        try:
            # Get normal image from camera.
            reg_img = core.vision.camera_handler.grab_regular()

            # Detect tags.
            TagDetector.detect_ar_tag(reg_img)
            # Filter tags.
            # TagDetector.filter_ar_tags()

            # Draw tags.
            reg_img = TagDetector.track_ar_tags(reg_img)
            core.vision.feed_handler.handle_frame("artag", reg_img)

            # # Print info about tags if 1 or more detected.
            # if len(tags) >= 1:
            #     output = f"{len(tags)} Tags Detected: "
            #     for tag in tags:
            #         output += f"ID: {tag.id}\n"
            #     logger.info(output)

            # ids = []

            # if core.waypoint_handler.gps_data:
            #     ar_tags.clear()

            #     if (core.waypoint_handler.gps_data.leg_type == "MARKER" and len(tags) > 0) or (
            #         core.waypoint_handler.gps_data.leg_type == "GATE" and len(tags) > 1
            #     ):
            #         for t in tags:
            #             if t.detected >= ARUCO_FRAMES_DETECTED:
            #                 ids.append(t.id)
            #                 ar_tags.append(t)
            # else:
            #     ar_tags.clear()

            if str(core.states.state_machine.state) == "Idle":
                TagDetector.clear_tags()

        except Exception:
            # Because we are using async functions, they don't print out helpful tracebacks. We must do this instead.
            logger.critical(traceback.format_exc())

        await asyncio.sleep(1 / core.vision.camera_handler.get_fps())


def clear_tags():
    """
    Clears the tag list.

    :returns: None
    """
    ar_tags.clear()


def is_marker():
    """
    Returns whether there is a visible marker.

    :return: detect (bool) - whether or not something was detected
    """
    flag = len(ar_tags) > 0 and ar_tags[0].id in [0, 1, 2, 3]
    # flag = False
    return flag


def is_gate():
    """
    Returns whether there are multiple visible AR tags. We don't look for
    2 specfically because we don't want a false positive to cause this bool
    to fail and abort immediately.

    :return: detect (bool) - whether or not something was detected
    """
    flag = len(ar_tags) >= 2 and ar_tags[0].id in [4, 5] and ar_tags[1].id in [4, 5]

    return flag


def get_tags():
    """
    Returns a list of all the tags found.

    :return: tags - A list of class objects of the type Tag
    """
    return ar_tags
