import asyncio
from collections import namedtuple
from typing import List
from algorithms.AR_tag import Tag
import core
import algorithms
import logging
from core.constants import FRAMES_DETECTED

# Dict to hold the obstacle info
ar_tags = []


async def async_ar_tag_detector():
    """
    Async function to find obstacles.
    """

    logger = logging.getLogger(__name__)
    while True:
        reg_img = core.vision.camera_handler.grab_regular()

        tags, reg_img = algorithms.AR_tag.detect_ar_tag(reg_img)
        core.vision.feed_handler.handle_frame("artag", reg_img)

        ids = []

        if core.waypoint_handler.gps_data:
            ar_tags.clear()

            if (core.waypoint_handler.gps_data.leg_type == "MARKER" and len(ids) > 0) or (core.waypoint_handler.gps_data.leg_type == "GATE" and len(ids) > 1):
                for t in tags:
                    if t.dectected >= FRAMES_DETECTED:
                        ids.append(t.id)
                        ar_tags.append(t)
        # else:
        #     ar_tags.clear()

        logger.debug("Running AR Tag async")
        logger.info(f"Tags Spotted: {ids}")

        await asyncio.sleep(1 / core.vision.camera_handler.get_fps())


def clear_tags():
    ar_tags.clear()


def is_marker():
    """
    Returns whether there is a visible marker.

    Returns:
    -------------
        detect (bool) - whether or not something was detected
    """
    # flag = len(ar_tags) > 0 and ar_ids[0] in [0, 1, 2, 3]
    flag = False
    return flag


def is_gate():
    """
    Returns whether there are multiple visible AR tags. We don't look for
    2 specfically because we don't want a false positive to cause this bool
    to fail and abort immediately.

    Returns:
    -------------
        detect (bool) - whether or not something was detected
    """
    flag = len(ar_tags) >= 2 and ar_tags[0].id in [0, 1] and ar_tags[1].id in [0, 1]
    logger = logging.getLogger(__name__)

    logger.info(f"1111111111111111111111111111{flag}")
    return flag


def get_tags() -> List[Tag]:
    """
    Returns a list of all the tags found.

    Returns:
    --------
        tags - A list of class objects of the type Tag
    """
    return ar_tags
