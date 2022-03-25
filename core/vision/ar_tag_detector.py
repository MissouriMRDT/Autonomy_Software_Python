import asyncio
from typing import List
from algorithms.AR_tag import Tag
import core
import algorithms
import logging

# Dict to hold the obstacle info
ar_tags = []
ar_ids = []


async def async_ar_tag_detector():
    """
    Async function to find obstacles.
    """
    logger = logging.getLogger(__name__)
    while True:
        reg_img = core.vision.camera_handler.grab_regular()

        tags, reg_img, ids = algorithms.AR_tag.detect_ar_tag(reg_img)

        core.vision.feed_handler.handle_frame("artag", reg_img)

        if core.waypoint_handler.gps_data:
            if core.waypoint_handler.gps_data.leg_type == "MARKER" and len(tags) > 0:
                ar_tags.clear()
                ar_ids.clear()
                ar_tags.extend(tags)
                ar_ids.extend(ids)
            elif core.waypoint_handler.gps_data.leg_type == "GATE" and len(tags) > 1:
                ar_tags.clear()
                ar_ids.clear()
                ar_tags.extend(tags)
                ar_ids.extend(ids)
        # else:
        #     ar_tags.clear()
        #     ar_ids.clear()

        logger.debug("Running AR Tag async")
        logger.info(f"Tags Spotted: {ar_ids}")

        await asyncio.sleep(1 / core.vision.camera_handler.get_fps())


def clear_tags():
    ar_tags.clear()
    ar_ids.clear()


def is_marker():
    """
    Returns whether there is a visible marker.

    Returns:
    -------------
        detect (bool) - whether or not something was detected
    """
    flag = len(ar_tags) > 0 and ar_ids[0] in [0, 1, 2, 3]
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
    flag = len(ar_tags) >= 2 and ar_ids[0] in [4, 5] and ar_ids[1] in [4, 5]
    logger = logging.getLogger(__name__)

    logger.info(f"1111111111111111111111111111{flag}")
    return flag


def get_tags() -> List[Tag]:
    """
    Returns a list of all the tags found.

    Returns:
    --------
        tags - A list of named tuples of the type Tag
    """
    return ar_tags
