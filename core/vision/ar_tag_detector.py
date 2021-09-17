import asyncio
from typing import List
from algorithms.AR_tag import Tag
import core
import algorithms
import logging

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

        if len(tags) > 0:
            ar_tags.clear()
            ar_tags.extend(tags)
        else:
            ar_tags.clear()

        logger.debug("Running AR Tag async")

        await asyncio.sleep(1 / core.vision.camera_handler.get_fps())


def is_marker():
    """
    Returns whether there is a visible marker.

    Returns:
    -------------
        detect (bool) - whether or not something was detected
    """
    return len(ar_tags) > 0


def is_gate():
    """
    Returns whether there are multiple visible AR tags. We don't look for
    2 specfically because we don't want a false positive to cause this bool
    to fail and abort immediately.

    Returns:
    -------------
        detect (bool) - whether or not something was detected
    """
    return len(ar_tags) > 1


def get_tags() -> List[Tag]:
    """
    Returns a list of all the tags found.

    Returns:
    --------
        tags - A list of named tuples of the type Tag
    """
    return ar_tags
