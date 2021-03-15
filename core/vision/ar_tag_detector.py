import asyncio
from typing import List
from algorithms.AR_tag import Tag
import core
import algorithms

# Dict to hold the obstacle info
ar_tag = {}


async def async_ar_tag_detector():
    """
    Async function to find obstacles.
    """
    while True:
        reg_img = core.vision.camera_handler.grab_regular()

        tags, reg_img = algorithms.AR_tag.detect_ar_tag(reg_img)

        core.vision.feed_handler.handle_frame("artag", reg_img)

        if len(tags) > 0:
            ar_tag["detected"] = True
            ar_tag["tags"] = tags
        else:
            ar_tag["detected"] = False
            ar_tag["tags"] = []

        await asyncio.sleep(1 / 30)


def is_ar_tag():
    """
    Returns whether there is an obstacle.

    Returns:
    -------------
        detect (bool) - whether or not something was detected
    """
    return ar_tag["detected"]


def get_tags() -> List[Tag]:
    """
    Returns a list of all the tags found.

    Returns:
    --------
        tags - A list of named tuples of the type Tag
    """
    return ar_tag["tags"]
