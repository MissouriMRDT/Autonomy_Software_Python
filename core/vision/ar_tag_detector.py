#
# Mars Rover Design Team
# ar_tag_detector.py
#
# Created on Feb 23, 2021
# Updated on Aug 21, 2022
#

import asyncio
from typing import List
from algorithms.ar_tag import Tag
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

        tags, reg_img = algorithms.ar_tag.detect_ar_tag(reg_img)
        core.vision.feed_handler.handle_frame("artag", reg_img)

        ids = []

        if len(tags) > 0:
            ar_tags.clear()

            for t in tags:
                if t.detected >= FRAMES_DETECTED:
                    ids.append(t.id)
                    ar_tags.append(t)
        else:
            ar_tags.clear()

        logger.debug("Running AR Tag async")

        await asyncio.sleep(1 / core.vision.camera_handler.get_fps())


def clear_tags():
    ar_tags.clear()
    algorithms.ar_tag.detected_tags.clear()


def is_marker():
    """
    Returns whether there is a visible marker.

    :return: detect (bool) - whether something was detected
    """

    return len(ar_tags) > 0


def is_gate():
    """
    Returns whether there are multiple visible AR tags. We don't look for
    2 specifically because we don't want a false positive to cause this bool
    to fail and abort immediately.

    :return: detect (bool) - whether something was detected
    """

    return len(ar_tags) > 1


def get_tags() -> List[Tag]:
    """
    Returns a list of all the tags found.

    :return: tags - A list of named tuples of the type Tag
    """

    return ar_tags
