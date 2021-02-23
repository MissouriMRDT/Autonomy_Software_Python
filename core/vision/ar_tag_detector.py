import core
import algorithms
import cv2
import asyncio

# Dict to hold the obstacle info
ar_tag = {"detected": False, "tags": None}

tag_cascade = cv2.CascadeClassifier("algorithms/cascade30.xml")


async def async_ar_tag_detector():
    """
    Async function to find obstacles
    """
    while True:
        reg_img = core.vision.camera_handler.grab_regular()
        rect_img = reg_img.copy()
        gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)

        tags = tag_cascade.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in tags:
            rect_img = cv2.rectangle(reg_img.copy(), (x, y), (x + w, y + h), (255, 0, 0), 2)

        core.vision.camera_handler.feed_handler.handle_frame("artag", rect_img)

        if len(tags) > 0:
            ar_tag["detected"] = True
            ar_tag["tags"] = tags

        await asyncio.sleep(1 / 30)


def is_ar_tag():
    """
    Returns whether there is an obstacle
    Returns:
    -------------
        detect (bool) - whether or not something was detected
    """
    return ar_tag["detected"]


def get_tags():
    return ar_tag["tags"]