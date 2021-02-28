import core
import algorithms
import cv2
import asyncio

# Dict to hold the obstacle info
ar_tag = {"detected": False, "tags": None}

#tag_cascade = cv2.CascadeClassifier("algorithms/cascade30.xml")


def async_ar_tag_detector():
    """
    Async function to find obstacles
    """
    while True:
        reg_img = core.vision.camera_handler.grab_regular()
        tag_cascade = cv2.CascadeClassifier("algorithms/cascade30.xml")
        gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)

        tags = tag_cascade.detectMultiScale(gray, 1.05, 5)

        for (x, y, w, h) in tags:
            #print(x, y, w, h)
            reg_img = cv2.rectangle(reg_img, (x, y), (x + w, y + h), (255, 0, 0), 2)


        core.vision.camera_handler.feed_handler.handle_frame("artag", reg_img)

        if len(tags) > 0:
            ar_tag["detected"] = True
            ar_tag["tags"] = tags
            return ar_tag
        return {}


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