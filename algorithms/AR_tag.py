import core 
import cv2

tag_cascade = cv2.CascadeClassifier("algorithms/cascade30.xml")


def detect_ar_tag():
    reg_img = core.vision.camera_handler.grab_regular()
    rect_img = reg_img.copy()
    gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)
    
    tags = tag_cascade.detectMultiScale(gray, 1.3, 5)
    
    for (x, y, w, h) in tags:
        rect_img = cv2.rectangle(reg_img.copy(), (x, y), (x + w, y + h), (255, 0, 0), 2)
    
    core.vision.camera_handler.feed_handler.handle_frame("artag", rect_img)

    return tags
