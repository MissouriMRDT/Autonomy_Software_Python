import core 
import cv2

tag_cascade = cv2.CascadeClassifier("cascade.xml")


def detect_ar_tag():
    reg_img = core.vision.camera_handler.grab_regular()

    gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)
    
    tags = tag_cascade.detectMultiScale(gray, 1.3, 5)
    
    return tags
