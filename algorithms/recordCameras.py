import numpy as np
import cv2
import time


try:
    camera1 = cv2.VideoCapture(1)
    camera1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

except Exception:
    raise Exception("Could not connect to camera1")

try:
    camera2 = cv2.VideoCapture(2)
    camera2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

except Exception:
    raise Exception("Could not connect to camera2")

pass
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video1 = "../logs/cameraLeft" + time.strftime("%Y%m%d-%H%M%S") + ".avi"
video2 = "../logs/cameraRight" + time.strftime("%Y%m%d-%H%M%S") + ".avi"
videoOut1 = cv2.VideoWriter(video1, fourcc, 10, (640,480))
videoOut2 = cv2.VideoWriter(video2, fourcc, 10, (640,480))

while True:
    grabbed1, frame1 = camera1.read()
    grabbed2, frame2 = camera2.read()
    videoOut1.write(frame1)
    videoOut2.write(frame2)
    time.sleep(0.05)
    print("Recording Film")


camera1.release()
videoOut1.release()
camera2.release()
videoOut2.release()

