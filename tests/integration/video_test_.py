import numpy as np
import cv2
import time

print("enter a filename to be run through an openCV filter: ", end='')
filename = input()

cap = cv2.VideoCapture(filename)

if not cap.isOpened():
    print("Error opening file")

else:
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret:
            cv2.imshow("frame",frame)
            cv2.waitKey(100)
    else:
        ret = False

cap.release()

cv2.destroyAllWindows()
