import numpy as np
import cv2
from ArTag import HammingMarker
import ArTagEncoding
import time

print("test")
#test = HammingMarker()
test2 = HammingMarker.generate()
im2 = test2.generate_image()
#im = test.generate_image()
#cv2.imshow("frame", im)
cv2.imwrite("./marker2957.png", im2)
cv2.waitKey(1)
time.sleep(5)
print("test")
print("test")
