import time
import json

from drivers.nav_board import NavBoard
from drivers.rovecomm import RoveComm
from algorithms.quaternion import Quaternion

headingOffset = 0

rovecomm_node = RoveComm()
navBoard = NavBoard(rovecomm_node)

time.sleep(1)

quaternion = Quaternion(navBoard)
quaternion.headingOffset = 0

time.sleep(1)

print("Rotate slowly 360 degrees... Press enter to continue")
input()

print("Orient towards north... Press enter to continue")
input()

headingOffset = 0 - quaternion.heading

print(" ")
print("headingOffset = ",  headingOffset)
    
calibration = {"gps_offset": headingOffset}
with open('heading_calibration.json', 'w') as calfile:
    json.dump(calibration, calfile)
