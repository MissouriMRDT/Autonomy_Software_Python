import time

from drivers.navBoard import NavBoard
from drivers.rovecomm import RoveComm

rovecomm_node = RoveComm()
navBoard = NavBoard(rovecomm_node)

while True:
    print(navBoard.gyroscopeXYZ())
    print(navBoard.accelerometerXYZ())
    print(navBoard.magnetometerXYZ())
    print("...")
    time.sleep(1)
