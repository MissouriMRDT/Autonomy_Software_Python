import time

from drivers.nav_board import NavBoard
from drivers.rovecomm import RoveComm

rovecomm_node = RoveComm()
navBoard = NavBoard(rovecomm_node)

while True:
    print(navBoard.gyroscope_xyz())
    print(navBoard.accelerometer_xyz())
    print(navBoard.magnetometer_xyz())
    print("...")
    time.sleep(1)
