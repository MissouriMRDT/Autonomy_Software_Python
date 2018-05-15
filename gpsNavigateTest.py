from drivers.navBoard import NavBoard
from drivers.gps.gpsNavboard import GPS
from drivers.rovecomm import RoveComm
from drivers.driveBoard import DriveBoard
from drivers.lidar import LiDAR
from gpsNavigate import GPSNavigate
from algorithms.geoMath import Coordinate
from algorithms.quaternion import Quaternion

import time, struct

# Hardware Setup
rovecomm_node = RoveComm()
drive = DriveBoard(rovecomm_node)
gps = GPS(rovecomm_node)
navBoard = NavBoard(rovecomm_node)
quaternion = Quaternion(navBoard)
lidar = LiDAR(rovecomm_node)

navigate = GPSNavigate(gps, quaternion, drive, lidar)

# RoveComm autonomy control DataIDs
ENABLE_AUTONOMY = 2576
DISABLE_AUTONOMY = 2577
ADD_WAYPOINT = 2578
CLEAR_WAYPOINTS = 2579
WAYPOINT_REACHED = 2580

autonomy_enabled = False

# Assign callbacks for incoming messages
def add_waypoint_handler(packet_contents):
    latitude, longitude = struct.unpack("<dd", packet_contents)
    navigate.setWaypoint(Coordinate(latitude, longitude))

def enable_autonomy(packet_contents):
    global autonomy_enabled
    global drive
    autonomy_enabled = True
    print("Autonomy Enabled")
    drive.enable()

def disable_autonomy(packet_contents):
    global autonomy_enabled
    global drive
    autonomy_enabled = False
    print("Autonomy Disabled :(")
    drive.disable()

    

rovecomm_node.callbacks[ENABLE_AUTONOMY] = enable_autonomy
rovecomm_node.callbacks[DISABLE_AUTONOMY] = disable_autonomy
rovecomm_node.callbacks[ADD_WAYPOINT] = add_waypoint_handler

# Set waypoint to use and use a while loop for update thread

while True:
    while autonomy_enabled:
        print('.', end='')
        if navigate.update_controls():
            autonomy_enabled = False
            drive.disable()
            print()
            print("Autonomy Finished! :)")
            rovecomm_node.send(WAYPOINT_REACHED, contents="")
        time.sleep(.5)
    print("Autonomy in holding pattern...")
    time.sleep(5)
