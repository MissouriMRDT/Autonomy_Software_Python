import time
import queue
import logging

import rover_states as rs
from drivers.rovecomm import RoveComm
from drivers.driveBoard import DriveBoard
from drivers.gps.gpsNavboard import GPS
from drivers.mag.compass import Compass
from gpsNavigate import GPSNavigate

# Hardware Setup
rovecomm_node = RoveComm()
drive = DriveBoard(rovecomm_node)
gps = GPS(rovecomm_node)
compass = Compass(rovecomm_node)

state_switcher = rs.StateSwitcher()
waypoints = queue.Queue()
gps_navigator = GPSNavigate(gps, compass, drive)

current_goal = None

while True:

    if state_switcher.state == rs.Idle():
        time.sleep(0.2)
        if not waypoints.empty():
            state_switcher.handle_event(rs.AutonomyEvents.START)
            current_goal = waypoints.get_nowait()
            gps_navigator.setWaypoint(current_goal)

    elif state_switcher.state == rs.Navigating():
        pass

    elif state_switcher.state == rs.Searching():
        pass

    elif state_switcher.state == rs.ApproachingMarker():
        pass

    elif state_switcher.state == rs.Shutdown():
        pass
