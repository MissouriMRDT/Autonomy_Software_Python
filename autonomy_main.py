import time
import queue
import logging

import rover_states as rs
import constants
from drivers.rovecomm import RoveComm
from drivers.driveBoard import DriveBoard
from drivers.gps.gpsNavboard import GPS
from drivers.mag.compass import Compass
from gpsNavigate import GPSNavigate
from algorithms.objecttracking import ObjectTracker

# Hardware Setup
rovecomm_node = RoveComm()
drive = DriveBoard(rovecomm_node)
gps = GPS(rovecomm_node)
compass = Compass(rovecomm_node)

state_switcher = rs.StateSwitcher()
waypoints = queue.Queue()
gps_navigator = GPSNavigate(gps, compass, drive)
tracker = ObjectTracker()


def set_gps_waypoint():
    current_goal = waypoints.get_nowait()
    gps_navigator.setWaypoint(current_goal)


while True:

    if state_switcher.state == rs.Idle():
        time.sleep(0.2)
        if not waypoints.empty():
            state_switcher.handle_event(rs.AutonomyEvents.START, then=set_gps_waypoint())

    elif state_switcher.state == rs.Navigating():
        reached_goal = gps_navigator.update_controls()
        time.sleep(0.01)
        if reached_goal:
            state_switcher.handle_event(rs.AutonomyEvents.REACHED_GPS_COORDINATE,
                                        then=logging.info("GPS coordinate reached"))

    elif state_switcher.state == rs.Searching():
        if gps_navigator.distance_to_goal < constants.VISION_RANGE and waypoints.empty():
            logging.info("Reached vision range, searching for marker...")
            ball_in_frame, center, radius = tracker.track_ball()
            if ball_in_frame:
                logging.info("Marker seen at %s with r=%i, locking on..." % (center, radius))
                state_switcher.handle_event(rs.AutonomyEvents.MARKER_SIGHTED)

    elif state_switcher.state == rs.ApproachingMarker():
        ball_in_frame, center, radius = tracker.track_ball()
        if ball_in_frame:
            left, right, distance = drive_to_marker(drive, tracker, center, radius)

        else:
            state_switcher.handle_event(rs.AutonomyEvents.MARKER_UNSEEN,
                                        then=logging.info("Visual lock lost"))

    elif state_switcher.state == rs.Shutdown():
        pass
