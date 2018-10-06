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

current_goal = None

while True:

    if state_switcher.state == rs.Idle():
        time.sleep(0.2)
        if not waypoints.empty():
            state_switcher.handle_event(rs.AutonomyEvents.START)
            current_goal = waypoints.get_nowait()
            gps_navigator.setWaypoint(current_goal)

    elif state_switcher.state == rs.Navigating():
        reached_goal = gps_navigator.update_controls()
        time.sleep(0.01)
        if reached_goal:
            state_switcher.handle_event(rs.AutonomyEvents.REACHED_GPS_COORDINATE)

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
            angle_to_ball = constants.FIELD_OF_VIEW * ((center[0] - constants.WIDTH / 2) / constants.WIDTH)
            distance = constants.SCALING_FACTOR / radius
            logging.info("Distance to marker: %f" % distance)
            if distance > constants.TARGET_DISTANCE:
                logging.info("Angle to marker: %f" % angle_to_ball)
                drive.move(constants.DRIVE_POWER, angle_to_ball)
            if distance <= constants.TARGET_DISTANCE:
                state_switcher.handle_event(rs.AutonomyEvents.REACHED_MARKER)
                drive.move(-constants.DRIVE_POWER, angle_to_ball)
        else:
            logging.info("Visual lock lost")
            state_switcher.handle_event(rs.AutonomyEvents.MARKER_UNSEEN)

    elif state_switcher.state == rs.Shutdown():
        pass
