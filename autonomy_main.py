import time
import queue
import logging
import struct

import rover_states as rs
import constants
from drivers.rovecomm import RoveComm
from drivers.drive_board import DriveBoard
from drivers.nav_board import NavBoard
from gpsNavigate import GPSNavigate
from algorithms.objecttracking import ObjectTracker
import algorithms.followBall

# Hardware Setup
rovecomm_node = RoveComm()
drive = DriveBoard(rovecomm_node)
nav_board = NavBoard(rovecomm_node)

state_switcher = rs.StateSwitcher()
waypoints = queue.Queue()
gps_navigator = GPSNavigate(nav_board, "", drive)
tracker = ObjectTracker()


# Assign callbacks for incoming messages
def add_waypoint_handler(packet_contents):
    latitude, longitude = struct.unpack("<dd", packet_contents)
    waypoint = constants.Coordinate(latitude, longitude)
    waypoints.put(waypoint)
    logging.info("Added waypoint %s" % (waypoint,))


def enable_autonomy(packet_contents):
    if state_switcher.state == rs.Shutdown:
        state_switcher.handle_event(rs.AutonomyEvents.START, then=logging.info("Enabling Autonomy"))
    drive.enable()


def disable_autonomy(packet_contents):
    if state_switcher.state != rs.Shutdown:
        state_switcher.handle_event(rs.AutonomyEvents.ABORT, then=logging.info("Disabling Autonomy"))
    drive.disable()


def clear_waypoint_handler(packet_contents):
    global waypoints
    waypoints = queue.Queue()
    logging.info("Waypoints Cleared")


def set_gps_waypoint():
    current_goal = waypoints.get_nowait()
    gps_navigator.setWaypoint(current_goal)


rovecomm_node.callbacks[constants.DataID.ENABLE_AUTONOMY] = enable_autonomy
rovecomm_node.callbacks[constants.DataID.DISABLE_AUTONOMY] = disable_autonomy
rovecomm_node.callbacks[constants.DataID.ADD_WAYPOINT] = add_waypoint_handler
rovecomm_node.callbacks[constants.DataID.CLEAR_WAYPOINTS] = clear_waypoint_handler


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
            left, right, distance = algorithms.followBall.drive_to_marker(drive, tracker, center, radius)

        else:
            state_switcher.handle_event(rs.AutonomyEvents.MARKER_UNSEEN,
                                        then=logging.info("Visual lock lost"))

    elif state_switcher.state == rs.Shutdown():
        pass
