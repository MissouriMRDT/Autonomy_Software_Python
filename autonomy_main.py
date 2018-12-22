import time
import queue
import logging
import struct

import rover_states as rs
import constants
from drivers.rovecomm import RoveComm
from drivers.drive_board import DriveBoard
from drivers.nav_board import NavBoard
from algorithms.objecttracking import ObjectTracker
import algorithms.followBall
import algorithms.gps_navigate as gps_nav
from algorithms.gps_navigate import GPSData

# Hardware Setup
rovecomm_node = RoveComm()
drive = DriveBoard(rovecomm_node)
nav_board = NavBoard(rovecomm_node)

state_switcher = rs.StateSwitcher()
waypoints = queue.Queue()
gps_data = GPSData()
tracker = ObjectTracker()


# Assign callbacks for incoming messages
def add_waypoint_handler(packet_contents):
    latitude, longitude = struct.unpack("<dd", packet_contents)
    waypoint = constants.Coordinate(latitude, longitude)
    waypoints.put(waypoint)
    print("Added waypoint %s" % (waypoint,))


def enable_autonomy(packet_contents):
    print("What's happening? ")
    print(state_switcher.state)
    print("Seriously")
    if state_switcher.state == rs.Idle():
        state_switcher.handle_event(rs.AutonomyEvents.START, then=logging.info("Enabling Autonomy"))
        set_gps_waypoint()
        print("Throwing START")
    elif state_switcher.state == rs.Shutdown():
        state_switcher.handle_event(rs.AutonomyEvents.RESTART, then=logging.info("Restarting Autonomy"))
        print("Throwing RESTART")

    drive.enable()


def disable_autonomy(packet_contents):
    if state_switcher.state != rs.Shutdown():
        state_switcher.handle_event(rs.AutonomyEvents.ABORT, then=logging.info("Disabling Autonomy"))
        print("Throwing ABORT")
    drive.disable()


def clear_waypoint_handler(packet_contents):
    global waypoints
    waypoints = queue.Queue()
    logging.info("Waypoints Cleared")


def set_gps_waypoint():
    global gps_data
    current_goal = waypoints.get_nowait()
    gps_data.goal = current_goal
    gps_data.start = nav_board.location()


rovecomm_node.callbacks[constants.DataID.ENABLE_AUTONOMY] = enable_autonomy
rovecomm_node.callbacks[constants.DataID.DISABLE_AUTONOMY] = disable_autonomy
rovecomm_node.callbacks[constants.DataID.ADD_WAYPOINT] = add_waypoint_handler
rovecomm_node.callbacks[constants.DataID.CLEAR_WAYPOINTS] = clear_waypoint_handler

state_switcher.state = rs.ApproachingMarker()
drive.enable()

while True:

    if state_switcher.state == rs.Navigating():
        goal, start = gps_data.data()
        if gps_nav.reached_goal(goal, nav_board.location(), start):
            state_switcher.handle_event(rs.AutonomyEvents.REACHED_GPS_COORDINATE,
                                        then=logging.info("GPS coordinate reached"))
            print("Throwing REACHED_GPS_COORDINATE")

            drive.send_drive(0, 0)
            break

        left, right = gps_nav.calculate_move(goal, nav_board.location(), start, drive, nav_board)
        drive.send_drive(left, right)

    elif state_switcher.state == rs.Searching():
        logging.info("Reached vision range, searching for marker...")
        ball_in_frame, center, radius = tracker.track_ball()
        if ball_in_frame:
            logging.info("Marker seen at %s with r=%i, locking on..." % (center, radius))
            state_switcher.handle_event(rs.AutonomyEvents.MARKER_SIGHTED)
            print("Throwing MARKER_SIGHTED")

    elif state_switcher.state == rs.ApproachingMarker():
        ball_in_frame, center, radius = tracker.track_ball()
        if ball_in_frame:
            (left, right), distance = algorithms.followBall.drive_to_marker(50, drive, center, radius)

            if distance < .5:
                state_switcher.handle_event(rs.AutonomyEvents.REACHED_MARKER, then=logging.info("Reached Marker"))
                drive.send_drive(0, 0)
            else:
                print("Driving To: " + str(left) + ", " + str(right))
                drive.send_drive(left, right)

        else:
            state_switcher.handle_event(rs.AutonomyEvents.MARKER_UNSEEN,
                                        then=logging.info("Visual lock lost"))
            print("Throwing MARKER_UNSEEN")

    elif state_switcher.state == rs.Shutdown():
        pass

    time.sleep(1)
    print(state_switcher.state)

