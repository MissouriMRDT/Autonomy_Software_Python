import time
from collections import deque
import logging

import core.rover_states as rs
import core.constants as constants
from core.constants import ApproachState
from core.rovecomm import RoveCommEthernetUdp
from interfaces.drive_board import DriveBoard
from interfaces.nav_board import NavBoard
# from algorithms.objecttracking import ObjectTracker
from algorithms.color_based_tracking import ObjectTracker
# from algorithms.CannyTracking import ObjectTracker
import algorithms.gps_navigate as gps_nav
import algorithms.marker_search as marker_search
from algorithms.gps_navigate import GPSData
import algorithms.follow_marker as follow_marker

# Hardware Setup
rovecomm_node = RoveCommEthernetUdp()
drive = DriveBoard()
nav_board = NavBoard(rovecomm_node)

state_switcher = rs.StateSwitcher()
waypoints = deque()
gps_data = GPSData()
tracker = ObjectTracker()


# Assign callbacks for incoming messages
def add_waypoint_handler(packet_contents):
    latitude, longitude = packet_contents.data
    waypoint = constants.Coordinate(latitude, longitude)
    waypoints.append(waypoint)
    print("Added waypoint %s" % (waypoint,))


def enable_autonomy(packet_contents):
    print(state_switcher.state)
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
    waypoints = deque()
    logging.info("Waypoints Cleared")


def set_gps_waypoint():
    global gps_data
    current_goal = waypoints.popleft()
    gps_data.goal = current_goal
    gps_data.start = nav_board.location()


# Define the callbacks to execute when rovecomm recieves the command
rovecomm_node.callbacks[constants.DataID.ENABLE_AUTONOMY] = enable_autonomy
rovecomm_node.callbacks[constants.DataID.DISABLE_AUTONOMY] = disable_autonomy
rovecomm_node.callbacks[constants.DataID.ADD_WAYPOINT] = add_waypoint_handler
rovecomm_node.callbacks[constants.DataID.CLEAR_WAYPOINTS] = clear_waypoint_handler

# state_switcher.state = rs.ApproachingMarker()
# drive.enable()

time.sleep(2)

while True:

    # GPS Navigation:
    # Travel Point to Point (P2P) from the current GPS to the target given from Basestation
    if state_switcher.state == rs.Navigating():
        goal, start = gps_data.data()
        if gps_nav.get_approach_status(goal, nav_board.location(), start) != ApproachState.APPROACHING:

            # If there are more points, set the new one and start from top
            if waypoints.count > 0:
                print("Reached mid point!")
                set_gps_waypoint()
                continue

            state_switcher.handle_event(rs.AutonomyEvents.REACHED_GPS_COORDINATE,
                                        then=logging.info("GPS coordinate reached"))
            gps_data.start = nav_board.location()
            gps_data.goal = nav_board.location()
            print("Throwing REACHED_GPS_COORDINATE")

            rovecomm_node.write(drive.send_drive(0, 0))
            continue

        left, right = gps_nav.calculate_move(goal, nav_board.location(), start, drive, nav_board)
        rovecomm_node.write(drive.send_drive(left, right))

    # Search Pattern:
    # Travel in a defined pattern to find the target object, the tennis ball
    elif state_switcher.state == rs.Searching():
        goal, start = gps_data.data()
        if gps_nav.get_approach_status(goal, nav_board.location(), start) != ApproachState.APPROACHING:
            goal = marker_search.calculate_next_coordinate(start, nav_board.location())
            gps_data.goal = goal

        print("...searching for marker...")
        ball_in_frame, center, radius = tracker.track_ball()

        if ball_in_frame:
            rovecomm_node.write(drive.send_drive(0, 0))
            logging.info("Marker seen at %s with r=%i, locking on..." % (center, radius))
            state_switcher.handle_event(rs.AutonomyEvents.MARKER_SIGHTED)
            print("Throwing MARKER_SIGHTED")
            continue

        left, right = gps_nav.calculate_move(goal, nav_board.location(), start, drive, nav_board)
        rovecomm_node.write(drive.send_drive(left, right))

    # Approach Marker:
    # Travel to the found object
    elif state_switcher.state == rs.ApproachingMarker():
        ball_in_frame, center, radius = tracker.track_ball()
        print("Ball Radius: " + str(radius))
        if ball_in_frame:
            (left, right), distance = follow_marker.drive_to_marker(50, drive, center, radius)

            if distance < .5:
                rovecomm_node.write(drive.send_drive(0, 0))
                state_switcher.handle_event(rs.AutonomyEvents.REACHED_MARKER, then=logging.info("Reached Marker"))
                continue
            else:
                print("Driving To: " + str(left) + ", " + str(right))
                rovecomm_node.write(drive.send_drive(left, right))

        else:
            state_switcher.handle_event(rs.AutonomyEvents.MARKER_UNSEEN,
                                        then=logging.info("Visual lock lost"))
            print("Throwing MARKER_UNSEEN")

    elif state_switcher.state == rs.Shutdown():
        pass

    time.sleep(.1)
    print(state_switcher.state)
