import time
from collections import deque
import logging
import struct
import math

import rover_states as rs
import constants
from drivers.rovecomm import RoveCommEthernetUdp
from drivers.drive_board import DriveBoard
from drivers.nav_board import NavBoard
# from algorithms.objecttracking import ObjectTracker
# from algorithms.ColorBasedTracking import ObjectTracker
from algorithms.CannyTracking import ObjectTracker
import algorithms.gps_navigate as gps_nav
import algorithms.marker_search as marker_search
from algorithms.gps_navigate import GPSData
import algorithms.geomath as geomath
import algorithms.followBall as follow_ball

logging = "./logs/" + time.strftime("%Y%m%d-%H%M%S") + ".txt"
loggingFile = open(logging,'w')
loggingFile.write("Beginning Logging at time in file name\n")
loggingFile.close()


# Hardware Setup
rovecomm_node = RoveCommEthernetUdp(logging)
drive = DriveBoard(logging)
nav_board = NavBoard(rovecomm_node, logging)

state_switcher = rs.StateSwitcher(logging)
waypoints = deque()
gps_data = GPSData()
tracker = ObjectTracker(logging)



# Assign callbacks for incoming messages
def add_waypoint_handler(packet_contents):
    latitude, longitude = packet_contents.data
    waypoint = constants.Coordinate(latitude, longitude)
    waypoints.append(waypoint)
    with open(logging, 'a') as f:
        f.write("Add Waypoint lat: " + str(waypoint.latitude) + ", lon: " + str(waypoint.longitude) + "\n")
    print("Added waypoint %s" % (waypoint,))


def enable_autonomy(packet_contents):
    print(state_switcher.state)
    if state_switcher.state == rs.Idle():
        with open(logging, 'a') as f:
            f.write("Enable switching from Idle to Start\n")
        state_switcher.handle_event(rs.AutonomyEvents.START, then=logging.info("Enabling Autonomy"))
        set_gps_waypoint()
        print("Throwing START")
    elif state_switcher.state == rs.Shutdown():
        with open(logging, 'a') as f:
            f.write("Enable switching from Shutdown to Restart\n")
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
    with open(logging, 'a') as f:
        f.write("Clear Waypoint all cleared\n")


def set_gps_waypoint():
    global gps_data
    current_goal = waypoints.popleft()
    gps_data.goal = current_goal
    gps_data.start = nav_board.location()
    with open(logging, 'a') as f:
        f.write("Set Waypoint set new waypoint")


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
        with open(logging, 'a') as f:
            f.write("Navigating goal: " + str(goal) +", start:" + str(start) + "\n")
        if gps_nav.reached_goal(goal, nav_board.location(), start):
            with open(logging, 'a') as f:
                f.write("Navigating Reached Waypoint\n")
            # If there are more points, set the new one and start from top
            if waypoints:
                with open(logging, 'a') as f:
                    f.write("Navigating Continuing to next Waypoint\n")
                print("Reached mid point!")
                set_gps_waypoint()
                continue

            state_switcher.handle_event(rs.AutonomyEvents.REACHED_GPS_COORDINATE,
                                        then=logging.info("GPS coordinate reached"))
            gps_data.start = nav_board.location()
            gps_data.goal = nav_board.location()
            print("Throwing REACHED_GPS_COORDINATE")
            with open(logging, 'a') as f:
                f.write("Navigating Reached final waypoint\n")
            rovecomm_node.write(drive.send_drive(0, 0))
            continue

        left, right = gps_nav.calculate_move(goal, nav_board.location(), start, drive, nav_board)
        with open(logging, 'a') as f:
            f.write("Navigating driving to goal with Left: " + str(left) + ", Right: " + str(right) + "\n")
        print("Drive motors: " + str(left) + ", " + str(right))
        rovecomm_node.write(drive.send_drive(left, right))


    # Search Pattern:
    # Travel in a defined pattern to find the target object, the tennis ball
    elif state_switcher.state == rs.Searching():
        goal, start = gps_data.data()
        with open(logging, 'a') as f:
            f.write("Searching heading to goal: " + str(goal) + ", start: " + str(start) + "\n")
        if gps_nav.reached_goal(goal, nav_board.location(), start):
            goal = marker_search.calculate_next_coordinate(start, nav_board.location())
            lat, lon = nav_board.location()
            with open(logging, 'a') as f:
                f.write("Searching search pattern to goal: " + str(goal) + ", start: " + str(start) + " location: (" + str(lat) + "," + str(lon) + ")\n")
            gps_data.goal = goal

        print("...searching for marker...")
        ball_in_frame, center, radius = tracker.track_ball()
        with open(logging, 'a') as f:
            f.write("Searching ballInFrame: " + str(ball_in_frame) + ", center: " + str(center) + ", radius" + str(radius) + "\n")
        if ball_in_frame:
            rovecomm_node.write(drive.send_drive(0, 0))
            with open(logging, 'a') as f:
                f.write("Searching ball seen, switching to Approaching\n")
            # logging.info("Marker seen at %s with r=%i, locking on..." % (center, radius))
            state_switcher.handle_event(rs.AutonomyEvents.MARKER_SIGHTED)
            print("Throwing MARKER_SIGHTED")
            continue

        left, right = gps_nav.calculate_move(goal, nav_board.location(), start, drive, nav_board)
        print("Drive motors: " + str(left) + ", " + str(right))
        with open(logging, 'a') as f:
            f.write("Searching no ball, continuing search. Driving Left: " + str(left) + ", Right: " + str(right) + "\n")
        rovecomm_node.write(drive.send_drive(left, right))

    # Approach Marker:
    # Travel to the found object
    elif state_switcher.state == rs.ApproachingMarker():
        ball_in_frame, center, radius = tracker.track_ball()
        with open(logging, 'a') as f:
            f.write("Approaching ballInFrame: " + str(ball_in_frame) + ", center: " + str(center) + ", radius: " + str(radius) + "\n")
        
        print("Ball Radius: " + str(radius))
        if ball_in_frame:
            (left, right), distance = follow_ball.drive_to_marker(50, drive, center, radius)
            with open(logging, 'a') as f:
                f.write("Approaching found ball, driving to distance: " + str(distance) + ", with motors Left: " + str(left) + ", Right: " + str(right) + "\n")
            if distance < .5:
                rovecomm_node.write(drive.send_drive(0, 0))
                state_switcher.handle_event(rs.AutonomyEvents.REACHED_MARKER, then=logging.info("Reached Marker"))
                # with open(logging, 'a') as f:
                    # f.write("Approaching already close to ball, not moving. Switching to Reached Marker\n")
                continue
            else:
                print("Driving To: " + str(left) + ", " + str(right))
                rovecomm_node.write(drive.send_drive(left, right))

        else:
            # with open(logging, 'a') as f:
                # f.write("Approaching lost sight of marker, switching from Approaching to Searching\n"
            state_switcher.handle_event(rs.AutonomyEvents.MARKER_UNSEEN)
            print("Throwing MARKER_UNSEEN")
        

    elif state_switcher.state == rs.Shutdown():
        with open(logging, 'a') as f:
            f.write("Shutdown in shutdown state\n")
        pass

    elif state_switcher.state == rs.Idle():
        with open(logging, 'a') as f:
            f.write("Idle state\n")

    time.sleep(.1)
    print(state_switcher.state)
    #with open(logging, 'a') as f:
        #f.write("State: " + str(state_switcher.state) + "\n")


