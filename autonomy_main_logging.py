import time
from collections import deque
import logging
import struct
import math

import rover_states as rs
import constants
from drivers.rovecomm import RoveCommEthernetUdp
from drivers.rovecomm import RoveCommPacket
from drivers.drive_board import DriveBoard
from drivers.nav_board import NavBoard
from algorithms.ColorBasedTracking import ObjectTracker
import algorithms.gps_navigate as gps_nav
import algorithms.marker_search as marker_search
from algorithms.gps_navigate import GPSData
import algorithms.geomath as geomath
import algorithms.followBall as follow_ball

outString = "./logs/" + time.strftime("%Y%m%d-%H%M%S") + ".txt"
outFile = open(outString,'w')
outFile.write(time.strftime("%Y%m%d-%H%M%S") + "\n")
outFile.close()

print(outString)

# Hardware Setup
rovecomm_node = RoveCommEthernetUdp(outString)
print("RoveComm")
drive = DriveBoard(outString)
print("DriveBoard")
nav_board = NavBoard(rovecomm_node, outString)
print("NavBoard")
state_switcher = rs.StateSwitcher(outString)
print("StateSwitcher")
waypoints = deque()
print("waypoint")
gps_data = GPSData()
print("GPSData")
tracker = ObjectTracker()
print("Cameras")
loopDelay = 0.07
print("Setup complete")

# Assign callbacks for incoming messages
def add_waypoint_handler(packet_contents):
    latitude, longitude = packet_contents.data
    waypoint = constants.Coordinate(latitude, longitude)
    waypoints.append(waypoint)
    print("YO im addin a point")
    outFile = open(logging, 'a')
    tempString = time.strftime("%H%M%S") +  " Add Waypoint: lat,lon: " + str(latitude) + "," + str(longitude) + "\n"
    outFile.write(tempString)
    outFile.close()
    print("Added waypoint %s" % (waypoint,))


def enable_autonomy(packet_contents):
    print(state_switcher.state)
    if state_switcher.state == rs.Idle():
        with open(outString, 'a') as f:
            f.write(time.strftime("%H%M%S") + " Enable: switching from Idle to Start\n")
        print("Yo yo de o itsd a hard-no")
        state_switcher.handle_event(rs.AutonomyEvents.START, rs.Idle())
        set_gps_waypoint()
        print("Throwing START")
    elif state_switcher.state == rs.Shutdown():
        with open(outString, 'a') as f:
            f.write(time.strftime("%H%M%S") + " Enable: switching from Shutdown to Restart\n")
        state_switcher.handle_event(rs.AutonomyEvents.RESTART, rs.Shutdown(),  then=logging.info("Restarting Autonomy"))
        print("Throwing RESTART")

    drive.enable()


def disable_autonomy(packet_contents):
    if state_switcher.state != rs.Shutdown():
        state_switcher.handle_event(rs.AutonomyEvents.ABORT, rs.Shutdown(), then=logging.info("Disabling Autonomy"))
        print("Throwing ABORT")
        with open(outString, 'a') as f:
            f.write(time.strftime("%H%M%S") + " Shutdown event\n")
    drive.disable()


def clear_waypoint_handler(packet_contents):
    global waypoints
    waypoints = deque()
    logging.info("Waypoints Cleared")
    with open(outString, 'a') as f:
        f.write(time.strftime("%H%M%S") + " Clear Waypoint all cleared\n")


def set_gps_waypoint():
    global gps_data
    print(waypoints)
    current_goal = waypoints.popleft()
    gps_data.goal = current_goal
    gps_data.start = nav_board.location()
    with open(outString, 'a') as f:
        f.write(time.strftime("%H%M%S") + " Set Waypoint: set new waypoint\n")


# Define the callbacks to execute when rovecomm recieves the command
rovecomm_node.callbacks[constants.DataID.ENABLE_AUTONOMY] = enable_autonomy
rovecomm_node.callbacks[constants.DataID.DISABLE_AUTONOMY] = disable_autonomy
rovecomm_node.callbacks[constants.DataID.ADD_WAYPOINT] = add_waypoint_handler
rovecomm_node.callbacks[constants.DataID.CLEAR_WAYPOINTS] = clear_waypoint_handler

# gps_data.goal = constants.Coordinate(nav_board._location[0],nav_board._location[1] )
# gps_data.start = constants.Coordinate(nav_board._location[0],nav_board._location[1])
# nav_board._location = constants.Coordinate(0,0)

# these two are for testing ball tracking
# state_switcher.state = rs.Searching()
# drive.enable()

# looping = 0
# test = False
# state_switcher.handle_event(rs.AutonomyEvents.OBSTACLE_AVOIDANCE, rs.ApproachingMarker())
# drive.enable()
# state_switcher.handle_event(rs.AutonomyEvents.START, rs.Idle())



time.sleep(2)

while True:

    # GPS Navigation:
    # Travel Point to Point (P2P) from the current GPS to the target given from Basestation
    if state_switcher.state == rs.Navigating():
        goal, start = gps_data.data()
        ball_in_frame, center, radius = tracker.track_ball()
        with open(outString, 'a') as f:
            temp = nav_board._location
            f.write(time.strftime("%H%M%S") + " Navigating: Driving to " + str(goal[0]) + "," + str(goal[1]) + " from " + str(start[0]) + "," + str(start[1]) + ". Currently at: " + str(temp[0]) + "," + str(temp[1]) + "\n")
        # looping += 1
        # if looping > 25:
            # looping = 0
            # state_switcher.handle_event(rs.AutonomyEvents.REACHED_GPS_COORDINATE, rs.Navigating())
        if gps_nav.reached_goal(goal, nav_board.location(), start):
            with open(outString, 'a') as f:
                f.write(time.strftime("%H%M%S") + " Navigating: reached goal\n")
            # If there are more points, set the new one and start from top
            if waypoints:
                
                print("Reached mid point!")
                set_gps_waypoint()
                with open(outString, 'a') as f:
                    f.write(time.strftime("%H%M%S") + " Navigating: Reached midpoint, grabbing new point " + str(goal[0]) + "," + str(goal[1]) + "\n")
                continue

            state_switcher.handle_event(rs.AutonomyEvents.REACHED_GPS_COORDINATE, rs.Navigating(),
                                        then=logging.info("GPS coordinate reached"))
            gps_data.start = nav_board.location()
            gps_data.goal = nav_board.location()
            print("Throwing REACHED_GPS_COORDINATE")
            
            rovecomm_node.write(drive.send_drive(0, 0))
            # rovecomm_node.write(RoveCommPacket(1000, 'h', (0,0), ip_octet_4=140))
            continue

        left, right = gps_nav.calculate_move(goal, nav_board.location(), start, drive, nav_board)
        # time.sleep(loopDelay * 3)
        with open(outString, 'a') as f:
            f.write(time.strftime("%H%M%S") + " Navigating: Driving at " + str(left) + "," + str(right) + "\n")
        print("Drive motors: " + str(left) + ", " + str(right))
        rovecomm_node.write(drive.send_drive(left, right))
        # rovecomm_node.write(RoveCommPacket(1000, 'h', (0,0), ip_octet_4=140))


    # Search Pattern:
    # Travel in a defined pattern to find the target object, the tennis ball
    elif state_switcher.state == rs.Searching():
        goal, start = gps_data.data()
        print(goal)
        
        if gps_nav.reached_goal(goal, nav_board.location(), start):
            goal = marker_search.calculate_next_coordinate(start, nav_board.location())
            lat, lon = nav_board.location()
            
            gps_data.goal = goal

        print("...searching for marker...")
        ball_in_frame, center, radius = tracker.track_ball()

        with open(outString, 'a') as f:
            f.write(time.strftime("%H%M%S") + " Searching: Ball Sighted, " + str(ball_in_frame) + ". With center and radius " + str(center) + "," + str(radius) + "\n")

        # if ball_in_frame:
            # rovecomm_node.write(drive.send_drive(0, 0))
            
            # logging.info("Marker seen at %s with r=%i, locking on..." % (center, radius))
            # state_switcher.handle_event(rs.AutonomyEvents.MARKER_SIGHTED, rs.Searching())
            # with open(outString, 'a') as f:
                # f.write(time.strftime("%H%M%S") + " Searching: Marker Sighted, entering ApproachingMarker()\n")
            # print("Throwing MARKER_SIGHTED")
            # continue

        left, right = gps_nav.calculate_move(goal, nav_board.location(), start, drive, nav_board)
        print("Drive motors: " + str(left) + ", " + str(right))
        with open(outString, 'a') as f:
            f.write("Searching: " + str(left) + "," + str(right) + " \n")
        rovecomm_node.write(drive.send_drive(left, right))
        # rovecomm_node.write(RoveCommPacket(1000, 'h', (0,0), ip_octet_4=140))

    # Approach Marker:
    # Travel to the found object
    elif state_switcher.state == rs.ApproachingMarker():
        # if test == False:
            # print("Testing obstacle avoidance")
            # test = True    
            # state_switcher.handle_event(rs.AutonomyEvents.OBSTACLE_AVOIDANCE, rs.ApproachingMarker())
            # continue
        ball_in_frame, center, radius = tracker.track_ball()
        
        
        print("Ball Radius: " + str(radius))
        if ball_in_frame:
            (left, right), distance = follow_ball.drive_to_marker(125, drive, center, radius, nav_board)
            print("Ball in Frame")
            # distance = 2 # for testing, comment out later
            if distance < .5:
                rovecomm_node.write(drive.send_drive(0, 0))
                # rovecomm_node.write(RoveCommPacket(1000, 'h', (0,0), ip_octet_4=140))
                # state_switcher.handle_event(rs.AutonomyEvents.REACHED_MARKER, rs.ApproachingMarker(), then=logging.info("Reached Marker")) # commented for testing, remove the commenting for running
                with open(outString, 'a') as f:
                    f.write(time.strftime("%H%M%S") + " ApproachingMarker: Reached Marker, entering Idle()\n")
                continue
            else:
                print("Driving To: " + str(left) + ", " + str(right))
                with open(outString, 'a') as f:
                    f.write(time.strftime("%H%M%S") + " ApproachingMarker: Not reached marker, driving to " + str(left) + "," + str(right) + "\n")
                rovecomm_node.write(drive.send_drive(left, right))
                # rovecomm_node.write(RoveCommPacket(1000, 'h', (0,0), ip_octet_4=140))

        # else: # commented for testing, remove the commenting for running.
            # state_switcher.handle_event(rs.AutonomyEvents.MARKER_UNSEEN, rs.ApproachingMarker())
            # with open(outString, 'a') as f:
                # f.write(time.strftime("%H%M%S") + " ApproachingMarker: lost sight of marker, returning to Searching()\n")
            # print("Throwing MARKER_UNSEEN")
        

    elif state_switcher.state == rs.Shutdown():
        pass

    elif state_switcher.state == rs.Idle():
        #state_switcher.handle_event(rs.AutonomyEvents.START, rs.Idle())
       
        pass

    elif state_switcher.state == rs.ObstacleAvoidance():
        with open(outString, 'a') as f:
            f.write(time.strftime("%H%M%S") + " ObstacleAvoidance: starting avoidance movement\n")
        rovecomm_node.write(drive.send_drive(0,0))
        # print(drive.send_drive(0,0))
        # rovecomm_node.write(RoveCommPacket(1000, 'h', (0,0), ip_octet_4=140))
        start = nav_board._location
        rovecomm_node.write(drive.send_drive(-100,-100)) # tune this drive number
        # rovecomm_node.write(RoveCommPacket(1000, 'h', (-100,-100), ip_octet_4=140))
        junk, distance = geomath.haversine(start[0], start[1], nav_board._location[0], nav_board._location[1])
        while distance < 0.002:
                if state_switcher.state != rs.ObstacleAvoidance():
                    rovecomm_node.write(drive.send_drive(0,0))
                    continue
                junk, distance = geomath.haversine(start[0], start[1], nav_board._location[0], nav_board._location[1])
                print (distance)
                time.sleep(loopDelay)
                # distance = 3
        rovecomm_node.write(drive.send_drive(0,0))
        with open(outString, 'a') as f:
            f.write(time.strftime("%H%M%S") + " ObstacleAvoidance: finished reversing, finding new waypoint.\n")
        # rovecomm_node.write(RoveCommPacket(1000, 'h', (0,0), ip_octet_4=140))
        r = 6371 # radius of earth
        brng = math.radians(nav_board._heading - 90) # target heading
        d = 0.005 # 5 meters
        lat1 = math.radians(nav_board._location[0])
        lon1 = math.radians(nav_board._location[1])
        lat2 = math.asin(math.sin(lat1)*math.cos(d/r) + math.cos(lat1)*math.sin(d/r)*math.sin(brng))
        lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/r)*math.cos(lat1),math.cos(d/r)-math.sin(lat1)*math.sin(lat2))
        lat2 = math.degrees(lat2)
        lon2 = math.degrees(lon2)
        target = constants.Coordinate(lat2, lon2)
        # we could turn everything from the declaration of r to here into a function and call it as such
        with open(outString, 'a') as f:
            f.write(time.strftime("%H%M%S") + " ObstacleAvoidance: new target " + str(target[0]) + "," + str(target[1]) + "\n")
        print(target)
        left, right = gps_nav.calculate_move(target, nav_board.location(), start, drive, nav_board)
        # if nav_board._distToGround > constants.LIDAR_MAXIMUM:
            # rovecomm_node.write(drive.send_drive(0,0))
            # continue
        rovecomm_node.write(drive.send_drive(left,right))
        distance = 10
        while distance > gps_nav.WAYPOINT_DISTANCE_THRESHOLD * 0.001: # the value in gps_nav is currently in kilometers as of 5/17/19, temporarily fixing this here.
            # add ball tracking after we turn away from our initial heading.
            if state_switcher.state != rs.ObstacleAvoidance():
                rovecomm_node.write(drive.send_drive(0,0))
                with open(outString, 'a') as f:
                    f.write(time.strftime("%H%M%S") + " ObstacleAvoidance: Shutdown thrown, sending 0,0 drive and stopping\n")
                continue
            time.sleep(loopDelay)
            left, right = gps_nav.calculate_move(target, nav_board.location(), start, drive, nav_board)
            rovecomm_node.write(drive.send_drive(left,right))
            junk, distance = geomath.haversine(nav_board._location[0], nav_board._location[1], target[0], target[1])
            # distance = 0.0
            # if nav_board._distToGround > constants.LIDAR_MAXIMUM:
                # rovecomm_node.write(drive.send_drive(0,0))
                # continue
        print("End of Avoidance")
        with open(outString, 'a') as f:
            f.write(time.strftime("%H%M%S") + " ObstacleAvoidance: Leaving obstacle avoidance, returning to " + str(state_switcher.previousState) + "\n")
        state_switcher.handle_event(rs.AutonomyEvents.END_OBSTACLE_AVOIDANCE, rs.ObstacleAvoidance())


    time.sleep(loopDelay)
    # time has been tuned from 0.1 seconds to 0.07 seconds to allow approximately 10 runs per second with additional logging code
    print(state_switcher.state)


