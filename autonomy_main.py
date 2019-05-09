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

#logging = "./logs/" + time.strftime("%Y%m%d-%H%M%S") + ".txt"
#loggingFile = open(logging,'w')
#loggingFile.write("Beginning Logging at time in file name\n")
#loggingFile.close()

#print(logging)

# Hardware Setup
rovecomm_node = RoveCommEthernetUdp("hi")
drive = DriveBoard("hi")
nav_board = NavBoard(rovecomm_node, "hi")

state_switcher = rs.StateSwitcher("hi")
waypoints = deque()
gps_data = GPSData()
tracker = ObjectTracker()

print("Setup complete")

# Assign callbacks for incoming messages
def add_waypoint_handler(packet_contents):
    latitude, longitude = packet_contents.data
    waypoint = constants.Coordinate(latitude, longitude)
    waypoints.append(waypoint)
    print("YO im addin a point")
    #loggingFile = open(logging, 'a')
    #log = "Add waypoint lat,lon: " + str(latitude) + "," + str(longitude) + "\n"
    #loggingFile.write(log)
    #loggingFile.close()
    print("Added waypoint %s" % (waypoint,))


def enable_autonomy(packet_contents):
    print(state_switcher.state)
    if state_switcher.state == rs.Idle():
       # with open(logging, 'a') as f:
        #    f.write("Enable switching from Idle to Start\n")
        print("Yo yo de o itsd a hard-no")
        state_switcher.handle_event(rs.AutonomyEvents.START, rs.Idle())
        set_gps_waypoint()
        print("Throwing START")
    elif state_switcher.state == rs.Shutdown():
#        with open(logging, 'a') as f:
 #           f.write("Enable switching from Shutdown to Restart\n")
        state_switcher.handle_event(rs.AutonomyEvents.RESTART, rs.Shutdown(),  then=logging.info("Restarting Autonomy"))
        print("Throwing RESTART")

    drive.enable()


def disable_autonomy(packet_contents):
    if state_switcher.state != rs.Shutdown():
        state_switcher.handle_event(rs.AutonomyEvents.ABORT, rs.Shutdown(), then=logging.info("Disabling Autonomy"))
        print("Throwing ABORT")
    drive.disable()


def clear_waypoint_handler(packet_contents):
    global waypoints
    waypoints = deque()
    logging.info("Waypoints Cleared")
  #  with open(logging, 'a') as f:
   #     f.write("Clear Waypoint all cleared\n")


def set_gps_waypoint():
    global gps_data
    print(waypoints)
    current_goal = waypoints.popleft()
    gps_data.goal = current_goal
    gps_data.start = nav_board.location()
   # with open(logging, 'a') as f:
    
#    f.write("Set Waypoint set new waypoint")


# Define the callbacks to execute when rovecomm recieves the command
rovecomm_node.callbacks[constants.DataID.ENABLE_AUTONOMY] = enable_autonomy
rovecomm_node.callbacks[constants.DataID.DISABLE_AUTONOMY] = disable_autonomy
rovecomm_node.callbacks[constants.DataID.ADD_WAYPOINT] = add_waypoint_handler
rovecomm_node.callbacks[constants.DataID.CLEAR_WAYPOINTS] = clear_waypoint_handler

# state_switcher.state = rs.ApproachingMarker()
# drive.enable()
# test = False
# state_switcher.handle_event(rs.AutonomyEvents.OBSTACLE_AVOIDANCE, rs.ApproachingMarker())
# drive.enable()

time.sleep(2)

while True:

    # GPS Navigation:
    # Travel Point to Point (P2P) from the current GPS to the target given from Basestation
    if state_switcher.state == rs.Navigating():
        goal, start = gps_data.data()
        ball_in_frame, center, radius = tracker.track_ball()
        
        if gps_nav.reached_goal(goal, nav_board.location(), start):
            
            # If there are more points, set the new one and start from top
            if waypoints:
                
                print("Reached mid point!")
                set_gps_waypoint()
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
        
        if ball_in_frame:
            rovecomm_node.write(drive.send_drive(0, 0))
            
            # logging.info("Marker seen at %s with r=%i, locking on..." % (center, radius))
            state_switcher.handle_event(rs.AutonomyEvents.MARKER_SIGHTED, rs.Searching(), Searching())
            print("Throwing MARKER_SIGHTED")
            continue

        left, right = gps_nav.calculate_move(goal, nav_board.location(), start, drive, nav_board)
        print("Drive motors: " + str(left) + ", " + str(right))
        
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
            (left, right), distance = follow_ball.drive_to_marker(50, drive, center, radius)
            
            if distance < .5:
                rovecomm_node.write(drive.send_drive(0, 0))
                # rovecomm_node.write(RoveCommPacket(1000, 'h', (0,0), ip_octet_4=140))
                state_switcher.handle_event(rs.AutonomyEvents.REACHED_MARKER, rs.ApproachingMarker(), then=logging.info("Reached Marker"))
                continue
            else:
                print("Driving To: " + str(left) + ", " + str(right))
                rovecomm_node.write(drive.send_drive(left, right))
                # rovecomm_node.write(RoveCommPacket(1000, 'h', (0,0), ip_octet_4=140))

        else:
            state_switcher.handle_event(rs.AutonomyEvents.MARKER_UNSEEN, rs.ApproachingMarker())
            print("Throwing MARKER_UNSEEN")
        

    elif state_switcher.state == rs.Shutdown():
        pass

    elif state_switcher.state == rs.Idle():
        #state_switcher.handle_event(rs.AutonomyEvents.START, rs.Idle())
       
        pass

    elif state_switcher.state == rs.ObstacleAvoidance():
        rovecomm_node.write(drive.send_drive(0,0))
        # print(drive.send_drive(0,0))
        # rovecomm_node.write(RoveCommPacket(1000, 'h', (0,0), ip_octet_4=140))
        start = nav_board._location
        rovecomm_node.write(drive.send_drive(-100,-100)) # tune this drive number
        # rovecomm_node.write(RoveCommPacket(1000, 'h', (-100,-100), ip_octet_4=140))
        junk, distance = geomath.haversine(start[0], start[1], nav_board._location[0], nav_board._location[1])
        while distance < 2:
                junk, distance = geomath.haversine(start[0], start[1], nav_board._location[0], nav_board._location[1])
                print (distance)
                time.sleep(0.1)
                distance = 3
        rovecomm_node.write(drive.send_drive(0,0))
        # rovecomm_node.write(RoveCommPacket(1000, 'h', (0,0), ip_octet_4=140))
        r = 6371 # radius of earth
        brng = math.radians(nav_board._heading - 90) # target heading
        d = 0.0005 # 5 meters
        lat1 = math.radians(nav_board._location[0])
        lon1 = math.radians(nav_board._location[1])
        lat2 = math.asin(math.sin(lat1)*math.cos(d/r) + math.cos(lat1)*math.sin(d/r)*math.sin(brng))
        lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/r)*math.cos(lat1),math.cos(d/r)-math.sin(lat1)*math.sin(lat2))
        lat2 = math.degrees(lat2)
        lon2 = math.degrees(lon2)
        target = constants.Coordinate(lat2, lon2)
        print(target)
        left, right = gps_nav.calculate_move(target, nav_board.location(), start, drive, nav_board)
        if nav_board._distToGround > constants.LIDAR_MAXIMUM:
            rovecomm_node.write(drive.send_drive(0,0))
            continue
        rovecomm_node.write(drive.send_drive(left,right))
        distance = 10
        while distance > gps_nav.WAYPOINT_DISTANCE_THRESHOLD:
            time.sleep(0.1)
            left, right = gps_nav.calculate_move(target, nav_board.location(), start, drive, nav_board)
            rovecomm_node.write(drive.send_drive(left,right))
            junk, distance = geomath.haversine(nav_board._location[0], nav_board._location[1], target[0], target[1])
            distance = 0.0
            if nav_board._distToGround > constants.LIDAR_MAXIMUM:
                rovecomm_node.write(drive.send_drive(0,0))
                continue
        print("End of Avoidance")
        state_switcher.handle_event(rs.AutonomyEvents.END_OBSTACLE_AVOIDANCE, rs.ObstacleAvoidance())


    time.sleep(.1)
    print(state_switcher.state)


