import logging
import core
import constants
import waypoints
import time
from core import RoveComm, rover_states
from interfaces import drive_board, nav_board
from algorithms.gps_navigate import GPSData

logger = logging.getLogger(__name__)
state_switcher = core.rover_states.StateSwitcher()
gps_data = GPSData
loopDelay = 0.1


def enable_autonomy(packet_contents):
    global gps_data
    logger.info("Autonomy Enable Event")
    if state_switcher.state == rover_states.Idle():
        state_switcher.handle_event(rover_states.AutonomyEvents.START, rover_states.Idle())
        gps_data = waypoints.set_gps_waypoint()

    elif state_switcher.state == rover_states.Shutdown():
        state_switcher.handle_event(rover_states.AutonomyEvents.RESTART, rover_states.Shutdown())

    drive_board.enable()


def disable_autonomy(packet_contents):
    logger.info("Autonomy Disable Event")
    if state_switcher.state != rover_states.Shutdown():
        state_switcher.handle_event(rover_states.AutonomyEvents.ABORT, rover_states.Shutdown())
    drive_board.disable()


def main():

    # Configure the necessary callbacks for RoveComm packets
    RoveComm.set_callback(constants.DataID.ENABLE_AUTONOMY, enable_autonomy)
    RoveComm.set_callback(constants.DataID.DISABLE_AUTONOMY, disable_autonomy)
    RoveComm.set_callback(constants.DataID.ADD_WAYPOINT, waypoints.add_waypoint_handler)
    RoveComm.set_callback(constants.DataID.CLEAR_WAYPOINTS, waypoints.clear_waypoint_handler)

    while True:
        # GPS Navigation:
        # Travel Point to Point (P2P) from the current GPS to the target given from Basestation
        if state_switcher.state == rover_states.Navigating():
            goal, start = gps_data.data()
            ball_in_frame, center, radius = tracker.track_ball()
            temp = nav_board._location
            Logger.write_line(time.strftime("%H%M%S") + " Navigating: Driving to " + str(goal[0]) + "," + str(goal[1]) + " from " + str(start[0]) + "," + str(start[1]) + ". Currently at: " + str(temp[0]) + "," + str(temp[1]))
            #looping += 1
            #if looping > 25:
                #looping = 0
                #print("LOoping hit 25")
                #rovecomm_node.write(RoveCommPacket(14003, 'B', (2, 0), ip_octet_4="131"))
                #state_switcher.handle_event(rs.AutonomyEvents.REACHED_GPS_COORDINATE, rs.Navigating())
                #continue
            if gps_nav.reached_goal(goal, nav_board.location(), start):
                Logger.write_line(time.strftime("%H%M%S") + " Navigating: reached goal (" + str(nav_board._location[0]) + "," + str(nav_board._location[1]) + ")")
                # If there are more points, set the new one and start from top
                if waypoints:
                    
                    print("Reached mid point!")
                    set_gps_waypoint()
                    Logger.write_line(time.strftime("%H%M%S") + " Navigating: Reached midpoint, grabbing new point " + str(goal[0]) + "," + str(goal[1]))
                    continue

                state_switcher.handle_event(rs.AutonomyEvents.REACHED_GPS_COORDINATE, rs.Navigating())
                gps_data.start = nav_board.location()
                gps_data.goal = nav_board.location()
                print("Throwing REACHED_GPS_COORDINATE")
                #notify.notify_finish()
                rovecomm_node.write(drive.send_drive(0, 0))
                rovecomm_node.write(RoveCommPacket(14003, 'B', (2, 2), ip_octet_4="131"))
                #state_switcher.handle_event(rs.AutonomyEvents.ABORT, rs.Navigating())
                continue

            left, right = gps_nav.calculate_move(goal, nav_board.location(), start, drive, nav_board, 250)
            # time.sleep(loopDelay * 3)
            #rovecomm_node.write(RoveCommPacket(14003, 'B', (2, 0), ip_octet_4="22"))
            Logger.write_line(time.strftime("%H%M%S") + " Navigating: Driving at " + str(left) + "," + str(right))
            print("Drive motors: " + str(left) + ", " + str(right))
            rovecomm_node.write(drive.send_drive(left, right))



        # Search Pattern:
        # Travel in a defined pattern to find the target object, the tennis ball
        elif state_switcher.state == rs.Searching():
            goal, start = gps_data.data()
            print(goal)
            print(nav_board._location)
            Logger.write_line(time.strftime("%H%M%S") + " Searching: Location, Goal," + str(nav_board._location[0]) + "," + str(nav_board._location[1]) + "," + str(goal[0]) + "," + str(goal[1]) + ",")
            
            if gps_nav.reached_goal(goal, nav_board.location(), start):
                rovecomm_node.write(drive.send_drive(0, 0))
                time.sleep(1)
                goal = marker_search.calculate_next_coordinate(start, goal)
                print("New Goal: " + str(goal.lat) + ", " + str(goal.lon))
                lat, lon = nav_board.location()
                Logger.write_line(time.strftime("%H%M%S") + " Searching: Adding New Waypoint " + str(goal[0]) + "," + str(goal[1]))
                
                gps_data.goal = goal
            #looping += 1
            #if looping > 25:
                #looping = 0
                #nav_board._location = gps_data.goal
            print("...searching for marker...")
            #ball_in_frame, center, radius = tracker.track_ball()

            #Logger.write_line(time.strftime("%H%M%S") + " Searching: Ball Sighted, " + str(ball_in_frame) + ". With center and radius " + str(center) + "," + str(radius))
            Logger.write_line(time.strftime("%H%M%S") + " Searching: Target Waypoint is " + str(goal[0]) + "," + str(goal[1]))
            
            #if ball_in_frame:
                #rovecomm_node.write(drive.send_drive(0, 0))
                #time.sleep(1)
                
                #Logger.write_line("Marker seen at %s with r=%i, locking on..." % (center, radius))
                #state_switcher.handle_event(rs.AutonomyEvents.MARKER_SIGHTED, rs.Searching())
                #Logger.write_line(time.strftime("%H%M%S") + " Searching: Marker Sighted, entering ApproachingMarker()")
                #print("Throwing MARKER_SIGHTED")
                #continue

            left, right = gps_nav.calculate_move(goal, nav_board.location(), start, drive, nav_board, 100)
            print("Drive motors: " + str(left) + ", " + str(right))
            Logger.write_line(time.strftime("%H%M%S") + "Searching: " + str(left) + "," + str(right))
            rovecomm_node.write(drive.send_drive(left, right))

        elif state_switcher.state == rover_states.Shutdown():
            pass

        elif state_switcher.state == rover_states.Idle():
            pass

        # TODO: Scrap this and actually build a system that rate limits outputs
        # In previous years, the code was rate limited with this delay to ensure that outgoing autonomy commands weren't being spammed at incredible rates
        time.sleep(loopDelay)

        logger.debug("Current State: {rover_states.EventsToString[state_switcher.state]}")
