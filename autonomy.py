import logging
import core
import time
import algorithms
from core import rovecomm, rover_states, constants, waypoints
from interfaces import drive_board, nav_board
from algorithms.gps_navigate import GPSData

logger = logging.getLogger(__name__)
state_switcher = core.rover_states.StateSwitcher()
gps_data = GPSData
loopDelay = 0.07


def enable_autonomy(packet_contents):
    global gps_data
    logger.info("Autonomy Enable Event")
    if state_switcher.state == rover_states.Idle():
        state_switcher.handle_event(rover_states.AutonomyEvents.START, rover_states.Idle())
        gps_data = waypoints.set_gps_waypoint()

    elif state_switcher.state == rover_states.Shutdown():
        state_switcher.handle_event(rover_states.AutonomyEvents.RESTART, rover_states.Shutdown())

    if waypoints.waypoints:
        if state_switcher.state == rover_states.Idle():
            state_switcher.handle_event(rover_states.AutonomyEvents.START, rover_states.Idle())
            gps_data = waypoints.set_gps_waypoint()
        elif state_switcher.state == rover_states.Shutdown():
            state_switcher.handle_event(rover_states.AutonomyEvents.RESTART, rover_states.Shutdown())
       # drive_board.enable()
    else:
        # eventually display an error on basestation
        pass
    drive_board.enable()


def disable_autonomy(packet_contents):
    logger.info("Autonomy Disable Event")
    if state_switcher.state != rover_states.Shutdown():
        state_switcher.handle_event(rover_states.AutonomyEvents.ABORT, rover_states.Shutdown())
    drive_board.disable()


def main():
    global gps_data
    logger.info("Entering Autonomy Main")

    # Configure the necessary callbacks for RoveComm packets
    logger.info("Configured RoveComm callbacks")
    rovecomm.set_callback(constants.DataID.ENABLE_AUTONOMY, enable_autonomy)
    rovecomm.set_callback(constants.DataID.DISABLE_AUTONOMY, disable_autonomy)
    rovecomm.set_callback(constants.DataID.ADD_WAYPOINT, waypoints.add_waypoint)
    rovecomm.set_callback(constants.DataID.CLEAR_WAYPOINTS, waypoints.clear_waypoints)

    while True:
        # GPS Navigation:
        # Travel Point to Point (P2P) from the current GPS to the target given from Basestation
        if state_switcher.state == rover_states.Navigating():

            goal, start = gps_data.data()
            temp = nav_board._location
            logger.debug(f"Navigating: Driving to ({goal[0]}, {goal[1]}) from ({start[0]}, {start[1]}. Currently at: ({temp[0]}, {temp[1]}")

            if algorithms.gps_navigate.get_approach_status(goal, nav_board.location(), start) != constants.ApproachState.APPROACHING:
                logger.info(f"Navigating: Reached goal ({nav_board._location[0]}, {nav_board._location[1]})")

                # If there are more points, set the new one and start from top
                if waypoints.waypoints:
                    gps_data = waypoints.set_gps_waypoint()
                    logger.info(f"Navigating: Reached midpoint, grabbing new point ({goal[0]}, {goal[1]})")
                    continue

                state_switcher.handle_event(rover_states.AutonomyEvents.REACHED_GPS_COORDINATE, rover_states.Navigating())
                gps_data.start = nav_board.location()
                gps_data.goal = nav_board.location()
                core.rovecomm.write(drive_board.send_drive(0, 0))

                continue

            left, right = algorithms.gps_navigate.calculate_move(goal, nav_board.location(), start, 250)
            logger.debug(f"Navigating: Driving at ({left}, {right})")
            core.rovecomm.write(drive_board.send_drive(left, right))

        # Search Pattern:
        # Travel in a defined pattern to find the target object, the tennis ball
        elif state_switcher.state == rover_states.Searching():
            goal, start = gps_data.data()
            logger.debug(f"Searching: Location ({nav_board._location[0]},{nav_board._location[1]}) to Goal ({goal[0]}, {goal[1]})")

            if algorithms.gps_navigate.get_approach_status(goal, nav_board.location(), start) != constants.ApproachState.APPROACHING:
                core.rovecomm.write(drive_board.send_drive(0, 0))

                time.sleep(1)
                goal = algorithms.marker_search.calculate_next_coordinate(start, goal)

                logger.info(f"Searching: Adding New Waypoint ({goal[0]}, {goal[1]}")
                gps_data.goal = goal

            logger.debug(f"Searching: Target Waypoint is ({goal[0]},{goal[1]}")

            left, right = algorithms.gps_navigate.calculate_move(goal, nav_board.location(), start, 100)
            logger.debug(time.strftime("%H%M%S") + "Searching: Driving speeds (" + str(left) + "," + str(right))

            core.rovecomm.write(drive_board.send_drive(left, right))

        elif state_switcher.state == rover_states.Shutdown():
            pass

        elif state_switcher.state == rover_states.Idle():
            pass

        # TODO: Scrap this and actually build a system that rate limits outputs
        # In previous years, the code was rate limited with this delay to ensure that outgoing autonomy commands weren't being spammed at incredible rates
        time.sleep(loopDelay)

        logger.debug(f"Current State: {rover_states.EventsToString[state_switcher.state]}")
