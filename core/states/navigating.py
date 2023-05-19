#
# Mars Rover Design Team
# navigating.py
#
# Created on Dec 02, 2020
# Updated on Aug 21, 2022
#

import utm
import core
import time
import interfaces
import algorithms
import numpy as np
import matplotlib.pyplot as plt
from core.states import RoverState
from algorithms import geomath, stanley_controller, heading_hold, small_movements
from algorithms.obstacle_avoider import ASTAR

from core import constants


class Navigating(RoverState):
    """
    The goal of this state is to navigate to the GPS coordinates provided by base
    station in succession, the last of which is the coordinate provided by the judges
    for that leg of the task. Coordinates before the last are simply the operators in
    base station’s best guess of the best path for the rover due to terrain identified
    on RED’s map.
    """

    def start(self):
        """
        Schedule Navigating
        """
        # Create state specific variable.
        self.astar = ASTAR(max_queue_length=0)
        self.rover_position_state = None
        self.path_xs = []
        self.path_ys = []
        self.path_yaws = []
        self.rover_xs = []
        self.rover_ys = []
        self.rover_yaws = []
        self.rover_vs = []
        self.last_idx = -1
        self.target_idx = 0
        self.path_start_time = 0
        self.time_since_last_path = 0
        self.stuck_check_timer = 0
        self.stuck_check_last_position = [0, 0]

        # Clear AR Tags.
        core.vision.ar_tag_detector.clear_tags()

    def exit(self):
        """
        Cancel all state specific coroutines
        """
        # Cancel all state specific coroutines and reset state variables.
        self.astar.clear_obstacles()
        self.path_xs.clear()
        self.path_ys.clear()
        self.path_yaws.clear()
        self.rover_xs.clear()
        self.rover_ys.clear()
        self.rover_yaws.clear()
        self.rover_vs.clear()
        self.target_idx = 0
        # Set position state back to none.
        self.rover_position_state = None

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events

        :param event:
        :return: RoverState
        """
        state: RoverState = None

        if event == core.AutonomyEvents.NO_WAYPOINT:
            state = core.states.Idle()

        elif event == core.AutonomyEvents.REACHED_MARKER:
            state = core.states.Idle()

        elif event == core.AutonomyEvents.REACHED_GPS_COORDINATE:
            state = core.states.SearchPattern()

        elif event == core.AutonomyEvents.NEW_WAYPOINT:
            # Generate path.
            path = self.astar.plan_astar_avoidance_route(
                max_route_size=constants.NAVIGATION_PATH_ROUTE_LENGTH, near_object_threshold=0.0, waypoint_thresh=0.3
            )

            # If path was generated successfully, then put it in our future path. Cut out old future.
            if path is not None:
                # Set path start timer.
                self.path_start_time = time.time()

                # Cut off path data after our current location.
                self.path_xs = self.path_xs[: self.target_idx]
                self.path_ys = self.path_ys[: self.target_idx]
                self.path_yaws = self.path_yaws[: self.target_idx]

                # Append new path onto current.
                for point in path:
                    # Add new path starting from current location.
                    self.path_xs.append(point[0])
                    self.path_ys.append(point[1])

                # Manually calulate yaws since ASTAR doesn't given yaws.
                self.path_yaws = stanley_controller.calculate_yaws_from_path(
                    self.path_xs, self.path_ys, interfaces.nav_board.heading()
                )

                # Store last index of path.
                self.last_idx = len(self.path_xs) - 1
            else:
                # Print error.
                self.logger.error("Path didn't generate!")

            state = self

        elif event == core.AutonomyEvents.START:
            state = self

        elif event == core.AutonomyEvents.ABORT:
            state = core.states.Idle()

        elif event == core.AutonomyEvents.OBSTACLE_AVOIDANCE:
            state = core.states.Avoidance()
        elif event == core.AutonomyEvents.REVERSE:
            state = core.states.Reversing()
        elif event == core.AutonomyEvents.STUCK:
            state = core.states.Stuck()

        else:
            self.logger.error(f"Unexpected event {event} for state {self}")
            state = core.states.Idle()

        # Call exit() if we are not staying the same state
        if state != self:
            self.exit()

        # Return the state appropriate for the event
        return state

    async def run(self) -> RoverState:
        """
        Defines regular rover operation when under this state

        :return: RoverState
        """
        # Get current position and next desired waypoint position.
        current = interfaces.nav_board.location(force_absolute=True)
        gps_data = core.waypoint_handler.get_waypoint()

        """
        STATE TRANSITION AND WAYPOINT LOGIC.
        """

        # We should be navigating, so check if we have been in the same position for awhile.
        # Only check every predefined amount of seconds.
        if (time.time() - self.stuck_check_timer) > core.constants.STUCK_UPDATE_TIME:
            # Calculate distance from goal for checking for markers and gates.
            _, distance = algorithms.geomath.haversine(
                self.stuck_check_last_position[0], self.stuck_check_last_position[1], current[0], current[1]
            )
            # Convert km to m.
            distance *= 1000

            # Store new position.
            self.stuck_check_last_position[0], self.stuck_check_last_position[1] = current[0], current[1]

            # Check if we are stuck.
            if distance < core.constants.STUCK_MIN_DISTANCE:
                # Move to stuck state.
                return self.on_event(core.AutonomyEvents.STUCK)

            # Update timer.
            self.stuck_check_timer = time.time()

        # If the gps_data is none, there were no waypoints to be grabbed,
        # so log that and return
        if gps_data is None:
            self.logger.error("Navigating: No waypoint, please add a waypoint to start navigating")
            return self.on_event(core.AutonomyEvents.NO_WAYPOINT)

        # Pull info out of waypoint.
        goal, start, leg_type = gps_data.data()
        self.logger.debug(
            f"Navigating: Driving to ({goal[0]}, {goal[1]}) from ({start[0]}, {start[1]}. Currently at: ({current[0]}, {current[1]}"
        )

        # Calculate distance from goal for checking for markers and gates.
        bearing, distance = geomath.haversine(current[0], current[1], goal[0], goal[1])
        distance *= 1000  # convert from km to m
        if distance > constants.ARUCO_ENABLE_DISTANCE:
            core.vision.ar_tag_detector.clear_tags()

        # Move to approaching marker if 1 ar tag is spotted during marker leg type
        if (
            core.waypoint_handler.gps_data.leg_type == "MARKER"
            and core.vision.ar_tag_detector.is_marker()
            and distance < constants.ARUCO_ENABLE_DISTANCE
        ):
            return core.states.ApproachingMarker()

        # Move to approaching gate if 2 ar tags are spotted during gate leg type.
        if (
            (core.waypoint_handler.gps_data.leg_type == "GATE" or core.waypoint_handler.gps_data.leg_type == "MARKER")
            and core.vision.ar_tag_detector.is_gate()
            and distance < constants.ARUCO_ENABLE_DISTANCE
        ):
            core.waypoint_handler.gps_data.leg_type = "GATE"
            return core.states.ApproachingGate()

        # Move to obstacle avoidance if objects are detected and we aren't close to the goal.
        if (
            core.vision.obstacle_avoidance.is_obstacle()
            and core.vision.obstacle_avoidance.get_distance() < constants.AVOIDANCE_OBJECT_DISTANCE_MAX
            and (algorithms.geomath.haversine(current[0], current[1], goal[0], goal[1])[1] * 1000)
            > constants.AVOIDANCE_ENABLE_DISTANCE_THRESHOLD
        ):  # If distance to object is less than distance to goal, continue
            self.logger.info("Detected obstacle, now avoiding")
            return self.on_event(core.AutonomyEvents.OBSTACLE_AVOIDANCE)

        # If there are more points, set the new one and start from top
        if (
            algorithms.gps_navigate.get_approach_status(goal, current, start, core.WAYPOINT_DISTANCE_THRESHOLD)
            != core.ApproachState.APPROACHING
        ):
            if not core.waypoint_handler.is_empty():
                # Get new wapoint goal.
                gps_data = core.waypoint_handler.get_new_waypoint()
                self.logger.info(f"Navigating: Reached midpoint, grabbing new point ({goal[0]}, {goal[1]})")

                # Force path to expire and regenerate.
                self.path_start_time = 0
                self.last_idx = -1

                return self.on_event(core.AutonomyEvents.NEW_WAYPOINT)
            else:
                # Print logger info.
                self.logger.info(
                    f"Navigating: Reached goal ({interfaces.nav_board._location[0]}, {interfaces.nav_board._location[1]})"
                )
                # Stop all movement
                interfaces.drive_board.stop()

                # Set goal and start to current location
                core.waypoint_handler.set_goal(interfaces.nav_board.location(force_absolute=True))
                core.waypoint_handler.set_start(interfaces.nav_board.location(force_absolute=True))

                if leg_type == "POSITION":
                    self.logger.info("Reached Marker")

                    # Transmit that we have reached the marker
                    core.rovecomm_node.write(
                        core.RoveCommPacket(
                            core.manifest["Autonomy"]["Telemetry"]["ReachedMarker"]["dataId"],
                            "B",
                            (1,),
                        ),
                        False,
                    )

                    # Tell multimedia board to flash our LED matrix green to indicate reached marker
                    interfaces.multimedia_board.send_lighting_state(core.OperationState.REACHED_MARKER)
                    return self.on_event(core.AutonomyEvents.REACHED_MARKER)
                else:
                    return self.on_event(core.AutonomyEvents.REACHED_GPS_COORDINATE)
        else:
            # Force path to expire and regenerate.
            self.path_start_time = 0
            self.last_idx = -1

        # Calculate powers.
        left, right = algorithms.gps_navigate.calculate_move(goal, interfaces.nav_board.location(force_absolute=True), start, core.MAX_DRIVE_POWER)
        # Send drive.
        interfaces.drive_board.send_drive(left, right)

        return self
