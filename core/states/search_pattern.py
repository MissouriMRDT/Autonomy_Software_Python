#
# Mars Rover Design Team
# search_pattern.py
#
# Created on Dec 02, 2020
# Updated on Aug 21, 2022
#

import utm
import core
import math
import algorithms
import interfaces
import asyncio
import time
import numpy as np
import matplotlib.pyplot as plt
from core.states import RoverState
from algorithms.obstacle_avoider import ASTAR
from algorithms import stanley_controller
from algorithms import heading_hold
from core import constants


class SearchPattern(RoverState):
    """
    The searching state’s goal is to drive the rover in an ever expanding Archimedean spiral, searching for the AR Tag.
    The spiral type was chosen because of it’s fixed distance between each rotation’s path.
    """

    def start(self):
        """
        Schedule Search Pattern
        """
        # Create state specific variable.
        self.astar = ASTAR(core.constants.SEARCH_OBSTACLE_QUEUE_LENGTH)
        self.rover_position_state = None
        self.path_xs = []
        self.path_ys = []
        self.path_yaws = []
        self.rover_xs = []
        self.rover_ys = []
        self.rover_yaws = []
        self.rover_vs = []
        self.last_idx = 0
        self.target_idx = 0
        self.path_start_time = 0
        self.stuck_check_timer = 0
        self.stuck_check_last_position = [0, 0]

    def exit(self):
        """
        Cancel all state specific tasks
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

        if event == core.AutonomyEvents.MARKER_SEEN:
            state = core.states.ApproachingMarker()

        elif event == core.AutonomyEvents.GATE_SEEN:
            state = core.states.ApproachingGate()

        elif event == core.AutonomyEvents.START:
            state = self

        elif event == core.AutonomyEvents.SEARCH_FAILED:
            state = core.states.Idle()

        elif event == core.AutonomyEvents.ABORT:
            state = core.states.Idle()

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
        current = interfaces.nav_board.location()
        gps_data = core.waypoint_handler.gps_data

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
            self.logger.error("SearchPattern: No waypoint, please add a waypoint to start navigating")
            return self.on_event(core.AutonomyEvents.NO_WAYPOINT)

        # Pull info out of waypoint.
        goal, start, leg_type = gps_data.data()
        self.logger.debug(
            f"Searching: Location ({interfaces.nav_board._location[0]}, {interfaces.nav_board._location[1]}) to Goal ({goal[0]}, {goal[1]})"
        )

        # Check to see if gate or marker was detected
        # If so, immediately stop all movement to ensure that we don't lose sight of the AR tag(s)
        if core.vision.ar_tag_detector.is_gate() and leg_type == "GATE":
            interfaces.drive_board.stop()

            # Sleep for a brief second
            await asyncio.sleep(core.EVENT_LOOP_DELAY)

            self.logger.info("Search Pattern: Gate seen")
            return self.on_event(core.AutonomyEvents.GATE_SEEN)

        elif core.vision.ar_tag_detector.is_marker() and leg_type == "MARKER":
            interfaces.drive_board.stop()

            # Sleep for a brief second
            await asyncio.sleep(core.EVENT_LOOP_DELAY)

            self.logger.info("Search Pattern: Marker seen")
            return self.on_event(core.AutonomyEvents.MARKER_SEEN)

        # If Stanley has reached the last index of the path generate new one even though we aren't there yet.
        if self.target_idx == self.last_idx:
            # Sleep for a little bit before we move to the next point, allows for AR Tag to be picked up
            await asyncio.sleep(core.EVENT_LOOP_DELAY)

            # Find and set the next goal in the search pattern
            goal = algorithms.marker_search.calculate_next_coordinate(start, goal)
            core.waypoint_handler.set_goal(goal)
            self.logger.info(f"Search Pattern: Adding New Waypoint ({goal[0]}, {goal[1]}")

            # If path is empty use rover's current location.
            if len(self.path_xs) > 0:
                # Get UTM zone.
                utm_current = utm.from_latlon(current[0], current[1])
                # Get last coordinate of path and convert it to GPS.
                start_coord = utm.to_latlon(*(self.path_xs[-1], self.path_ys[-1], utm_current[2], utm_current[3]))
            else:
                start_coord = current
            # Generate path.
            path = self.astar.plan_astar_avoidance_route(
                max_route_size=40, near_object_threshold=0.0, start_gps=start_coord, waypoint_thresh=0.3
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

        """
        PATH GENERATION AND FOLLOWING.
        """
        # Check if path is empty.
        if len(self.path_xs) > 1:
            # Get current gps location. Convert to utm so we can work with meters.
            utm_current = utm.from_latlon(current[0], current[1])
            # Get current heading. Must subtract 90 because stanley controller expect a unit circle oriented heading.
            heading = np.deg2rad(interfaces.nav_board.heading() - 90)
            # Normalize heading to -pi and pi
            heading = stanley_controller.normalize_angle(-heading)

            # If this is the first run interation for the avoidance state, then initialize some variables with the current rover information.
            if self.rover_position_state is None:
                # Create new state and give intial values of the rovers current position, heading, and velocity.
                self.rover_position_state = stanley_controller.State(utm_current[0], utm_current[1], heading)

                # Store the initial rover state in x, y, yaw, and velocity arrays.
                self.rover_xs.append(self.rover_position_state.x)
                self.rover_ys.append(self.rover_position_state.y)
                self.rover_yaws.append(self.rover_position_state.yaw)
                self.rover_vs.append(self.rover_position_state.v)
                self.target_idx, _ = stanley_controller.calc_target_index(
                    self.rover_position_state, self.path_xs, self.path_ys
                )
            else:
                # Get velocity adjustment with P controller.
                acceleration = stanley_controller.pid_control(
                    constants.AVOIDANCE_MAX_SPEED_MPS, self.rover_position_state.v
                )
                # Call stanley controller and get steering control adjustment and the next best target along the path.
                delta_adjustment, self.target_idx = stanley_controller.stanley_control(
                    self.rover_position_state, self.path_xs, self.path_ys, self.path_yaws, self.target_idx
                )
                # Calculate adjusted heading. Add 90 to convert back to gps heading.
                goal_heading = -np.rad2deg(heading + delta_adjustment) + 90
                goal_speed = constants.MAX_DRIVE_POWER * (1 + acceleration)

                # Update the current rover state.
                self.rover_position_state.update(utm_current[0], utm_current[1], heading)

                # Store the initial rover state in x, y, yaw, and velocity arrays.
                self.rover_xs.append(self.rover_position_state.x)
                self.rover_ys.append(self.rover_position_state.y)
                self.rover_yaws.append(self.rover_position_state.yaw)
                self.rover_vs.append(self.rover_position_state.v)

                # Write path 1 second before it expires.
                if int(time.time()) % 2 == 0:
                    plt.cla()
                    # Plot path, current location, and target_index.
                    plt.gca().set_aspect("equal", adjustable="box")
                    plt.plot(self.path_xs, self.path_ys, ".r", label="course")
                    plt.plot(self.rover_xs, self.rover_ys, "-b", label="trajectory")
                    plt.plot(self.path_xs[self.target_idx], self.path_ys[self.target_idx], "xg", label="target")
                    plt.axis("equal")
                    plt.grid(True)
                    plt.title("Rover Velocity (M/S):" + str(self.rover_position_state.v))
                    plt.savefig("logs/!stanley_utm_path.png")

                # Send drive board commands to drive at a certain speed at a certain angle.
                left, right = heading_hold.get_motor_power_from_heading(goal_speed, goal_heading)
                # Set drive powers.
                self.logger.info(f"SearchPattern: Driving at ({left}, {right})")
                interfaces.drive_board.send_drive(left, right)

                # Check if the rover is too far from the target position in path. Manually lead rover back onto path.
                if (
                    math.sqrt(
                        math.pow(self.path_xs[self.target_idx] - utm_current[0], 2)
                        + math.pow(self.path_ys[self.target_idx] - utm_current[1], 2)
                    )
                    > constants.SEARCH_PATTERN_MAX_ERROR_FROM_PATH
                ):
                    path = self.astar.plan_astar_avoidance_route(
                        max_route_size=30, near_object_threshold=0.0, start_gps=current, waypoint_thresh=0.3
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
            # Stop the drive board.
            interfaces.drive_board.stop()
            # Print debug that path has completed.
            self.logger.info("SearchPattern ASTAR path is empty.")

        return self
