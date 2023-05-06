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
from algorithms import geomath
from core.vision.ar_tag_detector import clear_tags
from algorithms.obstacle_avoider import ASTAR
from algorithms import stanley_controller
from algorithms import heading_hold
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
        self.astar = ASTAR()
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
                max_route_size=constants.NAVIGATION_PATH_ROUTE_LENGTH, near_object_threshold=0.0
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
        gps_data = core.waypoint_handler.get_waypoint()

        """
        STATE TRANSITION AND WAYPOINT LOGIC.
        """
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
        if distance > constants.ARUCO_GOAL_DISTANCE_THRESH:
            clear_tags()

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

        # If Stanley has reached the last index of the path generate new one even though we aren't there yet.
        if self.target_idx == self.last_idx or len(self.path_xs) <= 1:
            # If there are more points, set the new one and start from top
            if not core.waypoint_handler.is_empty():
                # Get new wapoint goal.
                gps_data = core.waypoint_handler.get_new_waypoint()
                self.logger.info(f"Navigating: Reached midpoint, grabbing new point ({goal[0]}, {goal[1]})")

                # Force path to expire and regenerate.
                self.path_start_time = 0
                self.last_idx = -1

                return self.on_event(core.AutonomyEvents.NEW_WAYPOINT)
            elif (
                algorithms.gps_navigate.get_approach_status(goal, current, start, core.WAYPOINT_DISTANCE_THRESHOLD)
                != core.ApproachState.APPROACHING
            ):
                # Print logger info.
                self.logger.info(
                    f"Navigating: Reached goal ({interfaces.nav_board._location[0]}, {interfaces.nav_board._location[1]})"
                )
                # Stop all movement
                interfaces.drive_board.stop()

                # Set goal and start to current location
                core.waypoint_handler.set_goal(interfaces.nav_board.location())
                core.waypoint_handler.set_start(interfaces.nav_board.location())

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

        """
        PATH GENERATION AND FOLLOWING.
        """
        # Update time since last path generation.
        self.time_since_last_path = time.time() - self.path_start_time
        # Now that we have our gps waypoints. Generate a path if not already done.
        if self.time_since_last_path > constants.NAVIGATION_PATH_EXPIRATION_SECONDS or len(self.path_xs) <= 1:
            # Generate path.
            path = self.astar.plan_astar_avoidance_route(
                max_route_size=constants.NAVIGATION_PATH_ROUTE_LENGTH, near_object_threshold=0.0
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

        # Check if path is empty.
        if len(self.path_xs) > 0:
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
                    plt.savefig("logs/!navigation_gps_path.png")

                # Send drive board commands to drive at a certain speed at a certain angle.
                left, right = heading_hold.get_motor_power_from_heading(goal_speed, goal_heading)
                # Set drive powers.
                self.logger.info(f"Navigation: Driving at ({left}, {right})")
                interfaces.drive_board.send_drive(left, right)
        else:
            # Stop the drive board.
            interfaces.drive_board.stop()
            # Print debug that path has completed.
            self.logger.info("Navigate ASTAR path is empty.")

        return self
