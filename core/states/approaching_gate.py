#
# Mars Rover Design Team
# approaching_gate.py
#
# Created on May 19, 2021
# Updated on Aug 21, 2022
#

import core
from core import constants
import interfaces
from algorithms import stanley_controller, heading_hold
from algorithms.obstacle_avoider import ASTAR
import algorithms
from core.states import RoverState
import numpy as np
from numpy import NaN
import time
import math
import asyncio
import utm
from matplotlib import pyplot as plt


class ApproachingGate(RoverState):
    """
    Within approaching gate, 3 waypoints are calculated in front of, between, and through the viewed gate,
    allowing the rover to traverse through the gate fully.
    """

    def start(self):
        """
        Schedule AR Tag detection
        """
        # Create state specific variable.
        self.astar = ASTAR(core.constants.GATE_OBSTACLE_QUEUE_LENGTH)
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
        self.num_detection_attempts = 0
        self.gate_detection_attempts = 0
        self.gate_coords = [(0, 0), (0, 0)]
        self.tags = []
        self.trench_mid_points = []
        self.through_gate_timer = 0

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

        if event == core.AutonomyEvents.REACHED_MARKER:
            # Tell multimedia board to flash our LED matrix green to indicate reached marker
            interfaces.multimedia_board.send_lighting_state(core.OperationState.REACHED_MARKER)
            # Clear markers.
            core.vision.ar_tag_detector.clear_tags()
            # Move to idle state.
            state = core.states.Idle()

        elif event == core.AutonomyEvents.START:
            state = self

        elif event == core.AutonomyEvents.MARKER_UNSEEN:
            state = core.states.SearchPattern()

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
        Asynchronous state machine loop

        :return: RoverState
        """
        # Get current position and next desired waypoint position.
        current = interfaces.nav_board.location()
        # Get UTM location so we have UTM zone for GPS and UTM conversion.
        utm_current = utm.from_latlon(current[0], current[1])
        # Get current heading. Must subtract 90 because stanley controller expect a unit circle oriented heading.
        heading = np.deg2rad(interfaces.nav_board.heading() - 90)
        # Use get_tags to create an array of the 2 gate posts
        # (named tuples containing the distance and relative angle from the camera)
        self.tags = core.vision.ar_tag_detector.get_valid_tags()
        # Gate waypoint data.
        goal, _, leg_type = core.waypoint_handler.get_waypoint().data()

        # If we've seen at least 5 frames of 2 tags, assume it's a gate
        if (
            len(self.tags) == 2
            and self.gate_detection_attempts >= constants.ARUCO_FRAMES_DETECTED
            and leg_type == "GATE"
            and algorithms.geomath.haversine(
                current[0],
                current[1],
                (self.gate_coords[0][0] + self.gate_coords[1][0]) / 2,
                (self.gate_coords[0][1] + self.gate_coords[1][1]) / 2,
            )[1]
            * 1000
            > core.constants.GATE_UPDATE_PATH_MAX_MARKER_DISTANCE
        ):
            """
            WAYPOINT LOGIC.
            """
            # Clear previous obstacles.
            self.astar.clear_obstacles()
            # Store AR tags as obstacles.
            obstacle_list = [(self.tags[0].angle, self.tags[0].distance), (self.tags[1].angle, self.tags[1].distance)]
            # Update ASTAR object.
            self.astar.update_obstacles(
                obstacle_list,
                min_object_distance=0.3,
                max_object_distance=99999,
                min_object_angle=-180,
                max_object_angle=180,
            )
            # The update_obstacles method automatically converts the angles and distances to gps, so pull out gps coords.
            self.gate_coords = self.astar.get_obstacle_coords()
            # Convert gate gps coords to utm xs and ys.
            gate_xs = [utm.from_latlon(t[0], t[1])[0] for t in self.gate_coords]
            gate_ys = [utm.from_latlon(t[0], t[1])[1] for t in self.gate_coords]

            ######################################################################################################################
            # Calculate the intersection line and perp line to the obstacles to make a 'trench' for the rover to drive through.
            # Check https://www.desmos.com/calculator/xcbgsmlk6x for an interactive graph.
            ######################################################################################################################
            # Create list to store UTM coords.
            gate1_obstacles = []
            gate2_obstacles = []
            # Store the UTM easting/northing for each gate.
            x1, y1 = gate_xs[0], gate_ys[0]
            x2, y2 = gate_xs[1], gate_ys[1]
            # Find slope of line that pases through both points.
            try:
                m = (y2 - y1) / (x2 - x1)
            except ZeroDivisionError:
                # If points are vertically aligned just set, slope to NaN.
                m = NaN

            # Find radius' from -3 to 3 in increments of 0.5
            for r in [x * 0.125 for x in range(-16, 16)]:
                # If slope if zero, just add to y values.
                if m == 0:
                    # Append simple obstacles.
                    gate1_obstacles.append((x1, y1 + r))
                    gate2_obstacles.append((x2, y2 + r))
                # Check if slope is undefined.
                elif m == NaN:
                    # Append simple obstacles.
                    gate1_obstacles.append((x1 + r, y1))
                    gate2_obstacles.append((x2 + r, y2))
                else:
                    # This is the equation that will spit out the x coordinate of the first gates trench line given the radius of a circle
                    # with origin at the gatepoint.
                    # Find appropriate x point for first gate marker.
                    gate1_rad_x = (
                        m * (-m * ((-2 * x1) - ((2 * x1) / math.pow(m, 2))) - ((2 * r) * math.sqrt(1 + math.pow(m, 2))))
                    ) / (2 * (math.pow(m, 2) + 1))
                    # Find appropriate x point for second gate marker.
                    gate2_rad_x = (
                        m * (-m * ((-2 * x2) - ((2 * x2) / math.pow(m, 2))) + ((2 * r) * math.sqrt(1 + math.pow(m, 2))))
                    ) / (2 * (math.pow(m, 2) + 1))

                    # Find the cooresponding Y point for the calculated X point using the perpendicular line of slope m at each gate point.
                    # Find Y point for first gate marker.
                    gate1_rad_y = (-1 / m) * (gate1_rad_x - x1) + y1
                    # Find Y point for second gate marker.
                    gate2_rad_y = (-1 / m) * (gate2_rad_x - x2) + y2

                    # Add points to gate obstacle list.
                    gate1_obstacles.append((gate1_rad_x, gate1_rad_y))
                    gate2_obstacles.append((gate2_rad_x, gate2_rad_y))

            # Add gate1 and gate2 obstacles to astar list.
            self.astar.update_obstacle_coords(gate1_obstacles[5:-5], input_gps=False)
            self.astar.update_obstacle_coords(gate2_obstacles[5:-5], input_gps=False)

            # Find midpoint between matching points in the trench.
            mid_points = []
            for point1, point2 in zip(gate1_obstacles, gate2_obstacles[::-1]):
                # Calulate midpoint.
                midpoint = ((point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2)
                # Append point to list.
                mid_points.append(midpoint)
            # Store in member variables so we can plot it later.
            self.trench_mid_points = mid_points

            # Find which end of the trenchline is closer to the rover.
            end1_distance = algorithms.geomath.utm_distance(
                utm_current[0], utm_current[1], self.trench_mid_points[0][0], self.trench_mid_points[0][1]
            )[1]
            end2_distance = algorithms.geomath.utm_distance(
                utm_current[0], utm_current[1], self.trench_mid_points[-1][0], self.trench_mid_points[-1][1]
            )[1]
            # Set whatever end is closer to us as the current goal.
            if end1_distance < end2_distance:
                # Set waypoint goal.
                core.waypoint_handler.set_goal(
                    utm.to_latlon(
                        self.trench_mid_points[0][0], self.trench_mid_points[0][1], utm_current[2], utm_current[3]
                    )
                )
            else:
                # Set waypoint goal.
                core.waypoint_handler.set_goal(
                    utm.to_latlon(
                        self.trench_mid_points[-1][0], self.trench_mid_points[-1][1], utm_current[2], utm_current[3]
                    )
                )

                # Reverse midpoint list.
                self.trench_mid_points = self.trench_mid_points[::-1]

            # Generate path from rovers current position to new goal.
            path = self.astar.plan_astar_avoidance_route(
                max_route_size=50,
                near_object_threshold=constants.GATE_NEAR_OBSTACLE_THRESH,
                start_gps=current,
                waypoint_thresh=0.1,
            )

            # If path was generated successfully, then put it in our future path. Cut out old future.
            if path is not None:
                # Set path start timer.
                self.path_start_time = time.time()

                # Clear path xs, ys, and yaws.
                self.path_xs.clear()
                self.path_ys.clear()
                self.path_yaws.clear()

                # Append new ASTAR path onto current.
                for point in path:
                    # Add new path starting from current location.
                    self.path_xs.append(point[0])
                    self.path_ys.append(point[1])

                # Add trenchline to end of generated path.
                for point in self.trench_mid_points:
                    # Add new path starting from current location.
                    self.path_xs.append(point[0])
                    self.path_ys.append(point[1])

                # Manually calulate yaws since ASTAR doesn't given yaws.
                self.path_yaws = stanley_controller.calculate_yaws_from_path(
                    self.path_xs, self.path_ys, interfaces.nav_board.heading()
                )

                # Store last index of path.
                self.last_idx = len(self.path_xs) - 1

                # Update waypoint goal.
                core.waypoint_handler.set_goal(
                    utm.to_latlon(self.path_xs[-1], self.path_ys[-1], utm_current[2], utm_current[3])
                )

        """
        PATH GENERATION AND FOLLOWING.
        """
        # Check if path is empty.
        if len(self.path_xs) > 1:
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
                    # Get and plot Obstacle Coordinates
                    obstacle_coords = self.astar.get_obstacle_coords()
                    xo = [utm.from_latlon(t[0], t[1])[0] for t in obstacle_coords]
                    yo = [utm.from_latlon(t[0], t[1])[1] for t in obstacle_coords]
                    plt.plot(xo, yo, "s", label="obstacles")
                    # Get and plot Gate Coordinates
                    xo = [utm.from_latlon(t[0], t[1])[0] for t in self.gate_coords]
                    yo = [utm.from_latlon(t[0], t[1])[1] for t in self.gate_coords]
                    plt.plot(xo, yo, "1", label="gate")
                    # Get and plot Trench line Coordinates
                    xo = [t[0] for t in self.trench_mid_points]
                    yo = [t[1] for t in self.trench_mid_points]
                    plt.plot(xo, yo, "p", label="trenchline")
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
                left, right = heading_hold.get_motor_power_from_heading(
                    core.constants.GATE_APPROACH_DRIVE_POWER, goal_heading
                )
                # Set drive powers.
                self.logger.info(f"ApproachingGate: Driving at ({left}, {right})")
                interfaces.drive_board.send_drive(left, right)

                # Check if the rover is too far from the target position in path. Manually lead rover back onto path.
                if (
                    math.sqrt(
                        math.pow(self.path_xs[self.target_idx] - utm_current[0], 2)
                        + math.pow(self.path_ys[self.target_idx] - utm_current[1], 2)
                    )
                    > constants.GATE_MAX_ERROR_FROM_PATH
                ):
                    # Generate path from rovers current position.
                    path = self.astar.plan_astar_avoidance_route(
                        max_route_size=50,
                        near_object_threshold=constants.GATE_NEAR_OBSTACLE_THRESH,
                        start_gps=current,
                        waypoint_thresh=0.2,
                    )

                    # If path was generated successfully, then put it in our future path. Cut out old future.
                    if path is not None:
                        # Set path start timer.
                        self.path_start_time = time.time()

                        # Clear path xs, ys, and yaws.
                        self.path_xs.clear()
                        self.path_ys.clear()
                        self.path_yaws.clear()

                        # Append new ASTAR path onto current.
                        for point in path:
                            # Add new path starting from current location.
                            self.path_xs.append(point[0])
                            self.path_ys.append(point[1])

                        # Add trenchline to end of generated path.
                        for point in self.trench_mid_points:
                            # Add new path starting from current location.
                            self.path_xs.append(point[0])
                            self.path_ys.append(point[1])

                        # Manually calulate yaws since ASTAR doesn't given yaws.
                        self.path_yaws = stanley_controller.calculate_yaws_from_path(
                            self.path_xs, self.path_ys, interfaces.nav_board.heading()
                        )

                        # Store last index of path.
                        self.last_idx = len(self.path_xs) - 1

                        # Update waypoint goal.
                        core.waypoint_handler.set_goal(
                            utm.to_latlon(self.path_xs[-1], self.path_ys[-1], utm_current[2], utm_current[3])
                        )
        else:
            # Stop the drive board.
            interfaces.drive_board.stop()
            # Print debug that path has completed.
            self.logger.info("ApproachingGate ASTAR path is empty.")

        # Check if we've reached the end of the path.
        if self.target_idx == self.last_idx and not self.target_idx == 0 and not self.last_idx == 0:
            # Continue following path for user defined amount of time.
            if self.through_gate_timer == 0:
                # Get current time.
                self.through_gate_timer = time.time()

            # Don't transfer states until timer expires.
            if (time.time() - self.through_gate_timer) > constants.GATE_DRIVE_THROUGH_TIME:
                # Return to idle.
                return self.on_event(core.AutonomyEvents.REACHED_MARKER)

        # If we grabbed more than one, see if it's a gate
        elif len(self.tags) > 1:
            self.gate_detection_attempts += 1
            self.logger.info(f"2 Markers in frame, count:{self.gate_detection_attempts}")

        # Call AR Tag tracking code to find position and size of AR Tag
        if not core.vision.ar_tag_detector.is_gate():
            self.num_detection_attempts += 1

            # If we have attempted to track an AR Tag unsuccesfully
            # MAX_DETECTION_ATTEMPTS times, we will return to search pattern
            if self.num_detection_attempts >= core.constants.GATE_MAX_DETECTION_ATTEMPTS:
                # Reset gate counter.
                self.gate_detection_attempts = 0
                # Print logs.
                self.logger.info("Lost sign of gate, returning to Search Pattern")
                # Get current position and next desired waypoint position.
                current = interfaces.nav_board.location()
                # Set goal and current waypoint as current location.
                core.waypoint_handler.set_goal(current)
                core.waypoint_handler.set_start(current)

                return self.on_event(core.AutonomyEvents.MARKER_UNSEEN)

        return self
