#
# Mars Rover Design Team
# avoidance.py
#
# Created on Feb 24, 2020
# Updated on Aug 21, 2022
#

import time
from algorithms import obstacle_avoider
from algorithms import stanley_controller
from algorithms import heading_hold
import matplotlib.pyplot as plt
import core
import core.constants
import interfaces
import numpy as np
import utm
from core.states import RoverState


class Avoidance(RoverState):
    """
    The goal of this state is to navigate around a detected obstacle
    """

    def start(self):
        # Create state specific variable.
        self.astar = obstacle_avoider.ASTAR_AVOIDER()
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
        self.path_start_time = 0.0

    def exit(self):
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
        """
        state: RoverState = core.states.Idle()

        if event == core.AutonomyEvents.START:
            state = self

        elif event == core.AutonomyEvents.ABORT:
            state = core.states.Idle()

        elif event == core.AutonomyEvents.END_OBSTACLE_AVOIDANCE:
            state = core.states.state_machine.get_prev_state()

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
        """
        # Create instance variables.
        path_expiration = core.constants.AVOIDANCE_PATH_EXPIRATION_SECONDS

        # Get boolean toggle for if one or more obstacles have been detected.
        is_obstacle = core.vision.obstacle_avoidance.is_obstacle()
        # Update time since last path generation.
        time_since_last_path = time.time() - self.path_start_time

        # Get the object location list.
        object_locations = core.vision.obstacle_avoidance.get_obstacle_locations()
        # Update astar algorithm with the new obstacles if we currently detect any object.
        if object_locations is not None:
            self.astar.update_obstacles(
                object_locations,
                min_object_distance=2.0,
                max_object_distance=7.0,
                min_object_angle=-40,
                max_object_angle=40,
            )

        # If one or more obstacles have been detected and time since last path generation has exceeded limit, then attempt to plan a new avoidance route.
        if is_obstacle and time_since_last_path > path_expiration:
            # Pass object list to obstalce avoider algorithm for processing/calculating of path.
            path = self.astar.plan_astar_avoidance_route(max_route_size=40, near_object_threshold=2.0)

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

        # Check if path is empty.
        if len(self.path_xs) > 0:
            # Get current gps location.
            gps_current = interfaces.nav_board.location()
            current = utm.from_latlon(gps_current[0], gps_current[1])
            # Get current heading. Must subtract 90 because stanley controller expect a unit circle oriented heading.
            heading = np.deg2rad(interfaces.nav_board.heading() - 90)
            # Normalize heading to -pi and pi
            heading = stanley_controller.normalize_angle(-heading)

            # If this is the first run interation for the avoidance state, then initialize some variables with the current rover information.
            if self.rover_position_state is None:
                # Create new state and give intial values of the rovers current position, heading, and velocity.
                self.rover_position_state = stanley_controller.State(current[0], current[1], heading)

                # Store the initial rover state in x, y, yaw, and velocity arrays.
                self.rover_xs.append(self.rover_position_state.x)
                self.rover_ys.append(self.rover_position_state.y)
                self.rover_yaws.append(self.rover_position_state.yaw)
                self.rover_vs.append(self.rover_position_state.v)
                self.target_idx, _ = stanley_controller.calc_target_index(
                    self.rover_position_state, self.path_xs, self.path_ys
                )
            else:
                # Make sure we aren't at the end of the path.
                if self.last_idx > self.target_idx:
                    # Get velocity adjustment with P controller.
                    acceleration = stanley_controller.pid_control(
                        core.constants.AVOIDANCE_MAX_SPEED_MPS, self.rover_position_state.v
                    )
                    # Call stanley controller and get steering control adjustment and the next best target along the path.
                    delta_adjustment, self.target_idx = stanley_controller.stanley_control(
                        self.rover_position_state, self.path_xs, self.path_ys, self.path_yaws, self.target_idx
                    )
                    # Calculate adjusted heading. Add 90 to convert back to gps heading.
                    goal_heading = -np.rad2deg(heading + delta_adjustment) + 90

                    # Update the current rover state.
                    self.rover_position_state.update(current[0], current[1], heading)
                    # self.rover_position_state.update_bicycle(acceleration, delta_adjustment)

                    # Store the initial rover state in x, y, yaw, and velocity arrays.
                    self.rover_xs.append(self.rover_position_state.x)
                    self.rover_ys.append(self.rover_position_state.y)
                    self.rover_yaws.append(self.rover_position_state.yaw)
                    self.rover_vs.append(self.rover_position_state.v)

                    # Write path 1 second before it expires.
                    if int(time.time()) % 3 == 0:
                        plt.cla()
                        # Get and plot Obstacle Coordinates
                        obstacle_coords = self.astar.get_obstacle_coords()
                        xo = [utm.from_latlon(t[0], t[1])[0] for t in obstacle_coords]
                        yo = [utm.from_latlon(t[0], t[1])[1] for t in obstacle_coords]
                        # Plot path, current location, and target_index.
                        plt.plot(xo, yo, "s", label="obstacles")
                        plt.gca().set_aspect("equal", adjustable="box")
                        plt.plot(self.path_xs, self.path_ys, ".r", label="course")
                        plt.plot(self.rover_xs, self.rover_ys, "-b", label="trajectory")
                        plt.plot(self.path_xs[self.target_idx], self.path_ys[self.target_idx], "xg", label="target")
                        plt.axis("equal")
                        plt.grid(True)
                        plt.title("Rover Velocity (M/S):" + str(self.rover_position_state.v))
                        plt.savefig("logs/avoidance_gps_path.png")

                    # Send drive board commands to drive at a certain speed at a certain angle.
                    left, right = heading_hold.get_motor_power_from_heading(
                        core.constants.MAX_DRIVE_POWER, goal_heading
                    )
                    # Set drive powers.
                    self.logger.info(f"Avoidance: Driving at ({left}, {right})")
                    interfaces.drive_board.send_drive(left, right)
        else:
            # Stop the drive board.
            interfaces.drive_board.stop()
            # Print debug that path has completed.
            self.logger.info("Avoidance path is empty. Going back to Navigating.")

        # Condition for moving out of avoidance state.
        if (
            self.target_idx >= self.last_idx
        ) or self.astar.get_distance_from_goal() <= core.constants.AVOIDANCE_ENABLE_DISTANCE_THRESHOLD:
            # Clear matplotlib plt object.
            plt.clf()
            # Move states.
            return self.on_event(core.AutonomyEvents.END_OBSTACLE_AVOIDANCE)

        return self
