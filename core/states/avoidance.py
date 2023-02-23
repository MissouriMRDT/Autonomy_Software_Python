#
# Mars Rover Design Team
# avoidance.py
#
# Created on Feb 24, 2020
# Updated on Aug 21, 2022
#

import time
from algorithms import obstacle_avoider
import matplotlib.pyplot as plt
import core
import core.constants
import interfaces
import algorithms
from core.states import RoverState


class Avoidance(RoverState):
    """
    The goal of this state is to navigate around a detected obstacle
    """

    def start(self):
        # Create state specific variable.
        self.astar = obstacle_avoider.ASTAR_AVOIDER()
        self.path = []
        self.next_lat = None
        self.next_lon = None
        self.previous_loc = interfaces.nav_board.location()
        self.last_point = False
        self.path_start_time = 0.0

    def exit(self):
        # Cancel all state specific coroutines and reset state variables.
        self.path.clear()

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
                min_object_distance=1.0,
                max_object_distance=7.0,
                min_object_angle=-40,
                max_object_angle=40,
            )

        # If one or more obstacles have been detected and time since last path generation has exceeded limit, then attempt to plan a new avoidance route.
        if is_obstacle and time_since_last_path > path_expiration:
            # Pass object list to obstalce avoider algorithm for processing/calculating of path.
            path = self.astar.plan_astar_avoidance_route(max_route_size=40, near_object_threshold=1.0)

            # If path was generated successfully, then overwrite current path with new one.
            if path is not None:
                #########################################################
                # DEBUG THIS IS HELPFUL
                #########################################################
                # Get Obstacle Coordinates
                obstacle_coords = self.astar.get_obstacle_coords()
                # Append obstacle coords to path.
                all_points = path + obstacle_coords
                # Split XY array to X and Y arrays.
                xs = [t[0] * -1 for t in all_points]
                ys = [t[1] for t in all_points]
                # Plot and save output.
                plt.scatter(xs, ys)
                plt.gca().set_aspect("equal", adjustable="box")
                plt.savefig("logs/avoidance_gps_path.png")

                # Store path.
                self.path = path
                # Pop first element out.
                self.next_lat, self.next_lon = self.path.pop(0)
                # Set path start timer.
                self.path_start_time = time.time()

        # Check if path is empty.
        if len(self.path) > 0 or self.last_point:
            # Create goal coordinate with next lat lon vars.
            goal = core.Coordinate(self.next_lat, self.next_lon)
            # Get current gps location.
            current = interfaces.nav_board.location()

            # Drives to each of the points in the list of points around the object in sequence
            if (
                algorithms.gps_navigate.get_approach_status(
                    goal,
                    current,
                    self.previous_loc,
                    core.constants.WAYPOINT_DISTANCE_THRESHOLD,
                )
                == core.ApproachState.APPROACHING
            ):
                # Find and set the next goal in the path.
                left, right = algorithms.gps_navigate.calculate_move(
                    goal,
                    current,
                    self.previous_loc,
                    core.constants.MAX_DRIVE_POWER,
                )
                self.logger.info(f"Avoiding: Driving at ({left}, {right})")
                interfaces.drive_board.send_drive(left, right)
            else:
                # Store the old path point.
                self.previous_loc = core.Coordinate(self.next_lat, self.next_lon)
                # If we are at the last point in the path set toggle to loop one more time.
                if len(self.path) > 1:
                    # Pop out the next lat and lon waypoint in the path.
                    self.next_lat, self.next_lon = self.path.pop(0)
                    # Set toggle.
                    self.last_point = False
                elif not self.last_point:
                    # Pop out the next lat and lon waypoint in the path.
                    self.next_lat, self.next_lon = self.path.pop(0)
                    # Set toggle.
                    self.last_point = True
                elif len(self.path) <= 0:
                    # If path is empty and we have already completed last point iteration, reset toggle.
                    self.last_point = False
        else:
            # Stop the drive board.
            # interfaces.drive_board.stop()
            # Print debug that path has completed.
            self.logger.info(
                "Avoidance state completed a path from obstacle_avoider algorithm. Going back to Navigating."
            )
            print(len(self.path))

        # Condition for moving out of avoidance state.
        if (
            len(self.path) <= 1 and not self.last_point
        ) or self.astar.get_distance_from_goal() <= core.constants.AVOIDANCE_ENABLE_DISTANCE_THRESHOLD:
            # Clear matplotlib plt object.
            plt.clf()
            # Move states.
            return self.on_event(core.AutonomyEvents.END_OBSTACLE_AVOIDANCE)

        return self
