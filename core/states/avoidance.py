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
        self.rover_position_state = None
        self.path_xs = []
        self.path_ys = []
        self.rover_xs = []
        self.rover_ys = []
        self.rover_yaws = []
        self.rover_vs = []
        self.target_idx = None
        self.previous_loc = interfaces.nav_board.location()
        self.last_point = False
        self.path_start_time = 0.0

    def exit(self):
        # Cancel all state specific coroutines and reset state variables.
        self.astar.clear_obstacles()
        self.path_xs.clear()
        self.path_ys.clear()
        self.rover_xs.clear()
        self.rover_ys.clear()
        self.rover_yaws.clear()
        self.rover_vs.clear()
        self.target_idx = None
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
                min_object_distance=1.0,
                max_object_distance=7.0,
                min_object_angle=-40,
                max_object_angle=40,
            )

        # If one or more obstacles have been detected and time since last path generation has exceeded limit, then attempt to plan a new avoidance route.
        if is_obstacle and time_since_last_path > path_expiration:
            # Pass object list to obstalce avoider algorithm for processing/calculating of path.
            path = self.astar.plan_astar_avoidance_route(max_route_size=40, near_object_threshold=2.0)

            # If path was generated successfully, then overwrite current path with new one.
            if path is not None:
                # Store path.
                self.path_xs = [point[0] for point in path]
                self.path_ys = [point[1] for point in path]
                # Clear other rover state variables when path is regenerated.
                self.path_xs.clear()
                self.path_ys.clear()
                self.rover_xs.clear()
                self.rover_ys.clear()
                self.rover_yaws.clear()
                self.rover_vs.clear()
                self.target_idx = None
                # Set path start timer.
                self.path_start_time = time.time()

        # Check if path is empty.
        if len(self.path_xs) > 0 or self.last_point:
            # Get current gps location.
            current = interfaces.nav_board.location()
            # Get current heading.
            heading = interfaces.nav_board.heading()

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

            # Make sure we aren't at the end of the path.
        else:
            # Stop the drive board.
            # interfaces.drive_board.stop()
            # Print debug that path has completed.
            self.logger.info(
                "Avoidance state completed a path from obstacle_avoider algorithm. Going back to Navigating."
            )

        # Condition for moving out of avoidance state.
        if (
            len(self.path_xs) <= 1 and not self.last_point
        ) or self.astar.get_distance_from_goal() <= core.constants.AVOIDANCE_ENABLE_DISTANCE_THRESHOLD:
            # Clear matplotlib plt object.
            plt.clf()
            # Move states.
            return self.on_event(core.AutonomyEvents.END_OBSTACLE_AVOIDANCE)

        return self
