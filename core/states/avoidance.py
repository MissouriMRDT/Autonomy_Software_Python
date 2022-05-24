import time
import cv2
from algorithms import obstacle_avoider
import asyncio
from core.waypoints import WaypointHandler
import matplotlib.pyplot as plt
import core
import time
import interfaces
import algorithms
from core.states import RoverState

# Initialize avoider algorithm.
astar = obstacle_avoider.ASTAR_AVOIDER()


class Avoidance(RoverState):
    """
    The goal of this state is to navigate around a detected obstacle
    """

    def start(self):
        # Create state specific variable.
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
        state: RoverState = None

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
        path_expiration = 10.0

        # This is the ASTAR detection method.
        if core.vision.obstacle_avoidance.DETECTION_METHOD == 2:
            # Get boolean toggle for if one or more obstacles have been detected.
            is_obstacle = core.vision.obstacle_avoidance.is_obstacle()
            # Update time since last path generation.
            time_since_last_path = time.time() - self.path_start_time

            # Get the object location list.
            object_locations = core.vision.obstacle_avoidance.get_obstacle_locations()
            # Update astar algorithm with the new obstacles if we currently detect any object.
            if object_locations is not None:
                astar.update_obstacles(
                    object_locations,
                    min_object_distance=1.0,
                    max_object_distance=5.0,
                    min_object_angle=-20,
                    max_object_angle=20,
                )

            # If one or more obstacles have been detected and time since last path generation has exceeded limit, then attempt to plan a new avoidance route.
            if is_obstacle and time_since_last_path > path_expiration:
                # Pass object list to obstalce avoider algorithm for processing/calculating of path.
                path = astar.plan_astar_avoidance_route(max_route_size=40, near_object_threshold=1.5)

                # If path was generated successfully, then overwrite current path with new one.
                if path is not None:
                    #########################################################
                    # DEBUG THIS IS HELPFUL
                    #########################################################
                    # Get Obstacle Coordinates
                    obstacle_coords = astar.get_obstacle_coords()
                    print("Obstacle Coords Length:", len(obstacle_coords))
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
                        0.6,
                    )
                    == core.ApproachState.APPROACHING
                ):
                    # Find and set the next goal in the path.
                    left, right = algorithms.gps_navigate.calculate_move(
                        goal,
                        current,
                        self.previous_loc,
                        175,
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
                interfaces.drive_board.stop()
                # Print debug that path has completed.
                self.logger.info(
                    "Avoidance state completed a path from obstacle_avoider algorithm. Going back to Navigating."
                )

            # Condition for moving out of avoidance state.
            if (len(self.path) <= 0 and not self.last_point) or astar.get_distance_from_goal() <= 5.0:
                # Cleat matplotlib plt object.
                plt.clf()
                # Move states.
                return self.on_event(core.AutonomyEvents.END_OBSTACLE_AVOIDANCE)

        elif core.vision.obstacle_avoidance.DETECTION_METHOD == 0:
            # Finding the obstacle
            is_obstacle = core.vision.obstacle_avoidance.is_obstacle()
            self.logger.info(f"{is_obstacle}")

            if is_obstacle:
                angle = core.vision.obstacle_avoidance.get_angle()
                distance = core.vision.obstacle_avoidance.get_distance()

                # Calculate the absolute heading of the obstacle, in relation to the rover
                angle = (interfaces.nav_board.heading() + angle) % 360

                # find the gps coordinate of the obstacle
                obstacle_lat, obstacle_lon = obstacle_avoider.coords_obstacle(
                    distance, interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], angle
                )

                points = obstacle_avoider.plan_avoidance_route(
                    angle, distance, obstacle_lat, obstacle_lon, type="Circle"
                )

                previous_loc = interfaces.nav_board.location()

                # Drives to each of the points in the list of points around the object in sequence
                for point in points:
                    new_lat, new_lon = point
                    self.logger.info(f"Driving towards : Lat: {new_lat}, Lon: {new_lon} now")
                    while (
                        algorithms.gps_navigate.get_approach_status(
                            core.Coordinate(new_lat, new_lon),
                            interfaces.nav_board.location(),
                            previous_loc,
                            0.5,
                        )
                        == core.ApproachState.APPROACHING
                    ):
                        left, right = algorithms.gps_navigate.calculate_move(
                            core.Coordinate(new_lat, new_lon),
                            interfaces.nav_board.location(),
                            previous_loc,
                            250,
                        )

                        self.logger.debug(f"Navigating: Driving at ({left}, {right})")
                        interfaces.drive_board.send_drive(left, right)
                        time.sleep(0.01)
                    interfaces.drive_board.stop()
                    previous_loc = core.Coordinate(new_lat, new_lon)

            # Move states.
            return self.on_event(core.AutonomyEvents.END_OBSTACLE_AVOIDANCE)

        return self
