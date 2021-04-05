import time
import cv2
from algorithms import obstacle_avoider, obstacle_detector
import asyncio
from core.vision import obstacle_avoidance
from core.waypoints import WaypointHandler
import core
import interfaces
import algorithms
from core.states import RoverState


class Avoidance(RoverState):
    """
    The goal of this state is to navigate to the GPS coordinates provided by base
    station in succession, the last of which is the coordinate provided by the judges
    for that leg of the task. Coordinates before the last are simply the operators in
    base station’s best guess of the best path for the rover due to terrain identified
    on RED’s map.
    """

    def start(self):
        loop = asyncio.get_event_loop()
        self.obstacle_task = loop.create_task(core.vision.obstacle_avoidance.async_obstacle_detector())

    def exit(self):
        # Cancel all state specific coroutines
        self.obstacle_task.cancel()
        pass

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

            points = obstacle_avoider.plan_avoidance_route(angle, distance, obstacle_lat, obstacle_lon, type="Circle")

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

        return self.on_event(core.AutonomyEvents.END_OBSTACLE_AVOIDANCE)
