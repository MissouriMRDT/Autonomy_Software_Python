import time
import core
import algorithms
import interfaces
import asyncio
from core.states import RoverState


class Avoidance(RoverState):
    """
    The searching state’s goal is to drive the rover in an ever expanding Archimedean spiral, searching for the AR Tag.
    The spiral type was chosen because of it’s fixed distance between each rotation’s path.
    """

    def start(self):
        loop = asyncio.get_event_loop()

    def exit(self):
        pass

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events
        """
        state: RoverState = None

        if event == core.AutonomyEvents.END_OBSTACLE_AVOIDANCE:
            state = core.states.Navigating()

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
        angle = core.vision.obstacle_avoidance.get_angle()
        distance = core.vision.obstacle_avoidance.get_distance()

        self.logger.info(f"{distance}")
        self.logger.info(f"Angle: {angle}")

        if angle < 0:
            angle = algorithms.obstacle_avoider.get_relative_angle_subtract(interfaces.nav_board.heading(), -angle)
        else:
            angle = algorithms.obstacle_avoider.get_relative_angle_add(interfaces.nav_board.heading(), angle)

        self.logger.info(f"Angle: {angle}")

        # Saving our starting location
        one_meter_from_obstacle = interfaces.nav_board.location()

        # find the gps coordinate of the obstacle
        obstacle_lat, obstacle_lon = algorithms.obstacle_avoider.coords_obstacle(
            distance, interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], angle
        )

        # Chart the course around the obstacle
        points = algorithms.obstacle_avoider.plan_avoidance_route(
            angle, distance, obstacle_lat, obstacle_lon, type="Circle"
        )

        previous_loc = one_meter_from_obstacle

        # Drives to each of the points in the list of points around the object in sequence
        for point in points:
            new_lat, new_lon = point
            self.logger.info(f"Driving towards : Lat: {new_lat}, Lon: {new_lon} now")
            while (
                algorithms.gps_navigate.get_approach_status(
                    core.constants.Coordinate(new_lat, new_lon), interfaces.nav_board.location(), previous_loc
                )
                == core.constants.ApproachState.APPROACHING
            ):
                left, right = algorithms.gps_navigate.calculate_move(
                    core.constants.Coordinate(new_lat, new_lon), interfaces.nav_board.location(), previous_loc, 250
                )
                self.logger.debug(f"Navigating: Driving at ({left}, {right})")
                interfaces.drive_board.send_drive(left, right)
                time.sleep(0.1)

            interfaces.drive_board.stop()
            previous_loc = core.constants.Coordinate(new_lat, new_lon)

        return self.on_event(core.AutonomyEvents.END_OBSTACLE_AVOIDANCE)
