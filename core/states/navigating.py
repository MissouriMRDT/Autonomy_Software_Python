import core
from core.state import RoverState
from core import waypoints, constants, states
from algorithms.gps_navigate import get_approach_status, calculate_move
from interfaces import nav_board, drive_board


class Navigating(RoverState):
    """
    The goal of this state is to navigate to the GPS coordinates provided by base station in succession, the last of which is the coordinate provided by the judges for that leg of the task.
    Coordinates before the last are simply the operators in base station’s best guess of the best path for the rover due to terrain identified on RED’s map.
    """

    async def run(self) -> RoverState:
        gps_data = waypoints.get_waypoint()

        # If the gps_data is none, there were no waypoints to be grabbed,
        # so log that and return
        if gps_data is None:
            self.logger.error("Navigating: No waypoint, please add a waypoint to start navigating")
            return states.Idle()

        goal, start = gps_data
        current = nav_board.location()

        self.logger.debug(f"Navigating: Driving to ({goal[0]}, {goal[1]}) from ({start[0]}, {start[1]}. Currently at: ({current[0]}, {current[1]}")

        # Check if we are still approaching the goal
        if get_approach_status(goal, current, start) != constants.ApproachState.APPROACHING:
            self.logger.info(f"Navigating: Reached goal ({nav_board._location[0]}, {nav_board._location[1]})")

            # If there are more points, set the new one and start from top
            if waypoints.waypoints:
                waypoints.set_gps_waypoint()
                gps_data = waypoints.get_waypoint()
                self.logger.info(f"Navigating: Reached midpoint, grabbing new point ({goal[0]}, {goal[1]})")
                return states.Navigating()

            # Otherwise Trigger Search Pattern
            else:
                # Stop all movement
                drive_board.send_drive(0, 0)

                # return states.SearchPattern()

        left, right = calculate_move(goal, nav_board.location(), start, constants.DRIVE_POWER)
        self.logger.debug(f"Navigating: Driving at ({left}, {right})")
       drive_board.send_drive(left, right)
        return states.Navigating()
