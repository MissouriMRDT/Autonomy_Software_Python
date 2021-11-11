import asyncio
from algorithms import geomath
from core.vision import obstacle_avoidance
from core.waypoints import WaypointHandler
import core
import interfaces
import algorithms
from core.states import RoverState
from interfaces import nav_board


class Navigating(RoverState):
    """
    The goal of this state is to navigate to the GPS coordinates provided by base
    station in succession, the last of which is the coordinate provided by the judges
    for that leg of the task. Coordinates before the last are simply the operators in
    base station’s best guess of the best path for the rover due to terrain identified
    on RED’s map.
    """

    def start(self):
        pass

    def exit(self):
        # Cancel all state specific coroutines
        pass

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events
        """
        state: RoverState = None

        if event == core.AutonomyEvents.NO_WAYPOINT:
            state = core.states.Idle()

        elif event == core.AutonomyEvents.REACHED_MARKER:
            state = core.states.Idle()

        elif event == core.AutonomyEvents.REACHED_GPS_COORDINATE:
            state = core.states.SearchPattern()

        elif event == core.AutonomyEvents.NEW_WAYPOINT:
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
        """
        last_leg_type = core.waypoint_handler.last_leg_type

        # Based on last leg type, give rover room to begin driving
        # Back up 2 meters
        if last_leg_type == "POST" or last_leg_type == "MARKER":
            backup_distance = 2 # meters
            interfaces.drive_board.backup(backup_distance)
            core.waypoint_handler.reset_last_leg_type()

        # create new position leg type 2 meters in front of rover and insert in from of queue
        elif last_leg_type == "GATE":
            forward_distance = 2
            heading = interfaces.nav_board.heading()
            latitude, longitude = nav_board.location()
            goal_latitude, goal_longitude = geomath.reverse_haversine(heading, forward_distance, latitude, longitude)
            waypoint = core.Coordinate(goal_latitude, goal_longitude)
            core.waypoint_handler.waypoints.appendleft(("POSITION", waypoint))
            self.logger.info(f"Added Position Waypoint to Front of Queue: lat ({goal_latitude}), lon ({goal_longitude})")
            core.waypoint_handler.reset_last_leg_type()

        gps_data = core.waypoint_handler.get_waypoint()

        # If the gps_data is none, there were no waypoints to be grabbed,
        # so log that and returnfg
        if gps_data is None:
            self.logger.error("Navigating: No waypoint, please add a waypoint to start navigating")
            return self.on_event(core.AutonomyEvents.NO_WAYPOINT)

        goal, start, leg_type = gps_data.data()
        current = interfaces.nav_board.location()
        self.logger.debug(
            f"Navigating: Driving to ({goal[0]}, {goal[1]}) from ({start[0]}, {start[1]}. Currently at: ({current[0]}, {current[1]}"
        )

        if (
            core.vision.obstacle_avoidance.is_obstacle()
            and core.vision.obstacle_avoidance.get_distance() < 1.5
            and core.vision.obstacle_avoidance.get_distance()
            < (
                algorithms.geomath.haversine(current[0], current[1], goal[0], goal[1])[1] * 1000
            )  # If distance to goal is less than distance to object, continue
        ):
            self.logger.info("Detected obstacle, now avoiding")
            return self.on_event(core.AutonomyEvents.OBSTACLE_AVOIDANCE)

        # Check if we are still approaching the goal
        if (
            algorithms.gps_navigate.get_approach_status(
                goal, current, start, 0.75 if (leg_type == "POSITION") else core.WAYPOINT_DISTANCE_THRESHOLD
            )
            != core.ApproachState.APPROACHING
        ):
            self.logger.info(
                f"Navigating: Reached goal ({interfaces.nav_board._location[0]}, {interfaces.nav_board._location[1]})"
            )

            # If there are more points, set the new one and start from top
            if not core.waypoint_handler.is_empty():
                gps_data = core.waypoint_handler.get_new_waypoint()
                self.logger.info(f"Navigating: Reached midpoint, grabbing new point ({goal[0]}, {goal[1]})")
                return self.on_event(core.AutonomyEvents.NEW_WAYPOINT)

            # Otherwise Trigger Next State
            else:
                # Stop all movement
                interfaces.drive_board.stop()

                # Set goal and start to current location
                core.waypoint_handler.set_goal(interfaces.nav_board.location())
                core.waypoint_handler.set_start(interfaces.nav_board.location())

                if leg_type == "POST":
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
                elif leg_type == "POSITION":
                    self.logger.info("Reached position waypoint")
                    return self.on_event(core.AutonomyEvents.REACHED_MARKER)
                else:
                    return self.on_event(core.AutonomyEvents.REACHED_GPS_COORDINATE)

        left, right = algorithms.gps_navigate.calculate_move(
            goal, interfaces.nav_board.location(), start, core.MAX_DRIVE_POWER
        )

        self.logger.debug(f"Navigating: Driving at ({left}, {right})")
        interfaces.drive_board.send_drive(left, right)
        return self
