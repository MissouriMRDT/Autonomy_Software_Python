#
# Mars Rover Design Team
# search_pattern.py
#
# Created on Dec 02, 2020
# Updated on Aug 21, 2022
#

import utm
import core
import algorithms
import interfaces
import asyncio
import time
import numpy as np
import matplotlib.pyplot as plt
from core.states import RoverState
from algorithms.obstacle_avoider import ASTAR


class SearchPattern(RoverState):
    """
    The searching state’s goal is to drive the rover in an ever expanding Archimedean spiral, searching for the AR Tag.
    The spiral type was chosen because of it’s fixed distance between each rotation’s path.
    """

    def start(self):
        """
        Schedule Search Pattern
        """
        # Create state specific variable.
        self.rover_xs = []
        self.rover_ys = []
        self.stuck_check_timer = 0
        self.stuck_check_last_position = [0, 0]

    def exit(self):
        """
        Cancel all state specific tasks
        """
        # Clear rover path.
        self.rover_xs.clear()
        self.rover_ys.clear()

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events

        :param event:
        :return: RoverState
        """
        state: RoverState = None

        if event == core.AutonomyEvents.MARKER_SEEN:
            state = core.states.ApproachingMarker()

        elif event == core.AutonomyEvents.GATE_SEEN:
            state = core.states.ApproachingGate()

        elif event == core.AutonomyEvents.START:
            state = self

        elif event == core.AutonomyEvents.SEARCH_FAILED:
            state = core.states.Idle()

        elif event == core.AutonomyEvents.ABORT:
            state = core.states.Idle()

        elif event == core.AutonomyEvents.STUCK:
            state = core.states.Stuck()

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
        gps_data = core.waypoint_handler.gps_data

        """
        STATE TRANSITION AND WAYPOINT LOGIC.
        """
        # We should be navigating, so check if we have been in the same position for awhile.
        # Only check every predefined amount of seconds.
        if (time.time() - self.stuck_check_timer) > core.constants.STUCK_UPDATE_TIME:
            # Calculate distance from goal for checking for markers and gates.
            _, distance = algorithms.geomath.haversine(
                self.stuck_check_last_position[0], self.stuck_check_last_position[1], current[0], current[1]
            )
            # Convert km to m.
            distance *= 1000

            # Store new position.
            self.stuck_check_last_position[0], self.stuck_check_last_position[1] = current[0], current[1]

            # Check if we are stuck.
            if distance < core.constants.STUCK_MIN_DISTANCE:
                # Move to stuck state.
                return self.on_event(core.AutonomyEvents.STUCK)

            # Update timer.
            self.stuck_check_timer = time.time()

        # If the gps_data is none, there were no waypoints to be grabbed,
        # so log that and return
        if gps_data is None:
            self.logger.error("SearchPattern: No waypoint, please add a waypoint to start navigating")
            return self.on_event(core.AutonomyEvents.NO_WAYPOINT)

        # Pull info out of waypoint.
        goal, start, leg_type = gps_data.data()
        self.logger.debug(
            f"Searching: Location ({interfaces.nav_board._location[0]}, {interfaces.nav_board._location[1]}) to Goal ({goal[0]}, {goal[1]})"
        )

        # Check to see if gate or marker was detected
        # If so, immediately stop all movement to ensure that we don't lose sight of the AR tag(s)
        if core.vision.ar_tag_detector.is_gate() and leg_type == "GATE":
            interfaces.drive_board.stop()

            # Sleep for a brief second
            await asyncio.sleep(core.EVENT_LOOP_DELAY)

            self.logger.info("Search Pattern: Gate seen")
            return self.on_event(core.AutonomyEvents.GATE_SEEN)

        elif core.vision.ar_tag_detector.is_marker() and leg_type == "MARKER":
            interfaces.drive_board.stop()

            # Sleep for a brief second
            await asyncio.sleep(core.EVENT_LOOP_DELAY)

            self.logger.info("Search Pattern: Marker seen")
            return self.on_event(core.AutonomyEvents.MARKER_SEEN)

        if algorithms.gps_navigate.get_approach_status(goal, current, start) != core.ApproachState.APPROACHING:
            interfaces.drive_board.stop()

            # Sleep for a little bit before we move to the next point, allows for AR Tag to be picked up
            await asyncio.sleep(core.EVENT_LOOP_DELAY)

            # Find and set the next goal in the search pattern
            goal = algorithms.marker_search.calculate_next_coordinate(start, goal)
            core.waypoint_handler.set_goal(goal)

        # Calculate drive power.
        left, right = algorithms.gps_navigate.calculate_move(goal, current, start, core.MAX_DRIVE_POWER)
        # Send drive.
        interfaces.drive_board.send_drive(left, right)
        # Send drive.
        self.logger.info(f"Search Pattern: Driving at ({left}, {right})")

        # Store rover position path.
        utm_current = utm.from_latlon(current[0], current[1])
        utm_goal = utm.from_latlon(goal[0], goal[1])
        self.rover_xs.append(utm_current[0])
        self.rover_ys.append(utm_current[1])
        # Write path 1 second before it expires.
        if int(time.time()) % 2 == 0:
            plt.cla()
            # Plot path, current location, and goal.
            plt.gca().set_aspect("equal", adjustable="box")
            plt.plot(self.rover_xs, self.rover_ys, "-b", label="trajectory")
            # Plot goal.
            plt.plot(utm_goal[0], utm_goal[1], "^", label="goal")
            # Plot rover.
            plt.plot(utm_current[0], utm_current[1], "2", label="rover")
            plt.axis("equal")
            plt.grid(True)
            plt.title(f"Simple Search Pattern - Heading: {int(interfaces.nav_board.heading())}")
            plt.savefig("logs/!rover_path.png")

        return self
