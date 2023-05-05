#
# Mars Rover Design Team
# approaching_gate.py
#
# Created on May 19, 2021
# Updated on Aug 21, 2022
#

import core
import core.constants
import interfaces
import logging
from algorithms.obstacle_avoider import ASTAR
from core.states import RoverState
import time
import math


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
        self.astar = ASTAR()
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
        # Create logger object.
        logger = logging.getLogger(__name__)

        # Call AR Tag tracking code to find position and size of AR Tag
        if core.vision.ar_tag_detector.is_gate():
            # Use get_tags to create an array of the 2 gate posts
            # (named tuples containing the distance and relative angle from the camera)
            tags = core.vision.ar_tag_detector.get_tags()
            leg_type = core.waypoint_handler.get_waypoint().data()[2]

            # If we've seen at least 5 frames of 2 tags, assume it's a gate
            if (
                len(tags) == 2
                and self.gate_detection_attempts >= core.constants.ARUCO_FRAMES_DETECTED
                and leg_type == "GATE"
            ):
                
                # return self.on_event(core.AutonomyEvents.REACHED_MARKER)

            # If we grabbed more than one, see if it's a gate
            elif len(tags) > 1:
                self.gate_detection_attempts += 1
                self.logger.info(f"2 Markers in frame, count:{self.gate_detection_attempts}")

        else:
            self.num_detection_attempts += 1
            self.gate_detection_attempts = 0

            # If we have attempted to track an AR Tag unsuccesfully
            # MAX_DETECTION_ATTEMPTS times, we will return to search pattern
            if self.num_detection_attempts >= core.MAX_DETECTION_ATTEMPTS:
                self.logger.info("Lost sign of gate, returning to Search Pattern")
                return self.on_event(core.AutonomyEvents.MARKER_UNSEEN)

        return self
