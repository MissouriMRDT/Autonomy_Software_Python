#
# Mars Rover Design Team
# idle.py
#
# Created on Nov 20, 2020
# Updated on Aug 21, 2022
#

import core
import time
import logging
import interfaces
import algorithms
from core.states import RoverState


class Idle(RoverState):
    """
    This is the default state for the state machine. In this state the program does nothing explicit.
    Its singular purpose is to keep the python program running to receive and transmit rovecomm commands
    from base station that configure the next leg’s settings and confirm them.
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.idle_time = time.time()
        self.realigned = False

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events

        :param event:
        :return: RoverState
        """
        state: RoverState = None
        if event == core.AutonomyEvents.START:
            # Set time to zero.
            self.idle_time = 0
            # Check if an ar tag is in front of us.
            if (
                len(
                    [
                        tag.distance
                        for tag in core.vision.ar_tag_detector.get_valid_tags()
                        if tag.distance <= core.constants.NAVIGATION_BACKUP_TAG_DISTANCE_THRESH
                    ]
                )
                >= 1
            ):
                # Move to reversing state.
                state = core.states.Reversing()
            else:
                # Change states.
                state = core.states.Navigating()

        elif event == core.AutonomyEvents.ABORT:
            state = self

        else:
            self.logger.error(f"Unexpected event {event} for state {self}")
            state = self

        # Call exit() if we are not staying the same state
        if state != self:
            self.exit()

        # Return the state appropriate for the event
        return state

    async def run(self):
        """
        Defines regular rover operation when under this state
        """
        # Send no commands to drive board, the watchdog will trigger and stop the rover from driving anyway
        # The only way to get out of this is through the state machine enable(), triggered by RoveComm

        # Only realign if not mode sim.
        if core.MODE != "SIM":
            # Check if time zero.
            if self.idle_time == 0:
                self.idle_time = time.time()

            # Check if idle time is over threshold and update position.
            if time.time() - self.idle_time > core.constants.IDLE_TIME_GPS_REALIGN and not self.realigned:
                # Check odd time.
                if int(time.time()) % 5 == 0:
                    # Check accuracy of nav board.
                    if interfaces.nav_board.accuracy()[0] < core.constants.IDLE_GPS_ACCUR_THRESH:
                        # Realign gps with relative.
                        interfaces.nav_board.realign()
                        # Print warning that GPS location has been realigned.
                        self.logger.warning(f"Relative positional tracking has been realigned to current GPS location.")
                        # Set toggle.
                        self.realigned = True
            else:
                # Reset toggle.
                self.realigned = False

        return self
