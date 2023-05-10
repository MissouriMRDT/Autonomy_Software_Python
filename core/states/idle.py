#
# Mars Rover Design Team
# idle.py
#
# Created on Nov 20, 2020
# Updated on Aug 21, 2022
#

import core
import time
import interfaces
from core.states import RoverState


class Idle(RoverState):
    """
    This is the default state for the state machine. In this state the program does nothing explicit.
    Its singular purpose is to keep the python program running to receive and transmit rovecomm commands
    from base station that configure the next leg’s settings and confirm them.
    """
    def __init__(self):
        self.idle_time = time.time()

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
        
        # Check if time zero.
        if self.idle_time == 0:
            self.idle_time = time.time()

        # Check if idle time is over threshold and update position.
        if time.time() - self.idle_time > core.constants.IDLE_TIME_GPS_REALIGN:
            # Check accuracy of nav board.
            if interfaces.nav_board.accuracy()[0] < core.constants.IDLE_GPS_ACCUR_THRESH:
                # Realign gps with relative.
                interfaces.nav_board.reset_start_utm()


        return self

