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
from core.states import RoverState


class Stuck(RoverState):
    """
    In this state the program will command the rover to backup.
    Its singular purpose is to prevent the rover from running into a marker of obstacle.
    """

    def start(self):
        # Intialize state member variables.
        self.logger = logging.getLogger(__name__)
        # Stop drive.
        interfaces.drive_board.stop()

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events

        :param event:
        :return: RoverState
        """
        state: RoverState = None

        if event == core.AutonomyEvents.START:
            # Update the state display on lighting to Autonomy. Go back to normal display mode.
            interfaces.multimedia_board.send_lighting_state(core.OperationState.AUTONOMY)
            # Change states to Idle
            state = core.states.Idle()

        elif event == core.AutonomyEvents.ABORT:
            # Change states to Idle
            state = core.states.Idle()

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
        # If time is even set color to pink.
        if time.time() % 2 == 0:
            # Set color to pink
            interfaces.multimedia_board.send_rgb((255, 127, 127))
        elif time.time() % 1 == 0:
            # Set color to normal autonomy red.
            interfaces.multimedia_board.send_lighting_state(core.OperationState.AUTONOMY)

        return self
