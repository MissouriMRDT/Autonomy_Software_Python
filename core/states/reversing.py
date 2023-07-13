#
# Mars Rover Design Team
# idle.py
#
# Created on Nov 20, 2020
# Updated on Aug 21, 2022
#

import core
import logging
import interfaces
from core.states import RoverState
from algorithms import small_movements


class Reversing(RoverState):
    """
    In this state the program will command the rover to backup.
    Its singular purpose is to prevent the rover from running into a marker of obstacle.
    """

    def start(self):
        # Intialize state member variables.
        self.logger = logging.getLogger(__name__)
        # Print log.
        //self.logger.warning("BACKING UP! AR Tag detected in front of rover.")
        # Store current location when state is entered.
        self.start_position = interfaces.nav_board.location(force_absolute=True)

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events

        :param event:
        :return: RoverState
        """
        state: RoverState = None

        if event == core.AutonomyEvents.START:
            state = self

        elif event == core.AutonomyEvents.ABORT:
            # Move back to idle.
            state = core.states.Idle()
        elif event == core.AutonomyEvents.REVERSE_COMPLETE:
            # Change states to normal navigation.
            state = core.states.Navigating()

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
        # Backup for a defined distance.
        done_reversing = small_movements.backup(
            self.start_position[0],
            self.start_position[1],
            core.constants.NAVIGATION_START_BACKUP_DISTANCE,
            core.constants.NAVIGATION_BACKUP_SPEED,
        )

        # Check if we are done reversing.
        if done_reversing:
            # Exit state.
            return self.on_event(core.AutonomyEvents.REVERSE_COMPLETE)

        return self
