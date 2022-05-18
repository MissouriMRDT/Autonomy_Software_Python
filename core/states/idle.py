from cgitb import small
import core
from core.states import RoverState
from algorithms import small_movements
import asyncio
import argparse
import run

class Idle(RoverState):
    """
    This is the default state for the state machine. In this state the program does nothing explicit.
    Its singular purpose is to keep the python program running to receive and transmit rovecomm commands
    from base station that configure the next leg’s settings and confirm them.
    """

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events
        """
        state: RoverState = None

        if event == core.AutonomyEvents.START:
            if core.backup == "YES":
                small_movements.time_drive(2, False) #Back up 2 meters.
                small_movements.rotate_rover(-60)
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
        return self
