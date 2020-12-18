import core
from core.states import RoverState


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
        return core.states.Idle()

    async def run(self):
        """
        Defines regular rover operation when under this state
        """
        # Send no commands to drive board, the watchdog will trigger and stop the rover from driving anyway
        return self
