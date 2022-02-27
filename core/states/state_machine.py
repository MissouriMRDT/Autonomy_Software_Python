from core.states.state import RoverState
import core
import logging
import interfaces


class StateMachine(object):
    """
    The state machine "handler" which will deal with switching between various states
    """

    def __init__(self):
        self.state: RoverState = core.states.Idle()
        self.prev_state: RoverState = core.states.Idle()

        # Set shutdown and start flags
        self.enable_flag = False
        self.disable_flag = False
        self.logger = logging.getLogger(__name__)

        # Add callbacks for autonomy controls
        core.rovecomm_node.set_callback(core.manifest["Autonomy"]["Commands"]["StartAutonomy"]["dataId"], self.enable)
        core.rovecomm_node.set_callback(
            core.manifest["Autonomy"]["Commands"]["DisableAutonomy"]["dataId"], self.disable
        )

    def enable(self, packet):
        self.enable_flag = True

    def disable(self, packet):
        self.disable_flag = True

    def get_prev_state(self):
        return self.prev_state

    def get_state_str(self) -> int:
        return core.states.StateMapping[str(self.state)]

    async def run(self):
        # Handle transitions for enabling/disabling
        if self.enable_flag is True:
            self.state = self.state.on_event(core.AutonomyEvents.START)
            # Update the state display on lighting to Autonomy
            interfaces.multimedia_board.send_lighting_state(core.OperationState.AUTONOMY)
            self.enable_flag = False

        elif self.disable_flag is True:
            self.state = self.state.on_event(core.AutonomyEvents.ABORT)
            # Update the state display on lighting to Teleop
            interfaces.multimedia_board.send_lighting_state(core.OperationState.TELEOP)
            self.disable_flag = False

        # Run the current state
        new_state = await self.state.run()

        # Save prev state if we transition
        if new_state != self.state:
            self.prev_state = self.state

        self.state = new_state
