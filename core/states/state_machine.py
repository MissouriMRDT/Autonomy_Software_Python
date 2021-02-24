import core
import logging
import interfaces


class StateMachine(object):
    """
    The state machine "handler" which will deal with switching between various states
    """

    def __init__(self):
        self.state = core.states.ApproachingMarker()

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

    async def run(self):
        # Handle transitions for enabling/disabling
        if self.enable_flag is True:
            self.state = self.state.on_event(core.AutonomyEvents.START)
            # Update the state display on lighting to Autonomy
            # interfaces.multimedia_board.send_lighting_state(core.OperationState.AUTONOMY)
            self.enable_flag = False

        elif self.disable_flag is True:
            self.state = self.state.on_event(core.AutonomyEvents.ABORT)
            # Update the state display on lighting to Teleop
            # interfaces.multimedia_board.send_lighting_state(core.OperationState.TELEOP)
            self.disable_flag = False

        # Run the current state
        self.state = await self.state.run()
