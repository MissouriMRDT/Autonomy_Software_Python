import core
import logging
from enum import Enum


class StateMachine(object):
    """
    The state machine "handler" which will deal with switching between various states
    """

    def __init__(self):
        self.state = core.states.Idle()

        # Set shutdown and start flags
        self.enable_flag = False
        self.disable_flag = False
        self.logger = logging.getLogger(__name__)

        # Add callbacks for autonomy controls
        core.rovecomm_node.set_callback(core.ENABLE_AUTONOMY_ID, self.enable)
        core.rovecomm_node.set_callback(core.DISABLE_AUTONOMY_ID, self.disable)

    def enable(self, packet):
        self.enable_flag = True

    def disable(self, packet):
        self.disable_flag = True

    async def run(self):
        # Handle transitions for enabling/disabling
        if self.enable_flag is True:
            self.state = self.state.on_event(core.START)
            self.enable_flag = False

        elif self.disable_flag is True:
            self.state = self.state.on_event(core.ABORT)
            self.disable_flag = False

        self.logger.info(self.state)

        # Run the current state
        self.state = await self.state.run()
