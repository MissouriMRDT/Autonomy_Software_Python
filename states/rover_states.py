import core
import logging
import states
from enum import Enum


class StateMachine(object):
    """
    The state machine "handler" which will deal with switching between various states
    """

    def __init__(self):
        self.state = states.Idle()

        # Set shutdown and start flags
        self.enable_flag = False
        self.disable_flag = False

        # Add callbacks for autonomy controls
        core.rovecomm_node.set_callback(core.ENABLE_AUTONOMY_ID, self.enable)
        core.rovecomm_node.set_callback(core.DISABLE_AUTONOMY_ID, self.disable)

    def enable(self):
        self.enable_flag = True

    def disable(self):
        self.disable_flag = True

    async def run(self):
        # Handle transitions into Idle and Navigating states
        if self.enable_flag is True:
            self.state = Navigating()
            self.enable_flag = False

        elif self.disable_flag is True:
            self.state = Idle()
            self.disable_flag = False

        print(self.state)
        # Run the current state
        self.state = await self.state.run()
