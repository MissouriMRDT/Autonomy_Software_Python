import core
import time
import logging


class MultiMedia:
    """
    Interface for the multimedia, a seperate compute unit that controls a LED matrix
    onboard the rover
    """

    def __init__(self):
        # Set up RoveComm and Logger
        self.logger = logging.getLogger(__name__)

    def set_lighting_state(self, state):
        """
        Writes a operation state to the lighting board to display on the LED matrix
        """
        # Write a lighting state packet (TCP)
        core.rovecomm_node.write(
            core.RoveCommPacket(
                core.manifest["Multimedia"]["Commands"]["StateDisplay"]["dataId"],
                "B",
                (state,),
                core.manifest["Multimedia"]["Ip"],
                core.UDP_OUTGOING_PORT,
            ),
            True,
        )
