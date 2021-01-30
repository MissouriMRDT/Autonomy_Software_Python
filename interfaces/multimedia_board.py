import core
import logging


class MultiMedia:
    """
    Interface for the multimedia, a seperate compute unit that controls a LED matrix
    onboard the rover
    """

    def __init__(self):
        # Set up logger
        self.logger = logging.getLogger(__name__)

    def send_lighting_state(self, state):
        """
        Writes a operation state to the lighting board to display on the LED matrix

        Parameters:
            state (enum) :  The operation state of the rover, defined in
            core/constants.py
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

    def send_rgb(self, rgb):
        """
        Writes a rgb value to the LED matrix

        Parameters:
            rgb (tuple): the r, g, b values to be displayed on the LED matrix
        """
        # Write a lighting rgb packet (TCP)
        core.rovecomm_node.write(
            core.RoveCommPacket(
                core.manifest["Multimedia"]["Commands"]["LEDRGB"]["dataId"],
                "B",
                rgb,
                core.manifest["Multimedia"]["Ip"],
                core.UDP_OUTGOING_PORT,
            ),
            True,
        )
