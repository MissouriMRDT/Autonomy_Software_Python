#
# Mars Rover Design Team
# multimedia_board.py
#
# Created on Jan 29, 2021
# Updated on Aug 21, 2022
#

import core
import logging


class MultiMedia:
    """
    Interface for the multimedia, a separate compute unit that controls a LED matrix onboard the rover
    """

    def __init__(self):
        # Set up logger
        self.logger = logging.getLogger(__name__)

    def send_lighting_state(self, state):
        """
        Writes an operation state to the lighting board to display on the LED matrix

        :param state: (enum) :  The operation state of the rover, defined in core/constants.py
        """
        # Write a lighting state packet (TCP)
        core.rovecomm_node.write(
            core.RoveCommPacket(
                core.manifest["Core"]["Commands"]["StateDisplay"]["dataId"],
                "B",
                (state,),
                core.manifest["Core"]["Ip"],
                core.UDP_OUTGOING_PORT,
            ),
            False,
        )

    def send_rgb(self, rgb):
        """
        Writes a rgb value to the LED matrix

        :param rgb: (tuple): the r, g, b values to be displayed on the LED matrix
        """
        # Write a lighting rgb packet (TCP)
        core.rovecomm_node.write(
            core.RoveCommPacket(
                core.manifest["Core"]["Commands"]["LEDRGB"]["dataId"],
                "B",
                rgb,
                core.manifest["Core"]["Ip"],
                core.UDP_OUTGOING_PORT,
            ),
            True,
        )
