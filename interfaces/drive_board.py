#
# Mars Rover Design Team
# drive_board.py
#
# Created on Jul 19, 2020
# Updated on Aug 21, 2022
#

from typing import Tuple
import core
from algorithms.clamp import clamp
import logging


class DriveBoard:
    """
    The drive board interface wraps all driving commands for the autonomy system. It will send drive commands to the
    drive board on the rover, as well as calculate motor speeds for a desired vector.
    """

    def __init__(self):
        self._targetSpdLeft: int = 0
        self._targetSpdRight: int = 0
        self.logger: logging.Logger = logging.getLogger(__name__)

    def calculate_move(self, speed: float, angle: float) -> Tuple[int, int]:
        """
        Calculates the drives speeds given the vector (speed, angle)

        :param speed: -1000 to 1000
        :param angle: -360 = turn in place left, 0 = straight, 360 = turn in place right
        :return: Tuple[int, int]
        """

        speed_left = speed_right = speed

        if angle > 0:
            speed_right = speed_right * (1 - (angle / 180.0))
        elif angle < 0:
            speed_left = speed_left * (1 + (angle / 180.0))

        self._targetSpdLeft: int = int(clamp(speed_left, core.MIN_DRIVE_POWER, core.MAX_DRIVE_POWER))
        self._targetSpdRight: int = int(clamp(speed_right, core.MIN_DRIVE_POWER, core.MAX_DRIVE_POWER))

        self.logger.debug(f"Driving at ({self._targetSpdLeft}, {self._targetSpdRight})")

        return self._targetSpdLeft, self._targetSpdRight

    def send_drive(self, target_left: int, target_right: int) -> None:
        """
        Sends a rovecomm packet with the specified drive speed

        :param target_left: (int16) - the speed to drive left motors
        :param target_right: (int16) - the speed to drive right motors
        """

        # Write a drive packet (UDP)
        core.rovecomm_node.write(
            core.RoveCommPacket(
                core.manifest["Drive"]["Commands"]["DriveLeftRight"]["dataId"],
                "h",
                (target_left, target_right),
                core.manifest["Drive"]["Ip"],
                core.UDP_OUTGOING_PORT,
            ),
            False,
        )

    def stop(self) -> None:
        """
        Sends a rovecomm packet with a 0, 0 to indicate full stop
        """

        # Write a drive packet of 0s (to stop)
        core.rovecomm_node.write(
            core.RoveCommPacket(
                core.manifest["Drive"]["Commands"]["DriveLeftRight"]["dataId"],
                "h",
                (0, 0),
                core.manifest["Drive"]["Ip"],
                core.UDP_OUTGOING_PORT,
            ),
            False,
        )
