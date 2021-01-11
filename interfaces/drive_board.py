from typing import Tuple
import core
import logging
import core.constants as constants


def clamp(n, min_n, max_n):
    return max(min(max_n, n), min_n)


class DriveBoard:
    """
    The drive board interface wraps all driving commands for the autonomy system. It will send drive commands to the drive board on the rover,
    as well as calculate motor speeds for a desired vector.
    """

    def __init__(self):
        self._targetSpdLeft = 0
        self._targetSpdRight = 0
        self.logger = logging.getLogger(__name__)

    def calculate_move(self, speed, angle) -> Tuple[int, int]:
        """Speed: -1000 to 1000
        Angle: -360 = turn in place left, 0 = straight, 360 = turn in place right"""

        speed_left = speed_right = speed

        if angle > 0:
            speed_right = speed_right * (1 - (angle / 180.0))
        elif angle < 0:
            speed_left = speed_left * (1 + (angle / 180.0))

        self._targetSpdLeft = int(clamp(speed_left, -constants.DRIVE_POWER, constants.DRIVE_POWER))
        self._targetSpdRight = int(clamp(speed_right, -constants.DRIVE_POWER, constants.DRIVE_POWER))
        self.logger.debug(f"Driving at ({self._targetSpdLeft}, {self._targetSpdRight})")

        return self._targetSpdLeft, self._targetSpdRight

    def send_drive(self, target_left, target_right):
        """
        Sends a rovecomm packet with the specified drive speed
        """
        # Write a drive packet (UDP)
        core.rovecomm_node.write(
            core.RoveCommPacket(
                core.DRIVE_DATA_ID,
                "h",
                (target_left, target_right),
                core.DRIVE_BOARD_IP,
            ),
            False,
        )

    def stop(self):
        """
        Sends a rovecomm packet with a 0, 0 to indicate full stop
        """
        # Write a drive packet of 0s (to stop)
        core.rovecomm_node.write(
            core.RoveCommPacket(core.DRIVE_DATA_ID, "h", (0, 0), core.DRIVE_BOARD_IP),
            False,
        )
