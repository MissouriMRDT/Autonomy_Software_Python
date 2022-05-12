from typing import Tuple
from algorithms import geomath
import core
from core import constants
import logging
import time
import interfaces
from interfaces import nav_board


def clamp(n, min_n, max_n):
    """
    Clamps value n between min_n and max_n

    Parameters:
    -----------
        n - the value to be clamped
        min_n - the minimum value it can be
        max_n - the maximum value it can be
    """
    return max(min(max_n, n), min_n)


class DriveBoard:
    """
    The drive board interface wraps all driving commands for the autonomy system. It will send drive commands to the drive board on the rover,
    as well as calculate motor speeds for a desired vector.
    """

    def __init__(self):
        self._targetSpdLeft: int = 0
        self._targetSpdRight: int = 0
        self.logger: logging.Logger = logging.getLogger(__name__)

    def calculate_move(self, speed: float, angle: float) -> Tuple[int, int]:
        """
        Calculates the drives speeds given the vector (speed, angle)

        Parameters:
        -----------
            Speed: -1000 to 1000
            Angle: -360 = turn in place left, 0 = straight, 360 = turn in place right
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

        Parameters:
        -----------
            target_left (int16) - the speed to drive left motors
            target_right (int16) - the speed to drive right motors
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

    def backup(self, target_distance, speed = -200):
        """
        Backs rover up for a specified distance at specified speed

        Parameters:
        -----------
            target_distance (float) - distance to travel backwards (meters)
            speed (int16) - the speed to drive right and left motors (between -1000 and -1)
        """

        # Force distance to be positive and speed to be negative
        target_distance = abs(target_distance)
        speed = -abs(speed)

        # Initialize
        distance_traveled = 0
        start_latitude, start_longitude = interfaces.nav_board.location()

        # Check distance traveled until target distance is reached
        while(distance_traveled < target_distance):
            self.send_drive(speed, speed)
            current_latitude, current_longitude = interfaces.nav_board.location()
            bearing, distance_traveled = geomath.haversine(start_latitude, start_longitude, current_latitude, current_longitude)
            distance_traveled *= 1000 # convert km to m
            self.logger.info(f"Backing Up: {distance_traveled} meters / {target_distance} meters")
            time.sleep(core.EVENT_LOOP_DELAY)

        # Stop rover
        self.logger.info(f"Backing Up: COMPLETED")
        self.stop()


    def time_drive(self, distance):
        """
        Drives the rover in a straiht line for time x.
        x is calculated by dividing goal distance by the constant METERS_PER_SECOND

        Parameters:
        -----------
            distance (float) - distance to travel
        """
        goal_time = distance / constants.METERS_PER_SECOND
        t1 = time.time()
        t2 = time.time()

        while t2 - t1 < goal_time:
            t2 = time.time()
            interfaces.drive_board.send_drive(constants.MAX_DRIVE_POWER, constants.MAX_DRIVE_POWER)

        interfaces.drive_board.stop()
