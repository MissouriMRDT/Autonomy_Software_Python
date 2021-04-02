import logging
import interfaces
from algorithms.PID_controller import PIDcontroller

pid = PIDcontroller(Kp=3, Ki=0.25, Kd=0, wraparound=360)


def clamp(x, minimum, maximum):
    """
    Clamps the x value between the min and max values

    Returns:
    --------
        val - the clamped value
    """
    return max(minimum, min(x, maximum))


def get_motor_power_from_heading(speed, goal_heading):
    """
    Derives motor power for (left, right) from the goal heading
    Uses a PID loop to adjust the turn rate to match goal heading

    Parameters:
    -----------
        speed (int) - speed to driver rover in -1000, 1000
        goal heading (int) - target heading (degrees) for the rover to drive in

    Returns:
    --------
        left, right (ints) - the adjusted motor power (-1000,1000) for left and right
        sides of the rover
    """

    logger = logging.getLogger(__name__)

    heading_correction = pid.update(goal_heading, interfaces.nav_board.heading())
    heading_correction = clamp(heading_correction, -180, 180)
    logger.debug(f"Heading: {heading_correction}, Speed: {speed}")
    return interfaces.drive_board.calculate_move(speed, heading_correction)
