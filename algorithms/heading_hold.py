import logging
import interfaces
from algorithms.PID_controller import PIDcontroller

pid = PIDcontroller(Kp=3, Ki=0.25, Kd=0, wraparound=360)


def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))


def get_motor_power_from_heading(speed, goal_heading):
    logger = logging.getLogger(__name__)

    heading_correction = pid.update(goal_heading, interfaces.nav_board.heading())
    heading_correction = clamp(heading_correction, -180, 180)
    logger.debug(f"Heading: {heading_correction}, Speed: {speed}")
    return interfaces.drive_board.calculate_move(speed, heading_correction)
