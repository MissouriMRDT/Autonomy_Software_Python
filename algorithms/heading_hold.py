import logging
import interfaces
from algorithms.PID_controller import PIDcontroller

logger = logging.getLogger(__name__)
pid = PIDcontroller(Kp=3, Ki=0.25, Kd=0, wraparound=360)


def clamp(n, min_n, max_n):
    return max(min(max_n, n), min_n)


def get_motor_power_from_heading(speed, goal_heading):
    heading_correction = pid.update(goal_heading, interfaces.nav_board.heading())
    clamp(heading_correction, -180, 180)
    logger.info(f"Heading: {heading_correction}, Speed: {speed}")
    return interfaces.drive_board.calculate_move(speed, heading_correction)
