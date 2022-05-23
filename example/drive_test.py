import core
import interfaces
import logging
import time

DRIVE_SPEED = 500
SLEEP_TIME = 5

def main() -> None:
    logger = logging.getLogger(__name__)
    logger.info("Executing file: drive_test.py")

    logger.info("Resubscribe on Basestation")
    time.sleep(5)

    interfaces.drive_board.__init__()

    logger.info("Testing driving forward")
    left = DRIVE_SPEED
    right = DRIVE_SPEED
    interfaces.drive_board.send_drive(left, right)
    for i in range(SLEEP_TIME):
        logger.info(f"Driving at ({left}, {right})")
        time.sleep(1)
    interfaces.drive_board.stop()

    logger.info("Testing driving backward")
    left = -DRIVE_SPEED
    right = -DRIVE_SPEED
    interfaces.drive_board.send_drive(left, right)
    for i in range(SLEEP_TIME):
        logger.info(f"Driving at ({left}, {right})")
        time.sleep(1)
    interfaces.drive_board.stop()

    logger.info("Testing turning couter-clockwise")
    left = -DRIVE_SPEED
    right = DRIVE_SPEED
    interfaces.drive_board.send_drive(left, right)
    for i in range(SLEEP_TIME):
        logger.info(f"Driving at ({left}, {right})")
        time.sleep(1)
    interfaces.drive_board.stop()

    logger.info("Testing turning clockwise")
    left = DRIVE_SPEED
    right = -DRIVE_SPEED
    interfaces.drive_board.send_drive(left, right)
    for i in range(SLEEP_TIME):
        logger.info(f"Driving at ({left}, {right})")
        time.sleep(1)
    interfaces.drive_board.stop()

    
   

    