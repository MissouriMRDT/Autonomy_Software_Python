import core
import interfaces
import logging
import time

from interfaces import nav_board


def main() -> None:
    while(1):
        logger = logging.getLogger(__name__)
        logger.info("Executing file: gps_test.py")

        currentPosition = nav_board.location
        logger.info(f"Current Position: ({currentPosition[0]}, {currentPosition[1]})")
        logger.info(f"Heading: {nav_board.heading}")
        logger.info(f"Pitch: {nav_board.pitch}, Roll: {nav_board.roll}")



    
