import core
import interfaces
import logging
import time


def main() -> None:
    logger = logging.getLogger(__name__)
    logger.info("Turning LED's RED")

    # Tell multimedia board to flash our LED matrix green to indicate reached marker
    interfaces.multimedia_board.send_lighting_state(core.OperationState.AUTONOMY)
