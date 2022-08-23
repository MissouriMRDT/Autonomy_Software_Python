#
# Mars Rover Design Team
# reached_marker_test.py
#
# Created on May 19, 2021
# Updated on Aug 21, 2022
#

import core
import interfaces
import logging
import time


def main() -> None:
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    logger.info("Resubscribe on Basestation")
    time.sleep(5)

    logger.info("Reached Marker")

    # Transmit that we have reached the marker
    core.rovecomm_node.write(
        core.RoveCommPacket(
            core.manifest["Autonomy"]["Telemetry"]["ReachedMarker"]["dataId"],
            "B",
            (1,),
        ),
        False,
    )

    # Tell multimedia board to flash our LED matrix green to indicate reached marker
    interfaces.multimedia_board.send_lighting_state(core.OperationState.REACHED_MARKER)
