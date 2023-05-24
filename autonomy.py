#
# Mars Rover Design Team
# run.py
#
# Created on Oct 23, 2016
# Updated on Aug 21, 2022
#

import core
import logging
import asyncio
import interfaces.nav_board
import utm

logger = logging.getLogger(__name__)


def custom_exception_handler(loop, context):
    """
    Autonomy Exception Handler

    :return: None
    """

    print(context)
    loop.stop()


def main() -> None:
    """
    Main Autonomy Loop

    :return: None
    """

    logger.info("Entering main autonomy loop")

    # Setting up the asyncio loop
    loop = asyncio.get_event_loop()
    loop.set_exception_handler(custom_exception_handler)

    # Setup feeds for AR Tag and obstacle avoidance
    core.vision.feed_handler.add_feed(2, "artag", stream_video=core.vision.STREAM_FLAG)
    core.vision.feed_handler.add_feed(3, "obstacle", stream_video=core.vision.STREAM_FLAG)

    # Create our two detection tasks
    loop.create_task(core.vision.ar_tag_detector.async_ar_tag_detector())
    # Only start obstacle detection if flag was enabled.
    if core.vision.AVOIDANCE_FLAG:
        loop.create_task(core.vision.obstacle_avoidance.async_obstacle_detector())

    # Run core autonomy state machine loop
    loop.run_until_complete(autonomy_state_loop())


async def autonomy_state_loop():
    """
    Asynchronous autonomy state machine loop

    :return: None
    """

    while True:
        # Run the current state in the state machine (and handle enable/disable)
        await core.states.state_machine.run()

        logger.info(f"Current State: {core.states.state_machine.state}")

        # Transmit the current state to Base Station

        core.rovecomm_node.write(
            core.RoveCommPacket(
                core.manifest["Autonomy"]["Telemetry"]["CurrentState"]["dataId"],
                "B",
                (core.states.state_machine.get_state_str(),),
                port=core.UDP_OUTGOING_PORT,
            ),
            False,
        )

        # Print debug
        print(interfaces.nav_board.location())
        print(interfaces.nav_board.heading())
        print(interfaces.nav_board._heading)

        # Core state machine runs every X ms, to prevent unnecessarily fast computation.
        # Sensor data is processed separately, as that is the bulk of processing time
        await asyncio.sleep(core.EVENT_LOOP_DELAY)


if __name__ == "__main__":
    # Run main()
    main()
