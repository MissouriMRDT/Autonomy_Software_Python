import core
import logging
import asyncio

logger = logging.getLogger(__name__)


def main() -> None:
    """
    Main autonomy loop
    """
    logger.info("Entering main autonomy loop")

    loop = asyncio.get_event_loop()

    # Create any additional async tasks
    # loop.create_task(do_ar_tag())

    # Run core autonomy state machine loop
    loop.run_until_complete(autonomy_state_loop())


async def autonomy_state_loop():
    while True:
        # Run the current state in the state machine (and handle enable/disable)
        await core.states.state_machine.run()

        logger.debug(f"Current State: {core.states.state_machine.state}")

        # Transmit the current state to Base Station
        core.rovecomm_node.write(
            core.RoveCommPacket(
                core.manifest["Autonomy"]["Telemetry"]["CurrentState"]["dataId"],
                "B",
                (core.states.StateMapping[core.states.state_machine.state],),
                port=core.UDP_OUTGOING_PORT,
            ),
            False,
        )

        # Core state machine runs every X ms, to prevent unecessarily fast computation.
        # Sensor data is processed seperately, as that is the bulk of processing time
        await asyncio.sleep(core.EVENT_LOOP_DELAY)


if __name__ == "__main__":
    # Run main()
    main()
