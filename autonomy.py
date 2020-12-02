import algorithms
import core
import logging
import interfaces
import asyncio


def main() -> None:
    '''
    Main autonomy loop
    '''
    logger = logging.getLogger(__name__)
    logger.info("Running main autonomy loop")

    while True:
        # Run the current state in the state machine (and handle enable/disable)
        core.state_machine.run()

        logger.debug(f"Current State: {core.state_machine.state}")

        # Core state machine runs every 100 ms, to prevent unecessarily fast computation
        # sensor data is processed seperately, as that is the bulk of processing time
        await asyncio.sleep(core.constants.EVENT_LOOP_DELAY)


if __name__ == "__main__":
    # Run main()
    main()
