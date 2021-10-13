import algorithms
import core
import logging
import interfaces


def main() -> None:
    """
    Main function for example script, tests geomath code
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")
    bearing, distance = algorithms.geomath.haversine(37.951424, -91.768959, 37.951524, -91.768100)

    # Test sending RoveComm packets
    packet = core.RoveCommPacket(200, "b", (1, 2), "")
    packet.SetIp("127.0.0.1")
    core.rovecomm_node.write(packet, False)

    # Test setting the lighting panel to indicate Autonomous operation
    interfaces.multimedia_board.send_lighting_state(core.OperationState.TELEOP)

    logger.info(f"Calculated bearing: {bearing}")
    logger.info(f"Calculate distance: {distance}")

    logger.info(f"Measured heading: {interfaces.nav_board.heading()}")

    try:
        print(1 / 0)
    except Exception:
        logger.exception("Unable to print")


if __name__ == "__main__":
    # Run main()
    main()
