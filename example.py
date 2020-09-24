import algorithms
import core
import logging


def main() -> None:
    '''
    Main function for example script, tests geomath code
    '''
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")
    bearing, distance = algorithms.geomath.haversine(37.951424, -91.768959, 37.951524, -91.768100)

    # Test sending RoveComm packets
    packet = core.RoveCommPacket(200, 'b', (1, 2), '')
    packet.SetIp('127.0.0.1')
    core.rovecomm_node.write(packet)

    logger.info(f"Calculated bearing: {bearing}")
    logger.info(f"Calculate distance: {distance}")


if __name__ == "__main__":
    # Run main()
    main()