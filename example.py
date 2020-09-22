from algorithms import geomath
import logging


def main() -> None:
    logger = logging.getLogger('Autonomy_Logger')
    logger.info("Executing function: main()")

    bearing, distance = geomath.haversine(37.951424, -91.768959, 37.951524, -91.768100)

    print(bearing)
    print(distance)


if __name__ == "__main__":
    # Run main()
    main()