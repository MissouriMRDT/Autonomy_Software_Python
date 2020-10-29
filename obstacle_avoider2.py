import algorithms
import core
import logging
import interfaces
from interfaces import drive_board, nav_board
from geopy import Point 
from geopy.distance import distance, VincentyDistance
import algorithms.geomath as geomath

def main() -> None:
    '''
    Main function for example script, tests geomath code
    '''
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")
    bearing, distance = algorithms.geomath.haversine(37.951424, -91.768959, 37.951524, -91.768100)

    # Get gps coords of obstacle
    obstacle_lat, obstacle_lon = coords_obstacle(distance, 37.5677, -91.7643, angle)


def semicircle_point(distance, origin_lat, origin_lon, bearing):
    radius = geomath.haversine(nav_board.location()[0], nav_board.location()[1], obstacle_lat, obstacle_lon)
    
    increments = 8

    for i in range(7):
        new_lat, new_lon = new_coords(radius, nav_board.location()[0], nav_board.location()[1], 180 / increments)

    while (algorithms.gps_navigate.get_approach_status((new_lat, new_lon), nav_board.location(), one_meter_from_obstacle) == constants.ApproachState.APPROACHING)

            left, right = algorithms.gps_navigate.calculate_move((new_lat, new_lon), nav_board.location(), one_meter_from_obstacle, 250)
            logger.debug(f"Navigating: Driving at ({left}, {right})")
            core.rovecomm.write(drive_board.send_drive(left, right))


    # Test sending RoveComm packets
    packet = core.RoveCommPacket(200, 'b', (1, 2), '')
    packet.SetIp('127.0.0.1')
    core.rovecomm.write(packet)

    logger.info(f"Calculated bearing: {bearing}")
    logger.info(f"Calculate distance: {distance}")

    logger.info(f"Measured heading: {interfaces.nav_board.heading()}")

def new_coords(distMeters, lat1, lon1, bearing):
    # given: lat1, lon1, bearing, distMiles
    lat2, lon2 = VincentyDistance(meters=distMeters).destination(Point(lat1, lon1), bearing)

    return (lat2, lon2)

if __name__ == "__main__":
    # Run main()
    main()
