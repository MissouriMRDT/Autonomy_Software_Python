import algorithms
import core
import logging
import interfaces
from interfaces import drive_board, nav_board
from geopy import Point 
from geopy.distance import distance, VincentyDistance

def main() -> None:
    '''
    Main function for example script, tests geomath code
    '''
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")
    angle, distance = obstacle_detection()

    one_meter_from_obstacle = nav_board.location()

    # find the gps coordinate of the obstacle
    obstacle_lat, obstacle_lon = coords_obstacle(distance, 37.5677, -91.7643, angle)

    # find the gps coordinate 2m to the right of our initial location
    new_lat, new_lon = coords_obstacle(2, nav_board.location()[0], nav_board.location()[1], 90)

    while (algorithms.gps_navigate.get_approach_status((new_lat, new_lon), nav_board.location(), one_meter_from_obstacle) == constants.ApproachState.APPROACHING):

            left, right = algorithms.gps_navigate.calculate_move((new_lat, new_lon), nav_board.location(), one_meter_from_obstacle, 250)
            logger.debug(f"Navigating: Driving at ({left}, {right})")
            core.rovecomm.write(drive_board.send_drive(left, right))

    core.rovecomm.write(drive_board.send_drive(0, 0))

    # Drive past the obstacle
    new_lat2, new_lon2 = coords_obstacle(4*distance, nav_board.location()[0], nav_board.location()[1], 0)

    while (algorithms.gps_navigate.get_approach_status((new_lat2, new_lon2), nav_board.location(), (new_lat, new_lon) == constants.ApproachState.APPROACHING)):

        left, right = algorithms.gps_navigate.calculate_move((new_lat2, new_lon2), nav_board.location(), (new_lat, new_lon), 250)
        logger.debug(f"Navigating: Driving at ({left}, {right})") 
        core.rovecomm.write(drive_board.send_drive(left, right))
    
    core.rovecomm.write(drive_board.send_drive(0,0))

    # Go back to path
    new_lat3, new_lon3 = coords_obstacle(2, nav_board.location()[0], nav_board.location()[1], 270)

    while (algorithms.gps_navigate.get_approach_status((new_lat3, new_lon3), nav_board.location(), (new_lat2, new_lon3) == constants.ApproachState.APPROACHING)):
        left, right = algorithms.gps_navigate.calculate_move((new_lat3, new_lon3), nav_board.location(), (new_lat2, new_lon2), 250)
        logger.debug(f"Navigating: Driving at ({left}, {right})") 
        core.rovecomm.write(drive_board.send_drive(left, right))

    core.rovecomm.write(drive_board.send_drive(0,0))


def coords_obstacle(distMeters, lat1, lon1, bearing):
    # given: lat1, lon1, bearing, distMiles
    lat2, lon2 = VincentyDistance(meters=distMeters).destination(Point(lat1, lon1), bearing)

    return (lat2, lon2)

def obstacle_detection():


    #returns angle, distance
    return(0, 1)

if __name__ == "__main__":
    # Run main()
    main()
