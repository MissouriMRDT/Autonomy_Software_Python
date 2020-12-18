import algorithms
import core
import logging
import interfaces
from interfaces import drive_board, nav_board
from geopy import Point 
from geopy.distance import distance, VincentyDistance
import matplotlib.pyplot as plt


def get_relative_angle_subtract(angle, angle2):
    diff = angle - angle2
    if diff > 0:
        return diff
    else: 
        return 360 + diff

def plan_avoidance_route(angle, distance):
    # Find point 2m to the right of our current location
    right_2M_lat, right_2M_lon = coords_obstacle(2, nav_board.location()[0], nav_board.location()[1], get_relative_angle_subtract(angle, 90))

    # Now find point 4m ahead of last point
    ahead_4X_lat, ahead_4X_lon = coords_obstacle(4*distance, right_2M_lat, right_2M_lon, angle)


    # Now move left 2m
    left_2M_lat, left_2M_lon = coords_obstacle(2, ahead_4X_lat, ahead_4X_lon, get_relative_angle_subtract(angle, 270))

    # return the GPS coordinates on our route
    return (right_2M_lat, right_2M_lon), (ahead_4X_lat, ahead_4X_lon), (left_2M_lat, left_2M_lon)



def main() -> None:
    '''
    Main function for example script, tests geomath code
    '''
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")
    
    drive_board.enable()

    points = []

    # Finding the obstacle
    angle, distance = obstacle_detection()

    # Saving our starting location
    one_meter_from_obstacle = nav_board.location()

    # find the gps coordinate of the obstacle
    obstacle_lat, obstacle_lon = coords_obstacle(distance, nav_board.location()[0], nav_board.location()[1], angle)

    # Chart the course around the obstacle
    (right_2M_lat, right_2M_lon), (ahead_4X_lat, ahead_4X_lon), (left_2M_lat, left_2M_lon) = plan_avoidance_route(angle, distance)

    #points.append((nav_board.location()[1], nav_board.location()[0]))
    points.append((right_2M_lon, right_2M_lat))
    points.append((ahead_4X_lon, ahead_4X_lat))
    points.append((left_2M_lon, left_2M_lat))

    #plt.plot(*zip(*points))
    #plt.show()

    previous_loc = one_meter_from_obstacle

    for point in points:
        new_lat, new_lon = point
        while (algorithms.gps_navigate.get_approach_status(core.constants.Coordinate(new_lat, new_lon), nav_board.location(), previous_loc) == core.constants.ApproachState.APPROACHING):
                logger.info(f"Target coordinates: Lat: {new_lat}, Lon: {new_lon}")
                left, right = algorithms.gps_navigate.calculate_move(core.constants.Coordinate(new_lat, new_lon), nav_board.location(), previous_loc, 250)
                logger.debug(f"Navigating: Driving at ({left}, {right})")
                core.rovecomm.write(drive_board.send_drive(left, right))

        core.rovecomm.write(drive_board.send_drive(0, 0))
        previous_loc = core.constants.Coordinate(new_lat, new_lon)
    
def coords_obstacle(distMeters, lat1, lon1, bearing):
    # given: lat1, lon1, bearing, distMiles
    destination = VincentyDistance(meters=distMeters).destination(Point(lat1, lon1), bearing)
    lat2, lon2 = destination.latitude, destination.longitude
    return (lat2, lon2)

def obstacle_detection():
    #returns angle, distance
    return(nav_board.heading(), 1)

if __name__ == "__main__":
    # Run main()
    main()
