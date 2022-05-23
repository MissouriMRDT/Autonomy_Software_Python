from math import fabs
import math
import interfaces
from geopy import Point
from geopy.distance import VincentyDistance
from algorithms import geomath
import heapq
import logging
import utm
import core

# Create logger for file.
logger = logging.getLogger(__name__)


class Node:
    """
    A node class for A* Pathfinding
    """

    def __init__(self, parent=None, position=None):
        # Create object variables.
        self.parent = parent
        self.position = position

        self.g = 0  # distance from start.
        self.h = 0  # heuristic, distance from end.
        self.f = 0  # node cost.

    def __eq__(self, other):
        return self.position == other.position

    def __repr__(self):
        return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # Defining less than for purposes of heap queue.
    def __lt__(self, other):
        return self.f < other.f

    # Defining greater than for purposes of heap queue.
    def __gt__(self, other):
        return self.f > other.f


def return_path(current_node, utm_zone):
    """
    Uses the current node in the heapq and works backwards though the queue, adding the next
    parent node to the list of points.

    Parameters:
    -----------
        current_node - the current node to working backwards from to build the path.

    Returns:
    --------
        path - the path containing the lat and lon gps coords.
    """
    # Create instance variables.
    path = []
    current = current_node

    # Loop backwards through each parent node until none exist.
    while current is not None:
        # Use the given UTM zone to convert the UTM coords back to GPS coords.
        gps_coords = utm.to_latlon(*(current.position[0], current.position[1], utm_zone[0], utm_zone[1]))
        # Append current nodes position.
        path.append(gps_coords)
        # Set current node equal to current node's parent node.
        current = current.parent
    # Return reversed path.
    return path[::-1]


def coords_obstacle(distMeters, lat1, lon1, bearing):
    """
    Calculates the lat and lon coords of the obstacle given the current position

    Parameters:
    -----------
        distMeters - the distance from the current position to the obstacle
        lat1 - the latitude of the current position
        lon1 - the longitude of the current position
        bearing - the angle from the current position to the obstacle

    Returns:
    --------
        The coords of the obstacle
    """

    destination = VincentyDistance(meters=distMeters).destination(Point(lat1, lon1), bearing)
    lat2, lon2 = destination.latitude, destination.longitude
    return (lat2, lon2)


def plan_avoidance_route(angle, distance, obstacle_lat, obstacle_lon, type="Rectangle"):
    """
    Plans a series of GPS coordinates around an obstacle, that can be used to navigate
    around it and avoid hazardous conditions for the rover

    Parameters:
    -----------
        angle - the angle of the obstacle
        distance - the distance from the rover to the obstacle
        obstalce_lat - The latitude of the given obstacle to avoid
        obstacle_lon - The longitude of the given obstacle to avoid
        type - the type of avoidance route, can be "Circle" or "Rectangle" (default)

    Returns:
    --------
        A list of points around the obstacle that provide a safe path of traversal
    """
    if type == "Rectangle":
        points = []
        # Find point 3.5m to the right of our current location
        right_2M_lat, right_2M_lon = coords_obstacle(
            3.5,
            interfaces.nav_board.location()[0],
            interfaces.nav_board.location()[1],
            (angle + 90) % 360,
        )
        points.append((right_2M_lat, right_2M_lon))

        # Now find point 5.5 times the original distance ahead of last point
        ahead_5_5X_lat, ahead_5_5X_lon = coords_obstacle(5.5 * distance, right_2M_lat, right_2M_lon, angle)
        points.append((ahead_5_5X_lat, ahead_5_5X_lon))

        # Now move left 3.5m
        left_2M_lat, left_2M_lon = coords_obstacle(3.5, ahead_5_5X_lat, ahead_5_5X_lon, (angle - 90) % 360)
        points.append((left_2M_lat, left_2M_lon))

        # return the GPS coordinates on our route
        return points

    elif type == "Circle":
        bearing, radius = geomath.haversine(
            interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], obstacle_lat, obstacle_lon
        )
        # Convert to meters
        radius *= 1000
        # Add in an additional .5 meters to compensate for regular GPS inaccuracy
        radius += 0.5
        points = []

        increments = 4
        angle_increments = 90 / (increments - 1)
        point_angle = (angle - 90) % 360 + angle_increments

        for i in range(increments):
            points.append(coords_obstacle(radius, obstacle_lat, obstacle_lon, point_angle))
            point_angle += angle_increments

        # return the GPS coordinates on our route
        return points


class ASTAR_AVOIDER:
    def __init__(self):
        # Create class variables.
        self.obstacle_coords = []

    def update_obstacles(
        self,
        object_locations,
        min_object_distance=1.0,
        min_object_angle=-40,
        max_object_angle=40,
    ):
        """
        Loops through the given array of obstacle angles and distances and calculate their GPS->UTM position.

        Parameters:
        -----------
            object_locations - A list of tuples containing each objects angle and distance in that order.
            min_object_distance - the limit before we ignore the object, we are going to hit it.
            min_object_angle - the angle limit before we ignore the object
            max_object_angle - the angle limit before we ignore the object

        Returns:
        --------
            Nothing
        """
        # Loop through each obstacle and calculate their coords and add them to the array.
        for object in object_locations:
            # Check the object distance to see if it meets the requirements.
            if object[1] > min_object_distance and (object[0] > min_object_angle and object[0] < max_object_angle):
                # Calculate the absolute heading of the obstacle, in relation to the rover
                angle = (interfaces.nav_board.heading() + object[0]) % 360

                # Find the gps coordinate of the obstacle
                try:
                    obstacle_lat, obstacle_lon = coords_obstacle(
                        object[1], interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], angle
                    )
                except Exception:
                    continue

                # Convert GPS coords to UTM coords.
                obstacle_easting, obstacle_northing, _, _ = utm.from_latlon(obstacle_lat, obstacle_lon)
                # Determine if these coords already exist or are within 0.5 meters away from another object.
                coord_to_close = False
                for object_coord in self.obstacle_coords:
                    # Calculate staight line distance from current obstacle to be added.
                    distance = math.sqrt(
                        math.pow(obstacle_easting - object_coord[0], 2)
                        + math.pow(obstacle_northing - object_coord[1], 2)
                    )
                    # If the object is closer than half a meter away, then disregard it.
                    if distance <= 1.0:
                        coord_to_close = True

                if coord_to_close:
                    continue

                # Append to array.
                self.obstacle_coords.append((obstacle_easting, obstacle_northing))
                # If the length of the array is greater than 50, remove the oldest element.
                if len(self.obstacle_coords) > 50:
                    self.obstacle_coords.pop(0)

        print("Object Array Length:", len(self.obstacle_coords))

    def plan_astar_avoidance_route(
        self,
        max_route_size=10,
        near_object_threshold=2.0,
    ):
        """
        Uses the given list of object angles and distances, converts those to GPS waypoints, and then uses the A* (astar)
        algorithm to find the shortest path around the obstacle to a given endpoint in front of the robot.
        This function uses heapq while finding the path as it should be more effecient than using lists.

        Parameters:
        -----------
            max_route_size - the max square area available for route planning.
            near_object_threshold - the minimum distance the rover can get from the objects along the path.

        Returns:
        --------
            path - a list of gps waypoints around the path that should be safe for traversal.
        """
        # Get current gps position.
        current_gps_pos = (interfaces.nav_board.location()[0], interfaces.nav_board.location()[1])
        # Convert the gps coords to UTM coords. These coords are in meters and they are easier to work with.
        current_utm_pos = utm.from_latlon(current_gps_pos[0], current_gps_pos[1])
        utm_zone = (current_utm_pos[2], current_utm_pos[3])

        # Only continue if obstacle_coords is not empty.
        if len(self.obstacle_coords) > 0:
            ####################################################################
            # Create start and end node.
            ####################################################################
            start = Node(None, (current_utm_pos[0], current_utm_pos[1]))
            # Calculate gps coord fixed distance in front of the rover.
            # Find the gps coordinate of the end point.
            gps_data = core.waypoint_handler.get_waypoint()
            waypoint_goal, _, _ = gps_data.data()
            waypoint_goal = utm.from_latlon(waypoint_goal[0], waypoint_goal[1])
            # Calculate the midpoint between the UTM start and end points.
            # endpoint_easting, endpoint_northing = (
            #     (start.position[0] + waypoint_goal[0]) / 2,
            #     (start.position[1] + waypoint_goal[1]) / 2,
            # )
            # end = Node(None, (endpoint_easting, endpoint_northing))
            end = Node(None, (waypoint_goal[0], waypoint_goal[1]))

            # Create open and closed list.
            open_list = []
            closed_list = []

            # Heapify open list and add our start node.
            heapq.heapify(open_list)
            heapq.heappush(open_list, start)

            # Define a stop condition.
            outer_iterations = 0
            max_interations = max_route_size * max_route_size // 2

            ####################################################################
            # Define movement search pattern. In this case check in a grid pattern
            # 0.5 meters away from current position.
            ####################################################################
            offset = 0.5
            adjacent_movements = (
                (0.0, -offset),
                (0.0, offset),
                (-offset, 0.0),
                (offset, 0.0),
                (-offset, -offset),
                (-offset, offset),
                (offset, -offset),
                (offset, offset),
            )

            ####################################################################
            # Loop until the algorithm has found the end.
            ####################################################################
            while len(open_list) > 0:
                # Increment counter.
                outer_iterations += 1

                # Get the current node.
                current_node = heapq.heappop(open_list)
                closed_list.append(current_node)

                # Check if we have hit the maximum number of iterations.
                if outer_iterations > max_interations:
                    # Print info message.
                    logger.info("Unable to solve path: too many iterations.")
                    return return_path(current_node, utm_zone)

                # Found the goal.
                if (
                    fabs(current_node.position[0] - end.position[0]) <= offset
                    and fabs(current_node.position[1] - end.position[1]) <= offset
                ):
                    return return_path(current_node, utm_zone)

                ####################################################################
                # Generate children locations for current node.
                ####################################################################
                children = []
                for new_position in adjacent_movements:
                    # Calculate child node position.
                    child_node_pos = (
                        current_node.position[0] + new_position[0],
                        current_node.position[1] + new_position[1],
                    )

                    # Check if the new child node is within range of our specified area.
                    if (
                        fabs(child_node_pos[0] - start.position[0]) > max_route_size
                        or fabs(child_node_pos[1] - start.position[1]) > max_route_size
                    ):
                        continue

                    # Make sure we are not close to another object.
                    coord_to_close = False
                    for coord in self.obstacle_coords:
                        # Calculate the straight-line distance from the obstacle.
                        distance = math.sqrt(
                            math.pow(coord[0] - child_node_pos[0], 2) + math.pow(coord[1] - child_node_pos[1], 2)
                        )
                        # Check if this obstacle is close to current child node.
                        if distance <= near_object_threshold:
                            coord_to_close = True
                    # If the current child node is too close to the object skip it.
                    if coord_to_close:
                        continue

                    # Create new node with child properties.
                    new_node = Node(current_node, child_node_pos)
                    # If everything checks out, add node to the list.
                    children.append(new_node)

                ####################################################################
                # Loop through children, calculate cost, make move.
                ####################################################################
                for child in children:
                    # Check if child is already on the closed list.
                    if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                        continue

                    # Calculate f, g, and h values.
                    child.g = current_node.g + offset
                    child.h = ((child.position[0] - end.position[0]) ** 2) + (
                        (child.position[1] - end.position[1]) ** 2
                    )
                    child.f = child.g + child.h

                    # Check if child is already in the open list or a child exists that has a greater cost.
                    if (
                        len(
                            [
                                open_node
                                for open_node in open_list
                                if child.position == open_node.position and child.g > open_node.g
                            ]
                        )
                        > 0
                    ):
                        continue

                    # Add the child to the open list.
                    heapq.heappush(open_list, child)

            # If unable to calulate path, then return nothing
            logger.info("Couldn't find path around obstacle to destination.")

        return None
