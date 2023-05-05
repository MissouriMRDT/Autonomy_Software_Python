from math import fabs
import math
import interfaces
from geopy import Point
from geopy.distance import VincentyDistance
import numpy as np
import heapq
import logging
import utm
import core
from core import constants

# Create logger for file.
logger = logging.getLogger(__name__)


class Node:
    """
    This class serves as an easy way to create 'nodes' within a path. Each node is created as a seperate object
    and contains a 'pointer' or reference to its parent node which is a completely seperate object.
    Each node also stores its own location, distance, heuristic, and total code withing the path. These values
    are not autocalculated and must be determined by the programmer.
    """

    def __init__(self, parent=None, position=None):
        """
        Initializes the Node class.

        :params parent: A reference to another Node object to treat as the parent.
        :params position: A tuple or list (with dimensions (1,2)) that stores integers representing the Nodes position.
        """
        # Create object variables.
        self.parent = parent
        self.position = position

        self.g = 0  # distance from start.
        self.h = 0  # heuristic, distance from end.
        self.f = 0  # node cost.

    def __eq__(self, other):
        """
        Equals operator for comparing two Nodes. Equality based off of position.

        :returns boolean is_equal: Whether or not the two nodes are equal.
        """
        return self.position == other.position

    def __repr__(self):
        """
        A to_string sort of method to print node stats.

        :returns string output: The node summary output.
        """
        return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    def __lt__(self, other):
        """
        Defining less than for purposes of heap queue.

        :returns boolean is_less_than: Whether or not this Node is less than the other.
        """
        return self.f < other.f

    def __gt__(self, other):
        """
        Defining greater than for purposes of heap queue.

        :returns boolean is_greater_than: Whether or not this Node is greater than the other.
        """
        return self.f > other.f


def return_path(current_node, utm_zone, return_gps=False):
    """
    Uses the current node in the heapq and works backwards though the queue, adding the next
    parent node to the list of points.

    :param current_node: The current node to working backwards from to build the path.
    :param utm_zone: Your current UTM zone on the planet Earth. This must be correct of GPS coords will not be converted correctly.
    :param return_gps: Whether or not to conver the coords from utm to gps before returning.

    :returns path: The compiled path containing the lat and lon gps coords.
    """
    # Create instance variables.
    path = []
    current = current_node

    # Loop backwards through each parent node until none exist.
    while current is not None:
        # Get coords from node.
        coords = current.position

        # Check if we should convert utm to gps.
        if return_gps:
            # Use the given UTM zone to convert the UTM coords back to GPS coords.
            coords = utm.to_latlon(*(coords[0], coords[1], utm_zone[0], utm_zone[1]))

        # Append current nodes position.
        path.append(coords)
        # Set current node equal to current node's parent node.
        current = current.parent

    # Return reversed path.
    return path[::-1]


def coords_obstacle(distMeters, lat1, lon1, bearing):
    """
    Calculates the lat and lon coords of the obstacle given the current position

    :params distMeters: the distance from the current position to the obstacle
    :params lat1: the latitude of the current position
    :params lon1: the longitude of the current position
    :params bearing: the angle from the current position to the obstacle

    :returns gps_location: A tuple with the coords of the obstacle in GPS format.
    """

    destination = VincentyDistance(meters=distMeters).destination(Point(lat1, lon1), bearing)
    lat2, lon2 = destination.latitude, destination.longitude
    return (lat2, lon2)


class ASTAR:
    def __init__(self):
        """
        Initialize the ASTAR class.
        """
        # Create class variables.
        self.obstacle_coords = []
        self.utm_zone = []
        self.start = Node()
        self.end = Node()

    def update_obstacles(
        self,
        object_locations,
        min_object_distance=1.0,
        max_object_distance=15.0,
        min_object_angle=-40,
        max_object_angle=40,
    ):
        """
        Loops through the given array of obstacle angles and distances and calculate their GPS->UTM position.

        :params object_locations: A list of tuples containing each objects angle and distance in that order.
        :params min_object_distance: the limit before we ignore the object, we are going to hit it.
        :params min_object_angle: the angle limit before we ignore the object
        :params max_object_angle: the angle limit before we ignore the object
        """
        # Loop through each obstacle and calculate their coords and add them to the array.
        for object in object_locations:
            # Check the object distance to see if it meets the requirements.
            if (object[1] > min_object_distance and object[1] < max_object_distance) and (
                object[0] > min_object_angle and object[0] < max_object_angle
            ):
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
                if len(self.obstacle_coords) > constants.AVOIDANCE_OBSTACLE_QUEUE_LENGTH:
                    self.obstacle_coords = self.obstacle_coords[: -constants.AVOIDANCE_OBSTACLE_QUEUE_LENGTH]

    def clear_obstacles(self):
        """
        Empty the obstacle queue.
        """
        self.obstacle_coords.clear()

    def get_obstacle_coords(self):
        """
        Returns the obstacle coords array.

        :returns coords: Returns a list of GPS locations currently stored in the object array.
        """
        # Create instance variables.
        coords = []

        # Convert each coord to GPS
        for object in self.obstacle_coords:
            coord = utm.to_latlon(*(object[0], object[1], self.utm_zone[0], self.utm_zone[1]))
            coords.append(coord)

        return coords

    def plan_astar_avoidance_route(
        self, max_route_size=10, near_object_threshold=2.0, start_gps=None, return_gps=False
    ):
        """
        Uses the given list of object angles and distances, converts those to GPS waypoints, and then uses the A* (astar)
        algorithm to find the shortest path around the obstacle to a given endpoint in front of the robot.
        This function uses heapq while finding the path as it should be more effecient than using lists.

        :params max_route_size: the max square area available for route planning.
        :params near_object_threshold: the minimum distance the rover can get from the objects along the path.
        :params start_gps: The start position to use for the path. Will use rover's current GPS position by defualt.
        :params return_gps: Whether or not to return the path in GPS coords or UTM. UTM by default.

        :returns path: A list of gps waypoints around the path that should be safe for traversal.
        """
        # Determine start position.
        if start_gps is None:
            # Get current gps position.
            current_gps_pos = (interfaces.nav_board.location()[0], interfaces.nav_board.location()[1])
        else:
            # Use given start position.
            current_gps_pos = start_gps
        # Convert the gps coords to UTM coords. These coords are in meters and they are easier to work with.
        current_utm_pos = utm.from_latlon(current_gps_pos[0], current_gps_pos[1])
        self.utm_zone = (current_utm_pos[2], current_utm_pos[3])

        ####################################################################
        # Create start and end node.
        ####################################################################
        self.start = Node(None, (current_utm_pos[0], current_utm_pos[1]))
        # Calculate gps coord fixed distance in front of the rover.
        # Find the gps coordinate of the end point.
        gps_data = core.waypoint_handler.get_waypoint()
        waypoint_goal, _, _ = gps_data.data()
        waypoint_goal = utm.from_latlon(waypoint_goal[0], waypoint_goal[1])
        self.end = Node(None, (waypoint_goal[0], waypoint_goal[1]))

        # Create open and closed list.
        open_list = []
        closed_list = []

        # Heapify open list and add our start node.
        heapq.heapify(open_list)
        heapq.heappush(open_list, self.start)

        # Define a stop condition.
        outer_iterations = 0
        max_interations = max_route_size * max_route_size // 2

        ####################################################################
        # Define movement search pattern. In this case check in a grid pattern
        # 0.5 meters away from current position.
        ####################################################################
        offset = constants.AVOIDANCE_PATH_NODE_INCREMENT
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
                logger.warning("Unable to solve path: too many iterations.")
                return return_path(current_node, self.utm_zone, return_gps)

            # Found the goal.
            if (
                fabs(current_node.position[0] - self.end.position[0]) <= constants.WAYPOINT_DISTANCE_THRESHOLD
                and fabs(current_node.position[1] - self.end.position[1]) <= constants.WAYPOINT_DISTANCE_THRESHOLD
            ):
                return return_path(current_node, self.utm_zone, return_gps)

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
                    fabs(child_node_pos[0] - self.start.position[0]) > max_route_size
                    or fabs(child_node_pos[1] - self.start.position[1]) > max_route_size
                ):
                    continue

                # Make sure we are not close to another object.
                coord_to_close = False
                for coord in self.obstacle_coords:
                    # Calculate the straight-line distance of the robot position from the obstacle.
                    robot_distance_from_obstacle = math.sqrt(
                        math.pow(current_utm_pos[0] - child_node_pos[0], 2)
                        + math.pow(current_utm_pos[1] - child_node_pos[1], 2)
                    )
                    # Calculate the straight-line distance of the new node from the obstacle.
                    node_distance_from_obstacle = math.sqrt(
                        math.pow(coord[0] - child_node_pos[0], 2) + math.pow(coord[1] - child_node_pos[1], 2)
                    )
                    # Check if we are getting closer to the obstacle.
                    if node_distance_from_obstacle <= near_object_threshold:
                        coord_to_close = True
                    # # Check if the robot is within circle radius of obstacle and pick the point that will move us away from it.
                    # if (
                    #     robot_distance_from_obstacle < near_object_threshold
                    #     and node_distance_from_obstacle > robot_distance_from_obstacle
                    # ):
                    #     coord_to_close = False
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
                child.h = ((child.position[0] - self.end.position[0]) ** 2) + (
                    (child.position[1] - self.end.position[1]) ** 2
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
        logger.warning("Couldn't find path around obstacle to destination.")
        return None

    def calculate_yaws_from_path(self, cx, cy, start_angle=0.0, radians=True):
        """
        Computes the appropiate absolute yaw from a given set of Xs and Ys. These should
        produce a sensible path and the two lists should be the same length.
        All yaw angle are calulated in radians by default.

        :param cx: The list of x points within the path.
        :param cy: The list of y points within the path.
        :param start_angle: The initial angle for the first point.
        :param radians: Whether of not to use radians for the angle. (On by default)
        :return: ([yaws]) An array containing the yaw angles for each point.
        """
        # Create instance variables.
        yaws = []

        # Check if both lists are equal size.
        if len(cx) == len(cy) and len(cx) > 0:
            # Loop through points. The zip function returns iterables for a sublist starting at 0:-1 and a sublist starting at 1:end for each list x and y.
            for i, x, y, x_next, y_next in zip(range(len(cx) - 1), cx[:-1], cy[:-1], cx[1:], cy[1:]):
                # Basic trig to find angle between two points.
                angle = math.atan2((y_next - y), (x_next - x))

                # Check if we are converting to degrees.
                if not radians:
                    angle = np.rad2deg(angle)

                # Append angle to yaws list.
                yaws.append(angle)

            # Copy second to last angle to last point since the last point doesn't have a point after it to find angle from.
            yaws.append(yaws[-1])

        return yaws
