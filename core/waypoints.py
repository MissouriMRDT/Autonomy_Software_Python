#
# Mars Rover Design Team
# waypoints.py
#
# Created on Oct 23, 2020
# Updated on Aug 21, 2022
#

import logging
import core
from collections import deque
import interfaces


class GPSData:
    def __int__(self, goal, start, leg="POSITION"):
        self.goal = goal
        self.start = start
        self.leg_type = leg

    def data(self):
        return self.goal, self.start, self.leg_type


class WaypointHandler:
    def __init__(self):
        # Class variables
        self.waypoints: deque = deque()
        self.gps_data: GPSData = None

        core.rovecomm_node.set_callback(
            core.manifest["Autonomy"]["Commands"]["AddPositionLeg"]["dataId"], self.add_position_waypoint
        )
        core.rovecomm_node.set_callback(
            core.manifest["Autonomy"]["Commands"]["AddMarkerLeg"]["dataId"], self.add_marker_waypoint
        )
        core.rovecomm_node.set_callback(
            core.manifest["Autonomy"]["Commands"]["AddGateLeg"]["dataId"], self.add_gate_waypoint
        )
        core.rovecomm_node.set_callback(
            core.manifest["Autonomy"]["Commands"]["ClearWaypoints"]["dataId"], self.clear_waypoints
        )

        self.logger = logging.getLogger(__name__)

    def add_marker_waypoint(self, packet) -> None:
        """
        Adds the data from the packet (expects lat, lon) to the waypoints deque
        as a marker

        :param packet:
        :return: None
        """

        latitude, longitude = packet.data
        waypoint = core.Coordinate(latitude, longitude)
        self.waypoints.append(("MARKER", waypoint))
        self.logger.info(f"Added Marker Waypoint: lat ({latitude}), lon({longitude})")

    def add_gate_waypoint(self, packet) -> None:
        """
        Adds the data from the packet (expects lat, lon) to the waypoints deque
        as a gate

        :param packet:
        :return: None
        """

        latitude, longitude = packet.data
        waypoint = core.Coordinate(latitude, longitude)
        self.waypoints.append(("GATE", waypoint))
        self.logger.info(f"Added Gate Waypoint: lat ({latitude}), lon({longitude})")

    def add_position_waypoint(self, packet) -> None:
        """
        Adds the data from the packet (expects lat, lon) to the waypoints deque
        as a gate

        :param packet:
        :return: None
        """

        latitude, longitude = packet.data
        waypoint = core.Coordinate(latitude, longitude)
        self.waypoints.append(("POSITION", waypoint))
        self.logger.info(f"Added Position Waypoint: lat ({latitude}), lon({longitude})")

    def clear_waypoints(self, packet="") -> None:
        """
        Clears the deque of waypoints

        :param packet:
        :return: None
        """
        self.waypoints.clear()
        self.gps_data: GPSData = None
        self.logger.info("Cleared all waypoints")

    def get_waypoint(self) -> GPSData:
        """
        Gets the current waypoint, pops a new from the deque if we haven't grabbed a
        waypoint from the deque yet

        :return: GPSData
        """

        # Pop off a waypoint from the queue if there is currently none
        if self.gps_data is None:
            return self.get_new_waypoint()

        return self.gps_data

    def set_goal(self, goal):
        """
        Sets the goal of the current waypoint

        :param goal:
        :return: None
        """

        # Set the goal to the passed in goal
        self.gps_data.goal = goal

    def set_start(self, start):
        """
        Sets the starting location of the current waypoint

        :param start:
        :return: None
        """

        self.gps_data.start = start

    def is_empty(self) -> bool:
        """
        Returns true if there are more waypoints in the deque

        :return: If waypoints is empty
        """

        if len(self.waypoints) > 0:
            return False
        else:
            return True

    def get_new_waypoint(self) -> GPSData:
        """
        Grabs a new waypoint from the queue, goal being the data in the deque and start
        being the current perceived location of the rover

        :return: GPSData
        """

        self.gps_data = GPSData()
        self.gps_data.start = interfaces.nav_board.location()

        try:
            leg_type, current_goal = self.waypoints.popleft()
        except IndexError:
            self.logger.error("Tried popping waypoint from empty deque")
            self.gps_data = None
            return None

        self.gps_data.goal = current_goal
        self.gps_data.leg_type = leg_type

        self.logger.info(f"Set Waypoint Target: lat ({current_goal.lat}), lon({current_goal.lon})")
        return self.gps_data
