import asyncio
from algorithms import obstacle_avoider
import algorithms.geomath as geomath
from core.vision.ar_tag_detector import is_gate
import core
import interfaces
import algorithms
from core.states import RoverState
import time
import math
from typing import Tuple
from dataclasses import dataclass

class ApproachingGate(RoverState):
    """
    Within approaching gate, 3 waypoints are calculated in front of, between, and throught the viewed gate, allowing the rover to traverse through the gate fully.
    """

    def start(self):
        # Schedule AR Tag detection
        self.num_detection_attempts = 0
        self.gate_detection_attempts = 0

    def exit(self):
        # Cancel all state specific coroutines
        pass

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events
        """
        state: RoverState = None

        if event == core.AutonomyEvents.REACHED_MARKER:
            state = core.states.Idle()

        elif event == core.AutonomyEvents.START:
            state = self

        elif event == core.AutonomyEvents.MARKER_UNSEEN:
            state = core.states.SearchPattern()

        elif event == core.AutonomyEvents.ABORT:
            state = core.states.Idle()

        else:
            self.logger.error(f"Unexpected event {event} for state {self}")
            state = core.states.Idle()

        # Call exit() if we are not staying the same state
        if state != self:
            self.exit()

        # Return the state appropriate for the event
        return state

    async def run(self) -> RoverState:

        # Call AR Tag tracking code to find position and size of AR Tag
        if not core.vision.ar_tag_detector.is_gate():
            self.num_detection_attempts += 1
            self.gate_detection_attempts = 0

            # If we have attempted to track an AR Tag unsuccesfully
            # MAX_DETECTION_ATTEMPTS times, we will return to search pattern
            if self.num_detection_attempts >= core.MAX_DETECTION_ATTEMPTS:
                self.logger.info("Lost sign of gate, returning to Search Pattern")
                return self.on_event(core.AutonomyEvents.MARKER_UNSEEN)

            return self

        # Use get_tags to create an array of the 2 gate posts
        # (named tuples containing the distance and relative angle from the camera)
        tags = core.vision.ar_tag_detector.get_tags()
        gps_data = core.waypoint_handler.get_waypoint()
        orig_goal, orig_start, leg_type = gps_data.data()

        # If we've seen at least 5 frames of 2 tags, assume it's a gate
        found_gate: bool = len(tags) == 2 and self.gate_detection_attempts >= 5

        if not found_gate:
            # If we grabbed more than one, see if it's a gate
            if len(tags) > 1:
                self.gate_detection_attempts += 1
                self.logger.info(f"2 Markers in frame, count:{self.gate_detection_attempts}")

            return self

        self.logger.info("Gate detected, beginning navigation")

        # Get
        post_1_coord = (tags[0].distance, math.radians(tags[0].angle))
        post_2_coord = (tags[1].distance, math.radians(tags[1].angle))

        targetBeforeGate, midpoint, targetPastGate = find_gate_path(post_1_coord, post_2_coord)

        start = core.Coordinate(interfaces.nav_board.location()[0], interfaces.nav_board.location()[1])

        points = [targetBeforeGate, midpoint, targetPastGate]

        # Approach the gate using GPS drive
        for point in points:
            while (
                    algorithms.gps_navigate.get_approach_status(
                        core.Coordinate(point[0], point[1]), interfaces.nav_board.location(), start, 0.5
                    )
                    == core.ApproachState.APPROACHING
            ):
                self.logger.info(f"Driving towards: Lat: {point[0]}, Lon: {point[1]}")
                left, right = algorithms.gps_navigate.calculate_move(
                    core.Coordinate(point[0], point[1]),
                    interfaces.nav_board.location(),
                    start,
                    250,
                )

                self.logger.debug(f"Diving at speeds: Left: {left} Right: {right}")

                interfaces.drive_board.send_drive(left, right)
                time.sleep(0.01)
            interfaces.drive_board.stop()

        self.logger.info("Reached Gate")

        # Transmit that we have reached the gate
        core.rovecomm_node.write(
            core.RoveCommPacket(
                core.manifest["Autonomy"]["Telemetry"]["ReachedMarker"]["dataId"],
                "B",
                (1,),
            ),
            False,
        )

        # Tell multimedia board to flash our LED matrix green to indicate reached marker
        interfaces.multimedia_board.send_lighting_state(core.OperationState.REACHED_MARKER)

        return self.on_event(core.AutonomyEvents.REACHED_MARKER)


def find_gate_path(polar_p1: Tuple[float, float], polar_p2: Tuple[float, float], debug_math: bool = False) \
        -> Tuple[core.Coordinate, core.Coordinate, core.Coordinate]:
    # This function finds two points that the rover can use to pass through a gate.
    # It does this by finding a line perpendicular to the gate that passes through the midpoint.
    # It then finds the points on that line that are exactly "distance" away from the midpoint.
    #
    # Parameters:
    #       polar_p1, polar_p2:
    #               Polar coordinates of the points of the gate posts relative to the rover's heading and position.
    #               (distance, angle_from_heading) -- (meters) -- angle must be passed as RADIANS.
    #
    # Returns:
    #   Three core.Coordinate objects with the GPS coordinates of each point that the rover needs to
    #   drive through the gate.
    #
    #   The first object is the point closest to the rover.

    # Visualization: https://www.desmos.com/calculator/gqjnyr1lyn

    # Distance each point will be from the gate (meters)
    distance: float = 2.0

    @dataclass
    class Vector2:
        x: float
        y: float

        @classmethod
        def from_polar(cls, length: float, angle: float) -> 'Vector2':
            x: float = length * math.cos(angle)
            y: float = length * math.sin(angle)
            return cls(x, y)

        @property
        def length(self) -> float:
            return math.hypot(self.x, self.y)

        @property
        def angle(self) -> float:
            return math.atan2(self.y, self.x)

    # Convert Polar coordinate input to rectangular coordinates
    p1: Vector2 = Vector2.from_polar(*polar_p1)
    p2: Vector2 = Vector2.from_polar(*polar_p2)

    # Compute the midpoint of p1 and p2
    midpoint: Vector2 = Vector2((p1.x + p2.x) / 2, (p1.y + p2.y) / 2)

    # Compute the difference between p1 and p2
    diff: Vector2 = Vector2(p1.x - p2.x, p1.y - p2.y)

    # Get the vector perpendicular to diff
    perp_diff: Vector2 = Vector2(-diff.y, diff.x)
    # Adjust the length to be equal to distance
    distance_mult: float = distance / perp_diff.length
    perp_diff.x *= distance_mult
    perp_diff.y *= distance_mult

    # Get new points on the line between p1 and p2 distance from midpoint
    result_p1: Vector2 = Vector2(midpoint.x + perp_diff.x, midpoint.y + perp_diff.y)
    result_p2: Vector2 = Vector2(midpoint.x - perp_diff.x, midpoint.y - perp_diff.y)

    # Code to test if the math of this function is correct
    # Example (run in python3 command line):
    # from math import tau
    # from random import random
    # from core.states.approaching_gate import find_gate_path
    # find_gate_path((20.0 * random(), tau * random()), (20.0 * random(), tau * random()), debug_math=True)
    if debug_math:
        distances = [
            math.hypot(result_p1.x - p1.x, result_p1.y - p1.y),
            math.hypot(result_p1.x - p2.x, result_p1.y - p2.y),
            math.hypot(result_p2.x - p1.x, result_p2.y - p1.y),
            math.hypot(result_p2.x - p2.x, result_p2.y - p2.y),
        ]

        distance_range = max(distances) - min(distances)

        print("Distances between gate posts and gate path endpoints (should all be equal):")
        print(f"\t{' '.join(f'{distance:.8f}' for distance in distances)} Range: {distance_range}")

        path_length = math.hypot(result_p1.x - result_p2.x, result_p1.y - result_p2.y)

        print(f"Path length (should be {2 * distance}): {path_length}")
        print()

        return None

    coord1 = camera_point_to_gps_coord(
        result_p1.length, math.degrees(result_p1.angle), interfaces.nav_board.heading()
    )
    coord2 = camera_point_to_gps_coord(
        result_p2.length, math.degrees(result_p2.angle), interfaces.nav_board.heading()
    )
    mid_coord = camera_point_to_gps_coord(
        midpoint.length, math.degrees(midpoint.angle), interfaces.nav_board.heading()
    )

    # Ensure that coord1 is closer to the rover
    if result_p2.length < result_p1.length:
        coord1, coord2 = coord2, coord1

    return coord1, mid_coord, coord2


def camera_point_to_gps_coord(distance, angle, heading):
    bearing = heading + angle
    lat, long = obstacle_avoider.coords_obstacle(
        distance, interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], bearing
    )

    return core.Coordinate(lat, long)
