import asyncio
from turtle import distance
from algorithms import AR_tag
import algorithms.geomath as geomath
import algorithms.small_movements as small_movements
import algorithms.obstacle_avoider as obs_avoid
from core.vision.ar_tag_detector import is_gate
import core
import interfaces
import algorithms
from core.states import RoverState
import time
import logging
import math
from core.vision.ar_tag_detector import clear_tags
import numpy as np
import core.constants as constants
import geopy.distance
import geopy

# NO GPS VERSION
# drives in a straight line through the gate (this will be bad at steep angles)

# Create logger object.
logger = logging.getLogger(__name__)

class ApproachingGate(RoverState):
    """
    Within approaching gate, 3 waypoints are calculated in front of, between, and throught the viewed gate, allowing the rover to traverse through the gate fully.
    """

    def start(self):
        # Schedule AR Tag detection
        self.num_detection_attempts = 0
        self.gate_detection_attempts = 0
        self.last_angle = 1000
        self.not_seen = 0
        self.is_first = True
        self.is_turning = False
        self.og_angle = 0

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

        # Use get_tags to create an array of the 2 gate posts
        # (named tuples containing the distance and relative angle from the camera)
        tags = core.vision.ar_tag_detector.get_tags()
        gps_data = core.waypoint_handler.get_waypoint()
        orig_goal, orig_start, leg_type = gps_data.data()

        self.logger.info("Gate detected, beginning navigation")

        start = core.Coordinate(interfaces.nav_board.location()[0], interfaces.nav_board.location()[1])

        # If we've seen at least 5 frames of 2 tags, assume it's a gate
        self.logger.info("Gate detected, beginning navigation")

        if not self.is_turning:
            distance = (tags[0].distance + tags[1].distance) / 2
            angle = ((tags[0].angle) + (tags[1].angle)) / 2
            if abs(tags[0].angle) + abs(tags[1].angle) > constants.AR_SKEW_THRESHOLD:
                self.is_first = False
                self.is_turning = True

        if self.is_first:
            
            # Get
            post_1_coord = [tags[0].distance, tags[0].angle]
            post_2_coord = [tags[1].distance, tags[1].angle]

            print("Current GPS coords", start[0], start[1])
            print("POST 1 COORD:", post_1_coord)
            print("POST 2 COORD:", post_2_coord)

            try:
                targetBeforeGate, midpoint, targetPastGate = find_gate_path(
                    post_1_coord, post_2_coord, (start[0], start[1]), interfaces.nav_board.heading()
                )
            except:
                return self

            print("TB4GATE:", targetBeforeGate.latitude, targetBeforeGate.longitude)
            print("MIDPOINT:", midpoint.latitude, midpoint)
            print("PAST GATE:", targetPastGate.latitude, targetPastGate.longitude)

            point = targetBeforeGate.latitude, targetBeforeGate.longitude
            print(point)



            while (
                algorithms.gps_navigate.get_approach_status(
                    core.Coordinate(point[0], point[1]), interfaces.nav_board.location(), start, 0.5
                )
                == core.ApproachState.APPROACHING
            ):
                print("FIRST GPS POINT")

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
            self.is_first = False
            self.is_turning = True
            self.last_angle = 1000
            self.not_seen = 0
            self.og_angle = ((tags[0].angle) + (tags[1].angle)) / 2
            core.vision.ar_tag_detector.clear_tags()
            return self

        if self.is_turning:
            print("TURNING")

            if self.og_angle < 0:
                interfaces.drive_board.send_drive(-150, 150)
            else:
                interfaces.drive_board.send_drive(150, -150)
            
            if core.vision.ar_tag_detector.is_gate():
                self.is_turning = False
                interfaces.drive_board.stop()
            else:
                return self

        # Calculate angle and distance of center point between ar tags.
        distance = (tags[0].distance + tags[1].distance) / 2
        angle = ((tags[0].angle) + (tags[1].angle)) / 2

        if angle == self.last_angle:
            self.not_seen += 1
            if self.not_seen > 10:
                # t1 = time.time()
                # t2 = time.time()
                # # drive past gate for 10 seconds
                # while t2 - t1 < 3:
                #     t2 = time.time()
                #     interfaces.drive_board.send_drive(150, 150)
                # interfaces.drive_board.stop()
                if not(0 < distance < 5) or np.isnan(distance):
                    distance = 3 
                interfaces.drive_board.time_drive(distance + 2)
                self.logger.info("Reached Marker")

                # Transmit that we have reached the marker
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

                # Clear ar tag list!?!?!?!?
                core.vision.ar_tag_detector.clear_tags()

                return self.on_event(core.AutonomyEvents.REACHED_MARKER)
        else:
            self.not_seen = 0

        self.last_angle = angle

        left, right = algorithms.follow_marker.drive_to_marker(300, angle)

        if self.not_seen == 0:
            if angle < -0.5:
                right *= 1.2
                # left *= 0.8
            elif angle > 0.5:
                left *= 1.2
                # right *= 0.8

        right = int(right)
        left = int(left)

        self.logger.info("Marker in frame")
        self.num_detection_attempts = 0

        # if distance < 1.25:
        #     interfaces.drive_board.stop()

        #     self.logger.info("Reached Marker")

        #     # Transmit that we have reached the marker
        #     core.rovecomm_node.write(
        #         core.RoveCommPacket(
        #             core.manifest["Autonomy"]["Telemetry"]["ReachedMarker"]["dataId"],
        #             "B",
        #             (1,),
        #         ),
        #         False,
        #     )

        #     # Tell multimedia board to flash our LED matrix green to indicate reached marker
        #     interfaces.multimedia_board.send_lighting_state(core.OperationState.REACHED_MARKER)
        #     return self.on_event(core.AutonomyEvents.REACHED_MARKER)
        # else:
        self.logger.info(f"Driving to target with speeds: ({left}, {right})")
        interfaces.drive_board.send_drive(left, right)

        return self


def find_gate_path(polar_p1, polar_p2, current_gps_pos, current_heading):

    # This function finds two points that the rover can use to pass through a gate.
    # It does this by finding a line perpinduclar to the gate that passes through the midpoint.
    # It then finds the points on that line that are exactly "distance" away from the midpoint.
    #
    # Parameters:
    #       polar_p1, polar_p2:
    #               Polar coordinates of the points of the gate posts relative to the rover's heading and position.
    #               (distance, angle_from_heading) -- (meters) -- angle must be passed as RADIANS.
    #
    # Returns:
    #   two core.Coordinate objects with the GPS coordinates of each point that the rover needs to
    #   drive through the gate.
    #
    #   The first object is the point closest to the rover.

    # Distance each point will be from the gate (meters)
    distance = 3

    # Convert Polar coordinate input to rectangular coordinates
    p1 = (polar_p1[0] * math.cos(math.radians(polar_p1[1])), polar_p1[0] * math.sin(math.radians(polar_p1[1])))
    p2 = (polar_p2[0] * math.cos(math.radians(polar_p2[1])), polar_p2[0] * math.sin(math.radians(polar_p2[1])))

    # Compute the midpoint of point_1 and point_2
    midpoint = ((p2[0] + p1[0]) / 2, (p2[1] + p1[1]) / 2)

    # Compute the slope and the y_intercept of the line that passes through the gate
    slope = -(p2[0] - p1[0]) / (p2[1] - p1[1])
    y_intercept = midpoint[1] + ((p2[0] - p1[0]) * (p2[0] + p1[0])) / (2 * (p2[1] - p1[1]))

    # x points for where exactly distance from the midpoint -- NEED TO EXPLAIN THIS A LITTLE BETTER
    pos_c = distance / (math.sqrt(1 + ((p2[0] - p1[0]) ** 2 / (p2[1] - p1[1]) ** 2))) + midpoint[0]
    neg_c = -distance / (math.sqrt(1 + ((p2[0] - p1[0]) ** 2 / (p2[1] - p1[1]) ** 2))) + midpoint[0]

    # xy points that the rover will use to drive through the gate
    ret_point_1 = (pos_c, pos_c * slope + y_intercept)
    ret_point_2 = (neg_c, neg_c * slope + y_intercept)

    # Distance to those points
    p1_distance = math.sqrt(ret_point_1[0] ** 2 + ret_point_1[1] ** 2)
    p2_distance = math.sqrt(ret_point_2[0] ** 2 + ret_point_2[1] ** 2)
    midpoint_distance = math.sqrt(midpoint[0] ** 2 + midpoint[1] ** 2)

    # Compute angle to those points
    if ret_point_1[0] < 0:
        p1_angle = math.pi - math.asin(ret_point_1[1] / p1_distance)
    else:
        p1_angle = math.asin(ret_point_1[1] / p1_distance)

    if ret_point_2[0] < 0:
        p2_angle = math.pi - math.asin(ret_point_2[1] / p2_distance)
    else:
        p2_angle = math.asin(ret_point_2[1] / p2_distance)

    # Compute angle of midpoint
    if midpoint[0] < 0:
        mid_angle = math.pi - math.asin(midpoint[1] / midpoint_distance)
    else:
        mid_angle = math.asin(midpoint[1] / midpoint_distance)

    gps_pos = geopy.Point(current_gps_pos[0], current_gps_pos[1])
    dest_point_1 = geopy.distance.distance(meters=p1_distance).destination(
        gps_pos, bearing=(current_heading + math.degrees(p1_angle)) % 360
    )
    dest_point_2 = geopy.distance.distance(meters=p2_distance).destination(
        gps_pos, bearing=(current_heading + math.degrees(p2_angle)) % 360
    )
    dest_point_mid = geopy.distance.distance(meters=midpoint_distance).destination(
        gps_pos, bearing=(current_heading + math.degrees(mid_angle)) % 360
    )

    print("----------")
    print(dest_point_1, dest_point_mid, dest_point_2)

    if p2_distance > p1_distance:
        return dest_point_1, dest_point_mid, dest_point_2
    else:
        return dest_point_2, dest_point_mid, dest_point_1
