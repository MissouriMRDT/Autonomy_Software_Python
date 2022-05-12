import asyncio
from turtle import distance
import algorithms.geomath as geomath
import algorithms.obstacle_avoider as obs_avoid
from core.vision.ar_tag_detector import is_gate
import core
import interfaces
import algorithms
from core.states import RoverState
import time
import math
from core.vision.ar_tag_detector import clear_tags
import numpy as np

# NO GPS VERSION
# drives in a straight line through the gate (this will be bad at steep angles)


class ApproachingGate(RoverState):
    """
    Within approaching gate, 3 waypoints are calculated in front of, between, and throught the viewed gate, allowing the rover to traverse through the gate fully.
    """

    def start(self):
        # Schedule AR Tag detection
        self.num_detection_attempts = 0
        self.gate_detection_attempts = 0
        self.last_distance = 100
        self.not_seen = 0

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
        if core.vision.ar_tag_detector.is_gate():
            # Use get_tags to create an array of the 2 gate posts
            # (named tuples containing the distance and relative angle from the camera)
            tags = core.vision.ar_tag_detector.get_tags()
            gps_data = core.waypoint_handler.get_waypoint()
            orig_goal, orig_start, leg_type = gps_data.data()

            self.logger.info("Gate detected, beginning navigation")

            start = core.Coordinate(interfaces.nav_board.location()[0], interfaces.nav_board.location()[1])
            post_1_coord = [tags[0].distance, tags[0].angle]
            post_2_coord = [tags[1].distance, tags[1].angle]
            targetBeforeGate, midpoint, targetPastGate = find_gate_path(
                    post_1_coord, post_2_coord, (start[0], start[1]), interfaces.nav_board.heading()
                )

            point = targetBeforeGate
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

            # Get
            distance = (tags[0].distance + tags[1].distance) / 2
            angle = ((tags[0].angle) + (tags[1].angle)) / 2

            print("---------", distance)
            print(">>>>>>>>>>>>>>", self.last_distance)
            if distance == self.last_distance or np.isnan(tags[0].distance) or np.isnan(tags[1].distance):
                self.not_seen += 1
                if self.not_seen > 10:
                    # t1 = time.time()
                    # t2 = time.time()
                    # # drive past gate for 10 seconds
                    # while t2 - t1 < 3:
                    #     t2 = time.time()
                    #     interfaces.drive_board.send_drive(150, 150)
                    # interfaces.drive_board.stop()
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
                    return self.on_event(core.AutonomyEvents.REACHED_MARKER)
            else:
                print("JJJJJJJJJJJJJJJ")
                self.not_seen = 0

            self.last_distance = distance

            left, right = algorithms.follow_marker.drive_to_marker(300, angle)
            # left = right = 150

            # angle = interfaces.nav_board.heading() + angle
            # if angle < 0:
            #     right = right * (1 - (angle / 180.0))
            # elif angle > 0:
            #     left = left * (1 + (angle / 180.0))
            # print("++++++", angle)
            # right = int(right)
            # left = int(left)

            print("??????", self.not_seen)
            if self.not_seen == 0:
                print("hi")
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

            print(f"DISTANCE: {distance}")
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
        else:
            self.num_detection_attempts += 1
            self.gate_detection_attempts = 0

            # If we have attempted to track an AR Tag unsuccesfully
            # MAX_DETECTION_ATTEMPTS times, we will return to search pattern
            if self.num_detection_attempts >= core.MAX_DETECTION_ATTEMPTS:
                self.logger.info("Lost sign of gate, returning to Search Pattern")
                return self.on_event(core.AutonomyEvents.MARKER_UNSEEN)

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
    #   The first object is the point closest dfgdfhffto the rover.

    # Distance each point will be from the gate (meters)
    distance = 6

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

    if p2_distance > p1_distance:
        return dest_point_1, dest_point_mid, dest_point_2
    else:
        return dest_point_2, dest_point_mid, dest_point_1
