#
# Mars Rover Design Team
# approaching_gate.py
#
# Created on May 19, 2021
# Updated on Aug 21, 2022
#

import core
import core.constants
import interfaces
import algorithms
from core.states import RoverState
import time
import math


class ApproachingGate(RoverState):
    """
    Within approaching gate, 3 waypoints are calculated in front of, between, and through the viewed gate,
    allowing the rover to traverse through the gate fully.
    """

    def start(self):
        """
        Schedule AR Tag detection
        """

        self.num_detection_attempts = 0
        self.gate_detection_attempts = 0

    def exit(self):
        """
        Cancel all state specific coroutines
        """

        pass

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events

        :param event:
        :return: RoverState
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
        """
        Asynchronous state machine loop

        :return: RoverState
        """

        # Call AR Tag tracking code to find position and size of AR Tag
        if core.vision.ar_tag_detector.is_gate():
            # Use get_tags to create an array of the 2 gate posts
            # (named tuples containing the distance and relative angle from the camera)
            tags = core.vision.ar_tag_detector.get_tags()
            gps_data = core.waypoint_handler.get_waypoint()
            orig_goal, orig_start, leg_type = gps_data.data()

            # If we've seen at least 5 frames of 2 tags, assume it's a gate
            if len(tags) == 2 and self.gate_detection_attempts >= 5 and leg_type == "GATE":
                self.logger.info("Gate detected, beginning navigation")

                # Here the angle of tags are opposite of the direction of radians
                # E.G. A positive tags[i].angle means right while a positive radian means left
                # So we flip it
                polGateMark1 = (tags[0].distance, -tags[0].angle)
                polGateMark2 = (tags[1].distance, -tags[1].angle)
                heading = interfaces.nav_board.heading()

                # Auxillary Functions
                def cosDeg(degs):
                    return math.cos(math.radians(degs))

                def sinDeg(degs):
                    return math.sin(math.radians(degs))

                def avg(a, b):
                    return (a + b) / 2

                def polarToCartesian(p):
                    return ((p[0] * cosDeg(p[1])), (p[0] * sinDeg(p[1])))

                def cartesianToPolar(p):
                    return (math.sqrt(p[0]**2 + p[1]**2), math.degrees(math.atan(p[1] / p[0])))

                """"
                    Context for the following math:
                    Imagine the 2 gate markers and the three target points to form a cross whose intersection 
                        is the midpoint between the two gates.
                    The 2 Gate Markers and the gate's midpoint form the Gate Arm of the cross.
                    The 3 Targets which includes the gate's midpoint will form the Route Arm of the cross
                    Note that the Gate Arm and Route Arm will be perpendicular to each other.
                """
                cartGateMark1 = polarToCartesian(polGateMark1)
                cartGateMark2 = polarToCartesian(polGateMark2)
                polGateMidpoint = cartesianToPolar((avg(cartGateMark1[0], cartGateMark2[0]), avg(cartGateMark1[1], cartGateMark2[1])))

                self.logger.info(f"Calculated Distance to gate: {polGateMidpoint[0]}")
                self.logger.info(f"Calculated Angle to gate: {polGateMidpoint[1]}")

                # Calculate the angle of the route arm with respect to the rover's orientation
                # Rover Axis is the axis aligned with the rover's current orientation
                furthestMarkerOnRoverAxis = cartGateMark1 if cartGateMark1[0] >= cartGateMark2[0] else cartGateMark2
                closestMarkerOnRoverAxis = cartGateMark1 if cartGateMark1[0] < cartGateMark2[0] else cartGateMark2
                diffMarkerRoverAxis = furthestMarkerOnRoverAxis[0] - closestMarkerOnRoverAxis[0]
                diffMarkerPerpRoverAxis = closestMarkerOnRoverAxis[1] - furthestMarkerOnRoverAxis[1]
                angleOfRouteArm = math.degrees(math.atan(diffMarkerRoverAxis / diffMarkerPerpRoverAxis))

                headingOfMidpoint = (heading - polGateMidpoint[1]) % 360
                headingOfRouteArm = (heading - angleOfRouteArm) % 360

                startingPoint = core.Coordinate(interfaces.nav_board.location()[0], interfaces.nav_board.location()[1])

                # Coordinates of the route arm
                targetBetweenGate = algorithms.obstacle_avoider.coords_obstacle(polGateMidpoint[0], startingPoint[0], startingPoint[1], headingOfMidpoint)
                targetBeforeGate = algorithms.obstacle_avoider.coords_obstacle(-3, targetBetweenGate[0], targetBetweenGate[1], headingOfRouteArm)
                targetPastGate   = algorithms.obstacle_avoider.coords_obstacle(3, targetBetweenGate[0], targetBetweenGate[1], headingOfRouteArm)

                points = [targetBeforeGate, targetBetweenGate, targetPastGate]
                pointsLabels = ["targetBeforeGate", "targetBetweenGate", "targetPastGate"]
                self.logger.info(f"All points: {points}")

                print("High Point")

                # Approach the gate using GPS drive
                for point, label in zip(points, pointsLabels):
                    while (
                        algorithms.gps_navigate.get_approach_status(
                            core.Coordinate(point[0], point[1]),
                            interfaces.nav_board.location(),
                            startingPoint,
                            core.constants.WAYPOINT_DISTANCE_THRESHOLD,
                        )
                        == core.ApproachState.APPROACHING
                    ):
                        self.logger.info(f"Driving towards: Lat: {point[0]}, Lon: {point[1]} Point: {label}")
                        left, right = algorithms.gps_navigate.calculate_move(
                            core.Coordinate(point[0], point[1]),
                            interfaces.nav_board.location(),
                            startingPoint,
                            core.MAX_DRIVE_POWER,
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

            # If we grabbed more than one, see if it's a gate
            elif len(tags) > 1:
                self.gate_detection_attempts += 1
                self.logger.info(f"2 Markers in frame, count:{self.gate_detection_attempts}")

        else:
            self.num_detection_attempts += 1
            self.gate_detection_attempts = 0

            # If we have attempted to track an AR Tag unsuccesfully
            # MAX_DETECTION_ATTEMPTS times, we will return to search pattern
            if self.num_detection_attempts >= core.MAX_DETECTION_ATTEMPTS:
                self.logger.info("Lost sign of gate, returning to Search Pattern")
                return self.on_event(core.AutonomyEvents.MARKER_UNSEEN)

        return self
