import asyncio
from core.vision.ar_tag_detector import is_gate
import core
import interfaces
import algorithms
from core.states import RoverState
import time
import math


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
        if core.vision.ar_tag_detector.is_gate():
            # Use get_tags to create an array of the 2 gate posts 
            # (named tuples containing the distance and relative angle from the camera)
            tags = core.vision.ar_tag_detector.get_tags()
            gps_data = core.waypoint_handler.get_waypoint()
            orig_goal, orig_start, leg_type = gps_data.data()

            # If we've seen at least 5 frames of 2 tags, assume it's a gate
            if len(tags) == 2 and self.gate_detection_attempts >= 5 and leg_type == "GATE":
                self.logger.info("Gate detected, beginning navigation")
                # compute the angle across from the gate
                # depending where the rover is facing, this is computed differently
                if tags[0].angle < 0 and tags[1].angle < 0:  # both tags on the right
                    larger = min(tags[0].angle, tags[1].angle)
                    smaller = max(tags[0].angle, tags[1].angle)
                    combinedAngle = abs(larger) - abs(smaller)
                elif tags[0].angle >= 0 and tags[1].angle >= 0:  # both tags on the left
                    larger = max(tags[0].angle, tags[1].angle)
                    smaller = min(tags[0].angle, tags[1].angle)
                    combinedAngle = larger - smaller
                else:  # one tag on left, one on right
                    combinedAngle = abs(tags[0].angle) + abs(tags[1].angle)
                # Calculate bearing and distance for the midpoint between the two tags
                # use law of cosines to get the distance between the tags, midpoint will be halfway between them
                gateWidth = math.sqrt(
                    (tags[0].distance ** 2)
                    + (tags[1].distance ** 2)
                    - 2 * tags[0].distance * tags[1].distance * math.cos(math.radians(combinedAngle))
                )
                self.logger.info(f"Gate width {gateWidth}")

                # we want to use the smaller side for our midpoint triangle
                D1 = min(tags[0].distance, tags[1].distance)

                # use law of sines to get the angle across from D1
                sinVal = (math.sin(math.radians(combinedAngle / 2)) * D1) / (gateWidth * 0.5)

                # arcsin is limited between -1 and 1, not sure why reflecting these values
                # across the y axis works, but it does. No proof it works in all cases
                if sinVal > 1:
                    sinDiff = sinVal - 1
                    sinVal = 1 - sinDiff
                elif sinVal < -1:
                    sinDiff = sinVal + 1
                    sinVal = -1 - sinDiff
                angleAcrossD1 = math.asin(sinVal)

                # deduce the last angle from 180 (pi)
                angleAcrossDm = math.pi - angleAcrossD1 - math.radians(combinedAngle / 2)

                # law of sines to get the last side of our triangle
                distToMidpoint = abs(
                    ((gateWidth / 2) * math.sin(angleAcrossDm)) / math.sin(math.radians(combinedAngle / 2))
                )
                self.logger.info(f"Calculated Distance to gate: {distToMidpoint}")

                # Last step to get angle to the midpoint, depending on where tags are relative to rover
                if tags[0].angle < 0 and tags[1].angle < 0:
                    angleToMidpoint = (interfaces.nav_board.heading() - (abs(larger) - (combinedAngle / 2))) % 360
                elif tags[0].angle >= 0 and tags[1].angle >= 0:
                    angleToMidpoint = (interfaces.nav_board.heading() + (abs(larger) - (combinedAngle / 2))) % 360
                else:
                    angleToMidpoint = (interfaces.nav_board.heading() + ((tags[0].angle + tags[1].angle) / 2)) % 360

                self.logger.info(f"Calculated Angle to gate: {angleToMidpoint}")

                start = core.Coordinate(interfaces.nav_board.location()[0], interfaces.nav_board.location()[1])

                # Get a GPS coordinate using our distance and bearing
                target = algorithms.obstacle_avoider.coords_obstacle(
                    distToMidpoint, start[0], start[1], angleToMidpoint
                )

                # Also calculate second point (to run through the gate)
                targetPastGateHeading = ((angleAcrossD1 - (math.pi / 2)) + interfaces.nav_board.heading()) % 360
                targetBeforeGate = algorithms.obstacle_avoider.coords_obstacle(
                    -3, target[0], target[1], targetPastGateHeading
                )
                targetPastGate = algorithms.obstacle_avoider.coords_obstacle(
                    3, target[0], target[1], targetPastGateHeading
                )

                print(f'Before {targetBeforeGate}\nDuring {target}\nAfter {targetPastGate}')
                points = [targetBeforeGate, target, targetPastGate]

                # Approach the gate using GPS drive
                for point in points:
                    while (
                        algorithms.gps_navigate.get_approach_status(
                            core.Coordinate(point[0], point[1]), interfaces.nav_board.location(), start, 1
                        )
                        == core.ApproachState.APPROACHING
                    ):
                        #print("AS: ", algorithms.gps_navigate.get_approach_status( core.Coordinate(point[0], point[1]), interfaces.nav_board.location(), start, 1))
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
