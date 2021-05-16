import asyncio
from core.vision.ar_tag_detector import is_ar_tag
import core
import interfaces
import algorithms
from core.states import RoverState
import time
import math


class ApproachingMarker(RoverState):
    """
    Within approaching marker, the rover explicitly follows the spotted marker until it reaches an acceptable distance from the rover, or loses sight of it.
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

    async def run(self) -> RoverState:

        # Call AR Tag tracking code to find position and size of AR Tag
        if core.vision.ar_tag_detector.is_ar_tag():
            # Grab the AR tags
            tags = core.vision.ar_tag_detector.get_tags()

            # If we've seen at least 5 frames of 2 tags, assume it's a gate
            if len(tags) == 2 and self.gate_detection_attempts >= 5:
                # Calculate bearing and distance for the midpoint between the two tags
                # use law of cosines to get the distance between the tags, midpoint will be halfway between them
                gateWidth = math.sqrt(
                    (tags[0].distance ** 2)
                    + (tags[1].distance ** 2)
                    - 2 * tags[0].distance * tags[1].distance * math.cos(tags[0].angle + tags[1].angle)
                )

                # use law of sines to get the angle across from tag[0]'s distance and deduce the last angle
                angleAcrossD1 = math.asin((math.sin(tags[0].angle) * tags[0].distance) / (gateWidth * 0.5))
                angleAcrossDm = 180 - angleAcrossD1 - tags[0].angle
                distToMidpoint = (tags[0].distance * math.sin(angleAcrossDm)) / math.sin(tags[0].angle)
                angleToMidpoint = (((tags[0].angle + tags[1].angle) / 2) + interfaces.nav_board.heading) % 360

                start = core.Coordinate(interfaces.nav_board.location()[0], interfaces.nav_board.location()[1])

                # Get a GPS coordinate using our distance and bearing
                target = algorithms.obstacle_avoider.coords_obstacle(
                    distToMidpoint, start[0], start[1], angleToMidpoint
                )

                # Also calculate second point (to run through the gate)
                # rightTriSide = math.sin(angleToMidpoint) * tags[0].distance
                # complementAngle = math.asin(rightTriSide / (gateWidth*.5))
                targetPastGateHeading = ((angleAcrossD1 - 90.0) + interfaces.nav_board.heading) % 360
                targetPastGate = algorithms.obstacle_avoider.coords_obstacle(
                    2, target[0], target[1], targetPastGateHeading
                )

                points = [target, targetPastGate]

                # Approach the gate using GPS drive
                for point in points:
                    while (
                        algorithms.gps_navigate.get_approach_status(
                            core.Coordinate(point[0], point[1]), interfaces.nav_board.location(), start, 0.5
                        )
                        == core.ApproachState.APPROACHING
                    ):
                        left, right = algorithms.gps_navigate.calculate_move(
                            core.Coordinate(point[0], point[1]),
                            interfaces.nav_board.location(),
                            start,
                            250,
                        )

                        # self.logger.debug(f"Navigating: Driving at ({left}, {right})")
                        interfaces.drive_board.send_drive(left, right)
                        time.sleep(0.01)
                    interfaces.drive_board.stop()
            # If we grabbed more than one, see if it's a gate
            elif len(tags) > 1:
                self.gate_detection_attempts += 1
            elif len(tags) == 1:
                self.gate_detection_attempts = 0

            # Currently only orienting based on one AR Tag-
            distToMidpoint = tags[0].distance
            angleToMidpoint = tags[0].angle

            left, right = algorithms.follow_marker.drive_to_marker(100, angleToMidpoint)

            self.logger.info("Marker in frame")
            self.num_detection_attempts = 0

            if distToMidpoint < 1.25:
                interfaces.drive_board.stop()

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
                self.logger.info(f"Driving to target with speeds: ({left}, {right})")
                interfaces.drive_board.send_drive(left, right)
        else:
            self.num_detection_attempts += 1
            self.gate_detection_attempts = 0

            # If we have attempted to track an AR Tag unsuccesfully
            # MAX_DETECTION_ATTEMPTS times, we will return to search pattern
            if self.num_detection_attempts >= core.MAX_DETECTION_ATTEMPTS:
                self.logger.info("Lost sign of marker, returning to Search Pattern")
                return self.on_event(core.AutonomyEvents.MARKER_UNSEEN)

        return self
