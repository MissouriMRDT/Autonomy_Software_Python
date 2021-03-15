import asyncio
from core.vision.ar_tag_detector import is_ar_tag
import core
import interfaces
import algorithms
from core.states import RoverState
import time


class ApproachingMarker(RoverState):
    """
    Within approaching marker, the rover explicitly follows the spotted marker until it reaches an acceptable distance from the rover, or loses sight of it.
    """

    def start(self):
        # Schedule AR Tag detection
        loop = asyncio.get_event_loop()
        self.ar_tag_task = loop.create_task(core.vision.ar_tag_detector.async_ar_tag_detector())
        self.num_detection_attempts = 0

    def exit(self):
        # Cancel all state specific coroutines
        self.ar_tag_task.cancel()
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
        if core.vision.ar_tag_detector.is_ar_tag():
            # Grab the AR tags
            tags = core.vision.ar_tag_detector.get_tags()

            # Currently only orienting based on one AR Tag-
            distance = tags[0].distance
            angle = tags[0].angle

            left, right = algorithms.follow_marker.drive_to_marker(125, distance, angle)

            self.logger.info("Marker in frame")
            self.num_detection_attempts = 0

            if distance < 1.25:
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

            # If we have attempted to track an AR Tag unsuccesfully
            # MAX_DETECTION_ATTEMPTS times, we will return to search pattern
            if self.num_detection_attempts >= core.MAX_DETECTION_ATTEMPTS:
                self.logger.info("Lost sign of marker, returning to Search Pattern")
                return self.on_event(core.AutonomyEvents.MARKER_UNSEEN)

        return self
