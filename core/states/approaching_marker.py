import asyncio
from core.vision.ar_tag_detector import is_marker
import core
import interfaces
import algorithms
from core.states import RoverState
import time
import math
from core.states import new_search_pattern


class ApproachingMarker(RoverState):
    """
    Within approaching marker, the rover explicitly follows the spotted marker until it reaches an acceptable distance from the rover, or loses sight of it.
    """

    def start(self):
        # Schedule AR Tag detection
        self.num_detection_attempts = 0
        self.gate_detection_attempts = 0
        self.last_angle = 1000
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
        if core.vision.ar_tag_detector.is_marker():
            # Grab the AR tags
            tags = core.vision.ar_tag_detector.get_tags()
            gps_data = core.waypoint_handler.get_waypoint()
            orig_goal, orig_start, leg_type = gps_data.data()

            # Currently only orienting based on one AR Tag
            distance = tags[0].distance
            angle = tags[0].angle

            # check to see if weve lost sight of tag
            if self.last_angle == angle:
                self.not_seen += 1
            else:
                self.not_seen = 0

            out_of_frame = False
            if self.not_seen > 10:
                out_of_frame = True

            self.last_angle = angle

            # calculate drive
            left, right = algorithms.follow_marker.drive_to_marker(300, angle)

            self.logger.info("Marker in frame")
            self.num_detection_attempts = 0

            print(f"DISTANCE: {distance}")
            if distance < 1.25 or out_of_frame:
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

                # Clear ar tag list!?!?!?!?
                core.vision.ar_tag_detector.clear_tags()

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
