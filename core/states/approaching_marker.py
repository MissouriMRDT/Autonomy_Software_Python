#
# Mars Rover Design Team
# approaching_marker.py
#
# Created on Dec 30, 2020
# Updated on Aug 21, 2022
#

import core
import interfaces
import algorithms
from core.states import RoverState


class ApproachingMarker(RoverState):
    """
    Within approaching marker, the rover explicitly follows the spotted marker until it reaches an acceptable distance from the rover, or loses sight of it.
    """

    def start(self):
        """
        Schedule AR Tag detection
        """
        self.num_detection_attempts = 0

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
        if core.vision.ar_tag_detector.is_marker():
            # Grab the AR tags
            tags = core.vision.ar_tag_detector.get_valid_tags()
            gps_data = core.waypoint_handler.get_waypoint()
            orig_goal, orig_start, leg_type = gps_data.data()

            # Currently only orienting based on one AR Tag
            distance = tags[0].distance
            angle = tags[0].angle
            self.logger.info(f"MARKER DISTANCE: {distance} ANGLE: {angle}")

            left, right = algorithms.follow_marker.drive_to_marker(core.constants.MARKER_MAX_APPROACH_SPEED, angle)

            self.logger.info("Marker in frame")
            self.num_detection_attempts = 0

            if distance < core.constants.ARUCO_MARKER_STOP_DISTANCE:
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
            # Increment counter.
            self.num_detection_attempts += 1
            # Stop drive.
            interfaces.drive_board.stop()

            # If we have attempted to track an AR Tag unsuccesfully
            # MAX_DETECTION_ATTEMPTS times, we will return to search pattern
            if self.num_detection_attempts >= core.MAX_DETECTION_ATTEMPTS:
                # Get current position and next desired waypoint position.
                current = interfaces.nav_board.location()
                # Set goal waypoint as current.
                core.waypoint_handler.set_goal(current)
                core.waypoint_handler.set_start(current)
                # Print log.
                self.logger.info("Lost sign of marker, returning to Search Pattern")
                return self.on_event(core.AutonomyEvents.MARKER_UNSEEN)

        return self
