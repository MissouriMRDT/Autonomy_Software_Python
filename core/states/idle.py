#
# Mars Rover Design Team
# idle.py
#
# Created on Nov 20, 2020
# Updated on Aug 21, 2022
#

import core
import time
import logging
import interfaces
import algorithms
from core.states import RoverState
import utm
import matplotlib.pyplot as plt


class Idle(RoverState):
    """
    This is the default state for the state machine. In this state the program does nothing explicit.
    Its singular purpose is to keep the python program running to receive and transmit rovecomm commands
    from base station that configure the next leg’s settings and confirm them.
    """

    def start(self):
        """
        Schedule Idle
        """
        self.logger = logging.getLogger(__name__)
        self.idle_time = time.time()
        self.realigned = False
        self.rover_xs = []
        self.rover_ys = []
        self.max_draw_length = 100

    def exit(self):
        """
        Cancel all state specific coroutines
        """
        # Clear rover path.
        self.rover_xs.clear()
        self.rover_ys.clear()

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events

        :param event:
        :return: RoverState
        """
        state: RoverState = None
        if event == core.AutonomyEvents.START:
            # Set time to zero.
            self.idle_time = 0
            # Check if an ar tag is in front of us.
            if (
                len(
                    [
                        tag.distance
                        for tag in core.vision.ar_tag_detector.get_valid_tags()
                        if tag.distance <= core.constants.NAVIGATION_BACKUP_TAG_DISTANCE_THRESH
                    ]
                )
                >= 1
            ):
                # Move to reversing state.
                state = core.states.Reversing()
            else:
                # Check reverse always toggle.
                if core.constants.NAVIGATION_ALWAYS_REVERSE_OUT_OF_IDLE:
                    # Change states.
                    state = core.states.Reversing()
                else:
                    # Change states.
                    state = core.states.Navigating()

        elif event == core.AutonomyEvents.ABORT:
            state = self

        else:
            self.logger.error(f"Unexpected event {event} for state {self}")
            state = self

        # Call exit() if we are not staying the same state
        if state != self:
            self.exit()

        # Return the state appropriate for the event
        return state

    async def run(self):
        """
        Defines regular rover operation when under this state
        """
        # Send no commands to drive board, the watchdog will trigger and stop the rover from driving anyway
        # The only way to get out of this is through the state machine enable(), triggered by RoveComm

        # Only realign if not mode sim and relative positioning is turned on.
        if core.MODE != "SIM" and core.vision.RELATIVE_POSITIONING:
            # Check if time zero.
            if self.idle_time == 0:
                self.idle_time = time.time()

            # Check if idle time is over threshold and update position.
            if time.time() - self.idle_time > core.constants.IDLE_TIME_GPS_REALIGN and not self.realigned:
                # Check odd time.
                if int(time.time()) % 5 == 0:
                    # Check accuracy of nav board.
                    if interfaces.nav_board.accuracy()[0] < core.constants.IDLE_GPS_ACCUR_THRESH:
                        # Realign gps with relative.
                        interfaces.nav_board.realign()
                        # Print warning that GPS location has been realigned.
                        self.logger.warning(f"Relative positional tracking has been realigned to current GPS location.")
                        # Set toggle.
                        self.realigned = True
            else:
                # Reset toggle.
                self.realigned = False
                pass

        

        # Store rover position path.
        current = interfaces.nav_board.location()
        utm_current = utm.from_latlon(current[0], current[1])
        self.rover_xs.append(utm_current[0])
        self.rover_ys.append(utm_current[1])
        # Write path 1 second before it expires.
        if int(time.time()) % 2 == 0:
            plt.cla()
            # Plot path, current location, and goal.
            plt.gca().set_aspect("equal", adjustable="box")
            plt.plot(self.rover_xs, self.rover_ys, "-b", label="trajectory")
            # Plot rover.
            plt.plot(utm_current[0], utm_current[1], "2", label="rover")
            plt.axis("equal")
            plt.grid(True)
            plt.title(f"IDLE - Heading: {int(interfaces.nav_board.heading())}")
            plt.savefig("logs/!rover_path.png")

            # Check length of the rover path.

            if len(self.rover_xs) > self.max_draw_length:
                # Cutoff old points.
                self.rover_xs = self.rover_xs[::-1][:self.max_draw_length][::-1]
                self.rover_ys = self.rover_ys[::-1][:self.max_draw_length][::-1]
                
        return self
