import core
from core.constants import LOWER_RATE_OF_CHANGE, SP_LEFT_MAX, SP_LEFT_MIN, SP_RIGHT_MAX
import interfaces
import asyncio
from core.states import RoverState
import time
from interfaces.drive_board import clamp

class SearchPattern(RoverState):
    """
    This state’s goal is to drive the rover in a pattern that mimics an Archimedean Spiral without using GPS coordinates.
    It accomplishes this via drive commands that decrease/increase in speed over time, causing it to turn in increasingly wider angles.
    This state should only be used when innaccurate GPS data causes GPS search pattern to be compromised.
    Command line argument for this state is --gps-search "GPS" or "NO_GPS", default = "GPS". 
    """

    def start(self):
        #Set a starting time variable
        global t1
        t1 = time.time()

    def exit(self):
        # Cancel all state specific tasks
        pass

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events
        """
        state: RoverState = None

        if event == core.AutonomyEvents.MARKER_SEEN:
            state = core.states.ApproachingMarker()

        elif event == core.AutonomyEvents.GATE_SEEN:
            state = core.states.ApproachingGate()

        elif event == core.AutonomyEvents.START:
            state = self

        elif event == core.AutonomyEvents.SEARCH_FAILED:
            state = core.states.Idle()

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
        
        #Set a second time variable that increases with each iteration of the loop
        t2 = time.time()

        if core.vision.ar_tag_detector.is_gate(): #Check for gate
            core.waypoint_handler.gps_data.leg_type = "GATE"
            interfaces.drive_board.stop()

            # Sleep for a brief second
            await asyncio.sleep(0.1)

            self.logger.info("Search Pattern: Gate seen")
            return self.on_event(core.AutonomyEvents.GATE_SEEN)

        elif core.vision.ar_tag_detector.is_marker(): #Check for marker
            interfaces.drive_board.stop()

            # Sleep for a brief second
            await asyncio.sleep(0.1)

            self.logger.info("Search Pattern: Marker seen")
            return self.on_event(core.AutonomyEvents.MARKER_SEEN)

        #Sleep so we don't overwhelm drive board
        await asyncio.sleep(0.1)
        
        seconds_elapsed = t2 - t1 #Calculates elapsed time from start
        print(f'Time elapsed: {seconds_elapsed:.2f}')

        #TODO: Fix the problem

        #Divider value decreases the rate of change over time.
        divider_value = seconds_elapsed * core.LOWER_RATE_OF_CHANGE + 0.57
        print(f'Divider value; {divider_value}')

        #Calculate drive speed. Currently left is set to decrease and right is set to increase.
        #Clamp divider value for testing purposes.
        left = int(core.MAX_DRIVE_POWER - ((seconds_elapsed * core.DECREMENT)) / (clamp(divider_value, 1, 100)))
        right = int(core.MIN_DRIVE_POWER + ((seconds_elapsed * core.INCREMENT)) / (clamp(divider_value, 1, 100)))
    
        print(f"Search Pattern: Driving at ({left}, {right})")
        self.logger.info(f"Search Pattern: Driving at ({left}, {right})")
        interfaces.drive_board.send_drive(left, right)

        return self