import asyncio
from algorithms import ar_tag
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

import matplotlib.pyplot as plt

# NO GPS VERSION
# drives in a straight line through the gate (this will be bad at steep angles)

# Create logger object.
logger = logging.getLogger(__name__)

OFFSET_THRESHOLD = 1
DISTANCE_THRESHOLD = 0.5
TURN_FREQ = 10

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

        self.tagL_id = 0
        self.state = 'mid'
        self.distance = None
        self.angle = None
        self.iteration = 1

        self.last_tags_both_detected = []

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
        tags = core.vision.ar_tag_detector.get_gate_tags()
        gps_data = core.waypoint_handler.get_waypoint()
        orig_goal, orig_start, leg_type = gps_data.data()

        self.logger.info("Gate detected, beginning navigation")

        start = core.Coordinate(interfaces.nav_board.location()[0], interfaces.nav_board.location()[1])

        # If we've seen at least 5 frames of 2 tags, assume it's a gate
        self.logger.info("Gate detected, beginning navigation")

        self.tag_nav(self.calc_point_before)

        return self

    def tag_nav(self, calc_point_func):
        if self.is_first:
            print('FINDING GATE')

            self.find_gate()
            tags = core.vision.ar_tag_detector.get_gate_tags()
            tagL, tagR = self.parse_tags(tags)
            self.distance,self.angle = calc_point_func(tagL, tagR)

            #self.calc_point_before(tagL, tagR)
            #gps = self.calc_gps(tagL, tagR)
            
            print("RECENTERING")
            
            self.recenter(self.angle)
            
            time.sleep(1)
            self.is_first = False

        elif self.distance is not None and self.distance < DISTANCE_THRESHOLD:
               print("COMPLETE")
               time.sleep(2)

        else:
            tags = core.vision.ar_tag_detector.get_gate_tags()

            if len(tags) == 2:
                print("TAG UPDATE")
                tagL, tagR = self.parse_tags(tags)
                self.distance, self.angle = calc_point_func(tagL, tagR)
                self.last_tags_both_detected = [tagL, tagR]
                #gps = self.calc_gps()

            elif len(tags) == 1:
                pass
                tagL, tagR = self.parse_tags(tags)

                which = 0 if tags[0].id == self.tagL_id else 1
                
                if which == 0:
                    tagR = self.calc_other_tag(tagL, which)
                else:
                    tagL = self.calc_other_tag(tagR, which)

                self.distance, self.angle = calc_point_func(tagL, tagR)

            else:
                pass

            print(f"Distance {self.distance}\nAngle {self.angle}")

            if self.iteration % TURN_FREQ == 0:
               self.recenter(self.angle)

            self.iteration += 1

            self.move(self.angle)
            time.sleep(0.01)

        return self

    def calc_point_before(self, tagL, tagR):
        def calc_line_intersect(cart1, cart2):
            slope = (cart1[1] - cart2[1])/(cart1[0]-cart2[0])
            b = cart1[1] - slope * cart1[0]
            return slope, b
        
        cartL = self.polar_to_cartesian(*tagL)
        cartR = self.polar_to_cartesian(*tagR)

        m, b = calc_line_intersect(cartL, cartR)

        mid_x, mid_y = self.midpoint(cartL, cartR)

        m_perp = -(1/m)
        theta = math.atan(m_perp)

        x_diff = math.cos(theta) * OFFSET_THRESHOLD
        y_diff = math.sin(theta) * OFFSET_THRESHOLD

        final_x = mid_x - x_diff
        final_y = mid_y - y_diff

        return self.cartesian_to_polar(final_x, final_y)

    def calc_point_midpoint(self, tagL, tagR): 
        c1 = self.polar_to_cartesian(*tagL)
        c2 = self.polar_to_cartesian(*tagR)

        m = self.midpoint(c1,c2)

        return self.cartesian_to_polar(*m) 

    def midpoint(self, p1, p2):
        return (p1[0] + p2[0])/2, (p1[1] + p2[1])/2

    def polar_to_cartesian(self, r,a):
        a = math.radians(a)
        x = r * math.cos(a)
        y = r * math.sin(a)
        return x,y

    def cartesian_to_polar(self, x,y):
        r = math.sqrt(x**2 + y**2)
        a = math.degrees(math.atan(y/x))
        return r,a

    def find_gate(self):
        while not core.vision.ar_tag_detector.is_gate():
            interfaces.drive_board.send_drive(150, -150)
        interfaces.drive_board.stop()

    def recenter(self, a):
        small_movements.rotate_rover(a)

    def move(self, a):
        left, right = algorithms.follow_marker.drive_to_marker(300, a)
        if a < -0.5:
            right *= 1.2
        elif a > 0.5:
            left *= 1.2

        right = int(right)
        left = int(left)

        interfaces.drive_board.send_drive(left, right)

    def calc_gps(self, curr, d, a):
        pass

    def calc_other_tag(self, known_tag, which):
        tagR_b, tagL_b = self.last_tags_both_detected[]

        if which:
            tagR = known_tag
        else:
            tagL = known_tag

    def heading_diff(facing, target):
        diff = target-facing
        if abs(diff) > 180:
            if diff > 0:
                return diff-360
            elif diff < 180:
                return diff % 360
        else:
            return diff
        
    def parse_tags(self, tags):
        tagL, tagR = None, None
        for tag in tags:
            if tag.id == self.tagL_id:
                tagL = (tag.distance, tag.angle)
            else:
                tagR = (tag.distance, tag.angle)
        return tagL, tagR