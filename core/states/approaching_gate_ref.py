import algorithms.geomath as geomath
import algorithms.small_movements as small_movements
from core.vision.ar_tag_detector import is_gate
import core
import interfaces
import algorithms
from core.states import RoverState
import time
import logging
import math
from core.vision.ar_tag_detector import clear_tags
import geopy.distance
import geopy
from collections.abc import Callable
from typing import Tuple
import numpy as np

# Create logger object.
logger = logging.getLogger(__name__)

GATE_OFFSET = 1
DISTANCE_THRESHOLD = 0.5
TURN_FREQ = 10
PAST_GATE_DISTANCE = 5

POLAR_COORD = Tuple[float, float]
CART_COORD = Tuple[float, float]


class ApproachingGate(RoverState):
    """
    Within approaching gate, 3 waypoints are calculated in front of, between, and throught the viewed gate, allowing the rover to traverse through the gate fully.
    """

    def start(self):
        # Is it the first iteration of the state?
        self.is_first = True
        self.gate_search = True
        # Id of the left tag
        self.tagL_id = 4
        # Current approaching_gate (AG) state
        self.state = "mid"
        # Angle and distance from current target
        self.distance, self.angle = 0, 0
        # Amount of iterations on tag_nav
        self.iteration = 1
        # GPS coordinates of the current target
        self.targ_coord = None
        # List of the tags the last time they were both spotted in one frame
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
        self.logger.info("Gate detected, beginning navigation")

        # Approach a point in front of the gate
        if self.state == "front":
            self.logger.info("Current AG State: Front")
            self.tag_nav(self.calc_point_before, "mid", dist_thres=2, turn_freq=10)

        # Approach a point in the middle of the gate
        elif self.state == "mid":
            self.logger.info("Current AG State: Middle")
            self.tag_nav(self.calc_point_midpoint, "zoom", dist_thres=0.5, turn_freq=10)

        # Go straight a certain distance
        elif self.state == "zoom":
            self.logger.info("Current AG State: Zoom")
            small_movements.time_drive(PAST_GATE_DISTANCE)
            self.state = "complete"
            interfaces.drive_board.stop()

        # We have accomplished the goal
        else:
            self.logger.info("Current AG State: Complete")
            return self.on_event(core.AutonomyEvents.REACHED_MARKER)

        return self

    def tag_nav(self, calc_point_func: Callable, next_state: str, dist_thres=1.0, turn_freq=10):
        """
        Travels to a target point that is defined by its position relative to the two gate tags.

        :param calc_point_func: A function that takes the polar position of the two gate markers with
            respect to the rover and returns the polar position of the target marker with respect to the rover.
        :param next_state: The next state after we reach the target
        :param dist_thres: How close we need to be to the target
        :param turn_freq: After how many iterations before rotating the rover to face the target
        :return: self
        """

        def verify_tag(tag):
            for v in tag:
                print("VAL: ", v)
                if np.isnan(v) or np.isinf(v):
                    print("NO")
                    return False
            print("YES")
            return True

        print(f"TAG NAV STATUS: {self.distance} {self.angle}")
        if self.targ_coord is not None:
            pos = interfaces.nav_board.location()
            print(f"POS: {pos}")
            print(f"TARGET: {self.targ_coord}")
            d, a = geomath.haversine(pos[0], pos[1], self.targ_coord[0], self.targ_coord[1])
            d = geopy.distance.geodesic(pos, self.targ_coord).km * 1000
            heading = interfaces.nav_board.heading()
            print("HEADING: ", heading)
            d_lat = self.targ_coord[0] - pos[0]
            d_lon = self.targ_coord[1] - pos[1]
            g = -math.degrees(math.atan(d_lat / d_lon))
            print(g)
            a = self.heading_diff(heading, g)
            print(f"GPS STATUS: {d} {a}")

        # Make sure gate is in view
        if self.gate_search:
            print("GATE SEARCH")
            self.find_gate()

        # First iteration of this state
        elif self.is_first:

            print("FIRST TIMER")
            # Initialize information on the gate tag's positions
            tags = core.vision.ar_tag_detector.get_gate_tags()

            tagL, tagR = self.parse_tags(tags)
            self.distance, self.angle = calc_point_func(tagL, tagR)
            self.last_tags_both_detected = [tagL, tagR]

            self.update_target_gps()

            # Rotate the rover so it's facing the target
            self.recenter(self.angle)

            self.is_first = False

        # Have we gotten close enough to the target?
        elif self.distance is not None and self.distance < dist_thres:
            self.state = next_state
            self.is_first = True
            self.gate_search = True

        # Move the rover towards the target
        else:
            tags = core.vision.ar_tag_detector.get_gate_tags()

            tagL, tagR = self.parse_tags(tags)

            val_tags = []
            val_ids = []
            if tagL is not None and verify_tag(tagL):
                val_tags.append(tagL)
                val_ids.append(self.tagL_id)
            if tagR is not None and verify_tag(tagR):
                val_tags.append(tagR)
                val_ids.append(self.tagL_id + 1)

            # Both tags are visible
            if len(val_tags) == 2:
                print("FULL RELIANCE")
                # tagL, tagR = self.parse_tags(tags)

                self.distance, self.angle = calc_point_func(tagL, tagR)

                self.last_tags_both_detected = [tagL, tagR]

                self.update_target_gps()

            # Only one tag is visible
            elif len(val_tags) == 1:
                print("SINGULAR RELIANCE")

                # Which tag isn't visible
                # which = 0 if tags[0].id == self.tagL_id else 1
                which = 0 if val_ids[0] == self.tagL_id else 1

                # Estimate the position of the missing tag
                # based on last time they were seen together
                if which:
                    tagL = self.calc_other_tag(tagR, which)
                else:
                    tagR = self.calc_other_tag(tagL, which)

                self.distance, self.angle = calc_point_func(tagL, tagR)

                self.update_target_gps()

            # Not tags seen
            else:
                pos = interfaces.nav_board.location()

                if self.targ_coord is None:
                    raise Exception("Target coordinate can't be empty")

                # Use the estimated GPS position of the tag to determine its relative position
                self.distance, self.angle = geomath.haversine(pos[0], pos[1], self.targ_coord[0], self.targ_coord[1])

            # Recenter the rover on the target
            if self.iteration % turn_freq == 0:
                self.recenter(self.angle)

            self.iteration += 1

            self.move(self.angle)

            time.sleep(0.01)

        return self

    def pol_target_gps(self) -> POLAR_COORD:
        """
        Find the polar position of the target with respect to the rover
        using gps.

        :return distance: distance from the rover
        :return angle: angle with respect to the rover's heading
        """

        curr = interfaces.nav_board.location()
        head = interfaces.nav_board.heading()

        if self.targ_coord is None:
            raise Exception("Target can't be none!!!")

        distance, bearing = geomath.haversine(curr[0], curr[1], self.targ_coord[0], self.targ_coord[1])
        angle = self.heading_diff(head, bearing)

        return distance, angle

    def update_target_gps(self) -> None:
        """ "
        Update the gps position of the target using the last recorded distance and angle.
        """

        curr = interfaces.nav_board.location()
        head = interfaces.nav_board.heading()

        try:
            self.targ_coord = self.calc_gps(curr, head, self.distance, self.angle)
        except ValueError:
            pass

    def calc_point_before(self, tagL: POLAR_COORD, tagR: POLAR_COORD) -> POLAR_COORD:
        """
        This function calculates the polar position of a point in front of the gate
        a distance GATE_OFFSET with respect to the rover.

        :param tagL: The polar position of the left gate tag
        :param tagR: The polar position of the right gate tag
        :return: The polar position of the tag before the gate
        """

        def find_intersect_line(cart1, cart2):
            slope = (cart1[1] - cart2[1]) / (cart1[0] - cart2[0])
            b = cart1[1] - slope * cart1[0]
            return slope, b

        cartL = polar_to_cartesian(*tagL)
        cartR = polar_to_cartesian(*tagR)

        # The slope and y-intercept of the line intersecting both gate tags
        m, b = find_intersect_line(cartL, cartR)

        # Midpoint of the gate (cartesian)
        mid_x, mid_y = midpoint(cartL, cartR)

        # Find the position GATE_OFFSET in front of the gate
        m_perp = -(1 / m)
        theta = math.atan(m_perp)

        x_diff = math.cos(theta) * GATE_OFFSET
        y_diff = math.sin(theta) * GATE_OFFSET

        final_x = mid_x - x_diff
        final_y = mid_y - y_diff

        return cartesian_to_polar(final_x, final_y)

    def calc_point_midpoint(self, tagL: POLAR_COORD, tagR: POLAR_COORD) -> POLAR_COORD:
        """
        This function calculates the polar position of a point in the middle of the gate
        with respect to the rover.

        :param tagL: The polar position of the left gate tag
        :param tagR: The polar position of the right gate tag
        :return: The polar position of the tag in the middle of the gate
        """

        c1 = polar_to_cartesian(*tagL)
        c2 = polar_to_cartesian(*tagR)

        m = midpoint(c1, c2)

        return cartesian_to_polar(*m)

    def find_gate(self) -> None:
        """
        Rotate the rover until the gate is in sight
        """

        # while not core.vision.ar_tag_detector.is_gate():
        #     interfaces.drive_board.send_drive(150, -150)
        # interfaces.drive_board.stop()

        if not core.vision.ar_tag_detector.is_gate():
            small_movements.rotate_rover(10)
        else:
            self.gate_search = False

    def recenter(self, a: float) -> None:
        """
        Rotate the rover until it's facing the specified angle with respect to it's current position.

        :param a: The angle of the target with respect to the rover
        """

        small_movements.rotate_rover(a)

    def move(self, a: float) -> None:
        """
        Move the rover in the direction of the specified angle

        :param a: The angle of the target with respect to the rover
        """

        left, right = algorithms.follow_marker.drive_to_marker(300, a)

        if a < -0.5:
            right *= 1.2
        elif a > 0.5:
            left *= 1.2

        right = int(right)
        left = int(left)

        interfaces.drive_board.send_drive(left, right)

    def calc_gps(self, pos, heading: float, d: float, a: float):
        """
        Calculate the gps postion of the polar position defined by d and a.

        :param pos: Current GPS position of the rover
        :param heading: Heading of the rover
        :param d: Distance to the target from the rover
        :param a: Angle to the target with respect to the rover's heading
        :return: The coordinate of the target point
        """

        pos = geopy.Point(pos[0], pos[1])
        targ_point = geopy.distance.distance(meters=d).destination(pos, bearing=(heading + math.degrees(a)) % 360)
        return core.Coordinate(targ_point[0], targ_point[1])

    def calc_other_tag(self, known_tag, which: int):
        """
        Calculate the position of the unknown tag using the known tags position

        :param known_tag: Polar position of the known tag with respect to the rover
        :param which: 0 means left and 1 means right is the known tag
        """

        tagL_b, tagR_b = self.last_tags_both_detected

        if tagR_b is None or tagL_b is None:
            raise Exception("Tags must be initialized")

        cart_tagL_b = polar_to_cartesian(*tagL_b)
        cart_tagR_b = polar_to_cartesian(*tagR_b)

        if which:
            tagR = known_tag
            cart_tagR = polar_to_cartesian(*tagR)

            diff_x = cart_tagL_b[0] - cart_tagR_b[0]
            diff_y = cart_tagL_b[1] - cart_tagR_b[1]

            cart_tagL = (cart_tagR[0] + diff_x, cart_tagR[1] + diff_y)
            tagL = cartesian_to_polar(*cart_tagL)

            return tagL

        else:
            tagL = known_tag
            cart_tagL = polar_to_cartesian(*tagL)

            diff_x = cart_tagR_b[0] - cart_tagL_b[0]
            diff_y = cart_tagR_b[1] - cart_tagL_b[1]

            cart_tagR = (cart_tagL[0] + diff_x, cart_tagL[1] + diff_y)
            tagR = cartesian_to_polar(*cart_tagR)

            return tagR

    def heading_diff(self, facing, target):
        diff = target - facing
        if abs(diff) > 180:
            if diff > 0:
                return diff - 360
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


def midpoint(p1: CART_COORD, p2: CART_COORD) -> CART_COORD:
    return (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2


def polar_to_cartesian(d: float, a: float) -> CART_COORD:
    a = -math.radians(a)
    x = d * math.cos(a)
    y = d * math.sin(a)
    return x, y


def cartesian_to_polar(x: float, y: float) -> POLAR_COORD:
    d = math.sqrt(x**2 + y**2)
    a = -math.degrees(math.atan(y / x))
    return d, a
