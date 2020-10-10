import algorithms.geomath as geomath
import algorithms.heading_hold as hh
import math

# User definable constants
WAYPOINT_DISTANCE_THRESHOLD = 1.5  # Meters
BEARING_FLIP_THRESHOLD = 30.0  # 180 +/- this many degrees
GPS_TRUST_SPEED = 30  # Speed in meters per second at which 100% of bearing is
# calculated using the delta in GPS coordinates
XTE_STRENGTH = 0.5  # Crosstrack Correction strength (0.0 - 1.0)


def reached_goal(goal, location, start):
    (s_bearing, s_distance) = geomath.haversine(start.lat, start.lon, goal.lat, goal.lon)
    (c_bearing, c_distance) = geomath.haversine(location.lat, location.lon, goal.lat, goal.lon)

    distanceMeters = c_distance * 1000.0
    close_enough = distanceMeters < WAYPOINT_DISTANCE_THRESHOLD

    bearing_diff = (s_bearing - c_bearing) % 360.0
    past_goal = 180 - BEARING_FLIP_THRESHOLD <= bearing_diff <= 180 + BEARING_FLIP_THRESHOLD

    #if past_goal:
        #print("PAST GOAL!")

    if close_enough:
        print("CLOSE ENOUGH!")

    #return past_goal or close_enough
    return close_enough

def calculate_move(goal, location, start, drive_board, nav_board, speed):

    (target_heading, target_distance) = geomath.haversine(location.lat, location.lon, goal.lat, goal.lon)
    # The Haversine stuff works, reliably. Please let's use it instead.
    # Crosstrack Correction as linear
    #(xte_bearing, xte_dist) = geomath.crosstrack_error_vector(start, goal, location)

    #goal_heading = geomath.weighted_average_angles([target_heading, xte_bearing], [1 - XTE_STRENGTH, XTE_STRENGTH])
    """
    dx = goal.lon - location.lon
    dy = goal.lat - location.lat

    if dx / dy > 0:
        goal_heading = math.degrees(math.atan(dx / dy))

        if dy < 0:
            goal_heading += 180

    elif dy < 0:
        goal_heading = 90 + math.degrees(math.atan(math.fabs(dy) / dx))

    else:
        goal_heading = 360 - math.degrees(math.atan(math.fabs(dx) / dy))

    print("Curr Location: " + str(location) + ", Goal Heading: " + str(goal_heading))

    c = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)) * 10000
    print(c)

    speed = 150
    
    if c < .9:
        speed = int(speed * c) + 10
    """
    #speed = 250 # increased for testing per Rausch
    print(target_distance)
    if target_distance < 0.003:
        speed = 100
    goal_heading = target_heading
    print("Current heading: " + str(nav_board.heading()) + ", Goal:" + str(goal_heading))
    return hh.get_motor_power_from_heading(speed, goal_heading, drive_board, nav_board)


class GPSData:

    def __int__(self, goal, start):
        self.goal = goal
        self.start = start

    def data(self):
        return self.goal, self.start
