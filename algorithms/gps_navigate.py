import algorithms.geomath as geomath
import algorithms.heading_hold as hh

# User definable constants
WAYPOINT_DISTANCE_THRESHOLD = 1.0  # Meters
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

    return past_goal or close_enough


def calculate_move(goal, location, start, drive_board, nav_board):

    (target_heading, target_distance) = geomath.haversine(location.lat, location.lon, goal.lat, goal.lon)

    # Crosstrack Correction as linear
    (xte_bearing, xte_dist) = geomath.crosstrack_error_vector(start, goal, location)

    goal_heading = geomath.weighted_average_angles([target_heading, xte_bearing], [1 - XTE_STRENGTH, XTE_STRENGTH])

    return hh.get_motor_power_from_heading(goal_heading, drive_board, nav_board)


class GPSData:

    def __int__(self, goal, start):
        self.goal = goal
        self.start = start

    def data(self):
        return self.goal, self.start
