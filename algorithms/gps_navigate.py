import time
import algorithms.geomath as geomath

# User definable constants
WAYPOINT_DISTANCE_THRESHOLD = 1.0  # Meters
BEARING_FLIP_THRESHOLD = 30.0  # 180 +/- this many degrees
SPEED = 30  # Percent
GPS_TRUST_SPEED = 30  # Speed in meters per second at which 100% of bearing is
# calculated using the delta in GPS coordinates
XTE_STRENGTH = 0.5  # Crosstrack Correction strength (0.0 - 1.0)


def get_current_heading(self):
    self.prevtime = time.time()

    # Get location
    self.last_location = self.location
    self.location = self.gps.location()

    (target_heading, target_distance) = geomath.haversine(
        self.location.lat, self.location.lon,
        self.goal.lat, self.goal.lon)

    self.distance_to_goal = target_distance

    # Crosstrack Correction as linear
    (xte_bearing, xte_dist) = geomath.crosstrack_error_vector(
        self.startpoint,
        self.goal,
        self.location)

    # Crosstrack correction as PID
    # xte_correction = self.xte_pid.update(setpoint=0, real_position=xte_bearing)

    goal_heading = geomath.weighted_average_angles(
        [target_heading, xte_bearing],
        [1 - XTE_STRENGTH, XTE_STRENGTH])

    # Skip the filtering, trust the magnetometer
    current_heading = self.headingRef.trueHeading

    self._decimation += 1
    if (self._decimation % 5) == 0:
        print("\n\tLocation \t: %s\n"
              "\n\tTarget Distance \t: %f\n"
              "\tTarget Heading  \t: %d\n"
              "\tMeasured Heading\t: %f\n" %
              (self.location, target_distance, target_heading, current_heading))

    # Adjust path
    return current_heading
