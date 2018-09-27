# System support
import json
import time
import logging
import algorithms.geomath as geomath
from collections import namedtuple
from headinghold import headingHold

Coordinate = namedtuple('Coordinate', ['lat', 'lon'])

# User definable constants
WAYPOINT_DISTANCE_THRESHOLD = 1.0  # Meters
BEARING_FLIP_THRESHOLD = 30.0  # 180 +/- this many degrees
SPEED = 30  # Percent
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


clamp = lambda n, minn, maxn: max(min(maxn, n), minn)


class GPSNavigate:

    def __init__(self, gps, headingRef, motors, gpsCalFile="gps_calibration.json"):
        self.gps = gps
        self.headingRef = headingRef
        self.motors = motors
        # self.lidar = lidar

        self.goal = gps.location()
        self.location = gps.location()
        self.last_location = self.location
        self.startpoint = self.location
        self.prevtime = time.time()
        # self.xte_pid = PIDcontroller(Kp=0.1, Ki=0.01, Kd=0)

        self._decimation = 0
        self.distance_to_goal = 0

        with open(gpsCalFile) as calfile:
            self.headingOffset = json.load(calfile)["gps_offset"]

    def setWaypoint(self, goal_coordinate):
        self.startpoint = self.location
        self.goal = goal_coordinate
        logging.info("Moving from %s to %s" % (self.startpoint, goal_coordinate))

    def update_controls(self):
        """
        Continue on path to goal
        :return:  Waypoint reached? True / False
        """
        if not reached_goal(self.goal, self.location, self.startpoint):
            Ts = time.time() - self.prevtime
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
            headingHold(goal_heading, self.headingRef, self.motors, SPEED)
            return False
        else:
            logging.info("WAYPOINT REACHED")
            # logging.info("\n\tLocation \t: %s\n"
            #             "\n\tTarget Distance \t: %f\n"
            #             "\tTarget Heading  \t: %d\n"
            #             "\tCrosstrack Error\t: %f\n"
            #             "\tCrosstrack Hdg  \t: %d\n"
            #             "\tMeasured Heading\t: %f\n" %
            #             (self.location, target_distance, target_heading, xte_dist, 
            #              xte_bearing, current_heading))
            return True
