# System support
import json
import time
import logging
import algorithms.geomath as geomath
from headinghold import headinghold

# User definable constants
WAYPOINT_DISTANCE_THRESHOLD = 1.0  # Meters
BEARING_FLIP_THRESHOLD = 30.0  # 180 +/- this many degrees
SPEED = 20  # Percent
GPS_TRUST_SPEED = 30  # Speed in meters per second at which 100% of bearing is
# calculated using the delta in GPS coordinates
XTE_STRENGTH = 0.25  # Crosstrack Correction strength (0.0 - 1.0)


def reached_goal(goal, location, start):
    (s_bearing, s_distance) = geomath.haversine(start.lat, start.lon, goal.lat, goal.lon)
    (c_bearing, c_distance) = geomath.haversine(location.lat, location.lon, goal.lat, goal.lon)

    distanceMeters = c_distance * 1000.0
    close_enough = distanceMeters < WAYPOINT_DISTANCE_THRESHOLD

    bearing_diff = (s_bearing - c_bearing) % 360.0
    past_goal = 180 - BEARING_FLIP_THRESHOLD <= bearing_diff <= 180 + BEARING_FLIP_THRESHOLD

    if past_goal or close_enough:
        return True
    else:
        return False


clamp = lambda n, minn, maxn: max(min(maxn, n), minn)


class Autonomy:
    def __init__(self, gps, magnetometer, motors, lidar):
        print("Starting Init")
        self.gps = gps
        self.magnetometer = magnetometer
        self.motors = motors
        self.lidar = lidar

        self.goal = None
        self.location = gps.location()
        self.last_location = self.location
        self.startpoint = self.location
        self.prevtime = time.time()
        # self.xte_pid = PIDcontroller(Kp=0.1, Ki=0.01, Kd=0)

        self._decimation = 0
        self.distance_to_goal = 0
        printnl("Init Complete\n")

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
            ## I think this code isn't needed -
            ## Magnetometer correction should show up in the integral term
            ## in the cross track correction PID loop
            # (gps_heading, dist_moved) = geomath.haversine(self.last_location.lat, self.last_location.lon, self.location.lat, self.location.lon)
            # # The GPS refreshes more slowly than the control loop
            # # Speed can only be calculated when the GPS updates
            # if(dist_moved > 0):
            #     dt = time.time() - gps.lastFixTime
            #     speed_m_s = 1000.0 * (dist_moved / dt)
            #     gps_weight = clamp((speed_m_s / GPS_TRUST_SPEED), 0.0, 1.0)
            # current_heading = gps_weight * gps_heading + (1.0 - gps_weight) * mag.heading()

            # Skip the filtering, trust the magnetometer
            current_heading = self.magnetometer.heading()

            self._decimation += 1
            if (self._decimation % 20) == 0:
                logging.info("\n\tLocation \t: %s\n"
                             "\n\tTarget Distance \t: %f\n"
                             "\tTarget Heading  \t: %d\n"
                             "\tCrosstrack Error\t: %f\n"
                             "\tCrosstrack Hdg  \t: %d\n"
                             "\tMeasured Heading\t: %f\n" %
                             (self.location, target_distance, target_heading, xte_dist, 
                              xte_bearing, current_heading))

            # Adjust path
            headinghold(goal_heading, current_heading, self.motors, SPEED)
            return False
        else:
            logging.info("WAYPOINT REACHED")
            #logging.info("\n\tLocation \t: %s\n"
            #             "\n\tTarget Distance \t: %f\n"
            #             "\tTarget Heading  \t: %d\n"
            #             "\tCrosstrack Error\t: %f\n"
            #             "\tCrosstrack Hdg  \t: %d\n"
            #             "\tMeasured Heading\t: %f\n" %
            #             (self.location, target_distance, target_heading, xte_dist, 
            #              xte_bearing, current_heading))
            return True


if __name__ == "__main__":
    from drivers import hmc5883l as magnetometer
    from drivers.navboard_gps import GPS
    from drivers.rovecomm import RoveComm
    from drivers.motorsRoveComm import Motors
    from algorithms.lidar import LiDAR

    # Hardware Setup
    motors = Motors()
    rovecomm_node = RoveComm()
    gps = GPS(rovecomm_node)
    lidar = LiDAR()

    # Using magnetic declination to compensate for how the compass is mounted.
    # TODO: This feels like the wrong way to do things
    mag = magnetometer.hmc5883l(gauss=1.3, declination=(-175, 0))

    autonomy = Autonomy(gps, mag, motors, lidar)

    with open('waypoints.json', 'rb') as waypointfile:
        autonomy.waypoints = json.load(waypointfile)

    autonomy.start()
