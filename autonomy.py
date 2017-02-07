# System support
import json
import time

from algorithms import geomath
from algorithms.headinghold import headinghold
from drivers import hmc5883l as magnetometer
from drivers.gps import GPS
from drivers.motorsRoveComm import Motors

# User definable constants
WAYPOINT_DISTANCE_THRESHOLD = 3.0   # Meters
BEARING_FLIP_THRESHOLD      = 30.0  # 180 +/- this many degrees
SPEED                       = 40    # Percent
GPS_TRUST_SPEED             = 30    # Speed in meters per second at which 100% of bearing is
                                    # calculated using the delta in GPS coordinates
XTE_STRENGTH                = 0.2  # Crosstrack Correction strength (0.0 - 1.0)


def reached_goal(goal, location, start):
    
    (s_bearing, s_distance) = geomath.haversine(start.lat, start.lon, goal.lat, goal.lon)
    (c_bearing, c_distance) = geomath.haversine(location.lat, location.lon, goal.lat, goal.lon)
    
    distanceMeters = c_distance * 1000.0    
    close_enough = distanceMeters < WAYPOINT_DISTANCE_THRESHOLD
    
    bearing_diff = (s_bearing - c_bearing) % 360.0
    past_goal = 180 - BEARING_FLIP_THRESHOLD      \
                 <= bearing_diff                  \
                 <= 180 + BEARING_FLIP_THRESHOLD
    
    if past_goal or close_enough:
        return True
    else:
        return False
    
clamp = lambda n, minn, maxn: max(min(maxn, n), minn)


class Autonomy:
    def __init__(self, gps, magnetometer, motors):
        self.gps = gps
        self.magnetometer = magnetometer
        self.motors = motors

        self.waypoints = []
        self.location = gps.location()
        self.last_location = self.location
        self.startpoint = self.location
        self.prevtime = time.time()
        #self.xte_pid = PIDcontroller(Kp=0.1, Ki=0.01, Kd=0)

        self._decimation = 0
        
    def addWaypoint(self, coordinate):
        self.waypoints.append(coordinate)

    def start(self):
        for waypoint in self.waypoints:
            self.startpoint = self.location
            print "Moving from ", self.startpoint, " to ", waypoint
            while not reached_goal(waypoint, self.location, self.startpoint):
                Ts = time.time() - self.prevtime
                self.prevtime = time.time()

                # Get location
                self.last_location = self.location
                self.location = gps.location()

                (target_heading, target_distance) = geomath.haversine(
                    self.location.lat, self.location.lon, 
                    waypoint.lat     , waypoint.lon)
                
                # Crosstrack Correction as linear
                (xte_bearing, xte_dist) = geomath.crosstrack_error_vector(
                    self.startpoint, 
                    waypoint, 
                    self.location)

                # Crosstrack correction as PID
                # xte_correction = self.xte_pid.update(setpoint=0, real_position=xte_bearing)

                goal_heading = geomath.weighted_angle_average(
                    [target_heading, xte_bearing], 
                    [1-XTE_STRENGTH, XTE_STRENGTH])
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
                current_heading = mag.heading()
                
                self._decimation += 1
                if (self._decimation % 20) == 0:
                    print "Target Distance \t:", target_distance
                    print "Target Heading  \t:", target_heading
                    print "Crosstrack Error\t:", xte_dist
                    print "Crosstrack Hdg  \t:", xte_bearing
                    print "Measured Heading\t:", current_heading 

                # Adjust path    
                headinghold(goal_heading, current_heading, self.motors, SPEED)
                time.sleep(0.01)    
                
            print "WAYPOINT REACHED"

if __name__ == "__main__":

    # Hardware Setup
    motors = Motors()
    gps = GPS("/dev/ttyS0")
    
    # Using magnetic declination to compensate for how the compass is mounted.
    # TODO: This feels like the wrong way to do things
    mag = magnetometer.hmc5883l(gauss = 1.3, declination = (-175,0))

    autonomy = Autonomy(gps, mag, motors)
    
    with open('waypoints.json', 'rb') as waypointfile:
        autonomy.waypoints = json.load(waypointfile)

    autonomy.start()    
