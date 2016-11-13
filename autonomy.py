# System support
import time
import pickle # For loading waypoints from file

# Hardware support
from gps import Coordinate, GPS
import hmc5883l as magnetometer
from motorsRoveComm import Motors

# Algorithms
from math import radians, cos, sin, asin, sqrt, atan2, degrees
from headinghold import headinghold

# User definable constants
WAYPOINT_DISTANCE_THRESHOLD = 3.0   # Meters
BEARING_FLIP_THRESHOLD      = 30.0  # 180 +/- this many degrees
SPEED                       = 40    # Percent
GPS_TRUST_SPEED             = 30    # Speed in meters per second at which 100% of bearing is
                                    # calculated using the delta in GPS coordinates

# Haversine Function for calculating bearing and distance
# Source: http://stackoverflow.com/questions/4913349/haversine-formula-in-python-bearing-and-distance-between-two-gps-points
def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the bearing and great circle distance between two points 
    on the earth (specified in decimal degrees) using 
    the haversine formula. Assumes a perfectly spherical earth,
    so take the results with a grain of salt.
    
    Returns
    -------
        bearing : float
            bearing to target as degrees clockwise from due north
        distance : float    
            distance to target in kilometers
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    distance = c * r
    
    bearing = atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1))
    bearing = degrees(bearing)
    bearing = (bearing + 360) % 360
    
    return (bearing, distance)

def reachedGoal(goal, location, start):
    
    (s_bearing, s_distance) = haversine(start.lat, start.lon, goal.lat, goal.lon)
    (c_bearing, c_distance) = haversine(location.lat, location.lon, goal.lat, goal.lon)
    
    distanceMeters = c_distance * 1000.0    
    close_enough = distanceMeters < WAYPOINT_DISTANCE_THRESHOLD
    
    bearing_diff = (s_bearing - c_bearing) % 360.0
    past_goal = 180 - BEARING_FLIP_THRESHOLD      \
                 <= bearing_diff                  \
                 <= 180 + BEARING_FLIP_THRESHOLD
    
    if(past_goal or close_enough):
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

        self._decimation = 0
        
    def addWaypoint(self, coordinate):
        self.waypoints.append(coordinate)

    def start(self):
        for waypoint in self.waypoints:
            self.startpoint = self.location
            print "Moving from ", self.startpoint, " to ", waypoint
            while not reachedGoal(waypoint, self.location, self.startpoint):
                Ts = time.time() - self.prevtime
                self.prevtime = time.time()

                # Get location
                self.last_location = self.location
                self.location = gps.location()

                (target_heading, target_distance) = haversine(self.location.lat, self.location.lon, waypoint.lat, waypoint.lon)
                (gps_heading, dist_moved) = haversine(self.last_location.lat, self.last_location.lon, self.location.lat, self.location.lon)

                # The GPS refreshes more slowly than the control loop
                # Speed can only be calculated when the GPS updates
                if(dist_moved > 0):
                    dt = time.time() - gps.lastFixTime
                    speed_m_s = 1000.0 * (dist_moved / dt)
                    gps_weight = clamp((speed_m_s / GPS_TRUST_SPEED), 0.0, 1.0)
                
                #current_heading = gps_weight * gps_heading + (1.0 - gps_weight) * mag.heading()

                # Skip the filtering, trust the magnetometer
                current_heading = mag.heading()
                
                self._decimation += 1
                if ( (self._decimation % 20) == 0 ):
		    print "Target Distance\t: ", target_distance
                    print "Target Heading\t:  ", target_heading
                    print "Measured Heading\t:", current_heading 

                # Adjust path    
                headinghold(target_heading, current_heading, self.motors, SPEED)
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
    
    with open('waypoints.dat', 'rb') as waypointfile:
        autonomy.waypoints = pickle.load(waypointfile)

    autonomy.start()    
