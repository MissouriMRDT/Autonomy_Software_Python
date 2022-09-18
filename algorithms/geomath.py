# Geographic math library
#
# Author: Owen Chiaventone, Edward Koharik
#
# Includes functions to calculate the bearing
# and distance between two points (haversine)
# and the crosstrack error between a line and a point

import math


# Haversine Function for calculating bearing and distance
# Source:
# http://stackoverflow.com/questions/4913349/haversine-formula-in-python-bearing-and-distance-between-two-gps-points
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
        distance : float (kilometers)
            distance to target in kilometers
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371  # Radius of earth in kilometers. Use 3956 for miles
    distance = c * r

    bearing = math.atan2(
        math.sin(lon2 - lon1) * math.cos(lat2),
        math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1),
    )
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360

    return bearing, distance

# Source by David M:
# https://stackoverflow.com/questions/7222382/get-lat-long-given-current-point-distance-and-bearing
def reverse_haversine(heading, distance, lat1, lon1):
    """
    Calculate the latitude and longitude of the location a specified
    distance in front of the rover. Assumes a perfectly spherical earth,
    so take the results with a grain of salt.

    Parameters:
    -----------
        bearing : float
            bearing to target as degrees clockwise from due north
        distance : float (meters)
            distance to target in meters

    Returns
    -------
        lat1 : float
            latitude of destmation that meets the parameters
        lon1 : float
            longitude of destmation that meets the parameters
    """

    R = 6378.1 # Radius of the Earth; DIFFERNT FROM HAVERSINE CONST; matches with google search
    bearing = (360 - heading + 90) % 360 # Convert heading (relative to north moving clockwise) to bearing (relative to east moving counter-clockwise)
    bearing = math.radians(bearing) # degrees to radians 
    distance = distance / 1000 # Distance in km

    # test for heading = 0, distance = 15000, lat1 = 52,20472, lon1 = 0.14056
    # lat2  52.20444 - the lat result I'm hoping for
    # lon2  0.36056 - the long result I'm hoping for.

    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)

    lat2 = math.asin( math.sin(lat1)*math.cos(distance/R) +
           math.cos(lat1)*math.sin(distance/R)*math.cos(bearing))

    lon2 = lon1 + math.atan2(math.sin(bearing)*math.sin(distance/R)*math.cos(lat1),
           math.cos(distance/R)-math.sin(lat1)*math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    return lat2, lon2
