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


def utm_distance(easting1, northing1, easting2, northing2):
    """
    Calculate the bearing and distance between two UTM points.

    :param easting1: The UTM easting of the first point.
    :param northing1: The UTM northing of the first point.
    :param easting2: The UTM easting of the second point.
    :param northing2: The UTM northing of the second point.

    :returns bearing: The bearing from point1 to point2.
    :returns distance: The distance between point1 and point2.
    """
    # Calculate bearing.
    bearing = math.degrees(math.atan2(easting2 - easting1, northing2 - northing1))
    # Calculate distance.
    distance = math.sqrt(math.pow(easting2 - easting1, 2) + math.pow(northing2 - northing1, 2))

    return bearing, distance
