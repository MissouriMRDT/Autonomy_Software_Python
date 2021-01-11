import numpy as np
from core.constants import Coordinate
import algorithms.geomath as geomath


"""
UNIT TEST
FILE: geomath.py

This file provides a unit test for the haversine() distance calculation function, it compares its output
to precomputed distances.

Adapted from original test written by: Owen Chiaventone, Edward Koharik
"""


def crosstrack_error_vector(source, destination, location):
    """
    Calculate the distance and bearing from <location> to
    the shortest line  between <source> and <destination>

    Does NOT compensate for the curvature of the earth
    Really only accurate on scales of 5 km or less

    Parameters
    ----------
        source      : GPS Coordinate
        destination : GPS Coordinate
        location    : GPS Coordinate
            Current location

    Returns
    --------
        bearing  : float (degrees)
            Angle (degrees clockwise of due north) of most
            direct path back to the line between <source> and destination
        distance : float (kilometers)
            Deviation from direct path in km
    """

    # Warning: Linear Algebra beyond this point

    # Represent the destination and location as
    # vectors relative to the source
    dest_vector = (destination.lat - source.lat, destination.lon - source.lon)
    loc_vector = (location.lat - source.lat, location.lon - source.lon)

    # Project the location vector onto the destination vector
    # to find the closest point as a vector relative to the source
    closest_point_vector = vector_project(loc_vector, dest_vector)

    # Transform back to a coordinate system relative to the earth
    closest_point = Coordinate(closest_point_vector[0] + source.lat, closest_point_vector[1] + source.lon)

    return geomath.haversine(location.lat, location.lon, closest_point.lat, closest_point.lon)


#####################################
# Linear Algebra helpers
#####################################


class Vector:
    def __init__(self, values):
        # casting vectors as numpy ndarrays is a more standard practice.
        self.values = np.array(values)

    def __add__(self, other):
        return Vector([sum(x) for x in zip(self.values, other)])

    def __mul__(self, scalar):
        return Vector([scalar * x for x in self.values])

    def __getitem__(self, key):
        return self.values[key]

    def __len__(self):
        return len(self.values)

    def __repr__(self):
        return repr(self.values)


def dot(vector1, vector2):
    """ Works on any iterable (tuples, lists, vectors, etc) """
    return sum(vector1[i] * vector2[i] for i in range(len(vector1)))


def vector_project(vector1, vector2):
    """
    Projects <vector1> onto <vector2>
    Forces the output to be on the line between (0,0) and vector2
    If projection would be outside of this line, it goes to the closest endpoint
    """
    scale_factor = dot(vector2, vector1) / dot(vector2, vector2)
    if scale_factor < 0:
        return 0, 0
    if scale_factor > 1:
        return vector2
    else:
        return np.array([scale_factor * v2_i for v2_i in vector2])


def almost_equal(x, y, tolerance=0.0001):
    return abs(x - y) < tolerance


def test_haversine_comparison():
    a = Coordinate(52.10000, 5.50000)  # Source
    b = Coordinate(52.26000, 5.45000)  # Destination
    c = Coordinate(52.19119, 5.52544)  # Location
    d = Coordinate(52.18507, 5.47346)  # Precalculated closest point to c on a->b
    # These coordinates come from example 2 at
    # http://geo.javawa.nl/coordcalc/index_en.html

    precalculated_crosstrack_error = 3.619  # kilometers

    print("Expected Distance  : ", precalculated_crosstrack_error)

    (cd_bearing, cd_distance) = geomath.haversine(c.lat, c.lon, d.lat, d.lon)
    (xte_bearing, xte_dist) = crosstrack_error_vector(a, b, c)

    print("Haversine Distance : ", cd_distance)
    print("Haversine bearing  : ", cd_bearing)
    print("XTE Dist           : ", xte_dist)
    print("xte_bearing        : ", xte_bearing)

    assert almost_equal(cd_distance, precalculated_crosstrack_error, tolerance=0.2)
    assert almost_equal(xte_dist, precalculated_crosstrack_error, tolerance=0.2)
    assert almost_equal(xte_bearing, cd_bearing, tolerance=20)
