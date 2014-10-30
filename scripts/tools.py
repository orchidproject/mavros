"""Utility functions used by package python scripts
"""
from math import radians, cos, sin, asin, sqrt

# Used for estimating distances between GPS waypoints
RADIUS_OF_EARTH_IN_METRES = 6367 * 1000.0

def distance_along_ground(wp1,wp2):
    """Returns the distance along the ground between two waypoints

       Parameters
       wp1 - the first waypoint
       wp2 - the second waypoint

       parameters can be of any type, provided they have member variables
       named latitude and longitude specifying coordinates in degrees.

       Returns the distance along the ground between the two points
    """
    #**************************************************************************
    #   python math functions work in radians, so we need to convert
    #   degrees to radians first
    #**************************************************************************
    lat1 = math.radians(wp1.latitude)
    lon1 = math.radians(wp1.longitude)
    lat2 = math.radians(wp2.latitude)
    lon2 = math.radians(wp2.longitude)

    #**************************************************************************
    #   Estimate the distance along the ground using the Haversine formula
    #**************************************************************************
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))

    horz_dist = RADIUS_OF_EARTH_IN_METRES * c

    return horz_dist
