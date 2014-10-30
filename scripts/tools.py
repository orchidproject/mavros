"""Utility functions used by package python scripts
"""
import mavros.msg as msg
from mavros.msg import Error
from math import radians, cos, sin, asin, sqrt

#******************************************************************************
# Used for estimating distances between GPS waypoints
#******************************************************************************
RADIUS_OF_EARTH_IN_METRES = 6367 * 1000.0

#******************************************************************************
#   Convenience Error definitions for returning results from callbacks
#******************************************************************************
SUCCESS_ERR                  = Error(code=Error.SUCCESS)
FAILURE_ERR                  = Error(code=Error.FAILURE)
UNSUPPORTED_MODE_ERR         = Error(code=Error.UNSUPPORTED_MODE)
UNSUPPORTED_FRAME_ERR        = Error(code=Error.UNSUPPORTED_FRAME)
COORDS_OUT_OF_RANGE_ERR      = Error(code=Error.COORDS_OUT_OF_RANGE)
MAV_TIMEOUT_ERR              = Error(code=Error.MAV_TIMEOUT)
PARAM_NOT_SET_ERR            = Error(code=Error.PARAM_NOT_SET)
BAD_PARAM_VALUE_ERR          = Error(code=Error.BAD_PARAM_VALUE)
KEY_VALUE_COUNT_MISMATCH_ERR = Error(code=Error.KEY_VALUE_COUNT_MISMATCH)
MAV_COMMAND_ERROR_ERR        = Error(code=Error.MAV_COMMAND_ERROR)
UNSUPPORTED_COMMAND_ERR      = Error(code=Error.UNSUPPORTED_COMMAND)
UNDEFINED_COMMAND_ERR        = Error(code=Error.UNDEFINED_COMMAND)
INTERNAL_ERR                 = Error(code=Error.INTERNAL)
UNDEFINED_WAYPOINT_ERR       = Error(code=Error.UNDEFINED_WAYPOINT)
WAYPOINT_VERIFICATION_FAILURE_ERR = \
    Error(code=Error.WAYPOINT_VERIFICATION_FAILURE)

#******************************************************************************
#   Utility classes
#******************************************************************************
class GlobalWaypoint:
    """Utility class for representing waypoints in global frame"""
    def __init__(self,lat=None,lon=None,alt=None):
        latitude = lat
        longitude = lon
        altitude = alt

    def from_waypoint_message(cls,msg):
        """Constructs a new GlobalWaypoint instance from a waypoint message

           Example usage:
           waypoint = GlobalWaypoint.from_waypoint_message(msg)

           Parameters
           msg - a mavros/Waypoint.msg in the global coordinate frame

           Returns
           - A new GlobalWaypoint with coordinates taken from msg
           - None if msg is not in the global coordinate frame
        """

        #**********************************************************************
        #   Don't accept waypoint if its not global.
        #   Return None to indicate failure
        #**********************************************************************
        if msg.Waypoint.FRAME_GLOBAL != msg.frame:
            return None

        #**********************************************************************
        #   Otherwise return a new representation, using mapping defined
        #   in mavros/Waypoint.msg
        #**********************************************************************
        return cls(lat=msg.x, lon=msg.y, alt=msg.z)



#******************************************************************************
#   Module functions
#******************************************************************************
def distance_along_ground(wp1,wp2):
    """Returns the distance along the ground between two waypoints

       Parameters
       wp1 - the first waypoint
       wp2 - the second waypoint

       Parameters can be of any type, provided they have member variables
       named latitude and longitude specifying coordinates in degrees.

       Parameters can also be mavros/Waypoint.msg objects, provided
       the are in the global frame.

       Returns the distance along the ground between the two points
       Returns None if arguments do not comply with specification above
    """

    #**************************************************************************
    #   Convert parameters if the are waypoint messages
    #   Return None if they are not in GLOBAL frame
    #**************************************************************************
    if type(wp1) is msg.Waypoint:
        wp1 = GlobalWaypoint.from_waypoint_message(wp1)
        if wp1 is None:
            return None
        
    if type(wp2) is msg.Waypoint:
        wp2 = GlobalWaypoint.from_waypoint_message(wp2)
        if wp2 is None:
            return None

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

def total_distance(wp1,wp2):
    """Returns total distance between waypoints, taking into account altitude

       Parameters
       wp1 - the first waypoint
       wp2 - the second waypoint

       Parameters can be of any type, provided they have member variables
       named latitude and longitude specifying coordinates in degrees.
    """

    vertical_distance = wp1.altitude-wp2.altitude
    horizontal_distance = distance_along_ground(wp1,wp2)

    # check for error from distance_along_ground function
    if horizontal_distance is None:
        return None

    total = math.sqrt(vertical_distance**2 + horizontal_distance**2)
    return total


