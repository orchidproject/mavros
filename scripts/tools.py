#!/usr/bin/env python
"""Utility functions used by package python scripts"""
from utm import from_latlon, to_latlon
import math
import rospy
import mavros.msg as msg
import diagnostic_msgs
import diagnostic_msgs.msg
from mavros.msg import Error
from math import radians, cos, sin, asin, sqrt

#******************************************************************************
# Tolerated difference between floats when testing for equality
#******************************************************************************
FLOAT_DIFF_TOLERANCE = 1.0e-10

#******************************************************************************
# Used for estimating distances between GPS waypoints
#******************************************************************************
RADIUS_OF_EARTH_IN_METRES = 6367 * 1000.0

#******************************************************************************
#   Range for valid WGS84 Coordinates
#******************************************************************************
MIN_VALID_LATITUDE  = -90.0
MAX_VALID_LATITUDE  = +90.0
MIN_VALID_LONGITUDE = -180.0
MAX_VALID_LONGITUDE = +180.0

#******************************************************************************
#   ROS Diagnostic error levels
#******************************************************************************
DIAG_UPDATE_FREQ = rospy.Duration(secs=0.2) # how often to update diagnostics
DIAG_STALE = diagnostic_msgs.msg.DiagnosticStatus.STALE
DIAG_ERROR = diagnostic_msgs.msg.DiagnosticStatus.ERROR
DIAG_WARN  = diagnostic_msgs.msg.DiagnosticStatus.WARN
DIAG_OK    = diagnostic_msgs.msg.DiagnosticStatus.OK

#******************************************************************************
#   Convenience Error definitions for returning results from callbacks
#******************************************************************************
SUCCESS_ERR                  = Error.SUCCESS
FAILURE_ERR                  = Error.FAILURE
UNSUPPORTED_MODE_ERR         = Error.UNSUPPORTED_MODE
UNSUPPORTED_FRAME_ERR        = Error.UNSUPPORTED_FRAME
COORDS_OUT_OF_RANGE_ERR      = Error.COORDS_OUT_OF_RANGE
MAV_TIMEOUT_ERR              = Error.MAV_TIMEOUT
PARAM_NOT_SET_ERR            = Error.PARAM_NOT_SET
BAD_PARAM_VALUE_ERR          = Error.BAD_PARAM_VALUE
KEY_VALUE_COUNT_MISMATCH_ERR = Error.KEY_VALUE_COUNT_MISMATCH
MAV_COMMAND_ERROR_ERR        = Error.MAV_COMMAND_ERROR
UNSUPPORTED_COMMAND_ERR      = Error.UNSUPPORTED_COMMAND
UNDEFINED_COMMAND_ERR        = Error.UNDEFINED_COMMAND
INTERNAL_ERR                 = Error.INTERNAL
UNDEFINED_WAYPOINT_ERR       = Error.UNDEFINED_WAYPOINT
WAYPOINT_VERIFICATION_FAILURE_ERR = \
    Error.WAYPOINT_VERIFICATION_FAILURE
NO_GPS_FIX_ERR               = Error.NO_GPS_FIX
SERVICE_CALL_FAILED_ERR      = Error.SERVICE_CALL_FAILED
UNKNOWN_CAMERA_ERR           = Error.UNKNOWN_CAMERA

#******************************************************************************
#   Utility classes
#******************************************************************************
class UTMWaypoint:
    """Utility class used to represent UTM coordinates"""

    def __init__(self, easting=0.0, northing=0.0, altitude=0.0,
        zone_number=None, zone_letter=None):
        """Constructs new instance with specified parameters

           Parameters
              easting - easting in metres from UTM zone origin [default: 0.0]
              northing - northing in metres from UTM zone origin [default: 0.0]
              zone_number - UTM zone number [default: None]
              zone_letter - UTM zone letter [default: None]
        """
        self.easting = easting
        self.northing = northing
        self.altitude = altitude
        self.zone_number = zone_number
        self.zone_letter = zone_letter

    @classmethod
    def from_waypoint_message(cls,wp):
        """Constructs a new UTMWaypoint from a waypoint message

           Parameters
           msg - waypoint in global frame

           Returns
           - A new UTMWaypoint with coordinates converted from msg
           - None if msg is not in the global coordinate frame
        """

        #**********************************************************************
        #   Don't accept waypoint if its not global.
        #   Return None to indicate failure
        #**********************************************************************
        if msg.Waypoint.FRAME_GLOBAL != wp.frame:
            return None

        #**********************************************************************
        #   Otherwise return a new representation, using mapping defined
        #   in mavros/Waypoint.msg
        #**********************************************************************
        (east, north, z_num, z_let) = from_latlon(wp.x,wp.y)       
        result = cls()  # a new UTMWaypoint instance
        result.easting = east
        result.northing = north
        result.altitude = wp.z
        result.zone_number = z_num
        result.zone_letter = z_let
        return result

    def to_waypoint_message(self):
        """Returns this UTMWaypoint as a waypoint message in the global frame

           Returns Waypoint message object specified in global frame.

           Only the x,y,z and frame attributes will be filled in.
           It is the caller's responsiblity to set all other attributes
           if required.
        """

        #**********************************************************************
        #   Convert this UTM position to global coordinates
        #**********************************************************************
        altitude = self.altitude  # altitude requires no conversion
        (latitude, longitude) = to_latlon(
                self.easting,
                self.northing,
                self.zone_number,
                self.zone_letter
                )

        #**********************************************************************
        #   Fill in frame and position
        #**********************************************************************
        wp = msg.Waypoint()
        wp.frame = msg.Waypoint.FRAME_GLOBAL
        wp.x = latitude
        wp.y = longitude
        wp.z = altitude

        #**********************************************************************
        #   Set other attributes to safe defaults. Worst case, if this
        #   waypoint was used unchanged to control drone, you'd expected to
        #   wait at this waypoint forever (because its effectively unreachable
        #   within 0 radius.
        #**********************************************************************
        wp.autocontinue = False
        wp.radius = 0.0
        wp.waitTime = rospy.Duration(secs=0.0)
        return wp

    def to_global_waypoint(self):
        """Converts this UTM waypoint to a Global waypoint"""
        wp = self.to_waypoint_message()
        return GlobalWaypoint.from_waypoint_message(wp)


class GlobalWaypoint:
    """Utility class for representing waypoints in global frame"""
    def __init__(self,lat=None,lon=None,alt=None):
        self.latitude = lat
        self.longitude = lon
        self.altitude = alt

    @classmethod
    def from_waypoint_message(cls,wp):
        """Constructs a new GlobalWaypoint instance from a waypoint message

           Example usage:
           waypoint = GlobalWaypoint.from_waypoint_message(wp)

           Parameters
           wp - a mavros/Waypoint.msg in the global coordinate frame

           Returns
           - A new GlobalWaypoint with coordinates taken from wp
           - None if wp is not in the global coordinate frame
        """

        #**********************************************************************
        #   Don't accept waypoint if its not global.
        #   Return None to indicate failure
        #**********************************************************************
        if msg.Waypoint.FRAME_GLOBAL != wp.frame:
            return None

        #**********************************************************************
        #   Otherwise return a new representation, using mapping defined
        #   in mavros/Waypoint.msg
        #**********************************************************************
        return cls(lat=wp.x, lon=wp.y, alt=wp.z)

    def to_waypoint_message(self):
        """Converts this waypoint into a mavros/Waypoint message

           Returns Waypoint message object specified in global frame.

           Only the x,y,z and frame attributes will be filled in.
           It is the caller's responsiblity to set all other attributes
           if required.
        """

        #**********************************************************************
        #   Fill in frame and position
        #**********************************************************************
        wp = msg.Waypoint()
        wp.frame = msg.Waypoint.FRAME_GLOBAL
        wp.x = self.latitude
        wp.y = self.longitude
        wp.z = self.altitude

        #**********************************************************************
        #   Set other attributes to safe defaults. Worst case, if this
        #   waypoint was used unchanged to control drone, you'd expected to
        #   wait at this waypoint forever (because its effectively unreachable
        #   within 0 radius.
        #**********************************************************************
        wp.autocontinue = False
        wp.radius = 0.0
        wp.waitTime = rospy.Duration(secs=0.0)
        return wp

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
    #   Convert UTM coordinates to global coordinates
    #**************************************************************************
    if isinstance(wp1, UTMWaypoint):
        wp1 = wp1.to_global_waypoint()

    if isinstance(wp2, UTMWaypoint):
        wp2 = wp2.to_global_waypoint()

    #**************************************************************************
    #   Convert parameters if they are waypoint messages
    #   Return None if they are not in GLOBAL frame
    #**************************************************************************
    if isinstance(wp1, msg.Waypoint):
        wp1 = GlobalWaypoint.from_waypoint_message(wp1)
        if wp1 is None:
            return None
        
    if isinstance(wp2, msg.Waypoint):
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
    #**************************************************************************
    #   Convert UTM coordinates to global coordinates
    #**************************************************************************
    if isinstance(wp1, UTMWaypoint):
        wp1 = wp1.to_global_waypoint()

    if isinstance(wp2, UTMWaypoint):
        wp2 = wp2.to_global_waypoint()

    #**************************************************************************
    #   Convert parameters if they are waypoint messages
    #   Return None if they are not in GLOBAL frame
    #**************************************************************************
    if isinstance(wp1, msg.Waypoint):
        wp1 = GlobalWaypoint.from_waypoint_message(wp1)
        if wp1 is None:
            return None
        
    if isinstance(wp2, msg.Waypoint):
        wp2 = GlobalWaypoint.from_waypoint_message(wp2)
        if wp2 is None:
            return None

    #**************************************************************************
    #   Calculate total distance
    #**************************************************************************
    vertical_distance = wp1.altitude-wp2.altitude
    horizontal_distance = distance_along_ground(wp1,wp2)

    # check for error from distance_along_ground function
    if horizontal_distance is None:
        return None

    total = math.sqrt(vertical_distance**2 + horizontal_distance**2)
    return total


