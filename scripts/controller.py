#!/usr/bin/env python
"""
    ROS Node for high-level UAV Control.

    Provides high-level UAV Control functionality via the
    following topics and services

    Services
    --------
    In future, we could implement some of these using the action library,
    but don't feel the extra complexity is warranted right now.

    /uav_name/control/select_camera -- select active camera (AR.Drone only)

    /uav_name/control/set_mode, mavros/SetMode, switch between emergency, auto
        and manual control.

    /uav_name/control/set_origin_here, tells the uav to set the SHARED origin
        for all uavs at its own current position

    /uav_name/control/clear_queue,  clears the waypoint queue
    /uav_name/control/pause_queue, pauses the waypoint queue
    /uav_name/control/resume_queue, resumes the waypoint queue if paused
    /uav_name/control/add_waypoints, adds the specified waypoints to the queue

    /uav_name/control/land, commands the UAV to land
    /uav_name/control/takeoff, commands the UAV to takeoff

    The following services control more complex path plans, and adds
    the resulting waypoints to the queue.
    /uav_name/control/add_sweep, tells the uav to perform sweep search
    /uav_name/control/add_spiral_out, tells the uav to spiral out
    /uav_name/control/add_spiral_in, tells the uav to spiral in

    Publications
    ------------
    /diagnostics - provides status information via the ROS diagnostics package
    /all/control/set_origin - broadcasts new origin for all UAVs (see below)
    /uav_name/manual_control - Used to control UAV velocity in manual mode

    Subscriptions
    -------------
    /uav_name/control/manual_control - when in manual mode, listens for velocity
        messages to control UAV directly

    /uav_name/state - state information published by driver about drone
    /uav_name/filtered_pos - current GPS position published by driver

    The following are used to issue commands to all UAVs at once, but
    not response or acknowledgement is given in response.
    /all/control/takeoff - tell all UAVs to take off
    /all/control/emergency - tell all UAV to enter emergency mode
    /all/control/land - tell all UAVs to land at once
    /all/control/set_origin - tell all UAVs to set specified GPS waypoint as
        origin for local coordinate frame.
"""
import sys
import threading
from optparse import OptionParser
from copy import deepcopy
from utm import from_latlon, to_latlon
import rospy
import diagnostic_updater
import diagnostic_msgs
from std_msgs.msg import Empty as EmptyMsg
from uav_utils import sweeps
import mavros.srv as srv
import mavros.msg as msg
from tools import *

#*******************************************************************************
#   Constants
#*******************************************************************************

# Tolerated difference between floats when testing for equality
FLOAT_DIFF_TOLERANCE = 1.0e-14

# prefix for things subscribed to by all UAV controllers
MULTI_UAV_CONTROL_PREFIX = "/all/control/"

# Time To Live before current position goes stale.
# If we current position isn't updated within this time, we don't trust it
# any longer.
CURRENT_POSITION_TTL = rospy.Duration(secs=1.0)

# Time To Live before target velocity is reset to zero. This is to prevent
# UAV flying away when there is no active Remote Control input
VELOCITY_TTL = rospy.Duration(secs=0.5)

# How often to send RC commands to drone to control its velocity
VELOCITY_UPDATE_RATE_IN_HERTZ = 10.0

# Constants used for conversion between velocities (in range [-1,1]) and
# RC channel inputs
RC_ZERO_POINT = 1500.0  # RC input that indicates zero velocity
RC_COEFFICIENT = 500.0  # 500*velocity+1500 = RC input

# Only the first 4 RC channels are used to control velocity.
# The rest are ignored.
RC_VEL_START = 0  # First RC velocity channel
RC_VEL_END = 4  # Last RC velocity channel + 1

# ROS Parameter namespace in which to look for drone parameters
# We assume that any key-value pairs in this namespace can be directly
# understood by drone using mavlink
DRONE_PARAM_NAMESPACE = "/drone_params"

# ROS Parameter namespace containing list of active drones. If the
# drone that this node is suppose to control is not in this list, the 
# node will terminate on startup.
ACTIVE_DRONE_NAMESPACE = "/drones_active"

# Mavlink parameter keys used to configure drone's safe flight zone
# If synced with drone, the drone should obey them automatically,
# but we use them for internal waypoint verification as an extra precaution
MIN_WAYPOINT_ALTITUDE_PARAM = 'WPT-ALTI-MIN'
MAX_WAYPOINT_ALTITUDE_PARAM = 'WPT-ALTI-MAX'
SAFE_FLIGHT_ZONE_RADIUS_PARAM = 'FLIGHT-ZONE-RAD'

# The following mavling parameters control the active camera on the AR.Drone
# Only one can be 1.0 (i.e active) at any given time. 
USE_BOTTOM_CAMERA_PARAM = 'CAM-RECORD_VERT'  # 1.0 for yes, 0.0 for false
USE_FRONT_CAMERA_PARAM = 'CAM-RECORD_HORI'  # 1.0 for yes, 0.0 for false

# Default mavlink parameters used when none are available from ROS Parameter
# server. Other parameters may be set in parameter server, but require that
# these ones always have sensible values
DEFAULT_MIN_WAYPOINT_ALTITUDE = 0.5  # in metres
DEFAULT_MAX_WAYPOINT_ALTITUDE = 5.0  # in metres
DEFAULT_SAFE_FLIGHT_ZONE_RADIUS = 50.0  # in metres

#*******************************************************************************
#   Classes
#*******************************************************************************
class Controller:
    """Main class --- implements high level control for drones
    """

    def __init__(self,uav_name):
        """Constructs a new Controller

           Parameters
           uav_name - name of UAV this object will control
        """

        # Name of UAV we will control
        self.uav_name = uav_name 

        # ROS namespace prefix for services provided by driver
        self.driver_prefix = '/' + uav_name + '/' 

        # ROS namespace prefix for services provided by this controller
        self.control_prefix = '/' + uav_name + '/control/' 

        # prefix used by this controller for logging messages
        self.log_prefix = "[CONTROL %s] " % uav_name

        # current mode we believe the drone is in
        self.uav_mode = srv.SetModeRequest.UNKNOWN

        # queue of waypoints still to be executed
        self.waypoint_queue = []

        # if true, waypoints are not being executed and uav_mode is AUTO then
        # the drone is put in a hold pattern, until this becomes false
        self.queue_is_paused = True

        # drone's current target waypoint
        self.current_waypoint = 0

        # Next Remote Control message to send during manual mode.
        # This encodes the target velocity, and should be zero when
        # no velocity has been requested within VELOCITY_TTL period.
        self.next_rc = msg.RC()
        self.next_rc.channel = [RC_ZERO_POINT]*len(self.next_rc.channel)

        # Time last time next_rc was updated. If this is more than
        # VELOCITY_TTL ago, we must reset to 0 to prevent drone flying away
        self.next_rc_timestamp = rospy.Time.now()

        # Drone's current position as a tools.GlobalWaypoint object
        # Undefined if we haven't heard its position for more than
        # CURRENT_POSITION_TTL
        self.current_position = None

        # time current position with last updated
        self.current_position_timestamp = rospy.Time.now()

        # origin used to define local cartesian coordinates
        # specified as a tools.UTMWaypoint, and is basically used to replace 
        # the usual origin of the local UTM zone. Undefined until explicitly
        # set by message received on set_origin topic
        self.origin = None

        # Local copy of mavlink parameters that should be set on drone
        # Should be initialised and synced with drone at start up
        self.drone_params = {}  # dictionary of key-value pairs

    def __logfatal(self,msg):
        """Used for logging fatal error messages
           
           Parameters
           msg - string to log
        """
        rospy.logfatal(self.log_prefix + msg)

    def __logerr(self,msg):
        """Used for logging error messages
           
           Parameters
           msg - string to log
        """
        rospy.logerr(self.log_prefix + msg)

    def __logwarn(self,msg):
        """Used for logging warning messages
           
           Parameters
           msg - string to log
        """
        rospy.logwarn(self.log_prefix + msg)

    def __loginfo(self,msg):
        """Used for logging information messages
           
           Parameters
           msg - string to log
        """
        rospy.loginfo(self.log_prefix + msg)

    def __logdebug(self,msg):
        """Used for logging debug messages
           
           Parameters
           msg - string to log
        """
        rospy.loginfo(self.log_prefix + msg)

    def __load_drone_params(self):
        """Loads drone parameters from ROS parameter server
           
           Parameters are stored on ROS parameter server as a single
           dictionary called 'drone_params'. This contains key-value pairs
           for directly loading onto drone using mavlink.

           Defaults are used for parameters required for safe flight
        """
        #**********************************************************************
        #   Load any parameters set on ROS parameter server
        #**********************************************************************
        self.drone_params = rospy.get_param(DRONE_PARAM_NAMESPACE,{})

        #**********************************************************************
        #   If any safe flight zone parameters are undefined, give a warning
        #   and set defaults
        #**********************************************************************
        if not MIN_WAYPOINT_ALTITUDE_PARAM in self.drone_params:
            self.__logwarn("Min Altitude not set -- using default %f" %
                DEFAULT_MIN_WAYPOINT_ALTITUDE)
            self.drone_params[MIN_WAYPOINT_ALTITUDE_PARAM] = \
                DEFAULT_MIN_WAYPOINT_ALTITUDE

        if not MAX_WAYPOINT_ALTITUDE_PARAM in self.drone_params:
            self.__logwarn("Max Altitude not set -- using default %f" %
                DEFAULT_MAX_WAYPOINT_ALTITUDE)
            self.drone_params[MAX_WAYPOINT_ALTITUDE_PARAM] = \
                DEFAULT_MAX_WAYPOINT_ALTITUDE

        if not SAFE_FLIGHT_ZONE_RADIUS_PARAM in self.drone_params:
            self.__logwarn("Safe flight radius not set -- using default %f" %
                DEFAULT_SAFE_FLIGHT_ZONE_RADIUS)
            self.drone_params[SAFE_FLIGHT_ZONE_RADIUS_PARAM] = \
                DEFAULT_SAFE_FLIGHT_ZONE_RADIUS

        return SUCCESS_ERR

    def __sync_params_with_drone(self):
        """Syncs internally stored parameters with drone

           Local parameters overwrite drone parameters.
           However, we keep any parameters on the drone that are not
           defined locally.
        """
        #**********************************************************************
        #   Try to load parameters on drone
        #**********************************************************************
        request = srv.SetParametersRequest()
        request.keys = self.drone_params.keys()
        request.values = self.drone_params.values()
        try:
            response = self.set_params_srv(request)

        except rospy.ServiceException as e:
            self.__logerr("Exception occurred while sending parameters "
                    "to drone %s" % e)
            return SERVICE_CALL_FAILED_ERR

        if SUCCESS_ERR != response.status:
            self.__logerr("Failed to send parameters to drone")
            self.__logdebug("Response is %s" % response.status)
            return response.status

        #**********************************************************************
        #   Try to read them back
        #**********************************************************************
        try:
            response = self.get_params_srv()

        except rospy.ServiceException as e:
            self.__logerr("Exception occurred while getting parameters "
                    "from drone %s" % e)
            return SERVICE_CALL_FAILED_ERR

        if SUCCESS_ERR != response.status:
            self.__logerr("Failed to get parameters to drone")
            self.__logdebug("Response is %s" % response)
            return response.status

        #**********************************************************************
        #   Ensure that any key-value pairs already defined locally have been
        #   set correctly on drone
        #**********************************************************************
        params_from_drone = dict(zip(response.keys,response.values))
        for key,value in self.drone_params.iteritems():

            # ensure parameter is defined on drone
            if not key in params_from_drone:
                self.__logerr("Failed to set parameter %s on drone" % key)
                return PARAM_NOT_SET_ERR

            # compare float values with some tolerance
            elif type(params_from_drone[key]) is float:
                value_err = abs(params_from_drone[key]-value)
                if value_err > FLOAT_DIFF_TOLERANCE:
                    self.__logerr("Parameter %s has intolerable float error "
                            "of %g on drone." % (key, value_err) )
                    return BAD_PARAM_VALUE_ERR

            # compare anything else precisely
            elif value != params_from_drone[key]:
                self.__logerr("Parameter %s has wrong value on drone." % key)
                return BAD_PARAM_VALUE_ERR

        #**********************************************************************
        #   If we're happy, then store any additional parameters stored on
        #   drone locally.
        #**********************************************************************
        self.drone_params = params_from_drone
        return SUCCESS_ERR

    def __current_position_safe(self):
        """Returns true if our current position is unsafe"""

        cur_pos = self.current_position  # convenience handle for position

        #**********************************************************************
        #   Before we start, ensure that mandatory safety parameters are set
        #   If they are not, we have a bug somewhere. They should always have
        #   at least some default value
        #**********************************************************************
        if not MIN_WAYPOINT_ALTITUDE_PARAM in self.drone_params:
            self.__logerr("Internal error - Min waypoint altitude not set.")
            return False

        if not MAX_WAYPOINT_ALTITUDE_PARAM in self.drone_params:
            self.__logerr("Internal error - Max waypoint altitude not set.")
            return False

        if not SAFE_FLIGHT_ZONE_RADIUS_PARAM in self.drone_params:
            self.__logerr("Internal error - safe flight radius not set.")
            return False

        #**********************************************************************
        #   If position is unknown we assume its not safe
        #**********************************************************************
        if cur_pos is None:
            return False

        #**********************************************************************
        #   If position is all zeros, we can pretty sure we don't have a
        #   a GPS lock, so we'll assume this is unsafe
        #**********************************************************************
        if 0.0 == cur_pos.latitude and 0.0 == cur_pos.longitude and \
            0.0 == cur_pos.altitude:

            return False

        #**********************************************************************
        #   If our altitude is outside allowable waypoint range by reasonable
        #   margin of error - that's unsafe
        #**********************************************************************
        if self.drone_params[MIN_WAYPOINT_ALTITUDE_PARAM] > cur_pos.altitude:
            return False

        if self.drone_params[MAX_WAYPOINT_ALTITUDE_PARAM] < cur_pos.altitude:
            return False

        #**********************************************************************
        #   If our position is too far from origin thats unsafe
        #**********************************************************************
        if self.origin is not None:

            dist_from_origin = total_distance(cur_pos,self.origin)

            if self.drone_params[SAFE_FLIGHT_ZONE_RADIUS_PARAM] < \
                dist_from_origin:

                return False

        #**********************************************************************
        #   In all other cases, assume everything is ok
        #**********************************************************************
        return True

    def __valid_waypoint(self,waypoint):
        """Validates a Waypoint message as far as possible

           Parameter:
           waypoint - mavros/Waypoint.msg object

           Returns True if the Waypoint is valid, false otherwise
        """

        #**********************************************************************
        #   Before we start, ensure that mandatory safety parameters are set
        #   If they are not, we have a bug somewhere. They should always have
        #   at least some default value
        #**********************************************************************
        if not MIN_WAYPOINT_ALTITUDE_PARAM in self.drone_params:
            self.__logerr("Internal error - Min waypoint altitude not set.")
            return False

        if not MAX_WAYPOINT_ALTITUDE_PARAM in self.drone_params:
            self.__logerr("Internal error - Max waypoint altitude not set.")
            return False

        if not SAFE_FLIGHT_ZONE_RADIUS_PARAM in self.drone_params:
            self.__logerr("Internal error - safe flight radius not set.")
            return False

        #**********************************************************************
        #   Validate latitude and longitude for Global waypoints
        #**********************************************************************
        if msg.Waypoint.FRAME_GLOBAL == waypoint.frame:
            
            # validate latitude (stored in coordinate x)
            if MIN_VALID_LATITUDE > waypoint.x:
                self.__logerr("global waypoint %s has out of range latitude" %
                    waypoint)
                return False

            if MAX_VALID_LATITUDE < waypoint.x:
                self.__logerr("global waypoint %s has out of range latitude" %
                    waypoint)
                return False

            # validate longitude (stored in coordinate y)
            if MIN_VALID_LONGITUDE > waypoint.y:
                self.__logerr("global waypoint %s has out of range longitude" %
                    waypoint)
                return False

            if MAX_VALID_LONGITUDE < waypoint.y:
                self.__logerr("global waypoint %s has out of range longitude" %
                    waypoint)
                return False

        #**********************************************************************
        #   Only allow LOCAL waypoints if our origin is set
        #**********************************************************************
        elif msg.Waypoint.FRAME_LOCAL == waypoint.frame:

            if self.origin is None:
                self.__logerr("local waypoint %s is invalid because origin "
                        "is not set." % waypoint)
                return False

        #**********************************************************************
        #   Disallow waypoints in any unsupported or undefined frame
        #**********************************************************************
        else:
            self.__logerr("waypoint %s has unknown or undefined frame" %
                    waypoint)
            return False

        #**********************************************************************
        #   Range check altitude (stored in z coordinate)
        #**********************************************************************
        if self.drone_params[MIN_WAYPOINT_ALTITUDE_PARAM] > waypoint.z:
            self.__logerr("waypoint %s altitude is too low" %
                    waypoint)
            return False

        if self.drone_params[MAX_WAYPOINT_ALTITUDE_PARAM] < waypoint.z:
            self.__logerr("waypoint %s altitude is too high" %
                    waypoint)
            return False

        #**********************************************************************
        #   Ensure we're a safe distance from our current position
        #**********************************************************************
        safe_distance = self.drone_params[SAFE_FLIGHT_ZONE_RADIUS_PARAM]
        if safe_distance < total_distance(self.current_position,waypoint):
            self.__logerr("waypoint %s is too far from current position" %
                    waypoint)
            return False

        #**********************************************************************
        #   If possible, ensure we're a safe distance from our origin
        #**********************************************************************
        if self.origin is not None:
            if safe_distance < total_distance(self.origin,waypoint):
                self.__logerr("waypoint %s is too far from origin" %
                    waypoint)
                return False

        #**********************************************************************
        #   If we get this far, assume the waypoint is valid
        #**********************************************************************
        return True

    def __are_waypoints_equivalent(self,wp1,wp2,tol=1.0):
        """Compares two waypoints to see if they are equivalent.

           Compares two waypoints to see if they are equivalent within margin
           of error. Local frame coordinates are first converted to global
           frame before comparision. 

           Parameters
           wp1 - the 1st waypoint as a mavros/Waypoint.msg object
           wp2 - the 2nd waypoint as a mavros/Waypoint.msg object
           tol - tolerance (in metres) for comparision [default: 1.0]

           Returns True if distance between waypoints is within tolerance.
           Note that altitude differences are taking into account during
           comparison.
        """

        #**********************************************************************
        #   Put waypoints in GLOBAL Coordinates frame
        #   Required by distance functions
        #**********************************************************************
        wp1 = self.__get_global_waypoint(wp1)
        wp2 = self.__get_global_waypoint(wp2)

        #**********************************************************************
        #   Calculate total distance between points
        #**********************************************************************
        distance = total_distance(wp1,wp2)
        if distance is None:
            self.__logerr("Internal error calculating distance between "
                    "waypoints. This shouldn't happen.")

        #**********************************************************************
        #   Return true if waypoints are equal within tolerance
        #**********************************************************************
        return distance < tol

    def __get_global_waypoint(self,wp):
        """Puts waypoints into global coordinate frame

           Local frame waypoints are in UTM offset by local origin.

           Parameter:
           wp - a mavros/Waypoint message in either the local or global
               coordinate frame.

           Returns the equivalent waypoint in the global frame (i.e. no change
               if already in the correct frame)

           Returns None if waypoint is not in recognised frame, or if local
           origin is not set.
        """

        #**********************************************************************
        #   If the local origin isn't set, we won't be able to do any
        #   conversion. Give a warning at this point, but then try our best
        #**********************************************************************
        if self.origin is None:
            self.__logwarn("Local origin isn't set. We won't be able to"
                    " to convert local coordinates if there are any.")

        #**********************************************************************
        #   If waypoint is already in global frame, then we're done
        #**********************************************************************
        if msg.Waypoint.FRAME_GLOBAL == wp.frame:
            return wp

        #**********************************************************************
        #  If its a local waypoint, convert to global frame
        #  Note mapping defined in mavros/Waypoint.msg is:
        #   x -- latitude
        #   y -- longitude
        #   z -- altitude (no change required)
        #**********************************************************************
        elif msg.Waypoint.FRAME_LOCAL == wp.frame:

            # abort if the local origin isn't set
            if self.origin is None:
                self.__logerr("Trying to convert local waypoint without "
                        " defined origin. Operation aborted.")
                return None

            # deep copy waypoint so we don't change original
            result = deepcopy(wp)

            # convert to UTM coordinates offset by our local origin
            utm_easting = wp.x + self.origin.easting
            utm_northing = wp.y + self.origin.northing

            # put latitude and longitude in result
            result.x, result.y = to_latlon(utm_easting, utm_northing,
                    self.origin.zone_number, self.origin.zone_letter)

            # set coordinate frame to GLOBAL in the result
            result.frame = msg.Waypoint.FRAME_GLOBAL

            return result

        #**********************************************************************
        #   Otherwise, the waypoint frame is not recognised --- something
        #   is wrong somewhere!
        #**********************************************************************
        else:
            self.__logerr("Unrecognised waypoint frame %d" % wp.frame)
            return None

    def __add_sweep_points(self,points,req):
        """Utility function for adding sweep search points to the queue

           Provides common implementation for sweep search callback functions.

           Parameters
           points - list of point pairs in form (eastings,northings) generated
                    by one of the uav_utils sweep functions.
           req - original sweep search request of type srv.AddSweepRequest
        """

        #**********************************************************************
        #   Convert points into waypoint messages
        #**********************************************************************
        waypoints = []
        for p in points:
            wp = msg.Waypoint()
            wp.frame = msg.Waypoint.FRAME_LOCAL
            wp.autocontinue = True
            wp.radius = req.radius
            wp.waitTime = req.waitTime
            wp.x = p[0]  # easting
            wp.y = p[1]  # northing
            wp.z = req.altitude
            waypoints.append(wp)

        #***********************************************************************
        #   Delegate the rest to the the add waypoints callback
        #***********************************************************************
        request = srv.AddWaypointsRequest()
        request.waypoints = waypoints
        return add_waypoints_cb(request)

    def __set_waypoints_from_queue(self):
        """Syncs the waypoints on the drone with the currently queued waypoints
           and restarts execution if the queue is not currently paused.
        """

        #**********************************************************************
        #   Not sure how drone will behave if its currently following
        #   waypoints, so as a precaution, tell it to stop what its doing
        #   before sending waypoints.
        #**********************************************************************
        if self.uav_mode == srv.SetModeRequest.AUTO:
            halt_response = self.__halt_drone()
            if SUCCESS_ERR != halt_response:
                self.__logerr("Failed to halt drone before sending "
                        "waypoints.")
                return halt_response
        else:
            self.__logerr("Can't send waypoints unless drone is in AUTO"
                    " mode. Please set mode to AUTO.")
            return UNSUPPORTED_COMMAND_ERR

        #**********************************************************************
        #   Get a copy of all waypoints in the queue without removing them.
        #**********************************************************************
        waypoint_list = list(self.waypoint_queue)

        #**********************************************************************
        # If any waypoints are in the local coordinate frame, convert them
        # to the global coordinate frame now.
        #**********************************************************************
        for i in range(len(waypoint_list)):
            waypoint_list[i] = self.__get_global_waypoint(waypoint_list[i])

        #**********************************************************************
        #   Try to send the waypoints to the drone
        #**********************************************************************
        try: 
            response = self.set_waypoints_srv(waypoint_list)
            if SUCCESS_ERR != response.status:
                self.__logerr("Failed to send waypoints to drone")
                return response.status

        except rospy.ServiceException as e:
            self.__logerr("Exception occurred while sending waypoints "
                    "to drone: %s" % e)
            return SERVICE_CALL_FAILED_ERR

        #**********************************************************************
        #   Pull the waypoints back of the drone and verify that they
        #   are correct
        #**********************************************************************
        # Get waypoints
        try:
            response = self.get_waypoints_srv()
            if SUCCESS_ERR != response.status:
                self.__logerr("Failed to retrieve waypoints from drone for"
                        " verification")
                return response.status

        except rospy.ServiceException as e:
            self.__logerr("Exception occurred while getting waypoints "
                    "from drone for verification: %s" % e)
            return SERVICE_CALL_FAILED_ERR

        # Validate number of waypoints
        if len(response.waypoints) != len(waypoint_list):
            self.__logerr("Wrong number of waypoints on drone after"
                    " update attempt")
            return WAYPOINT_VERIFICATION_FAILURE_ERR

        # Validate each waypoint in turn
        for k in range(len(waypoint_list)):
            drone_wp = response.waypoints[k]
            my_wp = waypoint_list[k]
            if not self.__are_waypoints_equivalent(drone_wp,my_wp):
                self.__logerr("Waypoint %d on drone is not correct after "
                        "attempted update" % k)
                return WAYPOINT_VERIFICATION_FAILURE_ERR

        self.__loginfo("Waypoints on drone updated and verified successfully")

        #**********************************************************************
        #   If queue is not paused, ask drone to start following waypoints
        #**********************************************************************
        if self.queue_is_paused:
            self.__loginfo("Queue is currently paused")
            return SUCCESS_ERR

        next_wp = 0  # always waypoint 0 after update
        return self.__execute_mission_on_drone(next_wp)
            
    def __execute_mission_on_drone(self,wp_number):
        """Asks the drone start mission from specified waypoint number

           Returns SUCCESS_ERR if successful
        """

        #**********************************************************************
        #   Make sure we're in AUTO mode
        #**********************************************************************
        if self.uav_mode != srv.SetModeRequest.AUTO:
            self.__logerr("Can't execute mission on drone unless we know "
                    "its in AUTO mode. Please set mode explicitly first.")
            return UNSUPPORTED_COMMAND_ERR

        #**********************************************************************
        #   Try to set the current mission to the specified waypoint
        #**********************************************************************
        try:
            response = self.set_mission_srv(wp_number)
            if SUCCESS_ERR != response.status:
                self.__logerr("Could not set current mission on drone")
                return response.status

        except rospy.ServiceException as e:
            self.__logerr("Exception occurred setting mission "
                    "on drone: %s" % e)
            return SERVICE_CALL_FAILED_ERR

        #**********************************************************************
        #   Try get the drone to resume execution
        #**********************************************************************
        cmdRequest = srv.MAVCommandRequest()
        cmdRequest.command = srv.MAVCommandRequest.CMD_RESUME
        cmdRequest.custom = srv.MAVCommandRequest.CUSTOM_NO_OP
        try:
            response = self.mav_command_srv(cmdRequest)
            if SUCCESS_ERR != response.status:
                self.__logerr("Could not start waypoint following on drone")
            return response.status

        except rospy.ServiceException as e:
            self.__logerr("MAVCommand service threw exception while trying to"
                    " resume waypoint execution: %s" % e)
            return SERVICE_CALL_FAILED_ERR
            
    def __halt_drone(self):
        """Asks the drone to loiter if in AUTO mode.

           Returns SUCCESS_ERR if successful
        """

        #**********************************************************************
        #   Make sure we're in AUTO mode
        #**********************************************************************
        if self.uav_mode != srv.SetModeRequest.AUTO:
            self.__logwarn("Can't halt drone unless we know its in AUTO mode."
                    " Please set mode explicitly first.")
            return UNSUPPORTED_COMMAND_ERR

        #**********************************************************************
        #   Try to halt the drone and return result status
        #**********************************************************************
        request = srv.MAVCommandRequest()
        request.command = srv.MAVCommandRequest.CMD_HALT
        request.custom = srv.MAVCommandRequest.CUSTOM_NO_OP
        try:
            response = self.mav_command_srv(request)
            if SUCCESS_ERR != response.status:
                self.__logerr("Could not halt drone")
            else:
                self.__loginfo("Halted drone.")
            return response.status

        except rospy.ServiceException as e:
            self.__logerr("MAVCommand service threw exception while trying to"
                    " halt drone: %s" % e)
            return SERVICE_CALL_FAILED_ERR

    def __ros_init(self):
        """Initialises ROS services, publications and subscriptions"""

        #***********************************************************************
        #   Wait for mavros driver to initialise
        #***********************************************************************
        self.__loginfo("Waiting for driver services")
        rospy.wait_for_service(self.driver_prefix + "mav_command")
        rospy.wait_for_service(self.driver_prefix + "set_waypoints")
        rospy.wait_for_service(self.driver_prefix + "get_waypoints")
        rospy.wait_for_service(self.driver_prefix + "set_params")
        rospy.wait_for_service(self.driver_prefix + "get_params")
        rospy.wait_for_service(self.driver_prefix + "set_mission")
        self.__loginfo("Initialising high-level control")

        #**********************************************************************
        #   Initialise proxy functions for remote services we call.
        #   These are all provided by the mavros driver node 
        #**********************************************************************
        self.mav_command_srv = rospy.ServiceProxy(self.driver_prefix +
            "mav_command", srv.MAVCommand)

        self.set_waypoints_srv = rospy.ServiceProxy(self.driver_prefix +
            "set_waypoints", srv.SetWaypoints)

        self.get_waypoints_srv = rospy.ServiceProxy(self.driver_prefix +
            "get_waypoints", srv.GetWaypoints)

        self.set_params_srv = rospy.ServiceProxy(self.driver_prefix +
            "set_params", srv.SetParameters)

        self.get_params_srv = rospy.ServiceProxy(self.driver_prefix +
            "get_params", srv.GetParameters)

        self.set_mission_srv = rospy.ServiceProxy(self.driver_prefix +
            "set_mission", srv.SetMission)

        #**********************************************************************
        #   Initialise ROS services we provide
        #**********************************************************************
        rospy.Service(self.control_prefix + "select_camera", srv.SelectCamera,
            self.select_camera_cb)

        rospy.Service(self.control_prefix + "set_mode", srv.SetMode,
            self.set_mode_cb)

        rospy.Service(self.control_prefix + "set_origin_here",
            srv.SimpleCommand, self.set_origin_here_cb)

        rospy.Service(self.control_prefix + "clear_queue", srv.SimpleCommand,
            self.clear_queue_cb)

        rospy.Service(self.control_prefix + "pause_queue", srv.SimpleCommand,
            self.pause_queue_cb)

        rospy.Service(self.control_prefix + "resume_queue", srv.SimpleCommand,
            self.resume_queue_cb)

        rospy.Service(self.control_prefix + "land", srv.SimpleCommand,
            self.land_cb)

        rospy.Service(self.control_prefix + "takeoff", srv.SimpleCommand,
            self.takeoff_cb)

        rospy.Service(self.control_prefix + "add_waypoints", srv.AddWaypoints,
            self.add_waypoints_cb)

        rospy.Service(self.control_prefix + "add_sweep", srv.AddSweep,
            self.add_sweep_cb)

        rospy.Service(self.control_prefix + "add_spiral_out", srv.AddSweep,
            self.add_spiral_out_cb)

        rospy.Service(self.control_prefix + "add_spiral_in", srv.AddSweep,
            self.add_spiral_in_cb)

        #**********************************************************************
        #   Register call back functions to for topics we subscribe to
        #**********************************************************************
        rospy.Subscriber(self.control_prefix + "manual_control", msg.Velocity,
            self.manual_control_cb)

        rospy.Subscriber(MULTI_UAV_CONTROL_PREFIX + "takeoff", EmptyMsg,
            self.takeoff_cb)

        rospy.Subscriber(MULTI_UAV_CONTROL_PREFIX + "land", EmptyMsg,
            self.land_cb)

        rospy.Subscriber(MULTI_UAV_CONTROL_PREFIX + "emergency",
            EmptyMsg, self.emergency_cb)

        rospy.Subscriber(MULTI_UAV_CONTROL_PREFIX + "set_origin", msg.Waypoint,
            self.set_origin_cb)

        rospy.Subscriber(self.driver_prefix + "state", msg.State, 
            self.driver_state_cb)

        rospy.Subscriber(self.driver_prefix + "filtered_pos",
            msg.FilteredPosition, self.filtered_position_cb)

        #**********************************************************************
        #   Advertise messages that we publish. 
        #   Note: don't need to advertise diagnostic messages explicitly.
        #   That's done by diagnostics updater.
        #**********************************************************************
        self.pub_origin = rospy.Publisher(MULTI_UAV_CONTROL_PREFIX + \
            "set_origin", msg.Waypoint, queue_size=1)

        self.pub_rc = rospy.Publisher(self.driver_prefix + \
            "manual_control", msg.RC, queue_size=1)

        #**********************************************************************
        #   Setup ROS diagnostics updater
        #**********************************************************************
        self.diag_updater = diagnostic_updater.Updater()
        self.diag_updater.setHardwareID("%s" % self.uav_name)
        self.diag_updater.add("controller",self.produce_diagnostics)

    def produce_diagnostics(self,status):
        """Used to produce ROS diagnostics about current controller state"""

        #**********************************************************************
        #   Initialise summary string and error level. These determine
        #   how diagnostics appear at top level of ROS runtime monitor GUI
        #
        #   Note: we assume that more serious error levels have higher
        #   integer values, so that we can use the maximum value of two
        #   levels to choose the most serious one
        #**********************************************************************
        summary_string = ""  # we'll add to this later
        error_level = DIAG_OK  # initiall assume everything is ok

        #**********************************************************************
        #   Generate diagnostics about origin
        #**********************************************************************
        is_origin_set = not (self.origin is None)
        if is_origin_set:
            origin_status = "SET"
        else:
            origin_status = "NOT SET"
            error_level = max(error_level,DIAG_WARN)
            summary_string = summary_string + "ORIGIN NOT SET. "

        status.add("Origin set", origin_status)

        if is_origin_set:
            global_origin = self.origin.to_global_waypoint()
            status.add("origin latitude",global_origin.latitude)
            status.add("origin longitude",global_origin.longitude)
            status.add("origin altitude",global_origin.altitude)
        else:
            status.add("origin latitude",None)
            status.add("origin longitude",None)
            status.add("origin altitude",None)

        #**********************************************************************
        #   Calculate diagnostics about current position
        #**********************************************************************
        is_pos_stale = rospy.Time.now()-self.current_position_timestamp > \
                       CURRENT_POSITION_TTL
        is_pos_set = not (self.current_position is None)

        if not is_pos_set:
            error_level = max(error_level,DIAG_WARN)
            summary_string = summary_string + "POSITION UNKNOWN. "
            pos_status = "Not known"
        elif is_pos_stale:
            error_level = max(error_level,DIAG_ERROR)
            pos_status = "stale"
        else:
            pos_error_level = DIAG_OK
            pos_status = "Up to date"

        status.add("Position status", pos_status)

        if is_pos_set:
            status.add("current latitude",self.current_position.latitude)
            status.add("current longitude",self.current_position.longitude)
            status.add("current altitude",self.current_position.altitude)
        else:
            status.add("current latitude",None)
            status.add("current longitude",None)
            status.add("current altitude",None)

        #**********************************************************************
        #   If possible, calculate distance from origin
        #**********************************************************************
        if is_pos_set and is_origin_set:
            pos = self.current_position
            origin = self.origin.to_global_waypoint()
            distance_from_origin = total_distance(pos,origin)
        else:
            distance_from_origin = "UNKNOWN"
        status.add("Distance from origin", distance_from_origin)

        #**********************************************************************
        #   Give a warning if we're outside safe flight zone
        #**********************************************************************
        if not self.__current_position_safe():
            summary_string = summary_string + "UNSAFE POSITION. "
            error_level = max(error_level,DIAG_WARN)

        #**********************************************************************
        #   Generate useful diagnostics about current UAV Mode
        #**********************************************************************
        if srv.SetModeRequest.UNKNOWN == self.uav_mode:  
            mode_status = "UNKNOWN"
            summary_string = summary_string + "UAV mode unknown. "
            error_level = max(error_level,DIAG_WARN)

        elif srv.SetModeRequest.AUTO == self.uav_mode:
            mode_status = "AUTO"
            summary_string = summary_string + "UAV in AUTO. "

        elif srv.SetModeRequest.MANUAL == self.uav_mode:
            mode_status = "MANUAL"
            summary_string = summary_string + "UAV in MANUAL. "

        elif srv.SetModeRequest.EMERGENCY == self.uav_mode:
            mode_status = "EMERGENCY"
            summary_string = summary_string + "UAV in EMERGENCY! "
            error_level = max(error_level,DIAG_WARN)

        else:
            summary_string = summary_string + "UAV mode undefined! "
            error_level = max(error_level,DIAG_ERROR)
            mode_status = "UNDEFINED"

        status.add("Controller state", mode_status)

        #**********************************************************************
        #   Fill in useful details about the queue
        #**********************************************************************
        status.add("Waypoint queue paused", self.queue_is_paused)
        status.add("Waypoints queued", len(self.waypoint_queue) )
        status.add("Current waypoint", self.current_waypoint)

        #**********************************************************************
        #   Fill in summary and report error conditions
        #**********************************************************************
        status.summary(error_level, summary_string)
        return status

    def update_diagnostics(self, event=None):
        """Used to periodically update ROS diagnostics"""

        self.diag_updater.update()

    def filtered_position_cb(self,msg):
        """Callback for drone's filtered position, received from driver"""

        #**********************************************************************
        #   Use value to update our copy of the current position
        #**********************************************************************
        if self.current_position is None:
            self.current_position = GlobalWaypoint()

        self.current_position.latitude = msg.latitude
        self.current_position.longitude = msg.longitude
        self.current_position.altitude = msg.relative_altitude

        #**********************************************************************
        #   Also update the timestamp, so we know if the value has gone stale.
        #**********************************************************************
        self.current_position_timestamp = msg.header.stamp

    def driver_state_cb(self,msg):
        """Callback for driver state messages

           Used to figure out when waypoints have been completed. Ideally this
           would actually be achieved used mavlink MISSION_REACHED messages,
           but its not clear if the AR.Drone actually sends these.
        """

        #***********************************************************************
        #   Calculate number of waypoints completed since last state message
        #   received.
        #   Note: important that we update our copy of the current
        #   waypoint before exiting successfully from this function.
        #***********************************************************************
        newly_completed = msg.current_waypoint - self.current_waypoint

        #***********************************************************************
        #   If no new waypoints have been completed, then there is nothing
        #   to update or do
        #***********************************************************************
        if 0 == newly_completed:
            self.__logdebug("driver state indicates no new waypoints"
                    " completed. Nothing to update.")
            return

        #***********************************************************************
        #   If the current waypoint id is smaller than the last one, this
        #   would imply that waypoints are not being followed in sequential
        #   order. Thus, we pause the queue and ask the user to sort out the
        #   mess.
        #***********************************************************************
        if 0 > newly_completed:
            self.__logerr("Drone appears to be going backwards along "
                    "waypoint queue. Is some other process controlling the"
                    " drone?")
            self.__logwarn("Pausing Queue until issue resolved.")
            self.pause_queue_cb()
            return

        #***********************************************************************
        #   If we've allegedly completed more waypoints than we have queued,
        #   then something is probably wrong
        #***********************************************************************
        if self.queue.qsize() <= newly_completed:
            self.__logerr("Drone appears to have completed more waypoints "
                    " than we told it to do. Either some other process is "
                    " controlling the drone, or we have bug.")
            self.__logwarn("Pausing Queue until issue resolved.")
            self.pause_queue_cb()
            return

        #***********************************************************************
        #   If we get this far, then at least one new waypoint has been
        #   executed. This shouldn't happen on a paused queue.
        #***********************************************************************
        if self.queue_is_paused:
            self.__logerr("Drone appears to have completed %d waypoints, even"
                    " though queue is paused. Attempting to pause queue again")
            self.pause_queue_cb()
            return

        #***********************************************************************
        #   If we've completed more than one waypoint, that might be ok,
        #   but again, sounds a bit iffy. Log a warning anyway.
        #***********************************************************************
        if 1 < newly_completed:
            self.__logwarn("Current waypoint is %d ahead of last one. Did we "
                    " miss something?" % newly_completed)

        #***********************************************************************
        #   Remove completed waypoints from the queue, and update our copy of
        #   the current target waypoint id
        #***********************************************************************
        self.current_waypoint = msg.current_waypoint
        del self.waypoint_queue[0:newly_completed] 
        self.__loginfo("%d waypoints dequeued" % newly_completed)
        return

    def select_camera_cb(self,req):
        """Callback for mavros/SelectCamera service

           Asks drone to select specified camera. Only makes since for
           AR.Drones.

           Parameters
           req - mavros/SelectCamera.srv request

           See service definition for details.
        """
        #***********************************************************************
        #   Check to see if camera parameters are present. If not, we
        #   may not be dealing with an AR.Drone
        #***********************************************************************
        if (USE_BOTTOM_CAMERA_PARAM not in self.drone_params) or \
            (USE_FRONT_CAMERA_PARAM not in self.drone_params):

            self.__logwarn("Can't select camera because camera parameters are"
                    " not present. Perhaps UAV is not an AR.Drone?")
            return UNSUPPORTED_COMMAND_ERR

        #***********************************************************************
        #   Select front camera on request
        #***********************************************************************
        if srv.SelectCameraRequest.FRONT == req.camera:

            self.drone_params[USE_BOTTOM_CAMERA_PARAM] = 0.0
            self.drone_params[USE_FRONT_CAMERA_PARAM]  = 1.0

        #***********************************************************************
        #   Select bottom camera on request
        #***********************************************************************
        elif srv.SelectCameraRequest.BOTTOM == req.camera:

            self.drone_params[USE_BOTTOM_CAMERA_PARAM] = 1.0
            self.drone_params[USE_FRONT_CAMERA_PARAM]  = 0.0

        #***********************************************************************
        #   Ignore requests for any unknown or undefined camera
        #***********************************************************************
        else:
            self.__logwarn("Ignoring request to switch to unknown camera %d" %
                    req.camera)
            return UNKNOWN_CAMERA_ERR

        #***********************************************************************
        #   Try to sync updated camera parameters with drone
        #***********************************************************************
        status = self.__sync_params_with_drone()
        if SUCCESS_ERR != status:
            self.__logerr("Failed to sync camera parameters with drone.")
        return status

    def set_mode_cb(self,req):
        """Callback for mavros/SetMode service
           Asks drone to enter specified control mode.
        """

        #***********************************************************************
        #   Refuse requests to enter UNKNOWN mode. Important this happens
        #   before comparing against current mode --- which is initialised
        #   to UNKNOWN, but should never be set to UNKNOWN on request
        #***********************************************************************
        if srv.SetModeRequest.UNKNOWN == req.mode:
            self.__logwarn("Ignoring request to enter UNKNOWN control mode")
            return

        #***********************************************************************
        #   If no change in mode is requested, then there is nothing to do
        #***********************************************************************
        if self.uav_mode == req.mode:
            self.__loginfo("Drone is already in requested mode: %d" % req.mode)
            return

        #***********************************************************************
        #   Ask drone to enter EMERGENCY mode (implemented as custom mavlink
        #   mode).
        #***********************************************************************
        if req.mode == srv.SetModeRequest.EMERGENCY:
            return self.emergency_cb()  # delegate to emergency callback

        #***********************************************************************
        #   Ask drone to enter MANUAL mode
        #***********************************************************************
        if req.mode == srv.SetModeRequest.MANUAL:

            # As precaution, reset target velocity to zero now, to ensure
            # UAV doesn't fly off
            self.next_rc.channel = [RC_ZERO_POINT]*len(self.next_rc.channel)
            self.next_rc_timestamp = rospy.Time.now()

            # Construct request
            request = srv.MAVCommandRequest()
            request.command = srv.MAVCommandRequest.CMD_MANUAL
            request.custom = srv.MAVCommandRequest.CUSTOM_NO_OP

            # try to send request
            try:
                response = self.mav_command_srv(request)
            except rospy.ServiceException as e:
                self.__logerr("MAVCommand service threw exception while trying"
                    " to set mode to MANUAL: %s" % e)
                return SERVICE_CALL_FAILED_ERR

            # if we get that far, return error status from driver
            if SUCCESS_ERR == response.status:
                self.uav_mode = srv.SetModeRequest.MANUAL
            else:
                self.__logerr("Drone failed to enter manual mode")
            return response.status

        #***********************************************************************
        #   Ask drone to enter AUTO mode
        #***********************************************************************
        if req.mode == srv.SetModeRequest.AUTO:
            request = srv.MAVCommandRequest()
            request.command = srv.MAVCommandRequest.CMD_AUTO
            request.custom = srv.MAVCommandRequest.CUSTOM_NO_OP

            try:
                response = self.mav_command_srv(request)
            except rospy.ServiceException as e:
                self.__logerr("MAVCommand service threw exception while trying"
                    " to set mode to AUTO: %s" % e)
                return SERVICE_CALL_FAILED_ERR

            if SUCCESS_ERR == response.status:
                self.uav_mode = srv.SetModeRequest.AUTO
            else:
                self.__logerr("Drone failed to enter auto mode")
            return response.status

        #***********************************************************************
        #   If we get this far --- command type is not recognised
        #***********************************************************************
        self.__logerr("Requested unrecognised mode - ignoring")
        return UNDEFINED_COMMAND_ERR

    def set_origin_here_cb(self,req=None):
        """Tells the UAV to set the origin for ALL UAVs.

           The origin for all UAVs will be set to the current location of this
           UAV.
        """

        #***********************************************************************
        #   If our current position is unknown, or has gone stale,
        #   then we can't set our position.
        #***********************************************************************
        pos = self.current_position # for convenience
        pos_stamp = self.current_position_timestamp
        if pos is None:
            self.__logwarn("Ignoring request to set origin at current position"
                    " because our position isn't currently known")
            return NO_GPS_FIX_ERR

        if rospy.Time.now()-pos_stamp > CURRENT_POSITION_TTL:
            self.__logwarn("Ignoring request to set origin at current position"
                    " because our last known position has gone stale.")
            return NO_GPS_FIX_ERR

        #**********************************************************************
        #   If current position is exactly zero then we probably don't
        #   have a GPS fix really - best abort
        #**********************************************************************
        if 0==pos.latitude and 0==pos.altitude and 0==pos.longitude:
            self.__logwarn("Ignoring request to set origin. Current position "
                    " is 0,0,0 - most likely because there is no GPS fix.")
            return NO_GPS_FIX_ERR

        #***********************************************************************
        #   Publish current origin on shared /set_origin topic
        #
        #   This should not only set everyone else's origin, it should
        #   also result in our own origin being set by invoking our own
        #   callback on this topic.
        #***********************************************************************
        msg = pos.to_waypoint_message()
        self.__logdebug("Sending new origin: %s" % msg)
        self.pub_origin.publish(msg)
        return SUCCESS_ERR

    def set_origin_cb(self,origin_wp):
        """Accepts a new origin for the local coordinate frame for this UAV"""

        #***********************************************************************
        #   Ensure received waypoint is in global frame
        #***********************************************************************
        if msg.Waypoint.FRAME_GLOBAL != origin_wp.frame:
            self.__logwarn("Ignoring received origin because its not in "
                    "global frame.")
            return UNSUPPORTED_FRAME_ERR

        #***********************************************************************
        #   Put in cartesian space by converting to local UTM point
        #   Use this value to set the origin
        #***********************************************************************
        self.origin = UTMWaypoint.from_waypoint_message(origin_wp)
        if self.origin is None:
            self.__logerror("Internal error. Origin could not be set.")
            return INTERNAL_ERR

        return SUCCESS_ERR

    def clear_queue_cb(self,req=None):
        """Callback for clearing the queue

           Parameters
           req - is an empty service request (no input required)

           Returns
           mavros/Error message indicating success or failure
        """
        #***********************************************************************
        #   As precaution, start by pausing queue and its execution on the
        #   drone.
        #***********************************************************************
        status = self.pause_queue_cb()
        if SUCCESS_ERR != status:
            self.__logerr("Failed to pause queue, prior to clearing "
                    " - error code: %d" % status)

        #***********************************************************************
        #   Clear internal queue structure
        #***********************************************************************
        del self.waypoint_queue[:]

        #***********************************************************************
        #   Try to clear all waypoints stored on the drone
        #***********************************************************************
        request = srv.MAVCommandRequest()
        request.command = srv.MAVCommandRequest.CMD_CLEAR_WAYPOINTS
        request.custom = srv.MAVCommandRequest.CUSTOM_NO_OP

        try:
            response = self.mav_command_srv(request)
            status = response.status
        except rospy.ServiceException as e:
            self.__logerr("MAVCommand service threw exception while trying"
                " to clear waypoints: %s" % e)
            status = SERVICE_CALL_FAILED_ERR

        if SUCCESS_ERR != status:
            self.__logerr("Could not clear waypoints on drone")
            self.__logwarn("Waypoints cleared but may be out of sync with"
                    " drone.")
        else:
            self.__loginfo("Waypoint queue cleared and synced with drone.")
        return status

    def pause_queue_cb(self,req=None):
        """Callback for pausing execution of the queue

           Parameters
           req - is an empty service request (no input required)

           Returns
           mavros/Error message indicating success or failure
        """
        #***********************************************************************
        #   Flag internal queue as paused
        #***********************************************************************
        self.queue_is_paused = True
        self.__loginfo("Waypoint queue is now paused.")

        #***********************************************************************
        #   If the drone is in AUTO mode --- ask it to stop execution
        #***********************************************************************
        status = SUCCESS_ERR  # error flag to return to caller
        if srv.SetModeRequest.AUTO == self.uav_mode:
            self.__loginfo("Trying to halt drone.")
            status = self.__halt_drone()
            if SUCCESS_ERR != status:
                self.__logerr("Failed to halt drone after pausing queue.")
        else:
            self.__loginfo("Drone not in AUTO mode, so will not be halted"
                    " following queue pause.")

        return status

    def resume_queue_cb(self,req=None):
        """Callback for resuming execution of the queue

           Parameters
           req - is an empty service request (no input required)

           Returns
           mavros/Error message indicating if the command was sent successfully
        """

        #***********************************************************************
        #   Only really meaningful to resume queue if drone is in AUTO mode
        #***********************************************************************
        if srv.SetModeRequest.AUTO != self.uav_mode:
            self.__logwarn("Can only resume queue while drone is in AUTO mode")
            return UNSUPPORTED_COMMAND_ERR

        #***********************************************************************
        #   To be safe, resync waypoints on drone from queue
        #***********************************************************************
        status = self.__set_waypoints_from_queue()
        if SUCCESS_ERR != status:
            self.__logerr("Waypoints could not be synced with drone.")
            return status
        else:
            self.__loginfo("Waypoints synced with drone.")

        #***********************************************************************
        #   Try to start execution from next waypoint
        #***********************************************************************
        self.queue_is_paused = False
        next_wp = 0  # always waypoint 0 call to set_waypoints_from_queue
        status = self.__execute_mission_on_drone(next_wp)
        if SUCCESS_ERR != status:
            self.queue_is_paused = True
            self.__logerr("Could not execute mission on drone. Pausing Queue.")
        return status

    def land_cb(self,req=None):
        """Callback for landing the drone

           Parameters
           req - is an empty service request (no input required)

           Returns
           mavros/Error message indicating if the command was sent successfully
        """
        request = srv.MAVCommandRequest()
        request.command = srv.MAVCommandRequest.CMD_LAND
        request.custom = srv.MAVCommandRequest.CUSTOM_NO_OP

        try:
            response = self.mav_command_srv(request)
        except rospy.ServiceException as e:
            self.__logerr("MAVCommand service threw exception while trying"
                " to land drone: %s" % e)
            return SERVICE_CALL_FAILED_ERR

        if SUCCESS_ERR != response.status:
           self.__logerr("Failed to send land request")
        else:
            self.__loginfo("Land request sent to drone.")
        return response.status

    def takeoff_cb(self,req=None):
        """Callback for telling the drone to take-off

           Parameters
           req - is an empty service request (no input required)

           Returns
           mavros/Error message indicating if the command was sent successfully
        """
        request = srv.MAVCommandRequest()
        request.command = srv.MAVCommandRequest.CMD_TAKEOFF
        request.custom = srv.MAVCommandRequest.CUSTOM_NO_OP

        try:
            response = self.mav_command_srv(request)
        except rospy.ServiceException as e:
            self.__logerr("MAVCommand service threw exception while trying"
                " to take-off: %s" % e)
            return SERVICE_CALL_FAILED_ERR

        if SUCCESS_ERR != response.status:
           self.__logerr("Failed to send takeoff request")
        else:
            self.__loginfo("Takeoff request sent successfully.")
        return response.status

    def emergency_cb(self,req=None):
        """Callback for telling the drone to enter emergency mode

           Parameters
           req - is an empty service request (no input required)

           Returns
           mavros/Error message indicating if the command was sent successfully
        """
        request = srv.MAVCommandRequest()
        request.command = srv.MAVCommandRequest.CMD_CUSTOM_MODE
        request.custom = srv.MAVCommandRequest.CUSTOM_ARDONE_EMERGENCY

        try:
            response = self.mav_command_srv(request)
        except rospy.ServiceException as e:
            self.__logerr("MAVCommand service threw exception while trying"
                " to enter EMERGENCY mode: %s" % e)
            return SERVICE_CALL_FAILED_ERR

        if SUCCESS_ERR == response.status:
            self.uav_mode = srv.SetModeRequest.EMERGENCY
            self.__logwarn("Drone now in emergency mode.")
        else:
            self.__logerr("Drone failed to enter emergency mode")

        return response.status

    def add_waypoints_cb(self,req):
        """Callback for adding a set of waypoints to the queue
           Implements mavros/AddWaypoints ROS service.
           See service definition for details
        """
        #***********************************************************************
        #   Validate all the waypoints
        #***********************************************************************
        for wp in req.waypoints:
            if not self.__valid_waypoint(wp):
                self.__logerr("Cannot add waypoints because one or more "
                        " waypoints are invalid.")
                return WAYPOINT_VERIFICATION_FAILURE_ERR

        #***********************************************************************
        #   Add them to the queue
        #***********************************************************************
        self.waypoint_queue.extend(req.waypoints)

        #***********************************************************************
        #   If we're in AUTO mode, sync them with the drone now
        #   Otherwise, they'll be synced when we next resume the queue in 
        #   AUTO mode.
        #***********************************************************************
        status = self.__set_waypoints_from_queue()
        if SUCCESS_ERR != status:
            self.__logerr("Failed to add and sync waypoints with drone")

        return status

    def add_sweep_cb(self,req):
        """Callback for adding a set of waypoints to the queue
           Implements mavros/AddSweep ROS service.
           See service definition for details
        """
        start = [req.start.easting, req.start.northing]
        end = [req.start.easting, req.start.northing]
        points = rect_sweep(start, end, req.row_width, req.wp_interval)
        return self.__add_sweep_points(points,req)

    def add_spiral_out_cb(self,req):
        """Callback for adding waypoints to spiral out"""
        start = [req.start.easting, req.start.northing]
        end = [req.start.easting, req.start.northing]
        points = spiral_sweep(start, end, req.row_width, req.wp_interval, False)
        return self.__add_sweep_points(points,req)

    def add_spiral_in_cb(self,req):
        """Callback for adding waypoints to spiral in"""
        start = [req.start.easting, req.start.northing]
        end = [req.start.easting, req.start.northing]
        points = spiral_sweep(start, end, req.row_width, req.wp_interval, True)
        return self.__add_sweep_points(points,req)

    def manual_control_cb(self,vel):
        """Callback for directly controlling drones velocity

           Parameters
           vel - mavros/Velocity.msg specifying velocity vector
        """
        #***********************************************************************
        #   Only accept velocity changes if we're in manual mode
        #***********************************************************************
        if srv.SetModeRequest.MANUAL != self.uav_mode:
            self.__logwarn("Velocities only accepted in manual mode")
            return

        #***********************************************************************
        #   Convert velocity into generic RC Command.
        #   This is stored, and resent periodically by main loop in start()
        #***********************************************************************
        self.next_rc_timestamp = rospy.Time.now()
        for i in range(RC_VEL_START,RC_VEL_END):
            self.next_rc.channel[i] = \
                vel.velocity[i]*RC_COEFFICIENT + RC_ZERO_POINT

        #***********************************************************************
        #   Send immediately for good measure
        #***********************************************************************
        self.__logdebug("received velocity - sending: %s" % self.next_rc)
        self.pub_rc.publish(self.next_rc)

    def start(self):
        """Starts execution of controller"""

        #***********************************************************************
        #   Initialise ROS services, subscriptions, and publications
        #***********************************************************************
        self.__ros_init()

        #***********************************************************************
        #   Try to load drone parameters from ROS Parameter server, but do
        #   best to continue
        #***********************************************************************
        status = self.__load_drone_params()
        if SUCCESS_ERR != status:
            self.__logerr("Failed to load parameters for drone.")

        #***********************************************************************
        #   Try to sync parameters with drone, but do best to continue
        #***********************************************************************
        status = self.__sync_params_with_drone()
        if SUCCESS_ERR != status:
            self.__logerr("Failed to sync parameters with drone.")

        #***********************************************************************
        #   Try to clear queue and sync with drone. If this doesn't work
        #   and error will be logged, but we'll do are best to continue.
        #***********************************************************************
        self.clear_queue_cb()

        #**********************************************************************
        #   Start diagnostics running
        #**********************************************************************
        diag_timer = rospy.Timer(DIAG_UPDATE_FREQ, self.update_diagnostics)

        #***********************************************************************
        #   Main loop - continue executing until we're interrupted
        #***********************************************************************
        loop_rate = rospy.Rate(VELOCITY_UPDATE_RATE_IN_HERTZ)
        while not rospy.is_shutdown():

            #********************************************************************
            #   If target velocity (for manual mode) has gone stale, reset it
            #   to zero, so drone doesn't fly away
            #********************************************************************
            if rospy.Time.now() - self.next_rc_timestamp > VELOCITY_TTL:
                self.next_rc.channel = [RC_ZERO_POINT]*len(self.next_rc.channel)
                self.next_rc_timestamp = rospy.Time.now()

            #********************************************************************
            #   If we're in manual mode, publish target velocity periodically
            #********************************************************************
            if srv.SetModeRequest.MANUAL == self.uav_mode:
                self.pub_rc.publish(self.next_rc)

            #********************************************************************
            #   Sleep until next RC message is due
            #********************************************************************
            loop_rate.sleep()


#*******************************************************************************
#   Main Program
#*******************************************************************************
if __name__ == '__main__':

    #**************************************************************************
    # Parse any arguments that follow the node command
    # In particular, we need the name of the UAV to control
    #**************************************************************************
    parser = OptionParser("mavros.py [options]")
    parser.add_option("-n", "--name", dest="name", default="Parrot",
                              help="Name of UAV to control")
    (opts, args) = parser.parse_args()

    try:
        #***********************************************************************
        #   Register ROS Node
        #***********************************************************************
        drone = opts.name  # name of drone to control
        rospy.init_node("%s_controller" % drone)

        #***********************************************************************
        #   If our drone is not listed as active on the ROS Parameter server
        #   then bail out now.
        #***********************************************************************
        active_drones = rospy.get_param(ACTIVE_DRONE_NAMESPACE,[])
        if drone not in active_drones:
            rospy.loginfo("Drone %s is not listed as active. %s will not "
                    " be launched." % (drone, rospy.get_name()) )
            sys.exit()
        else:
            rospy.loginfo("Activating controller for drone %s" % drone)

        #***********************************************************************
        #   Activate controller services for our drone
        #***********************************************************************
        controller = Controller(drone)
        controller.start()

    #***************************************************************************
    #   Exit if we're interrupted
    #***************************************************************************
    except rospy.ROSInterruptException:
        rospy.loginfo("%s exiting normally." % rospy.get_name())

