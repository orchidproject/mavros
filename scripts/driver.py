#!/usr/bin/env python
"""ROS node for low-level control of mavlink platforms

   COMMAND-LINE ARGUMENTS
   ----------------------

   -n, --name
   Name of platform to control. Used as topic prefix and to look up
   settings in ROS parameter server.

   -b, --baudrate
   Baudrate for communication with platform.

   -d, --device
   Device for communication with mavlink. This can be serial, tcp or
   udp port
                    
   -s, --source-system
   MAVLink source system for this node

   -t, --command-timeout
   Timeout for waiting for MAV commands accomplishment.

   --mina, --minimum-altitude
   Minimum altitude waypoints must have to be accepted (meters) [default=1]

   --maxa, --maximum-altitude
   Minimum altitude waypoints must have to be accepted (meters) [default=5]

   -r, --ros
   Use ROS parameter server [default False]

   -l, --local
   Local IP address to use [default 127.0.0.1]

   SUBSCRIBED TOPICS
   -----------------
   Topic                        Type    Description
   /uav_name/manual_control     RC      Sends Remote Control Commands to MAV

   PUBLISHED TOPICS
   ----------------
   Topic                    Type                Description
   /uav_name/gps            NavSatFix           Raw GPS fix from MAV
   /uav_name/state          State               MAV's current mode and mission
                                                information
   /uav_name/attitude       Attitude            Fused Attitude estimate from MAV
   /uav_name/status         Status              status including battery level
   /uav_name/filtered_pos   FilteredPosition    Fused Position estimate from MAV
   /uav_name/time_sync      TimeReference       Estimated lag between MAV and
                                                ROS time, used for
                                                synchronisation.

   SERVICES PROVIDED
   -----------------
   Service                  Type            Description
   /uav_name/mav_command    MAVCommand      Send MAV Command to drone e.g.
                                            change mode.
   /uav_name/get_waypoints  GetWaypoints    Gets waypoint mission list from MAV
   /uav_name/get_params     GetParameters   Gets Parameters from MAV as
                                            key/value pairs
   /uav_name/set_waypoints  SetWaypoints    Sets waypoint mission list on MAV
                                            (overwrites previous).
   /uav_name/set_params     SetParameters   Sets Parameters on MAV as
                                            key/value pairs

"""
#******************************************************************************
# Standard Imports
#******************************************************************************
import sys, os, math, tf
from socket import error

#******************************************************************************
# ROS Imports
#******************************************************************************
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import Vector3
from mavros.msg import Error
import mavros.msg
import mavros.srv

#******************************************************************************
# Import mavlink packages
#******************************************************************************
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)),
                '../mavlink/pymavlink'))
from mavutil import mavlink as mav
from mavutil import mavlink_connection as mav_connect
from mavutil import all_printable as print_msg
from tools import *

#******************************************************************************
# Constants used in calculation of position covariance
# Something to do with standard GPS error analysis techniques, but not sure
# what these are really based on.
#******************************************************************************
UERE_CONSTANT = 45.5 / 9  # User equivalent range error?
NE_CONSTANT = 1

#******************************************************************************
# Adhoc manual mode - apparently used to request manual control of MAVs that
# might not support manual mode
#******************************************************************************
ADHOC_MANUAL = 99

#******************************************************************************
# How long to sleep during busy waits for communication.
#
# Ideally, we'd redesign the whole thing to suspend threads that are waiting
# for responses, but this would require a major restructure. 
#
# The current busy wait design is not only (slightly) inefficient and
# inelegant, its also potentially error prone if multiple requests are
# handled in parallel. Really need proper locking procedures.
#******************************************************************************
BUSY_WAIT_INTERVAL = 0.1

#******************************************************************************
#   Range for valid WGS84 Coordinates
#******************************************************************************
MIN_VALID_LATITUDE  = -90.0
MAX_VALID_LATITUDE  = +90.0
MIN_VALID_LONGITUDE = -180.0
MAX_VALID_LONGITUDE = +180.0

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
MAV_COMMAND_ERROR_ERR       = Error(code=Error.MAV_COMMAND_ERROR)

class MavRosProxy:
    def __init__(self, name, device, baudrate, source_system=255,
                 command_timeout=5, altitude_min=1, altitude_max=5):

        #**********************************************************************
        #   Name of UAV we're communicating with
        #**********************************************************************
        self.uav_name = name

        #**********************************************************************
        #   Range of altitudes we will accept for new waypoints
        #**********************************************************************
        self.altitude_min = altitude_min
        self.altitude_max = altitude_max

        #**********************************************************************
        #   Variables for dealing with transmission
        #**********************************************************************
        self.device = device
        self.baudrate = baudrate
        self.source_system = source_system
        self.command_timeout = rospy.Duration(secs=command_timeout)
        self.connection = None

        #**********************************************************************
        #   Variables for keeping track of waypoint transmission
        #**********************************************************************
        self.current_waypoints = []        # array of waypoints
        self.waypoints_sent_to_mav = []  # array of bools 
        self.last_wp_ack_result = -1 
        self.last_wp_ack_time = rospy.Time.now()
        self.last_wp_request_time = rospy.Time.now()
        self.last_current_wp_report_time = rospy.Time.now()

        #**********************************************************************
        #   TODO Other stuff we haven't figured out / refactored yet
        #**********************************************************************
        self.command_ack = 0
        self.param_req = False
        self.yaw_offset = 0
        self.yaw_counter = 0

        #**********************************************************************
        # Containers for messages we need to publish
        #**********************************************************************
        self.state = mavros.msg.State()
        self.gps_msg = NavSatFix()
        self.filtered_pos_msg = mavros.msg.FilteredPosition()

        #**********************************************************************
        # Register ROS Publications
        #**********************************************************************
        self.pub_gps = rospy.Publisher(self.uav_name + '/gps', NavSatFix,
                                       queue_size=10)

        self.pub_state = rospy.Publisher(self.uav_name + '/state',
                                         mavros.msg.State, queue_size=10)

        self.pub_attitude = rospy.Publisher(self.uav_name + '/attitude',
                                            mavros.msg.Attitude, queue_size=10)

        self.pub_status = rospy.Publisher(self.uav_name + '/status',
                                          mavros.msg.Status, queue_size=10)

        self.pub_filtered_pos = rospy.Publisher(self.uav_name + '/filtered_pos',                                                mavros.msg.FilteredPosition,
                                                queue_size=10)

        self.pub_time_sync = rospy.Publisher(self.uav_name + '/time_sync',
                                             TimeReference, queue_size=10)

        #**********************************************************************
        # Register ROS Topic Subscriptions
        #**********************************************************************
        rospy.Subscriber(self.uav_name + "/manual_control", mavros.msg.RC,
                         self.manual_control_cb)

    def manual_control_cb(self, req):
        '''Callback for Manual Remote Control Inputs

           Simply sends RC inputs to MAV
           No connection or execution guarrantees are given.

           Parameters:
           req -- mavros/RC message
        '''
        self.connection.mav.rc_channels_override_send(
                self.connection.target_system,
                self.connection.target_component,
                req.channel[0],
                req.channel[1],
                req.channel[2],
                req.channel[3],
                req.channel[4],
                req.channel[5],
                req.channel[6],
                req.channel[7])

    def get_params_cb(self, req):
        '''Callback for getting parameters from MAV

           Implements mavros/GetParameters service

           If successful, returns parameter key-value pairs from MAV
           Otherwise appropriate error code is return as part of
           result message.

           See mavros/GetParameters.srv definition for details
        '''

        #**********************************************************************
        #   Initialise remote service response
        #**********************************************************************
        result = mavros.srv.GetParametersResponse()

        #**********************************************************************
        # Fetch all parameter values from MAV
        #**********************************************************************
        rospy.loginfo("Fetching MAV Parameters for %s" % self.uav_name)
        self.connection.param_fetch_complete = False
        self.connection.param_fetch_all()
        start_time = rospy.Time.now()
        while not self.connection.param_fetch_complete:
            rospy.sleep(BUSY_WAIT_INTERVAL)
            if (rospy.Time.now() - start_time) > self.command_timeout:
                rospy.logwarn("Request for parameters from %s timed out" %
                              self.uav_name)
                result.status = MAV_TIMEOUT_ERR
                return result

        #**********************************************************************
        # If we get this far, return parameter values and indicate successful
        # execution.
        #**********************************************************************
        result.keys = self.connection.params.keys()
        result.values = self.connection.params.values()
        result.status = SUCCESS_ERR
        rospy.loginfo("Returning MAV Parameters for %s" % self.uav_name)
        return result

    def set_params_cb(self, req):
        '''Callback for setting parametesr on MAV

           Implements mavros/SetParameters service

           Updates specified key-value parameter pairs on MAV
           Returns error status indicating either success or failure.

           See mavros/SetParameters service definition for details.
        '''

        #**********************************************************************
        #   Initialise remote service response
        #**********************************************************************
        result = mavros.srv.SetParametersResponse()

        #**********************************************************************
        #   Ensure that number of keys matches number of values
        #**********************************************************************
        if len(req.values) != len(req.keys):
            rospy.logerr("[MAVROS:%s] number of parameter keys must match" 
                         " number of values!" % self.uav_name)
            result.status = KEY_VALUE_COUNT_MISMATCH_ERR
            return result

        #**********************************************************************
        #   For each parameter
        #**********************************************************************
        for i in range(len(req.values)):

            #******************************************************************
            # Send request to set current parameter
            #******************************************************************
            start_time = rospy.Time.now()
            rospy.loginfo("[MAVROS:%s] SETTING: " % self.uav_name +
                          req.keys[i] + "=" + str(req.values[i]))
            self.param_req = True
            self.connection.param_fetch_complete = False
            self.connection.param_set_send(req.keys[i], req.values[i])

            #******************************************************************
            # Wait for operation to complete, or time out
            #******************************************************************
            while not self.connection.param_fetch_complete:
                rospy.sleep(BUSY_WAIT_INTERVAL)
                if (rospy.Time.now() - start_time) > self.command_timeout:
                    rospy.logwarn("Time out while setting param %s for %s" %
                                  (req.keys[i],self.uav_name) )
                    result.status = MAV_TIMEOUT_ERR
                    return result

            #******************************************************************
            #   Ensure that parameter has been set (at all)
            #******************************************************************
            if not req.keys[i] in self.connection.params:
                rospy.logwarn("Parameter %s was not set for %s after request" %
                              (req.keys[i], self.uav_name) )
                result.status = PARAM_NOT_SET_ERR
                return result

            #******************************************************************
            #   Also ensure that it is set to the *right* value!
            #******************************************************************
            elif req.values[i] != self.connection.params[ req.keys[i] ]:
                rospy.logwarn("Parameter %s for %s has wrong value" %
                              (req.keys[i], self.uav_name) )
                result.status = BAD_PARAM_VALUE_ERR
                return result

        #**********************************************************************
        #   If we get this far, return SUCCESS
        #**********************************************************************
        result.status = SUCCESS_ERR
        return result

    def mav_command_cb(self, req):
        start_time = rospy.Time.now()
        if req.command == mavros.srv._Command.CommandRequest.CMD_TAKEOFF:
            if "LAND" not in self.connection.mode_mapping().keys():
                rospy.loginfo("[MAVROS:%s]This vehicle can not fly." %
                              self.uav_name)
                return False
            if self.state.custom_mode != self.connection.mode_mapping()["LAND"]:
                rospy.loginfo("[MAVROS:%s]Already in TAKEOFF" % self.uav_name)
                return True

            self.connection.mav.command_long_send(self.connection.target_system,
                                                  0, mav.MAV_CMD_NAV_TAKEOFF,
                                                  0, 0, 0, 0, 0, 0, 0, 0)

            rospy.sleep(BUSY_WAIT_INTERVAL)
            while self.state.custom_mode == self.connection.mode_mapping()["LAND"]:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.loginfo(
                        "[MAVROS:%s]Timeout while trying to takeoff..." % self.uav_name + str(self.state.custom_mode))
                    return False
                rospy.sleep(BUSY_WAIT_INTERVAL)
            rospy.loginfo("[MAVROS:%s]Taken off" % self.uav_name)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_LAND:
            if "LAND" not in self.connection.mode_mapping().keys():
                rospy.loginfo("[MAVROS:%s]This vehicle can not fly." % self.uav_name)
                return False
            if self.state.custom_mode == self.connection.mode_mapping()["LAND"]:
                rospy.loginfo("[MAVROS:%s]Already in LANDED" % self.uav_name)
                return True
            self.connection.mav.command_long_send(self.connection.target_system, 0,
                                                  mav.MAV_CMD_NAV_LAND,
                                                  0, 0, 0, 0, 0, 0, 0, 0)
            rospy.sleep(BUSY_WAIT_INTERVAL)
            while self.state.custom_mode != self.connection.mode_mapping()["LAND"]:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.loginfo(
                        "[MAVROS:%s]Timeout while trying to land..." % self.uav_name + str(self.state.custom_mode))
                    return False
                rospy.sleep(BUSY_WAIT_INTERVAL)
            rospy.loginfo("[MAVROS:%s]Landed" % self.uav_name)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_HALT:
            # self.connection.mav.mission_set_current_send(self.connection.target_system,
            # self.connection.target_component, 0)
            self.connection.mav.command_long_send(self.connection.target_system, 0,
                                                  mav.MAV_CMD_OVERRIDE_GOTO,
                                                  1, mav.MAV_GOTO_DO_HOLD,
                                                  mav.MAV_GOTO_HOLD_AT_CURRENT_POSITION,
                                                  mav.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                  0, 0, 0, 0)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_RESUME:
            self.connection.mav.command_long_send(self.connection.target_system, 0,
                                                  mav.MAV_CMD_OVERRIDE_GOTO,
                                                  1, mav.MAV_GOTO_DO_CONTINUE,
                                                  mav.MAV_GOTO_HOLD_AT_CURRENT_POSITION,
                                                  mav.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                  0, 0, 0, 0)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_COMMAND:
            self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
                                                  req.custom,
                                                  1, 0, 0, 0, 0, 0, 0, 0)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_MANUAL:
            if "MANUAL" not in self.connection.mode_mapping().keys():
                rospy.loginfo(
                    "[MAVROS:%s]This vehicle might not support manual, sending " % self.uav_name + str(ADHOC_MANUAL))
                mode = ADHOC_MANUAL
            else:
                mode = self.connection.mode_mapping()["MANUAL"]
            if self.state.custom_mode == mode:
                rospy.loginfo("[MAVROS:%s]Already in MANUAL" % self.uav_name)
                return True
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                              mode)
            rospy.sleep(BUSY_WAIT_INTERVAL)
            while self.state.custom_mode != mode:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Timeout while going to MANUAL..." % self.uav_name)
                    return False
                rospy.sleep(BUSY_WAIT_INTERVAL)
            rospy.loginfo("[MAVROS:%s]Switched to MANUAL" % self.uav_name)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_AUTO:
            if "AUTO" not in self.connection.mode_mapping().keys():
                rospy.loginfo("[MAVROS:%s]This vehicle does not have auto. Sending mission start..." % self.uav_name)
                self.connection.set_mode_auto()
                return True
            if self.state.custom_mode == self.connection.mode_mapping()["AUTO"]:
                rospy.loginfo("[MAVROS:%s]Already in AUTO" % self.uav_name)
                return True
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                              self.connection.mode_mapping()["AUTO"])
            rospy.sleep(BUSY_WAIT_INTERVAL)
            while self.state.custom_mode != self.connection.mode_mapping()["AUTO"]:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Timeout while going to AUTO..." % self.uav_name)
                    return False
                rospy.sleep(BUSY_WAIT_INTERVAL)
            rospy.loginfo("[MAVROS:%s]Switched to AUTO" % self.uav_name)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_BASE_MODE:
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              req.custom, 0)
            rospy.loginfo("[MAVROS:%s]Base mode(%s)..." % (self.uav_name, req.custom))
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_CUSTOM_MODE:
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, req.custom)
            rospy.loginfo("[MAVROS:%s]Custom mode(%s)..." % (self.uav_name, req.custom))
            return True
        elif req.command == mavros.srv.CommandRequest.CMD_CLEAR_WAYPOINTS:
            return clear_waypoints_cmd(self)
        return False

    def wait_for_wp_ack(self,from_time):
        """Wait for MAV to acknowledge transmission of all waypoints

           Parameters:
           from_time - type: rospy.Time
                       Time acknowledgement is sent must be after this time
                       to be counted.

           Returns: mavros/Error.msg object indicating error or success.
        """

        #**********************************************************************
        #   Wait for mission acknowledgement
        #**********************************************************************
        while self.last_wp_ack_time < start_time:
            rospy.sleep(BUSY_WAIT_INTERVAL)
            if rospy.Time.now() - start_time > self.command_timeout:
                rospy.logerr("[MAVROS:%s]Timeout while updating waypoints..." %
                              self.uav_name)
                return MAV_TIMEOUT_ERR

        #**********************************************************************
        #   Check that acknowledgement reported success
        #**********************************************************************
        if self.last_wp_ack_result != mav.MAV_MISSION_ACCEPTED:
            rospy.logerr("[MAVROS:%s]Failed to update waypoints[%d]" %
                          (self.uav_name, self.last_wp_ack_result))
            return MAV_COMMAND_ERROR_ERR

        #**********************************************************************
        #   If we get this far, return success
        #**********************************************************************
        rospy.loginfo("[MAVROS:%s] Waypoints updated successfull" %
                      self.uav_name)
        return SUCCESS_ERR

    def clear_waypoints_cmd(self):
        """Executes a clear waypoints command on MAV"""

        #**********************************************************************
        #   Clear our own internal list of waypoints
        #**********************************************************************
        self.state.current_waypoint = 0
        self.state.num_of_waypoints = 0
        self.current_waypoints = []
        self.waypoints_sent_to_mav = []
           
        #**********************************************************************
        #   Ask MAV to clear waypoints
        #**********************************************************************
        start_time = rospy.Time.now() # must be set before clear_all_send
        self.last_wp_ack_result = -1  # for comparison to valid vals
        self.connection.waypoint_clear_all_send()

        #**********************************************************************
        #   Wait for MAV to acknowledge receipt of all waypoints
        #**********************************************************************
        return self.wait_for_wp_ack(self,start_time)
            
    def get_waypoints_cb(self, req):
        pass

    def set_waypoints_cb(self, req):
        """Callback implementing mavros/SetWaypoints service.

           Sets new waypoint list on MAV, overwriting any previous.
           Implements mavlink set waypoints protocol.

           See mavros/SetWaypoints.srv definition.
        """

        #**********************************************************************
        #   Validate waypoint list before attempting to send to MAV
        #**********************************************************************
        for wp in req.waypoints:

            #******************************************************************
            #   Check that waypoint is in GLOBAL frame
            #******************************************************************
            if wp.frame != mavros.msg.Waypoint.FRAME_GLOBAL:
                rospy.logwarn("[MAVROS:%s] Driver only supports waypoints in "
                              "global frame - ignoring waypoint request" %
                              self.uav_name)
                return UNSUPPORTED_FRAME_ERR

            #******************************************************************
            #   Check that altitude is in range
            #******************************************************************
            if self.altitude_min > wp.z:
                rospy.logwarn("[MAVROS:%s] requested waypoint is too low "
                              "- ignoring waypoint request" % self.uav_name)
                return COORDS_OUT_OF_RANGE_ERR

            if self.altitude_max < wp.z:
                rospy.logwarn("[MAVROS:%s] requested waypoint is too high "
                              "- ignoring waypoint request" % self.uav_name)
                return COORDS_OUT_OF_RANGE_ERR

            #******************************************************************
            #   Check that longitude is in range
            #******************************************************************
            if MIN_VALID_LONGITUDE > wp.y or wp.y > MAX_VALID_LONGITUDE:
                rospy.logwarn("[MAVROS:%s] requested longitude is invalid "
                              "- ignoring waypoint request" % self.uav_name)
                return COORDS_OUT_OF_RANGE_ERR

            #******************************************************************
            #   Check that latitude is in range
            #******************************************************************
            if MIN_VALID_LATITUDE > wp.x or wp.x > MAX_VALID_LATITUDE:
                rospy.logwarn("[MAVROS:%s] requested latitude is invalid "
                              "- ignoring waypoint request" % self.uav_name)
                return COORDS_OUT_OF_RANGE_ERR

        #**********************************************************************
        #   Try to clear waypoint list
        #**********************************************************************
        status = self.clear_waypoints_cmd():
        if SUCCESS_ERR != status:
            rospy.logerr("[MAVROS:%s] failed to clear waypoints prior to"
                         " updating list" % self.uav_name)
            return status

        #**********************************************************************
        #   Update our internal list of waypoints ready for transmission
        #**********************************************************************
        self.current_waypoints = req.waypoints
        self.waypoints_sent_to_mav = [False] * len(req.waypoints)
        
        #**********************************************************************
        # Tell MAV how many waypoints we're about to send, and wait for it
        # to start requesting all waypoints
        #**********************************************************************
        start_time = rospy.Time.now()  # to be safe set before count_send
        self.connection.waypoint_count_send(len(req.waypoints))
        while not all(self.waypoints_sent_to_mav):
            rospy.sleep(BUSY_WAIT_INTERVAL)
            if rospy.Time.now() - start_time > self.command_timeout:
                rospy.logerr("[MAVROS:%s]Time out waiting for mav to "
                              "request all waypoints" % self.uav_name)
                return MAV_TIMEOUT_ERR

        #**********************************************************************
        #   Wait for MAV to acknowledge receipt of all waypoints
        #**********************************************************************
        status = self.wait_for_wp_ack(self,start_time)
        if SUCCESS_ERR != status:
            rospy.logerr("[MAVROS:%s] Failed to successfully send waypoints" %
                          self.uav_name)
            return status

        #**********************************************************************
        #   Set current mission to the next waypoint
        #**********************************************************************
        start_time = rospy.Time.now()
        while self.last_current_wp_report_time < start_time:
            rospy.sleep(BUSY_WAIT_INTERVAL)
            if rospy.Time.now() - start_time > self.command_timeout:
                rospy.logerr("[MAVROS:%s]Time out setting current mission" %
                              self.uav_name)
                return MAV_TIMEOUT_ERR
            self.connection.mav.mission_set_current_send(
                    self.connection.target_system,
                    self.connection.target_component, 1)

        #**********************************************************************
        #   If we get this far, return success
        #**********************************************************************
        rospy.loginfo("[MAVROS:%s] Current mission set to next waypoint" %
                      self.uav_name)
        return SUCCESS_ERR

    def respond_to_wp_request(self, msg):
        """Transmits a single waypoint to the MAV on request

           Parameters:
           waypoint - single mavros/Waypoint.msg object to transmit

        """

        #**********************************************************************
        #   Ensure the requested waypoint is a valid sequence number
        #**********************************************************************
        if len(self.current_waypoints) <= msg.seq:
            rospy.logwarn("[MAVROS:%s] Ignoring request for waypoint with "
                          "out of range sequence number" % self.uav_name)
            return

        #**********************************************************************
        #   Send the waypoint.
        #   Note we currently set the target yaw orientiation always to north
        #   and the loiter circle radius to 0.
        #   In future, we might want to make these attributes of the
        #   mavros/Waypoint.msg, so we can set them dynamically
        #**********************************************************************
        waypoint = self.current_waypoints[msg.seq]
        waitTime_in_ms = waypoint.waitTime.to_sec() * 1000;
        self.connection.mav.mission_item_send(
                self.connection.target_system,      # target system for wp
                self.connection.target_component,   # target component for wp
                msg.seq,                            # wp sequence number
                mav.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # coordinate frame
                mav.MAV_CMD_NAV_WAYPOINT,           # goto waypoint
                0,                      # don't make this the current wp yet
                waypoint.autocontinue,  # continue to next waypoint or not
                waypoint.radius,        # range MAV must get within wp
                waitTime_in_ms,         # minimum time to spend at wp (ms)
                0,                      # radius of loiter circle (needed?)
                0,                      # Yaw orientiation (0=NORTH)
                waypoint.x,             # latitude
                waypoint.y,             # longitude
                waypoint.z)             # altitude (relative to ground)

        #**********************************************************************
        #   Log the request
        #**********************************************************************
        self.waypoints_sent_to_mav[msg.seq] = True
        self.last_wp_request_time = rospy.Time.now()
        rospy.loginfo(
            "[MAVROS:%s]MISSION_REQUEST: Waypoint %d sent for system %d"
            "for component %d" % (self.uav_name, msg.seq, msg.target_system,
                                  msg.target_component))

    def start(self):

        #**********************************************************************
        # Try to establish connection with MAV
        #**********************************************************************
        while not rospy.is_shutdown():
            try:
                self.connection = mav_connect(self.device, self.baudrate,
                                              self.source_system)
                break
            except error as e:
                rospy.logerr("[MAVROS]" + str(e) + "|" + str(self.device))
                rospy.sleep(1)

        rospy.loginfo("[MAVROS:%s]Waiting for Heartbeat..." % self.uav_name)
        self.connection.wait_heartbeat()
        rospy.loginfo("[MAVROS:%s]Sleeping for a second to initialise..." %
                      self.uav_name)
        rospy.sleep(1)
        rospy.loginfo("[MAVROS:%s]Connected!" % self.uav_name)

        #**********************************************************************
        # Register ROS Services
        #**********************************************************************
        rospy.Service(self.uav_name + "/mav_command", mavros.srv.MAVCommand,
                      self.mav_command_cb)

        rospy.Service(self.uav_name + "/set_waypoints", mavros.srv.SetWaypoints,
                      self.set_waypoints_cb)

        rospy.Service(self.uav_name + "/get_waypoints", mavros.srv.GetWaypoints,
                      self.get_waypoints_cb)

        rospy.Service(self.uav_name + "/set_params", mavros.srv.SetParameters,
                      self.set_params_cb)

        rospy.Service(self.uav_name + "/get_params", mavros.srv.GetParameters,
                      self.get_params_cb)

        #**********************************************************************
        # Request waypoints to synchronise list on start up
        #**********************************************************************
        self.connection.waypoint_request_list_send()

        #**********************************************************************
        # Receive and process mavlink messages one at a time forever 
        #**********************************************************************
        timeout_in_secs = self.command_timeout
        while not rospy.is_shutdown():

            #******************************************************************
            # Wait for next message
            #******************************************************************
            msg = self.connection.recv_match(blocking=True,
                                             timeout=timeout_in_secs)

            #******************************************************************
            #  Log warning if we do not receive a message for a while 
            #  We at least expect to hear regular heartbeats.
            #******************************************************************
            if not msg:
               rospy.logwarn("Nothing received from %s for %f seconds" %
                             (self.uav_name, timeout_in_secs) )
               continue

            #******************************************************************
            # Process received message based on type
            #******************************************************************
            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":
                self.state.base_mode = msg.base_mode
                self.state.custom_mode = msg.custom_mode
                self.state.system_status = msg.system_status
                self.state.header.stamp = rospy.Time.now()
                self.pub_state.publish(self.state)

            elif msg_type == "GPS_RAW_INT":
                self.pub_time_sync.publish(Header(stamp=rospy.Time.now()), rospy.Time.from_sec(msg.time_usec / 1E6),
                                           self.uav_name)
                self.gps_msg.header.frame_id = self.uav_name + "_base_link"
                self.gps_msg.header.stamp = rospy.Time().now()
                self.gps_msg.latitude = msg.lat / 1e07
                self.gps_msg.longitude = msg.lon / 1e07
                self.gps_msg.altitude = msg.alt / 1e07
                variance_value = UERE_CONSTANT * (msg.eph ** 2) + \
                                 NE_CONSTANT ** 2
                position_covariance = [0] * 9
                position_covariance[0] = variance_value
                position_covariance[4] = variance_value
                position_covariance[8] = variance_value
                self.gps_msg.position_covariance = position_covariance
                self.gps_msg.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                fix = NavSatStatus.STATUS_NO_FIX
                if msg.fix_type >= 3:
                    fix = NavSatStatus.STATUS_FIX
                self.gps_msg.status = NavSatStatus(status=fix, service=NavSatStatus.SERVICE_GPS)
                self.pub_gps.publish(self.gps_msg)

            elif msg_type == "ATTITUDE":
                self.pub_time_sync.publish(Header(stamp=rospy.Time.now()), rospy.Time.from_sec(msg.time_boot_ms / 1E3),
                                           self.uav_name)
                self.pub_attitude.publish(Header(stamp=rospy.Time.now()),
                                          msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)

            elif msg_type == "SYS_STATUS":
                status_msg = mavros.msg.Status()
                status_msg.header.stamp = rospy.Time.now()
                status_msg.battery_voltage = msg.voltage_battery
                status_msg.battery_current = msg.current_battery
                status_msg.battery_remaining = msg.battery_remaining
                status_msg.sensors_enabled = msg.onboard_control_sensors_enabled
                self.pub_status.publish(status_msg)

            elif msg_type == "GLOBAL_POSITION_INT":
                self.pub_time_sync.publish(Header(stamp=rospy.Time.now()), rospy.Time.from_sec(msg.time_boot_ms / 1E3),
                                           self.uav_name)
                self.filtered_pos_msg.header.stamp = rospy.Time().now()
                self.filtered_pos_msg.latitude = msg.lat / 1E7
                self.filtered_pos_msg.longitude = msg.lon / 1E7
                self.filtered_pos_msg.altitude = msg.alt / 1E3
                self.filtered_pos_msg.relative_altitude = msg.relative_alt / 1E3
                self.filtered_pos_msg.ground_x_speed = msg.vx
                self.filtered_pos_msg.ground_y_speed = msg.vy
                self.filtered_pos_msg.ground_z_speed = msg.vz
                # NORTH IS 180, WEST-90, SOUTH-0,
                # EVERYTHING BETWEEN SOUTH-EAST-NORTH SIDE IS 0 for AR Drone 2.0
                self.filtered_pos_msg.heading = msg.hdg
                self.pub_filtered_pos.publish(self.filtered_pos_msg)

            elif msg_type == "MISSION_CURRENT":
                self.state.current_waypoint = msg.seq
                self.last_current_wp_report_time = rospy.Time.now()

            elif msg_type == "MISSION_ITEM":
                pass
                #rospy.loginfo("[MAVROS:%s]%s" % (self.uav_name, msg))
                # header = Header()
                # header.stamp = rospy.Time.now()
                # self.pub_mission_item.publish(header, msg.seq, msg.current,
                # msg.autocontinue, msg.param1,
                # msg.param2, msg.param3, msg.param4,
                # msg.x, msg.y, msg.z)

            elif msg_type == "MISSION_COUNT":
                self.state.num_of_waypoints = msg.count
                rospy.loginfo("[MAVROS:%s]MISSION_COUNT: Number of Mission "
                              "Items - %s" % (self.uav_name, str(msg.count)))

            elif msg_type == "MISSION_ACK":
                self.last_wp_ack_time = rospy.Time.now()
                self.last_wp_ack_result = msg.type
                rospy.loginfo(
                    "[MAVROS:%s]MISSION_ACK: Mission Message ACK with response"
                    "- %s" % (self.uav_name, str(msg.type)))

            elif msg_type == "COMMAND_ACK":
                self.command_ack = rospy.Time.now()
                rospy.loginfo(
                    "[MAVROS:%s]COMMAND_ACK: Command Message ACK with result"
                    "- %s" % (self.uav_name, str(msg.result)))

            elif msg_type == "MISSION_REQUEST":
                self.respond_to_wp_request(msg)

            elif msg_type == "STATUSTEXT":
                rospy.loginfo("[MAVROS:%s]STATUSTEXT: Status severity is %d. "
                              "Text Message is %s" % 
                              (self.uav_name, msg.severity, msg.text))

            elif msg_type == "PARAM_VALUE":
                if self.param_req:
                    self.connection.param_fetch_complete = True
                    self.param_req = False

            #******************************************************************
            # Log receipt of any type of message we expect to receive, but
            # are ignoring for now
            #******************************************************************
            elif msg_type == "VFR_HUD":
                rospy.logdebug("[MAVROS:%s] Ignoring %s message -- we don't"
                               " care to process these right now." %
                               (self.uav_name, str(msg_type)))

            #******************************************************************
            # Log receipt of any other type of message that we don't really
            # expect to receive
            #******************************************************************
            else:
                rospy.loginfo("[MAVROS:%s] Received %s message." %
                              (self.uav_name, str(msg_type)))


#******************************************************************************
# Parse any arguments that follow the node command
#******************************************************************************
from optparse import OptionParser

parser = OptionParser("mavros.py [options]")

parser.add_option("-n", "--name", dest="name", default="ARDrone",
                  help="Name of UAV")
parser.add_option("-b", "--baudrate", dest="baudrate", type='int',
                  help="Baudrate of the MAVLink connection", default=115200)
parser.add_option("-d", "--device", dest="device", default="udp:192.168.1.2:14550",
                  help="Device on which to receive MAVLink connection (can be serial,tcp,udp)")
parser.add_option("-s", "--source-system", dest='source_system', type='int',
                  default=255, help='MAVLink source system for this node')
parser.add_option("-t", "--command-timeout", dest='command_timeout', type='int',
                  default=5, help='Timeout for waiting for commands accomplishment.')
parser.add_option("--mina", "--minimum-altitude", dest="minimum_altitude", default=1,
                  type='int', help="Minimum altitude waypoints must have to be accepted (meters)")
parser.add_option("--maxa", "--maximum-altitude", dest="maximum_altitude", default=5,
                  type='int', help="Minimum altitude waypoints must have to be accepted (meters)")

parser.add_option("-r", "--ros", action="store_true", dest="ros", help="Use ROS parameter server", default=False)
parser.add_option("-l", "--local", dest="local", default="127.0.0.1", help="Local IP address to use")
(opts, args) = parser.parse_args()

if __name__ == '__main__':
    try:
        if opts.ros:
            if opts.name in rospy.get_param("/drones_active"):
                drone = rospy.get_param("/drones/" + opts.name)
                opts.device = ":".join(("udp", opts.local, str(drone["out_port"])))
            else:
                opts.device = None
        if opts.device:
            rospy.init_node("mavros")
            mav_proxy = MavRosProxy(opts.name, opts.device, opts.baudrate, opts.source_system, opts.command_timeout,
                        opts.minimum_altitude, opts.maximum_altitude)
            mav_proxy.start()
    except rospy.ROSInterruptException:
        pass
