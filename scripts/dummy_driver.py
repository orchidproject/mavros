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
   /uav_name/set_mission    SetMission      Sets mission to preloaded waypoint

"""
#******************************************************************************
# Standard Imports
#******************************************************************************
import sys, os, math, tf
from socket import error
from math import sqrt

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
import diagnostic_updater
import diagnostic_msgs

#******************************************************************************
# Import mavlink packages
#******************************************************************************
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)),
                '../mavlink/pymavlink'))
from mavutil import mavlink as mav
from mavutil import mavlink_connection as mav_connect
from mavutil import all_printable as print_msg
from modes import *
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
#   Constants for controlling diagnostics
#******************************************************************************
LOW_BATTERY_THRESHOLD = 20 # warn if battery level remaining is below this
MAX_HEARTBEAT_INTERVAL = rospy.Duration(secs=3.0) # max time between heartbeats

#******************************************************************************
#   Return status flags for sending to MAV
#******************************************************************************
MAV_OK_STATUS  = 0
MAV_ERR_STATUS = 1

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
        #   Internal storage for things to return
        #**********************************************************************
        self.waypoints = []
        self.params = {}

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

        self.pub_filtered_pos = rospy.Publisher(self.uav_name + '/filtered_pos',
                                                mavros.msg.FilteredPosition,
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
        rospy.loginfo("manual control input received")

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
        result.keys = self.params.keys()
        result.values = self.params.values()
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

        self.params = dict(zip(req.keys, req.values))

        #**********************************************************************
        #   If we get this far, return SUCCESS
        #**********************************************************************
        rospy.loginfo("Parameters set to: %s" % self.params)
        result.status = SUCCESS_ERR
        return result

    def mav_command_cb(self, req):
        """Callback for sending MAV commands

           TODO: Refactor into different callbacks for each command
        """

        #**********************************************************************
        #   Mark the start time for timeouts
        #**********************************************************************
        start_time = rospy.Time.now()

        #**********************************************************************
        #   Execute Takeoff
        #**********************************************************************
        if req.command == mavros.srv.CommandRequest.CMD_TAKEOFF:
            if "LAND" not in self.connection.mode_mapping().keys():
                rospy.logwarn("[MAVROS:%s]This vehicle can not fly." %
                              self.uav_name)
                return UNSUPPORTED_COMMAND_ERR
            if self.state.custom_mode != self.connection.mode_mapping()["LAND"]:
                rospy.loginfo("[MAVROS:%s]Already in TAKEOFF" % self.uav_name)
                return SUCCESS_ERR

            self.connection.mav.command_long_send(self.connection.target_system,
                                                  0, mav.MAV_CMD_NAV_TAKEOFF,
                                                  0, 0, 0, 0, 0, 0, 0, 0)

            rospy.sleep(BUSY_WAIT_INTERVAL)
            while self.state.custom_mode == \
                  self.connection.mode_mapping()["LAND"]:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.logerr(
                        "[MAVROS:%s]Timeout while trying to takeoff..." %
                        self.uav_name + str(self.state.custom_mode))
                    return MAV_TIMEOUT_ERR
                rospy.sleep(BUSY_WAIT_INTERVAL)
            rospy.loginfo("[MAVROS:%s]Taken off" % self.uav_name)
            return SUCCESS_ERR

        #**********************************************************************
        #   Execute Land
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_LAND:
            if "LAND" not in self.connection.mode_mapping().keys():
                rospy.logwarn("[MAVROS:%s]This vehicle can not fly." %
                              self.uav_name)
                return UNSUPPORTED_COMMAND_ERR
            if self.state.custom_mode == self.connection.mode_mapping()["LAND"]:
                rospy.loginfo("[MAVROS:%s]Already in LANDED" % self.uav_name)
                return SUCCESS_ERR
            self.connection.mav.command_long_send(self.connection.target_system,
                                                  0, mav.MAV_CMD_NAV_LAND,
                                                  0, 0, 0, 0, 0, 0, 0, 0)
            rospy.sleep(BUSY_WAIT_INTERVAL)
            while self.state.custom_mode != \
                  self.connection.mode_mapping()["LAND"]:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.logwarn(
                        "[MAVROS:%s]Timeout while trying to land..." %
                        self.uav_name + str(self.state.custom_mode))
                    return MAV_TIMEOUT_ERR
                rospy.sleep(BUSY_WAIT_INTERVAL)
            rospy.loginfo("[MAVROS:%s]Landed" % self.uav_name)
            return SUCCESS_ERR

        #**********************************************************************
        #   Halt at current position
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_HALT:
            # self.connection.mav.mission_set_current_send(
            #   self.connection.target_system,
            #   self.connection.target_component, 0)
            self.connection.mav.command_long_send(self.connection.target_system,
                                          0, mav.MAV_CMD_OVERRIDE_GOTO,
                                          1, mav.MAV_GOTO_DO_HOLD,
                                          mav.MAV_GOTO_HOLD_AT_CURRENT_POSITION,
                                          mav.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                          0, 0, 0, 0)
            return SUCCESS_ERR

        #**********************************************************************
        #   Resume waypoints after halt
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_RESUME:
            self.connection.mav.command_long_send(self.connection.target_system,
                                        0,
                                        mav.MAV_CMD_OVERRIDE_GOTO,
                                        1, mav.MAV_GOTO_DO_CONTINUE,
                                        mav.MAV_GOTO_HOLD_AT_CURRENT_POSITION,
                                        mav.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                        0, 0, 0, 0)
            return SUCCESS_ERR

        #**********************************************************************
        #   Execute custom command
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_COMMAND:
            self.connection.mav.command_long_send(self.connection.target_system,
                                         self.connection.target_component,
                                         req.custom, 1, 0, 0, 0, 0, 0, 0, 0)
            return SUCCESS_ERR

        #**********************************************************************
        #   Change mode to manual
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_MANUAL:
            if "MANUAL" not in self.connection.mode_mapping().keys():
                rospy.logwarn(
                    "[MAVROS:%s]This vehicle might not support manual" %
                    self.uav_name + str(ADHOC_MANUAL))
                mode = ADHOC_MANUAL
            else:
                mode = self.connection.mode_mapping()["MANUAL"]
            if self.state.custom_mode == mode:
                rospy.loginfo("[MAVROS:%s]Already in MANUAL" % self.uav_name)
                return SUCCESS_ERR
            self.connection.mav.set_mode_send(self.connection.target_system,
                                         mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                         mode)
            rospy.sleep(BUSY_WAIT_INTERVAL)
            while self.state.custom_mode != mode:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Timeout while going to MANUAL" %
                                  self.uav_name)
                    return MAV_TIMEOUT_ERR
                rospy.sleep(BUSY_WAIT_INTERVAL)
            rospy.loginfo("[MAVROS:%s]Switched to MANUAL" % self.uav_name)
            return SUCCESS_ERR

        #**********************************************************************
        #   Change mode to auto
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_AUTO:
            if "AUTO" not in self.connection.mode_mapping().keys():
                rospy.loginfo("[MAVROS:%s]This vehicle does not have auto."
                              " Sending mission start..." % self.uav_name)
                self.connection.set_mode_auto()
                return SUCCESS_ERR
            if self.state.custom_mode == self.connection.mode_mapping()["AUTO"]:
                rospy.loginfo("[MAVROS:%s]Already in AUTO" % self.uav_name)
                return SUCCESS_ERR
            self.connection.mav.set_mode_send(self.connection.target_system,
                                        mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                        self.connection.mode_mapping()["AUTO"])
            rospy.sleep(BUSY_WAIT_INTERVAL)
            while self.state.custom_mode != \
                  self.connection.mode_mapping()["AUTO"]:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.logwarn("[MAVROS:%s]Timeout while going to AUTO..." %
                                  self.uav_name)
                    return MAV_TIMEOUT_ERR
                rospy.sleep(BUSY_WAIT_INTERVAL)
            rospy.loginfo("[MAVROS:%s]Switched to AUTO" % self.uav_name)
            return SUCCESS_ERR

        #**********************************************************************
        #   Change base mode
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_BASE_MODE:
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              req.custom, 0)
            rospy.loginfo("[MAVROS:%s]Base mode(%s)..." %
                          (self.uav_name, req.custom) )
            return SUCCESS_ERR

        #**********************************************************************
        #   Change custom mode
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_CUSTOM_MODE:
            self.connection.mav.set_mode_send(self.connection.target_system,
                                         mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                         req.custom)
            rospy.loginfo("[MAVROS:%s]Custom mode(%s)..." %
                          (self.uav_name, req.custom))
            return SUCCESS_ERR

        #**********************************************************************
        #   Clear waypoints
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_CLEAR_WAYPOINTS:
            return clear_waypoints_cmd(self)

        #**********************************************************************
        #   Return error if we get an undefined command
        #**********************************************************************
        else:
            return UNDEFINED_COMMAND_ERR

        #**********************************************************************
        #   We should never get here
        #**********************************************************************
        return INTERNAL_ERR

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
        while self.last_wp_ack_time < from_time:
            rospy.sleep(BUSY_WAIT_INTERVAL)
            if rospy.Time.now() - from_time > self.command_timeout:
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
        rospy.loginfo("[MAVROS:%s] Waypoints updated successfully" %
                      self.uav_name)
        return SUCCESS_ERR

    def clear_waypoints_cmd(self):
        """Executes a clear waypoints command on MAV"""

        #**********************************************************************
        #   Clear our own internal list of waypoints
        #**********************************************************************
        rospy.loginfo("[MAVROS:%s]Clearing waypoints on MAV" % self.uav_name)
        self.current_waypoints = []
        self.waypoints_synced_with_mav = []
           
        #**********************************************************************
        #   Ask MAV to clear waypoints
        #**********************************************************************
        start_time = rospy.Time.now() # must be set before clear_all_send
        self.last_wp_ack_result = -1  # for comparison to valid vals
        self.connection.waypoint_clear_all_send()

        #**********************************************************************
        #   Wait for MAV to acknowledge receipt of all waypoints
        #   and update MAV waypoint state information on success
        #**********************************************************************
        status = self.wait_for_wp_ack(start_time)
        if SUCCESS_ERR == status:
            self.state.num_of_waypoints = 0
        return status
            
    def get_waypoints_cb(self, req):
        """Request list of waypoints from MAV

           Sends request for all waypoints from MAV, and waits for main thread
           to update the waypoint list
        """

        #**********************************************************************
        #   Flag the waypoints as unsynced
        #**********************************************************************
        self.waypoints_synced_with_mav = [None]
        self.waypoints_synced_with_mav = [False]

        #**********************************************************************
        #   Send request for waypoints to MAV
        #**********************************************************************
        start_time = rospy.Time.now()   # important we record this b4 request
        self.connection.waypoint_request_list_send()

        #**********************************************************************
        #   Wait for waypoints to get in sync again
        #**********************************************************************
        while not all(self.waypoints_synced_with_mav):
            rospy.sleep(BUSY_WAIT_INTERVAL)
            if rospy.Time.now() - start_time > self.command_timeout:
                rospy.logerr("[MAVROS:%s]Time out waiting for mav to "
                              "send all waypoints" % self.uav_name)
                return [], MAV_TIMEOUT_ERR

        #**********************************************************************
        #   If we get this far, return waypoints successfully
        #**********************************************************************
        return self.current_waypoints, SUCCESS_ERR

    def handle_waypoint_from_mav(self,msg):
        """Handle waypoints sent from MAV

           Implements the next step in the waypoint request protocol.

           Syncs the specified waypoint if its not in sync, and requests
           the next waypoint if there are still more to be synced.

           If this is the last waypoint, a final acknowledgement is sent to the
           MAV.

           Otherwise, if all waypoints are already in sync, then this item
           has probably been sent in error.

           Parameters
           msg -- mavlink message of type MISSION_ITEM
        """

        #**********************************************************************
        #   Sanity check that we have a consistent state
        #**********************************************************************
        if len(self.waypoints_synced_with_mav) != \
           len(self.current_waypoints):

           rospy.logerr("[MAVROS %s] INTERNAL ERROR - "
                        "len(current_waypoints) != "
                        "len(waypoints_synced_with_mav)" % self.uav_name)

           return

        #**********************************************************************
        #   Ensure we have a MISSION_ITEM msg
        #**********************************************************************
        if msg.get_type() != "MISSION_ITEM":
            rospy.logerr("[MAVROS %s] INTERNAL ERROR: handle_waypoint_from_mav"
                         " called for non MISSION_ITEM message")
            return

        #**********************************************************************
        #   If all waypoints are already synced, then this is unexpected
        #   receipt.
        #**********************************************************************
        if all(self.waypoints_synced_with_mav):
            rospy.logwarn("[MAVROS %s] Received unexpected MISSION_ITEM "
                          "when all waypoints are synced" % self.uav_name)
            return

        #**********************************************************************
        #   Ensure waypoint is what we expect
        #**********************************************************************
        if msg.get_srcSystem() != self.connection.target_system:
            rospy.logwarn("[MAVROS %s] Received MISSION_ITEM for system %d "
                          " but our system is %d - ignoring" %
                          (self.uav_name, msg.get_srcSystem(),
                           self.connection.target_system) )
            return

        if msg.get_srcComponent() != self.connection.target_component:
            rospy.logwarn("[MAVROS %s] Received MISSION_ITEM for component %d "
                          " but our component is %d - ignoring" %
                          (self.uav_name, msg.get_srcComponent(),
                           self.connection.target_component) )
            return

        if msg.frame != mav.MAV_FRAME_GLOBAL_RELATIVE_ALT:
            rospy.logwarn("[MAVROS %s] Received MISSION_ITEM in "
                          " unexpected frame %d - ignoring" %
                          (self.uav_name, msg.frame) )
            return

        if msg.command != mav.MAV_CMD_NAV_WAYPOINT:
            rospy.logwarn("[MAVROS %s] Received MISSION_ITEM with "
                          " unexpected command type %d - ignoring" %
                          (self.uav_name, msg.frame) )
            return

        #**********************************************************************
        #   If waypoint is out of range, then ignore silently
        #   Note: we may wish to consider sending acknowledgement to MAV
        #   with error code (MAV_ERR_STATUS=1).
        #**********************************************************************
        if 0 > msg.seq or msg.seq >= len(self.current_waypoints):
            rospy.logwarn("[MAVROS %s] Received MISSION_ITEM with "
                          "out of range waypoint id" % self.uav_name)
            return

        #**********************************************************************
        #   Sync the specified waypoint
        #**********************************************************************
        waypoint = mavros.msg.Waypoint()
        waypoint.autocontinue = (1==msg.autocontinue)
        waypoint.radius = msg.param1
        waypoint.waitTime = rospy.Duration(secs=msg.param2/1000.0)
        waypoint.frame = mavros.msg.Waypoint.FRAME_GLOBAL
        waypoint.x = msg.x
        waypoint.y = msg.y
        waypoint.z = msg.z

        self.current_waypoints[msg.seq] = waypoint
        self.waypoints_synced_with_mav[msg.seq] = True

        #**********************************************************************
        #   If everything is now in sync, acknowledge receipt of all waypoints
        #**********************************************************************
        if all(self.waypoints_synced_with_mav):
            rospy.loginfo("[MAVROS %s] final waypoint received from MAV" %
                          self.uav_name)
            self.connection.mav.mission_ack_send(
                    self.connection.target_system,
                    self.connection.target_component, MAV_OK_STATUS)
            return

        #**********************************************************************
        #   Otherwise, request the next unsynced waypoint
        #   Note: list.index(X) returns index of first occurrence of X
        #**********************************************************************
        next_waypoint_id = self.waypoints_synced_with_mav.index(False)  
        self.connection.waypoint_request_send(next_waypoint_id)

        #**********************************************************************
        #   If we get this far, log success
        #**********************************************************************
        rospy.loginfo("[MAVROS %s] waypoint %d received from MAV" %
                      (self.uav_name, msg.seq) )
        return

    def handle_waypoint_count(self,msg):
        """Handles waypoint count message set from MAV

           Initiates waypoint protocol to retrieve waypoint list from MAV
        """

        rospy.loginfo("[MAVROS:%s]MISSION_COUNT: Number of Mission "
            "Items - %s" % (self.uav_name, str(msg.count)))

        #**********************************************************************
        #   Update message count based on message
        #**********************************************************************
        self.state.num_of_waypoints = msg.count
        self.current_waypoints = [None] * msg.count

        #**********************************************************************
        #   Mark all waypoints (if any) as unsynced
        #**********************************************************************
        self.waypoints_synced_with_mav = [False] * msg.count 

        #**********************************************************************
        #   If the count is zero, then be polite and acknowledge the message
        #   - as dictated by mavlink waypoint protocol
        #**********************************************************************
        if all(self.waypoints_synced_with_mav):
            rospy.logdebug("[MAVROS %s] Sending acknowledgement for zero count"
                    % self.uav_name)
            self.connection.mav.mission_ack_send(
                    self.connection.target_system,
                    self.connection.target_component, MAV_OK_STATUS)
            return

        #**********************************************************************
        #   If there are any waypoints, the initiate the mavlink waypoint
        #   request protocol by requesting the first one -- waypoint 0.
        #   Subsequent MISSION_ITEM messages sent in response should then be
        #   be handled in the main loop.
        #**********************************************************************
        self.connection.waypoint_request_send(0)

    def set_waypoints_cb(self, req):
        """Callback implementing mavros/SetWaypoints service.

           Sets new waypoint list on MAV, overwriting any previous.
           Implements mavlink set waypoints protocol.

           NOTE: On some devices, setting the current wp to 0 has special
           meaning e.g. tell the drone to go home. This implementation makes
           no allowances.

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
        status = self.clear_waypoints_cmd()
        if SUCCESS_ERR != status:
            rospy.logerr("[MAVROS:%s] failed to clear waypoints prior to"
                         " updating list" % self.uav_name)
            return status

        #**********************************************************************
        #   Update our internal list of waypoints ready for transmission
        #**********************************************************************
        self.current_waypoints = req.waypoints
        self.waypoints_synced_with_mav = [False] * len(req.waypoints)

        #**********************************************************************
        #   If there are no waypoints, we've already cleared them, so
        #   we're done.
        #   Note: If we don't stop here, drone quite happily requests
        #   waypoint 0, even though we don't have one!
        #**********************************************************************
        if 0==len(req.waypoints):
            rospy.loginfo("Waypoint list is now empty.")
            return SUCCESS_ERR
        
        #**********************************************************************
        # Tell MAV how many waypoints we're about to send, and wait for it
        # to start requesting all waypoints
        #**********************************************************************
        start_time = rospy.Time.now()  # to be safe set before count_send
        rospy.logdebug("sending waypoint count: %d" % len(req.waypoints))
        self.connection.waypoint_count_send(len(req.waypoints))
        while not all(self.waypoints_synced_with_mav):
            rospy.sleep(BUSY_WAIT_INTERVAL)
            if rospy.Time.now() - start_time > self.command_timeout:
                rospy.logerr("[MAVROS:%s]Time out waiting for mav to "
                              "request all waypoints" % self.uav_name)
                return MAV_TIMEOUT_ERR

        #**********************************************************************
        #   Wait for MAV to acknowledge receipt of all waypoints, before
        #   updating count of waypoints on MAV
        #**********************************************************************
        status = self.wait_for_wp_ack(start_time)
        if SUCCESS_ERR == status:
            self.state.num_of_waypoints = len(self.current_waypoints)
        else:
            rospy.logerr("[MAVROS:%s] Failed to successfully send waypoints" %
                          self.uav_name)
            return status

        #**********************************************************************
        #   If we get this far, return success
        #**********************************************************************
        rospy.loginfo("[MAVROS:%s] waypoints transmitted successfully" %
                      self.uav_name)
        return SUCCESS_ERR

    def set_mission_cb(self, msg):

        #**********************************************************************
        #   Ensure waypoint id is in range
        #**********************************************************************
        if 0 > msg.waypoint_id or \
               msg.waypoint_id >= len(self.current_waypoints):

            rospy.logerr("[MAVROS:%s] requested waypoint %d is undefined" %
                         (self.uav_name, msg.waypoint_id) )

            return UNDEFINED_WAYPOINT_ERR

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
                    self.connection.target_component, msg.waypoint_id)

        #**********************************************************************
        #   If we get this far, return success
        #**********************************************************************
        rospy.loginfo("[MAVROS:%s] Current mission set to waypoint %d" %
                      (self.uav_name, msg.waypoint_id) )

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
                "out of range sequence number %d" % (self.uav_name, msg.seq) )
            return

        #**********************************************************************
        #   Send the waypoint.
        #   Note we currently set the target yaw orientiation always to north
        #   and the loiter circle radius to 0.
        #   In future, we might want to make these attributes of the
        #   mavros/Waypoint.msg, so we can set them dynamically
        #**********************************************************************
        waypoint = self.current_waypoints[msg.seq]
        autocontinue_flag = 0
        if waypoint.autocontinue:
            autocontinue_flag = 1
        waitTime_in_ms = waypoint.waitTime.to_sec() * 1000;
        self.connection.mav.mission_item_send(
                self.connection.target_system,      # target system for wp
                self.connection.target_component,   # target component for wp
                msg.seq,                            # wp sequence number
                mav.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # coordinate frame
                mav.MAV_CMD_NAV_WAYPOINT,           # goto waypoint
                0,                      # don't make this the current wp yet
                autocontinue_flag,      # continue to next waypoint or not
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
        self.waypoints_synced_with_mav[msg.seq] = True
        self.last_wp_request_time = rospy.Time.now()
        rospy.loginfo(
            "[MAVROS:%s]MISSION_REQUEST: Waypoint %d sent for system %d"
            " for component %d" % (self.uav_name, msg.seq, msg.target_system,
                                  msg.target_component))

    def start(self):

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

        rospy.Service(self.uav_name + "/set_mission", mavros.srv.SetMission,
                      self.set_mission_cb)

        #**********************************************************************
        # Stay alive until interrupted
        #**********************************************************************
        rospy.spin()

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
            rospy.init_node("mavros_driver")
            mav_proxy = MavRosProxy(opts.name, opts.device, opts.baudrate, opts.source_system, opts.command_timeout,
                        opts.minimum_altitude, opts.maximum_altitude)
            mav_proxy.start()
    except rospy.ROSInterruptException:
        rospy.loginfo("%s exiting normally." % rospy.get_name())
