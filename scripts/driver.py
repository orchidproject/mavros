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
        self.current_waypoints = [None]           # array of waypoints
        self.waypoints_synced_with_mav = [False]  # True for synced waypoint
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
        self.status_msg = mavros.msg.Status()
        self.gps_msg = NavSatFix()
        self.gps_msg.status = NavSatStatus(status=NavSatStatus.STATUS_NO_FIX,
                service=NavSatStatus.SERVICE_GPS)
        self.filtered_pos_msg = mavros.msg.FilteredPosition()

        #**********************************************************************
        #   ROS Diagnostics Updater
        #**********************************************************************
        self.diag_updater = diagnostic_updater.Updater()
        self.diag_updater.setHardwareID("%s" % self.uav_name)
        self.diag_updater.add("state",self.state_diagnostics_cb)
        self.diag_updater.add("gps",self.gps_diagnostics_cb)
        self.diag_updater.add("battery",self.battery_diagnostics_cb)
        self.diag_updater.add("mission",self.mission_diagnostics_cb)

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

    def update_diagnostics(self, event):
        """Callback for updating diagnostics"""

        self.diag_updater.update()

    def battery_diagnostics_cb(self, status):
        """Callback for filling in ROS diagnostic messsages about battery"""

        #**********************************************************************
        #   Record battery precentage in message summary, with warning if
        #   too low.
        #**********************************************************************
        if LOW_BATTERY_THRESHOLD > self.status_msg.battery_remaining:
            status.summary(DIAG_WARN, "%d%% - Low!!" %
                    self.status_msg.battery_remaining)
        else:
            status.summary(DIAG_OK, "%d%% - OK" %
                    self.status_msg.battery_remaining)

        #**********************************************************************
        #   Fill in the details - voltage, current etc
        #**********************************************************************
        status.add("Battery voltage", self.status_msg.battery_voltage)
        status.add("Battery current", self.status_msg.battery_current)
        status.add("Battery remaining", self.status_msg.battery_remaining) 

        return status

    def mission_diagnostics_cb(self, status):
        """Callback for producing ROS diagnostics about current mission"""

        #**********************************************************************
        #   Fill in message with values related to mission
        #**********************************************************************
        horz_dist, vert_dist = self.distance_to_next_waypoint()
        status.add("metres to climb", vert_dist)
        status.add("metres to travel", horz_dist)
        status.add("mission count", self.state.num_of_waypoints)
        status.add("internal mission count", len(self.current_waypoints) )
        status.add("mission", self.state.current_waypoint)

        #**********************************************************************
        #   Give errors if mission is out of sync with drone
        #**********************************************************************
        if len(self.current_waypoints) != len(self.waypoints_synced_with_mav):
            status.summary(DIAG_ERROR, "Inconsistent internal waypoint count.")

        elif len(self.current_waypoints) != self.state.num_of_waypoints:
            status.summary(DIAG_ERROR, "Waypoint count out of sync with MAV.")

        elif not all(self.waypoints_synced_with_mav):
            status.summary(DIAG_WARN, "Waypoints out of sync with MAV.")

        #**********************************************************************
        #   Otherwise, put current mission in summary
        #**********************************************************************
        else:
            status.summary(DIAG_OK, "current waypoint: %d" %
                    self.state.current_waypoint)

        return status

    def gps_diagnostics_cb(self, status):
        """Callback for producing gps diagnostics"""

        #**********************************************************************
        #   If we have no satelite fix, put warning in summary message
        #**********************************************************************
        fix =  self.gps_msg.status.status
        if NavSatStatus.STATUS_FIX == fix:
            status.summary(DIAG_OK,"fix OK")
        else:
            status.summary(DIAG_WARN,"NO GPS FIX!!")

        #**********************************************************************
        #   Fill in current location 
        #**********************************************************************
        status.add("latitude", self.filtered_pos_msg.latitude)
        status.add("longitude", self.filtered_pos_msg.longitude)
        status.add("altitude", self.filtered_pos_msg.relative_altitude)

        #**********************************************************************
        #   Fill in next waypoint location (if defined)
        #**********************************************************************
        next_waypoint = self.get_target_waypoint()
        if next_waypoint is None:
            status.add("next_latitude",None)
            status.add("next_longitude",None)
            status.add("next_altitude",None)
        else:
            status.add("next_latitude",next_waypoint.latitude)
            status.add("next_longitude",next_waypoint.longitude)
            status.add("next_altitude",next_waypoint.altitude)

        #**********************************************************************
        #   Fill in distance to next waypoint location
        #**********************************************************************
        horz_dist, vert_dist = self.distance_to_next_waypoint()
        if horz_dist is None or vert_dist is None:
            status.add("metres_to_go","undefined")
        else:
            metres_to_go = math.sqrt(horz_dist**2 + vert_dist**2)
            status.add("metres_to_go",metres_to_go)

        return status

    def state_diagnostics_cb(self, status):
        """Callback for filling in ROS diagnostic messages

           Parameters
           status - DiagnosticStatusWrapper used to fill in current next
                    status message

           Returns updated status
        """

        #**********************************************************************
        #   Decide what error status to return and fill in summary
        #**********************************************************************
        time_since_last_heartbeat = rospy.Time.now() - self.state.header.stamp
        if self.connection is None:
            status.summary(DIAG_WARN, "No connection yet with MAV.")

        elif time_since_last_heartbeat > MAX_HEARTBEAT_INTERVAL:
            status.summary(DIAG_STALE, "Connection with MAV gone stale")

        else:
            status.summary(DIAG_OK, "OK")

        #**********************************************************************
        #   Report useful state variables
        #**********************************************************************
        status.add("base mode", self.state.base_mode)
        status.add("custom mode", self.state.custom_mode)
        status.add("MAV system status", self.state.system_status)
        status.add("last heartbeat: ", "%.2f" % 
                time_since_last_heartbeat.to_sec())

        return status

    def get_target_waypoint(self):
        """Returns the target waypoint if defined

           Returns a dictionary with values for 
           latitude, longitude and altitude.

           If the current mission is not set, None is returned
        """

        #**********************************************************************
        #   If the target waypoint is undefined, then return nothing
        #**********************************************************************
        if 0 > self.state.current_waypoint or \
               self.state.current_waypoint >= len(self.current_waypoints):
            return None

        target = self.current_waypoints[self.state.current_waypoint]

        if target is None:
            return None
        
        #**********************************************************************
        #   Otherwise, fill in the values from the current waypoint
        #   Not mapping between x,y,z is defined in mavros/Waypoint.msg
        #**********************************************************************
        result = GlobalWaypoint()
        result.latitude = target.x
        result.longitude = target.y
        result.altitude = target.z
        return result

    def distance_to_next_waypoint(self):
        """Calculates the distance to the next waypoint

           Returns a pair: (horizontal distance, vertical distance)
            
           - return values are in metres
           - horizontal distance is always non-negative
           - vertical distance is negative iff current position is above target
           - if current mission is undefined, then both values are set to None
        """

        #**********************************************************************
        #   If the target waypoint is undefined, then return nothing
        #**********************************************************************
        target = self.get_target_waypoint()
        if target is None:
            return (None, None)

        #**********************************************************************
        #   Estimate the distance to travel along the ground (in metres)
        #   between the current and target positions
        #**********************************************************************
        current = self.filtered_pos_msg
        horz_dist = distance_along_ground(current,target)

        #**********************************************************************
        #   Calculate vertical distance to travel in metres (up)
        #**********************************************************************
        vert_dist = target.altitude - current.altitude

        return horz_dist, vert_dist

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
        #   Start diagnostics running
        #**********************************************************************
        diag_timer = rospy.Timer(DIAG_UPDATE_FREQ,self.update_diagnostics)

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

        rospy.Service(self.uav_name + "/set_mission", mavros.srv.SetMission,
                      self.set_mission_cb)

        #**********************************************************************
        # Request waypoints to synchronise list on start up
        #**********************************************************************
        self.connection.waypoint_request_list_send()

        #**********************************************************************
        # Receive and process mavlink messages one at a time forever 
        #**********************************************************************
        timeout_in_secs = self.command_timeout.to_sec()
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
                self.pub_time_sync.publish(Header(stamp=rospy.Time.now()),
                        rospy.Time.from_sec(msg.time_usec / 1E6),
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
                self.gps_msg.status = NavSatStatus(status=fix,
                                              service=NavSatStatus.SERVICE_GPS)
                self.pub_gps.publish(self.gps_msg)

            elif msg_type == "ATTITUDE":
                self.pub_time_sync.publish(Header(stamp=rospy.Time.now()),
                    rospy.Time.from_sec(msg.time_boot_ms / 1E3),
                    self.uav_name)
                self.pub_attitude.publish(Header(stamp=rospy.Time.now()),
                                          msg.roll, msg.pitch, msg.yaw,
                                          msg.rollspeed, msg.pitchspeed,
                                          msg.yawspeed)

            elif msg_type == "SYS_STATUS":
                self.status_msg.header.stamp = rospy.Time.now()
                self.status_msg.battery_voltage = msg.voltage_battery
                self.status_msg.battery_current = msg.current_battery
                self.status_msg.battery_remaining = msg.battery_remaining
                self.status_msg.sensors_enabled = \
                    msg.onboard_control_sensors_enabled
                self.pub_status.publish(self.status_msg)

            elif msg_type == "GLOBAL_POSITION_INT":
                self.pub_time_sync.publish(Header(stamp=rospy.Time.now()),
                    rospy.Time.from_sec(msg.time_boot_ms / 1E3),
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
                self.handle_waypoint_from_mav(msg)

            elif msg_type == "MISSION_COUNT":
                self.handle_waypoint_count(msg)

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
            rospy.init_node("mavros_driver")
            mav_proxy = MavRosProxy(opts.name, opts.device, opts.baudrate, opts.source_system, opts.command_timeout,
                        opts.minimum_altitude, opts.maximum_altitude)
            mav_proxy.start()
    except rospy.ROSInterruptException:
        rospy.loginfo("%s exiting normally." % rospy.get_name())
