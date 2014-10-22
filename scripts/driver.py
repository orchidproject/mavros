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

class MavRosProxy:
    def __init__(self, name, device, baudrate, source_system=255,
                 command_timeout=5, altitude_min=1, altitude_max=5):
        self.uav_name = name
        self.device = device
        self.baudrate = baudrate
        self.source_system = source_system
        self.command_timeout = rospy.Duration(secs=command_timeout)
        self.connection = None
        # self.seq = 0
        self.mission_result = 0
        self.mission_ack = 0
        self.command_ack = 0
        self.list_ack = 0
        self.param_req = False

        self.altitude_min = altitude_min
        self.altitude_max = altitude_max
        self.yaw_offset = 0
        self.yaw_counter = 0
        self.last_current = 0
        # Global message containers
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
            rospy.sleep(0.1)
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
                rospy.sleep(0.1)
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

            rospy.sleep(0.1)
            while self.state.custom_mode == self.connection.mode_mapping()["LAND"]:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.loginfo(
                        "[MAVROS:%s]Timeout while trying to takeoff..." % self.uav_name + str(self.state.custom_mode))
                    return False
                rospy.sleep(0.01)
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
            rospy.sleep(0.1)
            while self.state.custom_mode != self.connection.mode_mapping()["LAND"]:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.loginfo(
                        "[MAVROS:%s]Timeout while trying to land..." % self.uav_name + str(self.state.custom_mode))
                    return False
                rospy.sleep(0.01)
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
            rospy.sleep(0.1)
            while self.state.custom_mode != mode:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Timeout while going to MANUAL..." % self.uav_name)
                    return False
                rospy.sleep(0.01)
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
            rospy.sleep(0.1)
            while self.state.custom_mode != self.connection.mode_mapping()["AUTO"]:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Timeout while going to AUTO..." % self.uav_name)
                    return False
                rospy.sleep(0.01)
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
            return clear_waypoints_cmd(self,req)
        return False

    def clear_waypoints_cmd(self,req):
        """Executes a clear waypoints command on MAV"""
        self.mission_result = -1
        self.connection.waypoint_clear_all_send()
        start_time = rospy.Time.now()
        while self.mission_ack < start_time:
            rospy.sleep(0.1)
            if rospy.Time.now() - start_time > self.command_timeout:
                rospy.loginfo("[MAVROS:%s]Timeout while clearing waypoints..." %
                              self.uav_name)
                # self.seq -= self.state.missions
                return False
        if self.mission_result == mav.MAV_MISSION_ACCEPTED:
            rospy.loginfo("[MAVROS:%s]Cleared waypoints" % self.uav_name)
            self.state.current = 0
            self.state.missions = 0
            return True
        else:
            rospy.loginfo("[MAVROS:%s]Failed to clear waypoints[%d]" %
                          (self.uav_name, self.mission_result))
            return False
            
    def get_waypoints_cb(self, req):
        pass

    def set_waypoints_cb(self, req):
        """Callback implementing mavros/SetWaypoints service.

           Sets new waypoint list on MAV, overwriting any previous.
           Implements mavlink set waypoints protocol.

           See mavros/SetWaypoints.srv definition.
        """
        old = self.state.num_of_waypoints
        seq = self.state.current_waypoint
        if old == 0:
            start_time = rospy.Time.now()
            self.connection.waypoint_count_send(len(req.waypoints))
            while self.list_ack < start_time:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Time out on sending MISSION_COUNT" % self.uav_name)
                    return False

        # Send entire list of waypoints
        for i in range(len(req.waypoints)):
            if req.waypoints[i].frame == mavros.msg.Waypoint.TYPE_GLOBAL:
                req.waypoints[i].frame = mav.MAV_FRAME_GLOBAL_RELATIVE_ALT
            elif req.waypoints[i].frame == mavros.msg.Waypoint.TYPE_NED:
                req.waypoints[i].frame = mav.MAV_FRAME_LOCAL_NED
            if req.waypoints[i].type == mavros.msg.Waypoint.TYPE_NAV:
                req.waypoints[i].type = mav.MAV_CMD_NAV_WAYPOINT
            elif req.waypoints[i].type == mavros.msg.Waypoint.TYPE_TAKEOFF:
                req.waypoints[i].type = mav.MAV_CMD_NAV_TAKEOFF
            elif req.waypoints[i].type == mavros.msg.Waypoint.TYPE_LAND:
                req.waypoints[i].type = mav.MAV_CMD_NAV_LAND
            if not self.altitude_min <= req.waypoints[i].altitude <= self.altitude_max:
                rospy.loginfo("[MAVROS:%s]Can't accept waypoints with such altitudes" % self.uav_name)
                return False
        for i in range(len(req.waypoints)):
            if not self.transmit_waypoint(req.waypoints[i]):
                rospy.loginfo("[MAVROS:%s]Failed to send %d'th waypoint" % (self.uav_name, i))
                return False
        start_time = rospy.Time.now()
        if seq == 0:
            while self.state.current_waypoint == 0:
                if rospy.Time.now() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Time out setting the current mission" % self.uav_name)
                    return False
                self.connection.mav.mission_set_current_send(self.connection.target_system,
                                                             self.connection.target_component, (old + 1))
                rospy.sleep(0.1)
        else:
            while self.last_current < start_time:
                rospy.sleep(0.1)
            while self.state.current != seq:
                if rospy.Time.now().to_sec() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Time out setting the current mission" % self.uav_name)
                    return False
                self.connection.mav.mission_set_current_send(self.connection.target_system,
                                                             self.connection.target_component, seq)
                rospy.sleep(0.1)

        return True

    def transmit_waypoint(self, waypoint):
        if not (self.altitude_min <= waypoint.altitude <= self.altitude_max):
            return False
        old = self.state.missions
        while True:
            self.mission_result = -1
            start_time = rospy.Time.now().to_sec()
            self.connection.mav.mission_item_send(self.connection.target_system, self.connection.target_component,
                                                  self.state.missions,
                                                  waypoint.frame,
                                                  waypoint.type, 0, waypoint.autocontinue,
                                                  waypoint.params[0], waypoint.params[1],
                                                  waypoint.params[2], waypoint.params[3],
                                                  waypoint.latitude, waypoint.longitude,
                                                  waypoint.altitude)
            rospy.sleep(0.1)
            self.connection.waypoint_request_list_send()
            while old != self.state.num_of_waypoints - 1:
                if (rospy.Time.now() - start_time) > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Timeout while sending waypoint..." % self.uav_name)
                    return False
                if self.mission_ack > start_time:
                    break
            if self.mission_result != mav.MAV_MISSION_INVALID_SEQUENCE:
                break
            # self.seq += 1
            rospy.sleep(0.1)
            break
        return old == (self.state.missions - 1)

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

        self.connection.waypoint_request_list_send()
        timeout_in_secs = self.command_timeout
        while not rospy.is_shutdown():
            msg = self.connection.recv_match(blocking=True,
                                             timeout=timeout_in_secs)
            if not msg:
               rospy.logwarn("Nothing received from %s for %f seconds" %
                             (self.uav_name, timeout_in_secs) )
               continue

            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                if print_msg(msg.data):
                    sys.stdout.write(msg.data)
                    sys.stdout.flush()

            elif msg_type == "RC_CHANNELS_RAW":
                pass
                # self.pub_rc.publish([msg.chan1_raw, msg.chan2_raw, msg.chan3_raw,
                # msg.chan4_raw, msg.chan5_raw, msg.chan6_raw,
                # msg.chan7_raw, msg.chan8_raw])

            elif msg_type == "HEARTBEAT":
                self.state.base_mode = msg.base_mode
                self.state.custom_mode = msg.custom_mode
                self.state.system_status = msg.system_status
                self.state.header.stamp = rospy.Time.now()
                self.pub_state.publish(self.state)
                # rospy.loginfo("[MAVROS:%s] HB:(%d,%d)" % (self.uav_name, msg.base_mode, msg.custom_mode))
                # self.connection.waypoint_request_list_send()

            elif msg_type == "VFR_HUD":
                pass
                # self.pub_vfr_hud.publish(msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt,
                # msg.climb)

            elif msg_type == "GPS_RAW_INT":
                self.pub_time_sync.publish(Header(stamp=rospy.Time.now()), rospy.Time.from_sec(msg.time_usec / 1E6),
                                           self.uav_name)
                self.gps_msg.header.frame_id = self.uav_name + "_base_link"
                self.gps_msg.header.stamp = rospy.Time().now()
                self.gps_msg.latitude = msg.lat / 1e07
                self.gps_msg.longitude = msg.lon / 1e07
                self.gps_msg.altitude = msg.alt / 1e07
                position_covariance = [0] * 9
                position_covariance[0] = UERE_CONSTANT * (msg.eph ** 2) + NE_CONSTANT ** 2
                position_covariance[4] = UERE_CONSTANT * (msg.eph ** 2) + NE_CONSTANT ** 2
                position_covariance[8] = UERE_CONSTANT * (msg.epv ** 2) + NE_CONSTANT ** 2
                self.gps_msg.position_covariance = position_covariance
                self.gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
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
            elif msg_type == "RAW_IMU":
                pass
                # self.imu_msg.header.stamp = rospy.Time().now()
                #self.imu_msg.linear_acceleration = Vector3(x=msg.xacc, y=msg.xacc, z=msg.xacc)
                #self.imu_msg.angular_velocity = Vector3(x=msg.xgyro, y=msg.ygyro, z=msg.zgyro)
                #self.imu_msg.orientation = tf.transformations.quaternion_from_euler(0, 0, math.atan2(msg.xmag, msg.ymag))
                #self.pub_raw_imu.publish(self.imu_msg)

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
                # NORTH IS 180, WEST-90, SOUTH-0, EVERYTHING BETWEEN SOUTH-EAST-NORTH SIDE IS 0 for AR Drone 2.0
                self.filtered_pos_msg.heading = msg.hdg
                self.pub_filtered_pos.publish(self.filtered_pos_msg)

            elif msg_type == "NAV_CONTROLLER_OUTPUT":
                pass
                # self.current_mission_msg.header.stamp = rospy.Time.now()
                # self.current_mission_msg.wp_dist = msg.wp_dist
                # self.current_mission_msg.target_bearing = msg.target_bearing
                #
                # self.pub_current_mission.publish(self.current_mission_msg)
                # self.pub_control_output.publish(msg.nav_roll, msg.nav_pitch,
                # msg.nav_bearing, msg.alt_error,
                # msg.aspd_error, msg.xtrack_error)

            elif msg_type == "MISSION_CURRENT":
                self.state.current_waypoint = msg.seq
                self.last_current = rospy.Time.now()
                # self.current_mission_msg.header.stamp = rospy.Time.now()
                # self.current_mission_msg.mission_num = msg.seq
                # self.pub_current_mission.publish(self.current_mission_msg)

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
                rospy.loginfo("[MAVROS:%s]MISSION_COUNT: Number of Mission Items - %s" % (self.uav_name, str(msg.count)))

            elif msg_type == "MISSION_ACK":
                self.mission_ack = rospy.Time.now()
                self.mission_result = msg.type
                rospy.loginfo(
                    "[MAVROS:%s]MISSION_ACK: Mission Message ACK with response - %s" % (self.uav_name, str(msg.type)))

            elif msg_type == "COMMAND_ACK":
                self.command_ack = rospy.Time.now()
                rospy.loginfo(
                    "[MAVROS:%s]COMMAND_ACK: Command Message ACK with result - %s" % (self.uav_name, str(msg.result)))

            elif msg_type == "MISSION_REQUEST":
                self.list_ack = rospy.Time.now()
                rospy.loginfo(
                    "[MAVROS:%s]MISSION_REQUEST: Mission Request for system %d for component %d with result %d"
                    % (self.uav_name, msg.target_system, msg.target_component, msg.seq))

            elif msg_type == "STATUSTEXT":
                rospy.loginfo("[MAVROS:%s]STATUSTEXT: Status severity is %d. Text Message is %s" % (
                    self.uav_name, msg.severity, msg.text))

            elif msg_type == "PARAM_VALUE":
                if self.param_req:
                    self.connection.param_fetch_complete = True
                    self.param_req = False
            else:
                rospy.loginfo("[MAVROS:%s] Received %s message." % (self.uav_name, str(msg_type)))


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
