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
        rospy.logdebug("manual control input received")

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
            rospy.loginfo("[MAVROS:%s]Takeoff" % self.uav_name)
            return SUCCESS_ERR

        #**********************************************************************
        #   Execute Land
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_LAND:
            rospy.loginfo("[MAVROS:%s]Landing" % self.uav_name)
            return SUCCESS_ERR

        #**********************************************************************
        #   Halt at current position
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_HALT:
            rospy.loginfo("[MAVROS:%s]Halting" % self.uav_name)
            return SUCCESS_ERR

        #**********************************************************************
        #   Resume waypoints after halt
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_RESUME:
            rospy.loginfo("[MAVROS:%s]Resuming" % self.uav_name)
            return SUCCESS_ERR

        #**********************************************************************
        #   Execute custom command
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_COMMAND:
            rospy.loginfo("[MAVROS:%s]Executing command" % self.uav_name)
            return SUCCESS_ERR

        #**********************************************************************
        #   Change mode to manual
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_MANUAL:
            rospy.loginfo("[MAVROS:%s]Now in manual" % self.uav_name)
            return SUCCESS_ERR

        #**********************************************************************
        #   Change mode to auto
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_AUTO:
            rospy.loginfo("[MAVROS:%s]Now in auto" % self.uav_name)
            return SUCCESS_ERR

        #**********************************************************************
        #   Change base mode
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_BASE_MODE:
            rospy.loginfo("[MAVROS:%s] changed base mode" % self.uav_name)
            return SUCCESS_ERR

        #**********************************************************************
        #   Change custom mode
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_CUSTOM_MODE:
            rospy.loginfo("[MAVROS:%s] changed custom mode" % self.uav_name)
            return SUCCESS_ERR

        #**********************************************************************
        #   Clear waypoints
        #**********************************************************************
        elif req.command == mavros.srv.CommandRequest.CMD_CLEAR_WAYPOINTS:
            return self.clear_waypoints_cmd()

        #**********************************************************************
        #   Return error if we get an undefined command
        #**********************************************************************
        else:
            return UNDEFINED_COMMAND_ERR

        #**********************************************************************
        #   We should never get here
        #**********************************************************************
        return INTERNAL_ERR

    def clear_waypoints_cmd(self):
        """Executes a clear waypoints command on MAV"""

        #**********************************************************************
        #   Clear our own internal list of waypoints
        #**********************************************************************
        rospy.loginfo("[MAVROS:%s]Clearing waypoints on MAV" % self.uav_name)
        self.current_waypoints = []
        return SUCCESS_ERR
            
    def get_waypoints_cb(self, req):
        """Request list of waypoints from MAV

           Sends request for all waypoints from MAV, and waits for main thread
           to update the waypoint list
        """
        return self.current_waypoints, SUCCESS_ERR

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
        rospy.loginfo("[MAVROS:%s] waypoints transmitted successfully" %
                      self.uav_name)
        rospy.loginfo("Waypoints are: %s" % self.current_waypoints)
        rospy.loginfo("Number of waypoints: %d" % len(self.current_waypoints) )
        return SUCCESS_ERR

    def set_mission_cb(self, msg):

        #**********************************************************************
        #   Ensure waypoint id is in range
        #   Note: waypoint 0 is always ok - even if the waypoint list is empty
        #**********************************************************************
        rospy.loginfo("[MAVROS] Processing mission request %s" % msg)
        waypoint_defined = (0 <= msg.waypoint_id) and \
            ( msg.waypoint_id < len(self.current_waypoints) )

        # make special case for waypoint 0 when queue is empty
        if 0==msg.waypoint_id and 0==len(self.current_waypoints):
            waypoint_defined = True

        if not waypoint_defined:
            rospy.logerr("[MAVROS:%s] requested waypoint %d is undefined" %
                         (self.uav_name, msg.waypoint_id) )
            return UNDEFINED_WAYPOINT_ERR

        #**********************************************************************
        #   Set current mission to the next waypoint
        #**********************************************************************
        rospy.loginfo("[MAVROS:%s] Current mission set to waypoint %d" %
                      (self.uav_name, msg.waypoint_id) )

        return SUCCESS_ERR

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
        # Periodically send dummy current position
        #**********************************************************************
        latitude_delta = 0.0
        while not rospy.is_shutdown():
            rospy.sleep(0.2)
            pos = mavros.msg.FilteredPosition()
            pos.header.stamp = rospy.Time.now()
            pos.latitude = 55.0 + latitude_delta
            pos.longitude = -2.0
            pos.altitude = 1.0
            pos.relative_altitude = 1.0
            self.pub_filtered_pos.publish(pos)
            latitude_delta += 0.000001

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
