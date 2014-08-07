#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header, Bool
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
import mavros.msg
import mavros.srv
import sys, struct, time, os
import math
import xmlrpclib
import tools

# #******************************************************************************
# Parse any arguments that follow the node command
# *******************************************************************************
from optparse import OptionParser

parser = OptionParser("mavros.py [options]")

parser.add_option("--vehicle-name", dest="vehicle_name", default="ARDrone",
                  help="name of vehicle")

parser.add_option("--type", dest="type",
                  help="ArduRover or ArduCopter", default="ARDrone")
parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="self.connection port baud rate", default=115200)
parser.add_option("--device", dest="device", default="/dev/ttyACM0",
                  help="serial device")
parser.add_option("--rate", dest="rate", default=1000, type='int',
                  help="parsing rate for mavros")
parser.add_option("--mavlink-rate", dest="mavlink_rate", default=10,
                  type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
parser.add_option("--enable-rc-control", dest="enable_rc_control", default=True,
                  help="enable listening to control messages")
parser.add_option("--enable-waypoint-control", dest="enable_waypoint_control",
                  default=True, help="enable listening to waypoint messages")

# Altitude Safety Parameters
parser.add_option("--launch-altitude", dest="launch_altitude", default=3000,
                  type='int', help="default launch altitude in milli-meters")
parser.add_option("--minimum-waypoint-altitude", dest="minimum_mission_altitude", default=1,
                  type='int', help="Minimum altitude waypoints must have to be accepted (milli-meters)")
parser.add_option("--maximum-altitude", dest="maximum_altitude", default=10,
                  type='int', help="Maximum allowable altitude for vehicle to fly")

# State and command Timeouts
parser.add_option("--waypoint-timeout", dest="waypoint_timeout", default=500,
                  type='int', help="Waypoint transmission timeout in milli-seconds")
parser.add_option("--arm-timeout", dest="arm_timeout", default=10000,
                  type='int', help="Arming/Disarming timeout in milli-seconds")
parser.add_option("--launch-timeout", dest="launch_timeout", default=15000,
                  type='int', help="Launch timeout in milli-seconds")
parser.add_option("--mode-timeout", dest="mode_timeout", default=1000,
                  type='int', help="Mode timeout in milli-seconds")

# Watchdog parameters                  
parser.add_option("--enable-watchdog", dest="enable_watchdog", default=False,
                  help="enable watchdog")
parser.add_option("--max-watchdog-time", dest="max_watchdog_time", default=3,
                  type='int', help="Max time in seconds before watchdog takes over")
parser.add_option("--watchdog-rate", dest="watchdog_rate", default=1,
                  type='int', help="Rate at which watchdog should check")

# ROS Failsafe
parser.add_option("--enable-ros-failsafe", dest="enable_ros_failsafe", default=False,
                  help="Enable Failsafe function to land vehicle if ROS shuts down")

(opts, args) = parser.parse_args()

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))
import mavutil

from tools import *

UERE_CONSTANT = 45.5 / 9
NE_CONSTANT = 1

ADHOC_MANUAL = 99


class MavRosProxy:
    def __init__(self, device, vehicle, baudrate, system, command_timeout=10000, altitude_min=1, altitude_max=10):
        self.connection = None
        self.device = device
        self.vehicle = vehicle
        self.baudrate = baudrate
        self.system = system
        self.command_timeout = command_timeout
        self.connection = mavutil.mavlink_connection(self.device, self.baudrate)
        self.base_mode = None
        self.custom_mode = None
        self.modes = None
        self.mission_ack = 0
        self.command_ack = 0
        self.param_req = False

        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.seq = 0
        self.flying = False

        self.altitude_min = altitude_min
        self.altitude_max = altitude_max
        # Global message containers
        self.gps_msg = NavSatFix()
        self.state_msg = mavros.msg.State()
        self.filtered_pos_msg = mavros.msg.FilteredPosition()
        self.current_mission_msg = mavros.msg.CurrentMission()
        self.pub_gps = rospy.Publisher('gps', NavSatFix, queue_size=10)
        self.pub_imu = rospy.Publisher('imu', Imu, queue_size=10)
        self.pub_rc = rospy.Publisher('rc', mavros.msg.RC, queue_size=10)
        self.pub_state = rospy.Publisher('state', mavros.msg.State, queue_size=10)
        self.pub_vfr_hud = rospy.Publisher('vfr_hud', mavros.msg.VFR_HUD, queue_size=10)
        self.pub_attitude = rospy.Publisher('attitude', mavros.msg.Attitude, queue_size=10)
        self.pub_raw_imu = rospy.Publisher('raw_imu', mavros.msg.Mavlink_RAW_IMU, queue_size=10)
        self.pub_status = rospy.Publisher('status', mavros.msg.Status, queue_size=10)
        self.pub_filtered_pos = rospy.Publisher('filtered_pos', mavros.msg.FilteredPosition, queue_size=10)
        self.pub_current_mission = rospy.Publisher('current_mission', mavros.msg.CurrentMission, queue_size=10)
        self.pub_mission_item = rospy.Publisher('mission_item', mavros.msg.MissionItem, queue_size=10)
        rospy.Subscriber("send_rc", mavros.msg.RC, self.send_rc_cb)

    def send_rc_cb(self, req):
        self.connection.mav.rc_channels_override_send(self.connection.target_system,
                                                      self.connection.target_component,
                                                      req.channel[0], req.channel[1], req.channel[2],
                                                      req.channel[3],
                                                      req.channel[4], req.channel[5], req.channel[6],
                                                      req.channel[7])

    def param_list_cb(self, req):
        if len(req.values) > len(req.names):
            rospy.loginfo("Values must be less or equal to names")
            return [], []
        for i in range(len(req.values)):
            rospy.loginfo("REQUESTED: " + req.names[i] + " " + str(req.values[i]))
            self.connection.param_set_send(req.names[i], req.values[i])
        self.connection.param_fetch_complete = False
        if len(req.names) == 1:
            self.param_req = True
            self.connection.param_fetch_one(req.names[0])
        else:
            self.connection.param_fetch_all()
        while not self.connection.param_fetch_complete:
            rospy.sleep(0.1)
        if len(req.names) == 0:
            return self.connection.params.keys(), self.connection.params.values()
        elif len(req.names) == 1:
            return req.names, [self.connection.params[req.names[0]]]
        else:
            return req.names, [self.connection.params[i] for i in req.names if i in self.connection.params.keys()]

    def command_cb(self, req):
        start_time = rospy.Time.now().to_nsec()
        if req.command == mavros.srv._Command.CommandRequest.CMD_TAKEOFF:
            if "LAND" not in self.modes.keys():
                rospy.loginfo("This vehicle can not fly.")
                return False
            if self.custom_mode != self.modes["LAND"]:
                rospy.loginfo("Already in TAKEOFF")
                return True

            self.connection.mav.command_long_send(self.connection.target_system,
                                                  0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                                  1, 0, 0, 0, 0, 0, 0, 0)

            rospy.sleep(0.1)
            while self.custom_mode == self.modes["LAND"]:
                if rospy.Time.now().to_nsec() - start_time > self.command_timeout * 1E6:
                    rospy.loginfo("Timeout while trying to takeoff..." + str(self.custom_mode))
                    return False
                rospy.sleep(0.01)
            rospy.loginfo("Taken off")
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_LAND:
            if "LAND" not in self.modes.keys():
                rospy.loginfo("This vehicle can not fly.")
                return False
            if self.custom_mode == self.modes["LAND"]:
                rospy.loginfo("Already in LANDED")
                return True
            self.connection.mav.command_long_send(self.connection.target_system,
                                                  0, mavutil.mavlink.MAV_CMD_NAV_LAND,
                                                  1, 0, 0, 0, 0, 0, 0, 0)
            rospy.sleep(0.1)
            while self.custom_mode != self.modes["LAND"]:
                if rospy.Time.now().to_nsec() - start_time > self.command_timeout * 1E6:
                    rospy.loginfo("Timeout while trying to land..." + str(self.custom_mode))
                    return False
                rospy.sleep(0.01)
            rospy.loginfo("Landed")
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_HALT:
            #self.connection.mav.mission_set_current_send(self.connection.target_system,
            #                                             self.connection.target_component, 0)
            self.connection.mav.command_long_send(self.connection.target_system, 0,
                                                  mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO,
                                                  1, mavutil.mavlink.MAV_GOTO_DO_HOLD,
                                                  mavutil.mavlink.MAV_GOTO_HOLD_AT_CURRENT_POSITION,
                                                  mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                  0, 0, 0, 0)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_RESUME:
            self.connection.mav.command_long_send(self.connection.target_system, 0,
                                                  mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO,
                                                  1, mavutil.mavlink.MAV_GOTO_DO_CONTINUE,
                                                  mavutil.mavlink.MAV_GOTO_HOLD_AT_CURRENT_POSITION,
                                                  mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                  0, 0, 0, 0)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_EMERGENCY:
            self.connection.mav.command_long_send(self.connection.target_system, 0,
                                                  100,
                                                  1, 0, 0, 0, 0, 0, 0, 0)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_MANUAL:
            if "MANUAL" not in self.modes.keys():
                rospy.loginfo("This vehicle might not support manual, sending " + str(ADHOC_MANUAL))
                mode = ADHOC_MANUAL
            else:
                mode = self.modes["MANUAL"]
            if self.custom_mode == mode:
                rospy.loginfo("Already in MANUAL")
                return True
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                              mode)
            rospy.sleep(0.1)
            while self.custom_mode != mode:
                if rospy.Time.now().to_nsec() - start_time > self.command_timeout * 1E6:
                    rospy.loginfo("Timeout while going to MANUAL...")
                    return False
                rospy.sleep(0.01)
            rospy.loginfo("Switched to MANUAL")
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_AUTO:
            if "AUTO" not in self.modes.keys():
                rospy.loginfo("This vehicle does not have auto. Sending mission start...")
                self.connection.set_mode_auto()
                return True
            if self.custom_mode == self.modes["AUTO"]:
                rospy.loginfo("Already in AUTO")
                return True
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                              self.modes["AUTO"])
            rospy.sleep(0.1)
            while self.custom_mode != self.modes["AUTO"]:
                if rospy.Time.now().to_nsec() - start_time > self.command_timeout * 1E6:
                    rospy.loginfo("Timeout while going to AUTO...")
                    return False
                rospy.sleep(0.01)
            rospy.loginfo("Switched to AUTO")
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_BASE_MODE:
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              req.custom, 0)
            rospy.loginfo("Base mode(%s)..." % req.custom)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_CUSTOM_MODE:
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, req.custom)
            rospy.loginfo("Custom mode(%s)..." % req.custom)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_CLEAR_WAYPOINTS:
            self.connection.waypoint_clear_all_send()
            rospy.sleep(0.1)
            while self.mission_ack < start_time:
                if rospy.Time.now().to_nsec() - start_time > self.command_timeout * 1E6:
                    rospy.loginfo("Timeout while clearing waypoints...")
                    return False
                rospy.sleep(0.01)
            rospy.loginfo("Cleared waypoints")
            return True
        return False

    def waypoint_list_cb(self, req):
        n = len(req.waypoints)
        start_time = rospy.Time.now().to_nsec()
        old = self.seq
        if old == 0 and self.altitude_min <= self.altitude <= self.altitude_max:
            self.connection.mav.mission_count_send(self.connection.target_system, self.connection.target_component,
                                                   n + 1)
            self.connection.mav.mission_item_send(self.connection.target_system, self.connection.target_component,
                                                  0,
                                                  mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                  1, 0,
                                                  0, 0, 0, 0,
                                                  self.latitude, self.longitude, self.altitude)
            # self.connection.mav.mission_item_send(self.connection.target_system, self.connection.target_component,
            #                                       0,
            #                                       mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            #                                       mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO,
            #                                       1, 0,
            #                                       mavutil.mavlink.MAV_GOTO_DO_HOLD,
            #                                       mavutil.mavlink.MAV_GOTO_HOLD_AT_CURRENT_POSITION,
            #                                       mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            #                                       0, self.latitude, self.longitude, self.altitude)
            self.seq = 1

        # Send entire list of waypoints
        for i in range(n):
            if req.waypoints[i].type == mavros.msg.Waypoint.TYPE_NAV:
                wp_type = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            elif req.waypoints[i].type == mavros.msg.Waypoint.TYPE_TAKEOFF:
                wp_type = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            elif req.waypoints[i].type == mavros.msg.Waypoint.TYPE_LAND:
                wp_type = mavutil.mavlink.MAV_CMD_NAV_LAND
            else:
                wp_type = req.waypoints[i].type
            self.connection.mav.mission_item_send(self.connection.target_system, self.connection.target_component,
                                                  self.seq + i,
                                                  # mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                  mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                  wp_type, 0, req.waypoints[i].autocontinue,
                                                  req.waypoints[i].params[0], req.waypoints[i].params[1],
                                                  req.waypoints[i].params[2], req.waypoints[i].params[3],
                                                  req.waypoints[i].latitude, req.waypoints[i].longitude,
                                                  req.waypoints[i].altitude)
        self.connection.waypoint_request_list_send()
        rospy.sleep(0.1)
        while old + n < self.seq:
            if rospy.Time.now().to_nsec() - start_time > self.command_timeout * 1E6:
                rospy.loginfo("Timeout while sending waypoints...")
                return False
            self.connection.waypoint_request_list_send()
        if old == 0 and self.altitude_min <= self.altitude <= self.altitude_max:
            self.connection.mav.mission_set_current_send(self.connection.target_system,
                                                         self.connection.target_component, 1)
        elif old == 0:
            self.connection.mav.mission_set_current_send(self.connection.target_system,
                                                         self.connection.target_component, 0)
        rospy.loginfo("Waypoints Sent: " + str(self.seq))
        return True

    def start(self):
        rospy.init_node("mavros")
        rospy.loginfo("Waiting for Heartbeat...")
        self.connection.wait_heartbeat()
        rospy.loginfo("Sleeping for a second to initialise...")
        rospy.sleep(1)
        rospy.loginfo("Connected!")
        rospy.Service("command", mavros.srv.Command, self.command_cb)
        rospy.Service("waypoints", mavros.srv.SendWaypoints, self.waypoint_list_cb)
        rospy.Service("params", mavros.srv.Parameters, self.param_list_cb)
        self.modes = self.connection.mode_mapping()
        while not rospy.is_shutdown():
            msg = self.connection.recv_match(blocking=False)
            if not msg:
                continue
            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                if mavutil.all_printable(msg.data):
                    sys.stdout.write(msg.data)
                    sys.stdout.flush()

            elif msg_type == "RC_CHANNELS_RAW":
                self.pub_rc.publish([msg.chan1_raw, msg.chan2_raw, msg.chan3_raw,
                                     msg.chan4_raw, msg.chan5_raw, msg.chan6_raw,
                                     msg.chan7_raw, msg.chan8_raw])

            elif msg_type == "HEARTBEAT":
                self.pub_state.publish(msg.base_mode, msg.custom_mode)
                self.custom_mode = msg.custom_mode
                #self.connection.waypoint_request_list_send()

            elif msg_type == "VFR_HUD":
                self.pub_vfr_hud.publish(msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt,
                                         msg.climb)

            elif msg_type == "GPS_RAW_INT":
                fix = NavSatStatus.STATUS_NO_FIX
                if msg.fix_type >= 3:
                    fix = NavSatStatus.STATUS_FIX

                header = Header()
                header.frame_id = 'base_link'
                header.stamp = rospy.Time.now()
                position_covariance = [0] * 9
                position_covariance[0] = UERE_CONSTANT * (msg.eph ** 2) + NE_CONSTANT ** 2
                position_covariance[4] = UERE_CONSTANT * (msg.eph ** 2) + NE_CONSTANT ** 2
                position_covariance[8] = UERE_CONSTANT * (msg.epv ** 2) + NE_CONSTANT ** 2

                self.pub_gps.publish(NavSatFix(header=header,
                                               latitude=msg.lat / 1e07,
                                               longitude=msg.lon / 1e07,
                                               altitude=msg.alt / 1e03,
                                               position_covariance=position_covariance,
                                               position_covariance_type=NavSatFix.COVARIANCE_TYPE_APPROXIMATED,
                                               status=NavSatStatus(status=fix, service=NavSatStatus.SERVICE_GPS)
                ))

            elif msg_type == "ATTITUDE":
                self.pub_attitude.publish(msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)

            elif msg_type == "RAW_IMU":
                self.pub_raw_imu.publish(Header(), msg.time_usec,
                                         msg.xacc, msg.yacc, msg.zacc,
                                         msg.xgyro, msg.ygyro, msg.zgyro,
                                         msg.xmag, msg.ymag, msg.zmag)

            elif msg_type == "SYS_STATUS":
                status_msg = mavros.msg.Status()
                status_msg.header.stamp = rospy.Time.now()
                status_msg.battery_voltage = msg.voltage_battery
                status_msg.battery_current = msg.current_battery
                status_msg.battery_remaining = msg.battery_remaining
                status_msg.sensors_enabled = msg.onboard_control_sensors_enabled
                self.pub_status.publish(status_msg)

            elif msg_type == "GLOBAL_POSITION_INT":
                header = Header()
                header.stamp = rospy.Time.now()
                self.latitude = msg.lat / 1E7
                self.longitude = msg.lon / 1E7
                self.altitude = msg.relative_alt / 1E3
                self.filtered_pos_msg.header = header
                self.filtered_pos_msg.latitude = msg.lat / 1E7
                self.filtered_pos_msg.longitude = msg.lon / 1E7
                self.filtered_pos_msg.altitude = msg.alt / 1E3
                self.filtered_pos_msg.relative_altitude = msg.relative_alt / 1E3
                self.filtered_pos_msg.ground_x_speed = msg.vx
                self.filtered_pos_msg.ground_y_speed = msg.vy
                self.filtered_pos_msg.ground_z_speed = msg.vz
                self.filtered_pos_msg.heading = msg.hdg
                self.pub_filtered_pos.publish(self.filtered_pos_msg)

            elif msg_type == "NAV_CONTROLLER_OUTPUT":
                self.current_mission_msg.header.stamp = rospy.Time.now()
                self.current_mission_msg.wp_dist = msg.wp_dist
                self.current_mission_msg.target_bearing = msg.target_bearing

                self.pub_current_mission.publish(self.current_mission_msg)
                self.pub_control_output.publish(msg.nav_roll, msg.nav_pitch,
                                                msg.nav_bearing, msg.alt_error,
                                                msg.aspd_error, msg.xtrack_error)

            elif msg_type == "MISSION_CURRENT":
                self.current_mission_msg.header.stamp = rospy.Time.now()
                self.current_mission_msg.mission_num = msg.seq
                self.pub_current_mission.publish(self.current_mission_msg)

            elif msg_type == "MISSION_ITEM":
                header = Header()
                header.stamp = rospy.Time.now()
                self.pub_mission_item.publish(header, msg.seq, msg.current,
                                              msg.autocontinue, msg.param1,
                                              msg.param2, msg.param3, msg.param4,
                                              msg.x, msg.y, msg.z)

            elif msg_type == "MISSION_COUNT":
                self.seq = msg.count
                rospy.loginfo("MISSION_COUNT: Number of Mission Items - " + str(msg.count))

            elif msg_type == "MISSION_ACK":
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    self.mission_ack = rospy.Time.now().to_nsec()
                rospy.loginfo("MISSION_ACK: Mission Message ACK with response - " + str(msg.type))

            elif msg_type == "COMMAND_ACK":
                self.command_ack = rospy.Time.now().to_nsec()
                rospy.loginfo("COMMAND_ACK: Command Message ACK with result - " + str(msg.result))

            elif msg_type == "MISSION_REQUEST":
                rospy.loginfo(
                    "MISSION_REQUEST: Mission Request for target system %d for target component %d with result %d"
                    % (msg.target_system, msg.target_component, msg.seq))

            elif msg_type == "STATUSTEXT":
                rospy.loginfo("STATUSTEXT: Status severity is %d. Text Message is %s" % (msg.severity, msg.text))

            elif msg_type == "PARAM_VALUE":
                #print msg
                if self.param_req:
                    self.connection.param_fetch_complete = True
                    self.param_req = False


if __name__ == '__main__':
    try:
        proxy = MavRosProxy(opts.device, opts.type, opts.baudrate, opts.SOURCE_SYSTEM)
        proxy.start()
    except rospy.ROSInterruptException:
        pass
