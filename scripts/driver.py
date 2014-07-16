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
parser.add_option("--minimum-waypoint-altitude", dest="minimum_mission_altitude", default=3000,
                  type='int', help="Minimum altitude waypoints must have to be accepted (milli-meters)")
parser.add_option("--maximum-altitude", dest="maximum_altitude", default=50000,
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
    def __init__(self, device, vehicle, baudrate, system, command_timeout=2000):
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

        # Global message containers
        self.gps_msg = NavSatFix()
        self.state_msg = mavros.msg.State()
        self.filtered_pos_msg = mavros.msg.FilteredPosition()
        self.current_mission_msg = mavros.msg.CurrentMission()
        self.pub_gps = rospy.Publisher('gps', NavSatFix)
        self.pub_imu = rospy.Publisher('imu', Imu)
        self.pub_rc = rospy.Publisher('rc', mavros.msg.RC)
        self.pub_state = rospy.Publisher('state', mavros.msg.State)
        self.pub_vfr_hud = rospy.Publisher('vfr_hud', mavros.msg.VFR_HUD)
        self.pub_attitude = rospy.Publisher('attitude', mavros.msg.Attitude)
        self.pub_raw_imu = rospy.Publisher('raw_imu', mavros.msg.Mavlink_RAW_IMU)
        self.pub_status = rospy.Publisher('status', mavros.msg.Status)
        self.pub_filtered_pos = rospy.Publisher('filtered_pos', mavros.msg.FilteredPosition)
        self.pub_control_output = rospy.Publisher('controller_output', mavros.msg.ControllerOutput)
        self.pub_current_mission = rospy.Publisher('current_mission', mavros.msg.CurrentMission)
        self.pub_mission_item = rospy.Publisher('mission_item', mavros.msg.MissionItem)
        rospy.Subscriber("send_rc", mavros.msg.RC, self.send_rc_cb)
        rospy.Service("command", mavros.srv.APMCommand, self.command_cb)
        rospy.Service("waypoints", mavros.srv.SendWaypointList, self.waypoint_list_cb)

    def send_rc_cb(self, data):
        self.connection.mav.rc_channels_override_send(self.connection.target_system,
                                                      self.connection.target_component,
                                                      data.channel[0], data.channel[1], data.channel[2],
                                                      data.channel[3],
                                                      data.channel[4], data.channel[5], data.channel[6],
                                                      data.channel[7])
        #rospy.loginfo("Sending rc: %s" % data)

    def command_cb(self, req):
        #rospy.loginfo(CUSTOM_MODES[self.vehicle])
        start_time = rospy.Time.now().to_nsec()
        if req.command == mavros.srv._APMCommand.APMCommandRequest.CMD_TAKEOFF:
            if "LAND" not in self.modes.keys():
                rospy.loginfo("This vehicle can not fly.")
                return False
            if self.custom_mode != self.modes["LAND"]:
                rospy.loginfo("Already in TAKEOFF")
                return True
            self.connection.mav.command_long_send(self.connection.target_system,
                                                  0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                                  0, 0, 0, 0, 0, 0, 0, 0)
            rospy.sleep(0.1)
            while self.custom_mode == self.modes["LAND"]:
                if rospy.Time.now().to_nsec() - start_time > self.command_timeout * 1E6:
                    rospy.loginfo("Timeout while trying to takeoff..." + str(self.custom_mode))
                    return False
                rospy.sleep(0.01)
            rospy.loginfo("Taken off")
            return True
        elif req.command == mavros.srv._APMCommand.APMCommandRequest.CMD_LAND:
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
        elif req.command == mavros.srv._APMCommand.APMCommandRequest.CMD_MANUAL:
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
        elif req.command == mavros.srv._APMCommand.APMCommandRequest.CMD_AUTO:
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
        elif req.command == mavros.srv._APMCommand.APMCommandRequest.CMD_BASE_MODE:
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              req.custom, 0)
            rospy.loginfo("Base mode(%s)..." % req.custom)
            return True
        elif req.command == mavros.srv._APMCommand.APMCommandRequest.CMD_CUSTOM_MODE:
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, req.custom)
            rospy.loginfo("Custom mode(%s)..." % req.custom)
            return True
        elif req.command == mavros.srv._APMCommand.APMCommandRequest.CMD_CLEAR_WAYPOINTS:
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
        sizeOfWaypointList = len(req.waypoints)
        rospy.loginfo("Sending Waypoint list of " + str(sizeOfWaypointList) + " items")
        self.connection.mav.mission_count_send(self.connection.target_system, self.connection.target_component,
                                               sizeOfWaypointList + 1)

        # Send Dummy Waypoint
        dummy_wp = mavros.msg.Waypoint()
        dummy_wp.latitude = self.gps_msg.latitude
        dummy_wp.longitude = self.gps_msg.longitude
        dummy_wp.waypoint_type = mavros.msg.Waypoint.TYPE_NAV

        if not self.transmit_waypoint(dummy_wp, 0):
            rospy.loginfo("Waypoint List Transmission Failed")
            return False

        # Send entire list of waypoints
        for i in range(0, sizeOfWaypointList):
            # Check and confirm altitude of each waypoint is greater than minimum altitude
            # if using ArduCopter
            if (req.waypoint[i].altitude < opts.minimum_mission_altitude):
                rospy.loginfo("Waypoint Altitude below minimum allowable altitude")
                return False

            if (not self.transmit_waypoint(req.waypoints[i], i + 1)):
                rospy.loginfo("Waypoint List Transmission Failed")
                return False

        # Set Current Waypoint
        self.connection.mav.mission_set_current_send(self.connection.target_system, self.connection.target_component, 1)

        # Send desired wp radius according to first waypoint of list
        self.connection.mav.param_set_send(self.connection.target_system, self.connection.target_component,
                                               "WPNAV_RADIUS",
                                               req.waypoints[0].pos_acc / 10, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        rospy.loginfo("Waypoints Sent")
        return True

    def transmit_waypoint(self, data, index):
        rospy.loginfo("Sending Waypoint " + str(index))

        start_time = rospy.Time.from_sec(time.time()).to_nsec()

        # Send the waypoint based on the type defined in the Waypoint.msg file
        if (data.waypoint_type == mavros.msg.Waypoint.TYPE_NAV):
            self.connection.mav.mission_item_send(self.connection.target_system, self.connection.target_component,
                                                  index,  # Waypoint Number
                                                  mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame
                                                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Mission Item Type
                                                  0,  # Is Current Waypoint
                                                  1,  # Should Autocontinue to next wp
                                                  data.hold_time / 1000,  # Hold Time (convert from ms to seconds)
                                                  0,  # Pos Acc Not used on APM
                                                  0,  # Orbit Not used on APM
                                                  0,  # Yaw Not used on APM
                                                  data.latitude / 1E7,  # local: x position, global: latitude
                                                  data.longitude / 1E7,  # local: y position, global: longitude
                                                  data.altitude / 1000)  # local: z position, global: altitude (convert from mm to meters)
        elif (data.waypoint_type == mavros.msg.Waypoint.TYPE_TAKEOFF):
            self.connection.mav.mission_item_send(self.connection.target_system, self.connection.target_component,
                                                  index,  # Waypoint Number
                                                  0,
                                                  mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Mission Item Type
                                                  0,  # Is Current Waypoint
                                                  1,  # Should Autocontinue to next wp
                                                  0,
                                                  # Min pitch (if airspeed sensor present), desired pitch without sensor
                                                  0,  # Empty
                                                  0,  # Empty
                                                  0,
                                                  # Yaw angle (if magnetometer present), ignored without magnetometer
                                                  data.latitude / 1E7,  # latitude
                                                  data.longitude / 1E7,  # longitude
                                                  data.altitude / 1000)  # altitude
        elif (data.waypoint_type == mavros.msg.Waypoint.TYPE_CONDITION_YAW):
            self.connection.mav.mission_item_send(self.connection.target_system, self.connection.target_component,
                                                  index,  # Waypoint Number
                                                  0,
                                                  mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # Mission Item Type
                                                  0,  # Is Current Waypoint
                                                  1,  # Should Autocontinue to next wp
                                                  data.yaw_from / 1000,  # Target angle:[0-360], 0 is north
                                                  0,  # Speed During yaw change: [deg per second] (0 use default)
                                                  0,
                                                  # Direction: negative: counter clockwise, positive: clockwise [-1,1] (currently not used)
                                                  0,  # Relative offset or absolute angle: [1,0]
                                                  0,  # local: x position, global: latitude
                                                  0,  # local: y position, global: longitude
                                                  0)  # local: z position, global: altitude (convert from mm to meters)
        elif (data.waypoint_type == mavros.msg.Waypoint.TYPE_CONDITION_CHANGE_ALT):
            self.connection.mav.mission_item_send(self.connection.target_system, self.connection.target_component,
                                                  index,  # Waypoint Number
                                                  0,
                                                  mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,  # Mission Item Type
                                                  0,  # Is Current Waypoint
                                                  1,  # Should Autocontinue to next wp
                                                  0,  # Descent / Ascend rate (m/s)
                                                  0,  # Empty
                                                  0,  # Empty
                                                  0,  # Empty
                                                  0,  # Empty
                                                  0,  # Empty
                                                  data.altitude / 1000)  # Finish Altitude (m)
        else:
            rospy.loginfo("Type of waypoint incorrect")
            return False
        return True

    def start(self):
        rospy.init_node("mavros")
        rospy.loginfo("Waiting for Heartbeat...")
        self.connection.wait_heartbeat()
        self.modes = self.connection.mode_mapping()
        rospy.loginfo("Requesting parameters...")
        self.connection.param_fetch_all()
        while not rospy.is_shutdown():
            msg = self.connection.recv_match(blocking=False)
            if not msg:
                continue
            if msg.get_type() == "BAD_DATA":
                if mavutil.all_printable(msg.data):
                    sys.stdout.write(msg.data)
                    sys.stdout.flush()
            else:
                msg_type = msg.get_type()
                if msg_type == "RC_CHANNELS_RAW":
                    self.pub_rc.publish([msg.chan1_raw, msg.chan2_raw, msg.chan3_raw,
                                         msg.chan4_raw, msg.chan5_raw, msg.chan6_raw,
                                         msg.chan7_raw, msg.chan8_raw])

                elif msg_type == "HEARTBEAT":
                    self.pub_state.publish(msg.base_mode, msg.custom_mode)
                    self.base_mode = msg.base_mode
                    self.custom_mode = msg.custom_mode

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
                    #rospy.loginfo(str(msg.epv) + " " + str(msg.eph))
                    self.pub_gps.publish(NavSatFix(header=header,
                                                   latitude=msg.lat / 1e07,
                                                   longitude=msg.lon / 1e07,
                                                   altitude=msg.alt / 1e03,
                                                   position_covariance=position_covariance,
                                                   position_covariance_type=NavSatFix.COVARIANCE_TYPE_APPROXIMATED,
                                                   status=NavSatStatus(status=fix, service=NavSatStatus.SERVICE_GPS)
                    ))

                    self.gps_msg.latitude = msg.lat
                    self.gps_msg.longitude = msg.lon

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
                    self.filtered_pos_msg.header = header
                    self.filtered_pos_msg.latitude = msg.lat
                    self.filtered_pos_msg.longitude = msg.lon
                    self.filtered_pos_msg.altitude = msg.alt
                    self.filtered_pos_msg.relative_altitude = msg.relative_alt
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
                    self.mission_request_buffer.append(msg.seq)

                elif msg_type == "STATUSTEXT":
                    rospy.loginfo("STATUSTEXT: Status severity is %d. Text Message is %s" % (msg.severity, msg.text))

                elif msg_type == "PARAM_VALUE":
                    rospy.loginfo("PARAM_VALUE: ID = %s, Value = %d, Type = %d, Count = %d, Index = %d"
                                  % (msg.param_id, msg.param_value, msg.param_type, msg.param_count, msg.param_index))


if __name__ == '__main__':
    try:
        MavRosProxy
        proxy = MavRosProxy(opts.device, opts.type, opts.baudrate, opts.SOURCE_SYSTEM)
        proxy.start()
    except rospy.ROSInterruptException:
        pass
