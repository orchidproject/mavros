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
from socket import error

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))
import mavutil

from tools import *

UERE_CONSTANT = 45.5 / 9
NE_CONSTANT = 1
ADHOC_MANUAL = 99


class MavRosProxy:
    def __init__(self, name, device, baudrate, source_system=255, command_timeout=5, altitude_min=1, altitude_max=5):
        self.name = name
        self.device = device
        self.baudrate = baudrate
        self.source_system = source_system
        self.command_timeout = command_timeout
        self.connection = None
        # self.seq = 0
        self.mission_result = 0
        self.mission_ack = 0
        self.command_ack = 0
        self.list_ack = 0
        self.param_req = False

        self.altitude_min = altitude_min
        self.altitude_max = altitude_max

        # Global message containers
        self.state = mavros.msg.State()
        self.gps_msg = NavSatFix()
        self.filtered_pos_msg = mavros.msg.FilteredPosition()

        self.pub_gps = rospy.Publisher(self.name + '/gps', NavSatFix, queue_size=10)
        self.pub_imu = rospy.Publisher(self.name + '/imu', Imu, queue_size=10)
        # self.pub_rc = rospy.Publisher('rc', mavros.msg.RC, queue_size=10)
        self.pub_state = rospy.Publisher(self.name + '/state', mavros.msg.State, queue_size=10)
        # self.pub_vfr_hud = rospy.Publisher('vfr_hud', mavros.msg.VFR_HUD, queue_size=10)
        self.pub_attitude = rospy.Publisher(self.name + '/attitude', mavros.msg.Attitude, queue_size=10)
        self.pub_raw_imu = rospy.Publisher(self.name + '/raw_imu', mavros.msg.Mavlink_RAW_IMU, queue_size=10)
        self.pub_status = rospy.Publisher(self.name + '/status', mavros.msg.Status, queue_size=10)
        self.pub_filtered_pos = rospy.Publisher(self.name + '/filtered_pos', mavros.msg.FilteredPosition, queue_size=10)
        # self.pub_current_mission = rospy.Publisher('current_mission', mavros.msg.CurrentMission, queue_size=10)
        #self.pub_mission_item = rospy.Publisher('mission_item', mavros.msg.MissionItem, queue_size=10)
        rospy.Subscriber(self.name + "/send_rc", mavros.msg.RC, self.send_rc_cb)

    def send_rc_cb(self, req):
        self.connection.mav.rc_channels_override_send(self.connection.target_system,
                                                      self.connection.target_component,
                                                      req.channel[0], req.channel[1], req.channel[2],
                                                      req.channel[3],
                                                      req.channel[4], req.channel[5], req.channel[6],
                                                      req.channel[7])

    def param_list_cb(self, req):
        if len(req.values) > len(req.names):
            rospy.loginfo("[MAVROS:%s]Values must be less or equal to names" % self.name)
            return [], []
        if len(req.names) == 0:
            self.connection.param_fetch_complete = False
            self.connection.param_fetch_all()
            while not self.connection.param_fetch_complete:
                rospy.sleep(0.1)
            return self.connection.params.keys(), self.connection.params.values()

        values = list()
        for i in range(len(req.values)):
            start_time = rospy.Time.now().to_sec()
            rospy.loginfo("[MAVROS:%s]REQUESTED: " % self.name + req.names[i] + " " + str(req.values[i]))
            self.param_req = True
            self.connection.param_fetch_complete = False
            self.connection.param_set_send(req.names[i], req.values[i])
            while not self.connection.param_fetch_complete:
                rospy.sleep(0.1)
                if (rospy.Time.now().to_sec() - start_time) > self.command_timeout:
                    values.append(0.0)
                    break
            if self.connection.param_fetch_complete:
                values.append(self.connection.params[req.names[i]])
        return req.names, values

    def command_cb(self, req):
        start_time = rospy.Time.now().to_sec()
        if req.command == mavros.srv._Command.CommandRequest.CMD_TAKEOFF:
            if "LAND" not in self.connection.mode_mapping().keys():
                rospy.loginfo("[MAVROS:%s]This vehicle can not fly." % self.name)
                return False
            if self.state.custom_mode != self.connection.mode_mapping()["LAND"]:
                rospy.loginfo("[MAVROS:%s]Already in TAKEOFF" % self.name)
                return True

            self.connection.mav.command_long_send(self.connection.target_system, 0,
                                                  mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                                  0, 0, 0, 0, 0, 0, 0, 0)

            rospy.sleep(0.1)
            while self.state.custom_mode == self.connection.mode_mapping()["LAND"]:
                if rospy.Time.now().to_sec() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Timeout while trying to takeoff..." % self.name + str(self.state.custom_mode))
                    return False
                rospy.sleep(0.01)
            rospy.loginfo("[MAVROS:%s]Taken off" % self.name)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_LAND:
            if "LAND" not in self.connection.mode_mapping().keys():
                rospy.loginfo("[MAVROS:%s]This vehicle can not fly." % self.name)
                return False
            if self.state.custom_mode == self.connection.mode_mapping()["LAND"]:
                rospy.loginfo("[MAVROS:%s]Already in LANDED" % self.name)
                return True
            self.connection.mav.command_long_send(self.connection.target_system, 0,
                                                  mavutil.mavlink.MAV_CMD_NAV_LAND,
                                                  0, 0, 0, 0, 0, 0, 0, 0)
            rospy.sleep(0.1)
            while self.state.custom_mode != self.connection.mode_mapping()["LAND"]:
                if rospy.Time.now().to_sec() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Timeout while trying to land..." % self.name + str(self.state.custom_mode))
                    return False
                rospy.sleep(0.01)
            rospy.loginfo("[MAVROS:%s]Landed" % self.name)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_HALT:
            # self.connection.mav.mission_set_current_send(self.connection.target_system,
            # self.connection.target_component, 0)
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
        elif req.command == mavros.srv._Command.CommandRequest.CMD_COMMAND:
            self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
                                                  req.custom,
                                                  1, 0, 0, 0, 0, 0, 0, 0)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_MANUAL:
            if "MANUAL" not in self.connection.mode_mapping().keys():
                rospy.loginfo("[MAVROS:%s]This vehicle might not support manual, sending " % self.name + str(ADHOC_MANUAL))
                mode = ADHOC_MANUAL
            else:
                mode = self.connection.mode_mapping()["MANUAL"]
            if self.state.custom_mode == mode:
                rospy.loginfo("[MAVROS:%s]Already in MANUAL" % self.name)
                return True
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                              mode)
            rospy.sleep(0.1)
            while self.state.custom_mode != mode:
                if rospy.Time.now().to_sec() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Timeout while going to MANUAL..." % self.name)
                    return False
                rospy.sleep(0.01)
            rospy.loginfo("[MAVROS:%s]Switched to MANUAL" % self.name)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_AUTO:
            if "AUTO" not in self.connection.mode_mapping().keys():
                rospy.loginfo("[MAVROS:%s]This vehicle does not have auto. Sending mission start..." % self.name)
                self.connection.set_mode_auto()
                return True
            if self.state.custom_mode == self.connection.mode_mapping()["AUTO"]:
                rospy.loginfo("[MAVROS:%s]Already in AUTO" % self.name)
                return True
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                              self.connection.mode_mapping()["AUTO"])
            rospy.sleep(0.1)
            while self.state.custom_mode != self.connection.mode_mapping()["AUTO"]:
                if rospy.Time.now().to_sec() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Timeout while going to AUTO..." % self.name)
                    return False
                rospy.sleep(0.01)
            rospy.loginfo("[MAVROS:%s]Switched to AUTO" % self.name)
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_BASE_MODE:
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              req.custom, 0)
            rospy.loginfo("[MAVROS:%s]Base mode(%s)..." % (self.name, req.custom))
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_CUSTOM_MODE:
            self.connection.mav.set_mode_send(self.connection.target_system,
                                              mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, req.custom)
            rospy.loginfo("[MAVROS:%s]Custom mode(%s)..." % (self.name, req.custom))
            return True
        elif req.command == mavros.srv._Command.CommandRequest.CMD_CLEAR_WAYPOINTS:
            # self.seq = self.state.missions
            self.mission_result = -1
            self.connection.waypoint_clear_all_send()
            rospy.sleep(0.1)
            while self.mission_ack < start_time:
                if rospy.Time.now().to_sec() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Timeout while clearing waypoints..." % self.name)
                    # self.seq -= self.state.missions
                    return False
                rospy.sleep(0.01)
            if self.mission_result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                rospy.loginfo("[MAVROS:%s]Cleared waypoints" % self.name)
                self.state.current = 0
                self.state.missions = 0
                return True
            else:
                rospy.loginfo("[MAVROS:%s]Failed to clear waypoints[%d]" % (self.name, self.mission_result))
                return False
        return False

    def waypoint_list_cb(self, req):
        old = self.state.missions
        if old == 0:
            start_time = rospy.Time.now().to_sec()
            self.connection.waypoint_count_send(len(req.waypoints))
            while self.list_ack < start_time:
                if rospy.Time.now().to_sec() - start_time > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Time out on sending MISSION_COUNT" % self.name)
                    return False
        # Send entire list of waypoints
        for i in range(len(req.waypoints)):
            if req.waypoints[i].frame == mavros.msg.Waypoint.TYPE_GLOBAL:
                req.waypoints[i].frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            elif req.waypoints[i].frame == mavros.msg.Waypoint.TYPE_NED:
                req.waypoints[i].frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
            if req.waypoints[i].type == mavros.msg.Waypoint.TYPE_NAV:
                req.waypoints[i].type = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            elif req.waypoints[i].type == mavros.msg.Waypoint.TYPE_TAKEOFF:
                req.waypoints[i].type = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            elif req.waypoints[i].type == mavros.msg.Waypoint.TYPE_LAND:
                req.waypoints[i].type = mavutil.mavlink.MAV_CMD_NAV_LAND
            if not self.transmit_waypoint(req.waypoints[i]):
                rospy.loginfo("[MAVROS:%s]Failed to send %d'th waypoint" % (self.name, i))
                return False
        if old == 0:
            self.connection.mav.mission_set_current_send(self.connection.target_system,
                                                         self.connection.target_component, 0)
        elif self.state.current == 0:
            self.connection.mav.mission_set_current_send(self.connection.target_system,
                                                         self.connection.target_component, old + 1)
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
            while old != self.state.missions - 1:
                if (rospy.Time.now().to_sec() - start_time) > self.command_timeout:
                    rospy.loginfo("[MAVROS:%s]Timeout while sending waypoint..." % self.name)
                    return False
                if self.mission_ack > start_time:
                    break
            if self.mission_result != mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE:
                break
            # self.seq += 1
            rospy.sleep(0.1)
            break
        return old == (self.state.missions - 1)

    def start(self):
        while not rospy.is_shutdown():
            try:
                self.connection = mavutil.mavlink_connection(self.device, self.baudrate, self.source_system)
                break
            except error as e:
                rospy.logerr("[MAVROS]" + str(e) + "|" + str(self.device))
                rospy.sleep(1)
        rospy.loginfo("[MAVROS:%s]Waiting for Heartbeat..." % self.name)
        self.connection.wait_heartbeat()
        rospy.loginfo("[MAVROS:%s]Sleeping for a second to initialise..." % self.name)
        rospy.sleep(1)
        rospy.loginfo("[MAVROS:%s]Connected!" % self.name)
        rospy.Service(self.name + "/command", mavros.srv.Command, self.command_cb)
        rospy.Service(self.name + "/waypoints", mavros.srv.SendWaypoints, self.waypoint_list_cb)
        rospy.Service(self.name + "/params", mavros.srv.Parameters, self.param_list_cb)
        self.connection.waypoint_request_list_send()
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
                pass
                # self.pub_rc.publish([msg.chan1_raw, msg.chan2_raw, msg.chan3_raw,
                # msg.chan4_raw, msg.chan5_raw, msg.chan6_raw,
                # msg.chan7_raw, msg.chan8_raw])

            elif msg_type == "HEARTBEAT":
                self.state.base_mode = msg.base_mode
                self.state.custom_mode = msg.custom_mode
                self.state.header.stamp = rospy.Time.now()
                self.pub_state.publish(self.state)
                #rospy.loginfo("[MAVROS:%s] HB:(%d,%d)" % (self.name, msg.base_mode, msg.custom_mode))
                # self.connection.waypoint_request_list_send()

            elif msg_type == "VFR_HUD":
                pass
                # self.pub_vfr_hud.publish(msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt,
                # msg.climb)

            elif msg_type == "GPS_RAW_INT":
                fix = NavSatStatus.STATUS_NO_FIX
                if msg.fix_type >= 3:
                    fix = NavSatStatus.STATUS_FIX

                self.gps_msg.header.frame_id = 'base_link'
                self.gps_msg.header.stamp = rospy.Time.now()
                self.gps_msg.latitude = msg.lat / 1e07
                self.gps_msg.longitude = msg.lon / 1e07
                self.gps_msg.altitude = msg.alt / 1e07
                position_covariance = [0] * 9
                position_covariance[0] = UERE_CONSTANT * (msg.eph ** 2) + NE_CONSTANT ** 2
                position_covariance[4] = UERE_CONSTANT * (msg.eph ** 2) + NE_CONSTANT ** 2
                position_covariance[8] = UERE_CONSTANT * (msg.epv ** 2) + NE_CONSTANT ** 2
                self.gps_msg.position_covariance = position_covariance
                self.gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                self.gps_msg.status = NavSatStatus(status=fix, service=NavSatStatus.SERVICE_GPS)
                self.pub_gps.publish(self.gps_msg)

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
                self.filtered_pos_msg.header.stamp = rospy.Time.now()
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
                self.state.current = msg.seq
                # self.current_mission_msg.header.stamp = rospy.Time.now()
                # self.current_mission_msg.mission_num = msg.seq
                # self.pub_current_mission.publish(self.current_mission_msg)

            elif msg_type == "MISSION_ITEM":
                rospy.loginfo("[MAVROS:%s]%s" %(self.name, msg))
                # header = Header()
                # header.stamp = rospy.Time.now()
                # self.pub_mission_item.publish(header, msg.seq, msg.current,
                # msg.autocontinue, msg.param1,
                #                              msg.param2, msg.param3, msg.param4,
                #                              msg.x, msg.y, msg.z)

            elif msg_type == "MISSION_COUNT":
                self.state.missions = msg.count
                rospy.loginfo("[MAVROS:%s]MISSION_COUNT: Number of Mission Items - %s" % (self.name, str(msg.count)))

            elif msg_type == "MISSION_ACK":
                self.mission_ack = rospy.Time.now().to_sec()
                self.mission_result = msg.type
                rospy.loginfo("[MAVROS:%s]MISSION_ACK: Mission Message ACK with response - %s" % (self.name, str(msg.type)))

            elif msg_type == "COMMAND_ACK":
                self.command_ack = rospy.Time.now().to_sec()
                rospy.loginfo("[MAVROS:%s]COMMAND_ACK: Command Message ACK with result - %s" % (self.name, str(msg.result)))

            elif msg_type == "MISSION_REQUEST":
                self.list_ack = rospy.Time.now().to_sec()
                rospy.loginfo(
                    "[MAVROS:%s]MISSION_REQUEST: Mission Request for system %d for component %d with result %d"
                    % (self.name, msg.target_system, msg.target_component, msg.seq))

            elif msg_type == "STATUSTEXT":
                rospy.loginfo("[MAVROS:%s]STATUSTEXT: Status severity is %d. Text Message is %s" % (self.name, msg.severity, msg.text))

            elif msg_type == "PARAM_VALUE":
                if self.param_req:
                    self.connection.param_fetch_complete = True
                    self.param_req = False


# #******************************************************************************
# Parse any arguments that follow the node command
# *******************************************************************************
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
            MavRosProxy(opts.name, opts.device, opts.baudrate, opts.source_system, opts.command_timeout,
                    opts.minimum_altitude, opts.maximum_altitude).start()
    except rospy.ROSInterruptException:
        pass
