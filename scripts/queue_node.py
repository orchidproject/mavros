#!/usr/bin/env python
import math

import rospy, tf, numpy
import mavros.srv, mavros.msg
from utm import from_latlon, to_latlon
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Vector3, PoseWithCovariance, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from uav_utils import sweeps


CAMERA_PARAMETER = "CAM-RECORD_HORI"

CMD_EXECUTE = 1
CMD_PAUSE = 2
CMD_CLEAR = 5
CMD_AUTO = 11
CMD_MANUAL = 12
CMD_MANUAL_TAKEOFF = 15
CMD_MANUAL_LAND = 16
CMD_SWITCH_CAMERA = 20
CMD_CALIBRATE_YAW = 30
CMD_EMERGENCY = 100

YAW_CALIBRATION_N = 20


class QueueNode:
    def __init__(self, name, timeouts=1, client_timeout=30):
        self.name = name
        self.prefix = "/" + name + "/"
        self.execute = False
        self.manual = False
        self.last_manual = 0
        self.last_client = 0
        self.client_timeout = client_timeout
        self.send = 0
        self.origin = None
        self.sending = False
        self.yaw_offset = 0
        self.yaw_counter = 0
        self.last_heading = 0
        self.timeouts = timeouts
        self.last_alt = 0

    def ros_init(self):
        # Message containers
        self.state = mavros.msg.State()
        self.empty_RC = mavros.msg.RC()
        for i in range(len(self.empty_RC.channel)):
            self.empty_RC.channel[i] = 1500
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu"
        self.imu_msg.orientation_covariance = (1, 0, 0,
                                               0, 1, 0,
                                               0, 0, 1)
        self.imu_msg.angular_velocity_covariance = (9999, 0, 0,
                                                    0, 9999, 0,
                                                    0, 0, 9999)
        self.imu_msg.linear_acceleration_covariance = (9999, 0, 0,
                                                       0, 9999, 0,
                                                       0, 0, 9999)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "odom"
        self.odom_msg.pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.odom_msg.pose.covariance = (1, 0, 0, 0, 0, 0,
                                         0, 1, 0, 0, 0, 0,
                                         0, 0, 1, 0, 0, 0,
                                         0, 0, 0, 9999, 0, 0,
                                         0, 0, 0, 0, 9999, 0,
                                         0, 0, 0, 0, 0, 9999)
        self.odom_msg.twist.covariance = (9999, 0, 0, 0, 0, 0,
                                          0, 9999, 0, 0, 0, 0,
                                          0, 0, 9999, 0, 0, 0,
                                          0, 0, 0, 9999, 0, 0,
                                          0, 0, 0, 0, 9999, 0,
                                          0, 0, 0, 0, 0, 9999)
        # Local variables
        self.queue = list()
        self.params = dict()
        # ROS topics and services
        self.mav_cmd = rospy.ServiceProxy(self.prefix + "command", mavros.srv.Command)
        self.mav_wps = rospy.ServiceProxy(self.prefix + "waypoints", mavros.srv.SendWaypoints)
        self.mav_params = rospy.ServiceProxy(self.prefix + "params", mavros.srv.Parameters)
        rospy.loginfo("[QUEUE:%s]Requesting parameters and clearing waypoints..." % self.name)
        local_params = rospy.get_param("/drone_params")
        self.front_camera = rospy.get_param("/" + self.name + "/front_camera")
        self.bottom_camera = rospy.get_param("/" + self.name + "/bottom_camera")
        params = self.mav_params(local_params.keys(), local_params.values())
        self.mav_cmd(5, 0)
        for i in range(len(params.names)):
            self.params[params.names[i]] = params.values[i]
        rospy.loginfo("[QUEUE:%s]Parameters received, Queue is ready." % self.name)
        self.mav_rc = rospy.Publisher(self.prefix + 'send_rc', mavros.msg.RC, queue_size=10)
        self.next_pub = rospy.Publisher(self.prefix + 'queue/next_instruction', mavros.msg.Instruction, queue_size=10)
        self.state_pub = rospy.Publisher(self.prefix + 'queue/state', mavros.msg.State, queue_size=10)
        self.imu_pub = rospy.Publisher(self.prefix + "queue/imu_data", Imu, queue_size=10)
        self.odom_pub = rospy.Publisher(self.prefix + "queue/odom", Odometry, queue_size=10)
        self.transform_pub = tf.TransformBroadcaster()
        self.transform_pub2 = tf.TransformBroadcaster()
        rospy.Service(self.prefix + "queue/cmd", mavros.srv.Queue, self.queue_cb)
        rospy.Subscriber(self.prefix + "queue/velocity", mavros.msg.Velocity, self.velocity_cb)
        rospy.Subscriber(self.prefix + "state", mavros.msg.State, self.update_queue_cb)
        rospy.Subscriber(self.prefix + "queue/instructions", mavros.msg.InstructionList, self.instructions_cb)
        rospy.Subscriber(self.prefix + "attitude", mavros.msg.Attitude, self.translate_attitude)
        rospy.Subscriber(self.prefix + "filtered_pos", mavros.msg.FilteredPosition, self.translate_pos)
        rospy.Subscriber(self.prefix + "queue/odom_combined", PoseWithCovarianceStamped, self.publish_transforms)
        # # RVIZ
        self.marker_pub = rospy.Publisher(self.prefix + "queue/camera_view_marker", Marker, queue_size=10)
        self.m = Marker()
        self.m.header.frame_id = "map"
        self.m.ns = "camera"
        self.m.id = 0
        self.m.type = Marker.TRIANGLE_LIST
        self.m.lifetime = rospy.Duration()
        self.m.action = Marker.MODIFY
        self.m.scale.x = 1
        self.m.scale.y = 1
        self.m.scale.z = 1
        self.m.pose.position.x = 0
        self.m.pose.position.y = 0
        self.m.pose.position.z = 0
        self.m.pose.orientation.x = 0
        self.m.pose.orientation.y = 0
        self.m.pose.orientation.z = 0
        self.m.pose.orientation.w = 1
        self.m.color.r = 0
        self.m.color.g = 1
        self.m.color.b = 0
        self.m.color.a = 1
        self.arrow = Marker()
        self.arrow.header.frame_id = "map"
        self.arrow.ns = "camera"
        self.arrow.id = 1
        self.arrow.type = Marker.ARROW
        self.arrow.lifetime = rospy.Duration()
        self.arrow.action = Marker.MODIFY
        self.arrow.scale.x = 1
        self.arrow.scale.y = 1
        self.arrow.scale.z = 1
        self.arrow.pose.position.x = 0
        self.arrow.pose.position.y = 0
        self.arrow.pose.position.z = 0
        self.arrow.pose.orientation.x = 0
        self.arrow.pose.orientation.y = 0
        self.arrow.pose.orientation.z = 0
        self.arrow.pose.orientation.w = 1
        self.arrow.color.r = 1
        self.arrow.color.g = 0
        self.arrow.color.b = 0
        self.arrow.color.a = 1

    def start(self):
        self.ros_init()
        while not rospy.is_shutdown():
            if self.manual:
                if rospy.Time.now().to_sec() - self.last_client > self.client_timeout:
                    rospy.loginfo("[QUEUE:%s]Client Timed Out! Trying to switch manual off." % self.name)
                    result = False
                    for i in range(self.timeouts):
                        result = self.mav_cmd(4, 0)
                    if result:
                        self.manual = False
                elif rospy.Time.now().to_sec() - self.last_manual > 0.2:
                    self.last_manual = rospy.Time.now().to_sec()
                    self.mav_rc.publish(self.empty_RC)
            elif self.execute and len(self.queue) > 0 and (self.queue[0].type == mavros.msg.Instruction.TYPE_TAKEOFF \
                                                                   or self.queue[
                    0].type == mavros.msg.Instruction.TYPE_LAND):
                result = False
                for i in range(self.timeouts):
                    if self.queue[0].type == mavros.msg.Instruction.TYPE_TAKEOFF:
                        result = self.mav_cmd(mavros.srv._Command.CommandRequest.CMD_TAKEOFF, 0)
                    elif self.queue[0].type == mavros.msg.Instruction.TYPE_LAND:
                        result = self.mav_cmd(mavros.srv._Command.CommandRequest.CMD_LAND, 0)
                    if result:
                        break
                if not result:
                    rospy.loginfo("[QUEUE:%s]UNABLE TO EXECUTE! PAUSING QUEUE..." % self.name)
                    self.execute = False
                else:
                    rospy.sleep(self.queue[0].waitTime)
                    rospy.loginfo("[QUEUE:%s]De queued %s" % (self.name, str(self.queue[0].type)))
                    self.queue.pop(0)
                    result = False
                    for i in range(self.timeouts):
                        result = self.transmit_waypoints()
                        if result:
                            break
                    if not result:
                        rospy.loginfo("[QUEUE:%s]UNABLE TO TRANSMIT! PAUSING QUEUE..." % self.name)
                        self.execute = False
            elif self.execute:
                if not self.transmit_waypoints():
                    rospy.loginfo("[QUEUE:%s]UNABLE TO TRANSMIT! PAUSING QUEUE..." % self.name)
                    self.execute = False
            if len(self.queue) == 0:
                next_instruction = mavros.msg.Instruction()
            else:
                next_instruction = self.queue[0]
            if len(self.queue) > 0:
                if self.manual:
                    self.state.base_mode = 3
                    # self.state.publish("In Manual")
                elif self.execute:
                    self.state.base_mode = 2
                    # self.state.publish("Executing")
                else:
                    self.state.base_mode = 0
                    # self.state.publish("Paused")
            else:
                if self.manual:
                    self.state.base_mode = 3
                    # self.state.publish("In Manual")
                elif self.execute:
                    self.state.base_mode = 1
                    # self.state.publish("Waiting")
                else:
                    self.state.base_mode = 0
                    # self.state.publish("Paused")
            self.next_pub.publish(next_instruction)
            self.state.header.stamp = rospy.Time.now()
            self.state.missions = len(self.queue)
            self.state_pub.publish(self.state)
            rospy.sleep(0.5)

    def velocity_cb(self, data):
        message = mavros.msg.RC()
        for i in range(4):
            message.channel[i] = data.velocity[i] * 500 + 1500
        for i in range(4, 8):
            message.channel[i] = 1500
        self.last_manual = rospy.Time.now().to_sec()
        self.mav_rc.publish(message)
        self.last_client = self.last_manual

    def update_queue_cb(self, req):
        while self.state.current < req.current:
            if len(self.queue):
                rospy.loginfo("[QUEUE:%s]De queued %s" % (self.name, str(self.queue[0].type)))
                self.queue.pop(0)
                self.send -= 1
            self.state.current += 1
        if req.current == 0 and ((self.state.current + 1) == req.missions):
            rospy.loginfo("[QUEUE:%s]De queued last waypoint" % self.name)
            self.queue.pop(0)
            self.send -= 1


        # if not self.sending:
        #     if self.state.current < req.current:
        #         if self.state.current == 0:
        #             self.state.current = 1
        #         while self.state.current < req.current:
        #             if len(self.queue):
        #                 rospy.loginfo("[QUEUE:%s]De queued %s" % (self.name, str(self.queue[0].type)))
        #                 self.queue.pop(0)
        #                 self.send -= 1
        #             self.state.current += 1
        #     elif req.current == 0 and ((self.state.current + 1) == req.missions):
        #         rospy.loginfo("[QUEUE:%s]De queued %s" % (self.name, str(self.queue[0].type)))
        #         self.queue.pop(0)
        #         self.send -= 1
        #         self.state.current = req.current

    def instructions_cb(self, req):
        rospy.loginfo("[QUEUE:%s] Adding %d instruction" % (self.name, len(req.inst)))
        while len(req.inst) > 0:
            head = req.inst.pop(0)
            if head.type == mavros.msg.Instruction.TYPE_SET_ORIGIN:
                self.origin = from_latlon(head.latitude, head.longitude)
                print head.latitude, ":", head.longitude
                rospy.loginfo("[QUEUE:%s]Origin set to %s" % (self.name, str(self.origin)))
            elif head.type == mavros.msg.Instruction.TYPE_SPIRAL_SWEEP or head.type == mavros.msg.Instruction.TYPE_RECT_SWEEP:
                if len(req.inst) < 1 or (req.inst[0].type != mavros.msg.Instruction.TYPE_SPIRAL_SWEEP and req.inst[
                    0].type != mavros.msg.Instruction.TYPE_RECT_SWEEP):
                    rospy.loginfo("[QUEUE:%s]Require start and end point of sweep to be after another" % self.name)
                elif head.frame != mavros.msg.Instruction.FRAME_LOCAL or not self.origin:
                    rospy.loginfo("[QUEUE:%s]Sweeps can't be in global frame or have not set up origin" % self.name)
                    return
                else:
                    end = req.inst.pop(0)
                    points = list()
                    if head.type == mavros.msg.Instruction.TYPE_SPIRAL_SWEEP:
                        points = sweeps.spiral_sweep((head.latitude, head.longitude), (end.latitude, end.longitude),
                                                     head.waitTime, head.range)
                    elif head.type == mavros.msg.Instruction.TYPE_RECT_SWEEP:
                        points = sweeps.rect_sweep((head.latitude, head.longitude), (end.latitude, end.longitude),
                                                   head.waitTime, head.range)
                    n = len(points)
                    rospy.loginfo("[QUEUE:%s]Adding %d instructions for sweep" % (self.name, n))
                    for i in range(n):
                        temp = mavros.msg.Instruction()
                        temp.type = mavros.msg.Instruction.TYPE_GOTO
                        temp.frame = mavros.msg.Instruction.FRAME_LOCAL
                        temp.waitTime = end.waitTime if (i == (n - 1)) else 0
                        temp.range = end.range
                        temp.latitude = points[i][0]
                        temp.longitude = points[i][1]
                        temp.altitude = head.altitude + i * (head.altitude - end.altitude) / (n - 1)
                        self.queue.append(temp)
            elif head.type == mavros.msg.Instruction.TYPE_GOTO:
                if head.frame == mavros.msg.Instruction.FRAME_LOCAL and not self.origin:
                    rospy.loginfo("[QUEUE:%s]Don't accept UTM coordinates without setting origin" % self.name)
                elif head.frame == mavros.msg.Instruction.FRAME_LOCAL or head.frame == mavros.msg.Instruction.FRAME_GLOBAL:
                    self.queue.append(head)
            elif head.type == mavros.msg.Instruction.TYPE_LAND or mavros.msg.Instruction.TYPE_TAKEOFF:
                self.queue.append(head)

    def translate_attitude(self, msg):
        if self.yaw_counter > 0:
            self.yaw_offset += msg.yaw
            self.yaw_counter -= 1
        else:
            msg.yaw += self.yaw_offset
        #msg.yaw = -math.pi/2
        self.imu_msg.header.stamp = msg.header.stamp
        q  = tf.transformations.quaternion_from_euler(msg.roll, -msg.pitch, -msg.yaw)
        self.imu_msg.orientation = Quaternion(q[0], q[1], q[2], q[3])
        self.imu_pub.publish(self.imu_msg)

    def translate_pos(self, msg):
        self.last_heading = msg.heading
        (x, y, zone_n, zone_l) = from_latlon(msg.latitude, msg.longitude)
        if self.origin and zone_n == self.origin[2] and zone_l == self.origin[3]:
            self.odom_msg.header.stamp = msg.header.stamp
            self.last_alt = msg.relative_altitude
            self.odom_msg.pose.pose.position = Point(x - self.origin[0], y - self.origin[1], 0)
            #self.odom_msg.twist.twist.linear = Vector3(msg.ground_x_speed, msg.ground_y_speed, msg.ground_z_speed)
            self.odom_pub.publish(self.odom_msg)

    def publish_transforms(self, msg):
        self.transform_pub.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, self.last_alt),
                                         (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                                         rospy.Time.now(), self.name + "_base", "map")

        # Transform camera points to Base points
        mat = tf.transformations.compose_matrix(translate=(0, 0, 0.02), angles=(0, math.pi, math.pi / 2))
        center = mat.dot((0, 0, 0, 1))
        pixel = (639, 0);
        pn = self.normalise_pixel(pixel)
        #print pn
        #camera_patch = [(-1, 1, 0.5, 1), (1, 1, 0.5, 1), (1, -1, 0.5, 1), (-1, -1, 0.5, 1)]
        #camera_patch = [(-0.625, 0.35, 1.34, 1), (0.625, 0.35, 1, 1), (0.625, -0.35, 1, 1), (-0.625, -0.35, 1, 1)]
        camera_patch = [self.normalise_pixel((0, 319)), self.normalise_pixel((639, 319)),
                        self.normalise_pixel((639, 0)), self.normalise_pixel((0, 0))]
        for i in range(len(camera_patch)):
            camera_patch[i] = mat.dot(camera_patch[i])
        #Transfrom Base points to World points
        mat = tf.transformations.compose_matrix(
            translate=(msg.pose.pose.position.x, msg.pose.pose.position.y, self.last_alt - 0.02),
            angles=tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)))
        center = mat.dot(center)
        for i in range(len(camera_patch)):
            camera_patch[i] = mat.dot(camera_patch[i])
            #print(str(center) +  ":" + str(camera_patch[i]))
            t = center[2] / (camera_patch[i][2] - center[2])
            camera_patch[i] = numpy.add(center, numpy.multiply(t, numpy.subtract(center, camera_patch[i])))
            #print(str(camera_patch[i]) + ":" + str(t))
            #print camera_patch[i][3], ": ", center[3], ": ", t, ": ",
        self.m.points = triangulate_points(camera_patch)
        self.marker_pub.publish(self.m)

    def normalise_pixel(self, x, camera=0):
        cp = self.bottom_camera
        if camera == 1:
            cp = self.front_camera
        xp = ((x[0] - cp["K"][2]) / cp["K"][0], (x[1] - cp["K"][5])/cp["K"][4])
        #print "init: ", xp
        error = 1
        k = cp["D"]
        while error > 0.001:
            r2 = xp[0]**2 + xp[1]**2
            k_radial = 1 + k[0] * r2 + k[1] * r2 * r2 + k[4] * r2 * r2 * r2
            delta_x = 2 * k[2] * xp[0] * xp[1] + (r2 + 2 * xp[0] * xp[0]) * k[3]
            delta_y = 2 * k[3] * xp[0] * xp[1] + (r2 + 2 * xp[1] * xp[1]) * k[2]
            xp_n = ((xp[0] - delta_x) * k_radial, (xp[1] - delta_y) * k_radial)
            error = (xp_n[0] - xp[0])**2 + (xp_n[1] - xp[1])**2
            xp = xp_n
        #print "Final: ", error
        return xp[0], xp[1], 1, 1


    def queue_cb(self, req):
        if req.command == CMD_CLEAR:
            if not self.manual and self.mav_cmd(5, 0).result:
                self.queue = list()
                self.send = 0
                # self.extra = 0
                rospy.loginfo("[QUEUE:%s]Queue cleared" % self.name)
                return True
            rospy.loginfo("[QUEUE:%s]Queue failed to clear" % self.name)
            return False
        elif req.command == CMD_PAUSE:
            if not self.execute:
                return True
            if self.mav_cmd(20, 0).result:
                self.execute = False
                rospy.loginfo("[QUEUE:%s]Queue paused" % self.name)
                return True
            rospy.loginfo("[QUEUE:%s]Queue failed to pause" % self.name)
            return False
        elif req.command == CMD_EXECUTE:
            # if self.execute:
            #     return True
            if self.mav_cmd(21, 0).result:
                self.execute = True
                rospy.loginfo("[QUEUE:%s]Starting execution of Queue" % self.name)
                return True
            rospy.loginfo("[QUEUE:%s]Failed to start execution of Queue" % self.name)
            return False
        elif req.command == CMD_MANUAL:
            self.last_client = rospy.Time.now().to_sec()
            if self.manual:
                return True
            result = False
            for i in range(self.timeouts):
                result = self.mav_cmd(3, 0).result
                if result:
                    break
            if result:
                self.manual = True
                return True
            else:
                return False
        elif req.command == CMD_AUTO:
            if not self.manual:
                return True
            result = False
            for i in range(self.timeouts):
                result = self.mav_cmd(4, 0).result
                if result:
                    break
            if result:
                self.manual = False
                return True
            else:
                return False
        elif req.command == CMD_MANUAL_TAKEOFF:
            # if not self.manual:
            # return False
            result = False
            for i in range(self.timeouts):
                result = self.mav_cmd(1, 0).result
                if result:
                    break
            return result
        elif req.command == CMD_MANUAL_LAND:
            # if not self.manual:
            # return False
            result = False
            for i in range(self.timeouts):
                result = self.mav_cmd(2, 0).result
                if result:
                    break
            return result
        elif req.command == CMD_SWITCH_CAMERA:
            if self.params[CAMERA_PARAMETER] > 0:
                params = self.mav_params([CAMERA_PARAMETER], [0.0])
            else:
                params = self.mav_params([CAMERA_PARAMETER], [1.0])
            if CAMERA_PARAMETER == params.names[0] and params.values[0] != self.params[CAMERA_PARAMETER]:
                self.params[CAMERA_PARAMETER] = params.values[0]
                return True
            else:
                return False
        elif req.command == CMD_EMERGENCY:
            return self.mav_cmd(0, 100).result
        elif req.command == CMD_CALIBRATE_YAW:
            if self.last_heading == 0:
                return False
            start_time = rospy.Time.now().to_sec()
            self.yaw_offset = 0
            self.yaw_counter = YAW_CALIBRATION_N
            while self.yaw_counter > 0:
                if rospy.Time.now().to_sec() - start_time > self.client_timeout:
                    rospy.loginfo("[QUEUE:%s]Timeout while calibrating yaw" % self.name)
                    return False
                rospy.sleep(0.1)
            self.yaw_offset = math.radians(self.last_heading) - self.yaw_offset / YAW_CALIBRATION_N
            rospy.loginfo("[QUEUE:%s]Yaw calibration complete" % self.name)
            return True
        return False

    def transmit_waypoints(self):
        waypoints = list()
        for i in range(self.send, len(self.queue)):
            if self.queue[i].type != mavros.msg.Instruction.TYPE_GOTO:
                break
            waypoint_msg = mavros.msg.Waypoint()
            if self.queue[i].frame == mavros.msg.Instruction.FRAME_LOCAL:
                # print "LOCAL(%.2f,%.2f)" % (self.queue[i].latitude, self.queue[i].longitude)
                waypoint_msg.latitude, waypoint_msg.longitude = to_latlon(self.queue[i].latitude + self.origin[0],
                                                                          self.queue[i].longitude + self.origin[1],
                                                                          self.origin[2],
                                                                          self.origin[3])
                # print waypoint_msg.latitude, ":", waypoint_msg.longitude
            else:
                waypoint_msg.latitude = self.queue[i].latitude
                waypoint_msg.longitude = self.queue[i].longitude
            waypoint_msg.type = mavros.msg.Waypoint.TYPE_NAV
            waypoint_msg.frame = mavros.msg.Waypoint.TYPE_GLOBAL
            waypoint_msg.autocontinue = 1
            waypoint_msg.altitude = self.queue[i].altitude
            waypoint_msg.params = [self.queue[i].waitTime, self.queue[i].range, 0, 0]
            waypoints.append(waypoint_msg)
        if len(waypoints) > 0:
            #self.sending = True
            self.mav_cmd(20, 0)
            rospy.sleep(1)
            self.execute = False
            if self.mav_wps(waypoints).result:
                self.send += len(waypoints)
                #self.sending = False
                rospy.sleep(1)
                self.mav_cmd(21, 0)
                self.execute = True
                return True
            else:
                rospy.loginfo("Could not transmit waypoints")
                self.execute = True
                return False
        return True


def triangulate_points(points):
    points_list = list()
    while len(points) > 2:
        points_list.append(
            Point(x=points[len(points) - 1][0], y=points[len(points) - 1][1], z=points[len(points) - 1][2]))
        points_list.append(Point(x=points[0][0], y=points[0][1], z=points[0][2]))
        points_list.append(Point(x=points[1][0], y=points[1][1], z=points[1][2]))
        points.pop(0)
    return points_list


# *******************************************************************************
# Parse any arguments that follow the node command
# *******************************************************************************
from optparse import OptionParser

parser = OptionParser("mosaic_node.py [options]")
parser.add_option("-n", "--name", dest="name", default="parrot",
                  help="Name of the prefix for the mavros node")
parser.add_option("-r", "--ros", action="store_true", dest="ros", help="Use ROS parameter server", default=False)
(opts, args) = parser.parse_args()

if __name__ == '__main__':
    try:
        if not opts.ros or opts.name in rospy.get_param("/drones_active"):
            rospy.wait_for_service("/" + opts.name + "/command")
            rospy.wait_for_service("/" + opts.name + "/waypoints")
            rospy.wait_for_service("/" + opts.name + "/params")
            rospy.init_node("queue_node")
            node = QueueNode(opts.name)
            node.start()
    except rospy.ROSInterruptException:
        pass