#!/usr/bin/env python
import rospy
import mavros.msg
import mavros.srv
from std_msgs.msg import String

CAMERA_PARAMETER = "CAM-RECORD_HORI"


class ROSNode:
    def __init__(self, prefix, timeouts=3, client_timeout=30):
        self.prefix = prefix
        self.execute = False
        self.manual = False
        self.last_manual = 0
        self.last_client = 0
        self.client_timeout = client_timeout
        self.extra = 0
        self.state = mavros.msg.State()
        #self.current = 0
        #self.last = 0
        #self.send = 0
        self.timeouts = timeouts
        self.queue = list()
        self.state = mavros.msg.State()
        self.mav_cmd = rospy.ServiceProxy(prefix + "command", mavros.srv.Command)
        self.mav_wps = rospy.ServiceProxy(prefix + "waypoints", mavros.srv.SendWaypoints)
        self.mav_params = rospy.ServiceProxy(prefix + "params", mavros.srv.Parameters)
        self.mav_rc = rospy.Publisher(prefix + 'send_rc', mavros.msg.RC, queue_size=10)
        self.next_pub = rospy.Publisher('next_instruction', mavros.msg.Instruction, queue_size=10)
        self.state_pub = rospy.Publisher('queue_state', mavros.msg.State, queue_size=10)
        self.params = dict()
        self.empty_RC = mavros.msg.RC()
        for i in range(len(self.empty_RC.channel)):
            self.empty_RC.channel[i] = 1500
        rospy.Subscriber("velocity", mavros.msg.Velocity, self.velocity_cb)
        rospy.Subscriber(prefix + "state", mavros.msg.State, self.update_queue_cb)

    def start(self):
        rospy.init_node("mosaic_node")
        rospy.loginfo("Requesting parameters and clearing waypoints...")
        params = self.mav_params([], [])
        self.mav_cmd(5, 0)
        rospy.Service("queue", mavros.srv.Queue, self.queue_cb)
        rospy.loginfo("Parameters received, Queue is ready.")
        for i in range(len(params.names)):
            self.params[params.names[i]] = params.values[i]
        while not rospy.is_shutdown():
            if self.manual:
                if rospy.Time.now().to_sec() - self.last_client > self.client_timeout:
                    rospy.loginfo("Client Timed Out! Trying to switch manual off.")
                    result = False
                    for i in range(self.timeouts):
                        result = self.mav_cmd(4, 0)
                    if result:
                        self.manual = False
                elif rospy.Time.now().to_sec() - self.last_manual > 0.2:
                    self.last_manual = rospy.Time.now().to_sec()
                    self.mav_rc.publish(self.empty_RC)
            elif self.execute and self.state.current+self.extra != len(self.queue):
                result = True
                for i in range(self.timeouts):
                    if self.queue[self.state.current+self.extra].type == mavros.msg.Instruction.TYPE_TAKEOFF:
                        result = self.mav_cmd([mavros.srv._Command.CommandRequest.CMD_TAKEOFF, 0])
                        self.extra += 1
                    elif self.queue[self.state.current+self.extra].type == mavros.msg.Instruction.TYPE_LAND:
                        result = self.mav_cmd([mavros.srv._Command.CommandRequest.CMD_LAND, 0])
                        self.extra += 1
                    if result:
                        break
                if not result:
                    rospy.loginfo("UNABLE TO EXECUTE! PAUSING QUEUE...")
                    self.extra -= 1
                    self.execute = False
                else:
                    result = False
                    for i in range(self.timeouts):
                        result = self.transmit_waypoints()
                        if result:
                            break
                    if not result:
                        rospy.loginfo("UNABLE TO TRANSMIT! PAUSING QUEUE...")
                        self.execute = False
            if self.state.current+self.extra >= len(self.queue):
                next_instruction = mavros.msg.Instruction()
            else:
                next_instruction = self.queue[self.state.current+self.extra]
            if len(self.queue) > 0:
                if self.manual:
                    self.state.base_mode = 3
                    #self.state.publish("In Manual")
                elif self.execute:
                    self.state.base_mode = 2
                    #self.state.publish("Executing")
                else:
                    self.state.base_mode = 0
                    #self.state.publish("Paused")
            else:
                if self.manual:
                    self.state.base_mode = 3
                    #self.state.publish("In Manual")
                elif self.execute:
                    self.state.base_mode = 1
                    #self.state.publish("Waiting")
                else:
                    self.state.base_mode = 0
                    #self.state.publish("Paused")
            self.next_pub.publish(next_instruction)
            self.state.header.stamp = rospy.Time.now()
            self.state.missions = len(self.queue)
            self.state_pub.publish(self.state)
            rospy.sleep(0.5)

    def velocity_cb(self, data):
        message = mavros.msg.RC()
        for i in range(4):
            message.channel[i] = data.velocity[i]*500 + 1500
        for i in range(4, 8):
            message.channel[i] = 1500
        self.last_manual = rospy.Time.now().to_sec()
        self.mav_rc.publish(message)
        self.last_client = self.last_manual

    def update_queue_cb(self, req):
        self.state = req

    def queue_cb(self, req):
        # print req
        if req.command == mavros.srv._Queue.QueueRequest.CMD_ADD:
            for i in req.instructions:
                self.queue.append(i)
            return True
        elif req.command == mavros.srv._Queue.QueueRequest.CMD_CLEAR:
            if not self.manual and self.mav_cmd(5, 0).result:
                self.queue = list()
                self.extra = 0
                return True
            return False
        elif req.command == mavros.srv._Queue.QueueRequest.CMD_PAUSE:
            if not self.execute:
                return True
            if self.mav_cmd(20, 0).result:
                self.execute = False
                return True
            return False
        elif req.command == mavros.srv._Queue.QueueRequest.CMD_EXECUTE:
            if self.execute:
                return True
            if self.mav_cmd(21, 0).result:
                self.execute = True
                return True
            return False
        elif req.command == mavros.srv._Queue.QueueRequest.CMD_MANUAL:
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
        elif req.command == mavros.srv._Queue.QueueRequest.CMD_AUTO:
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
        elif req.command == mavros.srv._Queue.QueueRequest.CMD_MANUAL_TAKEOFF:
            if not self.manual:
                return False
            result = False
            for i in range(self.timeouts):
                result = self.mav_cmd(1, 0).result
                if result:
                    break
            return result
        elif req.command == mavros.srv._Queue.QueueRequest.CMD_MANUAL_LAND:
            if not self.manual:
                return False
            result = False
            for i in range(self.timeouts):
                result = self.mav_cmd(2, 0).result
                if result:
                    break
            return result
        elif req.command == mavros.srv._Queue.QueueRequest.CMD_SWITCH_CAMERA:
            if self.params[CAMERA_PARAMETER] > 0:
                params = self.mav_params([CAMERA_PARAMETER], [0.0])
            else:
                params = self.mav_params([CAMERA_PARAMETER], [1.0])
            if CAMERA_PARAMETER == params.names[0] and params.values[0] != self.params[CAMERA_PARAMETER]:
                self.params[CAMERA_PARAMETER] = params.values[0]
                return True
            else:
                return False
        elif req.command == mavros.srv._Queue.QueueRequest.CMD_EMERGENCY:
            return self.mav_cmd(0, 100).result
        return False

    def transmit_waypoints(self):
        waypoints = list()
        for i in range(self.state.missions+self.extra, len(self.queue)):
            if self.queue[i].type != mavros.msg.Instruction.TYPE_GOTO:
                break
            waypoint_msg = mavros.msg.Waypoint()
            waypoint_msg.type = mavros.msg.Waypoint.TYPE_NAV
            waypoint_msg.frame = mavros.msg.Waypoint.TYPE_GLOBAL
            waypoint_msg.autocontinue = 1
            waypoint_msg.latitude = self.queue[i].latitude
            waypoint_msg.longitude = self.queue[i].longitude
            waypoint_msg.altitude = self.queue[i].altitude
            waypoint_msg.params = [self.queue[i].waitTime, self.queue[i].range, 0, 0]
            waypoints.append(waypoint_msg)
        if len(waypoints) > 0:
            if self.mav_wps(waypoints).result:
                return True
            else:
                return False
        return True


# *******************************************************************************
# Parse any arguments that follow the node command
# *******************************************************************************
from optparse import OptionParser

parser = OptionParser("mosaic_node.py [options]")
parser.add_option("-p", "--prefix", dest="prefix", default="/apm/",
                  help="prefix of the mavros node")
(opts, args) = parser.parse_args()

if __name__ == '__main__':
    try:
        rospy.wait_for_service(opts.prefix + "command")
        rospy.wait_for_service(opts.prefix + "waypoints")
        rospy.wait_for_service(opts.prefix + "params")
        node = ROSNode(opts.prefix)
        node.start()
    except rospy.ROSInterruptException:
        pass