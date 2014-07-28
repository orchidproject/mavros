import rospy
import mavros.msg
import mavros.srv
from std_msgs.msg import String


class ROSNode:
    def __init__(self, prefix):
        self.prefix = prefix
        self.execute = False
        self.queue = list()
        self.mav_cmd = rospy.ServiceProxy(prefix + "command", mavros.srv.Command)
        self.mav_wps = rospy.ServiceProxy(prefix + "waypoints", mavros.srv.SendWaypoints)
        self.mav_rc = rospy.Publisher(prefix + 'send_rc', mavros.msg.RC)
        self.next = rospy.Publisher('next_instruction', mavros.msg.Instruction)
        self.state = rospy.Publisher('queue_state', String)
        rospy.Service("queue", mavros.srv.Queue, self.queue_cb)

    def start(self):
        rospy.init_node("mosaic_node", anonymous=True)
        while not rospy.is_shutdown():
            if self.execute:
                #TODO
                pass
            else:
                rospy.sleep(0.5)
            if len(self.queue) > 0:
                self.next.publish(self.queue[0])
                if self.execute:
                    self.state.publish("Executing")
                else:
                    self.state.publish("Paused")
            else:
                self.next.publish([0, 0, 0, 0, 0, 0])
                if self.execute:
                    self.state.publish("Waiting")
                else:
                    self.state.publish("Paused")

    def queue_cb(self, req):
        if req.command == mavros.srv._Queue.QueueRequest.CMD_ADD:
            self.queue.append(mavros.msg.Instruction(req))
            return True
        elif req.command == mavros.srv._Queue.QueueRequest.CMD_CLEAR:
            if self.mav_cmd(5, 0):
                self.queue = list()
                return True
            return False
        elif req.command == mavros.srv._Queue.QueueRequest.CMD_PAUSE:
            if not self.execute:
                return True
            if self.mav_cmd(5, 0):
                self.execute = False
                return True
            return False
        elif req.command == mavros.srv._Queue.QueueRequest.CMD_EXECUTE:
            #TODO
            pass

# *******************************************************************************
# Parse any arguments that follow the node command
# *******************************************************************************
from optparse import OptionParser

parser = OptionParser("rosnode.py [options]")
parser.add_option("--prefix", dest="prefix", default="/apm/",
                  help="prefix of the mavros node")
(opts, args) = parser.parse_args()

if __name__ == '__main__':
    try:
        node = ROSNode(opts.prefix)
        node.start()
    except rospy.ROSInterruptException:
        pass