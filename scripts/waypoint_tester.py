#!/usr/bin/env python
import rospy
import mavros.msg
import mavros.srv
import sys


def publish_waypoints(n):
    if n == 1:
        start = 0
        inc = 1
    elif n == 2:
        start = 3
        inc = -1
    rospy.init_node('waypoint_tester')
    queue = rospy.ServiceProxy('/mosaic/queue', mavros.srv.Queue)
    while not rospy.is_shutdown():
        # Starting position should be 50.930042,-1.407951
        lat = [50.9297020, 50.9301280, 50.9302630, 50.9299370]
        lon = [-1.4081360, -1.4083990, -1.4077010, -1.4074140]
        instructions = list()
        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_TAKEOFF
        instructions.append(i)

        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_GOTO
        i.latitude = lat[start]
        i.longitude = lon[start]
        i.altitude = 3
        instructions.append(i)
        start += inc

        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_GOTO
        i.latitude = lat[start]
        i.longitude = lon[start]
        i.altitude = 3
        instructions.append(i)
        start += inc

        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_LAND
        instructions.append(i)

        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_TAKEOFF
        instructions.append(i)

        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_GOTO
        i.latitude = lat[start]
        i.longitude = lon[start]
        i.altitude = 3
        instructions.append(i)
        start += inc

        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_GOTO
        i.latitude = lat[start]
        i.longitude = lon[start]
        i.altitude = 3
        instructions.append(i)
        start += inc

        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_LAND
        instructions.append(i)

        resp = queue(1, instructions)
        print("Waypoints Sent. Response: " + str(resp.result))
        sys.exit()

# *******************************************************************************
# Parse any arguments that follow the node command
# *******************************************************************************
from optparse import OptionParser

parser = OptionParser("mosaic_node.py [options]")
parser.add_option("-n", "--number", dest="n", default=1,
                  help="Number of drone")
(opts, args) = parser.parse_args()

if __name__ == '__main__':
    try:
        publish_waypoints(opts.n)
    except rospy.ROSInterruptException:
        pass
