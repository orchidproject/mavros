#!/usr/bin/env python
import rospy
import mavros.msg
import mavros.srv
import sys


def publish_waypt():
    rospy.init_node('waypoint_tester')

    wayptListProxy = rospy.ServiceProxy('waypoints', mavros.srv.SendWaypoints)

    while not rospy.is_shutdown():
        wayptListMsg = list()
        # Starting position should be 50.930042,-1.407951
        lat = [50.9297020, 50.9301280, 50.9302630, 50.9299370]
        lon = [-1.4081360, -1.4083990, -1.4077010, -1.4074140]
        # Populate waypoint list message
        for i in range(0, 5):
            wayptMsg = mavros.msg.Waypoint()
            wayptMsg.waypoint_type = mavros.msg.Waypoint.TYPE_NAV
            wayptMsg.autocontinue = 1
            wayptMsg.latitude = lat[i % 4]
            wayptMsg.longitude = lon[i % 4]
            wayptMsg.params = mavros.msg.Waypoint.DEFAULT
            wayptListMsg.append(wayptMsg)
        rospy.sleep(1.0)
        resp = wayptListProxy(wayptListMsg)
        print("Waypoints Sent. Response: " + str(resp.result))
        sys.exit()


if __name__ == '__main__':
    try:
        publish_waypt()
    except rospy.ROSInterruptException:
        pass
