#!/usr/bin/env python
import rospy
import roscopter.msg
import roscopter.srv
import sys 


def publish_waypt():
    rospy.init_node('waypoint_tester')
	
    wayptListProxy = rospy.ServiceProxy('waypoint_list', roscopter.srv.SendWaypointList)

    while not rospy.is_shutdown():
        wayptListMsg = roscopter.msg.WaypointList()
	# Starting position should be 50.930042,-1.407951
	lat = [509297020, 509301280, 509302630, 509299370]
	lon = [-14081360, -14083990, -14077010, -14074140]
        # Populate waypoint list message
        for i in range (0,5):
            wayptMsg = roscopter.msg.Waypoint()
            wayptMsg.latitude = lat[i%4]
            wayptMsg.longitude = lon[i%4]
            wayptMsg.altitude = 5E3
            wayptMsg.pos_acc = 10
            wayptMsg.speed_to = 1E2
 	    wayptMsg.hold_time = 0
            wayptMsg.yaw_from = 0
	    wayptMsg.pan_angle = 0
	    wayptMsg.tilt_angle = 0
            wayptMsg.waypoint_type = roscopter.msg.Waypoint.TYPE_NAV
            wayptListMsg.waypoints.append(wayptMsg)
        rospy.sleep(1.0)
	req = roscopter.srv.SendWaypointListRequest(wayptListMsg)
        resp = wayptListProxy(waypoints=wayptListMsg.waypoints)
        print("Waypoints Sent. Response: " + str(resp.result))
	sys.exit()
        rospy.sleep(10.0)

if __name__ == '__main__':
    try:
        publish_waypt()
    except  rospy.ROSInterruptException:
        pass
