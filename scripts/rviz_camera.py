#!/usr/bin/env python
import math
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point32, PolygonStamped, Point


class RvizCameraNode:
    def __init__(self, name, frame_to="map", frame_from="Parrot_down"):
        self.name = name
        self.world_frame = frame_to
        self.camera_frame = frame_from
        self.prefix = "/" + name + "/"

    def start(self):
        rospy.init_node("rviz_camera")
        listener = tf.TransformListener()
        m = Marker()
        m.header.frame_id = self.world_frame
        m.ns = "camera"
        m.id = 0
        m.type = Marker.TRIANGLE_LIST
        m.lifetime = rospy.Duration()
        m.action = Marker.MODIFY
        m.scale.x = 1
        m.scale.y = 1
        m.scale.z = 1
        m.pose.position.x = 0
        m.pose.position.y = 0
        m.pose.position.z = 0.2
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1
        m.color.r = 0
        m.color.g = 1
        m.color.b = 0
        m.color.a = 1
        tl = (-1, 1, 0.5, 1)
        tr = (1, 1, 0.5, 1)
        dr = (1, -1, 0.5, 1)
        dl = (-1, -1, 0.5, 1)
        m.points.append(make_point(tl))
        m.points.append(make_point(tr))
        m.points.append(make_point(dl))
        m.points.append(make_point(dl))
        m.points.append(make_point(dr))
        m.points.append(make_point(tr))
        #poly = PolygonStamped()
        #poly.header.frame_id = self.world_frame
        #poly.polygon.points.append(Point32(x=-1, y=1, z=1))
        #poly.polygon.points.append(Point32(x=1, y=1, z=1))
        #poly.polygon.points.append(Point32(x=1, y=-1, z=1))
        #poly.polygon.points.append(Point32(x=-1, y=-1, z=1))
        #poly_pub = rospy.Publisher(self.prefix + "camera_view", PolygonStamped, queue_size=10)
        marker_pub = rospy.Publisher(self.prefix + "camera_view_marker", Marker, queue_size=10)
        while not rospy.is_shutdown():
            #poly.header.stamp = rospy.Time.now()
            #poly_pub.publish(poly)
            try:
                (trans, rot) = listener.lookupTransform("map", self.camera_frame, rospy.Time.from_sec(rospy.Time.now().to_sec()-1))
                mat = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))
                m.points = list()
                m.points.append(make_point(mat.dot(tl)))
                m.points.append(make_point(mat.dot(tr)))
                m.points.append(make_point(mat.dot(dl)))
                m.points.append(make_point(mat.dot(dl)))
                m.points.append(make_point(mat.dot(dr)))
                m.points.append(make_point(mat.dot(tr)))
                marker_pub.publish(m)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print e
            #m.header.stamp = rospy.Time.now()
            #marker_pub.publish(m)
            rospy.sleep(1)
        
def make_point(data):
    return Point(x=data[0], y=data[1], z=data[2])

# *******************************************************************************
# Parse any arguments that follow the node command
# *******************************************************************************
from optparse import OptionParser

parser = OptionParser("mosaic_node.py [options]")
parser.add_option("-n", "--name", dest="name", default="parrot",
                  help="Name of the prefix for the mavros node")
(opts, args) = parser.parse_args()

if __name__ == '__main__':
    try:
        node = RvizCameraNode(opts.name)
        node.start()
    except rospy.ROSInterruptException:
        pass
