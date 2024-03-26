#!/usr/bin/env python3

import tf
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

if __name__ == "__main__":

    rospy.init_node("visualisation_node", anonymous=True)
    pub = rospy.Publisher("/viz_topic", Marker, queue_size=10)
    listener = tf.TransformListener()

    marker = Marker()
    marker.header.frame_id = "base_footprint"
    marker.header.stamp = rospy.Time()
    marker.ns = "youbot"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    marker.scale.x = 1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    while not rospy.is_shutdown():

        try:
            (trans, rot) = listener.lookupTransform("base_footprint", "gripper_finger_link_l", rospy.Time.now())
            
            point = Point()
            point.x = trans[0]
            point.y = trans[1]
            point.z = trans[2]
            
            marker.points.append(point)

            pub.publish(marker)
            marker.points = []
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        


