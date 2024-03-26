#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

class Navigation:
    def __init__(self,points):
        self.points = points
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('rrt_path', Marker, queue_size=10)
        self.current_pose = [0,0]
        rospy.Subscriber('odom',Odometry,self.odom_callback)
        self.rate = rospy.Rate(1) 
        
    def publish_base_path(self):
        pub = rospy.Publisher('rrt_path', Marker, queue_size=10)
        
        marker = Marker()
        marker.header.frame_id = "odom"  
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.g = 1.0
        marker.color.a = 1.0

        marker_points = []
        for point in self.points:
            p = Point()
            p.x, p.y, p.z = [point[0],point[1],0]
            marker_points.append(p)
        marker.points = marker_points

        line_points = []
        for i in range(len(self.points) - 1):
            line_points.append(marker_points[i])
            line_points.append(marker_points[i + 1])
        marker.points = line_points

        while not rospy.is_shutdown():
            marker.header.stamp = rospy.Time.now()
            pub.publish(marker)
            self.rate.sleep()
            
        
    def velocity_command_base(self):
        p = 0.4
        waypoint_tolerance = 0.1
        waypoint_count = 1
        for point in self.points:
            if point==[0,0]:
                print(f"Rejecting {point}")
                continue
            error = np.array(point)-np.array(self.current_pose)
            waypoint_count+=1
            while np.linalg.norm(error)>waypoint_tolerance:
                control = error*p
                control_msg = Twist()
                control_msg.linear.x = control[0]
                control_msg.linear.y = control[1]
                self.vel_pub.publish(control_msg)
                self.rate.sleep()
                error = np.array(point)-np.array(self.current_pose)
            print(f"Waypoint {point} reached, Waypoints left {len(self.points)-waypoint_count}")
        control_msg = Twist()
        control_msg.linear.x = 0
        control_msg.linear.y = 0
        self.vel_pub.publish(control_msg)
    def odom_callback(self,msg):
        self.current_pose= [msg.pose.pose.position.x,msg.pose.pose.position.y]
    

