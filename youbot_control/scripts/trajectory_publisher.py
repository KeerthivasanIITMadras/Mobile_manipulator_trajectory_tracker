#!/usr/bin/env python

import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def publish_joint_trajectory():
    rospy.init_node('joint_trajectory_publisher', anonymous=True)
    pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        goal_msg = FollowJointTrajectoryActionGoal()
        goal_msg.goal.trajectory = JointTrajectory()

        goal_msg.goal.trajectory.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0]  
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]  
        point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0] 
        point.time_from_start = rospy.Duration(1)

        goal_msg.goal.trajectory.points.append(point)

        pub.publish(goal_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_trajectory()
    except rospy.ROSInterruptException:
        pass
