#!/usr/bin/env python
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from pydrake.solvers import MathematicalProgram, Solve
import time
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

robot = DHRobot([RevoluteDH(d =0.147 ,offset=0,alpha=np.pi/2,a=0.033),
                 RevoluteDH(d =0 ,offset=0,alpha=0,a=0.155),
                 RevoluteDH(d = 0,offset=0,alpha=0,a=0.135),
                 RevoluteDH(d = 0,offset=0,alpha=np.pi/2,a=0),
                 RevoluteDH(d = 0.2175,offset=0,alpha=0,a=0)])

class IK:
    def __init__(self,robot) -> None:
        self.robot = robot
        self.dt = 0.1
    
    def jacobain_inverse_kinematics(self,q,control):
        J = self.robot.jacob0(q)
        v = control
        dq = np.linalg.pinv(J)@v
        nq = q+dq*self.dt
        return nq

    def jacobian_with_optimization(self,q,control):
        J = robot.jacob0(q)
        prog = MathematicalProgram()
        v = prog.NewContinuousVariables(5,"v")
        vmax = 3.0
        v_des = control
        error = J@v-v_des
        prog.AddCost(error.dot(error))
        prog.AddBoundingBoxConstraint(-vmax,vmax,v)
        result = Solve(prog)
        return q+result.GetSolution(v)*self.dt

class Gazebo_interface:
    def __init__(self,robot) -> None:
        self.f = 0.1
        self.robot = robot
        self.tolerance = 0.15
        self.ik = IK(self.robot)
        rospy.init_node('gazebo_interface')
        rospy.Subscriber('joint_states',JointState,self.joint_state_callback)
        self.pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
        self.positions = None

    def go_to_goal(self,goal,current_state,joint_state):
        self.f = 0.5
        delta_x = goal-current_state
        control_input = self.f*delta_x
        final_state = None
        goal_msg = FollowJointTrajectoryActionGoal()
        goal_msg.goal.trajectory = JointTrajectory()
        goal_msg.goal.trajectory.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        point = JointTrajectoryPoint()
        while self.positions is None:
            pass
        current_state = self.positions[:5]
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]  
        point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0]
        time_increment = rospy.Duration(1)
        while np.linalg.norm(delta_x)>self.tolerance:
            joint_state = self.ik.jacobian_with_optimization(joint_state,control_input)
            point.positions = joint_state  
            point.time_from_start = time_increment
            goal_msg.goal.trajectory.points.append(point)
            self.pub.publish(goal_msg)
            current_state = self.robot.fkine(self.positions[:5])
            current_orientation = R.from_matrix(current_state.R).as_euler('xyz')
            current_state_new= np.zeros(6)
            current_state_new[:3]=current_state.t
            current_state_new[3:] = current_orientation
            delta_x = goal-current_state_new
            control_input = self.f*delta_x
            final_state = joint_state
            time_increment += rospy.Duration(1)
            print(np.linalg.norm(delta_x))
            goal_msg.goal.trajectory.points = []
            time.sleep(0.25)

        print("converged")
        return final_state
    
    def joint_state_callback(self,msg):
        self.positions = msg.position

if __name__=="__main__":
    robot = DHRobot([RevoluteDH(d =0.147 ,offset=0,alpha=np.pi/2,a=0.033),
                 RevoluteDH(d =0 ,offset=0,alpha=0,a=0.155),
                 RevoluteDH(d = 0,offset=0,alpha=0,a=0.135),
                 RevoluteDH(d = 0,offset=0,alpha=np.pi/2,a=0),
                 RevoluteDH(d = 0.2175,offset=0,alpha=0,a=0)])
    
    joint_state = np.zeros(5) # this is the joint_state
    goal_state = [0.3,0,0.1,0,0,0] # this is in the cartesian state
    current_state = robot.fkine(joint_state)
    current_orientation = R.from_matrix(current_state.R).as_euler('xyz')
    cartesian_state= np.zeros(6)
    cartesian_state[:3]=current_state.t
    cartesian_state[3:] = current_orientation
    gi = Gazebo_interface(robot)
    final_joint_state = gi.go_to_goal(goal_state,cartesian_state,joint_state)
    print(final_joint_state)
