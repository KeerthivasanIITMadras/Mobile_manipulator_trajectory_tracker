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
from scipy.optimize import minimize

robot = DHRobot([RevoluteDH(d =0.147 ,offset=0,alpha=np.pi/2,a=0.033),
                 RevoluteDH(d =0 ,offset=0,alpha=0,a=0.155),
                 RevoluteDH(d = 0,offset=0,alpha=0,a=0.135),
                 RevoluteDH(d = 0,offset=0,alpha=np.pi/2,a=0),
                 RevoluteDH(d = 0.2175,offset=0,alpha=0,a=0)])

class IK:
    def __init__(self,robot) -> None:
        self.robot = robot
        self.dt = 0.1
        self.q =None
    
    def jacobain_inverse_kinematics(self,q,control):
        J = self.robot.jacob0(q)
        v = control
        dq = np.linalg.pinv(J)@v
        nq = q+dq*self.dt
        return nq
    
    def objective_function(self,v):
        J = robot.jacob0(self.q)
        error = J@v-self.v_des
        return error.dot(error)

    def constraint_1(self,v):
        return self.q[0]+v[0]*self.dt
    
    def constraint_2(self,v):
        return 5.899212871740834-(self.q[0]+v[0]*self.dt)
    
    def constraint_3(self,v):
        return self.q[1]+v[1]*self.dt
    
    def constraint_4(self,v):
        return 2.705260340591211-(self.q[1]+v[1]*self.dt)

    def constraint_5(self,v):
        return -self.q[2]+v[2]*self.dt
    
    def constraint_6(self,v):
        return 5.183627878423159+(self.q[2]+v[2]*self.dt)

    def constraint_7(self,v):
        return self.q[3]+v[3]*self.dt
    
    def constraint_8(self,v):
        return 3.5779249665883754-(self.q[3]+v[3]*self.dt)   

    def constraint_9(self,v):
        return self.q[4]+v[4]*self.dt
    
    def constraint_10(self,v):
        return 5.8468529941810035-(self.q[4]+v[4]*self.dt)   
    
    def jacobian_with_optimization(self,q,control):
        self.q = q
        # J = robot.jacob0(q)
        self.v_des = control

        # print(J.shape)
        # exit()
        # prog = MathematicalProgram()
        # v = prog.NewContinuousVariables(5,"v")
        # vmax =0.1
        # v_des = control
        # error = J@v-v_des
        # prog.AddCost(error.dot(error))
        # prog.AddBoundingBoxConstraint(-vmax,vmax,v)
        # result = Solve(prog)
        # return q+result.GetSolution(v)*self.dt

        con1 = {'type': 'ineq', 'fun': self.constraint_1}
        con2 = {'type': 'ineq', 'fun': self.constraint_2}
        con3 = {'type': 'ineq', 'fun': self.constraint_3}
        con4 = {'type': 'ineq', 'fun': self.constraint_4}
        con5 = {'type': 'ineq', 'fun': self.constraint_5}
        con6 = {'type': 'ineq', 'fun': self.constraint_6}
        con7 = {'type': 'ineq', 'fun': self.constraint_7}
        con8 = {'type': 'ineq', 'fun': self.constraint_8}
        con9 = {'type': 'ineq', 'fun': self.constraint_9}
        con10 = {'type': 'ineq', 'fun': self.constraint_10}

        # b = (-1, 1)
        # bnds = [b, b, b, b, b]

        cons = [con1, con2, con3,con4,con5,con6,con7,con8,con9,con10]
        v = np.ones(5)
        sol = minimize(self.objective_function, v,
                       method='COBYLA', constraints=cons)
    
        # print(self.objective_function(sol.x))
        return self.q+sol.x*self.dt

class Gazebo_interface:
    def __init__(self,robot) -> None:
        self.f = 1
        self.robot = robot
        self.tolerance = 0.14
        self.ik = IK(self.robot)
        rospy.init_node('gazebo_interface')
        rospy.Subscriber('joint_states',JointState,self.joint_state_callback)
        self.pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)
        self.positions = None

    def go_to_goal(self,goal):

        final_state = None
        self.point = JointTrajectoryPoint()
        while self.positions is None:
            pass
        current_state = self.robot.fkine(self.positions[:5]) # current state of end effector is in cartesian
        joint_state = self.positions[:5]
        current_orientation = R.from_matrix(current_state.R).as_euler('xyz')
        current_state_new= np.zeros(6)
        current_state_new[:3]=current_state.t
        current_state_new[3:] = current_orientation
        delta_x = goal-current_state_new
        control_input = self.f*delta_x
        # print(np.linalg.norm(control_input))
        self.point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]  
        self.point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        while np.linalg.norm(delta_x[:3])>self.tolerance:
            joint_state = self.ik.jacobian_with_optimization(joint_state,control_input)
            current_state = self.robot.fkine(joint_state)
            current_orientation = R.from_matrix(current_state.R).as_euler('xyz')
            current_state_new= np.zeros(6)
            current_state_new[:3]=current_state.t
            current_state_new[3:] = current_orientation
            delta_x = goal-current_state_new
            control_input = self.f*delta_x
            final_state = joint_state
            print(np.linalg.norm(delta_x[:3]))

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
    
    goal_state = [0,0.0,0.0,0,0,0] # this is in the cartesian state

    gi = Gazebo_interface(robot)

    final_joint_state = gi.go_to_goal(goal_state)

    '''from this on , i am publishing the joint states we got through ik calculation'''
    rate = rospy.Rate(20)
    print(final_joint_state)
    time_increment = rospy.Duration(1)
    while not rospy.is_shutdown():
        goal_msg = FollowJointTrajectoryActionGoal()
        goal_msg.goal.trajectory = JointTrajectory()
        goal_msg.goal.trajectory.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        gi.point.positions = final_joint_state
        gi.point.time_from_start = time_increment
        goal_msg.goal.trajectory.points.append(gi.point)
        gi.pub.publish(goal_msg)
        rate.sleep()

