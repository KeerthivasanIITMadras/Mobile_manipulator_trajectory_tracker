#!/usr/bin/env python3

import math
import rospy
import random
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from scipy.interpolate import CubicSpline

initial_config = [0]*5

class SimpleRRT:

    def __init__(self, initial_config, goal_config) -> None:
        self.q = [initial_config]
        self.parent = [0]
        self.goal_config = goal_config

    def metric(self, q1, q2):
        return math.sqrt(sum((x-y)**2 for x,y in zip(q1, q2)))
    
    def expand(self, bounds, goal_bias):
        if random.random() < goal_bias:
            q_rand = self.goal_config[:]
        else:
            q_rand = [random.uniform(min_val, max_val) for min_val, max_val in bounds]
        self.q.append(q_rand)
        self.parent.append(0)
    
    def nearest_neighbour(self, q):
        min_dist = float('inf')
        nearest_idx = 0
        for i, q_node in enumerate(self.q):
            dist = self.metric(q, q_node)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        return nearest_idx

    def extend(self, q_near_idx, q_rand, step_size):
        q_near = self.q[q_near_idx]
        dist = self.metric(q_near, q_rand)
        if dist <= step_size:
            return q_rand
        
        delta_q = [(q_r-q_n)/dist*step_size for q_r, q_n in zip(q_rand, q_near)]
        q_new = [q_n + dq for q_n, dq in zip(q_near, delta_q)]
        return q_new
    
    def connect(self, q_new_idx, q_near_idx, bounds):
        q_new = self.q[q_new_idx]
        q_near = self.q[q_near_idx]

        if self.is_path_free(q_near, q_new, bounds):
            self.parent[q_new_idx] = q_near_idx
            return True
        return False
    
    def is_path_free(self, q1, q2, bounds):
        ## Need to do Collision Checking
        return True
    
    def grow_tree(self, max_iter, step_size, bounds, goal_bias):
        for _ in range(max_iter):
            self.expand(bounds, goal_bias)
            q_rand_idx = len(self.q) - 1
            q_near_idx = self.nearest_neighbour(self.q[q_rand_idx])
            q_new = self.extend(q_near_idx, self.q[q_rand_idx], step_size)

            if self.connect(q_rand_idx, q_near_idx, bounds):
                if self.is_goal_reached(q_new):
                    return self.construct_path(len(self.q)-1)
        return None
    
    def is_goal_reached(self, q):
        return self.metric(q, self.goal_config) < 1 ## Threshold

    def construct_path(self, goal_idx):
        path=[]
        while goal_idx != 0:
            path.append(self.q[goal_idx])
            goal_idx = self.parent[goal_idx] - 1 
        path.append(self.q[0])
        return path[::-1]


def publishAngles(path):

    jointPubs = [
        rospy.Publisher("/youbot/arm_joint_1_controller/command", Float64, queue_size=10),
        rospy.Publisher("/youbot/arm_joint_2_controller/command", Float64, queue_size=10),
        rospy.Publisher("/youbot/arm_joint_3_controller/command", Float64, queue_size=10),
        rospy.Publisher("/youbot/arm_joint_4_controller/command", Float64, queue_size=10),
        rospy.Publisher("/youbot/arm_joint_5_controller/command", Float64, queue_size=10)
    ]

    rate = rospy.Rate(50)
    num_points = len(path)
    time_points = np.linspace(0, 1, num_points)
    duration = 5.0
    num_intervals = int(duration*50)
    spline_functions = [CubicSpline(time_points, [waypoint[j] for waypoint in path]) for j in range(5)]

    for i in range(num_intervals):
        t = i /num_intervals
        waypoint = [spline(t) for spline in spline_functions]

        for j in range(5):
            jointPubs[j].publish(waypoint[j])
        rate.sleep()

def continousTrajectory(initial_config, goal_config):

    bounds = [(-1, 1) for _ in range(5)]
    max_iter = 1000
    step_size = 0.1
    path = []

    rrt = SimpleRRT(initial_config, goal_config)
    goal_bias = 0.2
    path = rrt.grow_tree(max_iter, step_size, bounds, goal_bias)
    print(f"Path: {path}")
    print(" ")
    publishAngles(path)


def joint_callback(msg):

    global initial_config
    initial_config = msg.position[0:5]

if __name__ == "__main__":

    rospy.init_node("SimpleRRT", anonymous=True)
    
    joint_state = rospy.Subscriber("/youbot/joint_states", JointState, joint_callback)

    goal_config = [[1.6, -0.75, -1.5, 1, 1.8], [1.4, 2.13, -1.5, 1.4, 2]]

    while initial_config == [0, 0, 0, 0, 0]:
        rospy.sleep(0.1) 

    for i in range(len(goal_config)):
        print(f"Initial Config: {initial_config}")
        print(" ")
        continousTrajectory(initial_config, goal_config[i])