#!/usr/bin/env python3

import matplotlib.pyplot as plt
import math
import rospy
import random
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from scipy.interpolate import CubicSpline
from navigation_control import Navigation
from rrtsimple import publishAngles

initial_config = [0]*7


def extract_points(tree):
    points = []
    for node in tree:
        points.append(node.config)
        for child in node.children:
            points.extend(extract_points([child]))
    return points


class Node:
    def __init__(self, config, parent=None) -> None:
        self.config = config  # this takes in a array value
        self.parent = parent  # this takes in a node
        self.children = []  # node gets appended to this


class BRRT:
    def __init__(self, initial_config, goal_config, bounds) -> None:
        self.Ta = [Node(initial_config, None)]
        self.Tb = [Node(goal_config, None)]
        self.bounds = bounds
        self.goal_config = goal_config
        self.numWaypoints = 0
        self.waypoints = []
        self.max_step_size = 0.1

    def expand(self):
        return [random.uniform(min_val, max_val)
                for min_val, max_val in self.bounds]

    def metric(self, q, q_node):
        return np.linalg.norm(np.array(q)-np.array(q_node)
                              )

    def nearest_neighbour(self, tree, q_rand) -> Node:
        nearest = None
        min_dist = float('inf')
        for node in tree:
            dist = self.metric(node.config, q_rand)
            if dist < min_dist:
                nearest = node
                min_dist = dist
        return nearest

    def stopping_configuration(self, qn, q_rand) -> np.ndarray:
        vector = (np.array(q_rand) - np.array(qn.config)) / \
            np.linalg.norm(np.array(q_rand) - np.array(qn.config))
        qs = np.array(qn.config)+self.max_step_size*vector
        return qs

    def swap_trees(self):
        if len(self.Tb) > len(self.Ta):
            self.Tb, self.Ta = self.Ta, self.Tb
        else:
            self.Tb, self.Ta = self.Ta, self.Tb

    def retrace_path(self, goal: Node):
        if goal.parent is None:
            self.numWaypoints += 1
            current_point = goal.config
            self.waypoints.append(current_point)
            return
        self.numWaypoints += 1
        current_point = goal.config
        self.waypoints.append(current_point)
        self.retrace_path(goal.parent)

    def path_to_goal(self):
        self.retrace_path(self.Tb[-1])
        self.waypoints.reverse()
        self.retrace_path(self.Ta[-1])
        return self.waypoints

    def run(self, K):
        for i in range(K):
            q_rand = self.expand()
            qn = self.nearest_neighbour(self.Ta, q_rand)
            qs = self.stopping_configuration(qn, q_rand)

            if self.metric(qs, qn.config) > 1e-5:
                self.Ta.append(Node(qs, qn))
                qn.children.append(Node(qs, qn))

                qn_prime = self.nearest_neighbour(self.Tb, qs)
                qs_prime = self.stopping_configuration(qn_prime, qs)

                if self.metric(qs_prime, qn_prime.config):
                    self.Tb.append(Node(qs_prime, qn_prime))
                    qn_prime.children.append(Node(qs_prime, qn_prime))

                if self.metric(qs, qs_prime) < 1:
                    return True, self.Ta, self.Tb

            self.swap_trees()
        return False, self.Ta, self.Tb


# if __name__ == "__main__":
#     qi = np.array([0, 0])
#     qg = np.array([0, 100])
#     bounds = [(0, 102), (0, 102)]
#     K = 1000
#     brrt = BRRT(qi, qg, bounds)
#     solution_found, Ta, Tb = brrt.run(K)
#     if solution_found:
#         print("Solution found!")
#         path_to_goal = brrt.path_to_goal()
#         print(f"Number of Waypoints = {brrt.numWaypoints}")

#         # points_ta = extract_points(Ta)
#         # points_tb = extract_points(Tb)

#         # plt.plot(*zip(*points_ta), 'bo-', label='Tree A')
#         # plt.plot(*zip(*points_tb), 'ro-', label='Tree B')
#         # plt.plot(*zip(*path_to_goal), 'g-', label='Shortest Path')
#         # plt.plot(qi[0], qi[1], 'go', label='Start')
#         # plt.plot(qg[0], qg[1], 'yo', label='Goal')
#         # plt.xlabel('X')
#         # plt.ylabel('Y')
#         # plt.title('RRT-Bidirectional Solution Path')
#         # plt.legend()
#         # plt.grid(True)
#         # plt.show()
#     else:
#         print("No solution found.")

def joint_callback(msg):

    global initial_config
    initial_config = msg.position[0:5]
    initial_config = list(initial_config)+[0, 0]


if __name__ == "__main__":

    rospy.init_node("BRRT", anonymous=True)

    joint_state = rospy.Subscriber(
        "/youbot/joint_states", JointState, joint_callback)

    goal_config = [[1.6, -0.75, -1.5, 1, 1.8], [1.4, 2.13, -1.5, 1.4, 2]]
    goal_config = [[1.4, 2.13, -1.5, 1.4, 2, 3, 3]]

    bounds = [(-1, 1) for _ in range(5)]
    for i in range(2):
        bounds.append((0, 7))
    K = 1000

    while initial_config == [0, 0, 0, 0, 0, 0, 0]:
        rospy.sleep(0.1)
    print(goal_config[0][:5])
    for i in range(len(goal_config)):
        print(f"Initial Config: {initial_config}")
        print(" ")
        brrt = BRRT(initial_config, goal_config[i], bounds)
        solution_found, Ta, Tb = brrt.run(K)
        if solution_found:
            print("Solution found!")
            waypoints = path_to_goal = brrt.path_to_goal()
            print(f"Number of Waypoints = {brrt.numWaypoints}")
            waypoints.reverse()
            joint_angles_trajectory = [row[:5] for row in waypoints]
            publishAngles(joint_angles_trajectory)
            cartesian_trajectory = [row[5:7] for row in brrt.waypoints]
            nav = Navigation(cartesian_trajectory)
            nav.velocity_command_base()
