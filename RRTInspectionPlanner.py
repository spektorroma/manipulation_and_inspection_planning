import numpy as np
from RRTTree import RRTTree
import time

class RRTInspectionPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob, coverage):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env, task="ip")

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.coverage = coverage

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''

        # initialize an empty plan.
        plan = []

        # TODO: Task 2.4

        # your stopping condition should look like this: 
        # while self.tree.max_coverage < self.coverage:
        
        pass

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps in the configuration space.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 2.3

        total_cost = 0.0
        for i in range(len(plan) - 1):
            total_cost += self.planning_env.robot.compute_distance(plan[i], plan[i+1])
        return total_cost

    def extend(self, near_config, rand_config):
        '''
        Compute and return a new configuration for the sampled one.
        @param near_config The nearest configuration to the sampled configuration.
        @param rand_config The sampled configuration.
        '''
        step = 50
        new_config = rand_config
        if self.planning_env.robot.compute_distance(near_config, rand_config) > step:
            step_size = 0.1  # Adjust as needed

            # Compute the direction vector from near_config to rand_config
            direction = rand_config - near_config

            # Normalize the direction vector
            direction /= np.linalg.norm(direction)

            # Extend from near_config towards rand_config by step_size
            new_config = near_config + step_size * direction
            new_config = np.round(new_config, 2)
        return new_config

    