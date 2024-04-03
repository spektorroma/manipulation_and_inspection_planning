import numpy as np
from RRTTree import RRTTree
import time

class RRTMotionPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        # TODO: Task 2.3

        self.tree.add_vertex(self.planning_env.start)

        range_min, range_max = -314, 314
        step = 1
        num_steps = int((range_max - range_min) / step) + 1

        index_array = np.array([0]*num_steps)
        element_probabilities_array = np.array([0]*num_steps)

        for i in range(len(self.planning_env.goal)):
            coords = np.arange(range_min, range_max + step, step)/100

            element_probabilities = np.ones_like(coords) * ((1 - self.goal_prob ** (1/4)) / (len(coords) - 1))  # Default probability
            goal_index = np.where(coords == self.planning_env.goal[i])[0]
            
            element_probabilities[goal_index] = self.goal_prob ** (1/4)

            index_array = np.vstack((index_array, coords))
            element_probabilities_array = np.vstack((element_probabilities_array, element_probabilities))

        index_array = index_array[1:]
        element_probabilities_array = element_probabilities_array[1:]
        
        while True:
            randomly_picked_first_angle = np.random.choice(index_array[0], p=element_probabilities_array[0])
            randomly_picked_second_angle = np.random.choice(index_array[1], p=element_probabilities_array[1])
            randomly_picked_third_angle = np.random.choice(index_array[2], p=element_probabilities_array[2])
            randomly_picked_fourth_angle = np.random.choice(index_array[3], p=element_probabilities_array[3])

            random_config = np.array([randomly_picked_first_angle, randomly_picked_second_angle, randomly_picked_third_angle, randomly_picked_fourth_angle])
            nearest_neighbor = self.tree.get_nearest_config(random_config)
            
            if self.ext_mode == 'E2':
                random_config = self.extend(nearest_neighbor[1], random_config)
            
            if self.planning_env.config_validity_checker(random_config) and self.planning_env.edge_validity_checker(nearest_neighbor[1], random_config) and self.tree.get_idx_for_config(nearest_neighbor[1]) != self.tree.get_idx_for_config(random_config):
                self.tree.add_vertex(random_config)
                self.tree.add_edge(self.tree.get_idx_for_config(nearest_neighbor[1]), self.tree.get_idx_for_config(random_config), self.planning_env.robot.compute_distance(nearest_neighbor[1], random_config))

            if self.tree.is_goal_exists(self.planning_env.goal):
                break

        
        current_idx = self.tree.get_idx_for_config(self.planning_env.goal)
        while current_idx != self.tree.get_root_id():
            plan.append(self.tree.vertices[current_idx].config)
            current_idx = self.tree.edges[current_idx]

        plan.append(self.planning_env.start)  # Add the start state to the plan
        plan.reverse()

        
        # print total path cost and time
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

        return np.array(plan)

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
        step = 100
        new_config = rand_config
        if self.planning_env.robot.compute_distance(near_config, rand_config) > 100:
            step_size = 0.1  # Adjust as needed

            # Compute the direction vector from near_config to rand_config
            direction = rand_config - near_config

            # Normalize the direction vector
            direction /= np.linalg.norm(direction)

            # Extend from near_config towards rand_config by step_size
            new_config = near_config + step_size * direction
            new_config = np.round(new_config, 2)
            print(new_config)

        return new_config