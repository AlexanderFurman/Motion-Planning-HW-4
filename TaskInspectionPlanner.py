import numpy as np
import time
from RRTTree import RRTTree
import json
from copy import deepcopy



class TaskInspectionPlanner(object):

    def __init__(self, planning_env, coverage):

        #load hyperparameters
        with open('params.json') as f:
            params = json.load(f)

        # set environment
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env, task="ip")

        # set search params
        self.ext_mode = params['ext_mode']
        self.goal_prob = params['goal_prob']
        self.coverage = coverage
        self.step_size = 0.5

        #other robot's configs
        self.gripper_configs = self.planning_env.gripper_plan
        self.timestamps = np.array([i for i in range(len(self.gripper_configs))])

        self.current_coverage = 0

        self.saved_configs = []

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan_configs, plan_timestamps = [], []


        #add start node
        self.tree.add_vertex(self.planning_env.inspector_start, timestamp=0, inspected_timestamps=[])

        #iteratively add vertices to the rrt
        while self.tree.max_coverage < self.coverage:

            new_config = np.array([np.random.uniform(-np.pi, np.pi) for _ in range(self.planning_env.insp_robot.dim)])
            new_timestamp = np.random.choice(self.timestamps[1:])

            # print(f'new_config = {new_config}, new_timestamp = {new_timestamp}')

            # goal biased sample:
            if (np.random.uniform(0, 1) < self.goal_prob):
                pass

            near_config_idx, near_config, near_timestamp = self.tree.get_nearest_config(new_config, new_timestamp)

            extended_config, extended_timestamp = self.extend(near_config, new_config, near_timestamp, new_timestamp)
            print(f'extended config = {extended_config}, extended timestamp = {extended_timestamp}')
            if extended_config is None:
                # print('fucked config =', extended_config)
                continue
            # extended_config = np.array(extended_config)

            no_collision = (self.planning_env.config_validity_checker(extended_config, 'inspector') and self.planning_env.edge_validity_checker(near_config, extended_config, 'inspector'))
            if no_collision:
                print('in here!')

                old_inspected_timestamps = self.tree.vertices[near_config_idx].inspected_timestamps
                current_inspected_timestamps = deepcopy(old_inspected_timestamps)
                if self.planning_env.is_gripper_inspected_from_vertex(extended_config, extended_timestamp):
                    current_inspected_timestamps.append(extended_timestamp)

                #add to collection of configs which managed to see a new POI
                if len(old_inspected_timestamps) < len(current_inspected_timestamps):
                    self.save_config(extended_config)

                extended_config_idx = self.tree.add_vertex(extended_config, extended_timestamp, inspected_timestamps=current_inspected_timestamps)
                edge_cost = np.linalg.norm(near_config - extended_config) #POSSIBLY CHANGE THIS
                self.tree.add_edge(near_config_idx, extended_config_idx, edge_cost)

                if self.tree.max_coverage > self.current_coverage:
                    print(f"current max coverage is {self.tree.max_coverage}")
                    self.current_coverage = self.tree.max_coverage

            # print(f'current size of tree = {len(self.tree.vertices)}')


        plan = self.find_plan()
        # self.stats[self.current_stats_mode].append([self.compute_cost(plan), time.time()-start_time])


        # store total path cost and time
        path_cost = self.compute_cost(plan_configs)
        computation_time = time.time()-start_time

        # print total path cost and time
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

        coverage = self.current_coverage

        return np.array(plan_configs), np.array(plan_timestamps), coverage, path_cost, computation_time

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps in the configuration space.
        @param plan A given plan for the robot.
        '''
        # compute cost of a given path
        plan_cost = 0.0
        for i in range(len(plan)-1):
            plan_cost += self.planning_env.insp_robot.compute_distance(plan[i], plan[i+1])
        return plan_cost

    def compute_edge_cost(self, config1, timestamp1, config2, timestamp2):
        #create this class, if you are interested in adding the differences in timestamps to the to the cost
        pass

    def save_config(self, config):
        self.saved_configs.append(config)
        



    def extend(self, near_config, rand_config, near_timestamp, rand_timestamp):
        '''
        Compute and return a new configuration for the sampled one.
        @param near_config The nearest configuration to the sampled configuration.
        @param rand_config The sampled configuration.
        '''
        # TODO: Task 2.4
        if self.ext_mode == 'E1':
            return rand_config, rand_timestamp

        step_dir = np.array(rand_config) - np.array(near_config)
        length = np.linalg.norm(step_dir)
        if length < 0.001:
            return None, None
        step = (step_dir / length) * min(self.step_size, length)
        extended_config = near_config + step
        extended_config = np.array(extended_config)

        total_length = np.linalg.norm(step_dir)
        ext_length = np.linalg.norm(extended_config - near_config)
        extended_timestamp = near_timestamp + int((ext_length/total_length)*rand_timestamp)

        return extended_config, extended_timestamp

    def find_plan(self):
        start_idx = self.tree.get_root_id()
        current_idx = self.tree.max_coverage_id

        plan = [self.tree.vertices[current_idx].config]

        while current_idx != start_idx:
            current_idx = self.tree.edges[current_idx]
            plan.append(self.tree.vertices[current_idx].config)
        print("plan =\n", plan)
        plan.reverse()
        return plan

    def get_configs(self, filename):
        configs = []
        with open(filename) as f:
            strings = f.readline().split(', ')
            config = [float(string) for string in strings]
            configs.append(config)
        return np.array(configs)
