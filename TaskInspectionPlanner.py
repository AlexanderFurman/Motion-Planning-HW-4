import numpy as np
import time
from RRTTree import RRTTree
import json
from copy import deepcopy
from scipy.optimize import minimize
from matplotlib import pyplot as plt
import matplotlib.image as mpimg

np.random.seed(2)

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
        print('last timestamp = ', self.timestamps[-1])

        self.current_coverage = 0

        self.saved_configs = []

        #Loss function hyper parameters
        self.epsilon_pos = 5
        self.epsilon_angle = np.deg2rad(3)

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
            flag = False
            # goal biased sample:
            if (np.random.uniform(0, 1) < self.goal_prob):
                new_config, new_timestamp = self.goal_sampling()
                flag = True

            near_config_idx, near_config, near_timestamp = self.tree.get_nearest_config(new_config, new_timestamp)

            extended_config, extended_timestamp = self.extend(near_config, new_config, near_timestamp, new_timestamp)
            # print(f'extended config = {extended_config}, extended timestamp = {extended_timestamp}')
            if extended_config is None:
                # print('fucked config =', extended_config)
                continue
            # extended_config = np.array(extended_config)

            no_collision = (self.planning_env.config_validity_checker(extended_config, 'inspector') and self.planning_env.edge_validity_checker(near_config, extended_config, 'inspector'))
            if no_collision:
                # if flag == True:
                    # print('adding goal to tree!')

                # print('in here!')

                old_inspected_timestamps = self.tree.vertices[near_config_idx].inspected_timestamps
                if self.planning_env.is_gripper_inspected_from_vertex(extended_config, extended_timestamp):
                    current_inspected_timestamps = self.tree.compute_union(old_inspected_timestamps, [extended_timestamp])
                else:
                    current_inspected_timestamps = deepcopy(old_inspected_timestamps)

                #add to collection of configs which managed to see a new POI
                # if len(old_inspected_timestamps) < len(current_inspected_timestamps):
                #     self.save_config(extended_config)
                    # print('improved')
                    # print(f'vertex parent info: \n timestamp: {self.tree.vertices[near_config_idx].timestamp} \n inspected timestamps: {old_inspected_timestamps}\n')
                    # print(f'new vertex info: \n timestamp: {extended_timestamp} \n inspected timestamps: {current_inspected_timestamps}\n')

                extended_config_idx = self.tree.add_vertex(extended_config, extended_timestamp, inspected_timestamps=current_inspected_timestamps)
                edge_cost = np.linalg.norm(near_config - extended_config) #POSSIBLY CHANGE THIS
                self.tree.add_edge(near_config_idx, extended_config_idx, edge_cost)

                #print(f'parent timestamp = {near_timestamp}, extended_timestamp = {extended_timestamp}')

                self.find_inspected_from_edge(self.tree.vertices[near_config_idx], self.tree.vertices[extended_config_idx])

                if self.tree.max_coverage > self.current_coverage:
                    print(f"current max coverage is {self.tree.max_coverage}")
                    self.current_coverage = self.tree.max_coverage

            # print(f'current size of tree = {len(self.tree.vertices)}')

        plan_configs, plan_timestamps = self.find_plan()
        # self.stats[self.current_stats_mode].append([self.compute_cost(plan), time.time()-start_time])


        # store total path cost and time
        path_cost = self.compute_cost(plan_configs)
        computation_time = time.time()-start_time

        # print total path cost and time
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan_configs)))
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
        extended_timestamp = near_timestamp + int((ext_length/total_length)*(rand_timestamp-near_timestamp))
        # print(f'near timestamp = {near_timestamp}, rand timestamp = {rand_timestamp}, extended timestamp = {extended_timestamp}')
        return extended_config, extended_timestamp

    def find_plan(self):
        start_idx = self.tree.get_root_id()
        current_idx = self.tree.max_coverage_id

        plan = [self.tree.vertices[current_idx].config]
        timestamps = [self.tree.vertices[current_idx].timestamp]

        while current_idx != start_idx:
            current_idx = self.tree.edges[current_idx]
            plan.append(self.tree.vertices[current_idx].config)
            timestamps.append(self.tree.vertices[current_idx].timestamp)
        print("plan =\n", plan)
        plan.reverse()
        timestamps.reverse()
        return plan, timestamps

    def get_configs(self, filename):
        configs = []
        with open(filename) as f:
            strings = f.readline().split(', ')
            config = [float(string) for string in strings]
            configs.append(config)
        return np.array(configs)

    def find_inspected_from_edge(self, vertex_near, vertex_extended):
        inspector_config1, inspector_config2, timestamp1, timestamp2 = vertex_near.config, vertex_extended.config, vertex_near.timestamp, vertex_extended.timestamp
        inspected_timestamps = []
        # iterate over timestamps and compute for each if gripper was inspected
        for timestamp in range(int(np.ceil(timestamp1)),int(np.floor(timestamp2))+1):
            if timestamp2 == timestamp1:
                continue
            # compute interpolated config for the requested time
            #print("timestamp1",timestamp1)
            #print("timestamp2", timestamp2)
            delta_config = ((timestamp - timestamp1) / (timestamp2 - timestamp1)) * (inspector_config2 - inspector_config1)
            current_config = inspector_config1 + delta_config

            # check if gripper is inspected and update accordingly
            is_inspected = self.planning_env.is_gripper_inspected_from_vertex(inspector_config=current_config, timestamp=timestamp)
            if is_inspected:
                inspected_timestamps.append(timestamp)

        total_inspected = self.tree.compute_union(vertex_extended.inspected_timestamps, inspected_timestamps)
        vertex_extended.inspected_timestamps = total_inspected
            

    def loss_function(self, config, sample_config):
        # print('type of config = ', type(config))
        gripper_pos = self.planning_env.grip_robot.compute_forward_kinematics(sample_config)[-1]
        insp_pos = self.planning_env.insp_robot.compute_forward_kinematics(config)

        camera_pos = insp_pos[-1]
        final_link_pos = insp_pos[-2]

        r_max = self.planning_env.insp_robot.vis_dist

        vec_grip_to_link = final_link_pos - gripper_pos
        vec_grip_to_camera = camera_pos - gripper_pos

        # L_pos = max(np.linalg.norm(vec_grip_to_camera) - (r_max + self.epsilon_pos), 0)
        L_pos = np.linalg.norm(vec_grip_to_camera)
        # print('L_POS = ', L_pos)
        L_angle = np.abs(np.arcsin((np.cross(vec_grip_to_camera, vec_grip_to_link))/(np.linalg.norm(vec_grip_to_link)*np.linalg.norm(vec_grip_to_camera)))) * self.epsilon_angle

        return L_pos + L_angle

    def goal_sampling(self):
        sample_timestamp = np.random.randint(1, self.timestamps[-1])
        sample_config = self.gripper_configs[sample_timestamp]
        theta0 = np.array([-np.deg2rad(150), 0, 0, 0])
        bounds = [(-np.pi, np.pi) for _ in range(self.planning_env.insp_robot.dim)]
        result = minimize(fun=self.loss_function, x0=theta0, args=(sample_config, ), bounds=bounds)
        return result.x, sample_timestamp

    def goal_sampling_example(self):
        sample_timestamp = np.random.randint(1, self.timestamps[-1])
        sample_config = self.gripper_configs[sample_timestamp]
        theta0 = np.array([-np.deg2rad(150), 0, 0, 0])
        bounds = [(-np.pi, np.pi) for _ in range(self.planning_env.insp_robot.dim)]
        result = minimize(fun=self.loss_function, x0=theta0, args=(sample_config, ), bounds=bounds)
        
        self.planning_env.visualize_map(gripper_config=sample_config, inspector_config=result.x)
        
        grip_pos = self.planning_env.grip_robot.compute_forward_kinematics(sample_config)[-1]
        insp_pos = self.planning_env.insp_robot.compute_forward_kinematics(result.x)[-1]
        distance = np.linalg.norm(grip_pos-insp_pos)
        print('DISTANCE = ', distance)
        print('CAN HE SEE GRIPGRIP??? ', (distance < self.planning_env.insp_robot.vis_dist))
        print('YE BUT LIKE< REALLY??< ', self.planning_env.config_validity_checker(result.x, 'inspector'))
        print(result)
        return self.planning_env.is_gripper_inspected_from_vertex(result.x, sample_timestamp)

