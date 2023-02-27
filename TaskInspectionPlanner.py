import numpy as np
import time

class TaskInspectionPlanner(object):

    def __init__(self, planning_env, coverage):

        # set environment
        self.planning_env = planning_env

        # set search params
        self.coverage = coverage

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan_configs, plan_timestamps = [], []

        # TODO: implement you planner here
        # Your stopping condition should look like this:
        # while coverage < self.coverage:

        # store total path cost and time
        path_cost = self.compute_cost(plan_configs)
        computation_time = time.time()-start_time

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