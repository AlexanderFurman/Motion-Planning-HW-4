import operator
import numpy as np

class RRTTree(object):
    
    def __init__(self, planning_env, task="mp"):
        
        self.planning_env = planning_env
        self.task = task
        self.vertices = {}
        self.edges = {}

        # inspecion planning properties
        if self.task == "ip":
            self.max_coverage = 0
            self.max_coverage_id = 0
            self.max_timestamp = 0

    def get_root_id(self):
        '''
        Returns the ID of the root in the tree.
        '''
        return 0

    def add_vertex(self, config, timestamp, inspected_timestamps=None):
        '''
        Add a state to the tree.
        @param config Configuration to add to the tree.
        '''
        vid = len(self.vertices)
        self.vertices[vid] = RRTVertex(config=config, timestamp = timestamp, inspected_timestamps=inspected_timestamps)

        # check if vertex has the highest coverage so far, and replace if so
        if self.task == "ip":
            v_coverage = self.compute_coverage(inspected_timestamps=inspected_timestamps)
            if v_coverage > self.max_coverage:
                self.max_coverage = v_coverage
                self.max_coverage_id = vid
            if timestamp > self.max_timestamp:
                self.max_timestamp = timestamp

        return vid

    def add_edge(self, sid, eid, edge_cost=0):
        '''
        Adds an edge in the tree.
        @param sid start state ID
        @param eid end state ID
        '''
        self.edges[eid] = sid
        self.vertices[eid].set_cost(cost=self.vertices[sid].cost + edge_cost)

    def is_goal_exists(self, config):
        '''
        Check if goal exists.
        @param config Configuration to check if exists.
        '''
        goal_idx = self.get_idx_for_config(config=config)
        if goal_idx is not None:
            return True
        return False

    def get_vertex_for_config(self, config):
        '''
        Search for the vertex with the given config and return it if exists
        @param config Configuration to check if exists.
        '''
        v_idx = self.get_idx_for_config(config=config)
        if v_idx is not None:
            return self.vertices[v_idx]
        return None

    def get_idx_for_config(self, config):
        '''
        Search for the vertex with the given config and return the index if exists
        @param config Configuration to check if exists.
        '''
        valid_idxs = [v_idx for v_idx, v in self.vertices.items() if (v.config == config).all()]
        if len(valid_idxs) > 0:
            return valid_idxs[0]
        return None

    def get_nearest_config(self, config, timestamp):
        '''
        Find the nearest vertex for the given config and returns its state index and configuration
        @param config Sampled configuration.
        '''
        # compute distances from all vertices
        dists = []
        for _, vertex in self.vertices.items():
            if vertex.timestamp < timestamp:
                # dists.append(self.planning_env.insp_robot.compute_distance(config, vertex.config))
                dists.append(self.compute_distance_with_time(vertex.config, vertex.timestamp, config, timestamp))
            else:
                dists.append(1000000000)
        # retrieve the id of the nearest vertex
        vid, _ = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid].config, self.vertices[vid].timestamp

    def compute_coverage(self, inspected_timestamps):
        total = len(self.planning_env.gripper_plan)
        return len(inspected_timestamps)/total

    def compute_union(self, inspected_points_1, inspected_points_2):
        return list(set(inspected_points_1 + inspected_points_2))

    def compute_distance_with_time(self, config1, timestamp1, config2, timestamp2):
        total_timestamps = 370
        vec1 = np.append(config1, 2*np.pi*timestamp1/total_timestamps)
        vec2 = np.append(config2, 2*np.pi*timestamp2/total_timestamps)
        distance = np.linalg.norm(vec2 - vec1)
        return distance



    # def get_max_timestamp(self):
    #     max = 0
    #     for vid in self.vertices:
    #         if self.vertices[vid].timestamp > max:
    #             max = self.vertices[vid].timestamp

    #     return max



class RRTVertex(object):

    def __init__(self, config, timestamp, cost=0, inspected_timestamps=None):

        self.config = config
        self.cost = cost
        self.inspected_timestamps = inspected_timestamps
        self.timestamp = timestamp

    def set_cost(self, cost):

        '''
        Set the cost of the vertex.
        '''
        self.cost = cost