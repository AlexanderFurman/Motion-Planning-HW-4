import os
import time
from datetime import datetime
import json
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from Robot import GrippingRobot, InspectingRobot
from shapely.geometry import Point, LineString, Polygon
import imageio
from utils import compute_inner_interpolated_configurations

class MapEnvironment(object):
    
    def __init__(self, json_file):

        # check if json file exists and load
        json_path = os.path.join(os.getcwd(), json_file)
        if not os.path.isfile(json_path):
            raise ValueError('Json file does not exist!');
        with open(json_path) as f:
            json_dict = json.load(f)

        # obtain boundary limits, start and inspection points
        self.xlimit = [0, json_dict['WIDTH']-1]
        self.ylimit = [0, json_dict['HEIGHT']-1]
        self.inspector_start = np.array(json_dict['INSPECTOR_START'])
        self.load_obstacles(obstacles=json_dict['OBSTACLES'])

        # load gripper robot path
        with open(json_dict['GRIPPER_PLAN_FILENAME'] + ".txt", 'r') as gripper_plan_file:
            self.gripper_plan = gripper_plan_file.read().splitlines()

        # data storage for gripper motion
        self.gripper_plan = np.array([[float(y) for y in x.split(', ')] for x in self.gripper_plan])
        self.gripper_start = self.gripper_plan[0]
        self.gripper_inspected =  np.zeros(len(self.gripper_plan))

        # create gripping and inspecting robots
        self.grip_robot = GrippingRobot()
        self.insp_robot = InspectingRobot(origin=[self.xlimit[1], self.ylimit[1]])

        # check that the start location is within limits and collision free
        if not self.config_validity_checker(config=self.inspector_start, robot_type='inspector'):
            raise ValueError('Start config must be within the map limits');

        # if you want to - you can display starting map here
        self.visualize_map(gripper_config=self.gripper_start, inspector_config=self.inspector_start)

    def load_obstacles(self, obstacles):
        '''
        A function to load and verify scene obstacles.
        @param obstacles A list of lists of obstacles points.
        '''
        # iterate over all obstacles
        self.obstacles, self.obstacles_edges = [], []
        for obstacle in obstacles:
            non_applicable_vertices = [x[0] < self.xlimit[0] or x[0] > self.xlimit[1] or x[1] < self.ylimit[0] or x[1] > self.ylimit[1] for x in obstacle]
            if any(non_applicable_vertices):
                raise ValueError('An obstacle coincides with the maps boundaries!');
            
            # make sure that the obstacle is a closed form
            if obstacle[0] != obstacle[-1]:
                obstacle.append(obstacle[0])
                self.obstacles_edges.append([LineString([Point(x[0],x[1]),Point(y[0],y[1])]) for (x,y) in zip(obstacle[:-1], obstacle[1:])])
            self.obstacles.append(obstacle)

    def config_validity_checker(self, config, robot_type):
        '''
        Verify that the config (given or stored) does not contain self collisions or links that are out of the world boundaries.
        Return false if the config is not applicable, and true otherwise.
        @param config The given configuration of the robot.
        @param robot_type The type of the robot to return the answer for.
        '''
        if robot_type == 'gripper':
            # compute robot links positions
            robot_positions = self.grip_robot.compute_forward_kinematics(given_config=config)

            # verify that the robot do not collide with itself
            if not self.grip_robot.validate_robot(robot_positions=robot_positions):
                return False
        else:
            robot_positions = self.insp_robot.compute_forward_kinematics(given_config=config)

            # verify that the robot do not collide with itself
            if not self.insp_robot.validate_robot(robot_positions=robot_positions):
                return False

        # verify that all robot joints (and links) are between world boundaries
        non_applicable_poses = [(x[0] < self.xlimit[0] or x[1] < self.ylimit[0] or x[0] > self.xlimit[1] or x[1] > self.ylimit[1]) for x in robot_positions]
        if any(non_applicable_poses):
            return False

        # verify that all robot links do not collide with obstacle edges
        # for each obstacle, check collision with each of the robot links
        robot_links = [LineString([Point(x[0],x[1]),Point(y[0],y[1])]) for x,y in zip(robot_positions.tolist()[:-1], robot_positions.tolist()[1:])]
        for obstacle_edges in self.obstacles_edges:
            for robot_link in robot_links:
                obstacle_collisions = [robot_link.crosses(x) for x in obstacle_edges]
                if any(obstacle_collisions):
                    return False

        return True

    def edge_validity_checker(self, config1, config2, robot_type):
        '''
        A function to check if the edge between two configurations is free from collisions. The function will interpolate between the two states to verify
        that the links during motion do not collide with anything.
        @param config1 The source configuration of the robot.
        @param config2 The destination configuration of the robot.
        @param robot_type The type of the robot to return the answer for.
        '''
        # interpolate between first config and second config to verify that there is no collision during the motion
        #print("im here")
        interpolated_configs, steps = compute_inner_interpolated_configurations(config1=config1, config2=config2)
        if interpolated_configs is not None:
            
            # compute robot links positions for interpolated configs
            if robot_type == 'gripper':
                robot_positions = np.apply_along_axis(self.grip_robot.compute_forward_kinematics, 1, interpolated_configs)

                # verify that the robot do not collide with itself during motion
                for config_positions in robot_positions:
                    if not self.grip_robot.validate_robot(config_positions):
                        return False
            else:
                robot_positions = np.apply_along_axis(self.insp_robot.compute_forward_kinematics, 1, interpolated_configs)

                # verify that the robot do not collide with itself during motion
                for config_positions in robot_positions:
                    if not self.insp_robot.validate_robot(config_positions):
                        return False

            # compute edges between joints to verify that the motion between two configs does not collide with anything
            edges_between_positions = []
            for j in range(1, robot_positions.shape[1]):
                for i in range(steps-1):
                    edges_between_positions.append(LineString([Point(robot_positions[i,j,0],robot_positions[i,j,1]),Point(robot_positions[i+1,j,0],robot_positions[i+1,j,1])]))

            # check collision for each edge between joints and each obstacle
            for edge_pos in edges_between_positions:
                for obstacle_edges in self.obstacles_edges:
                    obstacle_collisions = [edge_pos.crosses(x) for x in obstacle_edges]
                    if any(obstacle_collisions):
                        return False

            # verify that all robot joints (and links) are between world boundaries
            if len(np.where(robot_positions[:,:,0] < self.xlimit[0])[0]) > 0 or \
               len(np.where(robot_positions[:,:,1] < self.ylimit[0])[0]) > 0 or \
               len(np.where(robot_positions[:,:,0] > self.xlimit[1])[0]) > 0 or \
               len(np.where(robot_positions[:,:,1] > self.ylimit[1])[0]) > 0:
               return False

        return True

    def is_gripper_inspected_from_vertex(self, inspector_config, timestamp):
        '''
        A function will compute if the end-effector of the gripping robot is visible by the inspection robot.
        The function will return True if the gripper is visible in terms of distance and field of view (FOV) and is not hidden by any obstacle.
        @param inspector_config The given configuration of the robot.
        @param timestamp The current timestamp in the environment.
        '''
        # get inspecting robot end-effector position and orientation for point of view
        insp_robot_positions = self.insp_robot.compute_forward_kinematics(given_config=inspector_config)
        ee_pos = insp_robot_positions[-1]
        ee_angle = self.insp_robot.compute_ee_angle(given_config=inspector_config)
        
        # define angle range for the ee given its position and field of view (FOV)
        ee_angle_range = np.array([ee_angle - self.insp_robot.ee_fov/2, ee_angle + self.insp_robot.ee_fov/2])

        # compute the point of the gripper at the exact time
        delta_config = (timestamp - np.floor(timestamp)) * (self.gripper_plan[int(np.ceil(timestamp))] - self.gripper_plan[int(np.floor(timestamp))])
        grip_robot_positions = self.grip_robot.compute_forward_kinematics(given_config=self.gripper_plan[int(np.floor(timestamp))] + delta_config)
        gripper_pos = grip_robot_positions[-1]

        # compute angle of the gripper's ee w.r.t. position of the inspector's ee
        relative_gripper_pos = gripper_pos - ee_pos
        gripper_point_angle = self.compute_angle_of_vector(vec=relative_gripper_pos)

        # check that the point of the gripper is potentially visible with the distance from the inspecting end-effector
        if np.linalg.norm(relative_gripper_pos) <= self.insp_robot.vis_dist:

            # if the resulted angle is between the angle range of the ee, verify that there are no interfering obstacles
            if self.check_if_angle_in_range(angle=gripper_point_angle, ee_range=ee_angle_range):

                # define the vector between the two end effectors
                ee_to_gripper_point = LineString([Point(ee_pos[0],ee_pos[1]),Point(gripper_pos[0],gripper_pos[1])]) 

                # check if there are any collisions of the vector with some obstacle edge
                gripper_point_hidden = False
                for obstacle_edges in self.obstacles_edges:
                    for obstacle_edge in obstacle_edges:
                        if ee_to_gripper_point.intersects(obstacle_edge):
                            gripper_point_hidden = True

                # check if there are any intersections of the vector with the inspecting robot
                insp_robot_links_segments = [LineString([Point(x[0],x[1]),Point(y[0],y[1])]) for x,y in zip(insp_robot_positions.tolist()[:-2], insp_robot_positions.tolist()[1:-1])]
                for link_segment in insp_robot_links_segments:
                    if ee_to_gripper_point.intersects(link_segment):
                        gripper_point_hidden = True

                # check if there are any intersections of the vector with the gripping robot
                grip_robot_links_segments = [LineString([Point(x[0],x[1]),Point(y[0],y[1])]) for x,y in zip(grip_robot_positions.tolist()[:-2], grip_robot_positions.tolist()[1:-1])]
                for link_segment in grip_robot_links_segments:
                    if ee_to_gripper_point.intersects(link_segment):
                        gripper_point_hidden = True
                
                return not gripper_point_hidden

        return False

    def is_gripper_inspected_from_edge(self, inspector_config1, inspector_config2, timestamp1, timestamp2):
        '''
        A function will compute if the end-effector of the gripping robot is visible by the inspection robot during an edge.
        The function will update the data structure that is in charge of telling for each step of the gripper, if it was inspected or not.
        @param inspector_config1 The given configuration of the robot in the source vertex.
        @param inspector_config2 The given configuration of the robot in the target vertex.
        @param timestamp1 The timestamp of the source vertex.
        @param timestamp2 The timestamp of the target vertex.
        '''

        # do not look for inspection if the edge does not advance in time
        if timestamp2 - timestamp1 <= 0:
            return

        # iterate over timestamps and compute for each if gripper was inspected
        for timestamp in range(int(np.ceil(timestamp1)),int(np.floor(timestamp2))+1):

            # compute interpolated config for the requested time
            delta_config = ((timestamp - timestamp1) / (timestamp2 - timestamp1)) * (inspector_config2 - inspector_config1)
            current_config = inspector_config1 + delta_config

            # check if gripper is inspected and update accordingly
            is_inspected = self.is_gripper_inspected_from_vertex(inspector_config=current_config, timestamp=timestamp)
            if not self.gripper_inspected[timestamp]:
                self.gripper_inspected[timestamp] = is_inspected

    def compute_inspected_timestamps_for_edge(self, inspector_config1, inspector_config2, timestamp1, timestamp2):
        '''
        A function will compute if the end-effector of the gripping robot is visible by the inspection robot during an edge.
        The function will update the data structure that is in charge of telling for each step of the gripper, if it was inspected or not.
        @param inspector_config1 The given configuration of the robot in the source vertex.
        @param inspector_config2 The given configuration of the robot in the target vertex.
        @param timestamp1 The timestamp of the source vertex.
        @param timestamp2 The timestamp of the target vertex.
        '''

        # do not look for inspection if the edge does not advance in time
        if timestamp2 - timestamp1 <= 0:
            return

        # iterate over timestamps and compute for each if gripper was inspected
        inspected_timestamps = np.zeros(len(self.gripper_plan))
        for timestamp in range(int(np.ceil(timestamp1)),int(np.floor(timestamp2))+1):

            # compute interpolated config for the requested time
            delta_config = ((timestamp - timestamp1) / (timestamp2 - timestamp1)) * (inspector_config2 - inspector_config1)
            current_config = inspector_config1 + delta_config

            # check if gripper is inspected and update accordingly
            inspected_timestamps[timestamp] = self.is_gripper_inspected_from_vertex(inspector_config=current_config, timestamp=timestamp)

        return inspected_timestamps
        
    def compute_angle_of_vector(self, vec):
        '''
        A utility function to compute the angle of the vector from the end-effector to a point.
        @param vec Vector from the end-effector to a point.
        '''
        vec = vec / np.linalg.norm(vec)
        if vec[1] > 0:
            return np.arccos(vec[0])
        else: # vec[1] <= 0
            return -np.arccos(vec[0])

    def check_if_angle_in_range(self, angle, ee_range):
        '''
        A utility function to check if an inspection point is inside the FOV of the end-effector.
        @param angle The angle beteen the point and the end-effector.
        @param ee_range The FOV of the end-effector.
        '''
        # ee range is in the expected order
        if abs((ee_range[1] - self.insp_robot.ee_fov) - ee_range[0]) < 1e-5:
            if angle < ee_range.min() or angle > ee_range.max():
                return False
        # ee range reached the point in which pi becomes -pi
        else:
            if angle > ee_range.min() or angle < ee_range.max():
                return False

        return True

    # ------------------------#
    # Visualization Functions
    # ------------------------#
    def visualize_map(self, gripper_config=None, inspector_config=None, show_map=True):
        '''
        Visualize map with current config of robot and obstacles in the map.
        @param config The requested configuration of the robot.
        @param show_map If to show the map or not.
        '''
        # create empty background
        plt = self.create_map_visualization()

        # add obstacles
        plt = self.visualize_obstacles(plt=plt)

        # add start
        plt = self.visualize_point_location(plt=plt, config=inspector_config, color='r')

        # add robots with given configurations
        if gripper_config is not None:
            plt = self.visualize_robot(plt=plt, config=gripper_config, robot_type='gripper')
        if inspector_config is not None:
            plt = self.visualize_robot(plt=plt, config=inspector_config, robot_type='inspector')

        # show map
        if show_map:
            # plt.show() # replace savefig with show if you want to display map actively
            plt.savefig('map.png')
        
        return plt

    def create_map_visualization(self):
        '''
        Prepare the plot of the scene for visualization.
        '''
        # create figure and add background
        plt.figure()
        back_img = np.zeros((self.ylimit[1]+1, self.xlimit[1]+1))
        plt.imshow(back_img, origin='lower', zorder=0)

        return plt

    def visualize_obstacles(self, plt):
        '''
        Draw the scene's obstacles on top of the given frame.
        @param plt Plot of a frame of the plan.
        '''
        # plot obstacles
        for obstacle in self.obstacles:
            obstacle_xs, obstacle_ys = zip(*obstacle)
            plt.fill(obstacle_xs, obstacle_ys, "y", zorder=5)

        return plt
    
    def visualize_point_location(self, plt, config, color):
        '''
        Draw a point of start/goal on top of the given frame.
        @param plt Plot of a frame of the plan.
        @param config The requested configuration of the point.
        @param color The requested color for the point.
        '''
        # compute point location in 2D
        point_loc = self.insp_robot.compute_forward_kinematics(given_config=config)[-1]

        # draw the circle
        point_circ = plt.Circle(point_loc, radius=5, color=color, zorder=5)
        plt.gca().add_patch(point_circ)
    
        return plt

    def visualize_robot(self, plt, config, robot_type='gripper'):
        '''
        Draw the robot on top of the plt.
        @param plt Plot of a frame of the plan.
        @param config The requested configuration of the robot.
        @param robot_type The type of the robot to return the answer for.
        '''
        # get robot joints and end-effector positions.
        # and add position of robot placement ([0,0] - position of the first joint)

        if robot_type == 'gripper':
            robot_positions = self.grip_robot.compute_forward_kinematics(given_config=config)        
            #robot_positions = np.concatenate([np.expand_dims(self.grip_robot.origin, axis=0), robot_positions])

            # draw the robot
            plt.plot(robot_positions[:,0], robot_positions[:,1], color=self.grip_robot.link_color, linewidth=3.0, zorder=10) # links
            plt.scatter(robot_positions[:,0], robot_positions[:,1], color=self.grip_robot.joint_color, zorder=15) # joints
            plt.scatter(robot_positions[-1:,0], robot_positions[-1:,1], color=self.grip_robot.ee_color, zorder=15) # end-effector

        else:
            robot_positions = self.insp_robot.compute_forward_kinematics(given_config=config)
            #robot_positions = np.concatenate([np.expand_dims(self.insp_robot.origin, axis=0), robot_positions])

            # draw the robot
            plt.plot(robot_positions[:,0], robot_positions[:,1], color=self.insp_robot.link_color, linewidth=3.0, zorder=10) # links
            plt.scatter(robot_positions[:,0], robot_positions[:,1], color=self.insp_robot.joint_color, zorder=15) # joints
            plt.scatter(robot_positions[-1:,0], robot_positions[-1:,1], color=self.insp_robot.ee_color, zorder=15) # end-effector

            # add "visibility cone" to demonstrate what the robot sees
            # define the length of the cone and origin
            visibility_radius = 15
            cone_origin = robot_positions[-1,:].tolist()

            # compute a pixeled arc for the cone
            robot_ee_angle = self.insp_robot.compute_ee_angle(given_config=config)
            robot_fov_angles = np.linspace(start=self.insp_robot.ee_fov/2, stop=-self.insp_robot.ee_fov/2, num=visibility_radius)
            robot_fov_angles = np.expand_dims(np.tile(robot_ee_angle, robot_fov_angles.size) + robot_fov_angles, axis=0)
            robot_ee_angles = np.apply_along_axis(self.get_normalized_angle, 0, robot_fov_angles)
            robot_ee_xs = cone_origin[0] + visibility_radius * np.cos(robot_ee_angles)
            robot_ee_ys = cone_origin[1] + visibility_radius * np.sin(robot_ee_angles)

            # append robot ee location and draw polygon
            robot_ee_xs = np.append(np.insert(robot_ee_xs, 0, cone_origin[0]), cone_origin[0])
            robot_ee_ys = np.append(np.insert(robot_ee_ys, 0, cone_origin[1]), cone_origin[1])
            plt.fill(robot_ee_xs, robot_ee_ys, color=self.insp_robot.sensor_color, zorder=13, alpha=0.5)

        return plt

    def get_normalized_angle(self, angle):
        '''
        A utility function to get the normalized angle of the end-effector
        @param angle The angle of the robot's ee
        '''
        if angle > np.pi:
            return angle - 2*np.pi
        elif angle < -np.pi:
            return angle + 2*np.pi
        else:
            return angle

    def interpolate_inspector_plan(self, plan, plan_timestamps):
        '''
        Interpolate plan of configurations - add steps between each two configs to make visualization smoother and adjusted to 
        the plan of the gripper robot.
        @param plan Sequence of configs defining the plan.
        @param plan_timestamps Sequence of timestamps, each related to the adjacent config.
        '''
        plan_interpolated = [np.expand_dims(plan[0], axis=0)]
        for t in range(len(plan_timestamps)-1):

            # create steps according to number of timestamps between each step
            timestamps_range = np.arange(0, int(np.floor(plan_timestamps[t+1]))+1 - int(np.floor(plan_timestamps[t])), 1)
            timestamps_range_per_joint = np.tile(np.expand_dims(timestamps_range, axis=1), [1, self.insp_robot.dim])
            step = (plan[t+1] - plan[t]) / (plan_timestamps[t+1] - plan_timestamps[t])
            steps = np.tile(np.expand_dims(step, axis=0), [len(timestamps_range),1])

            # compute linear interpolation of config motion
            interpolated_steps = np.tile(np.expand_dims(plan[t], axis=0), [len(timestamps_range),1])
            interpolated_steps += np.multiply(steps, timestamps_range_per_joint)
            plan_interpolated.append(interpolated_steps[1:])

        plan_interpolated = np.concatenate(plan_interpolated)

        # fill gap at the end if missing steps are required
        if len(self.gripper_plan) > len(plan_interpolated):
            num_steps = len(self.gripper_plan) - len(plan_interpolated)

            plan_interpolated = np.concatenate([plan_interpolated ,np.tile(np.expand_dims(plan_interpolated[-1], axis=0), [num_steps, 1])])
        
        return plan_interpolated

    def visualize_plan(self, plan, plan_timestamps):
        '''
        Visualize the final plan as a GIF and stores it.
        @param plan Sequence of configs defining the plan.
        '''
        # switch backend - possible bugfix if animation fails
        #matplotlib.use('TkAgg')

        # interpolate plan and of inspector robot
        plan_p = self.interpolate_inspector_plan(plan=plan, plan_timestamps=plan_timestamps)

        # visualize each step of the given plan
        plan_images = []
        for i in range(len(self.gripper_plan)):

            # create background, obstacles, start and goal
            plt = self.create_map_visualization()
            plt = self.visualize_obstacles(plt=plt)
            plt = self.visualize_point_location(plt=plt, config=self.inspector_start, color='r')

            # add robots with current plan steps
            plt = self.visualize_robot(plt=plt, config=self.gripper_plan[i], robot_type='gripper')
            plt = self.visualize_robot(plt=plt, config=plan_p[i], robot_type='inspector')

            # convert plot to image
            canvas = plt.gca().figure.canvas
            canvas.draw()
            data = np.fromstring(canvas.tostring_rgb(), dtype=np.uint8, sep='')
            data = data.reshape(canvas.get_width_height()[::-1] + (3,))
            plan_images.append(data)
        
        # store gif
        plan_time = datetime.now().strftime("%d-%m-%Y_%H:%M:%S")
        imageio.mimsave(f'plan_{plan_time}.gif', plan_images, 'GIF', duration=0.05)
