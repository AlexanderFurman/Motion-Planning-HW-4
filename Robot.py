import itertools
import numpy as np
from shapely.geometry import Point, LineString

class Robot(object):
    
    def __init__(self, origin=[0,0]):

        # define default robot properties (might be redefined later by inheriting class)
        self.origin = np.array(origin)
        self.links = np.array([80.0,70.0,40.0,40.0])
        self.dim = len(self.links)

        # colors - for visualization
        self.link_color = 'orange'
        self.joint_color = 'coral'
        self.ee_color = 'cornflowerblue'

    def compute_distance(self, prev_config, next_config):
        '''
        Compute the euclidean distance betweeen two given configurations.
        @param prev_config Previous configuration.
        @param next_config Next configuration.
        '''
        return np.linalg.norm(next_config - prev_config)

    def compute_forward_kinematics(self, given_config):
        '''
        Compute the 2D position (x,y) of each one of the links (including end-effector) and return.
        @param given_config Given configuration.
        '''
        # positions are 2D points for each of the links of the robot (5 including starting point and end-effector)
        robot_positions = np.zeros((given_config.size, 2))
        robot_positions = np.concatenate([np.expand_dims(self.origin, axis=0), robot_positions])
        for i in range(1, len(robot_positions)):
            # if i>0:
            robot_positions[i,] = robot_positions[i-1,]
            
            # compute accumulated p(x,y) for joint i (based on DH convention for a X-DOF manipulator)
            robot_positions[i,0] += self.links[i-1]*np.cos(given_config[:i].sum())
            robot_positions[i,1] += self.links[i-1]*np.sin(given_config[:i].sum())
        
        return robot_positions
        
    def validate_robot(self, robot_positions):
        '''
        Verify that the given set of links positions does not contain self collisions.
        @param robot_positions Given links positions.
        '''
        # check intersections between any two segments of links, return if found
        links_segments = [LineString([Point(x[0],x[1]),Point(y[0],y[1])]) for x,y in zip(robot_positions.tolist()[:-1], robot_positions.tolist()[1:])]
        links_intersections = [x[0].crosses(x[1]) for x in itertools.combinations(links_segments, 2)]
        if any(links_intersections):
            return False

        return True


class GrippingRobot(Robot):
    
    def __init__(self):
        super(GrippingRobot, self).__init__()

        # define robot properties
        self.links = np.array([80.0,70.0,40.0,60.0])
        self.dim = len(self.links)

    
class InspectingRobot(Robot):
    
    def __init__(self, origin):
        super(InspectingRobot, self).__init__(origin=origin)

        # colors
        self.sensor_color = 'mediumpurple'
        self.link_color = 'yellowgreen'
        self.joint_color = 'teal'
        self.ee_color = 'cadetblue'

        # robot field of fiew (FOV) for inspecting points, from [-np.pi/6, np.pi/6]
        self.ee_fov = np.pi/3

        # visibility distance for the robot's end-effector. Farther than that, the robot won't see any points.
        self.vis_dist = 200.0

    def compute_ee_angle(self, given_config):
        '''
        Compute the 1D orientation of the end-effector w.r.t. world origin (or first joint)
        @param given_config Given configuration.
        '''
        ee_angle = given_config[0]
        for i in range(1,len(given_config)):
            ee_angle = self.compute_link_angle(ee_angle, given_config[i])

        return ee_angle

    def compute_link_angle(self, link_angle, given_angle):
        '''
        Compute the 1D orientation of a link given the previous link and the current joint angle.
        @param link_angle previous link angle.
        @param given_angle Given joint angle.
        '''
        if link_angle + given_angle > np.pi:
            return link_angle + given_angle - 2*np.pi
        elif link_angle + given_angle < -np.pi:
            return link_angle + given_angle + 2*np.pi
        else:
            return link_angle + given_angle