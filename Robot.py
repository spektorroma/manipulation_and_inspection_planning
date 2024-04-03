import itertools
import numpy as np
from matplotlib import pyplot as plt
from numpy.core.fromnumeric import size
from shapely.geometry import Point, LineString

class Robot(object):
    
    def __init__(self):

        # define robot properties
        self.links = np.array([80.0,70.0,40.0,40.0])
        self.dim = len(self.links)

        # robot field of fiew (FOV) for inspecting points, from [-np.pi/6, np.pi/6]
        self.ee_fov = np.pi/3

        # visibility distance for the robot's end-effector. Farther than that, the robot won't see any points.
        self.vis_dist = 60.0

    def compute_distance(self, prev_config, next_config):
        '''
        Compute the euclidean distance betweeen two given configurations.
        @param prev_config Previous configuration.
        @param next_config Next configuration.
        '''
        # TODO: Task 2.2
        return np.linalg.norm(np.array(self.compute_forward_kinematics(next_config)) - np.array(self.compute_forward_kinematics(prev_config)))

    def compute_forward_kinematics(self, given_config):
        '''
        Compute the 2D position (x,y) of each one of the links (including end-effector) and return.
        @param given_config Given configuration.
        '''
        # TODO: Task 2.2
        given_config_to_0 = np.cumsum(given_config)

        cosLen = np.cos(given_config_to_0) * self.links
        sinLen = np.sin(given_config_to_0) * self.links

        x = np.cumsum(cosLen)
        y = np.cumsum(sinLen)

        return np.transpose(np.array([x, y]))
            



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
        
    def validate_robot(self, robot_positions):
        '''
        Verify that the given set of links positions does not contain self collisions.
        @param robot_positions Given links positions.
        '''
        # TODO: Task 2.2
        lenght = len(robot_positions)
        for i in range(lenght - 1):
            firstStart = np.array([robot_positions[i][0], robot_positions[i][1]])
            firstFinish = np.array([robot_positions[i+1][0], robot_positions[i+1][1]])
            l1 = LineString([firstStart, firstFinish])
            for j in range(lenght - i - 3):
                secondStart = np.array([robot_positions[lenght - j - 1][0], robot_positions[lenght - j - 1][1]])
                secondFinish = np.array([robot_positions[lenght - j - 2][0], robot_positions[lenght - j - 2][1]])
                l2 = LineString([secondStart, secondFinish])
                if l1.intersects(l2):
                    return False
        return True

    