import gym
import numpy as np
import os
import rospy
import roslaunch
import subprocess
import time
import math
from random import randint
import copy as cp

from gym import utils, spaces
import gazebo_env
from gym.utils import seeding

from mavros_msgs.msg import OverrideRCIn, PositionTarget,State
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Float64;
from gazebo_msgs.msg import ModelStates,ModelState,ContactsState
from gazebo_msgs.srv import SetModelState

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped,Pose,Vector3,Twist,TwistStamped
from std_srvs.srv import Empty
from VelocityController import VelocityController
from nav_msgs.msg import OccupancyGrid, MapMetaData

class Slam():
    def __init__(self):
        # Connect to map
        #rospy.init_node('slam', anonymous=True)

        rospy.Subscriber('/rtabmap/grid_map',OccupancyGrid,self.call_map)
        rospy.Subscriber('/stereo_odometer/pose',PoseStamped,self.call_pose)

	self.map_publisher = rospy.Publisher('/slam/map',OccupancyGrid,queue_size=10)

        self.reset_proxy = rospy.ServiceProxy('/rtabmap/reset', Empty)

        self.rate = rospy.Rate(10)
        self.map = OccupancyGrid()
        self.local_map = []
        self.pose = PoseStamped()
        self.tlibrary = self.read_dictionary()
        self.sep_dict = self.seperate_dictionary(self.tlibrary)
        
    def call_map(self,msg):
        rospy.wait_for_message('/rtabmap/grid_map',PoseStamped,timeout=5)
        rospy.wait_for_message('/stereo_odometer/pose',PoseStamped,timeout=5)
        bubble = self.add_bubble(self.pose)
        msg.data = np.reshape(np.array(msg.data),(-1,msg.info.width))
        #grid_pos = self.convert_to_grid_cells(self.pose)
        #msg.data[grid_pos[1]][grid_pos[0]] = 50
        #origin_pos = self.convert_to_origin_cells(msg.info.origin)
        #msg.data[origin_pos[1]][origin_pos[0]] = 20
        for b in bubble:
            try:
                msg.data[b[1]][b[0]] = 0
            except IndexError:
                pass
        for d in self.convert_dictionary_to_cells(self.tlibrary):
            try:
                msg.data[d[1]][d[0]] = 40
            except IndexError:
                pass
        msg.data = msg.data.flatten()
        self.map = msg
        self.local_map = np.reshape(np.array(self.map.data),(-1,self.map.info.width))
        #self.local_map.fill(0)
        #self.reset_proxy()

    def call_pose(self,msg):
        self.pose = msg
    
    def flatten_dictionary(self,dictionary):
        flat_array = []
        i = 0
        for x in dictionary:
            for y in x:
                flat_array.append(dictionary[i][y])
            i+=1
        return flat_array

    def seperate_dictionary(self,dictionary):
        flat_array = []
        i = 0
        for x in dictionary:
            flat_array.append([])
            for y in x:
                flat_array[i].append(dictionary[i][y])
            i+=1
        return flat_array

    def convert_dictionary_to_cells(self,dictionary):
        array = self.flatten_dictionary(dictionary)
        temp_pose = PoseStamped()
        temp_pose.pose.position.z = 2
        output_array = []
        for a in array:
            temp_pose.pose.position.x = a[0]
            temp_pose.pose.position.y = a[1]
            temp_pose = self.convert_to_local_frame(temp_pose)
            output_array.append(self.convert_to_grid_cells(temp_pose))
        return output_array

    def read_dictionary(self):
        library = []
        with open('GYM/env/trajectory_library.txt','r') as infile:
            for line in infile:
                if line[0] == "{":
                    library.append(eval(line))
        return library

    def convert_to_local_frame(self,input_pos):
        output_pos = PoseStamped()
        output_pos.pose.position.x = input_pos.pose.position.x + self.pose.pose.position.x
        output_pos.pose.position.y = input_pos.pose.position.y + self.pose.pose.position.y
        output_pos.pose.position.z = input_pos.pose.position.z + self.pose.pose.position.z
        return output_pos

    def convert_to_origin_cells(self,input_pos):
        rospy.wait_for_message('/rtabmap/grid_map',PoseStamped,timeout=5)
        resolution = max(self.map.info.resolution,0.04)
        origin = self.map.info.origin
        cell_x = int((input_pos.position.x - origin.position.x)/resolution)
        cell_y = int((input_pos.position.y - origin.position.y)/resolution)
        return [cell_x,cell_y]

    def convert_to_grid_cells(self,input_pos):
        rospy.wait_for_message('/rtabmap/grid_map',PoseStamped,timeout=5)
        resolution = max(self.map.info.resolution,0.04)
        origin = self.map.info.origin
        cell_x = int((input_pos.pose.position.x - origin.position.x)/resolution)
        cell_y = int((input_pos.pose.position.y - origin.position.y)/resolution)
        return [cell_x,cell_y]

    def add_bubble(self,input_p):
        input_pos = cp.deepcopy(input_p)
        bubble = []
        resolution = self.map.info.resolution
        grid_pos = self.convert_to_grid_cells(input_pos)
        radius = 0.3556
        #radius = 0.5
        input_pos.pose.position.x = input_pos.pose.position.x + radius
        s1 = self.convert_to_grid_cells(input_pos)[0]
        input_pos.pose.position.x = input_pos.pose.position.x - 2*radius
        s2 = self.convert_to_grid_cells(input_pos)[0]

        input_pos.pose.position.y = input_pos.pose.position.y + radius
        s3 = self.convert_to_grid_cells(input_pos)[1]
        input_pos.pose.position.y = input_pos.pose.position.y - 2*radius
        s4 = self.convert_to_grid_cells(input_pos)[1]
        for x in range(s2,s1):
            for y in range(s4,s3):
                bubble.append([x,y])
        return bubble 

    def convert_dict_to_pose(self,array):
        pos = PoseStamped()
        pos.pose.position.z = 2
        pos.pose.position.x = array[0]
        pos.pose.position.y = array[1]
        return pos

    def bubble_overlap(self,bubble):
        uobst = 0.0
        for b in bubble:
            try:
                if (self.local_map[b[1]][b[0]] == -1 or self.local_map[b[1]][b[0]] >50):
                    uobst = uobst +1
            except IndexError:
                uobst+=1
        return uobst/float(len(bubble)) > 0.03

    def check_collision(self,points):
        for x in points:
            bubble = self.add_bubble(self.convert_to_local_frame(self.convert_dict_to_pose(x)))
            if self.bubble_overlap(bubble):
                return True
        return False

    def closest_angle(self,heading):
        angles = {0:math.pi/2,1:(3 *math.pi)/4,2:math.pi,3:math.pi/4,4:0}
        for a in range(len(angles)):
            angles[a] = abs(angles[a]-heading)
        return angles

    def auto_pilot_step(self,heading):
        paths = self.sep_dict
        possible_paths = {}
        most_direct = self.closest_angle(heading)
        for p in paths:
            if not self.check_collision(p):
                possible_paths[paths.index(p)] = most_direct[paths.index(p)]
        #print possible_paths
        if len(possible_paths) == 0:
            return -1
        else:
            return min(possible_paths,key=possible_paths.get)

    def crop_map(self,input_map):
        #TODO
        print "TODO"


