import gym
import numpy as np
import os
import rospy
import roslaunch
import subprocess
import time
import math
from random import randint
from slam import Slam
import copy as cp
import ast

from gym import utils, spaces
import gazebo_env
from gym.utils import seeding

from mavros_msgs.msg import OverrideRCIn, PositionTarget,State
from sensor_msgs.msg import LaserScan, NavSatFix,Image,PointCloud2,Imu
from std_msgs.msg import Float64;
from gazebo_msgs.msg import ModelStates,ModelState,ContactsState
from gazebo_msgs.srv import SetModelState, GetModelState

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped,Pose,Vector3,Twist,TwistStamped
from std_srvs.srv import Empty
from VelocityController import VelocityController
import cv2
from cv_bridge import CvBridge, CvBridgeError

#For attitude control
from attitude_PID import a_PID
from vel_PID import v_PID
import sensor_msgs.point_cloud2 as pc2
import pcl


class GazeboQuadEnv(gazebo_env.GazeboEnv):
    def _takeoff(self):
	print "Got Mavros"
        last_request = rospy.Time.now()
        #rospy.wait_for_service('mavros/set_mode')
        #rospy.wait_for_service('mavros/cmd/arming')
        self.rate = rospy.Rate(0.5)
        self.rate.sleep()
        self.rate = rospy.Rate(10)
        while not self.state.armed:
            if rospy.Time.now() - last_request > rospy.Duration.from_sec(5):
                self._reset()
                # Set OFFBOARD mode
                #Must be sending points before connecting to offboard
                for i in range(0,100):
                    self.local_pos.publish(self.pose)
                if self.state.mode != "OFFBOARD":
                    try:
                        success = self.mode_proxy(0,'OFFBOARD')
                        print success
                        print "offboard enabled"
                    except rospy.ServiceException, e:
                        print ("mavros/set_mode service call failed: %s"%e)

                if not self.state.armed:
                    print "arming"
                    try:
                       success = self.arm_proxy(True)
                       print success
                       print "armed"
                    except rospy.ServiceException, e:
                       print ("mavros/set_mode service call failed: %s"%e)
                last_request = rospy.Time.now()
                self.local_pos.publish(self.pose)
            self.rate.sleep()
        self.setup_position()


    def setup_position(self):
        print "Moving To Position..."
        self.pose.pose.position.y = 0
        while not rospy.is_shutdown() and not self.at_target(self.cur_pose,self.pose,0.3):
            if self.collision:
                self.hard_reset()
                return
            self.local_pos.publish(self.pose)
            self.rate.sleep()
        #while not rospy.is_shutdown():
         #   print "Forward"
          #  self.check_nearest_neighbor(self.radius,[0,5,0])
           # print "Origin"
            #print self.cur_pose.pose.position
            #self.check_nearest_neighbor(self.radius,[0,0,0])
            #self.local_pos.publish(self.pose)
            #self.rate.sleep()

        self.collision = False
        self.started = True
        print self.started
        print "Initialized"


    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "stereo.launch")    

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pos_cb)
        rospy.Subscriber('/stereo/depth_raw', Image, callback=self.callback_observe)
        rospy.Subscriber('/stereo/points2', PointCloud2, callback=self.stereo_cb)

        rospy.Subscriber('/mavros/state',State,callback=self.state_cb)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
	
	self.model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	self.model_position = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        
        self.reset_odometry = rospy.ServiceProxy('/stereo_odometer/reset_pose', Empty)

	self.local_pos = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=10)

        self.mode_proxy = rospy.ServiceProxy('mavros/set_mode', SetMode)

        self.arm_proxy = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        self.att_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude',PoseStamped,queue_size=10)

        self.throttle_pub = rospy.Publisher('/mavros/setpoint_attitude/att_throttle',Float64,queue_size=10)

        self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, callback=self.vel_cb)

        self.accel_sub = rospy.Subscriber('/mavros/imu/data', Imu, callback=self.imu_cb)

        rospy.Subscriber('/bumper_rotor0',ContactsState,self.callbackrotor)
        rospy.Subscriber('/bumper_rotor1',ContactsState,self.callbackrotor)
        rospy.Subscriber('/bumper_rotor2',ContactsState,self.callbackrotor)
        rospy.Subscriber('/bumper_rotor3',ContactsState,self.callbackrotor)

        self.state = ""
	
	self.successes = 0
        self.collision = False

        self.cur_pose = PoseStamped()

        self.pose = PoseStamped()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 2.5 
        self.pose.pose.position.z = 2
        self.pose.pose.orientation.x = 0
        self.pose.pose.orientation.y = 0
        self.pose.pose.orientation.z = 0.707 
        self.pose.pose.orientation.w = 0.707
        self.network_stepped = True

        self.started = False
        self.rate = rospy.Rate(10)
        self.collisions = 0
        self.auto_steps = 0
        self.network_steps = 0

        self.pause_sim = False
        self.steps = 0
        self.target = 0
        self.temp_pause = False
        self.accuracy = 0.2

        self.action = 0
        self.next_move = False
        self.takeover = False
        self.depth_cam = False
        self.mono_image = False
        self.bridge = CvBridge()

        #radius of drone + extra space
        self.radius = 1.0

        #Percent of danger allowed
        self.threshold = 0.05

        #attitude related variables
        self.cur_vel = TwistStamped()
        self.pid = a_PID()
        self.vpid = v_PID()
        self.y_vel = 2
        self.x_accel = 0
        self.actions = [-5.0,-2.5,0,2.5,5.0]
        self.cur_imu = Imu()
    
        #PointCloud
        self.kdtree = 0

        #Reading In Samples
        #self.vel_rows = [-5,-4.5,-4,-3.5,-3,-2.5,-2,-1.5,-1,-0.5,0,0.5,1,1.5,2,2.5,3,3.5,4,4.5,5] 
        #self.accel_rows = [-5,-4.5,-4,-3.5,-3,-2.5,-2,-1.5,-1,-0.5,0,0.5,1,1.5,2,2.5,3,3.5,4,4.5,5] 
        self.vel_rows = [-5,-4,-3,-2,-1,0,1,2,3,4,5]
        self.accel_rows = [-5,-4,-3,-2,-1,0,1,2,3,4,5]

        log = open("GYM/env/log_sample.txt","r")
        array = log.readline()
        self.array = self.process(ast.literal_eval(array),self.accel_rows,self.vel_rows)
        log.close()

    def pos_cb(self,msg):
        self.cur_pose = msg

    def imu_cb(self,msg):
        self.cur_imu = msg

    def stereo_cb(self,msg):
        points = pc2.read_points(msg,skip_nans=True,field_names=("x","y","z"))
        p_array = []
        for p in points:
            p_array.append(p)

        pointcloud = pcl.PointCloud()
        pointcloud.from_list(p_array)
        if pointcloud.size > 0:
            self.kd_tree = pcl.KdTreeFLANN(pointcloud)

    def vel_cb(self,msg):
        self.cur_vel = msg

    def callback_observe(self,msg):
        try:
            raw_image = self.bridge.imgmsg_to_cv2(msg,"passthrough")
            resized = cv2.resize(raw_image,(50,50),interpolation=cv2.INTER_AREA)
        except CvBridgeError, e:
            print e
        self.mono_image = resized.flatten()
        #For debugging
        #cv2.imshow('test',resized)
        #cv2.waitKey(0)

    def process(self,array,accel_rows,vel_rows):
        accel_range = len(accel_rows)
        vel_range = len(vel_rows)
	events = []
	last_index = -1
	for item in array:
	    if last_index == item[0]:
		events[last_index].append(item)
	    else:
		last_index = item[0]
		events.append([])

	base = [None]*accel_range
	for i in range(accel_range):
	    base[i]=[None]*vel_range

	lookup_table = {'0':cp.deepcopy(base),'1':cp.deepcopy(base),'2':cp.deepcopy(base),'3':cp.deepcopy(base),'4':cp.deepcopy(base)}
	section = lookup_table['1']
	for event in events:
	    section = lookup_table[str(event[0][1])]
	    vel = round(event[0][3])
	    accel = round(event[0][4])

	    if abs(accel)>max(accel_rows):
		if accel < 0:
		    accel = -1* max(accel_rows)
		else:
		    accel = max(accel_rows)
	    if abs(vel)>max(vel_rows):
		if vel<0:
		    vel = -1 * max(vel_rows)
		else:
		    vel = max(vel_rows)
	    vpos = vel_rows.index(vel)
	    apos = accel_rows.index(accel)
	    if section[apos][vpos] == None:
		section[apos][vpos] = []

	    section[apos][vpos].append(event)
	    lookup_table[str(event[0][1])] = section

	return lookup_table


    def filter_correct(self):
        pose = self.model_position('f450-stereo','world')
        return self.at_target(self.cur_pose,pose,5)

    def at_target(self,cur,target,accuracy):
        return (abs(cur.pose.position.x - target.pose.position.x) < accuracy) and (abs(cur.pose.position.y - target.pose.position.y) < accuracy) and (abs(cur.pose.position.z - target.pose.position.z) <accuracy) 

    def state_cb(self,msg):
        self.state = msg

    def start(self):
	counter = 15
	while counter != 0:
	    counter = counter -1
 	    time.sleep(1)
        
        self.get_data()

        self._takeoff()
        att_pos = PoseStamped()

        print "Main Running"
        while not rospy.is_shutdown():
            if not self.temp_pause:
                yacel = self.vpid.update(self.cur_vel.twist.linear.y,self.y_vel)
                yacel = 0
                w,i,j,k,thrust = self.pid.generate_attitude_thrust(self.x_accel,yacel,0,self.cur_pose.pose.position.z,self.cur_vel.twist.linear.z)
                att_pos.pose.orientation.x = i
                att_pos.pose.orientation.y = j 
                att_pos.pose.orientation.z = k 
                att_pos.pose.orientation.w = w
                self.att_pub.publish(att_pos)
                self.throttle_pub.publish(thrust)
            self.rate.sleep()
            self.pauser()

    def pauser(self):
        if self.pause_sim and not rospy.is_shutdown():
            rospy.wait_for_service('/gazebo/pause_physics')
            try:
                self.pause()
            except rospy.ServiceException, e:
                print ("/gazebo/pause_physics service call failed")
            while self.pause_sim and not rospy.is_shutdown():
                self.rate.sleep()

            rospy.wait_for_service('/gazebo/unpause_physics')
            try:
                self.unpause()
            except rospy.ServiceException, e:
                print ("/gazebo/unpause_physics service call failed")

    def callbackrotor(self,data):
        if len(data.states) > 0:
            if data.states[0].collision2_name != "":
                self.collision = True
                print "COLLISION"

    def observe(self):
        return self.mono_image

    def reset_quad(self):
        modelstate = ModelState()
        modelstate.model_name = "f450-stereo"
        modelstate.pose.position.z = 0.1
        modelstate.pose.position.x = 0
        modelstate.pose.position.y = -2
        modelstate.pose.orientation.z = 0.707 
        modelstate.pose.orientation.w = 0.707 
        self.model_state(modelstate)

    def wait_until_start(self):
        while True:
            if self.started:
                break
            self.rate.sleep()
        return

    def check_nearest_neighbor(self,radius,point):
        pc_point = pcl.PointCloud()
        point = [point[0],point[2],point[1]]
        pc_point.from_list([point])
        nearest =  self.kd_tree.nearest_k_search_for_cloud(pc_point,5)[1][0]
        #print nearest
        for kpoint in nearest:
            if kpoint < radius:
                return True
        return False

    def get_data(self):
        print "Waiting for mavros..."
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/mavros/global_position/rel_alt', Float64, timeout=5)
            except:
                pass

    def detect_done(self,reward):
        done = False
        if self.collision:
	    print "CRASH"
            done = True
            self.steps = 0
            self.collisions += 1
	    self.hard_reset()
        if self.cur_pose.pose.position.y > 40:
	    print "GOAL"
            done = True
            self.steps = 0
	    self.successes += 1
            self.hard_reset()
        return done,reward

    def bin(self,accel,vel,vel_rows,accel_rows):
        vel = round(vel)
        accel = round(accel)

        if abs(accel)>max(accel_rows):
            if accel < 0:
                accel = -1* max(accel_rows)
            else:
                accel = max(accel_rows)
        if abs(vel)>max(vel_rows):
            if vel<0:
                vel = -1 * max(vel_rows)
            else:
                vel = max(vel_rows)
        vpos = vel_rows.index(vel)
        apos = accel_rows.index(accel)
        return vpos,apos

    def dangerous(self,action):
        vpos,apos = self.bin(self.cur_imu.linear_acceleration.x,self.cur_vel.twist.linear.x,self.vel_rows,self.accel_rows)
        points = self.array[str(action)][apos][vpos]
        if points == None:
            #Conservative behavior for moves that have no data
            print "True",apos,vpos,self.actions[action] 
            return True
        else:
            if len(points) > 20:
                points = points[:20]
            local_percent = 0.0 
            total_points = 0.0
            for point in points:
                for subpoint in point:
                    total_points+=1.0
                    position = [subpoint[2][0],subpoint[2][1],0]
                    if self.check_nearest_neighbor(self.radius,position):
                        local_percent+=1

            print local_percent/total_points,local_percent,total_points
            return local_percent/total_points > self.threshold



    def _step(self, action):
        self.steps += 1

        if True: #self.dangerous(action):
            print "autopilot"
            self.auto_steps += 1
            self.network_stepped = False
            action = -1
            for a in range(len(self.actions)):
               if not self.dangerous(a):
                  action = a
                  print self.actions[action]
                  break #will take first safe action. Currently biased to left
        else:
            print "deep learner"
            self.network_steps += 1
            self.network_stepped = True

        action = -1

        if action == -1:
            self.y_vel = 0
            action = 2
            print "stop"
        else:
            self.y_vel = 2

        self.rate.sleep()
        self.x_accel = self.actions[action]
        last_request = rospy.Time.now()
        observation = self.observe()
        reward = self.get_reward(action)

        done,reward = self.detect_done(reward) 
        return observation, reward,done,{}

    def sigmoid(self,x):
        return (1 / (1 + math.exp(-x)))*2

    def get_reward(self,action):
        #TODO
        return 1+1

    def _killall(self, process_name):
        pids = subprocess.check_output(["pidof",process_name]).split()
        for pid in pids:
            os.system("kill -9 "+str(pid))

    def _reset(self):
        return self.observe()

    def land(self):
        while self.state.mode != "AUTO.LAND":
            try:
                success = self.mode_proxy(0,'AUTO.LAND')
                print "land enabled"
            except rospy.ServiceException, e:
                print ("mavros/set_mode service call failed: %s"%e)


    def hard_reset(self):
        # Resets the state of the environment and returns an initial observation.
        print "resetting"
        self.collision = False
        self.temp_pause = True
        self.land()
        while self.cur_pose.pose.position.z > 0.2:
            self.rate.sleep()
        self.reset_quad()
        while not self.filter_correct():
            self.rate.sleep()

        if self.state.armed:
            print "disarming"
            try:
               success = self.arm_proxy(False)
               print success
               print "disarmed"
            except rospy.ServiceException, e:
               print ("mavros/set_mode service call failed: %s"%e)

        self.action = 0
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = -2
        self.pose.pose.position.z = 2
        self.pose.pose.orientation.x = 0
        self.pose.pose.orientation.y = 0
        self.pose.pose.orientation.z = 0.707 
        self.pose.pose.orientation.w = 0.707
        self._takeoff()
        self.temp_pause = False

        print "hard world reset"
        print self.action
        return self.observe()
