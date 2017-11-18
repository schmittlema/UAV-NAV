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

from gym import utils, spaces
import gazebo_env
from gym.utils import seeding

from mavros_msgs.msg import OverrideRCIn, PositionTarget,State
from sensor_msgs.msg import LaserScan, NavSatFix,Image
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

        if not self.depth_cam:
            self.pose.pose.position.y = 2.5
            while not rospy.is_shutdown() and not self.at_target(self.cur_pose,self.pose,0.3):
                self.local_pos.publish(self.pose)
                self.rate.sleep()


        self.collision = False
        self.started = True
        print self.started
        print "Initialized"


    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "stereo.launch")    

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pos_cb)
        rospy.Subscriber('/stereo/depth_raw', Image, callback=self.callback_observe)

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

        #attitude related variables
        self.cur_vel = TwistStamped()
        self.pid = a_PID()
        self.vpid = v_PID()
        self.y_vel = 2
        self.x_accel = 0
        self.actions = [-5.0,-2.5,0,2.5,5.0]
        self.step_length = 1


    def pos_cb(self,msg):
        self.cur_pose = msg

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
        if self.cur_pose.pose.position.y > 40 or self.cur_pose.pose.position.y < 0:
	    print "GOAL"
            done = True
            self.steps = 0
	    self.successes += 1
            self.hard_reset()
        return done,reward

    def dangerous(self,action):
        #TODO
        return False

    def _step(self, action):
        self.steps += 1

        if self.dangerous(action):
            print "autopilot"
            self.auto_steps += 1
            self.network_stepped = False
            #TODO Change action
        else:
            print "deep learner"
            self.network_steps += 1
            self.network_stepped = True

        self.rate.sleep()
        self.x_accel = self.actions[action]
        last_request = rospy.Time.now()
        observation = self.observe()
        reward = self.get_reward(action)
        print self.actions[action],reward

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
