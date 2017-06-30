import gym
import numpy as np
import os
import rospy
import roslaunch
import subprocess
import time
import math
from random import randint

from gym import utils, spaces
import gazebo_env
from gym.utils import seeding

from mavros_msgs.msg import OverrideRCIn, PositionTarget,State
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Float64;
from gazebo_msgs.msg import ModelStates,ModelState
from gazebo_msgs.srv import SetModelState

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped,Pose,Vector3,Twist,TwistStamped
from std_srvs.srv import Empty
from VelocityController import VelocityController

#For Stereo
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt


class GazeboQuadEnv(gazebo_env.GazeboEnv):
    def _takeoff(self):
	print "Got Mavros"
        last_request = rospy.Time.now()
        while self.state.mode != "OFFBOARD" or not self.state.armed:
            if rospy.Time.now() - last_request > rospy.Duration.from_sec(5):
                self._reset()
                # Set OFFBOARD mode
                rospy.wait_for_service('mavros/set_mode')
                
                #Must be sending points before connecting to offboard
                for i in range(0,100):
                    self.local_pos.publish(self.pose)
                if self.state.mode != "OFFBOARD":
                    try:
                        success = self.mode_proxy(0,'OFFBOARD')
                        print success
                    except rospy.ServiceException, e:
                        print ("mavros/set_mode service call failed: %s"%e)
                    print "offboard enabled"

                if not self.state.armed:
                    print "arming"
                    rospy.wait_for_service('mavros/cmd/arming')
                    try:
                       success = self.arm_proxy(True)
                       print success
                    except rospy.ServiceException, e:
                       print ("mavros/set_mode service call failed: %s"%e)
                    print "armed"
                last_request = rospy.Time.now()
            self.local_pos.publish(self.pose)
            self.rate.sleep()

        timeout = 150
        err = 1
        while err > .3:
            err = abs(self.pose.pose.position.z - self.cur_pose.pose.position.z)
            self.local_pos.publish(self.pose)
            self.rate.sleep()
            timeout -= 1 
            if timeout < 0:
                self._reset()
                timeout = 150

        self.started = True
        print self.started
        print "Initialized"


    def _launch_apm(self):
        sim_vehicle_sh = str(os.environ["ARDUPILOT_PATH"]) + "/Tools/autotest/sim_vehicle.sh"
        subprocess.Popen(["xterm","-e",sim_vehicle_sh,"-j4","-f","Gazebo","-v","ArduCopter"])

    def _pause(self, msg):
        programPause = raw_input(str(msg))

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "mpsl.launch")    

        self.targetx = randint(-10,10)
        self.targety = 0
        self.max_distance = 1.6

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pos_cb)
        rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, callback=self.vel_cb)

        rospy.Subscriber('/mavros/state',State,callback=self.state_cb)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
	
	self.model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

	self.local_pos = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=10)

	self.local_vel = rospy.Publisher('mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)

        self.mode_proxy = rospy.ServiceProxy('mavros/set_mode', SetMode)

        self.arm_proxy = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        
        self.takeoff_proxy = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
        self.bridge = CvBridge()

        rospy.Subscriber('multisense_sl/camera/left/image_raw',Image,self.callbackleft)
        rospy.Subscriber('multisense_sl/camera/right/image_raw',Image,self.callbackright)
        rospy.Subscriber('camera/depth/image_raw',Image,self.callbackdepth)

        self.state = ""
        self.twist = TwistStamped()

        self.x_vel = 0
        self.y_vel = 0
	
	self.successes = 0


        self.stereo = cv2.StereoBM_create(16,50)
        self.depth_image = False
	
        self._seed()

        self.cur_pose = PoseStamped()
	self.cur_vel = TwistStamped()

        self.pose = PoseStamped()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 2

        self.hold_state = PoseStamped()
        self.hold_state.pose.position.x = 0
        self.hold_state.pose.position.y = 0

        self.started = False
        self.rate = rospy.Rate(10)

        self.pause_sim = False
        self.nowait = True
        self.new_pose = False
        self.steps = 0
        self.max_episode_length = 200
	self.old_reward = -10


    def pos_cb(self,msg):
        self.cur_pose = msg
        self.new_pose = True

    def vel_cb(self,msg):
        self.cur_vel = msg

    def state_cb(self,msg):
        self.state = msg

    def start(self):
	counter = 15
	while counter != 0:
	    counter = counter -1
 	    time.sleep(1)
        
        vController = VelocityController()
        vController.setTarget(self.pose.pose)

        self.get_data()
        self._takeoff()
        self.nowait = False

        print "Main Running"
        while not rospy.is_shutdown():
            des_vel = vController.update(self.cur_pose,self.x_vel,self.y_vel,self.hold_state)
            self.local_vel.publish(des_vel)
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

    def callbackleft(self,data):
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(data,"mono8")
        except CvBridgeError as e:
            print(e)

    def callbackright(self,data):
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(data,"mono8")
        except CvBridgeError as e:
            print(e)

    def callbackdepth(self,data):
        #print(self.depth_image.size)
        try:
            img = self.bridge.imgmsg_to_cv2(data,"passthrough")
            self.depth_image = cv2.resize(img,(0,0),fx=0.5,fy=0.5)
        except CvBridgeError, e:
            print e

        self.depth_image = np.array(self.depth_image, dtype=np.uint8)
        cv2.normalize(self.depth_image,self.depth_image,1,255,cv2.NORM_MINMAX)
        #self.depth_image = self.depth_image * 50

    def observe(self):
        return self.depth_image

    def observe_test(self):
        return [self.cur_pose.pose.position.x,self.targetx]
    
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

    def reset_model(self):
	modelstate = ModelState()
	modelstate.model_name = "f450"
	modelstate.pose.position.x = 0
	modelstate.pose.position.y = 0
	modelstate.pose.position.z = 2
	self.model_state(modelstate)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def detect_crash(self):
        if self.cur_pose.pose.position.z < 0.3:
            return True
        return False

    def detect_done(self,reward):
        done = False
        if self.detect_crash():
	    print "CRASH"
            done = True
            self.steps = 0
	    self.hard_reset()
	    self.old_reward = -10
        if reward == 10:
	    print "GOAL"
            done = True
            self.steps = 0
	    self.successes += 1
        if self.steps >= self.max_episode_length:
	    print "MAXOUT"
            done = True
            self.steps = 0
	    self.old_reward = -10
	    self.reset_model()
	    reward = reward -2
        return done,reward

    def _step(self, action):
        self.steps += 1
        self.pause_sim = 0 
        if action == 0: #HOLD
            self.x_vel = 0
        elif action == 1: #RIGHT
            self.x_vel = -1
        elif action == 2: #LEFT
            self.x_vel = 1

        self.rate.sleep()
	#while abs(self.cur_vel.twist.linear.x - self.x_vel) > 0.1:
	#    self.rate.sleep()
        observation = self.observe_test()
        reward = self.get_reward(action)

        done,reward = self.detect_done(reward) 

        if done:
            self.targetx = randint(-10,10)
            print "TARGET: ",self.targetx,self.targety
        self.pause_sim = 1
        return observation, reward,done,{}

    def sigmoid(self,x):
        return (1 / (1 + math.exp(-x)))*2

    def get_reward(self,action):
	raw = .2/abs(self.targetx-self.cur_pose.pose.position.x)
	direction = self.targetx-self.cur_pose.pose.position.x
	if raw >=1:
	    return 10
	else:
	    if action ==1 and direction < 0 or action==2 and direction > 0:
		return -0.001
	    else:
	        return -.01

    def collect_reward(self):
	#Returns a -1 if moving futher away and a +1 if moving closer.It will return 10 if it hits goal
	#Returns reward relative to how close you moved to the goal in the last time step
	cur_reward = self.get_state_reward()
	if cur_reward == 10:
	    self.old_reward = -10
	    return 10
        else:
	    diff = self.old_reward - cur_reward
	    self.old_reward = cur_reward
	    return diff* -1

    def get_state_reward(self):
	raw = .2/abs(self.targetx-self.cur_pose.pose.position.x)
	if raw >=1:
	    return 10
	else:
	    return -.01


    def _killall(self, process_name):
        pids = subprocess.check_output(["pidof",process_name]).split()
        for pid in pids:
            os.system("kill -9 "+str(pid))

    def _reset(self):
        return self.observe_test()

    def hard_reset(self):
        # Resets the state of the environment and returns an initial observation.
        print "resetting"
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_proxy()
        except rospy.ServiceException, e:
            print ("/gazebo/reset_world service call failed")

        self.started = False
        self.new_pose = False
        err = 10
        while not self.new_pose:
            self.rate.sleep()

        while not self.started:
            err = abs(self.pose.pose.position.z - self.cur_pose.pose.position.z)
            if err <.3:
                self.started = True
            self.rate.sleep()
        print "hard world reset"

        return self.observe_test()
