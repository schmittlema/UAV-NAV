import gym
import numpy as np
import os
import rospy
import roslaunch
import subprocess
import time
import math

from gym import utils, spaces
import gazebo_env
from gym.utils import seeding

from mavros_msgs.msg import OverrideRCIn, PositionTarget,State
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Float64;
from gazebo_msgs.msg import ModelStates

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
        while self.state != "OFFBOARD":
            if rospy.Time.now() - last_request > rospy.Duration.from_sec(3):
                # Set OFFBOARD mode
                rospy.wait_for_service('mavros/set_mode')
                
                #Must be sending points before connecting to offboard
                for i in range(0,100):
                    self.local_pos.publish(self.pose)
                try:
                    success = self.mode_proxy(0,'OFFBOARD')
                    print success
                except rospy.ServiceException, e:
                    print ("mavros/set_mode service call failed: %s"%e)
                print "offboard enabled"

                print "arming"
                rospy.wait_for_service('mavros/cmd/arming')
                try:
                   success = self.arm_proxy(True)
                   print success
                except rospy.ServiceException, e:
                   print ("mavros/set_mode service call failed: %s"%e)
                   print "armed"
                last_request = rospy.Time.now()

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

        RED = '\033[91m'
        BOLD = '\033[1m'
        ENDC = '\033[0m'        
        LINE = "%s%s##############################################################################%s" % (RED, BOLD, ENDC)
        msg = "\n%s\n" % (LINE)
        msg += "%sLoad Erle-Copter parameters in MavProxy console (sim_vehicle.sh):%s\n\n" % (BOLD, ENDC)
        msg += "MAV> param load %s\n\n" % (str(os.environ["ERLE_COPTER_PARAM_PATH"]))
        msg += "%sThen, press <Enter> here to launch Gazebo...%s\n\n%s" % (BOLD, ENDC,  LINE)
        self._pause(msg)

        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "mpsl.launch")    

        self.action_space = spaces.Discrete(4) # F, L, R, B
        self.reward_range = (-np.inf, np.inf)

        self.initial_latitude = None
        self.initial_longitude = None

        self.current_latitude = None
        self.current_longitude = None

        self.diff_latitude = None
        self.diff_longitude = None

        self.max_distance = 1.6

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pos_cb)

        rospy.Subscriber('/mavros/state',State,callback=self.state_cb)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

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


        self.stereo = cv2.StereoBM_create(16,50)
        self.depth_image = False
	
        self._seed()

        self.cur_pose = PoseStamped()

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


    def pos_cb(self,msg):
        self.cur_pose = msg

    def state_cb(self,msg):
        self.state = msg.mode

    def start(self):
	counter = 15
	while counter != 0:
	    counter = counter -1
 	    time.sleep(1)
        
        vController = VelocityController()
        vController.setTarget(self.pose.pose)

        self.get_data()
        self._takeoff()

        print "Main Running"
        while not rospy.is_shutdown():
            des_vel = vController.update(self.cur_pose,self.x_vel,self.y_vel,self.hold_state)
            self.local_vel.publish(des_vel)
            self.rate.sleep()
            self.pauser()

    def pauser(self):
        if self.pause_sim:
            rospy.wait_for_service('/gazebo/pause_physics')
            try:
                self.pause()
            except rospy.ServiceException, e:
                print ("/gazebo/pause_physics service call failed")
            while self.pause_sim:
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
    
    def wait_until_start(self):
        while True:
            if self.started:
                break
            time.sleep(1)
        return

    def get_data(self):
        print "Waiting for mavros..."
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/mavros/global_position/rel_alt', Float64, timeout=5)
            except:
                pass

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _state(self, action):
        return discretized_ranges, done

    def detect_crash(self):
        if self.cur_pose.pose.position.z < 0.5:
            return True
        return False

    def _step(self, action):
        self.pause_sim = False
        if action == 0: #HOLD
            if self.x_vel != 0:
                self.hold_state = self.cur_pose
            self.x_vel = 0

        elif action == 2: #LEFT
            self.x_vel = -2
        elif action == 1: #RIGHT
            self.x_vel = 2


        time.sleep(5)
        observation = self.observe()
        done = self.detect_crash()
        reward = self.get_reward()


        self.pause_sim = True
        return observation, reward,done,{}

    def get_reward(self):
        return 1


    def _killall(self, process_name):
        pids = subprocess.check_output(["pidof",process_name]).split()
        for pid in pids:
            os.system("kill -9 "+str(pid))

    def _reset(self):
        # Resets the state of the environment and returns an initial observation.
        print "resetting"
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_proxy()
        except rospy.ServiceException, e:
            print ("/gazebo/reset_world service call failed")
        print "world reset"

        return self.observe()
