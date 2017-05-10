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

    def _takeoff(self,pose):
        print "Waiting for mavros..."
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/mavros/global_position/rel_alt', Float64, timeout=5)
            except:
                pass
        
	print "Got Mavros"
        # Set OFFBOARD mode
        rospy.wait_for_service('mavros/set_mode')
        
        #Must be sending points before connecting to offboard
        for i in range(0,100):
            self.local_pos.publish(pose)
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

        for i in range(0,500):
            self.local_pos.publish(pose)


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
        #self.observation_space = spaces.Box(low=0, high=20) #laser values
        self.reward_range = (-np.inf, np.inf)

        self.initial_latitude = None
        self.initial_longitude = None

        self.current_latitude = None
        self.current_longitude = None

        self.diff_latitude = None
        self.diff_longitude = None

        self.max_distance = 1.6

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pos_cb)

        #self.pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        #self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        #self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

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


        self.twist = TwistStamped()

        self.reset = False
        self.x_vel = 1
        self.y_vel = 0


        self.stereo = cv2.StereoBM_create(16,50)
        self.depth_image = False
	
        self._seed()

        self.cur_pose = PoseStamped()

    def pos_cb(self,msg):
        self.cur_pose = msg

    def start(self):
	counter = 15
	while counter != 0:
	    counter = counter -1
 	    time.sleep(1)
        
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2

        vController = VelocityController()
        vController.setTarget(pose.pose)

        self._takeoff(pose)

        rate = rospy.Rate(10)
        print "Main Running"
        while not rospy.is_shutdown():
            des_vel = vController.update(self.cur_pose,self.x_vel,self.y_vel)
            self.local_vel.publish(des_vel)
            if self.reset:
                pose.pose.position.z = 2
            rate.sleep()

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
            img = self.bridge.imgmsg_to_cv2(data,"32FC1")
        except CvBridgeError, e:
            print e

        #img.astype(np.uint8)

        self.depth_image = np.array(img, dtype=np.float32)
        #print "SIZE"
        #print self.depth_image.shape
        #for c in range(0,np.size(self.depth_image,1)):
        #    for r in range(0,np.size(self.depth_image,0)):
        #        print self.depth_image[c][r]
        #plt.imshow(self.depth_image.squeeze())
        #plt.show()
        #cv2.normalize(depth_array,self.depth_image,0,1,cv2.NORM_MINMAX)

        cv2.imwrite('depth.png',80*self.depth_image)


    def observe(self):
        print "observing"
        #self.disparity = self.stereo.compute(self.left_img,self.right_img)
        #plt.imshow(self.disparity)
        #plt.imshow(self.depth_image)
        #plt.show()
        #cv2.imwrite("depth.png",self.depth_image)
        print "observed"


    def get_data(self):
        while True:
            data = rospy.wait_for_message('/mavros/global_position/rel_alt', Float64, timeout=5)
            print data


    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _state(self, action):
        return discretized_ranges, done

    def _step(self, action):
        if action == 0: #LEFT
            self.twist.twist.linear.z = 500
        elif action == 1: #RIGHT
            msg.channels[0] = 1450 # Roll
            msg.channels[1] = 1500 # Pitch
        elif action == 2: #UP
            msg.channels[0] = 1550 # Roll
            msg.channels[1] = 1500 # Pitch
        elif action == 3: #DOWN
            msg.channels[0] = 1500 # Roll
            msg.channels[1] = 1550 # Pitch
        elif action == 3: #NO_ACTION
            msg.channels[0] = 1500 # Roll
            msg.channels[1] = 1550 # Pitch

        observation = self._get_position()

        reward = self.get_reward()

        return observation, reward, done, {}

    def get_reward(self):
        print "hi"


    def _killall(self, process_name):
        pids = subprocess.check_output(["pidof",process_name]).split()
        for pid in pids:
            os.system("kill -9 "+str(pid))

    def _to_meters(self, n):
        return n * 100000.0

    def _get_position(self):
        #read position data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/mavros/global_position/global', NavSatFix, timeout=5)
            except:
                pass

        self.current_latitude = self._to_meters(data.latitude)
        self.current_longitude = self._to_meters(data.longitude)

        if self.initial_latitude == None and self.initial_longitude == None:
            self.initial_latitude = self.current_latitude
            self.initial_longitude = self.current_longitude
            print "Initial latitude : %f, Initial Longitude : %f" % (self.initial_latitude,self.initial_longitude,)

        print "Current latitude : %f, Current Longitude : %f" % (self.current_latitude,self.current_longitude,)

        self.diff_latitude = self.current_latitude - self.initial_latitude
        self.diff_longitude = self.current_longitude - self.initial_longitude

        print "Diff latitude: %f, Diff Longitude: %f" % (self.diff_latitude,self.diff_longitude,)

        return self.diff_latitude, self.diff_longitude

    def center_distance(self):
        return math.sqrt(self.diff_latitude**2 + self.diff_longitude**2)

    def _reset(self):
        # Resets the state of the environment and returns an initial observation.
        print "resetting"
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_proxy()
        except rospy.ServiceException, e:
            print ("/gazebo/reset_world service call failed")
        print "world reset"

        self.reset = True
        time.sleep(1)
        self.reset = False

        return self._get_position()
