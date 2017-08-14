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
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Float64;
from gazebo_msgs.msg import ModelStates,ModelState,ContactsState
from gazebo_msgs.srv import SetModelState

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped,Pose,Vector3,Twist,TwistStamped
from std_srvs.srv import Empty
from VelocityController import VelocityController


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

        self.pose.pose.position.y = 0
        while not rospy.is_shutdown() and not self.at_target(self.cur_pose,self.pose,0.1):
            self.local_pos.publish(self.pose)
            self.rate.sleep()

        self.pose.pose.position.y = 2.5
        while not rospy.is_shutdown() and not self.at_target(self.cur_pose,self.pose,0.1):
            self.local_pos.publish(self.pose)
            self.rate.sleep()


        self.started = True
        print self.started
        print "Initialized"


    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "mpsl.launch")    

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pos_cb)

        rospy.Subscriber('/mavros/state',State,callback=self.state_cb)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
	
	self.model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

	self.local_pos = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=10)

        self.mode_proxy = rospy.ServiceProxy('mavros/set_mode', SetMode)

        self.arm_proxy = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

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

        self.started = False
        self.rate = rospy.Rate(10)

        self.pause_sim = False
        self.nowait = True
        self.steps = 0
        self.direction = 0
        self.target = 0
        self.slam = Slam()
        self.temp_pause = False
        self.primitives = ["forward","left","hard_left","right","hard_right","backward"]
        self.accuracy = 0.2
        self.heading = math.pi/2
        #used to retrace steps if in dead-end
        self.old_moves = [] 
        self.old_move = PoseStamped()
        self.old_move.pose.position.z = 2
        self.old_move.pose.position.x = 0
        self.old_move.pose.position.y = 0
        self.old_move.pose.orientation.x = 0
        self.old_move.pose.orientation.y = 0
        self.old_move.pose.orientation.z = 0.707 
        self.old_move.pose.orientation.w = 0.707
        self.old_moves.append([0,self.old_move])
        #Moves that are retraced from so as not to go back and forth
        self.blacklist = [False,False,False,False,False]
        self.reverse = False

    def pos_cb(self,msg):
        self.cur_pose = msg

    def filter_correct(self):
        self.pose.pose.position.z = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.x = 0
        return self.at_target(self.cur_pose,self.pose,0.2)

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
        self.nowait = False

        print "Main Running"
        while not rospy.is_shutdown():
            if not self.temp_pause:
                self.local_pos.publish(self.pose)
            self.slam.map_publisher.publish(self.slam.map)
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
        #return self.depth_image
        return 1

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

    def reverse_add(self,one,two):
        if not self.reverse:
            return one + two
        else:
            return one - two

    def reverse_subtract(self,one,two):
        if not self.reverse:
            return one - two
        else:
            return one + two

    def modify_target(self,trial):
        target_position = PoseStamped()
        target = Pose()
        target.position.z = 2
        target.orientation.x = 0
        target.orientation.y = 0
        target.orientation.z = 0.707 
        if self.reverse:
            target.orientation.z = -0.707 
        target.orientation.w = 0.707

        if trial == 0:
            target.position.y = self.reverse_add(self.cur_pose.pose.position.y,.75)
            target.position.x = self.cur_pose.pose.position.x
        if trial == 1:
            target.position.y = self.reverse_add(self.cur_pose.pose.position.y,0.5)
            target.position.x = self.reverse_subtract(self.cur_pose.pose.position.x,0.5)
        if trial == 2:
            target.position.y = self.reverse_add(self.cur_pose.pose.position.y,0)
            target.position.x = self.reverse_subtract(self.cur_pose.pose.position.x,.75)
        if trial == 3:
            target.position.y = self.reverse_add(self.cur_pose.pose.position.y,0.5)
            target.position.x = self.reverse_add(self.cur_pose.pose.position.x,0.5)
        if trial == 4:
            target.position.y = self.reverse_add(self.cur_pose.pose.position.y,0)
            target.position.x = self.reverse_add(self.cur_pose.pose.position.x,0.75)

        if trial == -1:
            self.slam.accuracy = 0.1
            move = self.old_moves[-1]
            self.old_moves = self.old_moves[:-1]
            target.position.y = move[1].pose.position.y
            target.position.x = move[1].pose.position.x
            self.blacklist = [False,False,False,False,False]
            self.blacklist[move[0]] = True
            print "blacklisted: " + self.primitives[move[0]]
        else:
            self.slam.accuracy = 0.2
            self.old_move.pose.position.x = cp.deepcopy(self.cur_pose.pose.position.x)
            self.old_move.pose.position.y = cp.deepcopy(self.cur_pose.pose.position.y)
            self.old_moves.append([trial,cp.deepcopy(self.old_move)])

        target_position.pose = target
        return target_position

    def auto_reset(self):
        self.pose.pose.orientation.z =self.pose.pose.orientation.z * -1.0
        self.pose.pose.position.z = 2
        self.heading = self.heading * -1.0
        if self.reverse:
            self.reverse = False
            self.slam.flip = False

            self.pose.pose.position.x = 0
            self.pose.pose.position.y = 0
            while not rospy.is_shutdown() and not self.at_target(self.cur_pose,self.pose,0.1):
                self.rate.sleep()
            self.slam.reset_proxy()

            self.pose.pose.position.x = 0
            self.pose.pose.position.y = 2.5

            while not rospy.is_shutdown() and not self.at_target(self.cur_pose,self.pose,0.2):
                self.rate.sleep()

            self.old_moves = [] 
            self.old_move = PoseStamped()
            self.old_move.pose.position.x = 0
            self.old_move.pose.position.y = 0
            self.old_moves.append([0,self.old_move])
        else:
            self.reverse = True
            self.slam.flip = True


            self.slam.reset_proxy()

            self.pose.pose.position.x = 0
            self.pose.pose.position.y = 41
            while not rospy.is_shutdown() and not self.at_target(self.cur_pose,self.pose,0.1):
                self.rate.sleep()
            self.slam.reset_proxy()

            self.pose.pose.position.x = 0
            self.pose.pose.position.y = 40

            while not rospy.is_shutdown() and not self.at_target(self.cur_pose,self.pose,0.2):
                self.rate.sleep()

            self.old_moves = [] 
            self.old_move = PoseStamped()
            self.old_move.pose.position.z = 2
            self.old_move.pose.position.x = 0
            self.old_move.pose.position.y = 45
            self.old_moves.append([0,self.old_move])


    def detect_done(self,reward):
        done = False
        if self.collision:
	    print "CRASH"
            done = True
            self.steps = 0
	    self.hard_reset()
        if (not self.reverse and self.cur_pose.pose.position.y > 40) or self.cur_pose.pose.position.y < 0:
	    print "GOAL"
            done = True
            self.steps = 0
	    self.successes += 1
            self.auto_reset()
        return done,reward

    def dangerous(self,action):
        if not self.reverse:
            return self.slam.check_collision(self.slam.sep_dict[action]) or self.blacklist[action]
        else:
            return self.slam.check_collision(self.slam.rsep_dict[action]) or self.blacklist[action]

    def _step(self, action):
        self.steps += 1
        #self.pause_sim = 0

        if self.dangerous(action):
            print "autopilot"
            action = self.slam.auto_pilot_step(self.heading,self.blacklist)
        else:
            print "deep learner"

        print self.primitives[action]
        self.pose = self.modify_target(action)

        self.rate.sleep()
        observation = self.observe()
        reward = self.get_reward(action)

        done,reward = self.detect_done(reward) 

        #self.pause_sim = 1
        return observation, reward,done,{}

    def sigmoid(self,x):
        return (1 / (1 + math.exp(-x)))*2

    def get_reward(self,action):
        return 1

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
        self.reset_proxy()
        self.collision = False
        # Resets the state of the environment and returns an initial observation.
        print "resetting"
        self.temp_pause = True
        self.land()
        while not self.filter_correct():
            self.rate.sleep()
        self._takeoff()
        self.slam.reset_proxy()
        self.old_moves = [] 
        self.old_move = PoseStamped()
        self.old_move.pose.position.z = 2
        self.old_move.pose.position.x = 0
        self.old_move.pose.position.y = 0
        self.old_moves.append([0,self.old_move])
        self.temp_pause = False

        print "hard world reset"
        return self.observe()
