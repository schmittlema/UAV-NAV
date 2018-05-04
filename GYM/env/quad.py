import gym
import pcl
import numpy as np
import os
import rospy
import roslaunch
import subprocess
import time
import math
from random import randint
import random
from slam import Slam
import queue
import copy as cp
import ast

from gym import utils, spaces
import gazebo_env
from gym.utils import seeding

from mavros_msgs.msg import OverrideRCIn, PositionTarget,State
from sensor_msgs.msg import LaserScan, NavSatFix,Image,PointCloud2,Imu
from std_msgs.msg import Float64;
from gazebo_msgs.msg import ModelStates,ModelState,ContactsState
from gazebo_msgs.srv import SetModelState, GetModelState, SpawnModel,DeleteModel

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
from tf.transformations import euler_from_quaternion


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
        last_request = rospy.Time.now()
        while not rospy.is_shutdown() and not self.at_target(self.cur_pose,self.pose,0.3):
            if self.collision:
                self.hard_reset()
                return
            if rospy.Time.now() - last_request > rospy.Duration.from_sec(30):
                self.hard_reset()
                return
            self.local_pos.publish(self.pose)
            self.rate.sleep()

            #self.check_nearest_neighbor(self.radius,[0,1,0])

        self.collision = False
        self.started = True
        print self.started
        print "Initialized"


    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "plain.launch")    

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pos_cb)
        rospy.Subscriber('/camera/rgb/image_raw', Image, callback=self.callback_observe)
        rospy.Subscriber('/camera/depth/image_raw', Image, callback=self.callback_depth)
        self.image_pub = rospy.Publisher("camera_edited",Image)
        #rospy.Subscriber('/camera/depth/points', PointCloud2, callback=self.stereo_cb)

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
        #Autoworld
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        rospy.wait_for_service('gazebo/delete_model')
        self.spawn_proxy = rospy.ServiceProxy('gazebo/spawn_sdf_model',SpawnModel)
        self.delete_proxy = rospy.ServiceProxy('gazebo/delete_model',DeleteModel)


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
        self.pose.pose.position.z = 5
        #self.pose.pose.orientation.x = 0
        #self.pose.pose.orientation.y = 0
        #self.pose.pose.orientation.z = 0.707 
        #self.pose.pose.orientation.w = 0.707
        self.network_stepped = True

        self.started = False
        self.rate = rospy.Rate(10)
        self.collisions = 0
        self.auto_steps = 0
        self.network_steps = 0

        self.pause_sim = False
        self.steps = 0
        self.episode_distance = 0
        self.target = 0
        self.temp_pause = False
        self.accuracy = 0.2
        self.stop_count = 0

        self.action = 0
        self.takeover = False
        self.depth_cam = False
        self.mono_image = np.zeros(30000)
        self.bridge = CvBridge()

        #radius of drone + extra space
        self.radius = 1.5

        #Percent of danger allowed
        self.dsafe = 2.5
        self.danger = False
        self.num_interventions = 0
        self.safety_metric = True

        #attitude related variables
        self.cur_vel = TwistStamped()
        self.pid = a_PID()
        self.vpid = v_PID()
        self.y_vel = 2
        self.x_accel = 0
        self.actions = [-5.0,-2.5,0,2.5,5.0]
        self.cur_imu = Imu()
    
        #PointCloud
        self.kd_tree = 0
        self.depth = []
        self.resized = []


        self.last_spot = -20

        #Auto trees
        self.density = 12
        self.last_draw = -20
        f = open('model-worlds/tree.sdf','r')
        self.sdff = f.read()
        self.tree_id = 0
        self.tree_bank = {} 

        #DAgger
        self.d_star = open("/home/ubuntu/Training_data/train2_input.txt",'a')
        self.aug_count = 0

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

    def callback_depth(self,msg):
        try:
            raw_image = self.bridge.imgmsg_to_cv2(msg,"passthrough")
            resized = cv2.resize(raw_image,(100,100),interpolation=cv2.INTER_AREA)
        except CvBridgeError, e:
            print e
        self.depth = np.array(resized.flatten())
        self.resized = resized
        #self.depth = depth[~np.isnan(depth)]

    def callback_observe(self,msg):
        try:
            raw_image = self.bridge.imgmsg_to_cv2(msg,"passthrough")
            resized = cv2.resize(raw_image,(100,100),interpolation=cv2.INTER_AREA)
        except CvBridgeError, e:
            print e
        self.mono_image = np.array(resized.flatten())
        #republish edited
        try:
            edited = raw_image
            font = cv2.FONT_HERSHEY_SIMPLEX
            if self.danger:
                cv2.putText(edited,"DANGER!",(10,230),font,0.8,(0,0,255),2,8)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(edited,"bgr8"))
        except CvBridgeError, e:
            print e
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
        pose = self.model_position('f450-depth','world')
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
        target = Pose()
        target.position.x = self.cur_pose.pose.position.x 
        target.position.y = (self.cur_pose.pose.position.y + 4)
        target.position.z = 5.1 
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        #self.spawn_proxy("bob",self.sdff,'trees',target,'world')

        self.danger = False
        print "Main Running"
        while not rospy.is_shutdown():
            if not self.temp_pause:

                #Safety Metric 1
                dan,direct1 = self.dangerous()
                if dan and self.safety_metric:
                    print "DANGER!"
                    if not self.danger:
                        self.num_interventions +=1
                        dan2,direct2 = self.dangerous()
                        dan3,direct3 = self.dangerous()
                        if direct1 + direct2 + direct3 > 1:
                            direct = 0
                            print "right!!!"
                        else:
                            direct = 4
                            print "left!!!"

                        #self.y_vel = -4
                        daccell = self.actions[direct]
                    self.danger = True
                    self.x_accel = daccell 
                else:
                    self.danger = False
                    #self.y_vel = 2

                #Safety Metric 2
                #dan = self.dangerous2()
                #if dan != False:
                #    min_one = max(0,dan - 1)
                #    self.y_vel = (min_one/self.dsafe)*2
                #    self.x_accel = (min_one/self.dsafe)*self.x_accel
                #    print "REDUCED SPEED:",self.y_vel
                #else:
                #    self.y_vel = 2

                #if dan !=False:
                #    if dan < 2:
                #        print "Opposite Accel"
                #        if not danger:
                #            daccell = self.x_accel
                #            danger = True
                #            self.x_accel = -1 * daccell 
                #            if self.x_accel == 0:
                #                min_one = max(0,dan - 1)
                #                self.x_accel = (min_one/self.dsafe)*2
                #    else:
                #        danger = False

                yacel = self.vpid.update(self.cur_vel.twist.linear.y,self.y_vel)
                w,i,j,k,thrust = self.pid.generate_attitude_thrust(self.x_accel,yacel,0,5,self.cur_pose.pose.position.z,self.cur_vel.twist.linear.z)
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
        #return np.zeros(2500)
        return self.mono_image

    def reset_quad(self):
        modelstate = ModelState()
        modelstate.model_name = "f450-depth"
        modelstate.pose.position.z = 0.1
        modelstate.pose.position.x = 0
        modelstate.pose.position.y = -2
        #modelstate.pose.orientation.z = 0.707 
        #modelstate.pose.orientation.w = 0.707 
        self.model_state(modelstate)

    #DEPRECATED Because it never worked
    def move_tree(self,tid,loc):
        pose = self.model_position(str(tid),'world')
        modelstate = ModelState()
        modelstate.model_name = str(tid)
        modelstate.pose.position = loc
        self.model_state(modelstate)
        posea = self.model_position(str(tid),'world')
        print pose,posea
        while(pose.pose.position != loc):
           self.rate.sleep() 

    def wait_until_start(self):
        while True:
            if self.started:
                break
            self.rate.sleep()
        return
    
    def make_new_trees(self):
        base = 15
        if(self.cur_pose.pose.position.y - self.last_draw >= base):
            self.last_draw = self.cur_pose.pose.position.y
            start_x = (self.cur_pose.pose.position.x - 35) + random.randint(0,5)
            cur_x = self.cur_pose.pose.position.x
            y = int(base * round((self.cur_pose.pose.position.y + 20)/base))
            gap = 5
            end = start_x + 80
            if y in self.tree_bank:
                for entry in self.tree_bank[y]:
                    #if cur_x < entry[1] and cur_x > entry[0]:
                    if start_x > entry[0]-gap and start_x < entry[1] + gap:
                        start_x = entry[1] + gap
                    if end < entry[1]+gap and end > entry[0] - gap:
                        end = entry[0] - gap
                self.tree_bank[y].append([start_x,end])
            else:
                self.tree_bank[y] = []
                self.tree_bank[y].append([start_x,end])
            x = start_x
            while(x < end):
                target = Pose()
                target.position.x = x 
                target.position.y = y + random.randint(-4,4)
                target.position.z = 5.1 
                self.spawn_proxy(str(self.tree_id),self.sdff,'trees',target,'world')
                #self.tree_bank.put(self.tree_id)
                self.tree_id+=1
                x += self.density + random.randint(0,3)
                self.rate.sleep()

    def make_new_trees_DEPRECATED(self):
        if(self.cur_pose.pose.position.y - self.last_draw >= 10):
            self.last_draw = self.cur_pose.pose.position.y
            start_x = (self.cur_pose.pose.position.x - 35) + randint(0,5)
            x = start_x
            while(x < start_x + 80):
                target = Pose()
                target.position.x = x 
                target.position.y = (self.cur_pose.pose.position.y + 20) + randint(-4,4)
                target.position.z = 5.1 
                self.spawn_proxy(str(self.tree_id),self.sdff,'trees',target,'world')
                #self.tree_bank.put(self.tree_id)
                self.tree_id+=1
                x += self.density + randint(0,3)
                self.rate.sleep()

    def orient_point(self,point):
        orientation = self.cur_imu.orientation 
        roll,pitch,yaw = euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
        #print pitch,point[1],point[1]*math.cos(pitch)
        point[0] = point[0]*math.cos(roll)
        point[1] = point[1]*math.cos(pitch)
        point[2] = point[2] + ((point[1]*math.sin(pitch))+(point[0]*math.sin(roll)))
        return point

    def check_nearest_neighbor(self,radius,point):
        pc_point = pcl.PointCloud()
        point = self.orient_point(point)
        point = [point[0],point[2],point[1]]
        pc_point.from_list([point])
        nearest =  self.kd_tree.nearest_k_search_for_cloud(pc_point,5)[1][0]
        for kpoint in nearest:
            if kpoint < radius:
                return True
        return False

    def check_nearest_neighbor_pop(self,radius,point):
        pc_point = pcl.PointCloud()
        point = self.orient_point(point)
        point = [point[0],point[2],point[1]]
        pc_point.from_list([point])
        nearest =  self.kd_tree.nearest_k_search_for_cloud(pc_point,10)[1][0]
        population = 0
        for kpoint in nearest:
            if kpoint < radius:
                population +=1
        if population > 1:
            return True
        else:
            return False

    def get_data(self):
        print "Waiting for mavros..."
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/mavros/global_position/rel_alt', Float64, timeout=5)
            except:
                pass

    def dangerous(self):
        depth = self.depth
        if len(depth) > 0:
            min_v = np.nanmin(depth)
            if min_v < self.dsafe:
                min_i =np.nanargmin(depth)
                #min_index = (np.nanargmin(depth))/100
                min_index = min_i%100
                #DEBUGING INFO
                #print min_index,min_v,depth[min_i]
                if min_index < 50:
                    #print "right"
                    #action 0
                    min_i = 1
                else:
                    #print "left"
                    #action 4
                    min_i = 0
                return [True,min_i]
            else:
                return [False,0]
        else:
            return [False,0]

    def dangerous2(self):
        if len(self.depth) > 0:
            min_d = np.amin(self.depth)
            if min_d < self.dsafe + 1:
                return min_d 
        else:
            return False

    def detect_done(self,reward):
        done = False
        if self.collision:
	    print "CRASH"
            done = True
            self.steps = 0
            self.collisions += 1
            self.episode_distance = self.cur_pose.pose.position.y
	    self.hard_reset()
        if self.cur_pose.pose.position.y > 100:
	    print "GOAL"
            done = True
            self.episode_distance = 100
            self.steps = 0
	    self.successes += 1
            self.hard_reset()
        #if self.last_spot > self.cur_pose.pose.position.y:
        #    done = True
        #    print "overworked"
        #    self.hard_reset
        return done,reward

    def _step(self, action):
        self.make_new_trees()
        self.steps += 1

        before = rospy.Time.now()
        self.network_steps += 1
        self.network_stepped = True
        reward = self.get_reward(action)

        self.rate.sleep()
        self.x_accel = self.actions[action]
        #print self.x_accel,self.cur_pose.pose.position.y
        last_request = rospy.Time.now()
        observation = self.observe()

        done,reward = self.detect_done(reward) 
        self.last_spot = self.cur_pose.pose.position.y
        return observation, reward,done,{}

    def augment(self,raw_v):
        if np.count_nonzero(raw_v.clip(min=0)) > 1:
            sort = np.sort(raw_v)
            var = abs(sort[-2] - sort[-1])
        else:
            var = max(raw_v.clip(min=0))

        sig = False
        if len(self.depth) > 0:
            dobst = np.nanmin(self.depth)
            if dobst < self.dsafe + 1:
                sig = True
        
        #record here
        if var<=4.0 or sig:
            self.d_star.write(str(list(self.mono_image))+'\n')
            self.aug_count +=1
            print "RECORDED!",self.aug_count
            return 1
        else:
            return 0


    def sigmoid(self,x):
        return (1 / (1 + math.exp(-x)))*2

    def get_reward(self,action):
        if action < 2 or action > 2:
            return 1
        else:
            return 3

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
        #To be reinserted once delete bug is figured out
        #if self.tree_id > 1:
        #    for x in range(0,self.tree_id):
        #        self.delete_proxy(str(x))
        #    self.tree_id = 0
        #    self.last_draw = -20
        #rospy.wait_for_service('gazebo/spawn_sdf_model')
        #self.make_new_trees()
        self.last_draw = -10
        self.action = 0
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = -2
        self.pose.pose.position.z = 5
        #self.pose.pose.orientation.x = 0
        #self.pose.pose.orientation.y = 0
        #self.pose.pose.orientation.z = 0.707 
        #self.pose.pose.orientation.w = 0.707
        self._takeoff()
        self.temp_pause = False

        print "hard world reset"
        print self.action
        return self.observe()
