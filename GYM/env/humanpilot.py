import gym
import numpy as np
import os
import rospy
import roslaunch
import subprocess
import time
import math
import random
import pygame
from pygame.locals import *

from mavros_msgs.msg import OverrideRCIn, PositionTarget,State
from sensor_msgs.msg import LaserScan, NavSatFix,Imu
from std_msgs.msg import Float64;
from gazebo_msgs.msg import ModelStates,ModelState,ContactsState
from gazebo_msgs.srv import SetModelState, GetModelState,SpawnModel


from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped,Pose,Vector3,Twist,TwistStamped
from std_srvs.srv import Empty

#For Stereo
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from VelocityController import VelocityController
from attitude_PID import a_PID
from vel_PID import v_PID

np.set_printoptions(threshold=np.nan)
cur_pose = PoseStamped()
cur_vel = TwistStamped()
cur_imu = Imu()
state = State() 
bridge = CvBridge()
def pos_cb(msg):
    global cur_pose
    cur_pose = msg

def vel_cb(msg):
    global cur_vel
    cur_vel = msg

def imu_cb(msg):
    global cur_imu
    cur_imu = msg

def state_cb(msg):
    state = msg

def callback_observe(msg):
    global observation
    try:
        raw_image = bridge.imgmsg_to_cv2(msg,"passthrough")
        resized = cv2.resize(raw_image,(100,100),interpolation=cv2.INTER_AREA)
    except CvBridgeError, e:
        print e
    observation = np.array(resized.flatten())

#Setup
launchfile = "plain.launch"
subprocess.Popen("roscore")
print ("Roscore launched!")

# Launch the simulation with the given launchfile name
rospy.init_node('gym', anonymous=True)

fullpath = os.path.join(os.path.dirname(__file__),"launch", launchfile)

subprocess.Popen(["roslaunch",fullpath])
print ("Gazebo launched!")

local_pos = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=10)

rospy.Subscriber('stereo/left/image_raw', Image, callback=callback_observe)

#raw_pos = rospy.Publisher('mavros/setpoint_raw/local',PositionTarget, queue_size=10)

mode_proxy = rospy.ServiceProxy('mavros/set_mode', SetMode)

arm_proxy = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=pos_cb)

imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, callback=imu_cb)

vel_sub = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, callback=vel_cb)

model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

model_position = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

rospy.Subscriber('/mavros/state',State,callback=state_cb)

att_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude',PoseStamped,queue_size=10)
throttle_pub = rospy.Publisher('/mavros/setpoint_attitude/att_throttle',Float64,queue_size=10)

spawn_proxy = rospy.ServiceProxy('gazebo/spawn_sdf_model',SpawnModel)

start_pos = PoseStamped()
start_pos.pose.position.x = 0
start_pos.pose.position.y = 0
start_pos.pose.position.z = 2

pid = a_PID()
vpid = v_PID()
rate = rospy.Rate(10)
log_in = open("test_input.txt","a")
log_out = open("test_output.txt","a")
observation = 0
num_recorded = 0
pygame.display.init()
screen = pygame.display.set_mode ( ( 100 , 100 ) )
last_draw = -10
tree_bank = {}
tree_id = 0
density = 12
f = open('/home/ubuntu/UAV-NAV/model-worlds/tree.sdf','r')
sdff = f.read()

def make_new_trees():
    global last_draw
    global tree_bank 
    global tree_id 
    base = 15
    if(cur_pose.pose.position.y - last_draw >= base):
        last_draw = cur_pose.pose.position.y
        start_x = (cur_pose.pose.position.x - 35) + random.randint(0,5)
        cur_x = cur_pose.pose.position.x
        y = int(base * round((cur_pose.pose.position.y + 20)/base))
        gap = 5
        end = start_x + 80
        if y in tree_bank:
            for entry in tree_bank[y]:
                #if cur_x < entry[1] and cur_x > entry[0]:
                if start_x > entry[0]:
                    start_x = entry[1] + gap
                if end < entry[1]:
                    end = entry[0] - 2
            tree_bank[y].append([start_x,end])
        else:
            tree_bank[y] = []
            tree_bank[y].append([start_x,end])
        x = start_x
        while(x < end):
            target = Pose()
            target.position.x = x 
            target.position.y = y + random.randint(-4,4)
            target.position.z = 5.1 
            spawn_proxy(str(tree_id),sdff,'trees',target,'world')
            #self.tree_bank.put(self.tree_id)
            tree_id+=1
            x += density + random.randint(0,3)
            rate.sleep()


def record(observation,action):
    global num_recorded 
    #record observation/action in two different files
    if action != 2:
        log_in.write(str(list(observation))+'\n')
        label = [0,0,0,0,0]
        label[action] = 1
        log_out.write(str(label)+"\n")
        num_recorded+=1

def land():
    global state
    try:
        success = mode_proxy(0,'AUTO.LAND')
        print "landing"
    except rospy.ServiceException, e:
        print ("mavros/set_mode service call failed: %s"%e)

def reset_quad():
    modelstate = ModelState()
    modelstate.model_name = "f450-stereo"
    modelstate.pose.position.z = 0.1
    modelstate.pose.position.x = 0
    modelstate.pose.position.y = 0 
    #modelstate.pose.orientation.z = 0.707
    #modelstate.pose.orientation.w = 0.707
    model_state(modelstate)

def at_target(cur,target,accuracy):
    return (abs(cur.pose.position.x - target.pose.position.x) < accuracy) and (abs(cur.pose.position.y - target.pose.position.y) < accuracy) and (abs(cur.pose.position.z - target.pose.position.z) <accuracy)

def filter_correct():
    pose = model_position('f450-stereo','world')
    return at_target(cur_pose,pose,2)

def hard_reset():
    # Resets the state of the environment and returns an initial observation.
    print "resetting"
    land()
    while cur_pose.pose.position.z > 0.2:
        rate.sleep()
    reset_quad()
    while not filter_correct():
        rate.sleep()

    if state.armed:
        print "disarming"
        try:
           success = arm_proxy(False)
           print success
           print "disarmed"
        except rospy.ServiceException, e:
           print ("mavros/set_mode service call failed: %s"%e)

    start_pos.pose.position.x = 0 
    start_pos.pose.position.y = 0
    start_pos.pose.position.z = 2 
    #for random walking
    #takeoff()
    print "hard world reset"


def takeoff():
    print "Waiting for mavros..."
    data = None
    while data is None:
        try:
            data = rospy.wait_for_message('/mavros/global_position/rel_alt', Float64, timeout=5)
        except:
            pass
    print "wait for service"
    rospy.wait_for_service('mavros/set_mode')
    print "got service"

    for i in range(0,100):
        local_pos.publish(start_pos)

    #setup offboard
    try:
        success = mode_proxy(0,'OFFBOARD')
        print success
    except rospy.ServiceException, e:
        print ("mavros/set_mode service call failed: %s"%e)


    #Arm
    print "arming"
    rospy.wait_for_service('mavros/cmd/arming')
    try:
       success = arm_proxy(True)
       print success
    except rospy.ServiceException, e:
       print ("mavros/set_mode service call failed: %s"%e)
       print "armed"

    while cur_pose.pose.position.z < start_pos.pose.position.z-0.2:
        local_pos.publish(start_pos)
        rate.sleep()

    wait_for_user = False
    print "Waiting for user input... "
    while not wait_for_user:
    	events = pygame.event.get()
    	for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
		    wait_for_user = True
        pygame.display.update()
        local_pos.publish(start_pos)
        rate.sleep()


#Main method
takeoff()
xacel = 2
y_vel = 2
running = True 
xacells = [-5,-2.5,0,2.5,5]
start_position = PoseStamped()
print "Main Running"
while not rospy.is_shutdown() and running:
    #Make trees
    make_new_trees()
    #user input
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.KEYDOWN:
            keys = pygame.key.get_pressed()

    	    #Stop input
	    if keys[pygame.K_SPACE]:
                print "Shutting Down..."
		running = False
	    #Piloting
            if keys[pygame.K_UP]:
                print "forward"
		xacel = 2
            if keys[pygame.K_RIGHT]:
                print "right"
		if keys[pygame.K_UP]:
		    xacel = 1
		else:
		    xacel = 0
            if keys[pygame.K_LEFT]:
                print "left"
		if keys[pygame.K_UP]:
		    xacel = 3
		else:
		    xacel = 4
    #record
    if observation is not int and running:
    	record(observation,xacel)

    if num_recorded % 100 == 0 :
        print num_recorded,cur_pose.pose.position.y,cur_pose.pose.position.x

    if num_recorded > 50000:
        print "Shutting Down..."
	running = False

    yacel = vpid.update(cur_vel.twist.linear.y,y_vel)
    w,i,j,k,thrust = pid.generate_attitude_thrust(xacells[xacel],yacel,0,2,cur_pose.pose.position.z,cur_vel.twist.linear.z)
    start_pos.pose.orientation.x = i
    start_pos.pose.orientation.y = j 
    start_pos.pose.orientation.z = k 
    start_pos.pose.orientation.w = w

    pygame.display.update()
    att_pub.publish(start_pos)
    throttle_pub.publish(thrust)
    rate.sleep()

log_in.close()
log_out.close()
print "Done!"
hard_reset()

