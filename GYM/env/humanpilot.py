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
from gazebo_msgs.srv import SetModelState, GetModelState


from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped,Pose,Vector3,Twist,TwistStamped
from std_srvs.srv import Empty

#For Stereo
from sensor_msgs.msg import Image
import cv2
#from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from VelocityController import VelocityController
from attitude_PID import a_PID
from vel_PID import v_PID

cur_pose = PoseStamped()
cur_vel = TwistStamped()
cur_imu = Imu()
state = State() 
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

#Setup
launchfile = "mpsl.launch"
subprocess.Popen("roscore")
print ("Roscore launched!")

# Launch the simulation with the given launchfile name
rospy.init_node('gym', anonymous=True)

fullpath = os.path.join(os.path.dirname(__file__),"launch", launchfile)

subprocess.Popen(["roslaunch",fullpath])
print ("Gazebo launched!")

local_pos = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=10)

rospy.Subscriber('camera/rgb/image_raw', Image, callback=self.callback_observe)

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

start_pos = PoseStamped()
start_pos.pose.position.x = 0
start_pos.pose.position.y = 0
start_pos.pose.position.z = 2

pid = a_PID()
vpid = v_PID()
rate = rospy.Rate(10)
log_in = open("train_input.txt","w")
log_out = open("train_output.txt","w")
observation = 0
pygame.init()
pygame.display.init()
screen = pygame.display.set_mode ( ( 1 , 1 ) )

def record(observation,action):
	#record observation/action in two different files
	log_in.write(str(observation))
	label = [0,0,0,0,0]
	label[action] = 1
	log_out.write(str(label))

def callback_observe(self,msg):
    try:
        raw_image = self.bridge.imgmsg_to_cv2(msg,"passthrough")
        resized = cv2.resize(raw_image,(100,100),interpolation=cv2.INTER_AREA)
    except CvBridgeError, e:
        print e
    observation = resized.flatten()
        
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
    while not wait_for_user:
    	events = pygame.event.get()
    	for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.SPACE:
		    wait_for_user = True
        local_pos.publish(start_pos)
        rate.sleep()


#Main method
takeoff()
xacel = 0
y_vel = 2
running = True 
xacells = [-5,-2.5,0,2.5,5]
start_position = PoseStamped()
print "Main Running"
while not rospy.is_shutdown() and running:
    xacel = -1
    #user input
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.KEYDOWN:
    	    #Stop input
	    if event.key == pygame.SPACE:
		running = False
	    #Piloting
            if event.key == pygame.K_UP:
		xacel = 2
            if event.key == pygame.K_LEFT:
		if xacel == 2:
		    xacel = 1
		else:
		    xacel = 0
            if event.key == pygame.K_RIGHT:
		if xacel == 2:
		    xacel = 3
		else:
		    xacel = 4

    #record
    if observation != 0 and running and xacel != -1:
    	record(observation,xacel)

    yacel = vpid.update(cur_vel.twist.linear.y,y_vel)
    w,i,j,k,thrust = pid.generate_attitude_thrust(xacel,yacel,0,cur_pose.pose.position.z,cur_vel.twist.linear.z)
    start_pos.pose.orientation.x = i
    start_pos.pose.orientation.y = j 
    start_pos.pose.orientation.z = k 
    start_pos.pose.orientation.w = w
    att_pub.publish(start_pos)
    throttle_pub.publish(thrust)
    rate.sleep()

log_in.close()
log_out.close()
print "Done!"
hard_reset()

