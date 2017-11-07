import gym
import numpy as np
import os
import rospy
import roslaunch
import subprocess
import time
import math
import random

from gym import utils, spaces
import gazebo_env
from gym.utils import seeding

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
from cv_bridge import CvBridge, CvBridgeError
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
records=[]

def record(past,event_num):
    if running and rospy.Time.now() - past> rospy.Duration.from_sec(.1):
        records.append([event_num,index,[cur_pose.pose.position.x - start_position.pose.position.x,cur_pose.pose.position.y - start_position.pose.position.y],cur_vel.twist.linear.x,cur_imu.linear_acceleration.x])
        
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

#Main method
takeoff()
xacel = 0
y_vel = 2
running = False
xacells = [-5,-2.5,0,2.5,5]
start_position = PoseStamped()
past_time = 0
runs = 0
last_request = rospy.Time.now()
total_runs = 1000
print "Main Running"
while not rospy.is_shutdown() and runs < total_runs:
    yacel = vpid.update(cur_vel.twist.linear.y,y_vel)
    error = abs(cur_vel.twist.linear.y - y_vel)
    record(past_time,runs)
    if error < 0.1 and not running:
        past_time = rospy.Time.now()
        start_position = cur_pose
        index = random.randint(0,4)
        xacel = xacells[index]
        running = True
        last_request = rospy.Time.now()
    if running and rospy.Time.now() - last_request > rospy.Duration.from_sec(1):
        print runs
        running = False
        runs = runs+1
    w,i,j,k,thrust = pid.generate_attitude_thrust(xacel,yacel,0,cur_pose.pose.position.z,cur_vel.twist.linear.z)
    start_pos.pose.orientation.x = i
    start_pos.pose.orientation.y = j 
    start_pos.pose.orientation.z = k 
    start_pos.pose.orientation.w = w
    att_pub.publish(start_pos)
    throttle_pub.publish(thrust)
    rate.sleep()

log = open("log_sample.txt","w")
log.write(str(records))
log.close()
print "Done!"
hard_reset()

