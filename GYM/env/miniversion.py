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

from mavros_msgs.msg import OverrideRCIn, PositionTarget
from sensor_msgs.msg import LaserScan, NavSatFix,Imu
from std_msgs.msg import Float64;
from gazebo_msgs.msg import ModelStates

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

cur_pose = PoseStamped()
cur_vel = TwistStamped()
cur_imu = Imu()
def pos_cb(msg):
    global cur_pose
    cur_pose = msg

def vel_cb(msg):
    global cur_vel
    cur_vel = msg

def imu_cb(msg):
    global cur_imu
    cur_imu = msg

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

att_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude',PoseStamped,queue_size=10)
throttle_pub = rospy.Publisher('/mavros/setpoint_attitude/att_throttle',Float64,queue_size=10)

start_pos = PoseStamped()
start_pos.pose.position.x = 0
start_pos.pose.position.y = 0
start_pos.pose.position.z = 2

pid = a_PID()

#Setting rpos
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

#Main method
rate = rospy.Rate(10)
print "Main Running"
direction = 5
last_request = rospy.Time.now()
while not rospy.is_shutdown():
    if rospy.Time.now() - last_request > rospy.Duration.from_sec(3):
        #direction = direction * -1
        last_request = rospy.Time.now()
    w,i,j,k,thrust = pid.generate_attitude_thrust(0,direction,0,cur_pose.pose.position.z,cur_vel.twist.linear.z)
    start_pos.pose.orientation.x = i
    start_pos.pose.orientation.y = j 
    start_pos.pose.orientation.z = k 
    start_pos.pose.orientation.w = w
    att_pub.publish(start_pos)
    throttle_pub.publish(thrust)
    print cur_imu
    print cur_vel
    rate.sleep()
