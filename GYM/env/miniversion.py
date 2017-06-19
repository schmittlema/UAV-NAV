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
from sensor_msgs.msg import LaserScan, NavSatFix
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

cur_pose = PoseStamped()
def pos_cb(msg):
    global cur_pose
    cur_pose = msg

#Setup
launchfile = "mpsl.launch"
subprocess.Popen("roscore")
print ("Roscore launched!")

# Launch the simulation with the given launchfile name
rospy.init_node('gym', anonymous=True)

fullpath = os.path.join(os.path.dirname(__file__),"launch", launchfile)

subprocess.Popen(["roslaunch",fullpath])
print ("Gazebo launched!")

gzclient_pid = 0


local_pos = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=10)

#raw_pos = rospy.Publisher('mavros/setpoint_raw/local',PositionTarget, queue_size=10)

mode_proxy = rospy.ServiceProxy('mavros/set_mode', SetMode)

arm_proxy = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

takeoff_proxy = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)

vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=pos_cb)

start_pos = PoseStamped()
start_pos.pose.position.x = 0
start_pos.pose.position.y = 0
start_pos.pose.position.z = 10

target = Pose()
target.position.x = 0
target.position.y = 0
target.position.z = 10

vController = VelocityController()
vController.setTarget(target)

x_vel = -2
y_vel = 0

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
while not rospy.is_shutdown():
    #des_vel = vController.update(cur_pose,x_vel,y_vel)
    #vel_pub.publish(des_vel)
    local_pos.publish(start_pos)
    rate.sleep()
