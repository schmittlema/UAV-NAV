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


#self.local_pos = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=10)

raw_pos = rospy.Publisher('mavros/setpoint_raw/local',PositionTarget, queue_size=10)

mode_proxy = rospy.ServiceProxy('mavros/set_mode', SetMode)

arm_proxy = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

takeoff_proxy = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)

#Setting rpos
rpos = PositionTarget()
rpos.type_mask = 4035 #0b0000101111100011
rpos.header.frame_id = "home"
rpos.header.stamp = rospy.Time.now()
rpos.coordinate_frame = 1
rpos.position.z = 5

rpos.position.y = 0
rpos.position.x = 0

rpos.velocity.x = 2
rpos.velocity.y = 0
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
    rpos.header.stamp = rospy.Time.now()
    raw_pos.publish(rpos)

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
rate = rospy.Rate(5)
print "Main Running"
while not rospy.is_shutdown():
    rpos.header.stamp = rospy.Time.now()
    raw_pos.publish(rpos)
    rate.sleep()
