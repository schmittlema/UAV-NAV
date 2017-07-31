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
sample_size = 100
primitives = ["forward","left","hard_left","right","hard_right"]
primitive = primitives[0]


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

mode_proxy = rospy.ServiceProxy('mavros/set_mode', SetMode)

arm_proxy = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=pos_cb)

start_pos = PoseStamped()
start_pos.pose.position.x = 0
start_pos.pose.position.y = 0
start_pos.pose.position.z = 2

target = PoseStamped()
target.pose.position.x = .75
target.pose.position.y = 0
target.pose.position.z = 2

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

def at_target(cur,target):
   return (abs(cur.pose.position.x - target.pose.position.x) < .2) and (abs(cur.pose.position.y - target.pose.position.y) < .2) and (abs(cur.pose.position.z - target.pose.position.z) <.2) 

def removeOutliers(x, outlierConstant):
    a = np.array(x[:,0])
    b = np.array(x[:,1])
    upper_quartilea= np.percentile(a, 75)
    lower_quartilea= np.percentile(a, 25)

    upper_quartileb= np.percentile(b, 75)
    lower_quartileb= np.percentile(b, 25)
    IQRa = (upper_quartilea- lower_quartilea) * outlierConstant
    IQRb = (upper_quartileb- lower_quartileb) * outlierConstant
    quartileSeta = (lower_quartilea - IQRa, upper_quartilea + IQRa)
    quartileSetb = (lower_quartileb - IQRb, upper_quartileb + IQRb)
    resultList = []
    for y in a.tolist():
	z = b.tolist()[a.tolist().index(y)]
        if y >= quartileSeta[0] and y <= quartileSeta[1] and z >= quartileSetb[0] and z <= quartileSetb[1]:
            resultList.append([y,z])
    return resultList

def return_array(dictionary):
    result = {}
    for x in dictionary:
	if len(dictionary[x]) >= .95 * sample_size:
	    result[x] = np.mean(removeOutliers(np.array(dictionary[x]),1.5),axis=0)
    print primitive
    print result

def modify_target(trial):
    temp_target = PoseStamped()
    temp_target.pose.position.z = 2
    if trial == 0:
	temp_target.pose.position.x = cur_pose.pose.position.x + .75
	temp_target.pose.position.y = 0
    if trial == 1:
	temp_target.pose.position.x = cur_pose.pose.position.x + 0.5
	temp_target.pose.position.y = cur_pose.pose.position.y + 0.5
    if trial == 2:
	temp_target.pose.position.x = cur_pose.pose.position.x + .15
	temp_target.pose.position.y = cur_pose.pose.position.y + .75
    if trial == 3:
	temp_target.pose.position.x = cur_pose.pose.position.x + 0.5
	temp_target.pose.position.y = cur_pose.pose.position.y - 0.5
    if trial == 4:
	temp_target.pose.position.x = cur_pose.pose.position.x + 0.15
	temp_target.pose.position.y = cur_pose.pose.position.y - 0.75
    return temp_target


position_array = {}

#Main method
rate = rospy.Rate(10)
print "Main Running"
count = 0
resolution = 0.05
hz_pos = cur_pose
hz=0
trial = 0
while not rospy.is_shutdown():
    local_pos.publish(target)

    if abs(hz_pos.pose.position.y - cur_pose.pose.position.y) >= resolution or abs(hz_pos.pose.position.x - cur_pose.pose.position.x) >= resolution:
	if hz in position_array:
	    position_array[hz].append([cur_pose.pose.position.x-start_pos.pose.position.x,cur_pose.pose.position.y-start_pos.pose.position.y])
	else:
	    position_array[hz] = []
	    position_array[hz].append([cur_pose.pose.position.x-start_pos.pose.position.x,cur_pose.pose.position.y-start_pos.pose.position.y])

	hz_pos.pose.position.y = cur_pose.pose.position.y
	hz_pos.pose.position.x = cur_pose.pose.position.x
	hz +=1

    if at_target(cur_pose,target):
	target = modify_target(trial)
	start_pos.pose.position.x = cur_pose.pose.position.x
	start_pos.pose.position.y = cur_pose.pose.position.y
	count+=1
	if count ==sample_size:
	    return_array(position_array)
	    count = 0
	    if trial == 4:
		break
	    trial +=1
	    primitive = primitives[trial]
	    print trial
	    target = modify_target(trial)
	    position_array = {}
	    
	hz = 0

    rate.sleep()
