#testscript for slam
from slam import Slam
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
from VelocityController import VelocityController

cur_pose = PoseStamped()
def pos_cb(msg):
    global cur_pose
    cur_pose = msg

#rospy.init_node('tmp_test', anonymous=True)
slam = Slam()
rate = rospy.Rate(10)

vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
mode_proxy = rospy.ServiceProxy('mavros/set_mode', SetMode)
local_pos = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=10)
arm_proxy = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=pos_cb)

target_position = PoseStamped()
target = Pose()
target.position.x = 0
target.position.y = 3.5
target.position.z = 2
target_position.pose = target
target_position.pose.orientation.x = 0
target_position.pose.orientation.y = 0
target_position.pose.orientation.z = 0.707 
target_position.pose.orientation.w = 0.707


vController = VelocityController()
vController.setTarget(target)
x_vel = 0
y_vel = 0

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

for i in range(0,500):
    local_pos.publish(target_position)

#Setup offboard
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
   print "armed"
except rospy.ServiceException, e:
   print ("mavros/set_mode service call failed: %s"%e)

#Main method
print "Main Running"
while not rospy.is_shutdown():
    #des_vel = vController.update(cur_pose,x_vel,y_vel,target_position)
    #vel_pub.publish(des_vel)
    local_pos.publish(target_position)
    print slam.check_collision(slam.seperate_dictionary(slam.tlibrary)[0])
    slam.map_publisher.publish(slam.map)
    rate.sleep()
