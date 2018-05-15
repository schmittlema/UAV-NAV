import gym
import numpy as np
import os
import rospy
import roslaunch
import subprocess
import time
import math
import sensor_msgs.point_cloud2 as pc2
import pcl
from tf.transformations import euler_from_quaternion

from gym import utils, spaces
import gazebo_env
from gym.utils import seeding

from mavros_msgs.msg import OverrideRCIn, PositionTarget
from sensor_msgs.msg import LaserScan, NavSatFix, Image, PointCloud2, Imu
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
kd_tree = 0
cur_imu = 0

def pos_cb(msg):
    global cur_pose
    cur_pose = msg

def imu_cb(msg):
    global cur_imu
    cur_imu = msg
#Testing backup controller1
def stereo_cb(msg):
    global kd_tree
    points = pc2.read_points(msg,skip_nans=True,field_names=("x","y","z"))
    p_array = []
    for p in points:
        p_array.append(p)

    pointcloud = pcl.PointCloud()
    pointcloud.from_list(p_array)
    if pointcloud.size > 0:
        kd_tree = pcl.KdTreeFLANN(pointcloud)

def orient_point(point):
    orientation = cur_imu.orientation 
    roll,pitch,yaw = euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])

def check_nearest_neighbor(radius,point):
    pc_point = pcl.PointCloud()
    #point = orient_point(point)
    point = [point[0],point[2],point[1]]
    pc_point.from_list([point])
    nearest =  math.sqrt(kd_tree.nearest_k_search_for_cloud(pc_point,1)[1][0])
    print nearest

#Setup
launchfile = "dmpsl.launch"
subprocess.Popen("roscore")
print ("Roscore launched!")

# Launch the simulation with the given launchfile name
rospy.init_node('gym', anonymous=True)

fullpath = os.path.join(os.path.dirname(__file__),"launch", launchfile)

subprocess.Popen(["roslaunch",fullpath])
print ("Gazebo launched!")

local_pos = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=10)

rospy.Subscriber('/mavros/imu/data', Imu, callback=imu_cb)

#raw_pos = rospy.Publisher('mavros/setpoint_raw/local',PositionTarget, queue_size=10)

mode_proxy = rospy.ServiceProxy('mavros/set_mode', SetMode)

arm_proxy = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

takeoff_proxy = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)

vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=pos_cb)

rospy.Subscriber('/camera/depth/points', PointCloud2, callback=stereo_cb)

start_pos = PoseStamped()
start_pos.pose.position.x = 0
start_pos.pose.position.y = -2
start_pos.pose.position.z = 5

target = Pose()
target.position.x = 0
target.position.y = 2
target.position.z = 5


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
start_pos.pose.position.x = 100
rate = rospy.Rate(10)
print "Main Running"
while not rospy.is_shutdown():
    #des_vel = vController.update(cur_pose,x_vel,y_vel)
    #vel_pub.publish(des_vel)
    position = [cur_pose.pose.position.x,cur_pose.pose.position.y,0]
    position = [0,0,0]
    #check_nearest_neighbor(2,position)
    local_pos.publish(start_pos)
    rate.sleep()
