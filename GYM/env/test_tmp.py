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


slam = Slam()
rate = rospy.Rate(.5)
print "Main Running"
while not rospy.is_shutdown():
    #slam.map_publisher.publish(slam.map)
    #print slam.check_collision(slam.seperate_dictionary(slam.tlibrary)[0])
    #slam.map_publisher.publish(slam.map)
    print slam.auto_pilot_step(math.pi/2)
    rate.sleep()
