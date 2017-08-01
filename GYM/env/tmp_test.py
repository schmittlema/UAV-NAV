#testscript for slam
import rospy
from slam import Slam
import numpy as np

slam = Slam()
rate = rospy.Rate(.25)

while not rospy.is_shutdown():
    slam.map_publisher.publish(slam.map)
    rate.sleep()
