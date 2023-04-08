#!/usr/bin/env python
import message_filters
from obstacle_velocity.msg import obstacles_msgs
import rospy

def callback(robot1, robot2):
  # Solve all of perception here...
  print("xxxxxxxxxx")

rospy.init_node('testing', anonymous=True)
robot1_sub = message_filters.Subscriber('/robot1/speeds', obstacles_msgs)
robot2_sub = message_filters.Subscriber('/robot2/speeds', obstacles_msgs)

ts = message_filters.TimeSynchronizer([robot1_sub, robot2_sub], 10)
ts.registerCallback(callback)
rospy.spin()