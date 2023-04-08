#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

def talker():
    pub = rospy.Publisher('/robot1/cropped_map', OccupancyGrid, queue_size=1)
    rospy.init_node('fake_map_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    cropped = OccupancyGrid()
    while not rospy.is_shutdown():
        cropped.info.map_load_time = rospy.Time.now()
        cropped.info.resolution = 0.1
        cropped.info.width = 70
        cropped.info.height = 70
        cropped.info.origin.position.x = 7.5
        cropped.info.origin.position.y = 1
        cropped.info.origin.position.z = 0
        cropped.info.origin.orientation.x = 0
        cropped.info.origin.orientation.y = 0
        cropped.info.origin.orientation.z = 0
        cropped.info.origin.orientation.w = 1
        cropped.header.seq = 1
        cropped.header.stamp=rospy.Time.now()
        cropped.header.frame_id = "/map"
        cropped.data = [100] * 3500
        pub.publish(cropped)
        print("cropped sent")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass