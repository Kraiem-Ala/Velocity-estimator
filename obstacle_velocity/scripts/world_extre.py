#!/usr/bin/env python
from nav_msgs.msg import OccupancyGrid
import rospy 

def call_back(msg) :
	x , y = maptoworld(0,0,msg.info.origin.position.x,msg.info.origin.position.y,msg.info.resolution);
	print("0,0 = ( ", x ," , ",y)
	x , y = maptoworld(450,0,msg.info.origin.position.x,msg.info.origin.position.y,msg.info.resolution);
	print("450,0 = ( ", x ," , ",y)
	x , y = maptoworld(450,816,msg.info.origin.position.x,msg.info.origin.position.y,msg.info.resolution);
	print("450,816 = ( ", x ," , ",y)
	x , y = maptoworld(0,816,msg.info.origin.position.x,msg.info.origin.position.y,msg.info.resolution);
	print("0,816 = ( ", x ," , ",y)

def maptoworld(mx,my,cx,cy,resolution):
	wx = cx + (mx + 0.5) * resolution
	wy = cy + (my + 0.5) * resolution
	return wx , wy

def main():
	rospy.init_node('corners', anonymous=True)
	sub = rospy.Subscriber("/robot1/move_base/global_costmap/costmap",OccupancyGrid , call_back)
	rospy.spin()

if __name__ == '__main__':
	main()


