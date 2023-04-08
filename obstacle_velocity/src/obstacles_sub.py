#!/usr/bin/env python
from nav_msgs.msg import Odometry
from costmap_converter.msg import ObstacleArrayMsg
import rospy 
from math import sqrt 
readings = 0
centers =[]
last_number_of_centers = 0;
def obstacle_callback(data):
	global readings , last_number_of_centers ;
	print("---------", readings ,"---------")
	readings +=1
	obstacles = data.obstacles
	print("we have " ,len(obstacles), "obstacles")
	i = 0
	for obs in obstacles:
		print("obstacle ", i, "we have " ,len(obs.polygon.points), " points")
		print(obs.polygon.points)
		print("------ center -------")
		print(center(obs.polygon.points))
		print(" ")
		i+=1

def center(points):
	max_distance = 0 ;
	center_x = center_y = float()
	for i in range (len(points)-1):
		j=(i+1)
		for j  in range(len(points)-i-1) :
			distances = distace(points[i].x , points[i].y ,points[j].x , points[j].y )
			print(distances)
			if( distances > max_distance):
				max_distance = distances
				center_x = (points[i].x + points[j].x) / 2
				center_y = (points[i].y + points[j].y) / 2 
	return center_x , center_y

def distace(x1 ,y1 , x2 , y2):
	return (sqrt((x2-x1)**2 + (y2 - y1)**2))

def main():
	rospy.init_node('tracking', anonymous=True)
	sub = rospy.Subscriber("/robot1/critic_obstacles",ObstacleArrayMsg , obstacle_callback)
	rospy.spin()

if __name__ == '__main__':
	main()