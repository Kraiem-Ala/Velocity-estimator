#!/usr/bin/env python
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from obstacle_velocity.msg import obstacles_msgs
from obstacle_velocity.msg import obstacle
from costmap_converter.msg import ObstacleArrayMsg
from std_msgs.msg import Header
import rospy 
from math import sqrt , atan2 , pi
import tf 
import numpy as np 
class velocity_calculator(object):
	"""docstring for velocity_calculator"""
	def __init__(self , robot_name):
		super(velocity_calculator, self).__init__()
		begin = rospy.Time.now()
		rospy.loginfo_once(rospy.get_caller_id()+" started at %i %i" , begin.secs , begin.nsecs)
		self.header = Header()
		self.header.seq = 0
		self.header.frame_id='map'
		self.obstacles = []
		self.tries = 0
		self.frames = 0
		self.centers =[]
		self.last_centers=[]
		self.speeds = []
		self.orientation = []
		self.spped_tries = []
		self.robot_message = obstacles_msgs()
		self.robot_name = robot_name 
		self.odom_subscriber = rospy.Subscriber("/odom",Odometry ,self.odom_callback)
		self.obstacles_subscriber = rospy.Subscriber("/"+self.robot_name + "/critic_obstacles",ObstacleArrayMsg , self.obstacle_callback)
		self.speed_publisher = rospy.Publisher("/" +self.robot_name + '/speeds', obstacles_msgs, queue_size=2)
		self.speed_tries = 0


	def odom_callback(self , data):
		self.robot_message.robot_odometry =data.pose.pose.position
		self.robot_message.acceleration = round((data.twist.twist.linear.x - self.robot_message.velocity)*100 , 5)
		self.robot_message.velocity = data.twist.twist.linear.x
		(roll,pitch,yaw) = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x , data.pose.pose.orientation.y , data.pose.pose.orientation.z ,data.pose.pose.orientation.w))
		self.robot_message.robot_orientation = yaw 
	

	def obstacle_callback(self , data):
		self.obstacles = data.obstacles
		self.centers=[]
	
	def speed_calculate(self) :	
		self.header.seq += 1
		self.header.stamp = rospy.Time.now()
		self.robot_message.header = self.header
		if len(self.obstacles)==0 :
			pass
		else :
			self.tries+=1
			if self.tries%2 == 1 :
				pass
			else : 
				for obs in self.obstacles:
					if len(obs.polygon.points) <= 1:
						pass
					else:
						centre_x , centre_y = self.centroide(obs.polygon.points)
						self.centers.append((centre_x , centre_y)) 
				if (len(self.centers) > 0) :
					if self.frames % 2 == 0: 
						#print("frames" , self.frames)
						self.last_centers = self.centers[:]
						#print("last centers ",self.last_centers)
					else : 
						for i in range(min(len(self.last_centers) , len(self.centers))):
							x_speed = (self.centers[i][0] - self.last_centers[i][0])*1.67
							y_speed = (self.centers[i][1] - self.last_centers[i][1])*1.67
							speed = sqrt(x_speed**2 + y_speed**2)
							if (x_speed != 0) :
								orientation = atan2((self.centers[i][1] - self.last_centers[i][1]),(self.centers[i][0] - self.last_centers[i][0]))
							else :
								orientation = np.sign(y_speed)* (pi/2)
							if (speed > 1.15) :
								speed = 1.07

							self.orientation.append(orientation)
							self.speeds.append(speed)
						self.last_centers=[]
						#print (self.robot_name , "obstacles speeds")
						#print (self.speeds)
						for i in range(len(self.speeds)):
							obstacle_msg = obstacle() 
							obstacle_msg.ID = 0 
							obstacle_msg.obst_velocity = self.speeds[i]
							obstacle_msg.obst_pose.x = self.centers[i][0]
							obstacle_msg.obst_pose.y = self.centers[i][1]
							obstacle_msg.obst_orientation = self.orientation[i]
							self.robot_message.obstacles.append(obstacle_msg)
						self.orientation=[]
						self.speeds=[]

					self.frames+=1
		self.speed_publisher.publish(self.robot_message)
		self.robot_message.obstacles=[]



	def centroide (self , points ):
		max_distance = 0 ;
		center_x = center_y = float()
		for i in range (len(points)-1):
			j=(i+1)
			for j  in range(len(points)-i-1) :
				distances = self.distace(points[i].x , points[i].y ,points[j].x , points[j].y )
				if( distances > max_distance):
					max_distance = distances
					center_x = (points[i].x + points[j].x) / 2
					center_y = (points[i].y + points[j].y) / 2 
		#print(center_x , center_y)
		return center_x , center_y

	def distace(self ,x1 ,y1 , x2 , y2):
		return (sqrt((x2-x1)**2 + (y2 - y1)**2))

def main():
	rospy.init_node('tracking', anonymous=True)
	robot_name = rospy.get_param('~robot_name')
	ic = velocity_calculator(robot_name)
	rate = rospy.Rate(2) # 10hz
	while not rospy.is_shutdown():
		ic.speed_calculate()
		rate.sleep()

if __name__ == '__main__':
    main()




		