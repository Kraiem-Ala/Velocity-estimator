#!/usr/bin/env python
import cv2 
import rospy 
from PIL import Image as im
from nav_msgs.msg import OccupancyGrid
import numpy as np

class image_converter:
	def __init__(self):
		self.image_sub = rospy.Subscriber("/robot1/move_base/global_costmap/costmap",OccupancyGrid,self.callback)
		#self.image_sub = rospy.Subscriber("/robot1/cropped_map",OccupancyGrid,self.callback)
		self.width = 0
		self.height = 0
		self.resolution = 0
		self.length = 0

	def callback(self,data):
		self.width = data.info.width
		self.height = data.info.height
		self.resolution = data.info.resolution
		self.origin_x=data.info.origin.position.x
		self.origin_y=data.info.origin.position.y
		objects = []
		ll=list(data.data)
		costmap_mat = np.asarray(ll,dtype=np.uint8)
		costmap_mat = np.reshape(costmap_mat, (self.height , self.width))
		_ , threshhold = cv2.threshold(costmap_mat, 10, 255, cv2.THRESH_BINARY)
		_, contours, _= cv2.findContours(threshhold,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		#print(contours)
		#print(contours)
		backtorgb = cv2.cvtColor(costmap_mat,cv2.COLOR_GRAY2RGB)
		i = 0
		for cnt in contours:
			i= i+1 
			area = cv2.contourArea(cnt)
			#print(area)
			x,y,w,h=cv2.boundingRect(cnt)
			objects.append((x, y, w, h))
			#backtorgb = cv2.rectangle(backtorgb,(x , y) ,(x+h , y+w) , (255,0,0), 2 )
			if (len(cnt) > 4) :
				polygons = cv2.drawContours(backtorgb,[cnt],0,(0,255,255),2)
				print(i)
		cv2.imshow("G_costmap" , backtorgb)
		cv2.imshow("black_costmap" ,threshhold)
		cv2.waitKey(3)

	
def main():
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	rospy.spin()

if __name__ == '__main__':
    main()
