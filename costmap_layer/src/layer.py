import cv2
import csv
import numpy as np
from numpy import genfromtxt
my_data = genfromtxt("/home/kraiem/Desktop/PFE/simulation_workspace/src/costmap_layer/costmap/costmap816_2.csv", delimiter=',')
for row in my_data:
	for x in range(len(row)):
		row[x]=int(row[x]*255)
print(my_data)
image = np.zeros((450,816,1), np.uint8) 

for j in range(816):
	k=0
	for i in range (450) :
		if (i < 21):
			image[i][j] = 0
		else :
			if (k < 815) :
				cost = int((my_data[k][j] + my_data[k+1][j])/2.5)
				image[i][j] = cost
			k+=2
image = np.asarray(image,dtype=np.uint8)
  

        	
_ , threshhold = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY)
_, contours, _ = cv2.findContours(threshhold ,cv2.RETR_EXTERNAL ,cv2.CHAIN_APPROX_SIMPLE)
i = 0 
for cnt in contours:
	x,y,w,h=cv2.boundingRect(cnt)
	cv2.rectangle(threshhold,(x,y),(x+w,y+h),(255,0,0),2)
	print ("rectangle " , i ,"x1 = " , x , "y1 = " ,y ,"x2 = " , x , "y2 = " ,y+h ,"x3 = " , x+w , "y1 = " ,y+h ,"x4 = " , x+w , "y4 = " ,y )
	i+=1


cv2.imshow("costmap",my_data)
cv2.imshow("costmap_thresh",threshhold)
cv2.waitKey(0)
cv2.destroyAllWindows()