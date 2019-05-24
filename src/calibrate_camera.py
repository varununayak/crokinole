'''
Used to calibrate the camera with respect to the board

Calibrate the depth value and the center of the board


'''

import numpy as np
import cv2
from matplotlib import pyplot as plt

WHITE_THRESHOLD = 180 #average value of pixes threshold to distinguish white from black coins
CSD = 5	#colour search distance

A = np.array( [ [986.1724, 0,0]  ,  [0, 994.4793, 0] ,	[0,0,1] ])

#principle point of camera
p_principle = np.array([635.8148,370.9430])
p_principle = np.reshape(p_principle,(2,1))

#distortion parameters
radial_distort_x = -0.0383
radial_distort_y = 0.2577

#focal lengths
focus_x = 986.1724
focus_y = 994.4793

#translation from world frame to camera frame in mm 
t = np.array([0,0,1000])
t = np.reshape(t,(3,1))
#rotation from world frame to camera frame
R = np.array([  [ 1, 0, 0 ],[0, -1, 0 ],[0, 0, -1 ] 	])
AI = np.matmul(		A  ,  np.array([ [1,0,0,0],[0,1,0,0],[0,0,1,0]		 ])  )
Tform = np.hstack( (R, t )	)
Tform = np.vstack((Tform,[0,0,0,1]))
G = np.matmul(AI,Tform)

'''STEPS FOR CALIBRATION

1)	Turn on the camera above the board and try to align the 
	'X' of the camera with the board as well as possible. The goal
	is not to match the centers perfectly but to ensure that the 
	camera is facing perfectly downward toward the board. The 'X' helps.
2)	Set CALIB_DEPTH and CALIB_ANGLE to False.
3)	Put a white coin in the center and check the offset values. Update them.
4)	Now set CALIB_DEPTH to True.
5)	Place the coin in (254,0) of the board frame and check the X,Y values.
	We want X^2 + Y^2 = 254^2. Tune the Depth using the formul D_tuned = 254*D_current/sqrt(X^2 + Y^2)
6)  Once this is done, calculate theta as arctan(-Y/X). Note that the coin is still in 254,0.
7)	Set CALIB_ANGLE to True and check if you now print out 254,0, or somewhere close to it.



'''



#-------WHAT ARE YOU CALIBRATING--------#

CALIB_DEPTH = False
CALIB_ANGLE = False

#-------CALIBRATION VALUES--------#
#COPY THESE OVER TO CoinsUpdater.py

#pixel center offset
offset_pixel_x = 14
offset_pixel_y = -0.34

#Depth in mm, theta in radian
Depth = 1220
theta = -0.03

#---------------------------------#


def isWhite(img,i):
	avg = 0;
	x = int(i[0])
	y = int(i[1])
	#be careful of indexing here, 1 represents x-coordinate, 0 represents y
	avg = np.average(img[y-CSD:y+CSD,x-CSD:x+CSD])  #compute the average RGB value

	if avg < WHITE_THRESHOLD:
		return False		
	return True



def main():
	#capture the video with a VideoCapture object
	#argument 0 usually selects your laptop integrated webcam, other number (1,2,3.. try each!) grabs other cams
	cap = cv2.VideoCapture(0)	


	while(cap.isOpened()):
	    ret, frame = cap.read()
	    if ret==True:

	       	
	    	cimg = frame

	    	img = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)

	    	img = cv2.medianBlur(img,9)

	    	circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1.9,10,param1=60,param2=40,minRadius=8,maxRadius=14)

	    	if (circles is not None):

		    	for i in circles[0,:]:
		    		# draw the outer circle
		    		#cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
		    		if isWhite(img,i):#coin is white in colou
		    			cv2.circle(cimg,(i[0],i[1]),2,(255,0,0),3)	#blue colour center marker for our coin 


		    			pixel_x_cam = i[0]-320 - offset_pixel_x
		    			pixel_y_cam = -(i[1]-240) - offset_pixel_y;

		    			X = pixel_x_cam/focus_x*Depth
		    			Y = pixel_y_cam/focus_y*Depth
		    			
		    			if(CALIB_ANGLE):
		    				Xunrot = X;
		    				X = np.cos(theta)*Xunrot - np.sin(theta)*Y
		    				Y = np.sin(theta)*Xunrot + np.cos(theta)*Y



		    			if(CALIB_DEPTH):
		    				print(X,Y,"Need 254 at the points") #x and y coordinates in pixels    			
		    			else:
		    				#draw reference lines
							x1 =int(i[0])+100
							y1 =int(i[1])+100
							x0 =int(i[0])-100
							y0 =int(i[1])-100
							cv2.line(cimg,(x0,y0),(x1,y1),(0,0,255),6)
							x1 = int(i[0])+100
							y1 = int(i[1])-100
							x0 = int(i[0])-100
							y0 = int(i[1])+100
							cv2.line(cimg,(x0,y0),(x1,y1),(0,0,255),6)
							cv2.circle(cimg,(320,240),2,(255,0,0),3)							
							print(pixel_x_cam + offset_pixel_x,pixel_y_cam+offset_pixel_y,"Update offset values")

					cv2.imshow('Crokinole Detected Coins',cimg)		

		    		#else:

		    			#cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)	#red colour center marker for black (opponent)
			
			

	        if cv2.waitKey(1) & 0xFF == ord('q'):
	            break
	    else:
	        break

	# Release everything if job is finished
	cap.release()	
	cv2.destroyAllWindows()


if __name__ == "__main__":
	main()