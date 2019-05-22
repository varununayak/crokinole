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



#-------WHAT ARE YOU CALIBRATING--------#

#CALIB_CENTER = True
CALIB_DEPTH = True



#-------CALIBRATION VALUES--------#

offset_pixel_x = -13.15
offset_pixel_y = 18.65

#Depth in mm, scaling factor in bracket
Depth = 736.00*(254.00/153.00)

#---------------------------------#


def isWhite(img,i):
	avg = 0;

	#be careful of indexing here, 1 represents x-coordinate, 0 represents y
	avg = np.average(img[i[1]-CSD:i[1]+CSD,i[0]-CSD:i[0]+CSD])  #compute the average RGB value

	if avg < WHITE_THRESHOLD:
		return False		
	return True



def main():
	#capture the video with a VideoCapture object
	#argument 0 usually selects your laptop integrated webcam, other number (1,2,3.. try each!) grabs other cams
	cap = cv2.VideoCapture(2)	

	# Define the codec and create VideoWriter object
	fourcc = cv2.cv.CV_FOURCC(*'XVID')
	out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

	while(cap.isOpened()):
	    ret, frame = cap.read()
	    if ret==True:

	       	
	    	cimg = frame

	    	img = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)

	    	img = cv2.medianBlur(img,9)

	    	circles = cv2.HoughCircles(img,cv2.cv.CV_HOUGH_GRADIENT,1.9,10,param1=50,param2=30,minRadius=8,maxRadius=14)

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

		    			if(CALIB_DEPTH):
		    				print(X,Y,"Need 254 at the points") #x and y coordinates in pixels    			
		    			else:
		    				print(pixel_x_cam + offset_pixel_x,pixel_y_cam+offset_pixel_y,"Update offset values")


		    		#else:

		    			#cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)	#red colour center marker for black (opponent)
			

			cv2.imshow('Crokinole Detected Coins',cimg)

	        if cv2.waitKey(1) & 0xFF == ord('q'):
	            break
	    else:
	        break

	# Release everything if job is finished
	cap.release()
	out.release()
	cv2.destroyAllWindows()


if __name__ == "__main__":
	main()