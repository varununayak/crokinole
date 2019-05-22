'''
Provides a function that does openCv stuff on an image 
snapshot and returns a list of coins
'''


from CoinClass import Coin
import numpy as np
import cv2


#-----CALIBRATED VALUES HERE-----#

#focal lengths
focus_x = 986.1724
focus_y = 994.4793

#pixel offset of center of the board 
offset_pixel_x = -13.15
offset_pixel_y = 18.65

#Depth in mm, scaling factor in bracket
Depth = 736.00*(254.00/153.00)


#for colour of the coin
WHITE_THRESHOLD = 160 #average value of pixes threshold to distinguish white from black coins
BLACK_THRESHOLD = 120
CSD = 5	#colour search distance

CUE_POSITION_X = 167;
CUE_POSITION_Y = 9;

#----------------------------------#

# gets position of image in camera pixels 
# and returns the colour of the coin
def isWhite(img,i):
	avg = 0;
	#be careful of indexing here, 1 represents x-coordinate, 0 represents y
	avg = np.average(img[i[1]-CSD:i[1]+CSD,i[0]-CSD:i[0]+CSD])  #compute the average RGB value
	if avg < WHITE_THRESHOLD:
		return False		
	return True

def isBlack(img,i):
	avg = 0;
	#be careful of indexing here, 1 represents x-coordinate, 0 represents y
	avg = np.average(img[i[1]-CSD:i[1]+CSD,i[0]-CSD:i[0]+CSD])  #compute the average RGB value 
	if avg < BLACK_THRESHOLD:
		return True		
	return False

def isCue(X,Y):
	epsilion = 5 #error in mm
	if( np.sqrt(  (X-CUE_POSITION_X)**2 + (Y-CUE_POSITION_Y)**2 ) ) < epsilion:
		return True
	return False 

def updateBoardCoins():
	coins = [] 
	#capture the video with a VideoCapture object
	#argument 0 usually selects your laptop integrated webcam, other number (1,2,3.. try each!) grabs other cams
	#cap = cv2.VideoCapture(2)
	#ret, frame = cap.read()	#get the frame

	#for test
	frame = cv2.imread("../CrokinolePhotos/crokinole1.jpg",1)
	ret = True

	if ret==True:
		cimg = frame	#get the image as the coloured image
    	img = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY) #convert to grayscale
    	img = cv2.medianBlur(img,9) #run a filter over it to remove salt and pepper noise
    	#Hough Transform, you will probably need to play with the parameters here
    	circles = cv2.HoughCircles(img,cv2.cv.CV_HOUGH_GRADIENT,1.9,10,param1=50,param2=30,minRadius=8,maxRadius=14)
    	if (circles is not None):
	    	for i in circles[0,:]:
	    		coin = Coin()	#initialize the coin class
	    		if isWhite(img,i):#coin is white in colour
	    			cv2.circle(cimg,(i[0],i[1]),2,(255,0,0),3)	#blue colour center marker for our coin 
	    			#get the pixel value of the center of the coin
	    			pixel_x_cam = i[0]-320 - offset_pixel_x
	    			pixel_y_cam = -(i[1]-240) - offset_pixel_y
	    			#compute the position in mm
	    			X = pixel_x_cam/focus_x*Depth
	    			Y = pixel_y_cam/focus_y*Depth
	    			coin.setPosition(X,Y)
	    			if(isCue(X,Y)):
	    				coin.setIdentity(3)
	    			else:
	    				coin.setIdentity(1)	    				
	    			coins.append(coin)
	    		elif isBlack(img,i):#coin is black in colour
	    			cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)	#red colour center marker for black (opponent)
	    			#get the pixel value of the center of the coin
	    			pixel_x_cam = i[0]-640/2 - offset_pixel_x
	    			pixel_y_cam = -(i[1]-480/2) - offset_pixel_y
	    			#compute the position in mm
	    			X = pixel_x_cam/focus_x*Depth
	    			Y = pixel_y_cam/focus_y*Depth
	    			coin.setPosition(X,Y)
	    			coin.setIdentity(2)
	    			coins.append(coin)
	    		else:
	    			pass #circle not of interest
		

		cv2.imshow('Crokinole Detected Coins',cimg)

		print("Hit 'q' in the image window to get the coin positions")
        while True:
        	if cv2.waitKey(1) & 0xFF == ord('q'):
        		break

        		
       	else:
       		pass   

	# Release everything if job is finished
	#cap.release()
	cv2.destroyAllWindows()
	return coins



#--------------TEST HARNESS------------#

if __name__ == "__main__":
	coins = updateBoardCoins()
	print(coins[0].x,coins[0].y,coins[0].identity)


else:
	pass


#---------------------------------------#