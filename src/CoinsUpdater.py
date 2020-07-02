'''
Provides a function that does openCv stuff on an image 
snapshot and returns a list of coins present in that snapshot
'''


from calibrate_camera import CSD	# color search distance
from calibrate_camera import isBlack, isWhite
from disk import *
import numpy as np
import cv2
import time

# focal lengths
from calibrate_camera import FOCUS_X, FOCUS_Y

#-----CALIBRATED VALUES GO HERE-----#

# average value of pixes threshold to distinguish white from black coins
WHITE_THRESHOLD = 165
BLACK_THRESHOLD = 165

# pixel center offset
offset_pixel_x = -62.5
offset_pixel_y = -8

#Depth in mm, theta in radian
Depth = 1220*254/np.sqrt(251.4**2+2.4**2)
theta = -0.001

#----------------------------------#

CUE_POSITION_X = -140
CUE_POSITION_Y = -183

BOUNDARY_RADIUS = 255

#----------------------------------#

def isCue(X, Y):
    epsilion = 50  # error in mm
    if(np.sqrt((X-CUE_POSITION_X)**2 + (Y-CUE_POSITION_Y)**2)) < epsilion:
        return True
    return False

def withinBoardBoundary(X, Y):
    return ((np.sqrt(X**2 + Y**2) < BOUNDARY_RADIUS) and (np.sqrt(X**2 + Y**2) > 8))

def updateBoardCoins():
    """ returns a list of coins available in a video snapshot """
    list_of_coins = []
    # capture the video with a VideoCapture object
    # argument 0 usually selects your laptop integrated webcam, other number (1,2,3.. try each!) grabs other cams
    num_caps = 80
    frames = []
    cap = cv2.VideoCapture(-1)
    time.sleep(2)

    for k in range(num_caps):
        ret, frame = cap.read()
        if not ret:
            break
        else:
            if k > 20:
                frames.append(frame)

    for frame in frames:
        coins = []
        if ret == True:
            cimg = frame  # get the image as the coloured image
        img = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)  # convert to grayscale
        # run a filter over it to remove salt and pepper noise
        img = cv2.medianBlur(img, 9)
        # Hough Transform, you will probably need to play with the parameters here
        circles = cv2.HoughCircles(
            img, cv2.HOUGH_GRADIENT, 1.9, 10, param1=50, param2=30, minRadius=8, maxRadius=12)
        if (circles is not None):
            for i in circles[0, :]:
                coin = Disk()  # initialize the coin class
                if isWhite(img, i):  # coin is white in colour
                    # blue colour center marker for our coin
                    cv2.circle(cimg, (i[0], i[1]), 2, (255, 0, 0), 3)
                    # get the pixel value of the center of the coin
                    pixel_x_cam = i[0]-320 - offset_pixel_x
                    pixel_y_cam = -(i[1]-240) - offset_pixel_y
                    # compute the position in mm
                    X = pixel_x_cam/focus_x*Depth
                    Y = pixel_y_cam/focus_y*Depth
                    Xunrot = X
                    X = np.cos(theta)*Xunrot - np.sin(theta)*Y
                    Y = np.sin(theta)*Xunrot + np.cos(theta)*Y
                    if(isCue(X, Y)):
                        coin.set_position(X, Y)
                        coin.set_identity(3)
                        coins.append(coin)
                    elif(withinBoardBoundary(X, Y)):
                        coin.set_position(X, Y)
                        coin.set_identity(1)
                        coins.append(coin)
                    else:
                        pass
                elif isBlack(img, i):  # coin is black in colour
                    # red colour center marker for black (opponent)
                    cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)
                    # get the pixel value of the center of the coin
                    pixel_x_cam = i[0]-640/2 - offset_pixel_x
                    pixel_y_cam = -(i[1]-480/2) - offset_pixel_y
                    # compute the position in mm
                    X = pixel_x_cam/focus_x*Depth
                    Y = pixel_y_cam/focus_y*Depth
                    Xunrot = X
                    X = np.cos(theta)*Xunrot - np.sin(theta)*Y
                    Y = np.sin(theta)*Xunrot + np.cos(theta)*Y
                    if(withinBoardBoundary(X, Y)):
                        coin.set_position(X, Y)
                        coin.set_identity(2)
                        coins.append(coin)
                else:
                    pass  # circle not of interest

            list_of_coins.append(coins)
    numbers_of_coins = []
    for j in range(len(list_of_coins)):
        numbers_of_coins.append(len(list_of_coins[j]))
    print(numbers_of_coins)
    counts = np.bincount(numbers_of_coins)
    idx_max_freq = np.argmax(counts)
    cv2.destroyAllWindows()
    return list_of_coins[idx_max_freq]


#--------------TEST HARNESS------------#
if __name__ == "__main__":
    coins = updateBoardCoins()
    for coin in coins:
        print(coin.origin[0], coin.origin[1], coin.identity)
else:
    pass
#---------------------------------------#
