#!/usr/bin/env python
# -*- coding: utf-8 -*-

##import rospy

# import the necessary packages
import time
import cv2
import numpy as np
from math import atan2
from numpy import pi, cos, sin, array, shape



def run():

    cap = cv2.VideoCapture('testImages/some_boat.mp4')
    cv2.namedWindow('Result', cv2.WINDOW_NORMAL)

    while(cap.isOpened()):
        # Capture frame-by-frame
        ret, image = cap.read()

        if ret:

            image = cv2.resize(image, (640,480))

            horizon, horizon_height = horizonArea(image)

            masts = detectMast(horizon, horizon_height)


            cv2.imshow('Result', masts)
#            cv2.imshow('Origin', image)

            time.sleep(0.1)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                    break

            if key == 32:
                key = cv2.waitKey(1) & 0xFF
                while key != 32:
                    key = cv2.waitKey(1) & 0xFF

            elif key == ord('c'):
                cv2.imwrite('sample.png',masts)
                print("Picture saved")

        else:
            break

    cap.release()
    cv2.destroyAllWindows()






####################################################################################################################################
####################################################################################################################################
######################                           USEFUL FUNCTIONS                            #######################################
####################################################################################################################################
####################################################################################################################################




def horizonArea(image):

    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    rows,cols = grey.shape

    grad_y = cv2.Sobel(grey, -1, 0, 1, ksize = 3) #For horizon

    ret, bin_y = cv2.threshold(grad_y,100,255,0)

    horizontalLines = cv2.HoughLines(bin_y,1,np.pi/180,100)

    if horizontalLines is not None:
        for rho,theta in horizontalLines[0]:

            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho

            rotation = (theta-np.pi/2)*180/np.pi
    else:
        rotation = 0
        x0 = 0
        y0 = 0



    M = cv2.getRotationMatrix2D((x0, y0),rotation,1)

    rotated = cv2.warpAffine(image,M,(cols,rows))

    rows_rotated, cols_rotated = shape(rotated)[0], shape(rotated)[1]

    if x0 != 0 and y0 != 0:
        horizon = int(M[1,0]*x0 + M[1,1]*y0 + M[1,2])
        left = max( int(M[0,0]*0 + M[0,1]*0 + M[0,2]), int(M[0,0]*0 + M[0,1]*rows_rotated + M[0,2]))+1
        right = min( int(M[0,0]*cols_rotated + M[0,1]*0 + M[0,2]), int(M[0,0]*cols_rotated + M[0,1]*rows_rotated + M[0,2]))-1


    else:
        horizon = rows_rotated/2
        left = 1
        right = cols_rotated-1

    bottom_margin, top_margin = 0.08, 0.2
    cropped = rotated[int(horizon - top_margin*rows_rotated):int(horizon + bottom_margin*rows_rotated), left:right]

    horizon_height = int(top_margin*rows_rotated)

    return cropped, horizon_height


####################################################################################################################################
####################################################################################################################################

def detectMast(horizon, horizon_height):

    grey = cv2.cvtColor(horizon, cv2.COLOR_BGR2GRAY)
    grad_x = cv2.Sobel(grey, -1, 1, 0, ksize = 3)

    ret, bin_x = cv2.threshold(grad_x,50,255,0)

    kernel = np.zeros((7,7), np.uint8)
    kernel[:,3] = np.ones((7,), np.uint8)

    bin_x = cv2.morphologyEx(bin_x, cv2.MORPH_OPEN, kernel)

    verticalLines = cv2.HoughLines(bin_x,1,np.pi/180,0)

    possible_masts = []

    if verticalLines is not None:
        for verticalLine in verticalLines:
            print(verticalLine)
            for rho,theta in verticalLine:

                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 10000*(-b))
                y1 = int(y0 + 10000*(a))
                x2 = int(x0 - 10000*(-b))
                y2 = int(y0 - 10000*(a))

                xMast = x2 - ((x2-x1)*(y2-horizon_height))/(y2-y1)

                if -pi/4 < theta and theta < pi/4 and newMast(xMast, possible_masts):
                    possible_masts.append(xMast)
                    cv2.line(horizon, (xMast, horizon_height-10), (xMast, horizon_height+10), (0,0,255), 2)
                    cv2.line(horizon, (x1, y1), (x2, y2), (0,0,255), 1)

    return horizon



####################################################################################################################################
####################################################################################################################################




def newMast(possibleNew, mastList):
    check = True
    for mast in mastList:
        if abs(possibleNew - mast) < 30:
            check = False
    return check








if __name__ == "__main__":
    run()
