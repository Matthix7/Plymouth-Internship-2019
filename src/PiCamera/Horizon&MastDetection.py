#!/usr/bin/env python
# -*- coding: utf-8 -*-

##import rospy

# import the necessary packages
import time
import cv2
import numpy as np
from math import atan2
from numpy import pi, cos, sin, array, shape


def horizonArea(image):

    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    rows,cols = grey.shape

    grad_y = cv2.Sobel(grey, -1, 0, 1, ksize = 3) #For horizon

    ret, bin_y = cv2.threshold(grad_y,100,255,0)

    horizontalLines = cv2.HoughLines(bin_y,1,np.pi/180,100)

    copie = image.copy()

    try:
        for rho,theta in horizontalLines[0]:

            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 50*(-b))
            y1 = int(y0 + 50*(a))
            x2 = int(x0 - cols*(-b))
            y2 = int(y0 - cols*(a))

            rotation = (theta-np.pi/2)*180/np.pi

    except:
        rotation = 0
        x0 = 0
        y0 = 0


    M = cv2.getRotationMatrix2D((x0, y0),rotation,1)

    rotated = cv2.warpAffine(copie,M,(cols,rows))

    rows_rotated, cols_rotated = shape(rotated)[0], shape(rotated)[1]

    if x0 != 0 and y0 != 0:
        horizon = int(M[1,0]*x0 + M[1,1]*y0 + M[1,2])
        left = max( int(M[0,0]*0 + M[0,1]*0 + M[0,2]), int(M[0,0]*0 + M[0,1]*rows_rotated + M[0,2]))+1
        right = min( int(M[0,0]*cols_rotated + M[0,1]*0 + M[0,2]), int(M[0,0]*cols_rotated + M[0,1]*rows_rotated + M[0,2]))-1


    else:
        horizon = rows_rotated/2
        left = 1
        right = cols_rotated-1

    cropped = rotated[int(horizon - 0.08*rows_rotated):int(horizon + 0.2*rows_rotated), left:right]

    return cropped






def run():

    cap = cv2.VideoCapture('testImages/some_boats.mp4')
    cv2.namedWindow('Result', cv2.WINDOW_NORMAL)

    while(cap.isOpened()):
        # Capture frame-by-frame
        ret, image = cap.read()

        if ret:

            image = cv2.resize(image, (640,480))


            horizon = horizonArea(image)

            grad_x = cv2.Sobel(horizon, -1, 1, 0, ksize = 3)

            ret, bin_x = cv2.threshold(grad_x,50,255,0)

            verticalLines = cv2.HoughLines(bin_x,1,np.pi/180,100)



#            copie = horizon.copy()

#            try:
#                for rho,theta in verticalLines[0]:

#                    a = np.cos(theta)
#                    b = np.sin(theta)
#                    x0 = a*rho
#                    y0 = b*rho
#                    x1 = int(x0 + 50*(-b))
#                    y1 = int(y0 + 50*(a))
#                    x2 = int(x0 - cols*(-b))
#                    y2 = int(y0 - cols*(a))

#                    cv2.line(copie, (x1, y1), (x2, y2), (0,0,255), 2)

#            except:
#                pass



            cv2.imshow('Result', bin_x)
            cv2.imshow('Origin', image)


            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                    break

            if key == 32:
                key = cv2.waitKey(1) & 0xFF
                while key != 32:
                    key = cv2.waitKey(1) & 0xFF

        else:
            break

    cap.release()
    cv2.destroyAllWindows()







if __name__ == "__main__":
    run()
