#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

def run():

    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    cv2.namedWindow('Webcam', cv2.WINDOW_AUTOSIZE)

    # allow the camera to warmup
    time.sleep(0.1)

    # define range of blue color in HSV
    # voir https://www.google.com/search?client=firefox-b&q=%23D9E80F
    # convertir valeur dans [0,179], [0,255], [0,255]

    teinte_min = 160
    teinte_max = 207
    sat_min = 50
    sat_max = 100
    val_min = 51
    val_max = 100

    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)



        lower_blue = np.array([int(teinte_min/2),int(sat_min*255/100),int(val_min*255/100)])
        upper_blue = np.array([int(teinte_max/2),int(sat_max*255/100),int(val_max*255/100)])

        # Threshold the HSV image to get only yellow/green colors
        mask1 = cv2.inRange(hsv, lower_blue, upper_blue)
        mask1 = cv2.medianBlur(mask1, 5)


        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image,image, mask= mask1)

        ret1,thresh1 = cv2.threshold(mask1,127,255,0)
        im2,contours1,hierarchy1 = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, 2)

        if len(contours1) > 0:
            cnt1 = max(contours1, key = cv2.contourArea)

            (x1,y1),radius1 = cv2.minEnclosingCircle(cnt1)
            center1 = (int(x1),int(y1))
            radius1 = int(radius1)

            cv2.circle(image,center1,radius1,(255,0,0),2)
            cv2.circle(image,center1,5,(255,0,0),2)

        image = cv2.flip(image, 1 )

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        #show the frame
        cv2.imshow('Webcam',image)

        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q") or rospy.is_shutdown():
            break

        # if the SPACE key was pressed, take a picture
        elif key == 32:
            c += 1
            cv2.imwrite('fra%i.png'%c,image)
            print("Picture saved")

    cv2.destroyAllWindows()





