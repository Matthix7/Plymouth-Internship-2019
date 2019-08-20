#!/usr/bin/env python
# -*- coding: utf-8 -*-

##import rospy

# import the necessary packages
#from picamera.array import PiRGBArray
#from picamera import PiCamera
import time
import cv2
from cv2 import aruco
import numpy as np
from math import atan2
from numpy import pi, cos, sin, array, shape

def run():

#    # initialize the camera and grab a reference to the raw camera capture
#    camera = PiCamera()
#    camera.resolution = (640, 480)
#    camera.framerate = 32
#    rawCapture = PiRGBArray(camera, size=(640, 480))
    cv2.namedWindow('Webcam', cv2.WINDOW_NORMAL)

    cap = cv2.VideoCapture('testImages/some_buoys.mp4')


    # allow the camera to warmup
    time.sleep(0.1)

    #picture counter
    c=0



#    # capture frames from the camera
#    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

#        # grab the raw NumPy array representing the image, then initialize the timestamp
#        # and occupied/unoccupied text
#        image = frame.array

    while cap.isOpened():

        ret, image = cap.read()
        if not ret:
            break

        colorRange = getColorRange()
        center1, image = detectBuoy(image, colorRange)

        # clear the stream in preparation for the next frame
#        rawCapture.truncate(0)

        #show the frame
        cv2.imshow('Webcam',image)
        time.sleep(0.03)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
                break

        if key == 32:
            key = cv2.waitKey(1) & 0xFF
            while key != 32:
                key = cv2.waitKey(1) & 0xFF

        elif key == ord('c'):
            cv2.imwrite('sample_buoy.png',image)
            print("Picture saved")

    cv2.destroyAllWindows()



def getColorRange():
    # define range of buoy color in HSV
    # voir https://www.google.com/search?client=firefox-b&q=%23D9E80F

    hue_min = 345
    hue_max = 400
    sat_min = 45
    sat_max = 100
    val_min = 40
    val_max = 100

    lower = np.array([int(hue_min/2),int(sat_min*255/100),int(val_min*255/100)])
    upper = np.array([int(hue_max/2),int(sat_max*255/100),int(val_max*255/100)])

    return (lower, upper)


def getColorRangeTest():
    # define range of buoy color in HSV
    # voir https://www.google.com/search?client=firefox-b&q=%23D9E80F

    hue_min = 160
    hue_max = 207
    sat_min = 50
    sat_max = 100
    val_min = 51
    val_max = 100

    lower = np.array([int(hue_min/2),int(sat_min*255/100),int(val_min*255/100)])
    upper = np.array([int(hue_max/2),int(sat_max*255/100),int(val_max*255/100)])

    return (lower, upper)

def getColorRangeTest2():
    # define range of buoy color in HSV
    # voir https://www.google.com/search?client=firefox-b&q=%23D9E80F

    hue_min = 345
    hue_max = 354
    sat_min = 48
    sat_max = 96
    val_min = 21
    val_max = 70

    lower = np.array([int(hue_min/2),int(sat_min*255/100),int(val_min*255/100)])
    upper = np.array([int(hue_max/2),int(sat_max*255/100),int(val_max*255/100)])

    return (lower, upper)



def detectBuoy(image, resultImage, colorRange):

    lower, upper = colorRange[0], colorRange[1]

    # Convert BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only yellow/green colors
    mask1 = cv2.inRange(hsv, lower, upper)
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

        cv2.circle(resultImage,center1,radius1,(255,0,0),1)
        cv2.circle(resultImage,center1,1,(255,0,0),2)


    else:
        center1 = None
        radius1 = None

    return center1, radius1, resultImage




if __name__ == "__main__":
    run()
