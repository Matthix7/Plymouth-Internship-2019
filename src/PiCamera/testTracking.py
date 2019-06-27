#!/usr/bin/env python
# -*- coding: utf-8 -*-

##import rospy

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

def run():

    # Fill in the shape of the object to be detected and localised, in meters
    targetMaxDim = 0.11   
    targetMinDim = 0.068

    Sp = 0.11/49.5   #Camera scale factor

    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    cv2.namedWindow('Webcam', cv2.WINDOW_AUTOSIZE)

    # allow the camera to warmup
    time.sleep(0.1)

    #picture counter
    c=0

    # define range of blue color in HSV
    # voir https://www.google.com/search?client=firefox-b&q=%23D9E80F

    teinte_min = 160
    teinte_max = 207
    sat_min = 50
    sat_max = 100
    val_min = 51
    val_max = 100

    init = True
##    W, H = [], []

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


            rect = cv2.minAreaRect(cnt1)
            box = np.int0(cv2.boxPoints(rect))
            cv2.drawContours(image, [box], 0, (0,0,255),2)

            x,y = rect[0]
            w,h = rect[1]
            if w > h:
                w,h = h,w
            angle = rect[2]

            dist1 = targetMaxDim/(h*Sp)
            dist2 = targetMinDim/(w*Sp)
            print("Width = ", w, "\nHeight = ", h)
            print("dist1 = ", dist1, "\ndist2 = ", dist2)
##            H.append(h)
##            W.append(w)
            

        # TO BE MODIFIED displayed image
        image = cv2.flip(image, 0)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        #show the frame
        cv2.imshow('Webcam',image)

        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q") or key == 27:
            break

        # if the SPACE key was pressed, take a picture
        elif key == 32:
            c += 1
            cv2.imwrite('fra%i.png'%c,image)
            print("Picture saved")

    cv2.destroyAllWindows()
##    H,W = np.asarray(H), np.asarray(W)
##    print("Median (H,W) ", np.median(H), np.median(W))


if __name__ == "__main__":
    run()


