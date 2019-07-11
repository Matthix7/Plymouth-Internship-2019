#!/usr/bin/env python
# -*- coding: utf-8 -*-

##import rospy

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from numpy import tan

def getCamDistortData(filename):
    saveFile = open(filename, 'r')
    lines = saveFile.readlines()
    saveFile.close()

    for i in range(len(lines)):
        if lines[i] == '#mtx\n':
            i+=1
            mtxString = lines[i]+lines[i+1]+lines[i+2]
            mtxList = mtxString.split()
            mtx = np.zeros((3,3))
            cnt = 0
            
            for part in mtxList:
                               
                try:
                    while part[0] == '[':
                        part = part[1:]

                    while part[-1] == ']':
                        part = part[:-1]
                    
                    mtx[cnt//3][cnt%3] = float(part)
                    cnt+=1
                except:
                    pass

        if lines[i] == '#dist\n':
            i+=1
            distString = lines[i]+lines[i+1]
            distList = distString.split()
            dist = np.zeros((1,5))
            cnt = 0
            
            for part in distList:
                               
                try:
                    while part[0] == '[':
                        part = part[1:]

                    while part[-1] == ']':
                        part = part[:-1]
                    
                    dist[0][cnt] = float(part)
                    cnt+=1
                except:
                    pass
                      
            return mtx, dist




def run():

    # Fill in the shape of the object to be detected and localised, in meters
    targetMaxDim = 0.11   
    targetMinDim = 0.068

    Sf = tan(0.398)/203.55   #Camera scale factor, rad/pixel

    # Read the camera matrix from calibration file
    mtx, dist = getCamDistortData('calibration_data.txt')
    

    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32

    camera.exposure_mode = 'sports'
    
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
        image = cv2.undistort(frame.array, mtx, dist, None)
##        image = frame.array


        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


        lower_blue = np.array([int(teinte_min/2),int(sat_min*255/100),int(val_min*255/100)])
        upper_blue = np.array([int(teinte_max/2),int(sat_max*255/100),int(val_max*255/100)])

        # Threshold the HSV image to get only yellow/green colors
        mask1 = cv2.inRange(hsv, lower_blue, upper_blue)
        mask1 = cv2.medianBlur(mask1, 5)


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

            if h > 0 and w > 0:
                dist1 = targetMaxDim/(h*Sf)
                dist2 = targetMinDim/(w*Sf)
                print("dist1 = ", dist1, "\ndist2 = ", dist2)

            heading = Sf*(x-camera.resolution[0]/2)
            print("Heading = ", heading)







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


if __name__ == "__main__":
    run()
