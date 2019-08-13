#!/usr/bin/env python
# -*- coding: utf-8 -*-

##import rospy

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np


camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32

camera.exposure_mode = 'sports'
    
rawCapture = PiRGBArray(camera, size=(640, 480))

c = 33

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array

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
