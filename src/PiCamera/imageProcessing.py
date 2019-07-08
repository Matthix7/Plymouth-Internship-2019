#!/usr/bin/env python
# -*- coding: utf-8 -*-

##import rospy

# import the necessary packages
import time
import cv2
from cv2 import aruco
import numpy as np
from math import atan2
from numpy import pi, cos, sin, array, shape


from picamera.array import PiRGBArray
from picamera import PiCamera


from detectionBuoy import detectBuoy, getColorRange
from detectionHorizonMast import horizonArea, detectMast
from detectionAruco import detectAruco


def run():

    t0 = time.time()

    cv2.namedWindow('Result', cv2.WINDOW_NORMAL)

    horizon_prev = (0, 320, 240)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    c = 0

###############          VIDEO           #############################
######################################################################
##    Running on test video
#    cap = cv2.VideoCapture('testImages/some_buoys.mp4')

#    while(cap.isOpened()):

#        # Capture frame-by-frame
#        ret, image = cap.read()

#        if not ret:
#            break

#        image = cv2.resize(image, (640,480))

#################     CAMERA     ####################################
#####################################################################
#    Running with the camera
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32

    camera.exposure_mode = 'sports'

    rawCapture = PiRGBArray(camera, size=(640, 480))

    # allow the camera to warmup
    time.sleep(0.1)

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        image = cv2.resize(frame.array, (640,480))

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
#####################################################################
#####################################################################


        c += 1

#        cv2.imshow('Origin', image)

        #Find the area where horizon is located and return a frame containing the horizon, transformed to be horizontal.
        #Takes about 0.04s per frame.
        #horizon: image cropped around the horizon
        #horizon_height: vertical position in pixels of the horizon in the cropped image (for masts detection)
        #horizon_prev: vertical position in pixels of the horizon in the previous uncropped image, in case horizon is not
        #detected in the new image.
        horizon, horizon_height, horizon_prev = horizonArea(image, horizon_prev)

        #Find the areas where vertical lines are found (ie possible sailboats).
        #Takes about 0.1s per frame.
        #masts: image cropped around the horizon, where vertical lines are highlighted
        masts = detectMast(horizon, horizon_height)

        #Find the buoy in the cropped image and highlight them in the result image
        colorRange = getColorRange()
        center, buoy = detectBuoy(horizon, masts, colorRange)

        #Find the April Tags in the cropped image
        frame_markers, corners = detectAruco(horizon, buoy, aruco_dict)



        cv2.imshow('Result', horizon)
#        time.sleep(0.1)

#####################################################################
#############        INTERACTION          ###########################


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


    try:
        cap.release()
    except:
        pass
    cv2.destroyAllWindows()
    print("Total time : ",time.time()-t0)
    print("Computed frames : ", c)
    print("Time per frame : ", (time.time()-t0)/c - 0.1)


if __name__ == "__main__":
    run()
