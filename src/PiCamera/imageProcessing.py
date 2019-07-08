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


    cv2.namedWindow('Global', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Horizon', cv2.WINDOW_NORMAL)

    horizon_prev = (0, 320, 240)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    c = 0

###############          VIDEO           #############################
######################################################################
##    Running on test video
#    cap = cv2.VideoCapture('testImages/some_boat.mp4')

#    t0 = time.time()

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

    t0 = time.time()

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
        t1 = time.time()-t0
        horizon, horizon_height, horizon_prev = horizonArea(image, horizon_prev)
        print('T1', time.time()-t1)

        #Find the areas where vertical lines are found (ie possible sailboats).
        #Takes about 0.1s per frame.
        #masts: image cropped around the horizon, where vertical lines are highlighted
        t2 = time.time()-t0
        masts = detectMast(horizon, horizon_height)
        print('T2', time.time()-t2)

        #Find the buoy in the cropped image and highlight them in the result image
        t3 = time.time()-t0
        colorRange = getColorRange()
        center, buoy = detectBuoy(image, image.copy(), colorRange)
        print('T3', time.time()-t3)

        #Find the April Tags in the cropped image
        t4 = time.time()-t0
        frame_markers, corners = detectAruco(image, buoy, aruco_dict)
        print('T4', time.time()-t4)



        cv2.imshow('Global', frame_markers)
        cv2.imshow('Horizon', masts)
        time.sleep(0.05)

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
    print("Time per frame : ", (time.time()-t0)/c - 0.05)


if __name__ == "__main__":
    run()
