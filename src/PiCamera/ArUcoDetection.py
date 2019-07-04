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
import matplotlib.pyplot as plt
import matplotlib


def run():

    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32

    camera.exposure_mode = 'sports'

    rawCapture = PiRGBArray(camera, size=(640, 480))


    t0 = time.time()

    cv2.namedWindow('Webcam', cv2.WINDOW_AUTOSIZE)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    c = 0

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        c+=1

        image = cv2.resize(frame.array, (640,480))

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)



        markerLength = 3.8
        camera_matrix = array([[485.36568341, 0, 308.96642615], [0, 486.22575965, 236.66818825], [0, 0, 1]])
        dist_coeffs = array([[1.37958351e-01, -2.43061015e-01, -5.22562568e-05, -6.84849581e-03, -2.59284496e-02]])

        rvec, tvec = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs) # For a single marker

        for i in range(len(ids)):
            imgWithAruco = aruco.drawAxis(frame_markers, camera_matrix, dist_coeffs, rvec[i], tvec[i], 10)


        cv2.imshow("Webcam", imgWithAruco)



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


    cap.release()
    cv2.destroyAllWindows()
    print("Total time : ",time.time()-t0)
    print("Computed frames : ", c)
    print("Time per frame : ", (time.time()-t0)/c - 0.1)









