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


aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

image = cv2.imread("testImages/markersDetect.jpg")
image = cv2.resize(image, (640,480))

t0  = time.time()
cv2.namedWindow('Webcam', cv2.WINDOW_AUTOSIZE)

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


print(time.time()-t0)

key = cv2.waitKey(1) & 0xFF
while key != 27:
        time.sleep(0.1)
        cv2.imshow("Webcam", imgWithAruco)
        key = cv2.waitKey(1) & 0xFF











