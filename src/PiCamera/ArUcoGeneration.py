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

fig = plt.figure()
nx = 1
ny = 1
for i in range(1, nx*ny+1):
    ax = fig.add_subplot(ny,nx, i)
    ax.set_title("ArUco %d"%i)
    img = aruco.drawMarker(aruco_dict,i, 1300)
    plt.imshow(img, cmap = matplotlib.cm.gray, interpolation = "nearest")
    ax.axis("off")

plt.savefig("markers.png")
plt.show()
