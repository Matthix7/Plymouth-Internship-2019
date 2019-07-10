#!/usr/bin/env python
# -*- coding: utf-8 -*-


from threading import Thread
import cv2
import time
from numpy import array

from buoyArucoThread import BAdetector


ba = BAdetector()
ba.start()

image = cv2.imread('testImages/plsound.png')
ba.newImage(image)
frame_markers, corners, center = ba.read()
while frame_markers is None:
    frame_markers, corners, center = ba.read()

cv2.imshow('Horizon', frame_markers)

key = cv2.waitKey(1) & 0xFF
while not ( key == 27 or key == ord('q')):
    key = cv2.waitKey(1) & 0xFF

ba.stop()
