#!/usr/bin/env python
# -*- coding: utf-8 -*-


from threading import Thread
import cv2
import time
from numpy import array

from detectionBuoy import detectBuoy, getColorRange, getColorRangeTest
from detectionAruco import detectAruco
from cv2 import aruco

class BAdetector():
    def __init__(self):
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.frame_markers = None
        self.center = None
        self.corners = None
        self.stopped = False

#        self.colorRange = getColorRange() #For real buoys
        self.colorRange = getColorRangeTest() #For test target

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)


    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.detect, args=()).start()
        return self

    def detect(self):
        while not self.stopped:
            if self.frame is not None:

                self.center, buoy = detectBuoy(self.frame, self.frame.copy(), self.colorRange)
                self.frame_markers, self.corners = detectAruco(self.frame, buoy, self.aruco_dict)

                self.frame = None


    def newImage(self,image):
        self.frame = image
        self.frame_markers = None
        self.corners = None
        self.center = None

    def read(self):
        # return the frame most recently read
        return self.frame_markers, self.corners, self.center

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
