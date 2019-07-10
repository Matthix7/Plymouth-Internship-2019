#!/usr/bin/env python
# -*- coding: utf-8 -*-


from threading import Thread
import cv2
import time
from numpy import array

from detectionHorizonMast import horizonArea, detectMast


class HMdetector():
    def __init__(self, resolution=(640, 480)):
        self.horizon_prev = (0, resolution[0]/2, resolution[1]/2)

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.masts = None
        self.called = False
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.detect, args=()).start()
        return self

    def detect(self):
        while not self.stopped:
            if (not self.called) and (self.frame is not None):

                horizon, horizon_height, self.horizon_prev = horizonArea(self.frame, self.horizon_prev)

                self.masts = detectMast(horizon, horizon_height)
#            else:
#                time.sleep(0.05)



    def newImage(self,image):
        self.frame = image
        self.masts = None
        self.called = False

    def read(self):
        # return the frame most recently read
        self.called = True
        return self.masts

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
