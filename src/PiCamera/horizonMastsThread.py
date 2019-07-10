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
        self.processComplete = True
        self.stopped = False
        print('Initialized')

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.detect, args=()).start()
        print('Started')
        return self

    def detect(self):
        while not self.stopped:
            if self.frame is not None and self.processComplete:

                self.processComplete = False
                print(' Begin proccessing')
                horizon, horizon_height, self.horizon_prev = horizonArea(self.frame, self.horizon_prev)

                self.masts = detectMast(horizon, horizon_height)
                print('Processed')



    def newImage(self,image):
        self.frame = image
        self.masts = None
        self.processComplete = True
        print('newImage')

    def read(self):
        print('Called')
        while self.masts is None:
            pass
        print('Got Masts')
        return self.masts


    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
