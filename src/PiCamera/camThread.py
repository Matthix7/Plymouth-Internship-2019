#!/usr/bin/env python
# -*- coding: utf-8 -*-

####### USING https://www.pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and-opencv/ ###########


# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import cv2
import time
from numpy import array, tan

from chessboard_calibration import getCamDistortData



class PiVideoStream:
    def __init__(self, resolution=(640, 480), framerate=15, mode = 'sports', record = False):
        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.camera.exposure_mode = 'off'

        self.scale_factor = tan(0.398)/(resolution[0]*203.55/640)   #Camera scale factor, rad/pixel

        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,format="bgr", use_video_port=True)

        # Read the camera matrix from calibration file
        self.calibration_matrix, self.calibration_dist = getCamDistortData('~/workspaceRos/src/plymouth-internship-2019/src/PiCamera/calibration_data.txt')

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False

        # allow the camera to warmup
        time.sleep(0.1)

        self.record = record
        if record:
            # Define the codec and create VideoWriter object
            self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.out = cv2.VideoWriter('~/workspaceRos/src/plymouth-internship-2019/missionRecord/Mission_'+time.strftime('%c')+'(without calibration).avi',self.fourcc, framerate, resolution)


    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            if self.record:
                self.out.write(f.array)

            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = cv2.undistort(f.array, self.calibration_matrix, self.calibration_dist, None)
            self.rawCapture.truncate(0)

            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                if self.record:
                    self.out.release()
                return

    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

    def getScaleFactor(self):
        return self.scale_factor, self.camera.resolution







