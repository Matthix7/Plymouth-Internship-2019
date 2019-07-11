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


#from picamera.array import PiRGBArray
#from picamera import PiCamera


from detectionBuoy import detectBuoy, getColorRange, getColorRangeTest
from detectionHorizonMast import horizonArea, detectMast
from detectionAruco import detectAruco

#from camThread import PiVideoStream
from buoyArucoThread import BAdetector
from horizonMastsThread import HMdetector




def run():

    t0 = time.time()
    Tframe, T1, T2, T3, T4, T5, T6 = [], [], [], [], [], [], []
    tframe = 0


#    cv2.namedWindow('Global', cv2.WINDOW_NORMAL)
#    cv2.namedWindow('Horizon', cv2.WINDOW_NORMAL)

    hm = HMdetector()
    hm.start()

    ba = BAdetector()
    ba.start()

    c = 0

##############          VIDEO           #############################
#####################################################################
#    Running on test video
    cap = cv2.VideoCapture('testImages/PlymouthSound.mp4')

    t0 = time.time()

    dodo = 0

    while(cap.isOpened()):

        # Capture frame-by-frame
        ret, image = cap.read()

        if not ret:
            break

        image = cv2.resize(image, (640,480))



##################     CAMERA     ####################################
######################################################################
##    Running with the camera

#    vs = PiVideoStream().start()
#    time.sleep(2)

#    dodo = 0


#    while True:#c<20:

#        image = vs.read()



######################################################################
######################################################################

        if tframe != 0:
            Tframe.append(time.time()-tframe-dodo)
        tframe = time.time()

        c += 1


        t1 = time.time()

        hm.newImage(image)
        ba.newImage(image)
        time.sleep(0.01)

        masts = hm.read()
        frame_markers, corners, center = ba.read()

        while  masts is None: # orframe_markers is None
#            time.sleep(0.01)
            if masts is None:
                masts = hm.read()

#            if frame_markers is None:
#                frame_markers, corners, center = ba.read()

        T1.append(time.time()-t1)


        t5 = time.time()
#        cv2.imshow('Horizon', masts)
        cv2.imshow('Global', frame_markers)
        T5.append(time.time()-t5)


        time.sleep(dodo)

#####################################################################
#############        INTERACTION          ###########################

        t6 = time.time()
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
                break

        elif key == 32:
            key = cv2.waitKey(1) & 0xFF
            while key != 32:
                key = cv2.waitKey(1) & 0xFF

        elif key == ord('c'):
            cv2.imwrite('sample.png',masts)
            print("Picture saved")

        T6.append(time.time()-t6)

    try:
        cap.release()
    except:
        vs.stop()

    cv2.destroyAllWindows()
    hm.stop()
    ba.stop()

    print("Total time : ",time.time()-t0)
    print("Computed frames : ", c)
    print("Global time per frame : ", (time.time()-t0)/c - dodo)
    print("Time detectors : ", np.mean(T1))
    print("Time display : ", np.mean(T5))
    print("Time interact: ", np.mean(T6))
    print("Time per frame accurate: ", np.mean(Tframe))


if __name__ == "__main__":
    run()
