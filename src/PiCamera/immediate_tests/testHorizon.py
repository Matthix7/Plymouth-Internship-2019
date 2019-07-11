#!/usr/bin/env python
# -*- coding: utf-8 -*-

##import rospy

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np


def run():
    cv2.namedWindow('Webcam', cv2.WINDOW_AUTOSIZE)
    image = cv2.imread('testImages/brave.jpg')
    image = cv2.resize(image, (640,480))

    




    while True:

        cv2.imshow('Webcam', image)

        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q") or key == 27:
            break

        # if the SPACE key was pressed, take a picture
        elif key == 32:
            c += 1
            cv2.imwrite('fra%i.png'%c,image)
            print("Picture saved")

    cv2.destroyAllWindows()




if __name__ == "__main__":
    run()
