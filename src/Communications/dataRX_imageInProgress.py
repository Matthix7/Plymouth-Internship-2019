#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import serial
import time
import numpy as np
import cv2

# Around 140000/213100857 in 10s

def receiveImage(ser):
    tailleImage = ser.readline()[0:-1]
    tailleImage = int(tailleImage)

    sizeX = ser.readline()[0:-1]
    sizeX = int(sizeX)

    sizeY = ser.readline()[0:-1]
    sizeY = int(sizeY)

    rospy.loginfo("tailleImage %d\n", tailleImage)
    rospy.loginfo("tailleX %d\n", sizeX)
    rospy.loginfo("tailleY %d\n", sizeY)

    received = 0
    imageBytes = ''

    while received < tailleImage and not rospy.is_shutdown():
        imageBytes += ser.readline()[0:-1]
        received = len(imageBytes)
#        rospy.loginfo("received %d/%d", received, tailleImage)

    rospy.loginfo("received %d\n", received)

    return



def run():
    rospy.init_node('dataRX', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600)

    img = receiveImage(ser)
#    cv2.imshow('image',img)

    line = ''

    while not rospy.is_shutdown():
      line = ser.readline()   # read a '\n' terminated line
      line = line.decode()
      rospy.loginfo(rospy.get_caller_id() + "I heard %s", line)

      rate.sleep()
