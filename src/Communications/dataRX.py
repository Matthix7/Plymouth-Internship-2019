#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import serial
import time
import numpy as np
import cv2


def run():
    rospy.init_node('dataRX', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600)

    line = ''

    while not rospy.is_shutdown():
      line = ser.readline()   # read a '\n' terminated line
      line = line.decode()
      rospy.loginfo(rospy.get_caller_id() + "I heard %s", line)

      rate.sleep()
