#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String

import serial
import time
import cv2
import numpy as np




def run():
    rospy.init_node('coordinator', anonymous=True)

    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600, timeout = 0)

    # Enter XBee command mode
    ser.write('+++')
    rospy.sleep(1.2)
    ser.read(10)

    # Get local XBee adress
    ser.write('ATMY\r')
    rospy.sleep(0.1)
    ans = ser.read(10)
    ser.write('ATCN\r')

    rospy.loginfo("\nHello,\nI am Coordinator " + ans.split('\r')[0])

    while not rospy.is_shutdown() and ID < 3:
        line = ser.readline()   # read a '\n' terminated line
        words = line.split()
        ID = word[-1]
        rospy.loginfo(line)

































