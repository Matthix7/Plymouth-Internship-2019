#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String

import serial
import time
import cv2
import numpy as np




def run():
    rospy.init_node('endPoint', anonymous=True)

    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600, timeout = 0)

    # Enter XBee command mode
    ser.write('+++')
    rospy.sleep(1.2)
    ser.read(10)

    # Get local XBee adress
    ser.write('ATMY\r')
    rospy.sleep(0.1)
    ans = ser.read(10)
    ID = eval(ans.split('\r')[0])
    ser.write('ATCN\r')

    rospy.sleep(ID*0.5)
    ser.write("\nHello,\nI am Boat " + str(ID)+'\n')

















