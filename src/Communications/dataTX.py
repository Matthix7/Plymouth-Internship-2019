#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

import serial
import time
import cv2
import numpy as np


ser = serial.Serial("/dev/ttyUSB0",baudrate=57600)



def callback(data):
    rospy.loginfo("I heard and transmit %s", data.data)
    ser.write((data.data+'\n').encode())



def run():
    rospy.init_node('dataTX', anonymous=True)
    rospy.Subscriber("dataToSend", String, callback)

    rospy.loginfo(rospy.get_caller_id() + "\nHello")

    rospy.spin()







