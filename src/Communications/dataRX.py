#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import serial
from std_msgs.msg import Float64


def run():
    rospy.init_node('dataRX', anonymous=True)
    pub = rospy.Publisher('dataReceived', Float64, queue_size = 10)

    rate = rospy.Rate(10) # 10hz

    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600)

    line = ''

    while not rospy.is_shutdown():
      line = ser.readline()[0:-1]   # read a '\n' terminated line
      line = line.decode()
      rospy.loginfo("I heard %s", line)

      data = line.split('_')
      servo1 = int(data[1])
      servo2 = int(data[3])

      rate.sleep()
