#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import serial
from plymouth_internship_2019.msg import KeyboardServoCommand


def run():
    rospy.init_node('dataRX', anonymous=True)
    pub = rospy.Publisher('dataReceived', KeyboardServoCommand, queue_size = 10)

    rate = rospy.Rate(10) # 10hz

    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600, timeout = 0)

    line = ''

    while not rospy.is_shutdown():
      line = ser.readline()[0:-1]   # read a '\n' terminated line
      line = line.decode()
      rospy.loginfo("I heard %s", line)

      data = line.split('_')
      servo1 = int(data[1])
      servo2 = int(data[3])

      servoCommand = KeyboardServoCommand()
      servoCommand.servo_command_1 = servo1
      servoCommand.servo_command_2 = servo2

      pub.publish(servoCommand)


      ser.write("Boat heard "+str(servo1)+str(servo2)+'\n')

      rate.sleep()
