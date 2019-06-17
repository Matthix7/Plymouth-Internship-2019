#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import serial
from std_msgs.msg import Float32


def run():
    rospy.init_node('dataRX', anonymous=True)
    pub1 = rospy.Publisher('cmdRudder', Float32, queue_size = 10)
    pub2 = rospy.Publisher('cmdSail', Float32, queue_size = 10)

    rate = rospy.Rate(150)

    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600, timeout = 0)

    line = ''
    cnt = 0

    while not rospy.is_shutdown():
      line = ser.readline()   # read a '\n' terminated line
      line = line.decode()
      data = line.split('_')

      if line != '' and line[-1] == '\n' and len(data) == 4:
          rospy.loginfo("I heard %s", line[0:-1])

          try :
              servo1 = Float32()
	      servo1.data = eval(data[1])
              servo2 = Float32()
	      servo2.data = eval(data[3])
              pub1.publish(servo1)
              pub2.publish(servo2)

              cnt += 1
#              ser.write("Boat heard "+str(servo1.data)+'  '+str(servo2.data)+'\n')

          except:
              pass
      
      rate.sleep()

    ser.write("Boat heard "+str(cnt) +'commands \n')
















