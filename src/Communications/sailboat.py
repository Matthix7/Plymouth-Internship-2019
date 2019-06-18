#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

import serial




def run():

###################################################################
#    Initialisation
###################################################################

    rospy.init_node('endPoint', anonymous=True)

    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600, timeout = 0)

###################################################################
#    Get local XBee ID and send it to coordinator
###################################################################

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


###################################################################
#   Wait until all boats are connected
###################################################################

    while not rospy.is_shutdown() and ser.readline() != 'OK\n':
        rospy.sleep(ID*0.2)
        ser.write("Hello, I am Boat " + str(ID)+'\n')

    rospy.loginfo("Connected to Coordinator")


















