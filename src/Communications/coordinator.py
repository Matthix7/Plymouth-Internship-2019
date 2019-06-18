#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

import serial




def run():

###################################################################
#    Initialisation
###################################################################
    rospy.init_node('coordinator', anonymous=True)

    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600, timeout = 0)



###################################################################
#    Get local XBee ID
#(especially important for sailboats, not coordinator)
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

    rospy.loginfo("\nHello,\nI am Coordinator " + str(ID)+'\n')

###################################################################
#    Look for the boats connected in the XBee network
###################################################################

    # To check we have all the boats connected
    connected = [] #list of connected boats IDs
    expected = 2  # Number of boats expected to connect

    while not rospy.is_shutdown() and len(connected) < expected:
        line = ser.readline()

        if line != '' and line[-1] == '\n':
            words = line.split()

            ID = int(words[-1])
            if ID not in connected:
                connected.append(ID)
                rospy.loginfo('|'+line[0:-1]+'|')

    ser.write('OK\n')
    rospy.loginfo("Got boats " + str(connected)+' connected\n')






































