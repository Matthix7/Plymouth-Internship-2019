#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu

import serial


###################################################################
#    To execute when a message to transmit is received
###################################################################


def targetTransmission(data):
    global targetString
    targetString = str(data.x)+','+str(data.y)+','+str(data.theta)

def modeTransmission(data):
    global modeString
    modeString = data.data



###################################################################
#    Check message validity
###################################################################

def is_valid(line):
    return len(line) > 2 and line[0] == '#' and line[-2] == '='




###################################################################
###################################################################
#    Main
###################################################################
###################################################################



def run():

###################################################################
#    Initialisation
###################################################################
    global targetString, modeString
    targetString, modeString = 'init', 'init'

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

        if is_valid(line):
            line = line[0:-1]
            line = line[0:-1]
            line = line.replace('#','')
            line = line.replace('=','')

            words = line.split()

            ID = int(words[-1])
            if ID not in connected:
                connected.append(ID)
                rospy.loginfo('|'+line+'|')

    ser.write('OK\n')
    rospy.loginfo("Got boats " + str(connected)+' connected\n')


###################################################################
#   Transmit useful data for control
# Frame emitted:
# "#####ID1_GPSstring1_poseString1_ID2_GPSstring2_poseString2_ID3_GPSstring3_poseString3_targetString_modeString=====\n"
###################################################################

#    Receives the data relative to the target point
#    (depends on controlMode, common to all boats)
    rospy.Subscriber('poseTarget', Pose2D, targetTransmission)

#    Receives the string indicator of the control mode
    rospy.Subscriber('controlMode', String, modeTransmission)





###################################################################
#   Receive useful data from the boats
# Frame received:
# "#####ID_GPSstring_poseString=====\n"
###################################################################

    received = ['init']*len(connected)
    emission = 0

    while not rospy.is_shutdown():
        emission += 1
        line = ser.readline()

        if is_valid(line):
            line = line[0:-1]
            line = line.replace('#','')
            line = line.replace('=','')

            ID = line.split('_')[0]

            received[ID] = line

        if emission%3 == 0:
            receivedLines = ''
            for line in received:
                receivedLines += line+'_'
            ser.write("#####"+receivedLines+targetString+'_'+modeString+"=====\n")















###################################################################
#   Deconnection signal
###################################################################
    rospy.loginfo("End mission, disconnection of all connected boats\n")
    ser.write('#####**********=====\n')
    ser.write('#####**********=====\n')
    ser.write('#####**********=====\n')
    ser.write('#####**********=====\n')
    ser.write('#####**********=====\n')

































