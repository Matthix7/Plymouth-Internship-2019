#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu

import serial
from time import time

import pyudev


###################################################################
#    To execute when a message to transmit is received
###################################################################


def targetTransmission(data):
    global targetString
    targetString = data.data

def modeTransmission(data):
    global modeString
    modeString = data.data



###################################################################
#    Check message validity
###################################################################

def is_valid(line):
    a = (len(line) > 2)
    if a:
        b = (line[0] == '#')
        c = (line[-2] == '=')

        if b and c:
            msg = line[0:-1]
            msg = msg.replace('#','')
            msg = msg.replace('=','')

            try :
                size = int(msg.split('_')[0])

                if size == len(msg):
                    msg = msg[4:]
                    return True, msg
                else:
                    return False, ''

            except:
                return False, ''

        else:
            return False, ''
    else:
        return False, ''



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

    emission_freq = 6
    rate = rospy.Rate(3*emission_freq)

    compteur = 0

###################################################################
#    Look for XBee USB port
###################################################################

    context = pyudev.Context()
    usbPort = 'No XBee found'

    for device in context.list_devices(subsystem='tty'):
        if 'ID_VENDOR' in device and device['ID_VENDOR'] == 'FTDI':
            usbPort = device['DEVNAME']

    ser = serial.Serial(usbPort,baudrate=57600, timeout = 1/(2.5*emission_freq))


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
    expected = 1  # Number of boats expected to connect

    while not rospy.is_shutdown() and len(connected) < expected:
        line = ser.readline()

        check, msgReceived = is_valid(line)

        if check:
            words = msgReceived.split()
            IDboat = int(words[-1])

            if IDboat not in connected:
                connected.append(IDboat)
                rospy.loginfo('|'+msgReceived+'|')

    ser.write('OK\n')
    rospy.loginfo("Got boats " + str(connected)+' connected\n')


###################################################################
#   Transmit useful data for control (max 999 char)
# Frame emitted:
# "#####msgSize_ID1_GPSstring1_poseString1_ID2_GPSstring2_poseString2_ID3_GPSstring3_poseString3_targetString_modeString=====\n"
###################################################################

#    Receives the data relative to the target point
#    (depends on controlMode, common to all boats)
    rospy.Subscriber('commands', String, targetTransmission)

#    Receives the string indicator of the control mode
    rospy.Subscriber('controlMode', String, modeTransmission)





###################################################################
#   Receive useful data from the boats
# Frame received:
# "#####msgSize_ID_GPSstring_poseString=====\n"
###################################################################

    emission = 0
    received = ['ID_nothing_nothing_nothing_nothing_nothing']*3


    while not rospy.is_shutdown():
        emission += 1

        line = ser.readline()

#        rospy.loginfo(line)
        check, msgReceived = is_valid(line)

        if check:
            rospy.loginfo(msgReceived)
            compteur += 1

            try:
                IDboat = int(msgReceived.split('_')[0])

                received[IDboat-1] = msgReceived
            except:
                pass

        if not check:
            rospy.loginfo("Could not read\n"+line)



        if emission%3 == 0:
            receivedLines = ''
            for line in received:
                receivedLines += line+'_'

            msg = receivedLines+targetString+'_'+modeString

            size = str(len(msg)+4)
            for i in range(len(size),3):
                size = '0'+size

            msg = "#####"+size+'_'+msg+"=====\n"
            ser.write(msg)

            received = ['ID_nothing_nothing_nothing_nothing_nothing']*3


            rospy.loginfo("Emission " + str(emission//3))


        rate.sleep()











###################################################################
#   Deconnection signal
###################################################################
    rospy.loginfo("End mission, disconnection of all connected boats\n")
    ser.write('#####**********=====\n')
    ser.write('#####**********=====\n')
    rospy.sleep(1.)
    ser.write('#####**********=====\n')
    ser.write('#####**********=====\n')
    ser.write('#####**********=====\n')

    rospy.loginfo("Received"+str(compteur-2))































