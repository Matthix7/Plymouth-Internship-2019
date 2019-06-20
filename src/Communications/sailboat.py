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


def GPStransmission(data):
    global GPSstring
    GPSstring = data.data

def poseTransmission(data):
    global poseString
    poseString = str(data.x)+','+str(data.y)+','+str(data.theta)



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
    global GPSstring, poseString
    GPSstring, poseString = 'init', 'init'
    targetString, modeString = 'init', 'init'

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
        ser.write("'#####Hello, I am Boat " + str(ID)+'=====\n')

    rospy.loginfo("Connected to Coordinator")


    line = 'init'



###################################################################
#   Transmit useful data to broadcast
# Frame emitted:
# "#####ID_GPSstring_poseString=====\n"
###################################################################

#    Receives a GPS standard frame
#    Contains xBoat, yBoat and others
    rospy.Subscriber('GPS', String, GPStransmission)

#    Receives the rest of the data relative to the boat
#    msg.x = headingBoat
#    msg.y = headingWindBoat
#    msg.z = speedWindBoat
    rospy.Subscriber('poseBoat', Pose2D, poseTransmission)


########################################################################################################################
#   Receive useful data from the coordinator
# Frame received:
# "#####ID1_GPSstring1_poseString1_ID2_GPSstring2_poseString2_ID3_GPSstring3_poseString3_targetString_modeString=====\n"
########################################################################################################################




    while not rospy.is_shutdown() and line not in '**********':
        line = ser.readline()

        if is_valid(line):
            line = line[0:-1]
            line = line.replace('#','')
            line = line.replace('=','')

            rospy.loginfo(line)










###################################################################
#   Error treatment and deconnection signal reception
###################################################################

        if line == '':
            line = 'error'

    rospy.loginfo("End mission\n")














