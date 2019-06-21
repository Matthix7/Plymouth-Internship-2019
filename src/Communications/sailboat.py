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
    a = (len(line) > 2)
    if a:
        b = (line[0] == '#')
        c = (line[-1] == '=')

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
    global GPSstring, poseString
    compteur = 0
    GPSstring, poseString = 'init', 'init'
    targetString, modeString = 'init', 'init'

    rospy.init_node('endPoint', anonymous=True)

    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600, timeout = 0.02)

    rate = rospy.Rate(50)


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
        msg = "Hello, I am Boat " + str(ID)

        size = str(len(msg)+4)
        for i in range(len(size),3):
            size = '0'+size

        msg = "#####"+size+'_'+msg+"=====\n"
        ser.write(msg)

    rospy.loginfo("Connected to Coordinator")


    line = 'init'



###################################################################
#   Transmit useful data to broadcast (max 999 char)
# Frame emitted:
# "#####msgSize_ID_GPSstring_poseString=====\n"
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
# "#####msgSize_ID1_GPSstring1_poseString1_ID2_GPSstring2_poseString2_ID3_GPSstring3_poseString3_targetString_modeString=====\n"
########################################################################################################################


    compteur = 0

    while not rospy.is_shutdown() and line not in '**********':
        c = ''
        line = ''


        while c != '#' and not rospy.is_shutdown():
            c = ser.read(1)
        while c != '=' and not rospy.is_shutdown():
            line += c
            c = ser.read(1)
        line += c


        check, msgReceived = is_valid(line)

        if check:
            rospy.loginfo("Read\n"+msgReceived+'\n')
            compteur += 1

        elif not check:
            rospy.loginfo("Could not read\n"+ '|'+line+'|\n')

        compteur += 1

        msg = str(ID)+'_'+GPSstring+'_'+poseString+'_'+str(compteur)
        size = str(len(msg)+4)
        for i in range(len(size),3):
            size = '0'+size

        msg = "#####"+size+'_'+msg+"=====\n"
        ser.write(msg)

        line = line.replace('#','')
        line = line.replace('=','')

        rate.sleep()







###################################################################
#   Error treatment and deconnection signal reception
###################################################################

        if line == '':
            line = 'error'

    rospy.loginfo("End mission\n")
    rospy.loginfo("Received"+str(compteur))














