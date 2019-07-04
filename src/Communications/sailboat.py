#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D, Vector3

import serial
from time import time


###################################################################
#    To execute when a message to transmit is received
###################################################################

def sub_GPS(data):
    global gpsString
    gpsString = data.data

def sub_WIND_FORCE(data):
    global windForceString
    windForceString = str(data.data)

def sub_WIND_DIRECTION(data):
    global windDirectionString
    windDirectionString = str(data.data)

def sub_EULER_ANGLES(data):
    global eulerAnglesString
    eulerAnglesString = str(data.x)+','+str(data.y)+','+str(data.z)

def sub_POS(data):
    global posString
    posString = str(data.x)+','+str(data.y)+','+str(data.theta)

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
    global gpsString, windForceString, windDirectionString, eulerAnglesString, posString

    GPSstring, poseString = 'init', 'init'

    rospy.init_node('endPoint', anonymous=True)

    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600, timeout = 0.02)

    receiving_freq = 6 #Equal to coordinator emission_freq
    rate = rospy.Rate(receiving_freq)

    force1, force2, force3             = Float32(), Float32(), Float32()
    direction1, direction2, dircetion3 = Float32(), Float32(), Float32()
    gps1, gps2, gps3                   = String(), String(),, String()
    euler1, euler2, euler3             = Vector3(),Vector3(),Vector3()
    pos1, pos2, pos3                   = Pose2D(), Pose2D(), Pose2D()

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
    rospy.Subscriber('gps', String, sub_GPS)

#    Receives the speed of the wind
    rospy.Subscriber('wind_force', Float32, sub_WIND_FORCE)

#    Receives the direction of the wind
    rospy.Subscriber('wind_direction', Float32, sub_WIND_DIRECTION)

#    Receives the direction of the wind
    rospy.Subscriber('euler_angles', Vector3, sub_EULER_ANGLES)

#    Receives the direction of the wind
    rospy.Subscriber('pos', Pose2D, sub_POS)



###################################################################
#   Transmit data from the other boats to the boat's controller (max 999 char)
###################################################################
#    Publishes the string indicator of the control mode
    pubControlMode = rospy.Publisher('controlMode', String, queue_size = 2)

#    Publishes the data relative to the target point
#    (depends on controlMode, common to all boats)
    pubTarget = rospy.Publisher('poseTarget', String, queue_size = 2)

#    Publishes the data relative to each boat
    pubBoat1 = rospy.Publisher('Boat1', Pose2D, queue_size = 2)
    pubBoat2 = rospy.Publisher('Boat2', Pose2D, queue_size = 2)
    pubBoat3 = rospy.Publisher('Boat3', Pose2D, queue_size = 2)


########################################################################################################################
#   Receive useful data from the coordinator
# Frame received:
# "#####msgSize_ID1_windForceString1_windDirectionString1_gpsString1_eulerAnglesString1_posString1_ID2_windForceString2_windDirectionString2_gpsString2_eulerAnglesString2_posString2_ID3_windForceString3_windDirectionString3_gpsString3_eulerAnglesString3_posString3_targetString_modeString=====\n"
########################################################################################################################

    compteur = 0
    emission = 0

    while not rospy.is_shutdown() and line not in '**********':
        c = ''
        line = ''
        loopTime = time()

        windForceString, windDirectionString ,gpsString, eulerAnglesString, posString = "nothing", "nothing", "nothing", "nothing", "nothing"


#    Read what is in the buffer, start and stop with specific signals.

        while c != '#' and not rospy.is_shutdown():
            c = ser.read(1)
        while c != '=' and not rospy.is_shutdown():
            line += c
            c = ser.read(1)
        line += c

#    Check message syntax and checkSum
        check, msgReceived = is_valid(line)

        if check:
            rospy.loginfo("Read\n"+msgReceived+'\n')
            compteur += 1
            cursor = 1


            try:
                data = msgReceived.split('_')

#Collect the data from boat 1

                if data[cursor+1] != "nothing":
                    ID1 = data[cursor]

                    force1.data = float(data[cursor+1])

                    direction1.data = float(data[cursor+2])

                    gps1.data = data[cursor+3]

                    tmpEuler = data[cursor+4].split(',')
                    euler1.x = tmpEuler[0]
                    euler1.y = tmpEuler[1]
                    euler1.z = tmpEuler[2]

                    tmpPos = data[cursor+5].split(',')
                    pos1.x = tmpPos[0]
                    pos1.y = tmpPos[1]
                    pos1.theta = tmpPos[2]

                cursor += 6

#Collect the data from boat 2

                if data[cursor+1] != "nothing":
                    ID2 = data[cursor]

                    force2.data = float(data[cursor+1])

                    direction2.data = float(data[cursor+2])

                    gps2.data = data[cursor+3]

                    tmpEuler = data[cursor+4].split(',')
                    euler2.x = tmpEuler[0]
                    euler2.y = tmpEuler[1]
                    euler2.z = tmpEuler[2]

                    tmpPos = data[cursor+5].split(',')
                    pos2.x = tmpPos[0]
                    pos2.y = tmpPos[1]
                    pos2.theta = tmpPos[2]

                cursor += 6

#Collect the data from boat 3

                if data[cursor+1] != "nothing":
                    ID3 = data[cursor]

                    force3.data = float(data[cursor+1])

                    direction3.data = float(data[cursor+2])

                    gps3.data = data[cursor+3]

                    tmpEuler = data[cursor+4].split(',')
                    euler3.x = tmpEuler[0]
                    euler3.y = tmpEuler[1]
                    euler3.z = tmpEuler[2]

                    tmpPos = data[cursor+5].split(',')
                    pos3.x = tmpPos[0]
                    pos3.y = tmpPos[1]
                    pos3.theta = tmpPos[2]

                cursor += 6

#Collect the data from the operator

                targetString.data = data[cursor]

                modeString.data = data[cursor+1]

#Publish the data for internal use

                pubControlMode.publish(modeString)
                pubTarget.publish(targetString)

                pubBoat1.publish(pose1)
                pubBoat2.publish(pose2)
                pubBoat3.publish(pose3)



#Despite message controls, some errors may still occur...
            except:
                pass


        elif not check:
            rospy.loginfo("Could not read\n"+ '|'+line+'|\n')



#Creating the core message
        msg = str(ID)+'_'+windForceString+'_'+windDirectionString+'_'+gpsString+'_'+eulerAnglesString+'_'+posString

#Generating the checkSum message control
        size = str(len(msg)+4)
        for i in range(len(size),3):
            size = '0'+size

        msg = "#####"+size+'_'+msg+"=====\n"

        processTime = time() - loopTime
#Sleep while others are talking
        rospy.sleep( ID/4. * ((1./receiving_freq) - processTime))

        ser.write(msg)
        emission += 1
        rospy.loginfo("Emission "+str(emission))

        line = line.replace('#','')
        line = line.replace('=','')








###################################################################
#   Treatment error and deconnection signal reception
###################################################################

        if line == '':
            line = 'error'

    rospy.loginfo("End mission\n")
    rospy.loginfo("Received "+str(compteur))














