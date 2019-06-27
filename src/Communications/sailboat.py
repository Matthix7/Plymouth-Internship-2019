#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu

import serial
from time import time


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

    GPSstring, poseString = 'init', 'init'

    rospy.init_node('endPoint', anonymous=True)

    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600, timeout = 0.02)

    receiving_freq = 6 #Equal to coordinator emission_freq
    rate = rospy.Rate(receiving_freq)

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
# "#####msgSize_ID1_GPSstring1_poseString1_ID2_GPSstring2_poseString2_ID3_GPSstring3_poseString3_targetString_modeString=====\n"
########################################################################################################################


    compteur = 0
    emission = 0

    while not rospy.is_shutdown() and line not in '**********':
        c = ''
        line = ''
        loopTime = time()


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
            cursor = 0

            pose1, pose2, pose3 = Pose2D(), Pose2D(), Pose2D()
            targetString, modeString = String(), String()

            try:
                data = msgReceived.split('_')

                if data[cursor] != "NODATA":
                    ID1 = data[1]
                    GPSstring1 = String(data = data[2])
                    poseData1 = data[3].split(',')
                    pose1.x = eval(poseData1[0])
                    pose1.y = eval(poseData1[1])
                    pose1.theta = eval(poseData1[2])
                    cursor += 3
                else:
                    cursor += 1

                if data[cursor] != "NODATA":
                    ID2 = data[cursor]
                    GPSstring2 = String(data = data[cursor+1])
                    poseData2 = data[cursor+2].split(',')
                    pose2.x = eval(poseData2[0])
                    pose2.y = eval(poseData2[1])
                    pose2.theta = eval(poseData2[2])
                    cursor += 3
                else:
                    cursor += 1

                if data[cursor] != "NODATA":
                    ID3 = data[cursor]
                    GPSstring3 = String(data = data[cursor+1])
                    poseData3 = data[cursor+2].split(',')
                    pose3.x = eval(poseData3[0])
                    pose3.y = eval(poseData3[1])
                    pose3.theta = eval(poseData3[2])
                    cursor += 3
                else:
                    cursor += 1


                targetString.data = data[cursor]

                modeString.data = data[cursor+1]


                pubControlMode.publish(modeString)
                pubTarget.publish(targetString)

                pubBoat1.publish(pose1)
                pubBoat2.publish(pose2)
                pubBoat3.publish(pose3)




            except:
                pass




        elif not check:
            rospy.loginfo("Could not read\n"+ '|'+line+'|\n')



        msg = str(ID)+'_'+GPSstring+'_'+poseString+str(compteur)
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














