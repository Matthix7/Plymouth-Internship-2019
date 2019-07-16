#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D, Vector3

import serial
from time import time, sleep

import pyudev
import sys

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
                    msg = msg[5:]
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
    global windForceString, windDirectionString, gpsString, eulerAnglesString, posString

    GPSstring, poseString, line = 'init', 'init', 'init'

    rospy.init_node('endPoint', anonymous=True)




    ID1 = -1
    force1 = Float32()
    direction1 = Float32()
    gps1 = String()
    euler1 = Vector3()
    pos1 = Pose2D()

    boatData1 = [ID1, force1, direction1, gps1, euler1, pos1]


    rudder, sail, mode = Float32(), Float32(), Float32()


###################################################################
#    Look for XBee USB port
###################################################################

    context = pyudev.Context()
    usbPort = 'No XBee found'

    for device in context.list_devices(subsystem='tty'):
        if 'ID_VENDOR' in device and device['ID_VENDOR'] == 'FTDI':
            usbPort = device['DEVNAME']

    ser = serial.Serial(usbPort,baudrate=57600, timeout = 0.02)



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
    fleetInitMessage = ser.readline()
    while not rospy.is_shutdown() and "Connected" not in fleetInitMessage:
        rospy.sleep(ID*0.2)
        msg = "Hello, I am Boat " + str(ID)

        size = str(len(msg)+4)
        for i in range(len(size),3):
            size = '0'+size

        msg = "#####"+size+'_'+msg+"=====\n"
        ser.write(msg)
        fleetInitMessage = ser.readline()

    fleetSize = int(fleetInitMessage.split()[0])
    boatsData = [boatData1]*fleetSize

    rospy.loginfo("Connected to Coordinator, with "+str(fleetSize-1)+" other sailboats.")

    receiving_freq = 1/fleetSize #Equal to coordinator emission_freq
    rate = rospy.Rate(receiving_freq)

    sleep(5)

###################################################################
#   Transmit useful data to broadcast (max 999 char)
# Frame emitted:
# "#####msgSize_ID_GPSstring_poseString=====\n"
###################################################################

#    Receives a GPS standard frame
#    Contains xBoat, yBoat and others
    rospy.Subscriber('filter_send_trame_gps', String, sub_GPS) #topic name to change

#    Receives the speed of the wind
    rospy.Subscriber('filter_send_wind_force', Float32, sub_WIND_FORCE)

#    Receives the direction of the wind
    rospy.Subscriber('filter_send_wind_direction', Float32, sub_WIND_DIRECTION)

#    Receives the direction of the wind
    rospy.Subscriber('filter_send_euler_angles', Vector3, sub_EULER_ANGLES)

#    Receives the direction of the wind
    rospy.Subscriber('pos', Pose2D, sub_POS) #topic name to change



###################################################################
#   Transmit data from the other boats to the boat's controller (max 999 char)
###################################################################
#    Publishes the string indicator of the control mode
    pub_send_control_mode = rospy.Publisher('xbee_send_mode', Float32, queue_size = 2)

#    Publishes the data relative to the target point
#    (depends on controlMode, common to all boats)
    pub_send_u_rudder = rospy.Publisher('xbee_send_u_rudder', Float32, queue_size = 2)
    pub_send_u_sail = rospy.Publisher('xbee_send_u_sail', Float32, queue_size = 2)

#    Publishes the data relative to each boat
# List of publishers, 5 publishers for each boat connected to the network

    boatsPublishers = []
    for boat in range(fleetSize):
        pubWindForceName = "xbee_send_wind_force_"+str(boat)
        pubWindDirName = "xbee_send_wind_direction_"+str(boat)
        pubGPSName = "xbee_send_gps_"+str(boat)
        pubEulerName = "xbee_send_euler_"+str(boat)
        pubPosName = "xbee_send_pos_"+str(boat)
        boatsPublishers.append([ rospy.Publisher(pubWindForceName, Float32, queue_size = 2),\
                                 rospy.Publisher(pubWindDirName, Float32, queue_size = 2),\
                                 rospy.Publisher(pubGPSName, String, queue_size = 2),\
                                 rospy.Publisher(pubEulerName, Vector3, queue_size = 2),\
                                 rospy.Publisher(pubPosName, Pose2D, queue_size = 2)])




########################################################################################################################
#   Receive useful data from the coordinator
# Frame received:
# "#####msgSize_ID1_windForceString1_windDirectionString1_gpsString1_eulerAnglesString1_posString1_ID2_windForceString2_windDirectionString2_gpsString2_eulerAnglesString2_posString2_ID3_windForceString3_windDirectionString3_gpsString3_eulerAnglesString3_posString3_targetString_mode=====\n"
########################################################################################################################

    compteur = 0
    emission = 0

    while not rospy.is_shutdown() and line not in '**********':
        c = ''
        line = ''
        loopTime = time()

        windForceString, windDirectionString ,gpsString, eulerAnglesString, posString = "-999", "-999", "nothing", "-999,-999,-999", "-999,-999,-999"


#    Read what is in the buffer, start and stop with specific signals.

        while c != '#' and not rospy.is_shutdown():
            c = ser.read(1)
        while c != '=' and not rospy.is_shutdown():
            line += c
            c = ser.read(1)
        line += c

#    Check message syntax and checkSum
        check, msgReceived = is_valid(line)
        rospy.loginfo("Received\n|" + msgReceived + '|')

        if check:

            compteur += 1
            cursor = 0
            try:
                data = msgReceived.split('_')
                data_log = "Read\n"
                for boat in range(fleetSize):
                    data_log += str(data[6*boat:6*(boat+1)])+'\n'
                data_log += str(data[-2:])
            except:
                rospy.loginfo("Oops! "+str(sys.exc_info()[0])+'\n'+str(sys.exc_info()[1])+"\noccured.")


    #Collect the data from boats
            for boat in range(fleetSize):
                try:
                    if data[cursor] != "ID":
                        boatsData[boat][0] = data[cursor]  #ID

                        boatsData[boat][1].data = float(data[cursor+1]) #Wind force

                        boatsData[boat][2].data = float(data[cursor+2]) #Wind direction

                        boatsData[boat][3].data = data[cursor+3] #GPS frame

                        tmpEuler = data[cursor+4].split(',')     #Euler angles
                        boatsData[boat][4].x = float(tmpEuler[0])
                        boatsData[boat][4].y = float(tmpEuler[1])
                        boatsData[boat][4].z = float(tmpEuler[2])

                        tmpPos = data[cursor+5].split(',')       #Position
                        boatsData[boat][5].x = float(tmpPos[0])
                        boatsData[boat][5].y = float(tmpPos[1])
                        boatsData[boat][5].theta = float(tmpPos[2])

                except:
                    rospy.loginfo("Oops! "+str(sys.exc_info()[0])+'\n====>'+str(sys.exc_info()[1]))

                cursor += 6

    #Collect the data from the operator

            targetString = data[cursor]
            targetData = targetString.split(',')
            rudder = Float32(data = float(targetData[0]))
            sail = Float32(data = float(targetData[1]))

            mode.data = int(data[cursor+1])

    #Organise data to create a link between topics names and boats IDs
            boatsData = sorted(boatsData, key = lambda dataList: int(dataList[0]))


    #Publish the data for internal use

            pub_send_control_mode.publish(mode)
            pub_send_u_rudder.publish(rudder)
            pub_send_u_sail.publish(sail)

            for boat in range(fleetSize):
                for msg_publisher in range(len(boatsPublishers[0])):
                    rospy.loginfo("Publisher :"+str(boatsPublishers[boat][msg_publisher])
                    rospy.loginfo("message :"+str(boatsData[boat][msg_publisher])
                    boatsPublishers[boat][msg_publisher].publish(boatsData[boat][msg_publisher])



        elif not check:
            rospy.loginfo("Could not read\n"+ '|'+line+'|\n')



#Creating the core message
        msg = str(ID)+'_'+windForceString+'_'+windDirectionString+'_'+gpsString+'_'+eulerAnglesString+'_'+posString

#Generating the checkSum message control
        size = str(len(msg)+5)
        for i in range(len(size),4):
            size = '0'+size

        msg = "#####"+size+'_'+msg+"=====\n"

        processTime = time() - loopTime
#Sleep while others are talking
        rospy.sleep( ID/(fleetSize+1) * ((1./receiving_freq) - processTime))

        ser.write(msg)
        rospy.loginfo("Emitted\n|" + msg + '|')
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














