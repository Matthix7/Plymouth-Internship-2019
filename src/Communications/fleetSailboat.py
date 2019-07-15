#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D, Vector3

import serial
from time import time

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
    global windForceString, windDirectionString, gpsString, eulerAnglesString, posString

    GPSstring, poseString, line = 'init', 'init', 'init'

    rospy.init_node('endPoint', anonymous=True)


    receiving_freq = 6 #Equal to coordinator emission_freq
    rate = rospy.Rate(receiving_freq)

    force1 = Float32()
    direction1 = Float32()
    gps1 = String()
    euler1 = Vector3()
    pos1 = Pose2D()

    boatData1 = [force1, direction1, gps1, euler1, pos1]


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

    rospy.loginfo('||'+fleetInitMessage+'||')
    fleetSize = int(fleetInitMessage.split()[0])
    boatsData = [boatData1]*fleetSize

    rospy.loginfo("Connected to Coordinator, with "+str(fleetSize-1)+" other sailboats.")


    tie.sleep(5)

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
    pub_send_wind_force_1 = rospy.Publisher('xbee_send_wind_force_1', Float32, queue_size = 2)
    pub_send_wind_force_2 = rospy.Publisher('xbee_send_wind_force_2', Float32, queue_size = 2)
    pub_send_wind_force_3 = rospy.Publisher('xbee_send_wind_force_3', Float32, queue_size = 2)

    pub_send_wind_direction_1 = rospy.Publisher('xbee_send_wind_direction_1', Float32, queue_size = 2)
    pub_send_wind_direction_2 = rospy.Publisher('xbee_send_wind_direction_2', Float32, queue_size = 2)
    pub_send_wind_direction_3 = rospy.Publisher('xbee_send_wind_direction_3', Float32, queue_size = 2)

    pub_send_gps_1 = rospy.Publisher('xbee_send_gps_1', String, queue_size = 2)
    pub_send_gps_2 = rospy.Publisher('xbee_send_gps_2', String, queue_size = 2)
    pub_send_gps_3 = rospy.Publisher('xbee_send_gps_3', String, queue_size = 2)

    pub_send_euler_1 = rospy.Publisher('xbee_send_euler_1', Vector3, queue_size = 2)
    pub_send_euler_2 = rospy.Publisher('xbee_send_euler_2', Vector3, queue_size = 2)
    pub_send_euler_3 = rospy.Publisher('xbee_send_euler_3', Vector3, queue_size = 2)

    pub_send_pos_1 = rospy.Publisher('xbee_send_pos_1', Pose2D, queue_size = 2)
    pub_send_pos_2 = rospy.Publisher('xbee_send_pos_2', Pose2D, queue_size = 2)
    pub_send_pos_3 = rospy.Publisher('xbee_send_pos_3', Pose2D, queue_size = 2)



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

        if check:

            compteur += 1
            cursor = 0
            try:
                data = msgReceived.split('_')
                rospy.loginfo("Read\n"+str(data[:6])+'\n'+str(data[6:12])+'\n'+str(data[12:18])+'\n'+str(data[18:20]))


    #Collect the data from boat 1

                if data[cursor] != "ID":
                    ID1 = data[cursor]

                    force1.data = float(data[cursor+1])

                    direction1.data = float(data[cursor+2])

                    gps1.data = data[cursor+3]

                    rospy.loginfo("TOTO = "+str(data[cursor+3]))

                    tmpEuler = data[cursor+4].split(',')
                    euler1.x = float(tmpEuler[0])
                    euler1.y = float(tmpEuler[1])
                    euler1.z = float(tmpEuler[2])

                    tmpPos = data[cursor+5].split(',')
                    pos1.x = float(tmpPos[0])
                    pos1.y = float(tmpPos[1])
                    pos1.theta = float(tmpPos[2])

                cursor += 6

    #Collect the data from boat 2

                if data[cursor] != "ID":
                    ID2 = data[cursor]

                    force2.data = float(data[cursor+1])

                    direction2.data = float(data[cursor+2])

                    gps2.data = data[cursor+3]

                    tmpEuler = data[cursor+4].split(',')
                    euler2.x = float(tmpEuler[0])
                    euler2.y = float(tmpEuler[1])
                    euler2.z = float(tmpEuler[2])

                    tmpPos = data[cursor+5].split(',')
                    pos2.x = float(tmpPos[0])
                    pos2.y = float(tmpPos[1])
                    pos2.theta = float(tmpPos[2])

                cursor += 6

    #Collect the data from boat 3

                if data[cursor] != "ID":
                    ID3 = data[cursor]

                    force3.data = float(data[cursor+1])

                    direction3.data = float(data[cursor+2])


                    gps3.data = data[cursor+3]

                    tmpEuler = data[cursor+4].split(',')
                    euler3.x = float(tmpEuler[0])
                    euler3.y = float(tmpEuler[1])
                    euler3.z = float(tmpEuler[2])

                    tmpPos = data[cursor+5].split(',')
                    pos3.x = float(tmpPos[0])
                    pos3.y = float(tmpPos[1])
                    pos3.theta = float(tmpPos[2])

                cursor += 6

    #Collect the data from the operator

                targetString = data[cursor]
                targetData = targetString.split(',')
                rudder = Float32(data = float(targetData[0]))
                sail = Float32(data = float(targetData[1]))

                mode.data = int(data[cursor+1])

    #Publish the data for internal use

                pub_send_control_mode.publish(mode)
                pub_send_u_rudder.publish(rudder)
                pub_send_u_sail.publish(sail)

                pub_send_wind_force_1.publish(force1)
                pub_send_wind_direction_1.publish(direction1)
                pub_send_gps_1.publish(gps1)
                pub_send_euler_1.publish(euler1)
                pub_send_pos_1.publish(pos1)

                pub_send_wind_force_2.publish(force2)
                pub_send_wind_direction_2.publish(direction2)
                pub_send_gps_2.publish(gps2)
                pub_send_euler_2.publish(euler2)
                pub_send_pos_2.publish(pos2)

                pub_send_wind_force_3.publish(force3)
                pub_send_wind_direction_3.publish(direction3)
                pub_send_gps_3.publish(gps3)
                pub_send_euler_3.publish(euler3)
                pub_send_pos_3.publish(pos3)

#Despite message controls, some errors may still occur...
            except:
#                pass
                rospy.loginfo("Oops! "+str(sys.exc_info()[0])+'\n'+str(sys.exc_info()[1])+"\noccured.")


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














