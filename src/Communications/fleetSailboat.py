#!/usr/bin/env python
# -*- coding: utf-8 -*-


###################################################################################################
###################################################################################################
#####################################     OVERVIEW     ############################################
###################################################################################################
###################################################################################################

#The purpose of this file is to manage the communications between the sailboat where it is launched
#and the remote control computer (Coordinator), which itself broadcasts data to all the sailboats
#that are connected to the same XBee network. Therefore, each boat of the network may have access
#to the data relative to each other sailboat, while they are not too far from the Coordinator.

#Stage 1: After the Coordinator has been launched (to be done on the remote computer),
#        the sailboat identifies its Xbee device, connects itself to the XBee network and waits
#        for all other sailboats to be connected (fleet size to be managed in the Coordinator
#        file before starting). Once all are connected, the sailboat receives the fleet size and
#        goes to next stage.

#Stage 2: The sailboat prepares the ROS topics that will be used to make transmissions part more
#        convenient for downstream use.
#        - 5 subscribers that receive wind direction, wind speed, gps frame, euler angles and
#          position in a local coordinate system.
#        - 5 publishers per boat connected in the network, that communicate the data above for
#          each of them.
#        - 2 publishers that communicate the data coming from the operator:
#            ->one giving the control mode (ie automatic, keyboard control, other...)
#            ->one giving the commands necessary for this control mode

#Stage 3: Synchronised transmission begins. The sailboat first waits to receive a message from
#        the Coordinator, then reads it (sends in publishers) and waits for its time to talk
#        (to avoid simultaneous talks), and eventually sends the message containing all the data
#        from the subscribers.

#Stage 4: The transmission loop ends when the Coordinator is shut down: it sends a shutdown signal
#        that makes the loop end on the sailboats' side.


#Non default dependences: rospy, pyudev

###################################################################################################
###################################################################################################
###################################################################################################
###################################################################################################



import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D, Vector3

import serial
from time import time, sleep

import pyudev
import sys

###################################################################################################
#    To execute when a message to transmit is received by the subscribers.
###################################################################################################

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


###################################################################################################
#    Check message validity and clean it for downstream use (remove begin and end signals + size)
###################################################################################################

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



###################################################################################################
###################################################################################################
#    Main
###################################################################################################
###################################################################################################



def run():

    emission_freq = 0.3 #Equal to coordinator receiving_freq

###################################################################################################
#    Initialisation
###################################################################################################
    #Variables storing the data received by the subscribers
    global windForceString, windDirectionString, gpsString, eulerAnglesString, posString

    line = 'init' #Will store the string from serial read, ie coming from XBee

    dictLink = {i:i for i in range(30)}

    #Initialisationof the ROS node, endPoint refers to XBee network structure
    rospy.init_node('endPoint', anonymous=True)

    #The data for each boat in the network will be stored in this kind of variables
    ID1 = -1
    force1 = Float32()
    direction1 = Float32()
    gps1 = String()
    euler1 = Vector3()
    pos1 = Pose2D()

    boatData1 = [ID1, force1, direction1, gps1, euler1, pos1] #For one boat

    # Commands coming from the operator (for keyboard control and other control modes)
    rudder, sail, mode = Float32(), Float32(), Float32()


###################################################################################################
#    Look for XBee USB port, to avoid conflicts with other USB devices
###################################################################################################

    context = pyudev.Context()
    usbPort = 'No XBee found'

    for device in context.list_devices(subsystem='tty'):
        if 'ID_VENDOR' in device and device['ID_VENDOR'] == 'FTDI':
            usbPort = device['DEVNAME']

    #Initialise data transmission with the XBee
    ser = serial.Serial(usbPort,baudrate=57600, timeout = 0.02)



###################################################################################################
#    Get local XBee ID and send it to coordinator
###################################################################################################

    # Enter XBee "command mode"
    ser.write('+++')
    rospy.sleep(1.2)
    ser.read(10)

    # Get local XBee ID
    ser.write('ATMY\r')
    rospy.sleep(0.1)
    ans = ser.read(10)
    ID = eval(ans.split('\r')[0])

    # Exit XBee "command mode"
    ser.write('ATCN\r')



###################################################################################################
#   Wait until all boats are connected
###################################################################################################

    #When all expected boats are connected, the Coordinator sends a message with the fleet size
    #In the meanwhile, spam the Coodinator to be connected
    fleetInitMessage = ser.readline()

    while not rospy.is_shutdown() and "Connected" not in fleetInitMessage:
        rospy.sleep(ID*0.2) #Try avoiding simultaneous talks
        msg = "Hello, I am Boat " + str(ID) #Connection message

        #Transmission structure, details in the transmission loop part below
        size = str(len(msg)+5)
        for i in range(len(size),4):
            size = '0'+size

        msg = "#####"+size+'_'+msg+"=====\n"
        ser.write(msg)
        fleetInitMessage = ser.readline()

    #Every sailboat is connected, Coordinator sent the message
    fleetSize = int(fleetInitMessage.split('_')[0])
    fleetIDs = eval(fleetInitMessage.split('_')[1])
    dictLink = {fleetIDs[i]:(i+1) for i in range(len(fleetIDs))} #give local minimal IDs to the connected boats

    #Create the data stroing structure
    boatsData = [boatData1]*fleetSize

    #Print for check
    rospy.loginfo("Connected to Coordinator, with "+str(fleetSize-1)+" other sailboats.")
    rospy.loginfo("Other sailboats are "+str(fleetIDs))

    #Tell the frequency with which the Coordinator sends messages
    receiving_freq = emission_freq/fleetSize #Equal to coordinator emission_freq
    rate = rospy.Rate(receiving_freq)

    sleep(5)

###################################################################################################
#Subscribe to the topics that send the data to communicate to the other sailboats.
#This data comes from the sailboat's measuring systems and can be useful for the other boats
###################################################################################################

#    Receives a GPS standard frame
#    Contains xBoat, yBoat and others
    rospy.Subscriber('filter_send_trame_gps', String, sub_GPS)

#    Receives the speed of the wind
    rospy.Subscriber('filter_send_wind_force', Float32, sub_WIND_FORCE)

#    Receives the direction of the wind
    rospy.Subscriber('filter_send_wind_direction', Float32, sub_WIND_DIRECTION)

#    Receives the direction of the wind
    rospy.Subscriber('filter_send_euler_angles', Vector3, sub_EULER_ANGLES)

#    Receives the direction of the wind
    rospy.Subscriber('pos', Pose2D, sub_POS) #topic name to change



###################################################################################################
#Subscribe to the topics that send the data to communicate to the sailboat's controllers.
#This data comes from the other sailboats' measuring systems.
###################################################################################################

#    Publishes the string that indicates the control mode
    pub_send_control_mode = rospy.Publisher('xbee_send_mode', Float32, queue_size = 2)

#    Publishes the data relative to the target point
#    (depends on controlMode, common to all boats)
    pub_send_u_rudder = rospy.Publisher('xbee_send_u_rudder', Float32, queue_size = 2)
    pub_send_u_sail = rospy.Publisher('xbee_send_u_sail', Float32, queue_size = 2)

#    Publishes the data relative to each boat
# List of publishers, 5 publishers for each boat connected to the network

    boatsPublishers = []
    for boat in range(fleetSize):
        pubWindForceName = "xbee_send_wind_force_"+str(boat+1)
        pubWindDirName = "xbee_send_wind_direction_"+str(boat+1)
        pubGPSName = "xbee_send_gps_"+str(boat+1)
        pubEulerName = "xbee_send_euler_"+str(boat+1)
        pubPosName = "xbee_send_pos_"+str(boat+1)
        boatsPublishers.append([ rospy.Publisher(pubWindForceName, Float32, queue_size = 2),\
                                 rospy.Publisher(pubWindDirName, Float32, queue_size = 2),\
                                 rospy.Publisher(pubGPSName, String, queue_size = 2),\
                                 rospy.Publisher(pubEulerName, Vector3, queue_size = 2),\
                                 rospy.Publisher(pubPosName, Pose2D, queue_size = 2)])




###################################################################################################
# Transmission Loop
###################################################################################################

    #For statistics
    compteur = 0
    emission = 0

    #Message begin signal: '#####'
    #Message end signal: '=====\n'
    #Shutdown signal: '#####**********=====\n', so '**********' when cleaned.

    while not rospy.is_shutdown() and line not in '**********':

        #Re-initialise variables used in the loop to avoid communicating outdated data if one other boat is disconnected.
        c = ''
        line = ''
        loopTime = time()

        windForceString, windDirectionString ,gpsString, eulerAnglesString, posString = "-999", "-999", "nothing", "-999,-999,-999", "-999,-999,-999"

##########################################################################################################################################
# Receive useful data from the coordinator
# Frame received:
# "#####msgSize_ID1_windForceString1_windDirectionString1_gpsString1_eulerAnglesString1_posString1_ID2_..._targetString_modeString=====\n"
##########################################################################################################################################

        # Read what is in the buffer, start and stop with specific signals.
        # More reliable than ser.readline() for big message.

        while c != '#' and not rospy.is_shutdown():
            c = ser.read(1)
        while c != '=' and not rospy.is_shutdown():
            line += c
            c = ser.read(1)
        line += c

        # Check message syntax and checkSum and clean the message to use only the useful data
        check, msgReceived = is_valid(line)
#        rospy.loginfo("Received\n|" + msgReceived + '|')

        if check:

            compteur += 1
            cursor = 0

            #Organise the data by separating the data from each boat and the data from the operator
            try:
                data = msgReceived.split('_')
                data_log = "Read\n"
                for boat in range(fleetSize):
                    data_log += str(data[6*boat:6*(boat+1)])+'\n'
                data_log += str(data[-2:])
                rospy.loginfo(data_log)
            except:
                rospy.loginfo("Oops! "+str(sys.exc_info()[0])+'\n'+str(sys.exc_info()[1])+"\noccured.")


            #Collect the data from boats and store it the corresponding variables
            #that will be sent by the publishers.
            for boat in range(fleetSize):
                try:
                    #For one boat
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
                #Go to next data set (next boat or operator)
                cursor += 6


            #Collect the data from the operator and store it the corresponding variables
            #that will be sent by the publishers.

            targetString = data[cursor]
            targetData = targetString.split(',')
            rudder = Float32(data = float(targetData[0]))
            sail = Float32(data = float(targetData[1]))

            mode.data = int(data[cursor+1])

            #Organise data to create a link between topics names and boats IDs
            #By doing this, you can be sure that the data relative to one boat
            #will always be published in the same publishers, allowing therefore
            #to keep a track of each boat.
            boatsData = sorted(boatsData, key = lambda dataList: int(dataList[0]))


            #Publish the data for internal use (controllers, kalman filters, ...)

            pub_send_control_mode.publish(mode)
            pub_send_u_rudder.publish(rudder)
            pub_send_u_sail.publish(sail)

            for boat in range(fleetSize):
                for msg_publisher in range(len(boatsPublishers[0])):
                    boatsPublishers[boat][msg_publisher].publish(boatsData[boat][msg_publisher+1])



        elif not check:
            rospy.loginfo("Could not read\n"+ '|'+line+'|\n')


####################################################################################################
# Send useful data to the coordinator
# Frame emitted:
# "#####msgSize_ID_windForceString_windDirectionString_gpsString_eulerAnglesString_posString=====\n"
####################################################################################################


        #Creating the core message
        msg = str(ID)+'_'+windForceString+'_'+windDirectionString+'_'+gpsString+'_'+eulerAnglesString+'_'+posString

        #Generating the checkSum message control
        size = str(len(msg)+5)
        for i in range(len(size),4):
            size = '0'+size

        msg = "#####"+size+'_'+msg+"=====\n"

        processTime = time() - loopTime

        #Sleep while others are talking
        rospy.sleep( dictLink[ID]/emission_freq - processTime))

        #Emit the message
        ser.write(msg)

        rospy.loginfo("Emitted\n|" + msg + '|')
        emission += 1
        rospy.loginfo("Emission "+str(emission))

        #Clean the line to check wether it corresponds to the shutdown signal.
        line = line.replace('#','')
        line = line.replace('=','')








###################################################################################################
#   Treatment error and deconnection signal reception
###################################################################################################

        if line == '':
            line = 'error'

    #print the statistics
    rospy.loginfo("End mission\n")
    rospy.loginfo("Received "+str(compteur))














