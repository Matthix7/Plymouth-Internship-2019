#!/usr/bin/env python
# -*- coding: utf-8 -*-


###################################################################
#    Program that allows sending commands from the keyboard.
# Once launched, monitors your keyboard. Each pressed key, whatever
# window you are working in, will trigger program on_press.
# Pressing f1, f2, ...f[id] (according to the XBee ID of the wanted
# sailboat) will allow you to remote control a sailboat.
# You can control up to 2 sailboats at a time using
# WASD and Arrows.

# Press Esc to have all boats back to automatic mode,
# or f[id] to stop control of a remote-controlled boat.

# Command lines: you can send command lines to a specific boat
# or as broadcast command.
# To do so, press "insert". A window will pop up, in which you can
# prompt your command line.
# 1st param should be the receiver: id of the boat or 'all'.
# then the command line you want to be executed as if in the
# terminal of the receiver.
# Finally you can specify a '--relaunch' param that will close
# every running node (expect ros specific ones and fleetSailboat)
# before running your command.

# There is also a 'kill' command to close specific nodes.
# Use: "[receiver] kill [nodeName]"
# Program will close nodes containing nodeName as a substring.
# use '-a' to close all (expect ros specific ones and fleetSailboat)

# WARNING: ONLY ONE KEY CAN BE DETECTED AT A TIME.

# Special dependencies: rospy, pynput, pyautogui
###################################################################


import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import Imu

from numpy import pi, sign

from pynput import keyboard
from pynput.keyboard import Key

import pyautogui as pg

from time import time, sleep
import subprocess as s



def on_press(key):
    global dictMode, dictCommands, controlled, connected, padsUseDict, keyboardMode, dictLastMove, end
    if connected != []:
    ###################################################################
    #    Program executed each time a key is pressed on the keyboard.
    ###################################################################


    ###################################################################
    #    Useful variables
    ###################################################################
        if end:
            return False

        fList = [Key.f1, Key.f2, Key.f3, Key.f4, Key.f5, Key.f6, Key.f7, Key.f8, Key.f9, Key.f10, Key.f11, Key.f12]

    ##################################################################################################
    # Create a common variable type for all pads (WASD (ZQSD if French), Arrows, maybe more incoming)
    ##################################################################################################

        try:
            if key == Key.up:
                keyString = 'up'
            elif key == Key.down:
                keyString = 'down'
            elif key == Key.left:
                keyString = 'left'
            elif key == Key.right:
                keyString = 'right'
            else:
                keyString = key.char
        except:
            keyString = None


    ##################################################################################################
    # Set the pads in accordance with keyboard used
    ##################################################################################################

        if keyboardMode == 'azerty':
            WASD = ['z', 'q', 's', 'd']
        else:
            WASD = ['w', 'a', 's', 'd']

        ARROWS = ['up', 'left', 'down', 'right']

        padComponents = {'WASD':WASD, 'Arrows':ARROWS}


    ##################################################################################################
    # Reset command = press Esc
    ##################################################################################################

        if key == Key.esc:
                rospy.loginfo("Reinitialising all sailboats to automatic mode.")
                dictMode = {boat:0 for boat in connected}
                dictLastMove = {boat:time() for boat in connected}
                dictCommands = {boat:(initRudder,initSail) for boat in connected}
                padsUseDict = {boat:None for boat in connected}
                padsUseDict['WASD']=0
                padsUseDict['Arrows']=0
                controlled = []


    ##################################################################################################
    # Pressing insert will allow you to type a terminal command for the sailboat.
    # Ex: Press Insert, then type 'roslaunch plymouth_internship_2019 imageProcessing'
    # to launch the corresponding node.
    ##################################################################################################

        if key == Key.insert or key == Key.end:
                rospy.loginfo("Type a command to execute. \n  'all [command]' to apply to all sailboats, else '[id] [command]'.")
                userInput = pg.prompt(text='Usage: "[receiver] [terminal-style command] [--option] \n \
                \nExample1: "1 roslaunch myPackage myFile.launch --relaunch\
                \nExample2: "all kill partOfMyNodeName"', title='User input' , default='all roslaunch ')

                try:
                    if userInput is not None:
                        rospy.loginfo("You typed '"+userInput+"'.")
                        if userInput.split()[0] == 'all':
                            toLaunch = connected
                        else:
                            try:
                                if int(userInput.split()[0]) in connected:
                                    toLaunch = [eval(userInput.split()[0])]
                                else:
                                    toLaunch = []
                                    rospy.loginfo("You tried to send a command to a boat that is not part of the fleet.")
                            except:
                                toLaunch = []
                                rospy.loginfo("Make sure you typed correctly the description of the receiver.")

                        for boat in toLaunch:
                            dictMode[boat] = 2
                            dictCommands[boat] = userInput[userInput.index(' ')+1:]

                        sleep(2)

                        for boat in toLaunch:
                            dictMode[boat] = 0
                            dictCommands[boat] = (initRudder,initSail)
                            if boat in controlled:
                                controlled.remove(boat)
                            pad = padsUseDict[boat]
                            if pad is not None:
                                padsUseDict[pad] = 0
                                padsUseDict[boat] = None
                    else:
                        rospy.loginfo("Command cancelled.")

                except Exception as e:
                    rospy.loginfo('Error input: {0}'.format(e))



    ##################################################################################################
    # Connection of a boat to a pad.
    # Press f1, f2, ..., f[id] to connect and control with pad.
    # First connected will be controlled with arrows, second with WASD, but you will have
    # to disconnect one boat before controlling another.
    # IDs used are those of XBees.
    ##################################################################################################

        try:
            if key in fList:
                id = fList.index(key)+1

                if id in connected:
                    if dictMode[id] != 1:

                        if len(controlled) <= 2:
                            rospy.loginfo('Take control of boat '+str(id))
                            dictMode[id] = 1
                            controlled.append(id)

                            for pad in padsUseDict:
                                if padsUseDict[pad] == 0:
                                    padsUseDict[pad] = id
                                    padsUseDict[id] = pad
                                    rospy.loginfo('You can use '+pad+' to control boat '+str(id))
                                    break
                        else:
                            rospy.loginfo("You already control 2 sailoats, remove 1 of them !")

                    else:
                        rospy.loginfo('Back to autonomy for boat '+str(id))
                        dictMode[id] = 0
                        dictCommands[id] = (initRudder,initSail)
                        controlled.remove(id)
                        pad = padsUseDict[id]
                        padsUseDict[pad] = 0
                        padsUseDict[id] = None

                else:
                    rospy.loginfo('This boat is not part of the current fleet')

        except Exception as e:
            rospy.loginfo('Error2: {0}'.format(e))


    ##################################################################################################
    # Control of connected sailboats.
    # Commands for rudder and sail. 'Move forward' will close the sail.
    # Default is full open, rudder straight.
    # Rudder comes back straight if no command is issued for 0.1s .
    ##################################################################################################

        try:
            if keyString is not None:
                for pad in padComponents:  #ie WASD or Arrows
                    if keyString in padComponents[pad] and padsUseDict[pad]!=0:  #key is part of a pad and the pad is connected to a boat
                        id = padsUseDict[pad]
                        rospy.loginfo("Controlling boat "+str(id))
                        (rudder, sail) = dictCommands[id]

                        if keyString in (padComponents[pad][1], padComponents[pad][3]): #action on rudder
                            dictLastMove[id] = time()

                        rudder += sensibilite1 * (1 if keyString == padComponents[pad][3] else -1 if keyString == padComponents[pad][1] else 0)
                        sail -= sensibilite2 * (1 if keyString == padComponents[pad][0] else -1 if keyString == padComponents[pad][2] else 0)

                        sail = max(0, min(sail, pi/2))        #Security checks
                        rudder = max(-pi/4, min(rudder, pi/4))

                        dictCommands[id] = (rudder, sail)



        except AttributeError:
            rospy.loginfo('Wrong key: {0}'.format(e))

        except Exception as e:
            rospy.loginfo('Error3: {0}'.format(e))




def connected_callback(data):
    global connected
    connected = eval(data.data)  # eval(data.data[1:]) if running on test launch file
    rospy.loginfo("Received connection")






###################################################################
#    Main
###################################################################


def run():

###################################################################
#    Variables
###################################################################

    global sensibilite1, sensibilite2, pubCommand, initRudder, initSail
    global dictLastMove, end
    global controlMode, commands, connected, dictMode, dictCommands
    global controlled, padsUseDict, keyboardMode


    keyboardMode = 'qwerty' #'azerty'

###################################################################
#    keyBoardListener
###################################################################


    keyboardListener = keyboard.Listener(
        on_press=on_press)
    keyboardListener.start()
    end = False

###################################################################
#    initialisation of variables
###################################################################


    initRudder = 0.0
    initSail = pi/2
    timeLastMove = time()

    sensibilite1 = pi/100
    sensibilite2 = pi/100

    timeLastMove = time()

    connected = []
    controlled = []
    runningCommand = []


###################################################################
#    Ros initialisation
###################################################################


    rospy.init_node('operator', anonymous=True)

    rospy.Subscriber('xbee_send_connected', String, connected_callback)

#    Publishes the data relative to the target point
#    (depends on controlMode, common to all boats)
    pubCommand = rospy.Publisher('commands', String, queue_size = 2)

#    Publishes the string indicator of the control mode
    pubControlMode = rospy.Publisher('controlMode', String, queue_size = 2)

    rate = rospy.Rate(20)

    while connected == [] and not rospy.is_shutdown():
        sleep(1)
        rospy.loginfo("Waiting...")
    rospy.loginfo("Connected : "+str(connected))

###################################################################
#    Set list of connected boats and go to automatic mode
###################################################################

    dictMode = {boat:0 for boat in connected}
    dictLastMove = {boat:time() for boat in connected}
    dictCommands = {boat:(initRudder,initSail) for boat in connected}
    padsUseDict = {boat:None for boat in connected}
    padsUseDict['WASD']=0
    padsUseDict['Arrows']=0




###################################################################
#    Loop
###################################################################


    while not rospy.is_shutdown():

        for boat in connected:
            if dictMode[boat] == 1 and time()-dictLastMove[boat] > 0.1:
                dictCommands[boat] = (initRudder, dictCommands[boat][1])

        controlMode = String(data = str(dictMode))
        commands = String(data = str(dictCommands))


        pubControlMode.publish(controlMode)
        pubCommand.publish(commands)



        rate.sleep()

    end = True




