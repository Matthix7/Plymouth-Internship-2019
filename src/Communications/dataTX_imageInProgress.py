#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

import serial
import time
import cv2
import numpy as np


ser = serial.Serial("/dev/ttyUSB0",baudrate=57600)


def sendImage(frame):
    sizeX = str(np.shape(frame)[1])

    sizeY = str(np.shape(frame)[0])

    imageBytes = frame.tobytes()

    tailleImage = str(len(imageBytes))

#    print("Sent size X", sizeX)
#    print("Sent size Y", sizeY)
#    print("Sent size IMAGE", np.shape(frame))
#    print("\n Sent array\n", frame)


    ser.write(tailleImage+'\n')

#    rospy.loginfo("|")
#    for i in range(len(tailleImage)):
#        rospy.loginfo("%s", tailleImage[i])
#    rospy.loginfo("|")

    ser.write(sizeX+'\n')
    ser.write(sizeY+'\n')

    rospy.loginfo("tailleImage %d\n", len(imageBytes))
    rospy.loginfo("tailleX %s\n", sizeX)
    rospy.loginfo("tailleY %s\n", sizeY)

    sent = 0
    pas = 10000
    while sent < len(imageBytes) and not rospy.is_shutdown():
        if sent+pas >= len(imageBytes):
            ser.write(imageBytes[sent:]+'\n')
        else:
            ser.write(imageBytes[sent:sent+pas]+'\n')
        sent += pas
    rospy.loginfo("sent %d/%d", sent, len(imageBytes))





def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

#    ser.write((data.data+'\n').encode())



def run():
    rospy.init_node('dataTX', anonymous=True)
    rospy.Subscriber("data", String, callback)

    rospy.loginfo(rospy.get_caller_id() + "\nHello")


    img = cv2.imread("accordeon.png")
    sendImage(img)

    rospy.spin()







