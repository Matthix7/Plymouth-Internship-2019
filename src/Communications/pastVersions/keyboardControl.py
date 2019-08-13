#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from numpy import sign, pi
import cv2

###################################################################
#    Program that allows sending commands from the keyboard
#   /!\ Same command in broadcast /!\
#   Launch  key_teleop in a separate terminal
#   sudo apt install ros-melodic-teleop-tools ros-melodic-key-teleop
#   rosrun key_teleop key_teleop.py key_vel:=keyboardControl
###################################################################


def callback(data):
    global servo1, servo2, sensibilite1, sensibilite2, pub

    servo1 += sensibilite1 * sign(data.linear.x)
    servo2 += sensibilite2 * sign(data.angular.z)

    msg = String()
    msg.data = 'Servo1_'+str(servo1)+'_Servo2_'+str(servo2)

    pub.publish(msg)


def run():
    global servo1, servo2, sensibilite1, sensibilite2, pub

    servo1 = 0
    servo2 = 80
    sensibilite1 = pi/100
    sensibilite2 = 1

    pub = rospy.Publisher('dataToSend', String, queue_size = 10)

    rospy.init_node('keyboardController', anonymous=True)
    rospy.Subscriber("keyboardControl", Twist, callback)

    rospy.spin()










