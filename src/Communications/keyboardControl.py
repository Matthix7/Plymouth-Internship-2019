#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import cv2

def run():
    cv2.namedWindow('Keyboard Control', cv2.WINDOW_NORMAL)

    rospy.init_node('keyboardController', anonymous=True)
    pub = rospy.Publisher('dataToSend', String, queue_size = 10)

    rate = rospy.Rate(3) # hz


    servo1 = 0
    servo2 = 0
    sensibilite = 3

    while True and not rospy.is_shutdown():

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            servo1 = servo1 - sensibilite

        if key == ord('d'):
            servo1 = servo1 + sensibilite

        if key == ord('z'):
            servo2 = servo2 + sensibilite

        if key == ord('s'):
            servo2 = servo2 - sensibilite

        if key == 27:
            break

        msg = 'Servo1_'+str(servo1)+'_Servo2_'+str(servo2)+'\n'

        pub.publish(msg)
        rate.sleep()













