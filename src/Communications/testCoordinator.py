#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import Imu

from numpy import pi, sign


###################################################################
#    Program that allows sending commands from the keyboard
#   /!\ Same command in broadcast /!\
#   Launch  key_teleop in a separate terminal
#   sudo apt install ros-melodic-teleop-tools ros-melodic-key-teleop
#   rosrun key_teleop key_teleop.py key_vel:=keyboardControl
###################################################################



def callback(data):
    global rudder, sail, sensibilite1, sensibilite2, pubCommand

    rudder += sensibilite1 * sign(data.linear.x)
    sail -= sensibilite2 * sign(data.angular.z)

    commands = String(data=str(rudder)+','+str(sail))
    pubCommand.publish(commands)





###################################################################
#    Main
###################################################################


def run():

###################################################################
#    Initialisation
###################################################################

    global rudder, sail, sensibilite1, sensibilite2, pubCommand

    rudder = 0
    sail = 80
    sensibilite1 = pi/100
    sensibilite2 = 1


    rospy.init_node('testGenerator', anonymous=True)

    rospy.Subscriber("key_vel", Twist, callback)

#    Publishes the data relative to the target point
#    (depends on controlMode, common to all boats)
    pubCommand = rospy.Publisher('commands', String, queue_size = 2)

#    Publishes the string indicator of the control mode
    pubControlMode = rospy.Publisher('controlMode', String, queue_size = 2)



###################################################################
#    Test area
###################################################################

    rate = rospy.Rate(20)

    controlMode = String(data = "1")


    while not rospy.is_shutdown():

        pubControlMode.publish(controlMode)


        rate.sleep()








