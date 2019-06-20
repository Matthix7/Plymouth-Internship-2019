#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu

from numpy import pi

###################################################################
#    Main
###################################################################


def run():

###################################################################
#    Initialisation
###################################################################

    rospy.init_node('testGenerator', anonymous=True)

#    Publishes the data relative to the target point
#    (depends on controlMode, common to all boats)
    pubTarget = rospy.Publisher('poseTarget', Pose2D, queue_size = 2)

#    Publishes the string indicator of the control mode
    pubControlMode = rospy.Publisher('controlMode', String, queue_size = 2)


###################################################################
#    Test area
###################################################################

    rate = rospy.Rate(1)

    dataTarget = Pose2D(x = 100.5, y = 100.5, theta = 0.)

    controlMode = String(data = "Test")


    while not rospy.is_shutdown():

        pubTarget.publish(dataTarget)

        pubControlMode.publish(controlMode)


        rate.sleep()








