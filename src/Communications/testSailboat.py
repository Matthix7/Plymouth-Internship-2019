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

    rospy.init_node('testSailboat', anonymous=True)

#    Publishes a GPS standard frame
#    Contains xBoat, yBoat and others
    pubGPS = rospy.Publisher('GPS', String, queue_size = 2)

#    Publishes the rest of the data relative to the boat
#    msg.x = headingBoat
#    msg.y = headingWindBoat
#    msg.z = speedWindBoat
    pubPos = rospy.Publisher('poseBoat', Pose2D, queue_size = 2)


###################################################################
#    Test area
###################################################################

    rate = rospy.Rate(1)


    GPSstring = String(data = "$GPGGA,064036.289,4836.5375,N,00740.9373,E,1,04,3.2,200.2,M,,,,0000*0E")

    dataBoat = Pose2D(x = pi, y = pi/2, theta = 10.2)

    while not rospy.is_shutdown():

        pubGPS.publish(GPSstring)

        pubPos.publish(dataBoat)


        rate.sleep()








