#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg

from std_msgs.msg import Float32, String

# import the necessary packages
import time
import cv2
from cv2 import aruco
import numpy as np
from math import atan2
from numpy import pi, cos, sin, array, shape


from detectionBuoy import detectBuoy, getColorRange, getColorRangeTest
from detectionHorizonMast2 import horizonArea, detectMast
from detectionAruco import detectAruco
from camThread import PiVideoStream


def run():

####################    ROS initialisation     #########################
######################################################################

    rospy.init_node('imageProcessing', anonymous=True)
    r = rospkg.RosPack()
    package_path = r.get_path('plymouth_internship_2019')

#    Publishes an array with the headings leading to vertical lines (ie possible boats)
    pub_send_headings_boats = rospy.Publisher('camera_send_headings_boats', String, queue_size = 2)
    headings_boats_msg = String()

#    Publishes the heading leading to the biggest detected buoy
    pub_send_heading_buoy = rospy.Publisher('camera_send_heading_buoy', Float32, queue_size = 2)
    heading_buoy_msg = Float32()

#    Publishes a list with the headings leading to the detected ArUco codes (April Tags)
    pub_send_headings_arucos = rospy.Publisher('camera_send_headings_arucos', Float32, queue_size = 2)
    headings_arucos_msg = Float32()


###################    Code initialisation    #######################
####################################################################

    t0 = time.time()
    Tframe, T1, T2, T3, T4, T5, T6 = [], [], [], [], [], [], []
    tframe = 0


#    cv2.namedWindow('Global', cv2.WINDOW_NORMAL)
#    cv2.namedWindow('Horizon', cv2.WINDOW_NORMAL)

    horizon_prev = (0, 320, 240)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    c = 0

###############          VIDEO           #############################
######################################################################
##    Running on test video
#    cap = cv2.VideoCapture('testImages/PlymouthSound.mp4')

#    t0 = time.time()

#    dodo = 0.05

#    Sf, resolution = 0.398/203.55, (640,480)

#    horizon_prev = (0, 320, 240)
#    ret, image = cap.read()
#    image = cv2.resize(image, resolution)
#    horizon, horizon_height, horizon_prev = horizonArea(image, horizon_prev, init = True)

#    while(cap.isOpened()) and not rospy.is_shutdown()::

#        # Capture frame-by-frame
#        ret, image = cap.read()

#        if not ret:
#            break

#        image = cv2.resize(image, (640,480))



#################     CAMERA     ####################################
#####################################################################
#    Running with the camera

    vs = PiVideoStream(record = True).start()
    time.sleep(2)

    Sf, resolution = vs.getScaleFactor()
    dodo = 0

    horizon_prev = (0, resolution[0], resolution[1])
    image = vs.read()

    horizon, horizon_height, horizon_prev = horizonArea(image, horizon_prev, init = True)


    while not rospy.is_shutdown():

        image = vs.read()



######################################################################
######################################################################

        if tframe != 0:
            Tframe.append(time.time()-tframe-dodo)
        tframe = time.time()

        c += 1

##        #Find the area where horizon is located and return a frame containing the horizon, transformed to be horizontal.
##        #Takes about 0.04s per frame.
##        #horizon: image cropped around the horizon
##        #horizon_height: vertical position in pixels of the horizon in the cropped image (for masts detection)
##        #horizon_prev: vertical position in pixels of the horizon in the previous uncropped image, in case horizon is not
##        #detected in the new image.


        t1 = time.time()
        horizon, horizon_height, horizon_prev = horizonArea(image, horizon_prev)
        rotation = horizon_prev[0]
        T1.append(time.time()-t1)

##      Find the areas where vertical lines are found (ie possible sailboats).
##      Takes about 0.1s per frame.
##      masts: image cropped around the horizon, where vertical lines are highlighted

        t2 = time.time()
        masts, xMasts = detectMast(horizon, horizon_height)
        if xMasts is not None:
            headingsBoats = (np.asarray(xMasts)-resolution[0]/2)*Sf
        else:
            headingsBoats = None
        headings_boats_msg.data = str(headingsBoats)
        pub_send_headings_boats.publish(headings_boats_msg)
        T2.append(time.time()-t2)

##      Find the buoy in the original cropped image and highlight them in the result image
##      Check if the color range corresponds to what you look for!

        t3 = time.time()
        colorRange = getColorRangeTest() #For test target
#        colorRange = getColorRange() #For real buoys
        center, buoy = detectBuoy(image, image.copy(), colorRange)
        if center is not None:
            xBuoy = center[0]*cos(rotation*pi/180)+center[1]*sin(rotation*pi/180)
            headingBuoy = (xBuoy-resolution[0]/2)*Sf
        else:
            headingBuoy = -999
        heading_buoy_msg.data = headingBuoy
        pub_send_heading_buoy.publish(heading_buoy_msg)
        T3.append(time.time()-t3)

##      Find the April Tags in the original-sized image

        t4 = time.time()
        frame_markers, corners = detectAruco(image, buoy, aruco_dict)
#        headingsMarkers = []

#        for corner in corners:
##            print(corner[0,0,0], corner[0,0,1], rotation, resolution[0], Sf)
#            headingsMarkers.append(((corner[0,0,0]*cos(rotation*pi/180)+corner[0,0,1]*sin(rotation*pi/180))-resolution[0]/2)*Sf)
#        headings_arucos_msg.data = str(headingsMarkers)

        headings_arucos_msg.data = -999
        if corners != []:
            corner = corners[0]
            headings_arucos_msg.data = ((corner[0,0,0]*cos(rotation*pi/180)+corner[0,0,1]*sin(rotation*pi/180))-resolution[0]/2)*Sf

        pub_send_headings_arucos.publish(headings_arucos_msg)
        T4.append(time.time()-t4)


        t5 = time.time()
#        cv2.imshow('Horizon', masts)
#        cv2.imshow('Global', frame_markers)
        T5.append(time.time()-t5)


        time.sleep(dodo)

#####################################################################
#############        INTERACTION          ###########################

        t6 = time.time()
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
                break

        elif key == 32:
            key = cv2.waitKey(1) & 0xFF
            while key != 32:
                key = cv2.waitKey(1) & 0xFF

        elif key == ord('c'):
            cv2.imwrite(package_path+'/Samples/sample'+time.strftime('%c')+'.png',masts)
            print("Picture saved")

        T6.append(time.time()-t6)

    try:
        cap.release()
    except:
        vs.stop()

    cv2.destroyAllWindows()
    print("Total time : ",time.time()-t0)
    print("Computed frames : ", c)
    print("Global time per frame : ", (time.time()-t0)/c - dodo)
    print("Time horizon : ", np.mean(T1))
    print("Time masts   : ", np.mean(T2))
    print("Time buoy    : ", np.mean(T3))
    print("Time markers : ", np.mean(T4))
    print("Time display : ", np.mean(T5))
    print("Time interact: ", np.mean(T6))
    print("Time per frame accurate: ", np.mean(Tframe))


if __name__ == "__main__":
    run()
