#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D, Vector3
from gps_common.msg import GPSFix

# import the necessary packages
import time
import cv2
from cv2 import aruco
import numpy as np
from math import atan2
from numpy import pi, cos, sin, tan, array, shape
from sensor_msgs.msg import Image, CameraInfo
from apriltag_ros.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError


from detectionBuoy import detectBuoy, getColorRange, getColorRangeTest, getColorRangeTest2
from detectionHorizonMast2 import horizonArea, detectMast
from detectionAruco import detectAruco
from camThread import PiVideoStream





def sub_EULER_ANGLES(data):
    global roll
    roll = data.z

def sub_GPS(data):
    global timeString
    timeString = str(data.time).replace('.','_')


def sub_april_image(image_message):
    global aprilFrame, bridge, timeString, package_path, detected
    aprilFrame = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    try:
        if detected != []:
            cv2.imwrite(package_path+'/arucoDetected/aruco_frame_'+timeString+'.png', aprilFrame)
            print('Saved '+timeString)
    except Exception as e:
        rospy.loginfo('Error save: {0}'.format(e))

def sub_april_detection(data):
    global detected
    detected = data.detections



def run():
    global roll, timeString, bridge, aprilFrame, package_path, detected

    horizonDetection = rospy.get_param('horizonDetection', False)
    mastsDetection = rospy.get_param('mastsDetection', False)
    buoyDetection = rospy.get_param('buoyDetection', True)
    markerDetection = rospy.get_param('markerDetection', True)
    outputImage = rospy.get_param('outputImage', False)

    buoySize = rospy.get_param('buoySize', 1) #meters

    if mastsDetection:
        horizonDetection = True


    marker = 'aprilTag'  #aruco


####################    ROS initialisation     #########################
######################################################################

    rospy.init_node('imageProcessing', anonymous=True)

    rospy.loginfo("Configuration :\nhorizonDetection :"+str(horizonDetection)+"\
    \nmastsDetection :"+str(mastsDetection)+ "\nbuoyDetection :"+str(buoyDetection)+"\
    \nmarkerDetection :" + str(markerDetection)+ "\noutputImage :"+str(outputImage)+"\
    \nbuoySize :"+str(buoySize))


    r = rospkg.RosPack()
    package_path = r.get_path('plymouth_internship_2019')

#    Publishes an array with the headings leading to vertical lines (ie possible boats)
    pub_send_headings_boats = rospy.Publisher('camera_send_headings_boats', String, queue_size = 2)
    headings_boats_msg = String()

#    Publishes the heading leading to the biggest detected buoy
    pub_send_buoy = rospy.Publisher('camera_send_buoy', Vector3, queue_size = 2)
    buoy_msg = Vector3()

#    Publishes a list with the headings leading to the detected ArUco codes (April Tags)
    pub_send_headings_arucos = rospy.Publisher('camera_send_headings_arucos', Float32, queue_size = 2)
    headings_arucos_msg = Float32()

#    Receives the euler_angles  x=yaw y=pitch z=roll
    rospy.Subscriber('filter_send_euler_angles', Vector3, sub_EULER_ANGLES)

    rospy.Subscriber('filter_send_gps', GPSFix, sub_GPS)

    rospy.Subscriber('tag_detections_image', Image, sub_april_image)
    rospy.Subscriber('tag_detections', AprilTagDetectionArray, sub_april_detection)


###################    Code initialisation    ######################
####################################################################

    t0 = time.time()
    Tframe, T1, T2, T3, T4, T5, T6 = [], [], [], [], [], [], []
    tframe = 0

    if outputImage and (buoyDetection or markerDetection):
        cv2.namedWindow('Global', cv2.WINDOW_NORMAL)
    if outputImage and horizonDetection:
        cv2.namedWindow('Horizon', cv2.WINDOW_NORMAL)

    roll = 0

    timeString = 'unknownTime'

    bridge = CvBridge()
    camInfo = CameraInfo()
    detected = []

    if marker == "aruco":
        aruco_dict = aruco.Dictionary_get(aruco.DICT_APRILTAG_25h9)

    c = 0
    loopPeriod = 0.5
    if not horizonDetection:
        loopPeriod = 0.3

    rate = rospy.Rate(1/loopPeriod)


###############          VIDEO           #############################
######################################################################
##    Running on test video
#    cap = cv2.VideoCapture(package_path+'/src/PiCamera/testImages/aprilTags.avi')

#    image_pub = rospy.Publisher("camera_rect/image_rect",Image, queue_size = 0)
#    info_pub = rospy.Publisher("camera_rect/camera_info",CameraInfo, queue_size = 0)

#    t0 = time.time()

#    dodo = 0.

#    Sf, resolution = 0.398/203.55, (320, 240)

#    horizon_prev = (0, 160, 120)
#    rotation_prev = -999
#    ret, image = cap.read()
#    image = cv2.resize(image, resolution)
#    horizon, horizon_height, horizon_prev = horizonArea(image, horizon_prev, init = True)
#    newInit = False

#    outputImage = True

#    while(cap.isOpened()) and not rospy.is_shutdown():

#        # Capture frame-by-frame
#        ret, image = cap.read()

#        if not ret:
#            break

#        image = cv2.resize(image, (320, 240))

#        try:
#            image_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
#            info_pub.publish(camInfo)
#        except Exception as e:
#            rospy.loginfo('Error cam: {0}'.format(e))

#################     CAMERA     ####################################
#####################################################################
#    Running with the camera

    vs = PiVideoStream(resolution=(320, 240), framerate=5, mode = 'sports', record = True).start()
    time.sleep(2)

    Sf, resolution = vs.getScaleFactor()
    dodo = 0

    horizon_prev = (0, resolution[0], resolution[1])
    rotation_prev = -999
    image = vs.read()

    horizon, horizon_height, horizon_prev = horizonArea(image, horizon_prev, init = True)
    newInit = False


    while not rospy.is_shutdown():

        image = vs.read()[int(0.05*resolution[1]):int(0.95*resolution[1]), int(0.05*resolution[0]):int(0.95*resolution[0])]



######################################################################
######################################################################

        if tframe != 0:
            Tframe.append(time.time()-tframe-dodo)
        tframe = time.time()

        c += 1

        if horizonDetection:

##        Find the area where horizon is located and return a frame containing the horizon, transformed to be horizontal.
##        Takes about 0.04s per frame.
##        horizon: image cropped around the horizon
##        horizon_height: vertical position in pixels of the horizon in the cropped image (for masts detection)
##        horizon_prev: vertical position in pixels of the horizon in the previous uncropped image, in case horizon is not
##        detected in the new image.


            t1 = time.time()
            try:
                horizon, horizon_height, horizon_prev = horizonArea(image, horizon_prev, newInit)

                rotation = horizon_prev[0]
                x0 = horizon_prev[1]
                y0 = horizon_prev[2]
                M = horizon_prev[3]
                new_height = int(M[1,0]*x0 + M[1,1]*y0 + M[1,2])

                if new_height < 0.1*resolution[1] or new_height > 0.9*resolution[1]:
                    newInit = True
                else:
                    newInit = False

                T1.append(time.time()-t1)

                if mastsDetection:

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

            except:
                print('Did not detect horizon, please do not panic -', c)
                newInit = True
                if rotation_prev == -999:
                    rotation = 0
                else:
                    rotation = rotation_prev

        if not horizonDetection:
            rotation = roll*180/pi

        rotation_prev = rotation

        if buoyDetection:

##      Find the buoy in the original cropped image and highlight it in the result image
##      Check if the color range corresponds to what you look for!

            t3 = time.time()
#            colorRange = getColorRangeTest2() #For test target
            colorRange = getColorRange() #For real buoys
            center, radius, frame_markers = detectBuoy(image, image.copy(), colorRange)
            if center is not None:
                xBuoy = center[0]*cos(rotation*pi/180)+center[1]*sin(rotation*pi/180)
                headingBuoy = (xBuoy-resolution[0]/2)*Sf

                if radius > 5:
                    distBuoy = buoySize/tan(2*radius*Sf)
                else:
                    distBuoy = -999
            else:
                headingBuoy = -999
                distBuoy = -999

            buoy_msg.x = distBuoy
            buoy_msg.y = headingBuoy
            pub_send_buoy.publish(buoy_msg)
            T3.append(time.time()-t3)


        if markerDetection:
            if marker == "aruco":
        ##      Find the aruco in the original-sized image

                t4 = time.time()
                if buoyDetection:
                    frame_markers, corners = detectAruco(image, frame_markers, aruco_dict, timeString)
                else:
                    frame_markers, corners = detectAruco(image, image, aruco_dict, timeString)
        #        headingsMarkers = []

        #        for corner in corners:
        #            print(corner[0,0,0], corner[0,0,1], rotation, resolution[0], Sf)
        #            headingsMarkers.append(-((corner[0,0,0]*cos(rotation*pi/180)+corner[0,0,1]*sin(rotation*pi/180))-resolution[0]/2)*Sf)
        #        headings_arucos_msg.data = str(headingsMarkers)

                headings_arucos_msg.data = -999
                if corners != []:
                    corner = corners[0]
                    headings_arucos_msg.data = -((corner[0,0,0]*cos(rotation*pi/180)+corner[0,0,1]*sin(rotation*pi/180))-resolution[0]/2)*Sf

                pub_send_headings_arucos.publish(headings_arucos_msg)
                T4.append(time.time()-t4)

            if marker == "aprilTag":
                T4.append(0)   #all in callback



        t5 = time.time()
        if outputImage and horizonDetection:
            if mastsDetection:
                cv2.imshow('Horizon', masts)
            else:
                cv2.imshow('Horizon', horizon)
        if outputImage and (buoyDetection or markerDetection):
            cv2.imshow('Global', frame_markers)
        T5.append(time.time()-t5)

        dodo = max(0, loopPeriod - (time.time()-tframe))
        rate.sleep()

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
    print("Global time per frame : ", (time.time()-t0)/c)

    if horizonDetection:
        print("Time horizon : ", np.mean(T1))
    if mastsDetection:
        print("Time masts   : ", np.mean(T2))
    if buoyDetection:
        print("Time buoy    : ", np.mean(T3))
    if markerDetection:
        print("Time markers : ", np.mean(T4))
    print("Time display : ", np.mean(T5))
    print("Time interact: ", np.mean(T6))
    print("Time per frame accurate: ", np.mean(Tframe))


if __name__ == "__main__":
    run()
