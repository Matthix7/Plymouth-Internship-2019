#!/usr/bin/env python
# -*- coding: utf-8 -*-

##import rospy
import rospkg

# import the necessary packages

if __name__ == "__main__":
    from picamera.array import PiRGBArray
    from picamera import PiCamera
    import numpy as np
    from math import atan2
    from numpy import pi, cos, sin, array, shape

import cv2
from cv2 import aruco
import time


def run():

    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32

    camera.exposure_mode = 'sports'

    rawCapture = PiRGBArray(camera, size=(640, 480))

    # allow the camera to warmup
    time.sleep(0.1)


    t0 = time.time()

    cv2.namedWindow('Webcam', cv2.WINDOW_NORMAL)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_APRILTAG_25h9)

    c = 0

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        c+=1

        image = cv2.resize(frame.array, (640,480))

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        frame_markers, corners = detectAruco(image, image, aruco_dict, '')
#        print(corners[0][0])


        frame_markers = cv2.flip(frame_markers, 0)
        cv2.imshow("Webcam", frame_markers)



##        time.sleep(0.1)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
                break

        if key == 32:
            key = cv2.waitKey(1) & 0xFF
            while key != 32:
                key = cv2.waitKey(1) & 0xFF

        elif key == ord('c'):
            cv2.imwrite('sample_aruco.png',frame_markers)
            print("Picture saved")


    cv2.destroyAllWindows()
    print("Total time : ",time.time()-t0)
    print("Computed frames : ", c)
    print("Time per frame : ", (time.time()-t0)/c) # - 0.1)




def detectAruco(image, resultImage, dictionary, timeString):
    r = rospkg.RosPack()
    package_path = r.get_path('plymouth_internship_2019')

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(resultImage, corners, ids)

    if ids is not None:
        cv2.imwrite(package_path+'/arucoDetected/aruco_frame_'+timeString+'.png', frame_markers)
        print('Saved '+timeString)

#        markerLength = 3.8
#        camera_matrix = array([[485.36568341, 0, 308.96642615], [0, 486.22575965, 236.66818825], [0, 0, 1]])
#        dist_coeffs = array([[1.37958351e-01, -2.43061015e-01, -5.22562568e-05, -6.84849581e-03, -2.59284496e-02]])

#        rvec, tvec = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs) # For a single marker


#        for i in range(len(ids)):
#            frame_markers = aruco.drawAxis(frame_markers, camera_matrix, dist_coeffs, rvec[i], tvec[i], 10)


    return frame_markers, corners





if __name__ == "__main__":
    run()



