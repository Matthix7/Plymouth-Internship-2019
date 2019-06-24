#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.start_preview()
sleep(10)
camera.stop_preview()
