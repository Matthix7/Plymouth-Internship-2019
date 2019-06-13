#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import time

def run():
    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600)
    msg = 0

    while msg != '10\n':
      msg += 1
      ser.write((str(msg)+'\n').encode())

      time.sleep(1)
