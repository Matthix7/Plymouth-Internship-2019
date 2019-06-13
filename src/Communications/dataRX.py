#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import time

def run():
    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600)
    line = ''

    while line != '10\n':
      time.sleep(0.5)

      line = ser.readline()   # read a '\n' terminated line
      line = line.decode()
      print(line)
