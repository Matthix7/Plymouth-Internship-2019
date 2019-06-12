#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import time
ser = serial.Serial("/dev/ttyUSB0",baudrate=57600)
line = ''

while line != 'fin\n':
  time.sleep(0.5)

  line = ser.readline()   # read a '\n' terminated line
  line = line.decode()
  print(line)
