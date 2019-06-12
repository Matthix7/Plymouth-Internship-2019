#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import time


def run():
    ser = serial.Serial("/dev/ttyUSB0",baudrate=57600)

    msg = ''

    while msg != 'fin':
      msg = input()
      ser.write((msg+'\n').encode())

      time.sleep(1)
