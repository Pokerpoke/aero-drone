#!/usr/bin/env python
# -*- coding:utf-8 -*-

from dronekit import connect, VehicleMode
from aerodrone import *
import math
import json
import serial


def sgn(num):
    if num >= 0:
        return 1
    else:
        return -1


# vehicle = connect("192.168.0.24:14555", wait_ready=True)
vehicle = connect("/dev/ttyACM0", wait_ready=True)
arm_and_take_off(vehicle, 5)
vehicle.groundspeed = 1  # m/s


ser = serial.Serial("/dev/serial0", 115200, timeout=0.5)

while(True):
    ser.flushInput()
    line = ser.readline()
    try:
        openmv_data = json.loads(line)
    except ValueError:
        continue
    openmv_cx = openmv_data[u'cx']
    openmv_cy = openmv_data[u'cy']
    print(openmv_cx)
    print(openmv_cy)

    theta = vehicle.heading

    rotate_theta = (theta + 90) % 360

    dx = openmv_cx-160
    dy = openmv_cy-120

    dest_x = math.cos(rotate_theta/2/math.pi)*dx - \
        math.sin(rotate_theta/2/math.pi)*dy
    dest_y = math.sin(rotate_theta/2/math.pi)*dx + \
        math.cos(rotate_theta/2/math.pi)*dy

    print(str(dest_x)+", "+str(dest_y))

    goto(vehicle, 0.1*dest_x, 0.1*dest_y)

vehicle.mode = VehicleMode("LAND")
vehicle.close()
