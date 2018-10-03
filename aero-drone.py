#!/usr/bin/env python
# -*- coding:utf-8 -*-

from dronekit import connect, VehicleMode
from aerodrone import *
import math
import json


def sgn(num):
    if num >= 0:
        return 1
    else:
        return -1

#  vehicle = connect("127.0.0.1:14555", wait_ready=True)
#  arm_and_take_off(vehicle, 5)
#  vehicle.groundspeed = 1  # m/s


ser = serial.Serial("/dev/serial0", 115200, timeout=0.5)

while(True):
    line = ser.readline()
    try:
        openmv_data = json.loads(line)
    except ValueError:
        continue
    openmv_cx = openmv_data[u'cx']
    openmv_cy = openmv_data[u'cy']
    print(openmv_cx)
    print(openmv_cy)

    theta = vehicle.heading()

    rotate_theta = (theta + 90) % 360

    dx = openmv_cx-160
    dy = openmv_cy-120

    dest_x = math.cos(rotate_theta/2/math.pi)*dx - \
        math.sin(rotate_theta/2/math.pi)*dy
    dest_y = math.sin(rotate_theta/2/math.pi)*dx + \
        math.cos(rotate_theta/2/math.pi)*dy

    print(dest_x)
    print(dest_x)

    #  goto(sgn(dx)*10,

vehicle.mode = VehicleMode("LAND")
vehicle.close()
