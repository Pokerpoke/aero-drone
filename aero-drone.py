#!/usr/bin/env python
# -*- coding:utf-8 -*-

from dronekit import connect, VehicleMode
from aerodrone import *
import math

vehicle = connect("127.0.0.1:14555", wait_ready=True)
arm_and_take_off(vehicle, 5)
vehicle.groundspeed = 5  # m/s


# for theta in xrange(360):
#     x = 5 * math.cos(theta/180*math.pi)
#     y = 5 * math.sin(theta/180*math.pi)
#     goto(vehicle, x, y)

v_x = 0.3  # m/s
v_y = 0.4  # m/s

send_ned_velocity(vehicle, v_x, v_y, 0, 2)
# goto_position_target_local_ned(vehicle, 5, 5, 0)
v_x = 2  # m/s
v_y = -4  # m/s

send_ned_velocity(vehicle, v_x, v_y, 0, 2)

# goto(vehicle, 10, 10)
# goto(vehicle, 20, 20)

vehicle.mode = VehicleMode("LAND")
vehicle.close()
