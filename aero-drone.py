#!/usr/bin/env python
# -*- coding:utf-8 -*-

from dronekit import connect, VehicleMode
from aerodrone import *

vehicle = connect("127.0.0.1:14555", wait_ready=True)
arm_and_take_off(vehicle, 5)
vehicle.groundspeed = 5 # m/s
goto(vehicle, 10, 10)
goto(vehicle, 20, 20)

vehicle.mode = VehicleMode("LAND")
vehicle.close()
