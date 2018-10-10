#!/usr/bin/env python
# -*- coding:utf-8 -*-

from dronekit import connect, VehicleMode
from aerodrone import *
import math
import json
import serial
import argparse


def main():
    # vehicle = connect("192.168.0.24:14555", wait_ready=True)
    # vehicle = connect("/dev/ttyACM0", wait_ready=True)
    vehicle = connect("127.0.0.1:14555", wait_ready=True)
    arm_and_take_off(vehicle, 5)
    vehicle.groundspeed = 1  # m/s

    radis = 6.0  # m
    goto(vehicle, radis, 0)
    goto(vehicle, -radis/2, radis/2*1.732)
    goto(vehicle, -radis, 0)
    goto(vehicle, -radis/2, -radis/2*1.732)
    goto(vehicle, radis/2, -radis/2*1.732)
    goto(vehicle, radis, 0)
    goto(vehicle, radis/2, radis/2*1.732)

    condition_yaw(vehicle, 0)

    vehicle.mode = VehicleMode("LAND")
    vehicle.close()


if __name__ == '__main__':
    main()
