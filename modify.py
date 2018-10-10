#!/usr/bin/env python
# -*- coding:utf-8 -*-

from dronekit import connect, VehicleMode
from aerodrone import *
import math
import json
import serial
import argparse

# connect to vehicle
# vehicle = connect("192.168.0.24:14555", wait_ready=True)
# vehicle = connect("/dev/ttyACM0", wait_ready=True)
vehicle = connect("127.0.0.1:14555", wait_ready=True)
ser = serial.Serial("/dev/serial0", 115200, timeout=0.5)


class Position:
    x = 0
    v_x = 0
    y = 0
    v_y = 0

    def __init__(self, *args, **kwargs):
        self.x = 0
        self.v_x = 0
        self.y = 0
        self.v_y = 0

    def to_origin():
        return math.sqrt(x * x + y * y)

    def line_speed():
        return math.sqrt(v_x * v_x + v_y * v_y)


def help():
    parser = argparse.ArgumentParser(description='Process some arguments.')
    parser.add_argument('--sitl')


def found_target():
    """
    Read from serial.
    """
    ser.flushInput()
    line = ser.readline()
    try:
        openmv_data = json.loads(line)
    except ValueError:
        return False
    target_pos = Position()
    openmv_cx = openmv_data[u'cx']
    openmv_cy = openmv_data[u'cy']
    return (openmv_cx, openmv_cy)


def draw_circle(radis=6.0):
    """
    Move around to find target.
    """
    goto(vehicle, radis, 0)
    goto(vehicle, -radis/2, radis/2*1.732)
    goto(vehicle, -radis, 0)
    goto(vehicle, -radis/2, -radis/2*1.732)
    goto(vehicle, radis/2, -radis/2*1.732)
    goto(vehicle, radis, 0)
    goto(vehicle, radis/2, radis/2*1.732)
    return None


def main():

    arm_and_take_off(vehicle, 5)
    vehicle.groundspeed = 1  # m/s

    while (True):
        target_pos = draw_circle()
        if target_pos != None:
            break

    while(True):
        ser.flushInput()
        line = ser.readline()
        try:
            openmv_data = json.loads(line)
        except ValueError:
            continue
        openmv_cx = openmv_data[u'cx']
        openmv_cy = openmv_data[u'cy']

        theta = vehicle.heading

        rotate_theta = (theta + 90) % 360

        dx = openmv_cx-160
        dy = openmv_cy-120

        dest_x = math.cos(rotate_theta/2/math.pi)*dx - \
            math.sin(rotate_theta/2/math.pi)*dy
        dest_y = math.sin(rotate_theta/2/math.pi)*dx + \
            math.cos(rotate_theta/2/math.pi)*dy

        print(str(dest_x)+", "+str(dest_y))

        goto(vehicle, 0.01*dest_x, 0.01*dest_y)

    vehicle.mode = VehicleMode("LAND")
    vehicle.close()


if __name__ == '__main__':
    main()
