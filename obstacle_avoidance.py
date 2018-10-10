#!/usr/bin/env python
# -*- coding:utf-8 -*-

from dronekit import connect, VehicleMode
from dronekit import *
from aerodrone import *
import math
import json
import serial
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
print ('Distance measurement in progress...')
TRIG = 18
ECHO = 16
GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

vehicle = connect("127.0.0.1:14555", wait_ready=True)
arm_and_take_off(vehicle, 5)  # m
vehicle.groundspeed = 1  # m/s
right = 1  # m
left = 1  # m


def distance_to_obstacle():
    """
    Return distance to obstacle.
    """
    while(True):
        GPIO.output(TRIG, True)
        time.sleep(0.1)
        GPIO.output(TRIG, False)
        while GPIO.input(ECHO) == 0:
            pass
        pulse_start = time.time()
        while GPIO.input(ECHO) == 1:
            pass
        pulse_end = time.time()
        duration = pulse_end-pulse_start
        distance = round(duration*17150, 2)
        return distance


if __name__ == '__main__':
    try:
        while True:
            dist = distance_to_obstcle()
            print ("measured distance = %.1f cm" % dist)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print ("measurement stopper by user")
        GPIO.cleanup()


while(True):
    ser.flushInput()
    line = ser.readline()
    try:
        openmv_data = json.loads(line)
    except ValueError:
        continue

if not openmv_data is None:
    North = right*(math.sin(theta))
    East = right*(math.cos(theta))
    goto(vehicle, North, East)


vehicle.mode = VehicleMode("AUTO")
theta = vehicle.heading
rotate_theta = (theta+270) % 360
dist = distance_to_obstcle()
while dist <= 350:
    vehicle.mode = VehicleMode("GUIDED")
    North = left*(-math.sin(theta))
    East = left*(-math.cos(theta))
    goto(vehicle, North, East)
    print ("vehicle goto:"(North, East))
    if dist > 350:
        break