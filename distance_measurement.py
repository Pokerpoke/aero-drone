#!/usr/bin/env python
# -*- coding:utf-8 -*-

from dronekit import connect, VehicleMode
from aerodrone import *
import threading
import RPi.GPIO as GPIO
import time
import math
import json
import serial


vehicle = connect("127.0.0.1:14555", wait_ready=True)
DISTANCE_TO_OBSTACLE = 0
ARRIVED = False
lock_distance = threading.Lock()


def distance_measure():
    """
    Get distance to obstacle.
    """
    GPIO.setwarnings(False)
    TRIG = 18
    ECHO = 16
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
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

        global DISTANCE_TO_OBSTACLE
        # Lock
        lock_distance.acquire()
        try:
            DISTANCE_TO_OBSTACLE = round(duration * 17150.0, 2)
        finally:
            # release
            lock_distance.release()
        time.sleep(0.1)
        if ARRIVED:
            return None


def obstacle_avoidance():
    # degree of heading
    heading = vehicle.heading
    step_distance = 0.3  # m
    # forward theta
    f_theta = (heading) % 360
    # right theta
    r_theta = (heading+90.0) % 360
    # left theta
    l_theta = (heading-90.0) % 360
    while (True):
        if DISTANCE_TO_OBSTACLE > 350:
            # turn right
            goto(vehicle,
                 step_distance*math.cos(r_theta/180.0*math.pi)
                 step_distance*math.sin(r_theta/180.0*math.pi))
        elif DISTANCE_TO_OBSTACLE < 350:
            # forward
            goto(vehicle,
                 step_distance*math.cos(f_theta/180.0*math.pi),
                 step_distance*math.sin(f_theta/180.0*math.pi))
        if ARRIVED:
            return None


def print_distance():
    while (True):
        global DISTANCE_TO_OBSTACLE
        # lock_distance.acquire()
        print("Distance to obstacle: "+str(DISTANCE_TO_OBSTACLE))
        # lock_distance.release()
        time.sleep(0.5)
        if ARRIVED:
            return None


def main():
    # arm_and_take_off(vehicle, 5)
    # vehicle.groundspeed = 1  # m/s
    for func in [distance_measure,
                 obstacle_avoidance,
                 print_distance]:
        t = threading.Thread(target=func)
        t.start()
        t.join()

    while (True):
        try:
            pass
        except KeyboardInterrupt:
            lock_distance.acquire()
            ARRIVED = True
            lock_distance.release()


if __name__ == '__main__':
    main()
