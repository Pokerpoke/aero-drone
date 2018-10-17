#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""

Copyright (c) 2018 NUAA AeroLab

@file
@author   Jiang Yang (pokerpoke@qq.com)
@date     2018-10
@brief    
@version  0.0.1

Last Modified:  2018-10-16
Modified By:    Jiang Yang (pokerpoke@qq.com)

"""

from dronekit import connect, VehicleMode
from aerodrone import *
import threading
import RPi.GPIO as GPIO
import time
import math
import json
import serial
import sys


vehicle = connect("127.0.0.1:14555", wait_ready=True)
# vehicle = connect("192.168.0.17:14555", wait_ready=True)
distance_to_obstacle = 0.0
distance_lock = threading.Lock()
distance_udpated = threading.Event()

# playground
#
# |         |
# |    -----|
# |         |
# |-----    |
# |    ↑    |
# |   ←o    |
# o             : origin
# current_pos_y : ↑
# current_pos_x : ←
current_pos_y = 0.0
current_pos_x = 0.0

step_velocity = 0.25


def land():
    """
    Change mode to LAND, close vehicle and terminate program.
    """
    print("Mode change to land.")
    vehicle.mode = VehicleMode("LAND")
    vehicle.close()
    sys.exit()


def distance_measurement():
    """
    Get distance to obstacle.
    """
    global distance_to_obstacle

    GPIO.setwarnings(False)
    TRIG = 18
    ECHO = 16
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

    while (True):
        distance_udpated.clear()

        t_sum = 0.0

        # mean filter
        for _ in range(7):
            GPIO.output(TRIG, True)
            time.sleep(0.1)
            GPIO.output(TRIG, False)
            while GPIO.input(ECHO) == 0:
                pass
            pulse_start = time.time()
            while GPIO.input(ECHO) == 1:
                pass
            pulse_end = time.time()
            duration = pulse_end - pulse_start

            distance_to_obstacle_new = round(duration * 17150.0, 2)

            t_sum = t_sum + distance_to_obstacle_new

        t_sum = t_sum / 7.0

        # update distance
        distance_lock.acquire()
        distance_to_obstacle = t_sum
        # FOR DEBUG: print distance
        print("Distance to obstacle: " + str(distance_to_obstacle))
        distance_lock.release()
        # call other threads
        distance_udpated.set()


def move_forward(v, duration=1):
    """
    Move forward.

    :param v: velocity
    :param duration: move for specified seconds.
    """
    global current_pos_y
    send_body_ned_velocity(vehicle, v, 0, 0, duration)
    current_pos_y = current_pos_y + v * duration
    print("Move forward, y: " + str(current_pos_y))


def move_left(v, duration=1):
    """
    Move left.

    :param v: velocity
    :param duration: move for specified seconds.
    """
    global current_pos_x
    send_body_ned_velocity(vehicle, 0, -v, 0, duration)
    current_pos_x = current_pos_x + v * duration
    print("Move left, x: " + str(current_pos_x))


def move_right(v, duration=1):
    """
    Move right.

    :param v: velocity
    :param duration: move for specified seconds.
    """
    global current_pos_x
    send_body_ned_velocity(vehicle, 0, v, 0, duration)
    current_pos_x = current_pos_x - v * duration
    print("Move right, x: " + str(current_pos_x))


def back_to_center():
    """
    Move back to center.
    """
    global current_pos_x
    global step_velocity
    while (current_pos_x != 0.0):
        if current_pos_x > 0.0:
            move_right(step_velocity)
        elif current_pos_x < 0.0:
            move_left(step_velocity)


def obstacle_avoidance():
    """
    Turn right/left to avoide obstcle.
    """
    global distance_to_obstacle
    global current_pos_x
    global current_pos_y
    global step_velocity

    DISTANCE_THRESHOLD = 120.0

    while (True):
        # reach destination
        if current_pos_y > 30:
            print("Arrived destination")
            back_to_center()
            land()
        # wait for distance measurement
        distance_udpated.wait()
        # FOR DEBUG: compare with distance measurement output
        print("Distance to obstacle (used): "+str(distance_to_obstacle))
        # check if too needed to avoid
        if distance_to_obstacle < DISTANCE_THRESHOLD and distance_to_obstacle > 10:
            # avoid
            move_left(step_velocity)  # m/s
            # touch the border of playground
            if current_pos_x >= 2.0:
                back_to_center()
                move_right(0.5, 2)
                # move forward for 4m
                move_forward(0.5, 8)
                move_left(0.5, 2)
        elif distance_to_obstacle > DISTANCE_THRESHOLD:
            # go ahead
            if current_pos_x == 0:
                move_forward(0.5)
            # go across the obstacle
            elif current_pos_x != 0:
                move_left(0.5)
                # move forward for 4m
                move_forward(0.5, 8)
                back_to_center()
        distance_udpated.clear()


def main():
    arm_and_take_off(vehicle, 2)
    vehicle.groundspeed = 1  # m/s
    # wait for UAV to stablize
    time.sleep(1)
    threads = []
    for func in [distance_measurement,
                 obstacle_avoidance]:
        threads.append(threading.Thread(target=func))
        threads[-1].setDaemon(True)
        threads[-1].start()

    while (True):
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            land()


if __name__ == '__main__':
    main()
