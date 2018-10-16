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
import sys


vehicle = connect("127.0.0.1:14555", wait_ready=True)
# vehicle = connect("192.168.0.17:14555", wait_ready=True)
distance_to_obstacle = 0.0
distance_lock = threading.Lock()
distance_udpated = threading.Event()
current_pos_y = 0.0
current_pos_x = 0.0

step_velocity = 0.5
step_duration = 1.0


def distance_measure():
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
        for _ in range(5):
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

        t_sum = t_sum / 5.0

        # update distance
        # lock
        distance_lock.acquire()
        distance_to_obstacle = t_sum
        # for debug
        print("Distance to obstacle: " + str(distance_to_obstacle))
        # release lock
        distance_lock.release()
        # call other threads
        distance_udpated.set()


def move_forward(v, duration=1):
    global current_pos_y
    send_body_ned_velocity(vehicle, v, 0, 0, duration)
    current_pos_y = current_pos_y + v * duration
    return True


def move_left(v, duration=1):
    global current_pos_x
    send_body_ned_velocity(vehicle, 0, -v, 0, duration)
    current_pos_x = current_pos_x + v * duration
    return True


def move_right(v, duration=1):
    global current_pos_x
    send_body_ned_velocity(vehicle, 0, v, 0, duration)
    current_pos_x = current_pos_x - v * duration
    return True


def back_to_center():
    global current_pos_x
    global step_velocity
    while (current_pos_x != 0):
        if current_pos_x > 0:
            move_right(step_velocity)
        elif current_pos_x < 0:
            move_left(step_velocity)
    print("Current pos x: " + str(current_pos_x))
    return True


def obstacle_avoidance():
    """
    Turn right/left to avoide obstcle.
    """
    global distance_to_obstacle
    # playground
    #
    # |         |
    # |    -----|
    # |         |
    # |-----    |
    # |         |
    # |         |
    # current_pos_x : ←
    # current_pos_y : ↑
    global current_pos_x
    global current_pos_y
    global step_velocity
    while (True):
        if current_pos_y > 30:
            print("Arrived destination")
            back_to_center()
            vehicle.mode = VehicleMode("LAND")
            vehicle.close()
            sys.exit()
            break

        # if distance_udpated.is_set():
        distance_udpated.wait()
        # distance_lock.acquire()
        print("Distance to obstacle (used): "+str(distance_to_obstacle))
        if distance_to_obstacle < 130 and distance_to_obstacle > 10:
            move_left(step_velocity)  # m/s
            move_left(0.5)
            print("Current X: " + str(current_pos_x))
            if current_pos_x >= 2.0:
                back_to_center()
<<<<<<< HEAD
                move_right(0.5)  # m/s
                move_forward(2)
=======
                move_right(0.5) # m/s
                for _ in range(8):
                    move_forward(0.5)
>>>>>>> a81e4f40d9e105a16469cec32404da987c7ad8f2
                move_left(0.5)
        elif distance_to_obstacle > 100:
            if current_pos_x == 0:
                move_forward(0.5)
            elif current_pos_x != 0:
                # move_forward(2)  # m/s
                for _ in range(8):
                    move_forward(0.5)
            back_to_center()
            print("Current Y: " + str(current_pos_y))

        # distance_lock.release()
        distance_udpated.clear()


def main():
    arm_and_take_off(vehicle, 2)
    vehicle.groundspeed = 1  # m/s
    threads = []
    for func in [distance_measure,
                 obstacle_avoidance]:
        threads.append(threading.Thread(target=func))
        threads[-1].setDaemon(True)
        threads[-1].start()

    while (True):
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print("Mode change to land.")
            vehicle.mode = VehicleMode("LAND")
            vehicle.close()
            sys.exit()


if __name__ == '__main__':
    main()
