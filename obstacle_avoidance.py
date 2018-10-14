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
# vehicle = connect("192.168.0.19:14555", wait_ready=True)
DISTANCE_TO_OBSTACLE = 0
distance_lock = threading.Lock()
distance_udpated = threading.Event()


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
        # get lock
        distance_lock.acquire()

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
        DISTANCE_TO_OBSTACLE = round(duration * 17150.0, 2)

        # release
        distance_lock.release()
        distance_udpated.set()

        time.sleep(0.5)


def print_distance():
    """
    Print distance to obstacle.
    """
    while (True):
        global DISTANCE_TO_OBSTACLE
        distance_udpated.wait()
        distance_lock.acquire()
        print("Distance to obstacle: "+str(DISTANCE_TO_OBSTACLE))
        distance_lock.release()
        time.sleep(0.5)


def move_forward(v, duration=1):
    return send_body_ned_velocity(vehicle, v, 0, 0, duration)


def move_right(v, duration=1):
    return send_body_ned_velocity(vehicle, 0, v, 0, duration)


def move_left(v, duration=1):
    return send_body_ned_velocity(vehicle, 0, -v, 0, duration)


def obstacle_avoidance():
    """
    Turn right/left to avoide obstcle.
    """
    while (True):
        global DISTANCE_TO_OBSTACLE
        if distance_udpated.set():

            distance_lock.acquire()
            if DISTANCE_TO_OBSTACLE > 10 and DISTANCE_TO_OBSTACLE < 100:
                move_left(0.5)
            elif DISTANCE_TO_OBSTACLE > 100:
                move_forward(0.5)
                # pass
            distance_lock.release()

            distance_udpated.clear()
            continue
        move_forward(0.5)


def main():
    arm_and_take_off(vehicle, 3)
    vehicle.groundspeed = 1  # m/s
    threads = []
    for func in [distance_measure,
                 obstacle_avoidance,
                 print_distance]:
        threads.append(threading.Thread(target=func))
        threads[-1].setDaemon(True)
        threads[-1].start()
    # for thread in threads:
    #     thread.join()

    while (True):
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            break


if __name__ == '__main__':
    main()
