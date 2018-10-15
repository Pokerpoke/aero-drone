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


# vehicle = connect("127.0.0.1:14555", wait_ready=True)
vehicle = connect("192.168.0.17:14555", wait_ready=True)
distance_to_obstacle = 0.0
distance_lock = threading.Lock()
distance_udpated = threading.Event()
current_pos_y = 0.0
current_pos_x = 0.0


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

    A = 10

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

        global distance_to_obstacle
        distance_to_obstacle_new = round(duration * 17150.0, 2)

        distance_lock.acquire()
        if distance_to_obstacle_new < 120 and \
            distance_to_obstacle < 120 and \
            abs(distance_to_obstacle_new - distance_to_obstacle) > A:
            distance_to_obstacle = distance_to_obstacle
        else:
            distance_to_obstacle = distance_to_obstacle_new

        print("Distance to obstacle: "+str(distance_to_obstacle))

        # release lock
        distance_lock.release()
        distance_udpated.set()


def print_distance():
    """
    Print distance to obstacle.
    """
    while (True):
        global distance_to_obstacle
        distance_udpated.wait()
        distance_lock.acquire()
        print("Distance to obstacle: "+str(distance_to_obstacle))
        distance_lock.release()
        time.sleep(0.1)


def move_forward(v, duration=1):
    return send_body_ned_velocity(vehicle, v, 0, 0, duration)


def move_right(v, duration=1):
    return send_body_ned_velocity(vehicle, 0, v, 0, duration)


def move_left(v, duration=1):
    return send_body_ned_velocity(vehicle, 0, -v, 0, duration)

def back_to_center():
    global current_pos_x
    while (current_pos_x > 0):
        move_right(0.5)
        current_pos_x = current_pos_x - 0.5
    print("Current X: " + str(current_pos_x))


def obstacle_avoidance():
    """
    Turn right/left to avoide obstcle.
    """
    global distance_to_obstacle
    global current_pos_x
    global current_pos_y
    while (True):
        if current_pos_y > 30:
            print("Arrived destination")
            back_to_center()
            vehicle.mode = VehicleMode("LAND")
            vehicle.close()
            sys.exit()
            break

        if distance_udpated.is_set():
            # distance_lock.acquire()
            if distance_to_obstacle < 100 and distance_to_obstacle > 10:
                move_left(0.5)  # m/s
                current_pos_x = current_pos_x + 0.5
                print("Current X: " + str(current_pos_x))
                if current_pos_x >= 2.0:
                    back_to_center()
                    time.sleep(0.1)
                    move_right(0.5) # m/s
                    time.sleep(0.1)
                    move_forward(2)
                    time.sleep(0.1)
                    move_left(0.5)
                    time.sleep(0.1)
                    current_pos_y = current_pos_y + 2
            elif distance_to_obstacle > 100:
                if current_pos_x == 0:
                    move_forward(0.5)
                    current_pos_y = current_pos_y + 0.5
                elif current_pos_x > 0:
                    move_forward(2)  # m/s
                    current_pos_y = current_pos_y + 2
                time.sleep(0.1)
                back_to_center()
                print("Current Y: " + str(current_pos_y))
                pass

            # distance_lock.release()
            distance_udpated.clear()
            continue


def main():
    arm_and_take_off(vehicle, 3)
    vehicle.groundspeed = 1  # m/s
    threads = []
    # for func in [distance_measure,
                 # obstacle_avoidance,
                 # print_distance]:
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
