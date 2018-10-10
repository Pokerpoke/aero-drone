#!/usr/bin/env python
# -*- coding:utf-8 -*-

import threading
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
TRIG = 18
ECHO = 16
GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

DISTANCE_TO_OBSTACLE = 0
lock_distance = threading.Lock()


def distance_measure():
    """
    Get distance to obstacle.
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

        global DISTANCE_TO_OBSTACLE
        # Lock
        lock_distance.acquire()
        try:
            DISTANCE_TO_OBSTACLE = round(duration * 17150, 2)
        finally:
            # release
            lock_distance.release()
        time.sleep(0.1)


def print_distance():
    while (True):
        global DISTANCE_TO_OBSTACLE
        lock_distance.acquire()
        print(DISTANCE_TO_OBSTACLE)
        lock_distance.release()
        time.sleep(0.5)


def main():
    t1 = threading.Thread(target=distance_measure)
    t2 = threading.Thread(target=print_distance)
    t1.start()
    t2.start()
    t1.join()
    t2.join()


if __name__ == '__main__':
    main()
