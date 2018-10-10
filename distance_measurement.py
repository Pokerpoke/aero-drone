#!/usr/bin/env python
# -*- coding:utf-8 -*-

import threading

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


def print_distance():
    while (True):
        global DISTANCE_TO_OBSTACLE
        lock_distance.acquire()
        print(DISTANCE_TO_OBSTACLE)
        lock_distance.release()


def main():
    t1 = threading.Thread(target=distance_measure)
    t1.start()
    t1.join()


if __name__ == '__main__':
    main()
