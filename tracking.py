#!/usr/bin/env python
# -*- coding:utf-8 -*-

from dronekit import connect, VehicleMode
from aerodrone import *
import math
import json
import serial
import argparse
import threading
import sys

# connect to vehicle
# vehicle = connect("192.168.0.24:14555", wait_ready=True)
# vehicle = connect("/dev/ttyACM0", wait_ready=True)
vehicle = connect("127.0.0.1:14555", wait_ready=True)
# ser = serial.Serial("/dev/serial0", 115200, timeout=0.5)
target_pos = OpenMV_POS()
target_pos_lock = threading.Lock()
target_pos_updated = threading.Event()
got_target = threading.Event()
found_target = threading.Event()


def help():
    """
    Print help information.
    """
    parser = argparse.ArgumentParser(description='Process some arguments.')
    parser.add_argument('--sitl')


def find_target():
    """
    Draw circle and look for target.
    """
    while (True):
        target_pos_lock.acquire()
        # ser.flushInput()
        # line = ser.readline()

        # For debug
        line = json.dumps({"cx": 100, "cy": 80})
        try:
            openmv_data = json.loads(line)
        except ValueError:
            found_target.clear()
            continue
        openmv_cx = openmv_data[u"cx"]
        openmv_cy = openmv_data[u"cy"]

        theta = vehicle.heading

        rotate_theta = (theta + 90) % 360

        dx = openmv_cx-80
        dy = openmv_cy-60

        # rotation matrix
        # [cos_theta, -sin_theta]
        # [sin_theta,  cos_theta]
        dest_x = math.cos(rotate_theta/180.0*math.pi)*dx - \
            math.sin(rotate_theta/180.0*math.pi)*dy
        dest_y = math.sin(rotate_theta/180.0*math.pi)*dx + \
            math.cos(rotate_theta/180.0*math.pi)*dy

        # update target_pos
        target_pos.x = dest_x
        target_pos.y = dest_y
        target_pos_lock.release()

        print("Destation: "+str(dest_x)+", "+str(dest_y))

        # wake up other threads
        found_target.set()

    return None


def tracking():
    """
    Follow target.
    """
    # get_close_to_target
    vehicle.groundspeed = 3.0  # m/s
    target_pos_lock.acquire()
    goto(vehicle, 0.15*target_pos.x, 0.15*target_pos.y)
    target_pos_lock.release()

    # follow
    vehicle.groundspeed = 1.5  # m/s
    while (True):
        found_target.wait()
        goto(vehicle, 0.15*target_pos.x, 0.15*target_pos.y)
        found_target.clear()
    return None


def draw_circle(radis=6.0):
    """
    Move around to find target.
    """
    vehicle.groundspeed = 3  # m/s
    North = [-radis/2.0, -radis, -radis/2.0,
             radis/2.0, radis, radis/2.0]
    East = [radis/2*1.732, 0, -radis/2*1.732,
            -radis/2*1.732, 0, radis/2*1.732]
    goto(vehicle, radis, 0)
    found_target.wait(2)
    if found_target.is_set():
        return None
    while(True):
        for x, y in zip(North, East):
            goto(vehicle, x, y)
            found_target.wait(2)
            if found_target.is_set():
                return None


def main():
    # take off
    arm_and_take_off(vehicle, 5)
    vehicle.groundspeed = 3  # m/s

    threads = []
    threads.append(threading.Thread(target=find_target))
    threads[-1].setDaemon(True)
    threads[-1].start()

    draw_circle()
    print("Found target, start tracking...")

    for func in [tracking]:
        threads.append(threading.Thread(target=func))
        threads[-1].setDaemon(True)
        threads[-1].start()

    while(True):
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            vehicle.mode = VehicleMode("LAND")
            vehicle.close()
            sys.exit()


if __name__ == '__main__':
    main()
