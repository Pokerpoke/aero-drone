import sensor
import image
import time
import utime
import pyb
from pyb import UART
import json

threshold_index = 0

thresholds = [(48, 92, -64, 10, -33, -59),
              (51, 79, 29, 80, 3, 53),
              (30, 100, 15, 127, 15, 127),
              (0, 30, 0, 64, -128, 0)]

sensor.reset()  # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565)  # use RGB565.
sensor.set_framesize(sensor.QVGA)  # use QVGA for speed.
sensor.skip_frames(10)  # Let new settings take affect.
# sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # turn this off.
clock = time.clock()  # Tracks FPS.

led = pyb.LED(2)

uart = UART(3, 115200)


def find_max(blobs):
    max_size = 0
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob = blob
            max_size = blob.pixels()
    return max_blob


while(True):
    clock.tick()  # Track elapsed milliseconds between snapshots().
    led.on()
    img = sensor.snapshot()  # Take a picture and return the image.
    #img.median(1, percentile=0.5)
    threshold = thresholds[threshold_index]

    blobs = img.find_blobs([threshold])
    if blobs:
        max_blob = find_max(blobs)
        print('Blob numbers :', len(blobs))
        img.draw_rectangle(max_blob.rect())
        img.draw_cross(max_blob.cx(), max_blob.cy())

        output_str = json.dumps({
            'cx': max_blob.cx(),
            'cy': max_blob.cy()
        })
        print('Send to UART:', output_str)
        uart.write(output_str+'\r\n')
    else:
        print('not found!')
    print("FPS:", clock.fps())
