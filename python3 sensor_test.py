#!/usr/bin/env python3
"""
HC-SR04 Ultrasonic Distance Test
TRIG: GPIO 23
ECHO: GPIO 24 (through 10k/20k divider)
"""

import time
import RPi.GPIO as GPIO

TRIG = 23
ECHO = 24
SPD = 34300.0
START_TO = 0.02
END_TO = 0.03

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

def ping_cm():
    GPIO.output(TRIG, GPIO.LOW)
    time.sleep(0.002)
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(10e-6)
    GPIO.output(TRIG, GPIO.LOW)

    t0 = time.time()
    while GPIO.input(ECHO) == 0:
        if time.time() - t0 > START_TO:
            return None
    ts = time.time()

    t1 = time.time()
    while GPIO.input(ECHO) == 1:
        if time.time() - t1 > END_TO:
            return None
    te = time.time()

    return (SPD * (te - ts)) / 2.0

try:
    while True:
        d = ping_cm()
        if d:
            print(f"Distance: {d:.1f} cm")
        else:
            print("Distance: ---")
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")
