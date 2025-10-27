#!/usr/bin/env python3
"""
28BYJ-48 Stepper Motor Test via ULN2003
IN1–IN4: GPIO 17,18,27,22
"""

import time
import RPi.GPIO as GPIO

IN1, IN2, IN3, IN4 = 17, 18, 27, 22
PINS = (IN1, IN2, IN3, IN4)

SEQ = [
    (1,1,0,0),
    (0,1,1,0),
    (0,0,1,1),
    (1,0,0,1),
]

DELAY = 0.012   # slow = more torque
STEPS = 256     # ~⅛ revolution (full-step)

GPIO.setmode(GPIO.BCM)
for p in PINS:
    GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

def step(seq, delay=DELAY, steps=STEPS, reverse=False):
    if reverse:
        seq = list(reversed(seq))
    for _ in range(steps):
        for pat in seq:
            for pin, val in zip(PINS, pat):
                GPIO.output(pin, GPIO.HIGH if val else GPIO.LOW)
            time.sleep(delay)
    for p in PINS:
        GPIO.output(p, GPIO.LOW)

try:
    print("Forward…")
    step(SEQ)
    time.sleep(0.5)
    print("Reverse…")
    step(SEQ, reverse=True)
except KeyboardInterrupt:
    pass
finally:
    for p in PINS:
        GPIO.output(p, GPIO.LOW)
    GPIO.cleanup()
    print("GPIO cleaned up.")
