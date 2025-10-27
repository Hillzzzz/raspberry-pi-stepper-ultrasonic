#!/usr/bin/env python3
"""
Ultra Step — HC-SR04 + 28BYJ-48 Stepper via ULN2003
When distance < threshold, rotate motor N steps.
"""

import time, sys, signal
import RPi.GPIO as GPIO

# GPIO pins (BCM)
TRIG, ECHO = 23, 24            # ECHO via divider
IN1, IN2, IN3, IN4 = 17, 18, 27, 22
STEPPER_PINS = (IN1, IN2, IN3, IN4)

# Behavior settings
THRESHOLD_CM = 20.0
STEPS_ON_TRIGGER = 256         # 2048 ≈ full turn
MOTOR_STEP_DELAY = 0.012
ROTATE_DIRECTION = 1           # +1 forward, -1 reverse

# Sensor tuning
SPD = 34300.0
PING_GAP = 0.06
START_TO = 0.02
END_TO = 0.03
READS = 5

SEQ_FULL = [
    (1,1,0,0),
    (0,1,1,0),
    (0,0,1,1),
    (1,0,0,1),
]

_RUNNING = True
_CLEANED = False

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO, GPIO.IN)
    for p in STEPPER_PINS:
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

def cleanup():
    global _CLEANED
    if _CLEANED:
        return
    try:
        for p in STEPPER_PINS:
            GPIO.output(p, GPIO.LOW)
        GPIO.output(TRIG, GPIO.LOW)
    finally:
        GPIO.cleanup()
        _CLEANED = True

def sig_handler(sig, frm):
    global _RUNNING
    _RUNNING = False

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

def read_distance():
    vals = []
    for _ in range(READS):
        d = ping_cm()
        if d and 2.0 <= d <= 400.0:
            vals.append(d)
        time.sleep(PING_GAP)
    if not vals:
        return None
    vals.sort()
    n = len(vals)
    return vals[n//2] if n % 2 else 0.5*(vals[n//2-1] + vals[n//2])

def stepper_steps(n, direction=1):
    seq = SEQ_FULL if direction >= 0 else list(reversed(SEQ_FULL))
    for _ in range(n):
        for pat in seq:
            for pin, val in zip(STEPPER_PINS, pat):
                GPIO.output(pin, GPIO.HIGH if val else GPIO.LOW)
            time.sleep(MOTOR_STEP_DELAY)
    for p in STEPPER_PINS:
        GPIO.output(p, GPIO.LOW)

def main():
    global _RUNNING
    print("Ultra Step started. Ctrl+C to exit.")
    setup()
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)
    try:
        while _RUNNING:
            d = read_distance()
            if d is None:
                print("Distance: ---")
            else:
                print(f"Distance: {d:.1f} cm")
                if d < THRESHOLD_CM:
                    print(f"< {THRESHOLD_CM} cm → rotating {STEPS_ON_TRIGGER} steps")
                    stepper_steps(STEPS_ON_TRIGGER, ROTATE_DIRECTION)
            time.sleep(0.1)
    finally:
        cleanup()
        print("GPIO cleaned up. Bye.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        cleanup()
        print("\nStopped by user.")
        sys.exit(0)
