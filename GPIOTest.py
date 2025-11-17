#!/usr/bin/env python3
import serial
import time
import numpy as np
import RPi.GPIO as GPIO
import csv
from gpiozero import RotaryEncoder

#assign parameter values
ppr = 48 #pulse per rev for encoder 
tsample = 0.1 # sampling period for encoder reading
tdisp = 0.5 # freqency to show encoder reading on terminal
tstop = 5

Kp = 0.163 # proportional gain
Ki = 0.024 # integral gain
Kd = 0.105 # derivative gain

# create encoder object on GPIO pins 17 and 18
encoder = RotaryEncoder(17, 18, max_steps=0)

# Define motor pins forward pin (in1) 22, backward (in2) 23, PWM (en) 24
in1 = 22
in2 = 23
en = 24

d = 2.7559055

# initialize values
velCurr = 0
posCurr = 0
posLast = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)
motor_fwd_pwm = GPIO.PWM(in1, 100)
GPIO.output(in2,GPIO.LOW)
# GPIO.output(in1,GPIO.HIGH)
# GPIO.output(in2,GPIO.LOW)
p=GPIO.PWM(en,100)
p.start(100)
desiredV = 5
output = desiredV
motor_fwd_pwm.start(50)

