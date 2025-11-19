#!/usr/bin/env python3
import serial
import time
import numpy as np
from sendStringScript import sendString
import RPi.GPIO as GPIO
import csv
from gpiozero import RotaryEncoder
from enum import Enum

class RobotStates(Enum):
    INIT = 0,
    DEAD_RECKNONING = 1,
    LINE_FOLLOWING = 2,
    AIMING = 3,
    SHOOTING = 4

left_encoder_a = 24
left_encoder_b = 25

right_encoder_a = 22
right_encoder_b = 23

encoder_left = RotaryEncoder(a=left_encoder_a, b=left_encoder_b, max_steps=0)
encoder_right = RotaryEncoder(a=right_encoder_a, b=right_encoder_b, max_steps=0)

#assign parameter values
ppr = 48 #pulse per rev for encoder 
tsample = 0.1 # sampling period for encoder reading
tdisp = 0.5 # freqency to show encoder reading on terminal
tstop = 5

d = 2.7559055

# initialize values
right_velCurr = 0
left_velCurr = 0
left_posCurr = 0
right_posCurr = 0
left_posLast = 0
right_posLast = 0
tcurr = 0
tprev = 0
tstart = time.perf_counter()
GPIO.setmode(GPIO.BCM)

Kp_l = 0.6 # proportional gain
Ki_l = 0.0 # integral gain
Kd_l = 0.0 # derivative gain

Kp_r = 0.6 # proportional gain
Ki_r = 0.0 # integral gain
Kd_r = 0.0 # derivative gain

MAX_VEL = 23

prevError_l = 0
prevError_r = 0
integral = 0
lastIntegralReset = 0

UPPPER_BOUND = 100
LOWER_BOUND = -100

controlSignal_l = 0
controlSignal_r = 0

def scaleControlSignal(signal):

    scaledSignal = (signal / MAX_VEL) * UPPPER_BOUND # scaled to 400 max for motor driver 
    return max(LOWER_BOUND, min(UPPPER_BOUND, scaledSignal))

def clipVelocity(vel, max_vel):
    return max(0, min(MAX_VEL, vel))
	
def set_motor_velocity(des_left_vel, des_right_vel, left_posCurr, right_posCurr, left_posLast, right_posLast, tcurr, tprev): # in rev/s
    global prevError_l
    global prevError_r
    global integral
    global lastIntegralReset
    
    global controlSignal_l
    global controlSignal_r

    des_left_vel = clipVelocity(des_left_vel, MAX_VEL)
    des_right_vel = clipVelocity(des_right_vel, MAX_VEL)

    tsample = tcurr - tprev # basically delta t
    
    # in rev/sec
    left_velCurr = ((left_posCurr - left_posLast)/(tcurr - tprev))/ppr
    right_velCurr = ((right_posCurr - right_posLast)/(tcurr - tprev))/ppr

    error_l = des_left_vel - left_velCurr
    integral += error_l * tsample
    derivative = (error_l - prevError_l) / tsample
    controlSignal_l_loc = Kp_l * error_l + Ki_l * integral + Kd_l * derivative

    error_r = des_right_vel - right_velCurr
    derivative_r = (error_r - prevError_r) / tsample
    controlSignal_r_loc = Kp_r * error_r + Ki_r * integral + Kd_r * derivative_r

    print("Error Left: ", error_l)
    print("Error Right: ", error_r)

    if tcurr - lastIntegralReset >= 5:
        integral = 0
        lastIntegralReset = tcurr
    prevError_l = error_l
    prevError_r = error_r

    print("delta time: ", tsample)
    controlSignal_l += scaleControlSignal(controlSignal_l_loc)
    controlSignal_r += scaleControlSignal(controlSignal_r_loc)
    print("Control Signal Left: ", controlSignal_l)
    print("Control Signal Right: ", controlSignal_r)
    sendString('/dev/ttyACM0',115200,'<'+str(controlSignal_l)+','+str(controlSignal_r)+ ',' + str(currentState.value)+'>',0.0001)

    timesteps = 0
    if ser.in_waiting > 0:  #we wait until the arduino has sent something to us before we try to read anything from the serial port.
            line = ser.readline().decode('utf-8')
            print("Raw line: " , line) 
            line=line.split(',')
            if (len(line) < 4):
                return

    # print("Left Encoder: ", left_posCurr)
    # print("Right Encoder: ", right_posCurr)

    print("Left Velocity (rev/s): ", left_velCurr)
    print("Right Velocity (rev/s): ", right_velCurr)

if __name__ == '__main__':
    ser=serial.Serial('/dev/ttyACM0',115200)
    ser.reset_input_buffer() #clears anything the arduino has been sending while the Rpi isnt prepared to recieve.

    firstIteration = True
    prevTime = time.time()
    currentTime = time.time()
    currentState = RobotStates.INIT
    integral = 0    
    leftMotor=int(100)
    rightMotor=int(100)



    while True:
        tcurr = time.perf_counter() - tstart

        rightMotor = 0        
        leftMotor = 0

        left_posCurr = encoder_left.steps
        right_posCurr = encoder_right.steps

        set_motor_velocity(leftMotor, rightMotor, left_posCurr, right_posCurr, left_posLast, right_posLast, tcurr, tprev)

        tprev = tcurr
        right_posLast = right_posCurr
        left_posLast = left_posCurr

GPIO.cleanup()
left_encoder_a.close()
left_encoder_b.close()
