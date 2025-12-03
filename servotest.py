from gpiozero import AngularServo
import pigpio
from time import sleep

pi = pigpio.pi()

servo_gpio = 22

# Set the GPIO pin to output mode
pi.set_mode(servo_gpio, pigpio.OUTPUT)

# Set the PWM frequency (servos typically use 50Hz)
pi.set_PWM_frequency(servo_gpio, 50)

# Set the PWM range (servos typically use a range of 20000 for 50Hz)
# pi.set_PWM_range(servo_gpio, 1001)

try:
    while True:
        # Set servo to anti-clockwise position (e.g., 1000us pulsewidth)
        pi.set_servo_pulsewidth(servo_gpio, 500)
        sleep(1)

        # Set servo to middle position (e.g., 1500us pulsewidth)
        pi.set_servo_pulsewidth(servo_gpio, 1500)
        sleep(1)

        # Set servo to clockwise position (e.g., 2000us pulsewidth)
        pi.set_servo_pulsewidth(servo_gpio, 2500)
        sleep(1)

except KeyboardInterrupt:
    # Turn off the servo and stop the pigpio connection on exit
    pi.set_servo_pulsewidth(servo_gpio, 0)
    pi.stop()

# Specify the GPIO pin and pulse widths
# servo = AngularServo(22, min_pulse_width=0.0006, max_pulse_width=0.0025)

# while True:
#     servo.angle = 90   # Set to 90 degrees
#     sleep(2)           # Wait for 2 seconds
#     servo.angle = 0    # Set to 0 degrees
#     sleep(2)
#     servo.angle = -90  # Set to -90 degrees
#     sleep(2)