

#GPIO's with pwm port is 23, 24, 26
import RPi.GPIO as GPIO
import time
import sys

MOTOR_PINS = [23, 24, 26]
motors = [0, 0, 0]

def setup_motors():
    GPIO.setmode(GPIO.BOARD)
    for motor_pin in MOTOR_PINS:
        GPIO.setup(motor_pin, GPIO.OUT)

    for idx in range(len(MOTOR_PINS)):
        motors[idx] = GPIO.PWM(motor, 50)

    # motor_1 = GPIO.PWM(MOTOR_PIN_1, 50)
    # motor_2 = GPIO.PWM(MOTOR_PIN_2, 50)
    # motor_3 = GPIO.PWM(MOTOR_PIN_3, 50)

    #Testing moving the motor 1
    for motor in motors:
        motor.start(0)
    # motor_1.start(0)
    # motor_2.start(0)
    # motor_3.start(0)
    time.sleep(10)
    print("Motors are setup")

# Writes values of pwm_control_vector to each motor
def write_pwm(pwm_control_vector):
    for idx in range(len(pwm_control_vector)):
        motors[idx].ChangeDutyCycle(pwm_control_vector[idx] / 10.0)


#Receive pwm value from 0 to 100 and writes to motor (i.e. 3 is 30%, 4.6 = 46%)
def write_pwm_to_id(motor_id, pwm_value):
    if(motor_id - 1 < 0):
        print("Motor ids start from 1... not 0")
        sys.exit(0)
    motors[motor_id - 1].ChangeDutyCycle(pwm_value / 10.0)
    print(str(pwm_value) + " value written to motor " + str(motor_id))
