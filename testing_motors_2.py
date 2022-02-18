# sudo systemctl enable pigpiod To automate running the daemon at boot time, run:
# sudo pigpiod --> This is for single-session-use and will not persist after a reboot

# Using the pigpio library which enables hardware pwm instead of software pwm (which has much jitter)
import time
import pigpio
import RPi.GPIO as GPIO

ESC_MOTOR_1_GPIO = 13

GPIO.setmode(GPIO.BCM)

pi = pigpio.pi() # pi1 accesses the local Pi's GPIO

pi.set_servo_pulsewidth(ESC_MOTOR_1_GPIO, 1000)
time.sleep(2)
pi.set_servo_pulsewidth(ESC_MOTOR_1_GPIO, 2000)
time.sleep(2)
pi.set_servo_pulsewidth(ESC_MOTOR_1_GPIO, 1000)
time.sleep(2)

# pi.set_PWM_dutycycle(4,   0) # PWM off
# time.sleep(5)

# pi.set_PWM_dutycycle(4,   50)
# time.sleep(3)

# pi.set_PWM_dutycycle(4,   20)
# time.sleep(3)

# pi.set_PWM_dutycycle(4,   100)
# time.sleep(3)

# pi.set_PWM_dutycycle(4,   200)
# time.sleep(1.5)

# pi.set_PWM_dutycycle(4,   10)
# time.sleep(3)