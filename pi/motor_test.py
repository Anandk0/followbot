import RPi.GPIO as GPIO
import time

# Pin setup
IN1, IN2, ENA = 16, 17, 12  # Motor A
IN3, IN4, ENB = 20, 21, 13  # Motor B

GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)

# PWM setup
pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)

def test_motor_a():
    print("Testing Motor A forward...")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(50)
    time.sleep(2)
    pwm_a.ChangeDutyCycle(0)

def test_motor_b():
    print("Testing Motor B forward...")
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_b.ChangeDutyCycle(50)
    time.sleep(2)
    pwm_b.ChangeDutyCycle(0)

try:
    test_motor_a()
    test_motor_b()
finally:
    GPIO.cleanup()