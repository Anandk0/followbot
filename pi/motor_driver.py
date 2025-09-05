# motor_driver.py -- Direct L298N motor control for Raspberry Pi
import RPi.GPIO as GPIO
import time

class MotorDriver:
    def __init__(self, ena=18, in1=24, in2=23, enb=19, in3=21, in4=20):
        self.ena = ena  # Motor A enable (PWM)
        self.in1 = in1  # Motor A direction 1
        self.in2 = in2  # Motor A direction 2
        self.enb = enb  # Motor B enable (PWM)
        self.in3 = in3  # Motor B direction 1
        self.in4 = in4  # Motor B direction 2
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.ena, self.in1, self.in2, self.enb, self.in3, self.in4], GPIO.OUT)
        
        self.pwm_a = GPIO.PWM(self.ena, 1000)
        self.pwm_b = GPIO.PWM(self.enb, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        
        self.stop()

    def send(self, cmd: dict):
        """Process movement commands"""
        action = cmd.get('action')
        speed = cmd.get('speed', 50)
        
        if action == 'forward':
            self.forward(speed)
        elif action == 'backward':
            self.backward(speed)
        elif action == 'left':
            self.left(speed)
        elif action == 'right':
            self.right(speed)
        elif action == 'stop':
            self.stop()
        
        print(f"[MotorDriver] Command: {cmd}")

    def forward(self, speed=50):
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(speed)
        self.pwm_b.ChangeDutyCycle(speed)

    def backward(self, speed=50):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(speed)
        self.pwm_b.ChangeDutyCycle(speed)

    def left(self, speed=50):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        GPIO.output(self.in3, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(speed)
        self.pwm_b.ChangeDutyCycle(speed)

    def right(self, speed=50):
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(speed)
        self.pwm_b.ChangeDutyCycle(speed)

    def stop(self):
        GPIO.output([self.in1, self.in2, self.in3, self.in4], GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)

    def close(self):
        self.stop()
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()