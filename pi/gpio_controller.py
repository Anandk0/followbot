import time
import threading
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False

class GPIOController:
    def __init__(self):
        self.has_gpio = HAS_GPIO
        if not HAS_GPIO:
            print("RPi.GPIO not available - using mock GPIO")
            return
            
        # Motor pins (BCM numbering)
        self.MOTOR_L1 = 18    # Left motor direction 1
        self.MOTOR_L2 = 19    # Left motor direction 2
        self.MOTOR_R1 = 20    # Right motor direction 1
        self.MOTOR_R2 = 21    # Right motor direction 2
        self.MOTOR_L_PWM = 12 # Left motor PWM
        self.MOTOR_R_PWM = 13 # Right motor PWM
        
        # Ultrasonic pins
        self.TRIG_PIN = 23
        self.ECHO_PIN = 24
        
        self._setup_gpio()
        self._telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._telemetry_running = True
        self._telemetry_thread.start()
        self.last_distance = 999.0
        
    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Motor pins
        GPIO.setup(self.MOTOR_L1, GPIO.OUT)
        GPIO.setup(self.MOTOR_L2, GPIO.OUT)
        GPIO.setup(self.MOTOR_R1, GPIO.OUT)
        GPIO.setup(self.MOTOR_R2, GPIO.OUT)
        GPIO.setup(self.MOTOR_L_PWM, GPIO.OUT)
        GPIO.setup(self.MOTOR_R_PWM, GPIO.OUT)
        
        # PWM setup
        self.left_pwm = GPIO.PWM(self.MOTOR_L_PWM, 1000)  # 1kHz
        self.right_pwm = GPIO.PWM(self.MOTOR_R_PWM, 1000)
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        
        # Ultrasonic pins
        GPIO.setup(self.TRIG_PIN, GPIO.OUT)
        GPIO.setup(self.ECHO_PIN, GPIO.IN)
        
        self.stop_motors()
        
    def set_motor_speeds(self, left, right):
        """Set motor speeds: -100 to +100"""
        if not self.has_gpio:
            print(f"MOCK GPIO: Motors L={left}, R={right}")
            return
            
        # Left motor
        if left > 0:
            GPIO.output(self.MOTOR_L1, GPIO.HIGH)
            GPIO.output(self.MOTOR_L2, GPIO.LOW)
        elif left < 0:
            GPIO.output(self.MOTOR_L1, GPIO.LOW)
            GPIO.output(self.MOTOR_L2, GPIO.HIGH)
        else:
            GPIO.output(self.MOTOR_L1, GPIO.LOW)
            GPIO.output(self.MOTOR_L2, GPIO.LOW)
        
        # Right motor
        if right > 0:
            GPIO.output(self.MOTOR_R1, GPIO.HIGH)
            GPIO.output(self.MOTOR_R2, GPIO.LOW)
        elif right < 0:
            GPIO.output(self.MOTOR_R1, GPIO.LOW)
            GPIO.output(self.MOTOR_R2, GPIO.HIGH)
        else:
            GPIO.output(self.MOTOR_R1, GPIO.LOW)
            GPIO.output(self.MOTOR_R2, GPIO.LOW)
            
        # Set PWM duty cycle
        self.left_pwm.ChangeDutyCycle(abs(left))
        self.right_pwm.ChangeDutyCycle(abs(right))
        
    def stop_motors(self):
        """Stop all motors"""
        if not self.has_gpio:
            print("MOCK GPIO: Motors stopped")
            return
            
        GPIO.output(self.MOTOR_L1, GPIO.LOW)
        GPIO.output(self.MOTOR_L2, GPIO.LOW)
        GPIO.output(self.MOTOR_R1, GPIO.LOW)
        GPIO.output(self.MOTOR_R2, GPIO.LOW)
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)
        
    def get_distance(self):
        """Get ultrasonic distance in cm"""
        if not self.has_gpio:
            return 50.0  # Mock distance
            
        try:
            GPIO.output(self.TRIG_PIN, GPIO.LOW)
            time.sleep(0.000002)
            GPIO.output(self.TRIG_PIN, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.TRIG_PIN, GPIO.LOW)
            
            # Wait for echo
            timeout = time.time() + 0.03  # 30ms timeout
            while GPIO.input(self.ECHO_PIN) == GPIO.LOW and time.time() < timeout:
                pulse_start = time.time()
                
            while GPIO.input(self.ECHO_PIN) == GPIO.HIGH and time.time() < timeout:
                pulse_end = time.time()
                
            if 'pulse_start' in locals() and 'pulse_end' in locals():
                duration = pulse_end - pulse_start
                distance = (duration * 34300) / 2  # Speed of sound
                return min(distance, 400.0)  # Cap at 4m
            return 999.0
        except:
            return 999.0
            
    def _telemetry_loop(self):
        """Background thread for distance measurement"""
        while self._telemetry_running:
            self.last_distance = self.get_distance()
            time.sleep(0.1)  # 10Hz
            
    def get_telemetry(self):
        """Get current telemetry data"""
        return {
            "telemetry": {
                "distance_cm": self.last_distance,
                "bat_v": 3.3  # Placeholder
            }
        }
        
    def cleanup(self):
        """Cleanup GPIO resources"""
        self._telemetry_running = False
        if self.has_gpio:
            self.stop_motors()
            GPIO.cleanup()