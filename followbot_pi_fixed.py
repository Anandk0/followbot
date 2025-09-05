#!/usr/bin/env python3
"""
FollowBot Pi: Face-following car on Raspberry Pi (L298N)

A complete face-following robot implementation using:
- MediaPipe for real-time face detection
- L298N motor driver for differential drive control
- OpenCV for camera input and visualization
- Cross-platform GPIO abstraction (RPi.GPIO or dry-run mode)

Features:
- Robust face detection with confidence thresholding
- Differential steering based on face position
- Safety timeout when no face is detected
- Dry-run mode for development on non-Pi systems
- Comprehensive error handling and resource cleanup

Usage:
  python3 followbot_pi.py            # Run on Raspberry Pi
  python3 followbot_pi.py --dry-run  # Test mode (no GPIO)
  python3 followbot_pi.py --help     # Show all options

Hardware Requirements:
- Raspberry Pi with camera
- L298N motor driver
- Two DC motors
- GPIO connections as per pin configuration

Dependencies:
  pip install opencv-python mediapipe
  # On Raspberry Pi only:
  pip install RPi.GPIO

Author: Senior Developer
Version: 2.0
License: MIT
"""

import sys
import time
import argparse
import logging
from typing import Optional, Tuple, Union

import cv2
import mediapipe as mp

# Constants
SPEED_MIN = -100
SPEED_MAX = 100
DUTY_MIN = 0.0
DUTY_MAX = 100.0
DEFAULT_PWM_FREQ = 1000
DEFAULT_CAMERA_WIDTH = 640
DEFAULT_CAMERA_HEIGHT = 480

# -----------------------------
# GPIO Backend (RPi or DryRun)
# -----------------------------

class GPIOBase:
    """Base GPIO interface defining common constants and methods."""
    BCM = 'BCM'
    BOARD = 'BOARD'
    OUT = 'OUT'
    HIGH = 1
    LOW = 0

    def setmode(self, mode: str) -> None: pass
    def setwarnings(self, flag: bool) -> None: pass
    def setup(self, pin: int, direction: str) -> None: pass
    def output(self, pin: int, value: int) -> None: pass
    def PWM(self, pin: int, freq: int) -> 'PWMInterface': return DummyPWM()
    def cleanup(self) -> None: pass


class PWMInterface:
    """Interface for PWM control."""
    def start(self, duty: float) -> None: pass
    def ChangeDutyCycle(self, duty: float) -> None: pass
    def stop(self) -> None: pass


class DummyPWM(PWMInterface):
    """Mock PWM implementation for dry-run mode."""
    def __init__(self) -> None:
        self._duty_cycle = 0.0
        
    def start(self, duty: float) -> None:
        self._duty_cycle = max(DUTY_MIN, min(DUTY_MAX, duty))
        
    def ChangeDutyCycle(self, duty: float) -> None:
        self._duty_cycle = max(DUTY_MIN, min(DUTY_MAX, duty))
        
    def stop(self) -> None:
        self._duty_cycle = 0.0


class DryRunGPIO(GPIOBase):
    """Mock GPIO implementation for testing on non-Pi systems."""
    
    def __init__(self) -> None:
        self._mode: Optional[str] = None
        self._pins: dict = {}
        logging.info("Initialized DryRun GPIO backend")

    def setmode(self, mode: str) -> None:
        if mode not in (self.BCM, self.BOARD):
            raise ValueError(f"Invalid GPIO mode: {mode}")
        self._mode = mode
        logging.info(f"[DRY-RUN] GPIO mode set: {mode}")

    def setwarnings(self, flag: bool) -> None:
        logging.debug(f"[DRY-RUN] GPIO warnings: {flag}")

    def setup(self, pin: int, direction: str) -> None:
        if not isinstance(pin, int) or pin < 0:
            raise ValueError(f"Invalid pin number: {pin}")
        self._pins[pin] = {'dir': direction, 'val': 0}
        logging.info(f"[DRY-RUN] Setup pin {pin} as {direction}")

    def output(self, pin: int, value: int) -> None:
        if pin not in self._pins:
            self.setup(pin, self.OUT)
        if value not in (self.HIGH, self.LOW):
            raise ValueError(f"Invalid GPIO value: {value}")
        self._pins[pin]['val'] = value
        logging.debug(f"[DRY-RUN] Pin {pin} -> {value}")

    def PWM(self, pin: int, freq: int) -> PWMInterface:
        if freq <= 0:
            raise ValueError(f"Invalid PWM frequency: {freq}")
        logging.info(f"[DRY-RUN] PWM on pin {pin} at {freq} Hz")
        return DummyPWM()

    def cleanup(self) -> None:
        logging.info("[DRY-RUN] GPIO cleanup completed")
        self._pins.clear()


def load_gpio(force_dry_run: bool = False) -> Tuple[Union[GPIOBase, object], bool]:
    """Load GPIO backend, preferring RPi.GPIO if available.
    
    Args:
        force_dry_run: Force use of dry-run mode even if RPi.GPIO is available
        
    Returns:
        Tuple of (gpio_instance, is_dry_run)
    """
    if force_dry_run:
        return DryRunGPIO(), True
        
    try:
        import RPi.GPIO as RPiGPIO
        logging.info("Using RPi.GPIO backend")
        return RPiGPIO, False
    except (ImportError, ModuleNotFoundError) as e:
        logging.warning(f"RPi.GPIO not available ({e}). Using DRY-RUN mode.")
        return DryRunGPIO(), True


# -----------------------------
# Motor Driver (L298N)
# -----------------------------

class MotorDriver:
    """L298N motor driver controller for differential drive robots.
    
    Controls two DC motors using H-bridge configuration:
    - Motor A: ENA (PWM), IN1, IN2 (direction)
    - Motor B: ENB (PWM), IN3, IN4 (direction)
    
    Features:
    - Speed control via PWM (0-100% duty cycle)
    - Direction control via digital pins
    - Safety limits and input validation
    - Proper resource cleanup
    """
    
    def __init__(self, gpio: Union[GPIOBase, object], pin_mode: str,
                 ena: int, in1: int, in2: int,
                 enb: int, in3: int, in4: int,
                 pwm_freq: int = DEFAULT_PWM_FREQ) -> None:
        """Initialize motor driver.
        
        Args:
            gpio: GPIO backend instance
            pin_mode: 'BCM' or 'BOARD' pin numbering
            ena: Enable pin for motor A (PWM)
            in1, in2: Direction pins for motor A
            enb: Enable pin for motor B (PWM)
            in3, in4: Direction pins for motor B
            pwm_freq: PWM frequency in Hz
            
        Raises:
            ValueError: Invalid pin configuration or PWM frequency
        """
        # Validate inputs
        if pin_mode not in ('BCM', 'BOARD'):
            raise ValueError(f"Invalid pin mode: {pin_mode}")
        if pwm_freq <= 0:
            raise ValueError(f"Invalid PWM frequency: {pwm_freq}")
            
        pins = [ena, in1, in2, enb, in3, in4]
        if len(set(pins)) != len(pins):
            raise ValueError("Duplicate pin assignments detected")
            
        self.gpio = gpio
        self.pin_mode = pin_mode
        self.ena, self.in1, self.in2 = ena, in1, in2
        self.enb, self.in3, self.in4 = enb, in3, in4
        self.pwm_freq = pwm_freq
        
        # Initialize GPIO
        try:
            if hasattr(self.gpio, 'setmode'):
                mode = self.gpio.BCM if pin_mode == 'BCM' else self.gpio.BOARD
                self.gpio.setmode(mode)
                self.gpio.setwarnings(False)

            # Setup all pins as outputs
            for pin in pins:
                self.gpio.setup(pin, self.gpio.OUT)

            # Initialize PWM channels
            self.pwm_a = self.gpio.PWM(self.ena, self.pwm_freq)
            self.pwm_b = self.gpio.PWM(self.enb, self.pwm_freq)
            self.pwm_a.start(0)
            self.pwm_b.start(0)
            
            # Track motor states
            self._left_dir = 0
            self._right_dir = 0
            
            logging.info(f"Motor driver initialized: pins={pins}, freq={pwm_freq}Hz")
            
        except Exception as e:
            logging.error(f"Motor driver initialization failed: {e}")
            raise

    @staticmethod
    def _clip_speed(speed: float) -> float:
        """Clip speed to valid range."""
        return max(SPEED_MIN, min(SPEED_MAX, speed))
    
    @staticmethod
    def _clip_duty(duty: float) -> float:
        """Clip duty cycle to valid range."""
        return max(DUTY_MIN, min(DUTY_MAX, duty))

    def _set_motor(self, side: str, speed: float) -> None:
        """Set individual motor speed and direction.
        
        Args:
            side: 'left' or 'right'
            speed: Speed from -100 to 100 (negative = reverse)
            
        Raises:
            ValueError: Invalid side parameter
        """
        if side not in ('left', 'right'):
            raise ValueError(f"Invalid motor side: {side}")
            
        speed = self._clip_speed(speed)
        duty = self._clip_duty(abs(speed))
        
        try:
            if side == 'left':
                # Left motor: IN1/IN2, PWM_A
                if speed >= 0:
                    self.gpio.output(self.in1, self.gpio.HIGH)
                    self.gpio.output(self.in2, self.gpio.LOW)
                    self._left_dir = 1
                else:
                    self.gpio.output(self.in1, self.gpio.LOW)
                    self.gpio.output(self.in2, self.gpio.HIGH)
                    self._left_dir = -1
                self.pwm_a.ChangeDutyCycle(duty)
                
            else:  # right
                # Right motor: IN3/IN4, PWM_B
                if speed >= 0:
                    self.gpio.output(self.in3, self.gpio.HIGH)
                    self.gpio.output(self.in4, self.gpio.LOW)
                    self._right_dir = 1
                else:
                    self.gpio.output(self.in3, self.gpio.LOW)
                    self.gpio.output(self.in4, self.gpio.HIGH)
                    self._right_dir = -1
                self.pwm_b.ChangeDutyCycle(duty)
                
        except Exception as e:
            logging.error(f"Motor control error ({side}): {e}")
            self.emergency_stop()
            raise

    def set_speeds(self, left_speed: float, right_speed: float) -> None:
        """Set differential motor speeds.
        
        Args:
            left_speed: Left motor speed (-100 to 100)
            right_speed: Right motor speed (-100 to 100)
        """
        try:
            self._set_motor('left', left_speed)
            self._set_motor('right', right_speed)
            logging.debug(f"Motor speeds set: L={left_speed:.1f}, R={right_speed:.1f}")
        except Exception as e:
            logging.error(f"Failed to set motor speeds: {e}")
            raise

    def stop(self) -> None:
        """Coast stop - set PWM to 0 but allow free rotation."""
        try:
            self.pwm_a.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)
            # Set all direction pins low for coast mode
            for pin in (self.in1, self.in2, self.in3, self.in4):
                self.gpio.output(pin, self.gpio.LOW)
            logging.info("Motors stopped (coast mode)")
        except Exception as e:
            logging.error(f"Motor stop failed: {e}")
            
    def emergency_stop(self) -> None:
        """Emergency stop - immediately halt all motor activity."""
        try:
            self.stop()
            logging.warning("Emergency stop activated")
        except Exception as e:
            logging.critical(f"Emergency stop failed: {e}")

    def cleanup(self) -> None:
        """Clean up resources and stop motors safely."""
        try:
            self.stop()
            if hasattr(self.pwm_a, 'stop'):
                self.pwm_a.stop()
            if hasattr(self.pwm_b, 'stop'):
                self.pwm_b.stop()
        except Exception as e:
            logging.error(f"Motor cleanup error: {e}")
        finally:
            try:
                if hasattr(self.gpio, 'cleanup'):
                    self.gpio.cleanup()
                logging.info("Motor driver cleanup completed")
            except Exception as e:
                logging.error(f"GPIO cleanup error: {e}")


# -----------------------------
# Face Detector (MediaPipe)
# -----------------------------

class FaceDetector:
    """MediaPipe-based face detection for robot tracking.
    
    Provides real-time face detection with confidence filtering
    and bounding box extraction for robot control applications.
    
    Features:
    - Configurable detection confidence threshold
    - Multiple model selection options
    - Robust error handling
    - Efficient BGR to RGB conversion
    """
    
    def __init__(self, frame_w: int = DEFAULT_CAMERA_WIDTH, 
                 frame_h: int = DEFAULT_CAMERA_HEIGHT,
                 min_conf: float = 0.6, model_selection: int = 0) -> None:
        """Initialize face detector.
        
        Args:
            frame_w: Frame width in pixels
            frame_h: Frame height in pixels  
            min_conf: Minimum detection confidence (0.0-1.0)
            model_selection: MediaPipe model (0=short range, 1=full range)
            
        Raises:
            ValueError: Invalid parameters
        """
        # Validate inputs
        if frame_w <= 0 or frame_h <= 0:
            raise ValueError(f"Invalid frame dimensions: {frame_w}x{frame_h}")
        if not 0.0 <= min_conf <= 1.0:
            raise ValueError(f"Invalid confidence threshold: {min_conf}")
        if model_selection not in (0, 1):
            raise ValueError(f"Invalid model selection: {model_selection}")
            
        self.w = frame_w
        self.h = frame_h
        self.min_conf = min_conf
        self.model_selection = model_selection
        
        try:
            self._mp_face = mp.solutions.face_detection
            self._detector = self._mp_face.FaceDetection(
                model_selection=self.model_selection,
                min_detection_confidence=self.min_conf
            )
            logging.info(f"Face detector initialized: {frame_w}x{frame_h}, conf={min_conf}")
        except Exception as e:
            logging.error(f"Face detector initialization failed: {e}")
            raise

    def process(self, frame_bgr) -> Optional[Tuple[int, int, Tuple[int, int, int, int]]]:
        """Detect faces in BGR frame.
        
        Args:
            frame_bgr: OpenCV BGR image
            
        Returns:
            Tuple of (center_x, center_y, (x, y, width, height)) for first face,
            or None if no face detected
            
        Raises:
            ValueError: Invalid frame format
        """
        if frame_bgr is None:
            raise ValueError("Frame is None")
            
        try:
            # Convert BGR to RGB for MediaPipe
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            
            # Run detection
            results = self._detector.process(frame_rgb)
            
            if not results.detections:
                return None

            # Get first (highest confidence) detection
            detection = results.detections[0]
            bbox = detection.location_data.relative_bounding_box
            
            # Convert normalized coordinates to pixels
            x = max(0, int(bbox.xmin * self.w))
            y = max(0, int(bbox.ymin * self.h))
            w = min(self.w - x, int(bbox.width * self.w))
            h = min(self.h - y, int(bbox.height * self.h))
            
            # Calculate center point
            cx = x + w // 2
            cy = y + h // 2
            
            return (cx, cy, (x, y, w, h))
            
        except cv2.error as e:
            logging.error(f"OpenCV error in face detection: {e}")
            return None
        except Exception as e:
            logging.error(f"Face detection error: {e}")
            return None

    def close(self) -> None:
        """Clean up detector resources."""
        try:
            # MediaPipe handles cleanup automatically
            logging.info("Face detector closed")
        except Exception as e:
            logging.error(f"Face detector cleanup error: {e}")


# -----------------------------
# FollowBot Controller
# -----------------------------

class FollowBot:
    """Main robot controller for face-following behavior.
    
    Integrates face detection with motor control to create a robot
    that follows detected faces using differential steering.
    
    Features:
    - Proportional steering based on face position
    - Safety timeout when no face is detected
    - Configurable deadband to reduce jitter
    - Visual feedback overlay
    - Comprehensive error handling
    """
    
    def __init__(self, motor: MotorDriver, face: FaceDetector,
                 deadband: int = 10, base_speed: int = 55, 
                 turn_gain: float = 0.8, no_face_stop_ms: int = 600) -> None:
        """Initialize FollowBot controller.
        
        Args:
            motor: Motor driver instance
            face: Face detector instance
            deadband: Ignore face positions within this many pixels of center
            base_speed: Forward speed when face is centered (0-100)
            turn_gain: Steering sensitivity multiplier (0.0-2.0)
            no_face_stop_ms: Stop motors after this many ms without face
            
        Raises:
            ValueError: Invalid parameter values
        """
        # Validate inputs
        if not isinstance(motor, MotorDriver):
            raise ValueError("Invalid motor driver instance")
        if not isinstance(face, FaceDetector):
            raise ValueError("Invalid face detector instance")
        if deadband < 0:
            raise ValueError(f"Invalid deadband: {deadband}")
        if not 0 <= base_speed <= 100:
            raise ValueError(f"Invalid base speed: {base_speed}")
        if turn_gain < 0:
            raise ValueError(f"Invalid turn gain: {turn_gain}")
        if no_face_stop_ms < 0:
            raise ValueError(f"Invalid timeout: {no_face_stop_ms}")
            
        self.motor = motor
        self.face = face
        self.deadband = abs(deadband)
        self.base_speed = max(0, min(100, base_speed))
        self.turn_gain = float(turn_gain)
        self.no_face_stop_ms = max(0, no_face_stop_ms)
        self._last_face_time = 0.0
        
        logging.info(f"FollowBot initialized: deadband={deadband}, "
                    f"speed={base_speed}, gain={turn_gain}, timeout={no_face_stop_ms}ms")

    def _map_offset_to_norm(self, cx: int) -> int:
        """Convert face center X to normalized steering value.
        
        Args:
            cx: Face center X coordinate in pixels
            
        Returns:
            Normalized steering value (-100 to 100)
        """
        if self.face.w == 0:
            logging.warning("Face detector width is zero")
            return 0
            
        center_x = self.face.w // 2
        x_offset = cx - center_x
        
        # Avoid division by zero
        half_width = self.face.w // 2
        if half_width == 0:
            return 0
            
        x_norm = int((x_offset * 100) / half_width)
        return max(-100, min(100, x_norm))

    def _compute_speeds(self, x_norm: int) -> Tuple[float, float]:
        """Calculate differential motor speeds from steering input.
        
        Args:
            x_norm: Normalized steering value (-100 to 100)
            
        Returns:
            Tuple of (left_speed, right_speed)
        """
        # Apply deadband
        if abs(x_norm) <= self.deadband:
            return self.base_speed, self.base_speed
            
        # Differential steering calculation
        delta = self.turn_gain * (x_norm / 100.0) * self.base_speed
        left = self.base_speed - delta
        right = self.base_speed + delta
        
        # Clip to valid range
        left = max(SPEED_MIN, min(SPEED_MAX, left))
        right = max(SPEED_MIN, min(SPEED_MAX, right))
        
        return left, right

    def _draw_overlay(self, frame_bgr, cx: int, cy: int, bbox: Tuple[int, int, int, int], 
                     x_norm: int) -> None:
        """Draw visual feedback on frame.
        
        Args:
            frame_bgr: OpenCV BGR frame
            cx, cy: Face center coordinates
            bbox: Face bounding box (x, y, w, h)
            x_norm: Normalized steering value
        """
        try:
            x, y, w, h = bbox
            
            # Face bounding box (green)
            cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Face center point (red)
            cv2.circle(frame_bgr, (cx, cy), 6, (0, 0, 255), -1)
            
            # Center lines
            frame_center_x = self.face.w // 2
            cv2.line(frame_bgr, (0, cy), (self.face.w, cy), (0, 255, 0), 1)
            cv2.line(frame_bgr, (frame_center_x, 0), (frame_center_x, self.face.h), (0, 255, 0), 1)
            
            # Steering indicator
            color = (0, 255, 255) if abs(x_norm) <= self.deadband else (0, 165, 255)
            cv2.putText(frame_bgr, f"Steer: {x_norm:+d}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                       
        except Exception as e:
            logging.error(f"Overlay drawing error: {e}")

    def update(self, frame_bgr) -> Tuple[float, float, str]:
        """Update robot control based on current frame.
        
        Args:
            frame_bgr: OpenCV BGR frame from camera
            
        Returns:
            Tuple of (left_speed, right_speed, status_message)
        """
        current_time = time.time()
        
        try:
            # Detect face
            detection = self.face.process(frame_bgr)
            
            if detection:
                cx, cy, bbox = detection
                self._last_face_time = current_time
                
                # Calculate steering
                x_norm = self._map_offset_to_norm(cx)
                left_speed, right_speed = self._compute_speeds(x_norm)
                
                # Set motor speeds
                self.motor.set_speeds(left_speed, right_speed)
                
                # Draw overlay
                self._draw_overlay(frame_bgr, cx, cy, bbox, x_norm)
                
                status = f"Following: L={left_speed:.0f} R={right_speed:.0f}"
                logging.debug(f"Face at ({cx}, {cy}), steering={x_norm}, speeds=({left_speed:.1f}, {right_speed:.1f})")
                
                return left_speed, right_speed, status
                
            else:
                # No face detected - check timeout
                time_since_face = (current_time - self._last_face_time) * 1000
                
                if time_since_face > self.no_face_stop_ms:
                    self.motor.stop()
                    status = f"No face - stopped ({time_since_face:.0f}ms)"
                    return 0.0, 0.0, status
                else:
                    # Continue previous motion briefly
                    status = f"No face - searching ({time_since_face:.0f}ms)"
                    return 0.0, 0.0, status
                    
        except Exception as e:
            logging.error(f"FollowBot update error: {e}")
            self.motor.emergency_stop()
            return 0.0, 0.0, f"Error: {e}"

    def cleanup(self) -> None:
        """Clean up resources."""
        try:
            self.motor.cleanup()
            self.face.close()
            logging.info("FollowBot cleanup completed")
        except Exception as e:
            logging.error(f"FollowBot cleanup error: {e}")


# -----------------------------
# Main Application
# -----------------------------

def setup_logging(level: str = 'INFO') -> None:
    """Configure logging system."""
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        format='%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )


def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='FollowBot Pi: Face-following robot',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 followbot_pi.py                    # Run normally
  python3 followbot_pi.py --dry-run          # Test mode
  python3 followbot_pi.py --camera 1         # Use camera 1
  python3 followbot_pi.py --speed 40         # Slower speed
        """
    )
    
    parser.add_argument('--dry-run', action='store_true',
                       help='Force dry-run mode (no GPIO)')
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera index (default: 0)')
    parser.add_argument('--width', type=int, default=DEFAULT_CAMERA_WIDTH,
                       help=f'Camera width (default: {DEFAULT_CAMERA_WIDTH})')
    parser.add_argument('--height', type=int, default=DEFAULT_CAMERA_HEIGHT,
                       help=f'Camera height (default: {DEFAULT_CAMERA_HEIGHT})')
    parser.add_argument('--speed', type=int, default=55,
                       help='Base speed 0-100 (default: 55)')
    parser.add_argument('--confidence', type=float, default=0.6,
                       help='Face detection confidence 0.0-1.0 (default: 0.6)')
    parser.add_argument('--log-level', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                       default='INFO', help='Logging level (default: INFO)')
    parser.add_argument('--no-display', action='store_true',
                       help='Run headless (no video display)')
    
    return parser.parse_args()


def initialize_camera(camera_idx: int, width: int, height: int) -> cv2.VideoCapture:
    """Initialize camera with error handling."""
    try:
        cap = cv2.VideoCapture(camera_idx)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera {camera_idx}")
            
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Test frame capture
        ret, frame = cap.read()
        if not ret or frame is None:
            raise RuntimeError("Cannot read from camera")
            
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        logging.info(f"Camera initialized: {actual_w}x{actual_h}")
        
        return cap
        
    except Exception as e:
        logging.error(f"Camera initialization failed: {e}")
        raise


def main() -> None:
    """Main application entry point."""
    args = parse_arguments()
    setup_logging(args.log_level)
    
    logging.info("Starting FollowBot Pi v2.0")
    logging.info(f"Mode: {'DRY-RUN' if args.dry_run else 'HARDWARE'}")
    
    # Initialize components
    gpio = None
    motor = None
    face = None
    bot = None
    cap = None
    
    try:
        # GPIO and motor setup
        gpio, is_dry_run = load_gpio(args.dry_run)
        motor = MotorDriver(
            gpio=gpio,
            pin_mode='BCM',
            ena=18, in1=24, in2=23,  # Motor A
            enb=19, in3=21, in4=20,  # Motor B
            pwm_freq=DEFAULT_PWM_FREQ
        )
        
        # Face detection setup
        face = FaceDetector(
            frame_w=args.width,
            frame_h=args.height,
            min_conf=args.confidence
        )
        
        # Robot controller setup
        bot = FollowBot(
            motor=motor,
            face=face,
            base_speed=args.speed
        )
        
        # Camera setup
        cap = initialize_camera(args.camera, args.width, args.height)
        
        logging.info("All systems initialized - starting main loop")
        
        # Main control loop
        frame_count = 0
        start_time = time.time()
        
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                logging.warning("Failed to read camera frame")
                continue
                
            frame_count += 1
            
            # Update robot control
            left_speed, right_speed, status = bot.update(frame)
            
            # Add status overlay
            cv2.putText(frame, status, (10, frame.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Display frame (if not headless)
            if not args.no_display:
                try:
                    cv2.imshow('FollowBot Pi', frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:  # 'q' or ESC
                        logging.info("User requested quit")
                        break
                except cv2.error:
                    logging.warning("Display not available - running headless")
                    args.no_display = True
            
            # Performance logging
            if frame_count % 100 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                logging.info(f"Processed {frame_count} frames, {fps:.1f} FPS")
                
    except KeyboardInterrupt:
        logging.info("Interrupted by user")
    except Exception as e:
        logging.error(f"Application error: {e}")
        raise
    finally:
        # Cleanup all resources
        logging.info("Shutting down...")
        
        if bot:
            bot.cleanup()
        if motor:
            motor.cleanup()
        if face:
            face.close()
        if cap:
            cap.release()
            
        cv2.destroyAllWindows()
        logging.info("Shutdown complete")


if __name__ == '__main__':
    main()