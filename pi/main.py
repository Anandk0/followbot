import cv2, time, yaml, os
import numpy as np
from detector_movenet import MoveNetDetector
from tracker import SmoothTracker
from bno055_reader import BNO055Reader
from pid import PID
from esp8266_motor_driver import ESP8266MotorDriver
from control import Controller
from utils import draw_vis

def load_cfg(path):
    import os
    # Prevent path traversal attacks
    if not os.path.basename(path) == path or '..' in path:
        raise ValueError("Invalid config file path")
    with open(path,'r') as f: return yaml.safe_load(f)

def setup_camera(cfg):
    """Setup Pi Camera with proper support"""
    w, h = cfg["camera"]["width"], cfg["camera"]["height"]
    
    # Try picamera2 first (modern Pi Camera)
    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (w, h)})
        picam2.configure(config)
        picam2.start()
        print(f"Pi Camera (picamera2) started: {w}x{h}")
        return picam2, None, "picamera2"
    except ImportError:
        print("picamera2 not available")
    except Exception as e:
        print(f"picamera2 failed: {e}")
    
    # Try legacy picamera
    try:
        import picamera
        import picamera.array
        camera = picamera.PiCamera()
        camera.resolution = (w, h)
        camera.framerate = 30
        print(f"Pi Camera (legacy) started: {w}x{h}")
        return camera, None, "picamera"
    except ImportError:
        print("picamera not available")
    except Exception as e:
        print(f"picamera failed: {e}")
    
    # Fallback to OpenCV
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        print(f"Using OpenCV camera: {w}x{h}")
        return cap, None, "opencv"
    
    raise RuntimeError("No camera available")

def main():
    cfg = load_cfg("config.yaml")
    
    try:
        camera, camera_proc, camera_type = setup_camera(cfg)
    except RuntimeError as e:
        print(e)
        return

    det = MoveNetDetector(cfg["model"]["path"], cfg["model"]["input_size"])
    trk = SmoothTracker(alpha=0.35, timeout_ms=cfg["safety"]["no_person_timeout_ms"])
    
    # Try BNO055, fall back to mock if not connected
    try:
        bno = BNO055Reader()
        print("BNO055 sensor connected")
    except (ValueError, OSError, RuntimeError) as e:
        print(f"BNO055 not found: {e}")
        print("Using mock IMU (yaw=0)")
        class MockBNO:
            def yaw_deg(self): return 0.0
        bno = MockBNO()
    pid = PID(cfg["pid"]["kp"], cfg["pid"]["ki"], cfg["pid"]["kd"], cfg["pid"]["out_limit"])
    
    # Initialize motor driver with fallback
    esp_cfg = cfg.get("esp8266", {})
    use_mock = esp_cfg.get("use_mock", False)
    use_wifi = esp_cfg.get("use_wifi", False)
    
    if use_mock:
        print("Using mock motor driver (configured)")
        class MockMotorDriver:
            def send(self, cmd): print(f"MOCK MOTOR: {cmd}")
            def close(self): pass
        motors = MockMotorDriver()
    elif use_wifi:
        # Use WiFi connection
        esp_ip = esp_cfg.get("ip", "192.168.1.100")
        motors = WiFiMotorDriver(esp_ip)
        print(f"WiFi motor driver initialized for {esp_ip}")
    else:
        # Use serial connection
        port = esp_cfg.get("port", "/dev/ttyUSB0")
        baud = esp_cfg.get("baud", 115200)
        motors = ESP8266MotorDriver(port, baud)
        print(f"Serial motor driver initialized on {port}")
        if not motors.is_connected():
            print("WARNING: ESP8266 not responding, but continuing...")
    
    ctl = Controller(cfg, pid, bno, motors)

    last_t = time.time()
    try:
        frame_count = 0
        frame = None
        camera_error_count = 0
        
        while True:
            # Capture frame based on camera type
            if camera_type == "picamera2":
                frame = camera.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif camera_type == "picamera":
                import picamera.array
                with picamera.array.PiRGBArray(camera) as output:
                    camera.capture(output, 'bgr')
                    frame = output.array
            elif camera_type == "opencv":
                ret, frame = camera.read()
                if not ret:
                    frame = None
            else:
                frame = None
            
            if frame is None:
                camera_error_count += 1
                if camera_error_count > 5:
                    print(f"Camera error, restarting {camera_type}...")
                    if camera_type in ["picamera2", "picamera"]:
                        camera.close() if hasattr(camera, 'close') else None
                    elif camera_type == "opencv":
                        camera.release()
                    camera, camera_proc, camera_type = setup_camera(cfg)
                    camera_error_count = 0
                continue
            else:
                camera_error_count = 0
                    
            frame_count += 1
            if frame_count % 60 == 1:  # Reduce debug output
                print(f"Processing frame {frame_count}, shape: {frame.shape}")
            h, w = frame.shape[:2]

            # Skip if no frame
            if frame is None:
                continue

            # detection
            d = det.infer(frame)
            bbox = trk.update(d)

            # imu
            yaw = bno.yaw_deg()

            # control step
            now = time.time()
            dt = max(1e-3, now - last_t); last_t = now
            left, right, status = ctl.step(bbox, w, yaw, dt)

            draw_vis(frame, bbox, status)
            try:
                cv2.imshow("FollowBot", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except cv2.error:
                # Running headless - continue without display
                pass

    finally:
        motors.close()
        if 'camera' in locals() and camera:
            if camera_type == "picamera2":
                camera.stop()
                camera.close()
            elif camera_type == "picamera":
                camera.close()
            elif camera_type == "opencv":
                camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

