import cv2, time, yaml, os, subprocess, threading, signal
import numpy as np
from detector_movenet import MoveNetDetector
from tracker import SmoothTracker
from bno055_reader import BNO055Reader
from pid import PID
from esp8266_motor_driver import ESP8266MotorDriver
from wifi_motor_driver import WiFiMotorDriver
from control import Controller
from utils import draw_vis

def load_cfg(path):
    import os
    # Prevent path traversal attacks
    if not os.path.basename(path) == path or '..' in path:
        raise ValueError("Invalid config file path")
    with open(path,'r') as f: return yaml.safe_load(f)

def setup_camera(cfg):
    """Setup camera for maximum speed"""
    # Force low resolution for speed
    w, h = 320, 240
    
    # Try OpenCV first (fastest)
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        print(f"Using OpenCV camera: {w}x{h}@30fps")
        return cap, None, False
    
    # Fast libcamera-vid setup
    cmd = ['libcamera-vid', '--inline', '--nopreview', f'--width={w}', f'--height={h}', 
           '--framerate=30', '--timeout=0', '--codec=mjpeg', '--output=-']
    try:
        camera_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        print(f"Using fast libcamera-vid: {w}x{h}@30fps")
        return None, camera_proc, True
    except Exception as e:
        print(f"libcamera-vid failed: {e}")
    
    raise RuntimeError("No camera available")

def main():
    cfg = load_cfg("config.yaml")
    
    try:
        cap, camera_proc, use_subprocess = setup_camera(cfg)
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
        last_restart = 0
        
        while True:
            if use_subprocess:
                # Read MJPEG frame from rpicam-vid
                buffer = b''
                frame = None
                while True:
                    try:
                        chunk = camera_proc.stdout.read(1024)
                        if not chunk or camera_proc.poll() is not None:
                            break
                    except (BrokenPipeError, OSError):
                        print("Camera process terminated")
                        break
                    buffer += chunk
                    
                    # Find JPEG start
                    start = buffer.find(b'\xff\xd8')
                    if start == -1:
                        continue
                        
                    # Find JPEG end
                    end = buffer.find(b'\xff\xd9', start + 2)
                    if end == -1:
                        continue
                        
                    # Extract JPEG frame
                    jpeg_data = buffer[start:end+2]
                    buffer = buffer[end+2:]
                    
                    # Decode JPEG
                    frame = cv2.imdecode(np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        break
                        
                if frame is None:
                    camera_error_count += 1
                    print(f"Failed to decode MJPEG frame (errors: {camera_error_count})")
                    if camera_proc.poll() is not None or camera_error_count > 10:
                        print("Camera process died or too many errors, attempting restart")
                        if time.time() - last_restart > 3:  # Restart more frequently
                            try:
                                camera_proc.terminate()
                                camera_proc.wait(timeout=1)
                            except:
                                try:
                                    camera_proc.kill()
                                except:
                                    pass
                            cap, camera_proc, use_subprocess = setup_camera(cfg)
                            camera_error_count = 0
                            last_restart = time.time()
                        continue
                    time.sleep(0.1)  # Brief pause on error
                    continue
                else:
                    camera_error_count = 0  # Reset error count on success
            else:
                # Use camera capture
                ret, frame = cap.read()
                if not ret or frame is None:
                    camera_error_count += 1
                    print(f"Failed to read frame from camera (errors: {camera_error_count})")
                    if camera_error_count > 3:
                        print("Too many camera errors, attempting restart")
                        if time.time() - last_restart > 3:
                            if cap:
                                cap.release()
                            cap, camera_proc, use_subprocess = setup_camera(cfg)
                            camera_error_count = 0
                            last_restart = time.time()
                    time.sleep(0.1)
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
        if 'cap' in locals() and cap.isOpened():
            cap.release()
        if 'camera_proc' in locals() and camera_proc:
            try:
                if camera_proc.poll() is None:  # Process still running
                    camera_proc.terminate()
                    try:
                        camera_proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        camera_proc.kill()
                        camera_proc.wait()
            except (subprocess.TimeoutExpired, ProcessLookupError) as e:
                print(f"Camera cleanup error: {e}")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

