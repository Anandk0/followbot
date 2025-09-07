import cv2, time, yaml, os, subprocess, threading
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
    """Setup camera with fallback options"""
    w, h = cfg["camera"]["width"], cfg["camera"]["height"]
    
    # Try GStreamer first
    gst_pipeline = f'libcamerasrc ! video/x-raw,width={w},height={h},framerate=30/1 ! videoconvert ! appsink'
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    
    if cap.isOpened():
        print(f"GStreamer camera started: {w}x{h}")
        return cap, None, False
    
    # Try rpicam-vid
    cmd = ['rpicam-vid', '--inline', '--nopreview', f'--width={w}', f'--height={h}', '--framerate=30', '--timeout=0', '--codec=mjpeg', '--output=-']
    try:
        camera_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        print(f"Using rpicam-vid: {w}x{h}")
        return None, camera_proc, True
    except Exception as e:
        print(f"rpicam-vid failed: {e}")
    
    # Try default camera
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        print(f"Using default camera: {w}x{h}")
        return cap, None, False
    
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
    
    # Initialize ESP8266 motor driver
    esp_cfg = cfg.get("esp8266", {})
    port = esp_cfg.get("port", "/dev/ttyUSB0")
    baud = esp_cfg.get("baud", 115200)
    
    try:
        motors = ESP8266MotorDriver(port, baud)
        if motors.is_connected():
            print(f"ESP8266 motor driver connected on {port}")
        else:
            print(f"Failed to connect to ESP8266 on {port}")
            return
    except Exception as e:
        print(f"ESP8266 connection failed: {e}")
        return
    
    ctl = Controller(cfg, pid, bno, motors)

    last_t = time.time()
    try:
        frame_count = 0
        frame = None
        
        while True:
            if use_subprocess:
                # Read MJPEG frame from rpicam-vid
                buffer = b''
                frame = None
                while True:
                    try:
                        chunk = camera_proc.stdout.read(1024)
                        if not chunk:
                            break
                    except Exception:
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
                    print("Failed to decode MJPEG frame")
                    continue
            else:
                # Use camera capture
                ret, frame = cap.read()
                if not ret or frame is None:
                    print("Failed to read frame from camera")
                    continue
                    
            frame_count += 1
            if frame_count % 30 == 1:
                print(f"Processing frame {frame_count}, shape: {frame.shape}")
            h, w = frame.shape[:2]

            # no telemetry needed for direct motor control

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
            except cv2.error as e:
                print(f"OpenCV display error: {e}")
                print("Running headless - no display available")
                break
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        motors.close()
        if 'cap' in locals() and cap.isOpened():
            cap.release()
        if 'camera_proc' in locals():
            try:
                camera_proc.terminate()
                try:
                    camera_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    camera_proc.kill()
                    camera_proc.wait()
            except (subprocess.TimeoutExpired, ProcessLookupError) as e:
                print(f"Camera cleanup error: {e}")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

