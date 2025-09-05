import cv2, time, yaml, os, subprocess, threading, signal
import numpy as np

# Handle camera process cleanup
def signal_handler(signum, frame):
    print("\nShutting down gracefully...")
    if 'camera_proc' in globals():
        camera_proc.terminate()
    exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
from detector_movenet import MoveNetDetector
from tracker import SmoothTracker
from bno055_reader import BNO055Reader
from pid import PID
from serial_link import SerialLink
from control import Controller
from utils import draw_vis

def load_cfg(path):
    with open(path,'r') as f: return yaml.safe_load(f)

def main():
    cfg = load_cfg("config.yaml")

    # camera - use GStreamer with libcamera
    print("Starting camera with GStreamer...")
    w, h = cfg["camera"]["width"], cfg["camera"]["height"]
    
    # GStreamer pipeline for libcamera
    gst_pipeline = (
        f'libcamerasrc ! '
        f'video/x-raw,width={w},height={h},framerate=30/1 ! '
        f'videoconvert ! appsink'
    )
    
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print("Failed to open GStreamer pipeline, trying simple approach...")
        # Fallback to rpicam-vid with proper format
        cmd = [
            'rpicam-vid', '--inline', '--nopreview',
            f'--width={w}', f'--height={h}',
            '--framerate=30', '--timeout=0',
            '--codec=mjpeg', '--output=-'
        ]
        try:
            camera_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            print(f"Using rpicam-vid: {w}x{h}")
            use_subprocess = True
        except Exception as e:
            print(f"Camera failed: {e}")
            return
    else:
        print(f"GStreamer camera started: {w}x{h}")
        use_subprocess = False

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
    
    # Try serial connection with fallbacks
    serial_ports = [cfg["serial"]["port"], "/dev/ttyUSB0", "/dev/ttyACM0"]
    link = None
    for port in serial_ports:
        try:
            link = SerialLink(port, cfg["serial"]["baud"])
            print(f"Serial connected on {port}")
            break
        except Exception as e:
            print(f"Failed to connect to {port}: {e}")
    
    if not link:
        print("No serial connection - running in camera-only mode")
        class MockSerial:
            def send(self, data): pass
            def recv_nowait(self): return None
            def close(self): pass
        link = MockSerial()
    
    ctl = Controller(cfg, pid, bno, link)

    last_t = time.time()
    try:
        frame_count = 0
        
        while True:
            if use_subprocess:
                # Read MJPEG frame from subprocess
                # Look for JPEG start marker (0xFF 0xD8)
                buffer = b''
                while True:
                    chunk = camera_proc.stdout.read(1024)
                    if not chunk:
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
                # Use GStreamer capture
                ret, frame = cap.read()
                if not ret or frame is None:
                    print("Failed to read frame from GStreamer")
                    continue
                    
            frame_count += 1
            if frame_count % 30 == 1:
                print(f"Processing frame {frame_count}, shape: {frame.shape}")
            h, w = frame.shape[:2]

            # read serial telemetry
            rx = ctl.link.recv_nowait()
            if rx: ctl.handle_rx(rx)

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
        link.close()
        if 'cap' in locals():
            cap.release()
        if 'camera_proc' in locals():
            camera_proc.terminate()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

