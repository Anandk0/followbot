import cv2, time, yaml, os, subprocess, threading
import numpy as np
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

    # camera - use libcamera directly
    print("Starting libcamera stream...")
    w, h = cfg["camera"]["width"], cfg["camera"]["height"]
    
    # Start libcamera process
    cmd = [
        'libcamera-vid', '--inline', '--nopreview', 
        f'--width={w}', f'--height={h}',
        '--framerate=30', '--timeout=0', '--output=-'
    ]
    
    try:
        camera_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(f"Camera started: {w}x{h}")
    except Exception as e:
        print(f"Failed to start libcamera: {e}")
        return

    det = MoveNetDetector(cfg["model"]["path"], cfg["model"]["input_size"])
    trk = SmoothTracker(alpha=0.35, timeout_ms=cfg["safety"]["no_person_timeout_ms"])
    bno = BNO055Reader()
    pid = PID(cfg["pid"]["kp"], cfg["pid"]["ki"], cfg["pid"]["kd"], cfg["pid"]["out_limit"])
    link = SerialLink(cfg["serial"]["port"], cfg["serial"]["baud"])
    ctl = Controller(cfg, pid, bno, link)

    last_t = time.time()
    try:
        frame_count = 0
        frame_size = w * h * 3  # RGB bytes per frame
        
        while True:
            # Read raw frame data from libcamera
            raw_data = camera_proc.stdout.read(frame_size)
            if len(raw_data) != frame_size:
                print("Incomplete frame data")
                continue
                
            # Convert to numpy array and reshape
            frame = np.frombuffer(raw_data, dtype=np.uint8).reshape((h, w, 3))
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                
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
        if 'camera_proc' in locals():
            camera_proc.terminate()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

