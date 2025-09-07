import math, time

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

class Controller:
    def __init__(self, cfg, pid_yaw, bno, motors):
        self.cfg = cfg
        self.pid = pid_yaw
        self.bno = bno
        self.motors = motors
        self.last_obstacle_cm = None
        self.last_rx_ts = 0

    def _process_distance_telemetry(self, telem_data):
        """Process distance telemetry data"""
        if telem_data and "distance_cm" in telem_data:
            self.last_obstacle_cm = float(telem_data["distance_cm"])
            self.last_rx_ts = time.time()
            
    def handle_rx(self, msg):
        """Handle telemetry from ESP8266"""
        self._process_distance_telemetry(msg.get("telemetry"))
            
    def update_telemetry(self):
        """Update telemetry from ESP8266"""
        if hasattr(self.motors, 'get_telemetry'):
            self._process_distance_telemetry(self.motors.get_telemetry())

    def obstacle_blocked(self):
        if self.last_obstacle_cm is None: return False
        return self.last_obstacle_cm < self.cfg["safety"]["stop_distance_cm"]

    def drive_raw(self, left, right):
        left = int(clamp(left, -100, 100))
        right = int(clamp(right, -100, 100))
        print(f"Motor command: L={left}, R={right}")
        
        # Convert to motor commands with consistent logic
        if left > 0 and right > 0:
            self.motors.send({'action': 'forward', 'speed': max(abs(left), abs(right))})
        elif left < 0 and right < 0:
            self.motors.send({'action': 'backward', 'speed': max(abs(left), abs(right))})
        elif left > 0 and right < 0:
            self.motors.send({'action': 'right', 'speed': abs(left)})
        elif left < 0 and right > 0:
            self.motors.send({'action': 'left', 'speed': abs(right)})
        else:
            self.motors.send({'action': 'stop'})

    def stop(self):
        print("STOP command")
        self.motors.send({'action': 'stop'})

    def step(self, bbox, img_w, yaw_deg, dt):
        """
        bbox: (x,y,w,h) or None
        returns (left,right,status_str)
        """
        # Update telemetry from ESP8266
        self.update_telemetry()
        
        base = self.cfg["control"]["base_speed"]
        maxs = self.cfg["control"]["max_speed"]
        fov = self.cfg["camera"]["fov_deg"]
        dead = self.cfg["control"]["align_deadband_deg"]

        # safety first
        if self.obstacle_blocked():
            self.stop()
            return (0,0,f"STOP obstacle {self.last_obstacle_cm:.1f}cm")

        if bbox is None or yaw_deg is None:
            self.stop()
            return (0,0,"NO TARGET or NO IMU")

        x,y,w,h = bbox
        cx = x + w/2.0
        ex = (cx - (img_w/2)) / (img_w/2)      # -1..+1
        desired_yaw = yaw_deg + ex * (fov/2.0) * self.cfg["control"]["angle_gain"]
        err = (desired_yaw - yaw_deg)
        # wrap error to [-180,180]
        if err > 180: err -= 360
        if err < -180: err += 360

        u = self.pid.step(err, dt)             # -out..+out
        # angular command to differential speeds
        left = clamp(base - u, -maxs, maxs)
        right = clamp(base + u, -maxs, maxs)

        # forward gating by distance proxy (bbox height)
        tgt_h = self.cfg["control"]["follow_target_px"]
        if h >= tgt_h:         # too close
            left = -abs(left)*0.2
            right = -abs(right)*0.2
            status = f"BACKOFF yaw_err={err:.1f}"
        else:
            # allow forward only when roughly aligned
            if abs(err) <= dead:
                left = clamp(abs(left), 0, maxs)
                right = clamp(abs(right), 0, maxs)
                status = f"FORWARD yaw_ok"
            else:
                # rotate in place for strong yaw errors
                left = clamp(-u, -maxs, maxs)
                right = clamp(u, -maxs, maxs)
                status = f"ROTATE yaw_err={err:.1f}"

        self.drive_raw(left, right)
        return (left, right, status)

