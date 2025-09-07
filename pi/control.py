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
        
        # State machine for stop-assess-center-move
        self.state = "ASSESS"  # ASSESS, CENTERING, MOVING
        self.last_bbox_center = None
        self.centering_start_time = 0
        self.assess_start_time = 0
        self.stable_frames = 0
        self.target_stable = False

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
        
        # Send individual motor speeds directly to ESP8266
        try:
            self.motors.set_motor_speeds(left, right)
            return
        except AttributeError:
            pass
        
        # Fallback for mock driver
            # Fallback for mock driver
            if left == 0 and right == 0:
                self.motors.send({'action': 'stop'})
            elif left > 0 and right > 0:
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
        State machine: ASSESS -> CENTERING -> MOVING
        bbox: (x,y,w,h) or None
        returns (left,right,status_str)
        """
        # Update telemetry from ESP8266
        self.update_telemetry()
        
        base = self.cfg["control"]["base_speed"]
        maxs = self.cfg["control"]["max_speed"]
        fov = self.cfg["camera"]["fov_deg"]
        dead = self.cfg["control"]["align_deadband_deg"]
        current_time = time.time()

        # Safety first
        if self.obstacle_blocked():
            self.stop()
            self.state = "ASSESS"
            return (0,0,f"STOP obstacle {self.last_obstacle_cm:.1f}cm")

        if bbox is None or yaw_deg is None:
            self.stop()
            self.state = "ASSESS"
            return (0,0,"NO TARGET or NO IMU")

        x,y,w,h = bbox
        cx = x + w/2.0
        ex = (cx - (img_w/2)) / (img_w/2)      # -1..+1
        desired_yaw = yaw_deg + ex * (fov/2.0) * self.cfg["control"]["angle_gain"]
        err = (desired_yaw - yaw_deg)
        # wrap error to [-180,180]
        if err > 180: err -= 360
        if err < -180: err += 360

        # Check if target moved significantly
        if self.last_bbox_center is not None:
            center_diff = abs(cx - self.last_bbox_center)
            if center_diff > 50:  # Target moved significantly
                self.state = "ASSESS"
                self.stable_frames = 0
        
        self.last_bbox_center = cx
        
        # Distance check
        tgt_h = self.cfg["control"]["follow_target_px"]
        if h >= tgt_h:  # Too close
            self.stop()
            return (0,0,f"TOO_CLOSE h={h}")
        
        # State machine logic
        if self.state == "ASSESS":
            # Stop and assess for 0.5 seconds
            if not hasattr(self, 'assess_start_time') or self.assess_start_time == 0:
                self.assess_start_time = current_time
            
            self.stop()
            
            if current_time - self.assess_start_time > 0.5:
                if abs(err) <= dead:
                    self.state = "MOVING"
                    status = "ASSESS->MOVING (already centered)"
                else:
                    self.state = "CENTERING"
                    self.centering_start_time = current_time
                    status = f"ASSESS->CENTERING err={err:.1f}"
                self.assess_start_time = 0
            else:
                status = f"ASSESSING err={err:.1f}"
            
            return (0, 0, status)
            
        elif self.state == "CENTERING":
            # Turn in place to center on target
            if abs(err) <= dead:
                # Successfully centered
                self.stable_frames += 1
                if self.stable_frames >= 3:  # Stay centered for 3 frames
                    self.state = "MOVING"
                    self.stable_frames = 0
                    return (0, 0, "CENTERED->MOVING")
            else:
                self.stable_frames = 0
            
            # Timeout check - don't center forever
            if current_time - self.centering_start_time > 3.0:
                self.state = "MOVING"
                return (0, 0, "CENTERING_TIMEOUT->MOVING")
            
            # Pure rotation to center
            turn_speed = clamp(abs(err) * 1.5, 25, 50)  # Proportional turning
            if err > 0:  # Turn right
                left, right = turn_speed, -turn_speed
            else:  # Turn left
                left, right = -turn_speed, turn_speed
            
            self.drive_raw(left, right)
            return (left, right, f"CENTERING err={err:.1f}")
            
        elif self.state == "MOVING":
            # Move forward while maintaining center
            if abs(err) > dead * 2:  # Lost alignment significantly
                self.state = "ASSESS"
                return (0, 0, "LOST_ALIGNMENT->ASSESS")
            
            # Move forward with gentle corrections
            u = self.pid.step(err, dt)
            left = clamp(base - u*0.2, 0, maxs)  # Gentle correction
            right = clamp(base + u*0.2, 0, maxs)
            
            self.drive_raw(left, right)
            return (left, right, f"MOVING err={err:.1f}")
        
        # Fallback
        self.stop()
        return (0, 0, "UNKNOWN_STATE")

