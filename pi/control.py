import math, time

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

class Controller:
    def __init__(self, cfg, pid_yaw, bno, link):
        self.cfg = cfg
        self.pid = pid_yaw
        self.bno = bno
        self.link = link
        self.last_obstacle_cm = None
        self.last_rx_ts = 0

    def handle_rx(self, msg):
        # expected: {"telemetry":{"distance_cm":xx.x,"bat_v":x.x}}
        tele = msg.get("telemetry")
        if tele and "distance_cm" in tele:
            self.last_obstacle_cm = float(tele["distance_cm"])
            self.last_rx_ts = time.time()

    def obstacle_blocked(self):
        if self.last_obstacle_cm is None: return False
        return self.last_obstacle_cm < self.cfg["safety"]["stop_distance_cm"]

    def drive_raw(self, left, right):
        cmd = {"cmd":"set_speed","left": int(clamp(left,-100,100)),
               "right": int(clamp(right,-100,100))}
        print(f"Sending motor command: {cmd}")
        self.link.send(cmd)

    def stop(self):
        print("Sending STOP command")
        self.link.send({"cmd":"stop"})

    def step(self, bbox, img_w, yaw_deg, dt):
        """
        bbox: (x,y,w,h) or None
        returns (left,right,status_str)
        """
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
                s = base + min(30, abs(u))
                left = clamp(-u, -maxs, maxs)
                right = clamp(u, -maxs, maxs)
                status = f"ROTATE yaw_err={err:.1f}"

        self.drive_raw(left, right)
        return (left, right, status)

