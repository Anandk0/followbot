import time, math

class PID:
    def __init__(self, kp, ki, kd, out_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.ilim = out_limit
        self.olim = out_limit
        self.i = 0.0
        self.prev = None

    def reset(self):
        self.i, self.prev = 0.0, None

    def step(self, err, dt):
        p = self.kp * err
        self.i += self.ki * err * dt
        if self.ilim is not None:
            self.i = max(-self.ilim, min(self.ilim, self.i))
        d = 0.0
        if self.prev is not None:
            d = self.kd * (err - self.prev) / max(1e-3, dt)
        self.prev = err
        out = p + self.i + d
        if self.olim is not None:
            out = max(-self.olim, min(self.olim, out))
        return out

