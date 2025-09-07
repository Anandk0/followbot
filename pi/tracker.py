import time, math

class SmoothTracker:
    """
    Not full SORT. Keeps the most confident / largest bbox
    and exponentially smooths it to reduce jitter.
    """
    def __init__(self, alpha=0.3, timeout_ms=600):
        self.alpha = alpha
        self.timeout_ms = timeout_ms
        self.state = None
        self.last_ms = 0

    def update(self, det):
        if det is None:
            if self.state is not None:
                now = time.time()*1000
                if now - self.last_ms > self.timeout_ms:
                    self.state = None
            return self.state
        now = time.time()*1000
        x,y,w,h = det['bbox']
        if self.state is None:
            self.state = (x,y,w,h)
        else:
            sx,sy,sw,sh = self.state
            # Extract smoothing calculation for clarity
            new_x = int(self.alpha*x + (1-self.alpha)*sx)
            new_y = int(self.alpha*y + (1-self.alpha)*sy)
            new_w = int(self.alpha*w + (1-self.alpha)*sw)
            new_h = int(self.alpha*h + (1-self.alpha)*sh)
            self.state = (new_x, new_y, new_w, new_h)
        self.last_ms = now
        return self.state

