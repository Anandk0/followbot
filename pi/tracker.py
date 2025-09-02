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
        now = time.time()*1000
        if det is None:
            if now - self.last_ms > self.timeout_ms:
                self.state = None
            return self.state
        x,y,w,h = det['bbox']
        if self.state is None:
            self.state = (x,y,w,h)
        else:
            sx,sy,sw,sh = self.state
            self.state = (int(self.alpha*x + (1-self.alpha)*sx),
                          int(self.alpha*y + (1-self.alpha)*sy),
                          int(self.alpha*w + (1-self.alpha)*sw),
                          int(self.alpha*h + (1-self.alpha)*sh))
        self.last_ms = now
        return self.state

