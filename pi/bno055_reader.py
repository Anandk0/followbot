import time
try:
    import board, busio
    import adafruit_bno055
    HAS_BNO055 = True
except ImportError:
    HAS_BNO055 = False

class BNO055Reader:
    def __init__(self, bus=None):
        if not HAS_BNO055:
            raise RuntimeError("BNO055 libraries not available on this platform")
        i2c = bus or busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self._last_ok = time.time()

    @staticmethod
    def _wrap180(deg):
        if deg is None: return None
        # BNO yaw is 0..360; convert to -180..+180
        if deg > 180: deg -= 360
        return deg

    def yaw_deg(self):
        ypr = self.sensor.euler
        if ypr is None: return None
        yaw = ypr[0]
        return self._wrap180(yaw)

