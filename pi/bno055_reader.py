import time
import board, busio
import adafruit_bno055

class BNO055Reader:
    def __init__(self, bus=None, mock=False):
        self.mock = mock
        if mock:
            print("BNO055Reader: Running in mock mode")
            self.sensor = None
        else:
            try:
                i2c = bus or busio.I2C(board.SCL, board.SDA)
                self.sensor = adafruit_bno055.BNO055_I2C(i2c)
            except (ValueError, OSError) as e:
                print(f"BNO055Reader: Sensor not found, switching to mock mode. Error: {e}")
                self.mock = True
                self.sensor = None
        self._last_ok = time.time()

    @staticmethod
    def _wrap180(deg):
        if deg is None: return None
        # BNO yaw is 0..360; convert to -180..+180
        if deg > 180: deg -= 360
        return deg

    def yaw_deg(self):
        if self.mock:
            return 0.0  # Return fixed yaw for testing
        ypr = self.sensor.euler
        if ypr is None: return None
        yaw = ypr[0]
        return self._wrap180(yaw)

