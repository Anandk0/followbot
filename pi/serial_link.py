# serial_link.py  -- robust SerialLink for Pi -> ESP (JSONL)
import serial, json, threading, queue, time
from typing import Optional

class SerialLink:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.05):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.rx_q = queue.Queue()
        self._stop = False
        self.ser: Optional[serial.Serial] = None
        self._open_serial()
        self._t = threading.Thread(target=self._rx_loop, daemon=True)
        self._t.start()

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            # small delay to allow device to reset
            time.sleep(0.1)
        except (serial.SerialException, OSError) as e:
            self.ser = None
            print(f"[SerialLink] open error: {e}")

    def _rx_loop(self):
        buf = b""
        while not self._stop:
            if self.ser is None:
                self._open_serial()
                time.sleep(0.5)
                continue
            try:
                data = self.ser.read(256)
                if not data:
                    continue
                buf += data
                # Prevent buffer overflow
                if len(buf) > 4096:
                    buf = buf[-2048:]  # Keep last 2KB
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        obj = json.loads(line.decode("utf-8", errors="ignore"))
                        self.rx_q.put(obj)
                        print(f"[SerialLink] RECV: {obj}")
                    except (json.JSONDecodeError, UnicodeDecodeError) as e:
                        # keep debug info but don't crash
                        print(f"[SerialLink] rx json parse err: {e} raw={line!r}")
            except (serial.SerialException, OSError) as e:
                print(f"[SerialLink] read error: {e}")
                try:
                    self.ser.close()
                except (serial.SerialException, OSError):
                    pass
                self.ser = None
                time.sleep(0.5)

    def send(self, obj: dict):
        """Send JSON object as single-line JSONL. Retries on transient errors."""
        line = (json.dumps(obj) + "\n").encode("utf-8")
        if self.ser is None:
            self._open_serial()
            if self.ser is None:
                print("[SerialLink] WARNING: serial not open, dropping:", obj)
                return
        try:
            self.ser.write(line)
            self.ser.flush()
            print(f"[SerialLink] SENT: {obj}")
        except (serial.SerialException, OSError) as e:
            print(f"[SerialLink] write error: {e} -- attempting reconnect")
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None

    def recv_nowait(self):
        try:
            return self.rx_q.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self._stop = True
        try:
            if self.ser:
                self.ser.close()
        except (serial.SerialException, OSError):
            pass
