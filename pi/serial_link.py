import json, serial, threading, queue, time

class SerialLink:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.rx_q = queue.Queue()
        self._stop = False
        self._t = threading.Thread(target=self._rx_loop, daemon=True)
        self._t.start()

    def _rx_loop(self):
        buf = b""
        while not self._stop:
            try:
                data = self.ser.read(256)
                if not data: continue
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if line.strip():
                        try:
                            self.rx_q.put(json.loads(line.decode("utf-8")))
                        except Exception:
                            pass
            except Exception:
                time.sleep(0.05)

    def send(self, obj):
        line = (json.dumps(obj) + "\n").encode("utf-8")
        self.ser.write(line)

    def recv_nowait(self):
        try:
            return self.rx_q.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self._stop = True
        try: self.ser.close()
        except: pass

