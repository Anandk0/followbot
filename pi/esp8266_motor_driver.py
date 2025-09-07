# esp8266_motor_driver.py - ESP8266 Motor Controller Interface
import serial
import json
import time
import threading
from queue import Queue, Empty

class ESP8266MotorDriver:
    def __init__(self, port='/dev/ttyUSB0', baud=115200, timeout=1.0):
        self.port = port
        self.baud = baud
        self.ser = None
        self.connected = False
        self.telemetry = {}
        self.rx_queue = Queue()
        self.rx_thread = None
        self.running = False
        
        self.connect()
        
    def connect(self):
        """Establish serial connection to ESP8266"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
            time.sleep(2)  # Wait for ESP8266 to boot
            
            # Send ping to verify connection
            self.send_command({"cmd": "ping"})
            time.sleep(0.5)
            
            # Start receiver thread
            self.running = True
            self.rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
            self.rx_thread.start()
            
            self.connected = True
            print(f"[ESP8266] Connected to {self.port}")
            
        except Exception as e:
            print(f"[ESP8266] Failed to connect: {e}")
            self.connected = False
            
    def _rx_worker(self):
        """Background thread to receive data from ESP8266"""
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        try:
                            data = json.loads(line)
                            self.rx_queue.put(data)
                            
                            # Update telemetry if received
                            if "telemetry" in data:
                                self.telemetry.update(data["telemetry"])
                                
                        except json.JSONDecodeError:
                            print(f"[ESP8266] Invalid JSON: {line}")
                else:
                    time.sleep(0.01)  # Prevent busy-waiting
                            
            except Exception as e:
                print(f"[ESP8266] RX error: {e}")
                time.sleep(0.1)
                
    def send_command(self, cmd_dict):
        """Send JSON command to ESP8266"""
        if not self.connected or not self.ser:
            return False
            
        try:
            cmd_json = json.dumps(cmd_dict) + '\n'
            self.ser.write(cmd_json.encode('utf-8'))
            return True
        except Exception as e:
            print(f"[ESP8266] Send error: {e}")
            return False
            
    def send(self, cmd: dict):
        """Process movement commands (compatible with motor_driver.py interface)"""
        if not isinstance(cmd, dict):
            print(f"[ESP8266] Invalid command type: {type(cmd)}")
            return
        
        action = cmd.get('action')
        speed = max(-100, min(100, cmd.get('speed', 50)))  # Clamp speed
        
        if action == 'forward':
            self.set_motor_speeds(speed, speed)
        elif action == 'backward':
            self.set_motor_speeds(-speed, -speed)
        elif action == 'left':
            self.set_motor_speeds(-speed, speed)
        elif action == 'right':
            self.set_motor_speeds(speed, -speed)
        elif action == 'stop':
            self.stop()
        else:
            print(f"[ESP8266] Unknown action: {action}")
            
        print(f"[ESP8266] Command: {cmd}")
        
    def set_motor_speeds(self, left, right):
        """Set individual motor speeds (-100 to +100)"""
        cmd = {
            "cmd": "set_speed",
            "left": int(left),
            "right": int(right)
        }
        return self.send_command(cmd)
        
    def stop(self):
        """Stop all motors"""
        cmd = {"cmd": "stop"}
        return self.send_command(cmd)
        
    def get_telemetry(self):
        """Get latest telemetry data"""
        # Process pending messages (max 10 to prevent blocking)
        try:
            for _ in range(10):
                data = self.rx_queue.get_nowait()
                if "telemetry" in data:
                    self.telemetry.update(data["telemetry"])
        except Empty:
            pass
            
        return self.telemetry.copy()
        
    def get_distance(self):
        """Get distance sensor reading"""
        telem = self.get_telemetry()
        return telem.get("distance_cm", 999.0)
        
    def is_connected(self):
        """Check if ESP8266 is connected"""
        return self.connected and self.ser and self.ser.is_open
        
    def close(self):
        """Close connection and cleanup"""
        self.running = False
        
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
            
        if self.ser and self.ser.is_open:
            self.stop()  # Stop motors before closing
            time.sleep(0.1)
            self.ser.close()
            
        self.connected = False
        print("[ESP8266] Connection closed")

# Test function
if __name__ == "__main__":
    import time
    
    # Test the ESP8266 motor driver
    motor = ESP8266MotorDriver('/dev/ttyUSB0')  # Adjust port as needed
    
    if motor.is_connected():
        print("Testing motor movements...")
        
        # Test forward
        print("Forward")
        motor.send({'action': 'forward', 'speed': 30})
        time.sleep(2)
        
        # Test backward
        print("Backward")
        motor.send({'action': 'backward', 'speed': 30})
        time.sleep(2)
        
        # Test left turn
        print("Left")
        motor.send({'action': 'left', 'speed': 40})
        time.sleep(1)
        
        # Test right turn
        print("Right")
        motor.send({'action': 'right', 'speed': 40})
        time.sleep(1)
        
        # Stop
        print("Stop")
        motor.send({'action': 'stop'})
        
        # Show telemetry
        telem = motor.get_telemetry()
        print(f"Telemetry: {telem}")
        
    motor.close()