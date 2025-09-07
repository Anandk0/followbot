import requests
import json
import time

class WiFiMotorDriver:
    def __init__(self, esp_ip="192.168.1.143", timeout=2.0):
        self.esp_ip = esp_ip
        self.timeout = timeout
        self.base_url = f"http://{esp_ip}"
        self.connected = False
        self.last_status = {}
        
        # Test connection
        self.connect()
    
    def connect(self):
        """Test connection to ESP8266"""
        try:
            response = requests.get(f"{self.base_url}/", timeout=self.timeout)
            if response.status_code == 200:
                self.connected = True
                print(f"[WiFi] Connected to ESP8266 at {self.esp_ip}")
                return True
        except Exception as e:
            print(f"[WiFi] Connection failed: {e}")
        
        self.connected = False
        return False
    
    def send_command(self, cmd_dict):
        """Send JSON command via HTTP POST"""
        if not self.connected:
            if not self.connect():
                return False
        
        try:
            response = requests.post(
                f"{self.base_url}/motor",
                json=cmd_dict,
                timeout=self.timeout
            )
            return response.status_code == 200
        except Exception as e:
            print(f"[WiFi] Send error: {e}")
            self.connected = False
            return False
    
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
    
    def send(self, cmd: dict):
        """Process movement commands (compatible with motor_driver.py interface)"""
        action = cmd.get('action')
        speed = max(-100, min(100, cmd.get('speed', 50)))
        
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
        
        print(f"[WiFi] Command: {cmd}")
    
    def get_telemetry(self):
        """Get status from ESP8266"""
        try:
            response = requests.get(f"{self.base_url}/status", timeout=self.timeout)
            if response.status_code == 200:
                self.last_status = response.json()
                return self.last_status
        except Exception as e:
            print(f"[WiFi] Telemetry error: {e}")
        
        return self.last_status
    
    def get_distance(self):
        """Get distance sensor reading"""
        telem = self.get_telemetry()
        return telem.get("distance_cm", 999.0)
    
    def is_connected(self):
        """Check if ESP8266 is connected"""
        return self.connected
    
    def close(self):
        """Cleanup"""
        if self.connected:
            self.stop()
        print("[WiFi] Connection closed")

# Test function
if __name__ == "__main__":
    # Test the WiFi motor driver
    motor = WiFiMotorDriver("192.168.1.143")  # Fixed IP address
    
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