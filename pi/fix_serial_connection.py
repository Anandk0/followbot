#!/usr/bin/env python3
"""
Serial Connection Fix for ESP8266 I/O Errors
Handles [Errno 5] Input/output error by implementing robust reconnection
"""
import serial
import json
import time
import threading
from queue import Queue, Empty

class RobustESP8266Connection:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.connected = False
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.last_successful_command = time.time()
        
    def connect(self):
        """Establish connection with error recovery"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                time.sleep(0.5)
            
            print(f"Connecting to ESP8266 on {self.port}...")
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=2.0,
                write_timeout=2.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            # Wait for ESP8266 to boot and clear buffers
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            # Test connection with ping
            test_cmd = {"cmd": "ping"}
            self.ser.write((json.dumps(test_cmd) + '\n').encode())
            time.sleep(0.5)
            
            # Try to read response
            response_received = False
            for _ in range(10):  # Try for 1 second
                if self.ser.in_waiting:
                    try:
                        response = self.ser.readline().decode().strip()
                        if response:
                            print(f"ESP8266 responded: {response}")
                            response_received = True
                            break
                    except:
                        pass
                time.sleep(0.1)
            
            self.connected = True
            self.reconnect_attempts = 0
            self.last_successful_command = time.time()
            print("ESP8266 connection established!")
            return True
            
        except Exception as e:
            print(f"Connection failed: {e}")
            self.connected = False
            return False
    
    def send_command(self, cmd_dict):
        """Send command with automatic reconnection on failure"""
        if not self.connected:
            if not self.connect():
                return False
        
        max_retries = 3
        for attempt in range(max_retries):
            try:
                cmd_json = json.dumps(cmd_dict) + '\n'
                self.ser.write(cmd_json.encode())
                self.ser.flush()  # Force write
                self.last_successful_command = time.time()
                return True
                
            except (serial.SerialException, OSError) as e:
                print(f"Send error (attempt {attempt+1}): {e}")
                
                if "Input/output error" in str(e) or "Errno 5" in str(e):
                    print("I/O error detected - attempting reconnection...")
                    self.connected = False
                    
                    if attempt < max_retries - 1:  # Don't reconnect on last attempt
                        time.sleep(0.5)
                        if self.connect():
                            continue
                
                if attempt == max_retries - 1:
                    print("Max retries reached - command failed")
                    return False
                    
                time.sleep(0.2)
        
        return False
    
    def read_responses(self, timeout=1.0):
        """Read available responses with timeout"""
        responses = []
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode().strip()
                    if line:
                        try:
                            data = json.loads(line)
                            responses.append(data)
                        except json.JSONDecodeError:
                            responses.append({"raw": line})
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"Read error: {e}")
                break
                
        return responses
    
    def close(self):
        """Close connection safely"""
        self.connected = False
        if self.ser and self.ser.is_open:
            try:
                # Send stop command before closing
                self.ser.write(b'{"cmd":"stop"}\n')
                time.sleep(0.1)
                self.ser.close()
            except:
                pass
        print("Connection closed")

def test_robust_connection():
    """Test the robust connection with motor commands"""
    # Try common Windows COM ports
    ports = ['COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8']
    
    esp = None
    for port in ports:
        try:
            esp = RobustESP8266Connection(port)
            if esp.connect():
                print(f"Successfully connected to {port}")
                break
        except:
            continue
    
    if not esp or not esp.connected:
        print("No ESP8266 found on any COM port!")
        return
    
    try:
        # Test commands that were failing
        test_commands = [
            {"cmd": "ping"},
            {"cmd": "set_speed", "left": 30, "right": 30},
            {"cmd": "set_speed", "left": -30, "right": 30},  # Left turn
            {"cmd": "set_speed", "left": 30, "right": -30},  # Right turn
            {"cmd": "stop"}
        ]
        
        for i, cmd in enumerate(test_commands):
            print(f"\nTest {i+1}: {cmd}")
            
            if esp.send_command(cmd):
                print("✓ Command sent successfully")
                
                # Read responses
                responses = esp.read_responses(0.5)
                for resp in responses:
                    print(f"  Response: {resp}")
            else:
                print("✗ Command failed")
            
            time.sleep(1)
        
        print("\nTest completed!")
        
    finally:
        esp.close()

if __name__ == "__main__":
    test_robust_connection()