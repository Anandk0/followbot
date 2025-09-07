#!/usr/bin/env python3
"""
Simple ESP8266 test script to verify communication
"""
import serial
import json
import time
import sys

def test_esp8266(port='/dev/ttyUSB0', baud=115200):
    """Test ESP8266 communication with proper command format"""
    
    try:
        print(f"Connecting to ESP8266 on {port}...")
        ser = serial.Serial(port, baud, timeout=2.0)
        time.sleep(2)  # Wait for ESP8266 to boot
        
        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        print("ESP8266 connected! Testing commands...")
        
        # Test 1: Ping
        print("\n1. Testing ping...")
        cmd = {"cmd": "ping"}
        ser.write((json.dumps(cmd) + '\n').encode())
        time.sleep(0.5)
        
        # Read response
        while ser.in_waiting:
            response = ser.readline().decode().strip()
            if response:
                print(f"   Response: {response}")
        
        # Test 2: Motor commands
        test_commands = [
            {"cmd": "set_speed", "left": 30, "right": 30},   # Forward
            {"cmd": "set_speed", "left": -30, "right": -30}, # Backward  
            {"cmd": "set_speed", "left": -30, "right": 30},  # Left turn
            {"cmd": "set_speed", "left": 30, "right": -30},  # Right turn
            {"cmd": "stop"}                                   # Stop
        ]
        
        for i, cmd in enumerate(test_commands):
            print(f"\n{i+2}. Testing: {cmd}")
            ser.write((json.dumps(cmd) + '\n').encode())
            time.sleep(1)
            
            # Read any responses
            while ser.in_waiting:
                response = ser.readline().decode().strip()
                if response:
                    print(f"   Response: {response}")
        
        print("\n6. Monitoring telemetry for 5 seconds...")
        start_time = time.time()
        while time.time() - start_time < 5:
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                if response:
                    try:
                        data = json.loads(response)
                        if "telemetry" in data:
                            telem = data["telemetry"]
                            print(f"   Distance: {telem.get('distance_cm', 'N/A')} cm, "
                                  f"Motors: {telem.get('motors_active', 'N/A')}, "
                                  f"Uptime: {telem.get('uptime_ms', 'N/A')} ms")
                    except json.JSONDecodeError:
                        print(f"   Raw: {response}")
            time.sleep(0.1)
        
        ser.close()
        print("\nTest completed successfully!")
        return True
        
    except Exception as e:
        print(f"Test failed: {e}")
        return False

if __name__ == "__main__":
    # Try common ports
    ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', 'COM3', 'COM4', 'COM5']
    
    if len(sys.argv) > 1:
        ports = [sys.argv[1]]
    
    for port in ports:
        try:
            if test_esp8266(port):
                break
        except (FileNotFoundError, serial.SerialException):
            print(f"Port {port} not available")
            continue
    else:
        print("No ESP8266 found on any port!")