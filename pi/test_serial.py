#!/usr/bin/env python3
import serial
import json
import time

# Test ESP8266 serial communication
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    print("Serial connection opened")
    
    # Wait for ESP8266 to boot
    time.sleep(2)
    
    # Send test commands
    commands = [
        {"cmd": "set_speed", "left": 50, "right": 50},
        {"cmd": "stop"},
        {"cmd": "set_speed", "left": -30, "right": 30},
        {"cmd": "stop"}
    ]
    
    for cmd in commands:
        json_str = json.dumps(cmd) + '\n'
        print(f"Sending: {json_str.strip()}")
        ser.write(json_str.encode())
        
        # Read any response
        time.sleep(0.5)
        while ser.in_waiting:
            response = ser.readline().decode().strip()
            if response:
                print(f"Received: {response}")
        
        time.sleep(1)
    
    ser.close()
    print("Test completed")
    
except Exception as e:
    print(f"Error: {e}")