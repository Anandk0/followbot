#!/usr/bin/env python3
import serial
import json
import time
import sys
import serial.tools.list_ports

def find_serial_ports():
    """Find available COM ports on Windows"""
    ports = serial.tools.list_ports.comports()
    available_ports = []
    for port in ports:
        available_ports.append(port.device)
        print(f"Found port: {port.device} - {port.description}")
    return available_ports

def test_serial_port():
    # First, find available ports
    print("Scanning for available serial ports...")
    ports = find_serial_ports()
    
    if not ports:
        print("No serial ports found. Make sure your device is connected.")
        return False
    
    # Use the first available port (or specify manually)
    port = ports[0]  # Change this to specific port if needed
    
    try:
        # Try to open with exclusive access
        ser = serial.Serial(port, 115200, timeout=1, exclusive=True)
        print(f"Serial port {port} opened successfully")
        
        # Wait for ESP8266 to boot
        time.sleep(2)
        
        # Send test command
        test_cmd = {"cmd": "stop"}
        json_str = json.dumps(test_cmd) + '\n'
        print(f"Sending: {json_str.strip()}")
        ser.write(json_str.encode())
        
        # Read response
        time.sleep(0.5)
        while ser.in_waiting:
            response = ser.readline().decode().strip()
            if response:
                print(f"Received: {response}")
        
        ser.close()
        print("Test completed successfully")
        return True
        
    except serial.SerialException as e:
        if "Device or resource busy" in str(e):
            print(f"Port {port} is busy. Another process is using it.")
            print("\nTo fix this:")
            print("1. Close any Arduino IDE or serial monitors")
            print("2. Kill any running Python scripts with Ctrl+C")
            print("3. Unplug and replug the USB cable")
            print("4. Check Task Manager for python.exe processes")
            return False
        else:
            print(f"Serial error: {e}")
            return False
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    test_serial_port()