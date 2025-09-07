#!/usr/bin/env python3
"""
Test script for ESP8266 with fixed IP address
"""
import requests
import time
import json

ESP_IP = "192.168.1.143"  # Fixed IP address
BASE_URL = f"http://{ESP_IP}"

def test_connection():
    """Test basic connection to ESP8266"""
    try:
        response = requests.get(f"{BASE_URL}/", timeout=3)
        if response.status_code == 200:
            print(f"✓ ESP8266 connected at {ESP_IP}")
            print(f"  Response: {response.text}")
            return True
    except Exception as e:
        print(f"✗ Connection failed: {e}")
    return False

def test_motor_commands():
    """Test motor control commands"""
    commands = [
        {"cmd": "set_speed", "left": 30, "right": 30},   # Forward
        {"cmd": "set_speed", "left": -30, "right": -30}, # Backward  
        {"cmd": "set_speed", "left": -30, "right": 30},  # Left turn
        {"cmd": "set_speed", "left": 30, "right": -30},  # Right turn
        {"cmd": "stop"}                                   # Stop
    ]
    
    for i, cmd in enumerate(commands):
        try:
            response = requests.post(f"{BASE_URL}/motor", json=cmd, timeout=2)
            if response.status_code == 200:
                print(f"✓ Command {i+1}: {cmd} -> {response.json()}")
            else:
                print(f"✗ Command {i+1} failed: {response.status_code}")
        except Exception as e:
            print(f"✗ Command {i+1} error: {e}")
        
        time.sleep(1)

def get_status():
    """Get ESP8266 status and telemetry"""
    try:
        response = requests.get(f"{BASE_URL}/status", timeout=2)
        if response.status_code == 200:
            status = response.json()
            print(f"✓ Status received:")
            print(f"  Distance: {status.get('distance_cm', 'N/A')} cm")
            print(f"  Uptime: {status.get('uptime_ms', 0)/1000:.1f} seconds")
            print(f"  Free RAM: {status.get('free_heap', 'N/A')} bytes")
            print(f"  IP Address: {status.get('ip', 'N/A')}")
            return status
    except Exception as e:
        print(f"✗ Status error: {e}")
    return None

if __name__ == "__main__":
    print(f"Testing ESP8266 at fixed IP: {ESP_IP}")
    print("=" * 50)
    
    # Test connection
    if test_connection():
        print()
        
        # Get status
        get_status()
        print()
        
        # Test motor commands
        print("Testing motor commands...")
        test_motor_commands()
        
    else:
        print("\nTroubleshooting:")
        print("1. Make sure ESP8266 is powered on")
        print("2. Check WiFi credentials in esp8266_wifi_code.ino")
        print("3. Verify your network range (change 192.168.1.x if needed)")
        print("4. Upload the updated code to ESP8266")