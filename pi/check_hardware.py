#!/usr/bin/env python3
"""Hardware connection checker for FollowBot"""

import os
import glob
import subprocess

def check_serial_ports():
    """Check available serial ports"""
    print("=== Serial Ports ===")
    ports = glob.glob('/dev/tty*')
    usb_ports = [p for p in ports if 'USB' in p or 'ACM' in p]
    
    if usb_ports:
        print(f"Found USB serial ports: {usb_ports}")
        return usb_ports[0]  # Return first available
    else:
        print("No USB serial ports found")
        print("Available ports:", ports[-10:])  # Show last 10
        return None

def check_i2c():
    """Check I2C devices"""
    print("\n=== I2C Devices ===")
    try:
        result = subprocess.run(['i2cdetect', '-y', '1'], 
                              capture_output=True, text=True)
        print("I2C bus 1:")
        print(result.stdout)
        
        if '28' in result.stdout:
            print("✓ BNO055 IMU found at 0x28")
        else:
            print("✗ BNO055 IMU not found")
            
    except FileNotFoundError:
        print("i2cdetect not available. Install: sudo apt install i2c-tools")

def check_camera():
    """Check camera availability"""
    print("\n=== Camera ===")
    try:
        result = subprocess.run(['libcamera-hello', '--list-cameras'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("✓ libcamera working")
            print(result.stdout)
        else:
            print("✗ libcamera issues")
    except (FileNotFoundError, subprocess.TimeoutExpired):
        print("libcamera-hello not available")

def main():
    print("FollowBot Hardware Check")
    print("=" * 30)
    
    # Check serial
    port = check_serial_ports()
    if port:
        print(f"\nUpdate config.yaml with: port: \"{port}\"")
    
    # Check I2C
    check_i2c()
    
    # Check camera
    check_camera()
    
    print("\n=== Permissions ===")
    print("If serial fails, run: sudo usermod -a -G dialout $USER")
    print("Then logout/login or reboot")

if __name__ == "__main__":
    main()