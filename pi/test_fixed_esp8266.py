#!/usr/bin/env python3
"""
Test the fixed ESP8266 connection with robust error handling
"""
import time
import sys
import os
sys.path.append(os.path.dirname(__file__))

from esp8266_motor_driver import ESP8266MotorDriver

def test_robust_esp8266():
    """Test ESP8266 with the new robust connection"""
    
    # Try to find ESP8266 automatically
    try:
        from find_esp8266_port import find_esp8266_port
        port = find_esp8266_port()
        if not port:
            print("No ESP8266 found. Using default COM3...")
            port = "COM3"
    except:
        port = "COM3"  # Default fallback
    
    print(f"Testing ESP8266 on {port}...")
    
    # Create motor driver with robust connection
    motor = ESP8266MotorDriver(port, 115200)
    
    if not motor.is_connected():
        print("Failed to connect to ESP8266!")
        return False
    
    print("✓ ESP8266 connected successfully!")
    
    try:
        # Test the commands that were failing before
        test_sequence = [
            {'action': 'forward', 'speed': 30},
            {'action': 'left', 'speed': 25},
            {'action': 'right', 'speed': 25},
            {'action': 'backward', 'speed': 20},
            {'action': 'stop'}
        ]
        
        print("\nTesting motor commands...")
        for i, cmd in enumerate(test_sequence):
            print(f"Test {i+1}: {cmd}")
            
            # This should not fail with I/O errors now
            motor.send(cmd)
            time.sleep(1)
            
            # Check telemetry
            telemetry = motor.get_telemetry()
            if telemetry:
                print(f"  Telemetry: {telemetry}")
        
        print("\n✓ All commands sent successfully!")
        print("✓ No I/O errors encountered!")
        
        # Test rapid commands (this used to cause issues)
        print("\nTesting rapid command sequence...")
        for i in range(10):
            motor.send({'action': 'left', 'speed': 5})
            time.sleep(0.1)  # Very short delay
        
        motor.send({'action': 'stop'})
        print("✓ Rapid commands completed!")
        
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        return False
        
    finally:
        motor.close()
        print("Connection closed cleanly")

if __name__ == "__main__":
    print("ESP8266 Robust Connection Test")
    print("=" * 40)
    
    success = test_robust_esp8266()
    
    if success:
        print("\n🎉 ESP8266 connection is now working robustly!")
        print("You can now run your main followbot code without I/O errors.")
    else:
        print("\n❌ Test failed. Check your ESP8266 connection and try:")
        print("1. Run: python find_esp8266_port.py")
        print("2. Check ESP8266 is powered and code is uploaded")
        print("3. Try different USB cable/port")