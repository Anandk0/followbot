#!/usr/bin/env python3
"""
Test script for ESP8266 Motor Controller
Run this to verify ESP8266 communication and motor control
"""

import time
import sys
from esp8266_motor_driver import ESP8266MotorDriver

def test_esp8266():
    print("ESP8266 Motor Controller Test")
    print("=" * 40)
    
    # Try to connect
    ports = ['/dev/ttyUSB0', '/dev/ttyACM0', 'COM3', 'COM4', 'COM5']
    motor = None
    
    for port in ports:
        try:
            print(f"Trying port {port}...")
            motor = ESP8266MotorDriver(port)
            if motor.is_connected():
                print(f"✓ Connected to ESP8266 on {port}")
                break
            else:
                motor.close()
        except Exception as e:
            print(f"✗ Failed on {port}: {e}")
            continue
    
    if not motor or not motor.is_connected():
        print("❌ Could not connect to ESP8266")
        print("\nTroubleshooting:")
        print("1. Check ESP8266 is powered and programmed")
        print("2. Verify USB cable connection")
        print("3. Check serial port permissions")
        print("4. Try different COM port numbers")
        return False
    
    print("\n🔧 Testing motor movements...")
    
    try:
        # Test sequence
        movements = [
            ('stop', 0, 1),
            ('forward', 30, 2),
            ('stop', 0, 1),
            ('backward', 30, 2),
            ('stop', 0, 1),
            ('left', 40, 1.5),
            ('stop', 0, 1),
            ('right', 40, 1.5),
            ('stop', 0, 1)
        ]
        
        for action, speed, duration in movements:
            print(f"  {action.upper()} (speed: {speed}) for {duration}s")
            motor.send({'action': action, 'speed': speed})
            
            # Show telemetry during movement
            start_time = time.time()
            while time.time() - start_time < duration:
                telem = motor.get_telemetry()
                if telem:
                    dist = telem.get('distance_cm', 'N/A')
                    active = telem.get('motors_active', False)
                    print(f"    Distance: {dist}cm, Motors: {'ON' if active else 'OFF'}")
                time.sleep(0.5)
        
        print("\n📊 Final telemetry:")
        telem = motor.get_telemetry()
        for key, value in telem.items():
            print(f"  {key}: {value}")
            
        print("\n✅ Test completed successfully!")
        return True
        
    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted by user")
        return False
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        return False
    finally:
        if motor:
            motor.close()

if __name__ == "__main__":
    success = test_esp8266()
    sys.exit(0 if success else 1)