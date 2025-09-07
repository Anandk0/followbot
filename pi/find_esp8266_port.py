#!/usr/bin/env python3
"""
ESP8266 Port Detection Script
Automatically finds the correct COM port for ESP8266
"""
import serial
import serial.tools.list_ports
import json
import time

def find_esp8266_port():
    """Find ESP8266 on available COM ports"""
    print("Scanning for ESP8266...")
    
    # Get all available ports
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("No COM ports found!")
        return None
    
    print(f"Found {len(ports)} COM ports:")
    for port in ports:
        print(f"  {port.device}: {port.description}")
    
    # Test each port
    for port in ports:
        port_name = port.device
        print(f"\nTesting {port_name}...")
        
        try:
            # Try to connect
            ser = serial.Serial(
                port=port_name,
                baudrate=115200,
                timeout=2.0,
                write_timeout=2.0
            )
            
            time.sleep(2)  # Wait for ESP8266 boot
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            # Send ping command
            ping_cmd = {"cmd": "ping"}
            ser.write((json.dumps(ping_cmd) + '\n').encode())
            ser.flush()
            
            # Wait for response
            start_time = time.time()
            response_found = False
            
            while time.time() - start_time < 3.0:
                if ser.in_waiting:
                    try:
                        response = ser.readline().decode().strip()
                        if response:
                            print(f"  Response: {response}")
                            # Check if it looks like ESP8266 response
                            if any(keyword in response.lower() for keyword in ['pong', 'ok', 'ready', 'esp']):
                                response_found = True
                                break
                    except:
                        pass
                time.sleep(0.1)
            
            ser.close()
            
            if response_found:
                print(f"✓ ESP8266 found on {port_name}!")
                return port_name
            else:
                print(f"  No ESP8266 response on {port_name}")
                
        except Exception as e:
            print(f"  Error testing {port_name}: {e}")
            continue
    
    print("\nNo ESP8266 found on any port!")
    return None

def test_esp8266_communication(port):
    """Test basic ESP8266 communication"""
    print(f"\nTesting ESP8266 communication on {port}...")
    
    try:
        ser = serial.Serial(port, 115200, timeout=2.0)
        time.sleep(2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Test commands
        test_commands = [
            {"cmd": "ping"},
            {"cmd": "set_speed", "left": 0, "right": 0},
            {"cmd": "stop"}
        ]
        
        for cmd in test_commands:
            print(f"Sending: {cmd}")
            ser.write((json.dumps(cmd) + '\n').encode())
            ser.flush()
            time.sleep(0.5)
            
            # Read responses
            while ser.in_waiting:
                try:
                    response = ser.readline().decode().strip()
                    if response:
                        print(f"  Response: {response}")
                except:
                    pass
        
        ser.close()
        print("✓ Communication test completed!")
        return True
        
    except Exception as e:
        print(f"✗ Communication test failed: {e}")
        return False

if __name__ == "__main__":
    # Find ESP8266 port
    esp_port = find_esp8266_port()
    
    if esp_port:
        print(f"\n{'='*50}")
        print(f"ESP8266 FOUND ON: {esp_port}")
        print(f"{'='*50}")
        
        # Update config file
        try:
            import yaml
            with open('config.yaml', 'r') as f:
                config = yaml.safe_load(f)
            
            config['esp8266']['port'] = esp_port
            
            with open('config.yaml', 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            
            print(f"✓ Updated config.yaml with port: {esp_port}")
            
        except Exception as e:
            print(f"Could not update config.yaml: {e}")
            print(f"Please manually set esp8266.port to: {esp_port}")
        
        # Test communication
        test_esp8266_communication(esp_port)
        
    else:
        print("\nTroubleshooting tips:")
        print("1. Check ESP8266 is connected via USB")
        print("2. Install CH340/CP2102 drivers if needed")
        print("3. Try different USB cable")
        print("4. Check ESP8266 is powered and running")
        print("5. Verify ESP8266 code is uploaded correctly")