# ESP8266 Motor Controller Setup

## Hardware Connections

### ESP8266 NodeMCU Pins:
- **D1 (GPIO5)** → L298N IN1 (Left Motor Direction 1)
- **D2 (GPIO4)** → L298N IN2 (Left Motor Direction 2)  
- **D3 (GPIO0)** → L298N IN3 (Right Motor Direction 1)
- **D4 (GPIO2)** → L298N IN4 (Right Motor Direction 2)
- **D5 (GPIO14)** → L298N ENA (Left Motor PWM)
- **D6 (GPIO12)** → L298N ENB (Right Motor PWM)
- **D7 (GPIO13)** → HC-SR04 Trig Pin
- **D8 (GPIO15)** → HC-SR04 Echo Pin
- **GND** → Common Ground
- **VIN** → 5V Power Supply

### L298N Motor Driver:
- **OUT1, OUT2** → Left Motor
- **OUT3, OUT4** → Right Motor
- **12V** → Motor Power Supply
- **5V** → ESP8266 VIN (if using onboard regulator)

## Software Setup

1. **Upload ESP8266 Code:**
   ```bash
   # Install ArduinoJson library in Arduino IDE
   # Open esp8266_code.ino
   # Select NodeMCU 1.0 board
   # Upload to ESP8266
   ```

2. **Install Pi Dependencies:**
   ```bash
   cd pi/
   pip install -r requirements.txt
   ```

3. **Configure Serial Port:**
   Edit `pi/config.yaml`:
   ```yaml
   esp8266:
     port: "/dev/ttyUSB0"  # Linux/Pi
     # port: "COM3"        # Windows
     baud: 115200
   ```

4. **Test Connection:**
   ```bash
   cd pi/
   python test_esp8266_simple.py
   ```

5. **Run FollowBot:**
   ```bash
   cd pi/
   python main.py
   ```

## Troubleshooting

### Connection Issues
- **No serial connection:** Check USB cable, try different COM ports
- **Permission denied:** `sudo chmod 666 /dev/ttyUSB0` on Linux
- **I/O Error [Errno 5]:** Serial port corruption, try:
  ```bash
  # Disconnect ESP8266, then:
  sudo rmmod ftdi_sio
  sudo modprobe ftdi_sio
  # Or reboot system
  ```

### Hardware Issues
- **Motors not moving:** Check L298N power supply and connections
- **Distance sensor not working:** Verify HC-SR04 wiring and power
- **ESP8266 not responding:** Check power supply (3.3V/5V), try different USB cable

### Software Issues
- **Command format errors:** ESP8266 expects:
  - `{"cmd": "set_speed", "left": 50, "right": 50}` for movement
  - `{"cmd": "stop"}` to stop motors
  - `{"cmd": "ping"}` to test connection

### Testing
```bash
# Test ESP8266 communication
cd pi/
python test_esp8266_simple.py

# Test with specific port
python test_esp8266_simple.py /dev/ttyUSB0
```