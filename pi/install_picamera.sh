#!/bin/bash
# Install Pi Camera libraries

echo "Installing Pi Camera support..."

# Update system
sudo apt update

# Install picamera2 (modern)
sudo apt install -y python3-picamera2

# Install legacy picamera (fallback)
pip install picamera

# Enable camera interface
sudo raspi-config nonint do_camera 0

echo "Pi Camera libraries installed!"
echo "Reboot required: sudo reboot"