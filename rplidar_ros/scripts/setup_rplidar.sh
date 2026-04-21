#!/bin/bash

echo "Setting up udev rule for RPLidar..."

# Define udev rules file content
UDEV_RULES="KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", SYMLINK+=\"rplidar\""

# Create the udev rules file
RULES_FILE="/etc/udev/rules.d/rplidar.rules"

echo "Creating udev rules file: $RULES_FILE"
echo $UDEV_RULES | sudo tee $RULES_FILE > /dev/null

# Restart udev service to apply changes
echo "Restarting udev service..."
sudo udevadm control --reload
sudo udevadm trigger

# Verify the rule
echo "Checking udev rules..."
udevadm info -q all -n /dev/ttyUSB0 | grep DEVLINKS

# Add current user to dialout group for serial port access
echo "Adding current user to 'dialout' group..."
sudo usermod -a -G dialout duzhong

echo "Setup complete. Please unplug and replug your device or reboot your system."

