#!/bin/bash

echo "Setting up udev rule for dzactutor Device..."

# 定义 udev 规则的内容
UDEV_RULES='KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", SYMLINK+="dzactutor"'

# 创建 udev 规则文件
RULES_FILE="/etc/udev/rules.d/dzactutor.rules"

echo "Creating udev rules file: $RULES_FILE"
echo $UDEV_RULES | sudo tee $RULES_FILE > /dev/null

# 重启 udev 服务以应用更改
echo "Restarting udev service..."
sudo udevadm control --reload
sudo udevadm trigger

# 验证规则是否生效
echo "Checking udev rules..."
udevadm info -q all -n /dev/ttyACM0 | grep DEVLINKS

# 将用户 'duzhong' 添加到 dialout 组以访问串口设备
echo "Adding user 'duzhong' to 'dialout' group..."
sudo usermod -a -G dialout duzhong

echo "Setup complete. Please unplug and replug your device or reboot your system."

