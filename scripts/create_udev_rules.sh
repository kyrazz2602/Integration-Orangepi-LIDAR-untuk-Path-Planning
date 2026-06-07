#!/bin/bash

echo "remap the device serial port(ttyUSBX/ttyACMX) to rplidar and arduino"
echo "check connected devices using: ls -l /dev | grep -E 'rplidar|arduino'"
echo "start copy rplidar.rules to /etc/udev/rules.d/"

# Try to find the directory of the script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ -f "$SCRIPT_DIR/rplidar.rules" ]; then
    sudo cp "$SCRIPT_DIR/rplidar.rules" /etc/udev/rules.d/
elif [ -f "scripts/rplidar.rules" ]; then
    sudo cp scripts/rplidar.rules /etc/udev/rules.d/
else
    # Fallback to colcon_cd if available
    if [ -f /usr/share/colcon_cd/function/colcon_cd.sh ]; then
        source /usr/share/colcon_cd/function/colcon_cd.sh
        if colcon_cd rplidar_ros; then
            sudo cp scripts/rplidar.rules /etc/udev/rules.d/
        else
            echo "Error: colcon_cd failed to find rplidar_ros package."
            exit 1
        fi
    else
        echo "Error: Cannot find rplidar.rules file. Please run this script from the package directory."
        exit 1
    fi
fi

echo -e "\nRestarting udev\n"
if systemctl list-units --type=service | grep -q udev; then
    sudo systemctl restart udev
else
    sudo service udev reload
    sudo service udev restart
fi
sudo udevadm control --reload && sudo udevadm trigger
echo "finish"
