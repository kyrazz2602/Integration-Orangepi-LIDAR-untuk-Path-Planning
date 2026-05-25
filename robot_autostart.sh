#!/bin/bash

# ==========================================
# Skrip Auto-Start Robot untuk Orange Pi
# ==========================================

# 1. Muat environment ROS 2 (Sesuaikan 'humble' dengan versi ROS 2 Anda jika berbeda)
source /opt/ros/humble/setup.bash

# 2. Muat workspace Anda (Sesuaikan path ini dengan lokasi workspace ROS 2 di Orange Pi Anda)
# source /home/orangepi/ros2_ws/install/setup.bash

# Pindah ke direktori tempat script berada
cd "$(dirname "$0")"

echo "Memulai ROS 2 SLAM & Navigation..."
# Menjalankan launch file di background
ros2 launch rplidar_ros navigation_stack_launch.py &
ROS_PID=$!

# Tunggu 5 detik agar Core ROS 2 & Lidar menyala
sleep 5

echo "Memulai Firebase Bridge..."
# Menjalankan script Python Firebase di background
python3 rplidar-firebase-bridge.py &
FIREBASE_PID=$!

echo "Robot berjalan di background (ROS PID: $ROS_PID, Firebase PID: $FIREBASE_PID)"

# Menjaga script tetap hidup
wait $ROS_PID
wait $FIREBASE_PID
