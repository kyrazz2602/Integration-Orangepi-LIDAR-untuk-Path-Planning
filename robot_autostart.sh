#!/bin/bash

# ==========================================
# Skrip Auto-Start Robot untuk Orange Pi
# ==========================================

# 1. Muat environment ROS 2 (Sesuaikan 'humble' dengan versi ROS 2 Anda jika berbeda)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Pindah ke direktori tempat script berada
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 2. Deteksi dan muat workspace ROS 2 secara dinamis
# Mencari file install/setup.bash naik ke folder induk
WS_SETUP=""
CURRENT_DIR="$SCRIPT_DIR"
for i in {1..5}; do
    if [ -f "$CURRENT_DIR/install/setup.bash" ]; then
        WS_SETUP="$CURRENT_DIR/install/setup.bash"
        break
    fi
    CURRENT_DIR="$(dirname "$CURRENT_DIR")"
done

# Fallback ke path default jika tidak terdeteksi dinamis
if [ -z "$WS_SETUP" ] && [ -f "/home/orangepi/ros2_ws/install/setup.bash" ]; then
    WS_SETUP="/home/orangepi/ros2_ws/install/setup.bash"
fi

if [ -n "$WS_SETUP" ]; then
    echo "Sourcing workspace: $WS_SETUP"
    source "$WS_SETUP"
else
    echo "Peringatan: workspace install/setup.bash tidak ditemukan!"
fi

# 3. Deteksi dan aktifkan virtual environment (.venv) jika ada
if [ -d ".venv" ] && [ -f ".venv/bin/activate" ]; then
    echo "Mengaktifkan virtual environment (.venv)..."
    source .venv/bin/activate
elif [ -d "../.venv" ] && [ -f "../.venv/bin/activate" ]; then
    echo "Mengaktifkan virtual environment dari folder induk (../.venv)..."
    source ../.venv/bin/activate
fi

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
