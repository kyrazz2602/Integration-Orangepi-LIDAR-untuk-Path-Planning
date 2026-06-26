#!/bin/bash

# Redirect stdout and stderr to a log file
exec > /tmp/robot_autostart.log 2>&1

echo "=========================================="
echo "Skrip Auto-Start Robot untuk Orange Pi"
echo "=========================================="
echo "Waktu Mulai: $(date)"

# Loop tunggu device /dev/rplidar & /dev/arduino (maks 30 detik)
echo "Checking for hardware devices..."
MAX_WAIT=30
WAIT_TIME=0
while [ $WAIT_TIME -lt $MAX_WAIT ]; do
    if [ -e "/dev/rplidar" ] && [ -e "/dev/arduino" ]; then
        echo "Devices /dev/rplidar and /dev/arduino are ready!"
        break
    fi
    echo "Waiting for devices /dev/rplidar and /dev/arduino... (${WAIT_TIME}/${MAX_WAIT}s)"
    sleep 1
    WAIT_TIME=$((WAIT_TIME + 1))
done

if [ ! -e "/dev/rplidar" ] || [ ! -e "/dev/arduino" ]; then
    echo "Warning: Timeout waiting for devices. Continuing anyway..."
    echo "Status: /dev/rplidar exists: $([ -e /dev/rplidar ] && echo 'YES' || echo 'NO'), /dev/arduino exists: $([ -e /dev/arduino ] && echo 'YES' || echo 'NO')"
fi

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
# ==========================================
# CLEANUP: Bunuh semua proses lama
# ==========================================
echo "[CLEANUP] Membersihkan proses lama..."
pkill -9 -f "arduino_serial_bridge" 2>/dev/null || true
pkill -9 -f "rplidar_composition"   2>/dev/null || true
pkill -9 -f "async_slam_toolbox"    2>/dev/null || true
pkill -9 -f "nav2"                  2>/dev/null || true
pkill -9 -f "rosbridge_websocket"   2>/dev/null || true
pkill -9 -f "rplidar-firebase-bridge" 2>/dev/null || true

# Tunggu port benar-benar bebas
sleep 3

# Verifikasi port serial agar tidak ada proses tersangkut
for port in "/dev/ttyAS4" "/dev/arduino"; do
    if [ -e "$port" ] && lsof "$port" 2>/dev/null | grep -q python3; then
        echo "[WARN] $port masih dipegang! Force kill..."
        fuser -k "$port" 2>/dev/null || true
        sleep 2
    fi
done

echo "[OK] Semua proses lama sudah dibersihkan"

# Inisialisasi variabel PID kosong agar trap cleanup aman
ROS_PID=""
ROSBRIDGE_PID=""
FIREBASE_PID=""

# Fungsi cleanup untuk mematikan semua proses anak saat skrip dihentikan/dimatikan
cleanup() {
    echo "=========================================="
    echo "Menghentikan semua proses robot..."
    echo "=========================================="
    
    # Hentikan secara halus (SIGTERM)
    [ -n "$ROS_PID" ] && kill -TERM "$ROS_PID" 2>/dev/null
    [ -n "$ROSBRIDGE_PID" ] && kill -TERM "$ROSBRIDGE_PID" 2>/dev/null
    [ -n "$FIREBASE_PID" ] && kill -TERM "$FIREBASE_PID" 2>/dev/null
    sleep 2
    
    # Force kill jika masih berjalan (SIGKILL)
    [ -n "$ROS_PID" ] && kill -9 "$ROS_PID" 2>/dev/null
    [ -n "$ROSBRIDGE_PID" ] && kill -9 "$ROSBRIDGE_PID" 2>/dev/null
    [ -n "$FIREBASE_PID" ] && kill -9 "$FIREBASE_PID" 2>/dev/null
}
# Registrasi signal handler
trap cleanup EXIT SIGINT SIGTERM

echo "Memulai ROS 2 SLAM & Navigation..."
# Menjalankan launch file di background
ros2 launch rplidar_ros navigation_stack_launch.py &
ROS_PID=$!

echo "Memulai ROSBridge Websocket Server..."
# Menjalankan websocket server di background
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!

# Tunggu 5 detik agar Core ROS 2 & Lidar menyala
sleep 5

echo "Memulai Firebase Bridge..."
# Menjalankan script Python Firebase di background
python3 rplidar-firebase-bridge.py &
FIREBASE_PID=$!

echo "Robot berjalan di background (ROS PID: $ROS_PID, ROSBridge PID: $ROSBRIDGE_PID, Firebase PID: $FIREBASE_PID)"

# Menjaga script tetap hidup
wait $ROS_PID
wait $ROSBRIDGE_PID
wait $FIREBASE_PID
