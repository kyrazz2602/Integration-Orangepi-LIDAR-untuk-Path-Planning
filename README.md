# Integration OrangePi LIDAR untuk Path Planning (Airguard Autonomous Robot)

Repository ini berisi kode integrasi untuk robot otonom berbasis **ROS 2 (Humble)** yang berjalan di **Orange Pi**, dilengkapi dengan sensor **RPLidar A1**, kontroler **Arduino Mega**, **ESP32** (kendali manual), serta sinkronisasi data dan kendali via **Firebase Realtime Database**.

---

## 📌 Arsitektur Sistem

Robot ini menggunakan sistem kendali terdistribusi:
1. **Orange Pi (ROS 2 Humble)**: 
   - Menjalankan SLAM (GMapping / SLAM Toolbox) untuk pemetaan.
   - Nav2 (AMCL, A* Global Planner, DWA Local Planner) untuk navigasi otonom.
   - Jembatan data antara ROS 2 dengan Firebase via [rplidar-firebase-bridge.py](./rplidar-firebase-bridge.py).
   - Jembatan komunikasi serial dengan Arduino via [arduino_serial_bridge.py](./arduino_serial_bridge.py).
2. **Arduino Mega (Hardware Controller)**:
   - Mengendalikan motor driver BTS7960 (2 roda) menggunakan algoritma kontrol PID.
   - Membaca umpan balik kecepatan dari Encoder H12 (menghitung RPM roda) dan mengirimkan data odometri kembali ke Orange Pi.
   - Mengendalikan kipas penyaring udara secara manual/otomatis berdasarkan tingkat polutan (PM2.5, PM10, CO, VOC).
3. **ESP32**:
   - Berfungsi sebagai kendali manual offline via Serial3 ke Arduino Mega.
4. **Firebase Realtime Database**:
   - Menyimpan perintah manual (`Command/gerak`), koordinat tujuan navigasi otonom (`Command/goal_x`, `Command/goal_y`), dan menerima data jarak terdekat dari sensor LiDAR (`LiDAR/latest`).

---

## 📂 Struktur Direktori Utama

* **`arduino_firmware/`**: Berisi firmware [arduino_firmware.ino](./arduino_firmware/arduino_firmware.ino) untuk Arduino Mega.
* **`config/`**: Berisi parameter navigasi ROS 2 [nav2_params.yaml](./config/nav2_params.yaml).
* **`launch/`**: Berisi skrip peluncuran ROS 2:
  - [navigation_stack_launch.py](./launch/navigation_stack_launch.py): Meluncurkan RPLidar, TF static, GMapping, Arduino Bridge, dan Nav2.
  - [slam_nav_launch.py](./launch/slam_nav_launch.py): Alternatif launch menggunakan SLAM Toolbox online sync.
* **`arduino_serial_bridge.py`**: Node ROS 2 untuk menjembatani pesan `/cmd_vel` ke Arduino (RPM) dan data RPM roda ke `/odom`.
* **`rplidar-firebase-bridge.py`**: Skrip penghubung data ROS 2 dengan Firebase.
* **`robot_autostart.sh`**: Skrip bash untuk menjalankan semua komponen saat startup Orange Pi.
* **`robot_core.service`**: Konfigurasi systemd service untuk menjalankan autostart di latar belakang.

---

## 🛠️ Komponen Kode & Protokol Komunikasi

### 1. Arduino Serial Bridge (`arduino_serial_bridge.py`)
Menerjemahkan perintah pergerakan dari ROS ke Arduino dan sebaliknya.
* **Subscribers**: `/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))
* **Publishers**: `/odom` ([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)), `/tf` (`odom` -> `base_link`)
* **Protokol Serial (115200 bps)**:
  - Mengirim ke Arduino: `CMD,VEL,<rpm_kiri>,<rpm_kanan>\n`
  - Menerima dari Arduino: `ODOM,<rpm_kiri>,<rpm_kanan>\n`

### 2. Robot to Firebase Bridge (`rplidar-firebase-bridge.py`)
Menyediakan interface kontrol berbasis cloud.
* **Subscribers**: `/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)) & `/odom`
* **Action Client**: `/navigate_to_pose` ([nav2_msgs/action/NavigateToPose](https://navigation.ros.org/))
* **Protokol Firebase**:
  - Membaca `Command/gerak` (Nilai: `MAJU`, `MUNDUR`, `KIRI`, `KANAN`, `DIAM`) -> Diubah menjadi `/cmd_vel`.
  - Membaca `Command/goal_x` & `Command/goal_y` -> Diubah menjadi Nav2 Action Goal.
  - Menulis `LiDAR/latest` (Nilai: `timestamp` & `jarak_terdekat_cm`).

### 3. Firmware Arduino Mega (`arduino_firmware.ino`)
* Mengontrol 2 buah motor DC dengan driver BTS7960 dan Encoder H12 (PPR: 241.0).
* Memiliki kendali PID internal dengan parameter: `Kp = 0.15`, `Ki = 0.8`, `Kd = 0.01`.
* **Serial Port**:
  - `Serial` : Log Debug USB.
  - `Serial2`: Jalur komunikasi data ke Orange Pi (115200 baud).
  - `Serial3`: Jalur komunikasi data ke ESP32 (115200 baud).
* **Fungsi Kipas**:
  - Mendukung mode manual (`FAN:HIGH`, `FAN:NORMAL`, `FAN:LOW`, `FAN:OFF`) dan mode otomatis (`AUTO:<pm25>:<pm10>:<co>:<voc>`) yang menyesuaikan kecepatan kipas berdasarkan tingkat polutan udara.

---

## 🚀 Cara Menjalankan Sistem

### Persyaratan Awal
1. Pastikan ROS 2 Humble telah terinstal di Orange Pi.
2. Instal library python yang dibutuhkan:
   ```bash
   pip install firebase-admin python-dotenv pyserial
   ```
3. Konfigurasikan kredensial Firebase Anda pada file `airguard-b7ef4-firebase-adminsdk-fbsvc-6361f49d51.json` di direktori root paket ini.

### 1. Build Workspace
Pindahkan proyek ini ke direktori `src` workspace colcon Anda, lalu build:
```bash
cd ~/ros2_ws/
colcon build --symlink-install
source install/setup.bash
```

### 2. Konfigurasi Hak Akses Port Serial
Buat udev rules agar port USB Arduino dan RPLidar dapat diakses tanpa `sudo`:
```bash
# Jalankan script udev rule rplidar
cd src/rplidar_ros
source scripts/create_udev_rules.sh
```

### 3. Menjalankan secara Manual
**Menjalankan Navigation & SLAM Stack**:
```bash
ros2 launch rplidar_ros navigation_stack_launch.py
```
**Menjalankan Firebase Bridge**:
```bash
python3 rplidar-firebase-bridge.py
```

### 4. Konfigurasi Autostart (Orange Pi)
Untuk membuat robot berjalan otomatis setelah booting:
1. Edit file [robot_core.service](./robot_core.service) dan sesuaikan jalur direktori dengan konfigurasi Orange Pi Anda.
2. Salin file service ke folder systemd:
   ```bash
   sudo cp robot_core.service /etc/systemd/system/
   ```
3. Aktifkan dan jalankan service:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable robot_core.service
   sudo systemctl start robot_core.service
   ```
4. Cek status service:
   ```bash
   sudo systemctl status robot_core.service
   ```

---

## 📐 Koordinat Frame TF RPLIDAR
Posisi RPLIDAR harus disesuaikan dengan posisi fisik robot pada transformasi statis berikut di launch file:
```python
arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'laser_link']
```
* **X**: `0.1` meter (10 cm ke arah depan)
* **Y**: `0` meter
* **Z**: `0.2` meter (20 cm dari permukaan base)
