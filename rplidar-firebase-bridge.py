#!/usr/bin/env python3
"""
Robot to Firebase Bridge
Handles:
1. Publishing RPLidar scan data to Firebase
2. Listening to Firebase Commands for Manual Control and A* Path Planning
3. Publishing Robot Status back to Firebase
4. Reading ESP32 sensor data via UART7 and uploading to Firebase /Udara
5. Sending LCD formatting commands to ESP32 LCD
6. Processing fan commands from Firebase and publishing to ROS /fan_cmd
"""

import os
import sys
import math
import time
import socket
import threading
import serial
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

try:
    import firebase_admin
    from firebase_admin import db
    from firebase_admin import credentials
except ImportError:
    print("ERROR: firebase-admin not installed. Run: pip install firebase-admin")
    sys.exit(1)

try:
    from dotenv import load_dotenv

    load_dotenv(os.path.expanduser("~/.env.firebase"))
except ImportError:
    pass


def check_wifi_status() -> str:
    """Check WiFi connection status by trying to connect to Google DNS"""
    try:
        socket.setdefaulttimeout(1)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect(("8.8.8.8", 53))
        return "Connected"
    except Exception:
        return "Offline"


class RobotFirebaseBridge(Node):
    def __init__(self):
        super().__init__("robot_firebase_bridge")

        self.declare_parameter(
            "firebase_db_url",
            "https://airguard-b7ef4-default-rtdb.asia-southeast1.firebasedatabase.app",
        )
        self.firebase_db_url = self.get_parameter("firebase_db_url").value

        # ESP32 UART7 parameters
        self.declare_parameter("esp32_port", "/dev/esp32")
        self.declare_parameter("esp32_baudrate", 115200)
        self.esp32_port = self.get_parameter("esp32_port").value
        self.esp32_baudrate = self.get_parameter("esp32_baudrate").value

        # Fallback to direct UART7 port /dev/ttyAS7 if /dev/esp32 symlink doesn't exist
        if not os.path.exists(self.esp32_port) and os.path.exists("/dev/ttyAS7"):
            self.esp32_port = "/dev/ttyAS7"

        # Initialize Firebase
        self.db_ref = None
        self.firebase_ready = self._init_firebase()

        # ROS 2 Publishers & Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.fan_cmd_pub = self.create_publisher(String, "/fan_cmd", 10)

        # Nav2 Action Client for A* Path Planning
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Firebase Listener
        self.command_ref = self.db_ref.child("Command") if self.firebase_ready else None

        # Start Firebase Listener in a separate thread so it doesn't block ROS spin
        if self.firebase_ready and self.command_ref is not None:
            self.listener_thread = threading.Thread(
                target=self._start_firebase_listener, daemon=True
            )
            self.listener_thread.start()
        else:
            self.get_logger().warn("Firebase not ready. Listener thread skipped.")

        # Parameters
        self.declare_parameter("wheel_radius", 0.0325)
        self.declare_parameter("wheel_base", 0.07)
        self.declare_parameter("odom_publish_interval", 1.0)
        self.declare_parameter("publish_interval", 20)
        self.declare_parameter("manual_speed", 0.2)
        self.declare_parameter("manual_angular_speed", 0.5)
        self.declare_parameter("auto_save_map_interval", 30.0)

        self.R = self.get_parameter("wheel_radius").value
        self.L = self.get_parameter("wheel_base").value
        self.odom_publish_interval = self.get_parameter("odom_publish_interval").value
        self.publish_interval = self.get_parameter("publish_interval").value
        self.manual_speed = self.get_parameter("manual_speed").value
        self.manual_angular_speed = self.get_parameter("manual_angular_speed").value
        self.auto_save_map_interval = self.get_parameter("auto_save_map_interval").value

        # State Variables
        self.frame_count = 0
        self.current_odom = None
        self.last_gerak = None
        self.last_odom_publish_time = self.get_clock().now()
        self.goal_handle = None  # Track active Nav2 goal pose

        # ESP32 and Fan State Variables
        self.latest_pm25 = 0.0
        self.latest_pm10 = 0.0
        self.latest_co = 0.0
        self.latest_voc = 0.0
        self.latest_suhu = 0.0
        self.latest_battery_voltage = 0.0
        self.latest_battery_percent = 0
        self.nav_status = "IDLE"
        self.current_speed_cmd = "OFF"

        self.esp_ser_lock = threading.Lock()
        self.esp_ser = None
        self.esp_connected = False

        # Start ESP32 reader thread
        self.esp_read_thread = threading.Thread(
            target=self._esp32_serial_loop, daemon=True
        )
        self.esp_read_thread.start()

        # LCD update timer (every 3 seconds)
        self.lcd_page = 0
        self.lcd_timer = self.create_timer(3.0, self._update_lcd_timer_callback)

        # Timer for periodic map saving (runs every 30 seconds)
        self.auto_save_timer = self.create_timer(
            self.auto_save_map_interval, self.auto_save_map_callback
        )

        self.get_logger().info("=" * 60)
        self.get_logger().info("Robot Firebase Bridge Started")
        self.get_logger().info(f"ESP32 Serial Port: {self.esp32_port}")
        self.get_logger().info("=" * 60)

    def _init_firebase(self) -> bool:
        try:
            try:
                self.db_ref = db.reference()
                return True
            except ValueError:
                pass

            # Cari file JSON kredensial di beberapa lokasi
            cred_filename = "airguard-b7ef4-firebase-adminsdk-fbsvc-6361f49d51.json"
            script_dir = os.path.dirname(os.path.abspath(__file__))

            paths_to_check = [
                os.path.join(script_dir, cred_filename),  # Lokasi di install space / script dir
                os.path.join(os.getcwd(), cred_filename),  # Lokasi di root workspace saat ini
                os.path.join(os.path.expanduser("~"), cred_filename),  # Lokasi di home directory
            ]

            cred_path = None
            for path in paths_to_check:
                if Path(path).exists():
                    cred_path = path
                    break

            if not cred_path:
                self.get_logger().error(
                    f"Firebase credentials ({cred_filename}) not found in checked paths: {paths_to_check}"
                )
                return False

            self.get_logger().info(f"Using Firebase credentials from: {cred_path}")
            db_url = os.getenv("FIREBASE_DB_URL") or self.firebase_db_url
            cred = credentials.Certificate(cred_path)
            firebase_admin.initialize_app(cred, {"databaseURL": db_url})

            self.db_ref = db.reference()
            self.get_logger().info("✓ Firebase connected successfully")
            return True
        except Exception as e:
            self.get_logger().error(f"Firebase init error: {e}")
            return False

    def _start_firebase_listener(self):
        """Listen to Firebase Realtime Database changes on the 'Command' node"""
        try:
            if self.command_ref is not None:
                self.command_ref.listen(self._firebase_command_callback)
        except Exception as e:
            self.get_logger().error(f"Failed to start Firebase listener: {e}")

    def _esp32_serial_loop(self):
        """Thread to maintain serial connection and read sensor data from ESP32"""
        error_count = 0
        MAX_ERRORS = 10

        while rclpy.ok():
            if not self.esp_connected:
                self.get_logger().info(f"Connecting to ESP32 on {self.esp32_port}...")
                try:
                    with self.esp_ser_lock:
                        if self.esp_ser is not None:
                            try:
                                self.esp_ser.close()
                            except Exception:
                                pass
                        self.esp_ser = serial.Serial(self.esp32_port, self.esp32_baudrate, timeout=1.0)
                        self.esp_connected = True
                        error_count = 0
                    self.get_logger().info(f"Connected to ESP32 on {self.esp32_port}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to connect to ESP32: {e}")
                    time.sleep(3.0)
                    continue

            try:
                ser = None
                with self.esp_ser_lock:
                    if self.esp_ser is not None and self.esp_connected:
                        ser = self.esp_ser

                if ser is None:
                    time.sleep(0.1)
                    continue

                line_bytes = ser.readline()
                if not line_bytes:
                    continue

                line = line_bytes.decode("utf-8", errors="ignore").strip()

                # Parse sensor values from ESP32
                # Format: $DATA,pm25,pm10,co,voc,suhu,voltage,percent
                if line.startswith("$DATA,"):
                    parts = line.split(",")
                    if len(parts) == 8:
                        try:
                            self.latest_pm25 = float(parts[1])
                            self.latest_pm10 = float(parts[2])
                            self.latest_co = float(parts[3])
                            self.latest_voc = float(parts[4])
                            self.latest_suhu = float(parts[5])
                            self.latest_battery_voltage = float(parts[6])
                            self.latest_battery_percent = int(parts[7])

                            # Upload data to Firebase RTDB
                            self._upload_sensor_data_to_firebase()
                            
                            # If AUTO mode is active, send updated AUTO:pm25:pm10:co:voc to Mega
                            if self.current_speed_cmd == "AUTO":
                                self._update_fan_command()

                            error_count = 0
                        except ValueError as e:
                            self.get_logger().warn(f"Failed to parse ESP32 line '{line}': {e}")
            except Exception as e:
                error_count += 1
                self.get_logger().warn(f"Error reading from ESP32 (attempt {error_count}/{MAX_ERRORS}): {e}")
                if error_count >= MAX_ERRORS:
                    self.get_logger().error(f"Max ESP32 serial errors reached ({MAX_ERRORS}). Reconnecting...")
                    with self.esp_ser_lock:
                        self.esp_connected = False
                        if self.esp_ser is not None:
                            try:
                                self.esp_ser.close()
                            except Exception:
                                pass
                            self.esp_ser = None
                time.sleep(1.0)

    def _upload_sensor_data_to_firebase(self):
        if not self.firebase_ready or self.db_ref is None:
            return

        try:
            data = {
                "PM25": self.latest_pm25,
                "PM10": self.latest_pm10,
                "CO": self.latest_co,
                "VOC": self.latest_voc,
                "Suhu": self.latest_suhu,
                "Tegangan": self.latest_battery_voltage,
                "Persentase": self.latest_battery_percent
            }
            self.db_ref.child("Udara").update(data)
        except Exception as e:
            self.get_logger().error(f"Failed to upload ESP32 sensor data to Firebase: {e}")

    def _process_speed_command(self, speed: str):
        """Process speed command and publish to /fan_cmd"""
        self.current_speed_cmd = speed
        self._update_fan_command()

    def _update_fan_command(self):
        """Publish the appropriate command string to /fan_cmd"""
        msg = String()
        if self.current_speed_cmd == "AUTO":
            msg.data = f"AUTO:{self.latest_pm25:.1f}:{self.latest_pm10:.1f}:{self.latest_co:.1f}:{self.latest_voc:.4f}"
        else:
            msg.data = f"MANUAL:{self.current_speed_cmd}"

        self.fan_cmd_pub.publish(msg)
        self.get_logger().info(f"Published to /fan_cmd: {msg.data}")

        if self.firebase_ready and self.db_ref is not None:
            try:
                self.db_ref.child("Status").update({"kipas": self.current_speed_cmd})
            except Exception as e:
                self.get_logger().error(f"Failed to update Status/kipas in Firebase: {e}")

    def _update_lcd_timer_callback(self):
        """Timer to cycle pages and write formatting to ESP32 LCD"""
        wifi_status = check_wifi_status()
        fb_status = "OK" if self.firebase_ready else "Off"

        # Increment page (0 to 5)
        self.lcd_page = (self.lcd_page + 1) % 6

        line0 = ""
        line1 = ""

        if self.lcd_page == 0:
            line0 = f"PM2.5: {self.latest_pm25:.1f}"
            if self.latest_pm25 <= 35.4:
                line1 = "Status: Baik"
            elif self.latest_pm25 <= 125.4:
                line1 = "Status:Perhatian"
            else:
                line1 = "Status: Bahaya"

        elif self.lcd_page == 1:
            line0 = f"PM10 : {self.latest_pm10:.1f}"
            if self.latest_pm10 <= 154.0:
                line1 = "Status: Baik"
            elif self.latest_pm10 <= 354.0:
                line1 = "Status:Perhatian"
            else:
                line1 = "Status: Bahaya"

        elif self.lcd_page == 2:
            line0 = f"CO  : {self.latest_co:.1f} ppm"
            line1 = f"VOC : {self.latest_voc:.3f} mg"

        elif self.lcd_page == 3:
            line0 = f"Suhu: {self.latest_suhu:.1f} C"
            line1 = f"Bat : {self.latest_battery_percent}% {self.latest_battery_voltage:.1f}V"

        elif self.lcd_page == 4:
            line0 = f"WiFi: {wifi_status}"
            line1 = f"Firebase: {fb_status}"

        elif self.lcd_page == 5:
            line0 = f"Nav : {self.nav_status}"
            line1 = f"Kipas: {self.current_speed_cmd}"

        self.write_lcd(line0, line1)

    def write_lcd(self, line0: str, line1: str):
        """Send lines of text to ESP32 LCD"""
        line0 = line0[:16]
        line1 = line1[:16]

        cmd0 = f"$LCD,0,{line0}\n"
        cmd1 = f"$LCD,1,{line1}\n"

        with self.esp_ser_lock:
            if self.esp_connected and self.esp_ser is not None:
                try:
                    self.esp_ser.write(cmd0.encode("utf-8"))
                    time.sleep(0.05)  # small delay to prevent buffer overflow on ESP32
                    self.esp_ser.write(cmd1.encode("utf-8"))
                except Exception as e:
                    self.get_logger().error(f"Failed to write to ESP32 LCD: {e}")

    def cancel_nav_goal(self):
        """Cancel active Nav2 autonomous navigation goal if any"""
        if self.goal_handle is not None:
            self.get_logger().info(
                "Canceling active Nav2 goal due to manual command/STOP..."
            )
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def execute_manual_movement(self, gerak: str):
        """Processes and publishes twist command to ROS 2 cmd_vel, canceling nav goal if moving"""
        if gerak != self.last_gerak:
            self.last_gerak = gerak

            # Cancel active autonomous goal if a manual control input or STOP is received
            if gerak in ["MAJU", "MUNDUR", "KIRI", "KANAN", "DIAM"]:
                self.cancel_nav_goal()

            twist = Twist()
            speed = self.manual_speed  # m/s
            angular_speed = self.manual_angular_speed  # rad/s

            if gerak == "MAJU":
                twist.linear.x = speed
            elif gerak == "MUNDUR":
                twist.linear.x = -speed
            elif gerak == "KIRI":
                twist.angular.z = angular_speed
            elif gerak == "KANAN":
                twist.angular.z = -angular_speed
            elif gerak == "DIAM":
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"Published Manual Twist: {gerak}")

    def _firebase_command_callback(self, event):
        """Triggered when data in /Command changes"""
        path = event.path
        data = event.data
        self.get_logger().info(f"Received Firebase Command Update: {path} -> {data}")

        if data is None:
            # Ignore deletion events
            return

        # 1. Handle explicit change to manual movement command
        if path == "/gerak":
            gerak = str(data).upper()
            self.execute_manual_movement(gerak)

        elif path == "/save_map":
            if data is True or str(data).lower() in ["true", "1"]:
                self.get_logger().info("Manual map save command received from Firebase!")
                threading.Thread(target=self.save_map_worker, daemon=True).start()
                if self.command_ref is not None:
                    self.command_ref.child("save_map").set(False)

        # 2. Handle change to fan speed command
        elif path == "/speed":
            speed = str(data).upper()
            self._process_speed_command(speed)

        # 3. Handle initial bulk load or specific updates containing goals
        elif path == "/" and isinstance(data, dict):
            if (
                "goal_x" in data
                and "goal_y" in data
                and data["goal_x"] is not None
                and data["goal_y"] is not None
            ):
                goal_x = float(data["goal_x"])
                goal_y = float(data["goal_y"])
                self.send_nav_goal(goal_x, goal_y)

                # Clear goal from DB after reading so it doesn't loop
                if self.command_ref is not None:
                    self.command_ref.child("goal_x").delete()
                    self.command_ref.child("goal_y").delete()

            if "gerak" in data and data["gerak"] is not None:
                gerak = str(data["gerak"]).upper()
                self.execute_manual_movement(gerak)

            if "speed" in data and data["speed"] is not None:
                speed = str(data["speed"]).upper()
                self._process_speed_command(speed)

            if "save_map" in data and (data["save_map"] is True or str(data["save_map"]).lower() in ["true", "1"]):
                self.get_logger().info("Manual map save command received from Firebase (bulk)!")
                threading.Thread(target=self.save_map_worker, daemon=True).start()
                if self.command_ref is not None:
                    self.command_ref.child("save_map").set(False)

        # 4. Handle individual goal_x or goal_y updates if they are set separately (fallback)
        elif path in ["/goal_x", "/goal_y"]:
            if self.command_ref is not None:
                command_data = self.command_ref.get()
                if command_data and "goal_x" in command_data and "goal_y" in command_data:
                    if (
                        command_data["goal_x"] is not None
                        and command_data["goal_y"] is not None
                    ):
                        goal_x = float(command_data["goal_x"])
                        goal_y = float(command_data["goal_y"])
                        self.send_nav_goal(goal_x, goal_y)

                        # Clear goal from DB after reading so it doesn't loop
                        self.command_ref.child("goal_x").delete()
                        self.command_ref.child("goal_y").delete()

    def send_nav_goal(self, x, y):
        """Send goal to Nav2 Action Server"""
        self.get_logger().info(f"Sending Nav2 Goal: x={x}, y={y}")
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 Action Server not available!")
            if self.firebase_ready and self.db_ref is not None:
                self.db_ref.child("Status").update(
                    {"navigation_status": "SERVER_UNAVAILABLE"}
                )
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Facing forward

        self.nav_status = "SENDING_GOAL"
        if self.firebase_ready and self.db_ref is not None:
            self.db_ref.child("Status").update({"navigation_status": "SENDING_GOAL"})

        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            self.goal_handle = None
            self.nav_status = "REJECTED"
            if self.firebase_ready and self.db_ref is not None:
                self.db_ref.child("Status").update({"navigation_status": "REJECTED"})
            return

        self.get_logger().info("Goal accepted :)")
        self.nav_status = "NAVIGATING"
        if self.firebase_ready and self.db_ref is not None:
            self.db_ref.child("Status").update({"navigation_status": "NAVIGATING"})

        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.goal_handle = None  # Reset active goal reference on completion
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
            self.nav_status = "SUCCEEDED"
            if self.firebase_ready and self.db_ref is not None:
                self.db_ref.child("Status").update({"navigation_status": "SUCCEEDED"})
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Goal aborted!")
            self.nav_status = "ABORTED"
            if self.firebase_ready and self.db_ref is not None:
                self.db_ref.child("Status").update({"navigation_status": "ABORTED"})
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal canceled!")
            self.nav_status = "CANCELED"
            if self.firebase_ready and self.db_ref is not None:
                self.db_ref.child("Status").update({"navigation_status": "CANCELED"})
        else:
            self.get_logger().info(f"Goal finished with status code: {status}")
            self.nav_status = f"FINISHED_CODE_{status}"
            if self.firebase_ready and self.db_ref is not None:
                self.db_ref.child("Status").update(
                    {"navigation_status": f"FINISHED_CODE_{status}"}
                )

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg

        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_publish_time).nanoseconds / 1e9
        if dt >= self.odom_publish_interval:
            self.last_odom_publish_time = current_time
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            # Convert quaternion to yaw
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            # Calculate motor wheel RPM from linear & angular velocities
            # using differential drive kinematics matching hardware settings
            R = self.R  # wheel radius
            L = self.L  # wheel base
            v = msg.twist.twist.linear.x
            w = msg.twist.twist.angular.z

            v_left = v - (w * L) / 2.0
            v_right = v + (w * L) / 2.0

            rpm_left = (v_left / (2.0 * math.pi * R)) * 60.0
            rpm_right = (v_right / (2.0 * math.pi * R)) * 60.0

            # Determine actual movement status based on velocities
            if abs(v) < 0.015 and abs(w) < 0.05:
                actual_gerak = "DIAM"
            elif abs(v) >= abs(w):
                actual_gerak = "MAJU" if v > 0 else "MUNDUR"
            else:
                actual_gerak = "KIRI" if w > 0 else "KANAN"

            # Apply noise filter threshold to RPM (ignore values < 1.5 RPM to handle jitter)
            # If actual_gerak is DIAM, force both RPMs to 0.0
            if actual_gerak == "DIAM" or (abs(rpm_left) < 1.5 and abs(rpm_right) < 1.5):
                rpm_left = 0.0
                rpm_right = 0.0
            else:
                if abs(rpm_left) < 1.5:
                    rpm_left = 0.0
                if abs(rpm_right) < 1.5:
                    rpm_right = 0.0

            if self.firebase_ready and self.db_ref is not None:
                try:
                    self.db_ref.child("Status").update(
                        {
                            "x": round(x, 2),
                            "y": round(y, 2),
                            "yaw": round(yaw, 2),
                            "linear_velocity": round(v, 2),
                            "angular_velocity": round(w, 2),
                            "rpmKiri": round(abs(rpm_left), 1),
                            "rpmKanan": round(abs(rpm_right), 1),
                            "gerak": actual_gerak,
                        }
                    )
                except Exception as e:
                    self.get_logger().error(f"Firebase odom publish error: {e}")

    def scan_callback(self, msg: LaserScan):
        if not self.firebase_ready or self.db_ref is None:
            return
        self.frame_count += 1
        if self.frame_count % self.publish_interval != 0:
            return

        # Simplistic LiDAR processing to prevent huge payload
        min_distance = float("inf")
        for distance in msg.ranges:
            if math.isfinite(distance) and 0.01 < distance < 12.0:
                if distance < min_distance:
                    min_distance = distance

        data = {
            "timestamp": datetime.now().isoformat(),
            "jarak_terdekat_cm": (
                round(min_distance * 100, 2) if min_distance != float("inf") else 0
            ),
        }

        try:
            self.db_ref.child("LiDAR").child("latest").set(data)
        except Exception as e:
            self.get_logger().error(f"Firebase publish error: {e}")

    def auto_save_map_callback(self):
        """Timer callback to trigger map auto-save in a background thread"""
        if self.current_odom is not None:
            self.get_logger().info("Periodic map auto-save triggered...")
            threading.Thread(target=self.save_map_worker, daemon=True).start()

    def save_map_worker(self):
        """Worker thread to run map_saver_cli and slam_toolbox save_map service"""
        try:
            import subprocess
            script_dir = os.path.dirname(os.path.abspath(__file__))
            map_path = os.path.join(script_dir, "map_rumah")

            # 1. Save standard map using map_saver_cli
            cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path]
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=12.0)

            # 2. Save using slam_toolbox serialize service (saves pose graph)
            try:
                cmd_toolbox = [
                    "ros2", "service", "call",
                    "/slam_toolbox/save_map",
                    "slam_toolbox/srv/SaveMap",
                    f"{{name: {{data: '{map_path}'}}}}"
                ]
                subprocess.run(cmd_toolbox, capture_output=True, text=True, timeout=8.0)
            except Exception as e:
                self.get_logger().warning(f"Slam toolbox serialize service warning: {e}")

            if res.returncode == 0:
                self.get_logger().info(f"✓ Map auto-saved successfully to {map_path}")
                if self.firebase_ready and self.db_ref is not None:
                    self.db_ref.child("Status").update({
                        "map_auto_save_status": "SUCCESS",
                        "map_last_saved": datetime.now().isoformat()
                    })
            else:
                self.get_logger().warning(f"Map auto-save warning: {res.stderr}")
                if self.firebase_ready and self.db_ref is not None:
                    self.db_ref.child("Status").update({
                        "map_auto_save_status": "FAILED",
                        "map_auto_save_error": res.stderr[:100]
                    })
        except subprocess.TimeoutExpired:
            self.get_logger().error("Map auto-save timeout expired!")
        except Exception as e:
            self.get_logger().error(f"Error during auto-save: {e}")


def main(args=None):
    rclpy.init(args=args)
    bridge = RobotFirebaseBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
