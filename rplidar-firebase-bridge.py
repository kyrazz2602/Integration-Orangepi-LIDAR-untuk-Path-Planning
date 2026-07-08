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
7. Real-time map grid, scan points, A* path upload to Firebase for dashboard
"""

import os
import sys
import math
import time
import socket
import threading
import serial
import base64
from datetime import datetime
from pathlib import Path as FilePath

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
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
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.plan_sub = self.create_subscription(
            Path, "/plan", self.plan_callback, 10
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

        # Real-time mapping state variables
        self.latest_map_data = None       # Latest OccupancyGrid message
        self.latest_scan_points = []      # Latest LiDAR scan points [{x,y},...]
        self.latest_plan_path = []         # Latest A* path [{x,y},...]
        self.map_data_lock = threading.Lock()

        # ESP32 and Fan State Variables
        self.latest_pm25 = 0.0
        self.latest_pm10 = 0.0
        self.latest_co = 0.0
        self.latest_voc = 0.0
        self.latest_suhu = 0.0
        self.latest_battery_voltage = 0.0
        self.latest_battery_percent = 0
        self.latest_arus = 0.0
        self.nav_status = "IDLE"
        self.current_speed_cmd = "OFF"
        self.is_auto_mode = True

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

        # Timer for periodic map data upload to Firebase (every 5 seconds)
        self.map_upload_timer = self.create_timer(5.0, self._upload_map_data_callback)

        # Timer for periodic WiFi scanning (runs every 45 seconds)
        self.wifi_scan_timer = self.create_timer(45.0, self._wifi_scan_timer_callback)
        # Trigger initial WiFi scan in background
        threading.Thread(target=self._scan_wifi_worker, daemon=True).start()

        # Background thread to monitor internet/online status
        self.is_online = False
        self.wifi_status_thread = threading.Thread(
            target=self._wifi_status_monitor_loop, daemon=True
        )
        self.wifi_status_thread.start()

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
                os.path.join(script_dir, cred_filename),
                os.path.join(os.getcwd(), cred_filename),
                os.path.join(os.path.expanduser("~"), cred_filename),
            ]

            cred_path = None
            for path in paths_to_check:  # noqa: F841
                if FilePath(path).exists():
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
                if not line:
                    continue

                # Log raw line for debugging
                self.get_logger().info(f"[ESP32 RAW] '{line}'")

                # Parse sensor values from ESP32
                # Accept both "DATA," and "$DATA," prefixes
                clean_line = line.lstrip("$")
                if clean_line.startswith("DATA,"):
                    parts = clean_line.split(",")
                    if len(parts) >= 8:
                        try:
                            self.latest_pm25 = float(parts[1])
                            self.latest_pm10 = float(parts[2])
                            self.latest_co = float(parts[3])
                            self.latest_voc = float(parts[4])
                            self.latest_suhu = float(parts[5])
                            self.latest_battery_voltage = float(parts[6])
                            self.latest_battery_percent = int(parts[7])
                            if len(parts) >= 9:
                                self.latest_arus = float(parts[8])
                            else:
                                self.latest_arus = 0.0

                            # Upload data to Firebase RTDB
                            self._upload_sensor_data_to_firebase()
                            
                            # If AUTO mode is active, send updated AUTO:pm25:pm10:co:voc to Mega
                            if self.is_auto_mode:
                                self._update_fan_command()

                            error_count = 0
                        except ValueError as e:
                            self.get_logger().warn(f"Failed to parse ESP32 fields from '{line}': {e}")
                    else:
                        self.get_logger().warn(f"ESP32 line has insufficient fields ({len(parts)} < 8): '{line}'")
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
                "Persentase": self.latest_battery_percent,
                "Arus": self.latest_arus
            }
            self.db_ref.child("Udara").update(data)
            self.get_logger().info(f"✓ Uploaded sensor data to Firebase: {data}")
        except Exception as e:
            self.get_logger().error(f"Failed to upload ESP32 sensor data to Firebase: {e}")

    def _process_speed_command(self, speed: str):
        """Process speed command and publish to /fan_cmd"""
        self.current_speed_cmd = speed
        self._update_fan_command()

    def _update_fan_command(self):
        """Publish the appropriate command string to /fan_cmd"""
        msg = String()
        if self.is_auto_mode:
            msg.data = f"AUTO:{self.latest_pm25:.1f}:{self.latest_pm10:.1f}:{self.latest_co:.1f}:{self.latest_voc:.4f}"
            speed_val = "AUTO"
        else:
            msg.data = f"MANUAL:{self.current_speed_cmd}"
            speed_val = self.current_speed_cmd

        self.fan_cmd_pub.publish(msg)
        self.get_logger().info(f"Published to /fan_cmd: {msg.data}")

        if self.firebase_ready and self.db_ref is not None:
            try:
                self.db_ref.child("Status").update({"kipas": speed_val})
            except Exception as e:
                self.get_logger().error(f"Failed to update Status/kipas in Firebase: {e}")

    def _update_lcd_timer_callback(self):
        """Timer to send navigation status to ESP32 LCD"""
        status = "IDLE"
        mode = "MANUAL"
        
        # Check if manual movement is active
        if self.last_gerak is not None and self.last_gerak != "DIAM":
            status = self.last_gerak
            mode = "MANUAL"
        # Check if autonomous navigation is active
        elif self.nav_status in ["SENDING_GOAL", "NAVIGATING"]:
            status = self.nav_status
            mode = "OTONOM"
        # Check if a recent navigation result was achieved (show it for feedback)
        elif self.nav_status in ["SUCCEEDED", "ABORTED", "CANCELED"]:
            status = self.nav_status
            mode = "OTONOM"
            # Reset back to IDLE after showing it once
            self.nav_status = "IDLE"

        online_val = "1" if self.is_online else "0"
        nav_msg = f"NAV,{status},{mode},{online_val}\n"

        with self.esp_ser_lock:
            if self.esp_connected and self.esp_ser is not None:
                try:
                    self.esp_ser.write(nav_msg.encode("utf-8"))
                except Exception as e:
                    self.get_logger().error(f"Failed to write navigation status to ESP32: {e}")

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

        # 2.1 Handle change to auto mode command
        elif path == "/isAutoMode":
            self.is_auto_mode = bool(data)
            self._update_fan_command()

        # 2.2 Handle map action (SAVE / LOAD) from dashboard
        elif path == "/map_action" and isinstance(data, dict):
            self._handle_map_action(data)

        # 3. Handle WiFi configuration trigger directly
        elif path == "/wifi/trigger":
            if data is True or str(data).lower() in ["true", "1"]:
                if self.command_ref is not None:
                    wifi_data = self.command_ref.child("wifi").get()
                    if isinstance(wifi_data, dict):
                        self._handle_wifi_change(wifi_data)

        # 3.1 Handle WiFi scan trigger directly
        elif path == "/wifi/scan_trigger":
            if data is True or str(data).lower() in ["true", "1"]:
                self.get_logger().info("Manual WiFi scan command received from Firebase!")
                threading.Thread(target=self._scan_wifi_worker, daemon=True).start()
                if self.command_ref is not None:
                    try:
                        self.command_ref.child("wifi").update({"scan_trigger": False})
                    except Exception as e:
                        self.get_logger().error(f"Failed to reset scan_trigger in Firebase: {e}")

        # 4. Handle WiFi configuration update as a dictionary
        elif path == "/wifi":
            if isinstance(data, dict):
                self._handle_wifi_change(data)

        # 5. Handle initial bulk load or specific updates containing goals
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

            if "isAutoMode" in data and data["isAutoMode"] is not None:
                self.is_auto_mode = bool(data["isAutoMode"])
                self._update_fan_command()

            if "map_action" in data and isinstance(data["map_action"], dict):
                self._handle_map_action(data["map_action"])

            if "save_map" in data and (data["save_map"] is True or str(data["save_map"]).lower() in ["true", "1"]):
                self.get_logger().info("Manual map save command received from Firebase (bulk)!")
                threading.Thread(target=self.save_map_worker, daemon=True).start()
                if self.command_ref is not None:
                    self.command_ref.child("save_map").set(False)

            if "wifi" in data and isinstance(data["wifi"], dict):
                self._handle_wifi_change(data["wifi"])

        # 6. Handle individual goal_x or goal_y updates if they are set separately (fallback)
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

    def _handle_wifi_change(self, wifi_data: dict):
        """Trigger connection to a new WiFi network in a separate thread"""
        trigger = wifi_data.get("trigger", False)
        ssid = wifi_data.get("ssid")
        password = wifi_data.get("password")
        scan_trigger = wifi_data.get("scan_trigger", False)

        if (scan_trigger is True or str(scan_trigger).lower() in ["true", "1"]):
            self.get_logger().info("WiFi scan trigger received in wifi change handler")
            threading.Thread(target=self._scan_wifi_worker, daemon=True).start()
            if self.command_ref is not None:
                try:
                    self.command_ref.child("wifi").update({"scan_trigger": False})
                except Exception as e:
                    self.get_logger().error(f"Failed to reset scan_trigger in Firebase: {e}")

        if (trigger is True or str(trigger).lower() in ["true", "1"]) and ssid:
            # Launch background worker so it doesn't block the listener thread
            threading.Thread(
                target=self.wifi_connect_worker,
                args=(ssid, password or ""),
                daemon=True
            ).start()

    def _wifi_scan_timer_callback(self):
        """Timer callback for periodic WiFi scanning"""
        threading.Thread(target=self._scan_wifi_worker, daemon=True).start()

    def _wifi_status_monitor_loop(self):
        """Periodically check internet connectivity in the background"""
        while rclpy.ok():
            try:
                self.is_online = (check_wifi_status() == "Connected")
            except Exception as e:
                self.get_logger().error(f"Error checking wifi status: {e}")
                self.is_online = False
            time.sleep(10.0)

    def _scan_wifi_worker(self):
        """Scan available WiFi networks and upload them to Firebase"""
        try:
            import subprocess
            cmd = ["nmcli", "-t", "-f", "SSID", "device", "wifi", "list"]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=15.0)
            if result.returncode == 0:
                ssids = []
                for line in result.stdout.splitlines():
                    ssid = line.strip()
                    # Filter out empty, duplicate, or placeholder SSIDs
                    if ssid and ssid != "--" and ssid not in ssids:
                        ssids.append(ssid)
                
                # Upload list of SSIDs to Firebase
                if self.firebase_ready and self.db_ref is not None:
                    self.db_ref.child("Status").update({
                        "detected_wifis": ssids
                    })
                    self.get_logger().info(f"✓ Scanned and uploaded {len(ssids)} WiFi networks: {ssids}")
            else:
                self.get_logger().warning(f"WiFi scan warning: {result.stderr}")
        except subprocess.TimeoutExpired:
            self.get_logger().error("WiFi scan timeout expired!")
        except Exception as e:
            self.get_logger().error(f"Error scanning WiFi: {e}")

    def wifi_connect_worker(self, ssid: str, password: str):
        self.get_logger().info(f"Attempting to connect to WiFi SSID: '{ssid}'...")
        if self.firebase_ready and self.db_ref is not None:
            try:
                self.db_ref.child("Status").update({
                    "wifi_status": "Connecting...",
                    "wifi_error": ""
                })
            except Exception as e:
                self.get_logger().error(f"Failed to set WiFi connecting status: {e}")

        try:
            import subprocess
            # nmcli device wifi connect "SSID" password "PASSWORD"
            cmd = ["nmcli", "device", "wifi", "connect", ssid]
            if password:
                cmd.extend(["password", password])

            self.get_logger().info(f"Running WiFi command: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=25.0)

            if result.returncode == 0:
                self.get_logger().info(f"✓ Successfully connected to WiFi: {ssid}")
                if self.firebase_ready and self.db_ref is not None:
                    self.db_ref.child("Status").update({
                        "wifi_status": f"Connected to {ssid}",
                        "wifi_error": ""
                    })
            else:
                error_msg = result.stderr.strip() or result.stdout.strip()
                self.get_logger().error(f"✗ Failed to connect to WiFi: {error_msg}")
                if self.firebase_ready and self.db_ref is not None:
                    self.db_ref.child("Status").update({
                        "wifi_status": "Failed to connect",
                        "wifi_error": error_msg[:100]
                    })
        except subprocess.TimeoutExpired:
            self.get_logger().error("✗ WiFi connection attempt timed out.")
            if self.firebase_ready and self.db_ref is not None:
                self.db_ref.child("Status").update({
                    "wifi_status": "Timeout",
                    "wifi_error": "Connection timed out (25s)"
                })
        except Exception as e:
            self.get_logger().error(f"✗ WiFi connection error: {e}")
            if self.firebase_ready and self.db_ref is not None:
                self.db_ref.child("Status").update({
                    "wifi_status": "Error",
                    "wifi_error": str(e)[:100]
                })
        finally:
            # Reset trigger in Command/wifi/trigger to False
            if self.command_ref is not None:
                try:
                    self.command_ref.child("wifi").update({"trigger": False})
                except Exception as e:
                    self.get_logger().error(f"Failed to reset WiFi trigger in Firebase: {e}")

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

    def map_callback(self, msg: OccupancyGrid):
        """Receive occupancy grid from SLAM Toolbox and store for periodic Firebase upload"""
        with self.map_data_lock:
            self.latest_map_data = msg

    def plan_callback(self, msg: Path):
        """Receive A* planned path from Nav2 and store for Firebase upload"""
        path_points = []
        # Downsample path to max 100 points to limit payload
        step = max(1, len(msg.poses) // 100)
        for i in range(0, len(msg.poses), step):
            pose = msg.poses[i]
            path_points.append({
                "x": round(pose.pose.position.x, 3),
                "y": round(pose.pose.position.y, 3),
            })
        # Always include the final point
        if msg.poses and (len(msg.poses) - 1) % step != 0:
            last = msg.poses[-1]
            path_points.append({
                "x": round(last.pose.position.x, 3),
                "y": round(last.pose.position.y, 3),
            })

        with self.map_data_lock:
            self.latest_plan_path = path_points

        # Immediately upload path to Firebase for responsive dashboard display
        if self.firebase_ready and self.db_ref is not None:
            try:
                self.db_ref.child("Map").child("path").set({
                    "points": path_points,
                    "timestamp": datetime.now().isoformat(),
                })
                self.get_logger().info(f"✓ Uploaded A* path ({len(path_points)} pts) to Firebase")
            except Exception as e:
                self.get_logger().error(f"Failed to upload path to Firebase: {e}")

    def _upload_map_data_callback(self):
        """Periodic timer (every 5s): Upload map grid, scan points, and robot pose to Firebase"""
        if not self.firebase_ready or self.db_ref is None:
            return

        threading.Thread(target=self._upload_map_data_worker, daemon=True).start()

    def _upload_map_data_worker(self):
        """Background worker to upload map data to Firebase without blocking ROS callbacks"""
        try:
            map_ref = self.db_ref.child("Map")
            now_iso = datetime.now().isoformat()

            # 1. Upload downsampled occupancy grid
            with self.map_data_lock:
                map_msg = self.latest_map_data

            if map_msg is not None:
                info = map_msg.info
                width = info.width
                height = info.height
                resolution = info.resolution
                origin_x = info.origin.position.x
                origin_y = info.origin.position.y

                # Downsample by factor of 4 for bandwidth efficiency
                ds_factor = 4
                ds_w = width // ds_factor
                ds_h = height // ds_factor

                grid_data = map_msg.data
                ds_cells = []
                for row in range(ds_h):
                    for col in range(ds_w):
                        # Sample center cell of each ds_factor x ds_factor block
                        src_row = row * ds_factor + ds_factor // 2
                        src_col = col * ds_factor + ds_factor // 2
                        if src_row < height and src_col < width:
                            val = grid_data[src_row * width + src_col]
                        else:
                            val = -1
                        ds_cells.append(val)

                # Encode as base64 int8 array for compact transfer
                byte_data = bytes([(v + 128) & 0xFF for v in ds_cells])  # shift -1..100 to 0..228
                grid_b64 = base64.b64encode(byte_data).decode('ascii')

                map_ref.child("grid").set({
                    "width": ds_w,
                    "height": ds_h,
                    "resolution": round(resolution * ds_factor, 4),
                    "origin_x": round(origin_x, 4),
                    "origin_y": round(origin_y, 4),
                    "data_b64": grid_b64,
                    "timestamp": now_iso,
                })

            # 2. Upload scan points
            with self.map_data_lock:
                scan_pts = list(self.latest_scan_points)

            if scan_pts:
                map_ref.child("scan_points").set({
                    "points": scan_pts,
                    "timestamp": now_iso,
                })

            # 3. Upload robot pose (from latest odom)
            if self.current_odom is not None:
                odom = self.current_odom
                x = odom.pose.pose.position.x
                y = odom.pose.pose.position.y
                q = odom.pose.pose.orientation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                map_ref.child("robot_pose").set({
                    "x": round(x, 3),
                    "y": round(y, 3),
                    "yaw": round(yaw, 4),
                    "timestamp": now_iso,
                })

        except Exception as e:
            self.get_logger().error(f"Failed to upload map data to Firebase: {e}")

    def scan_callback(self, msg: LaserScan):
        if not self.firebase_ready or self.db_ref is None:
            return
        self.frame_count += 1
        if self.frame_count % self.publish_interval != 0:
            return

        # Calculate min distance AND build scan point cloud for mapping
        min_distance = float("inf")
        scan_points = []

        # Get current robot pose for transforming scan points to map frame
        pose_x, pose_y, pose_yaw = 0.0, 0.0, 0.0
        if self.current_odom is not None:
            pose_x = self.current_odom.pose.pose.position.x
            pose_y = self.current_odom.pose.pose.position.y
            q = self.current_odom.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            pose_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Downsample scan to max ~72 points (every 5th ray)
        step = max(1, len(msg.ranges) // 72)
        for i in range(0, len(msg.ranges), step):
            distance = msg.ranges[i]
            if math.isfinite(distance) and msg.range_min < distance < msg.range_max:
                if distance < min_distance:
                    min_distance = distance

                # Transform scan point to map frame
                angle = msg.angle_min + i * msg.angle_increment
                local_x = distance * math.cos(angle)
                local_y = distance * math.sin(angle)
                map_x = pose_x + local_x * math.cos(pose_yaw) - local_y * math.sin(pose_yaw)
                map_y = pose_y + local_x * math.sin(pose_yaw) + local_y * math.cos(pose_yaw)
                scan_points.append({
                    "x": round(map_x, 3),
                    "y": round(map_y, 3),
                })

        # Store scan points for periodic map upload
        with self.map_data_lock:
            self.latest_scan_points = scan_points

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

    def _handle_map_action(self, action_data: dict):
        action = action_data.get("action")
        status = action_data.get("status")
        map_name = action_data.get("mapName", "")
        timestamp = action_data.get("timestamp", int(time.time() * 1000))

        if status == "PENDING":
            if action == "SAVE":
                self.get_logger().info(f"Firebase map action SAVE received for '{map_name}'")
                threading.Thread(
                    target=self._save_map_action_worker,
                    args=(map_name, timestamp),
                    daemon=True
                ).start()
            elif action == "LOAD":
                self.get_logger().info(f"Firebase map action LOAD received for '{map_name}'")
                threading.Thread(
                    target=self._load_map_action_worker,
                    args=(map_name,),
                    daemon=True
                ).start()

    def _save_map_action_worker(self, map_name: str, timestamp: int):
        try:
            import subprocess
            script_dir = os.path.dirname(os.path.abspath(__file__))
            maps_dir = os.path.join(script_dir, "maps")
            os.makedirs(maps_dir, exist_ok=True)

            map_id = f"map_{timestamp}"
            map_path = os.path.join(maps_dir, map_id)

            self.get_logger().info(f"Saving map locally to: {map_path}")

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
                self.get_logger().info(f"✓ Map saved successfully to {map_path}")
                
                # 3. Create grid base64 representation to upload to SavedMaps
                # Get the map data from memory
                with self.map_data_lock:
                    map_msg = self.latest_map_data

                grid_ref_data = None
                if map_msg is not None:
                    info = map_msg.info
                    width = info.width
                    height = info.height
                    resolution = info.resolution
                    origin_x = info.origin.position.x
                    origin_y = info.origin.position.y

                    # Downsample by factor of 4
                    ds_factor = 4
                    ds_w = width // ds_factor
                    ds_h = height // ds_factor

                    grid_data = map_msg.data
                    ds_cells = []
                    for row in range(ds_h):
                        for col in range(ds_w):
                            src_row = row * ds_factor + ds_factor // 2
                            src_col = col * ds_factor + ds_factor // 2
                            if src_row < height and src_col < width:
                                val = grid_data[src_row * width + src_col]
                            else:
                                val = -1
                            ds_cells.append(val)

                    byte_data = bytes([(v + 128) & 0xFF for v in ds_cells])
                    grid_b64 = base64.b64encode(byte_data).decode('ascii')

                    grid_ref_data = {
                        "width": ds_w,
                        "height": ds_h,
                        "resolution": round(resolution * ds_factor, 4),
                        "origin_x": round(origin_x, 4),
                        "origin_y": round(origin_y, 4),
                        "data_b64": grid_b64,
                    }

                # 4. Write to SavedMaps in Firebase
                if self.firebase_ready and self.db_ref is not None:
                    saved_map_entry = {
                        "id": map_id,
                        "name": map_name,
                        "timestamp": timestamp,
                    }
                    if grid_ref_data is not None:
                        saved_map_entry["grid"] = grid_ref_data

                    self.db_ref.child("SavedMaps").child(map_id).set(saved_map_entry)
                    self.db_ref.child("Command").child("map_action").update({
                        "status": "SUCCESS"
                    })
                    self.get_logger().info(f"✓ Uploaded saved map details to /SavedMaps/{map_id}")
            else:
                self.get_logger().error(f"Failed to save map: {res.stderr}")
                if self.firebase_ready and self.db_ref is not None:
                    self.db_ref.child("Command").child("map_action").update({
                        "status": "ERROR"
                    })
        except Exception as e:
            self.get_logger().error(f"Error during save map action: {e}")
            if self.firebase_ready and self.db_ref is not None:
                try:
                    self.db_ref.child("Command").child("map_action").update({
                        "status": "ERROR"
                    })
                except Exception:
                    pass

    def _load_map_action_worker(self, map_name: str):
        try:
            # 1. Query Firebase SavedMaps to find the map with this name
            if not self.firebase_ready or self.db_ref is None:
                self.get_logger().error("Firebase not ready for load map action.")
                return

            saved_maps = self.db_ref.child("SavedMaps").get()
            map_id = None
            if isinstance(saved_maps, dict):
                for key, val in saved_maps.items():
                    if isinstance(val, dict) and val.get("name") == map_name:
                        map_id = val.get("id") or key
                        break

            if not map_id:
                self.get_logger().error(f"Map with name '{map_name}' not found in SavedMaps.")
                self.db_ref.child("Command").child("map_action").update({
                    "status": "ERROR"
                })
                return

            script_dir = os.path.dirname(os.path.abspath(__file__))
            maps_dir = os.path.join(script_dir, "maps")
            map_path = os.path.join(maps_dir, map_id)

            # Check if file exists. SLAM toolbox looks for .posegraph file
            posegraph_file = f"{map_path}.posegraph"
            if not os.path.exists(posegraph_file):
                self.get_logger().error(f"Posegraph file '{posegraph_file}' not found on local disk.")
                self.db_ref.child("Command").child("map_action").update({
                    "status": "ERROR"
                })
                return

            # 2. Call deserialize_map service of slam_toolbox
            import subprocess
            cmd = [
                "ros2", "service", "call",
                "/slam_toolbox/deserialize_map",
                "slam_toolbox/srv/DeserializePoseGraph",
                f"{{filename: '{map_path}', match_type: 1, initial_pose: {{position: {{x: 0.0, y: 0.0, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}}"
            ]
            self.get_logger().info(f"Loading map via SLAM Toolbox: {' '.join(cmd)}")
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=15.0)

            if res.returncode == 0:
                self.get_logger().info(f"✓ Map '{map_name}' loaded successfully via SLAM Toolbox")
                self.db_ref.child("Command").child("map_action").update({
                    "status": "SUCCESS"
                })
            else:
                self.get_logger().error(f"Failed to load map: {res.stderr}")
                self.db_ref.child("Command").child("map_action").update({
                    "status": "ERROR"
                })
        except Exception as e:
            self.get_logger().error(f"Error during load map action: {e}")
            if self.firebase_ready and self.db_ref is not None:
                try:
                    self.db_ref.child("Command").child("map_action").update({
                        "status": "ERROR"
                    })
                except Exception:
                    pass


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
