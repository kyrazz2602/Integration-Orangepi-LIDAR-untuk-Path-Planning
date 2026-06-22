#!/usr/bin/env python3
"""
ROS 2 Arduino Serial Bridge
Handles:
1. Receiving /cmd_vel (Twist) and sending per-wheel RPM commands to Arduino.
2. Reading Odometry (signed RPM) from Arduino and publishing /odom and TF.

Protocol:
  Bridge → Arduino: CMD,VEL,{rpmKiri:.1f},{rpmKanan:.1f}\n
  Arduino → Bridge: ODOM,{odomX},{odomY},{odomTheta},{signedRpmKanan},{signedRpmKiri}\n
"""

import math
import rclpy
from rclpy.node import Node
import serial
import threading
import subprocess
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Parameters
        import os
        default_port = '/dev/arduino' if os.path.exists('/dev/arduino') else '/dev/ttyAS4'
        self.declare_parameter('port', default_port)
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.033) # meters (radius roda)
        self.declare_parameter('wheel_base', 0.07)    # meters (jarak antar roda) — HARUS sama dengan Arduino
        self.declare_parameter('reconnect_delay', 5.0)
        self.declare_parameter('max_rpm', 80.0)       # batas RPM maksimum
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value
        self.reconnect_delay = self.get_parameter('reconnect_delay').value
        self.max_rpm = self.get_parameter('max_rpm').value
        
        # State variables for odometry
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.first_odom = True
        self.last_cmd_vel = Twist()  # Simpan cmd_vel terakhir untuk re-send
        
        # Setup Serial Connection & Lock
        self.serial_lock = threading.Lock()
        self.ser = None
        self.connected = False
        
        # Attempt initial connection
        self.connect_serial()

        # Publishers & Subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Timer: re-send CMD,VEL at 10Hz untuk menjaga watchdog Arduino
        self.vel_timer = self.create_timer(0.1, self.vel_timer_callback)
        
        # Start read thread
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to per-wheel RPM and send to Arduino immediately"""
        self.last_cmd_vel = msg
        self._send_vel_command(msg)

    def vel_timer_callback(self):
        """Re-send last velocity command at 10Hz to keep Arduino watchdog alive"""
        self._send_vel_command(self.last_cmd_vel)

    def _send_vel_command(self, msg: Twist):
        """Convert Twist to per-wheel RPM using differential drive kinematics
        and send CMD,VEL,rpmKiri,rpmKanan to Arduino"""
        v = msg.linear.x   # m/s
        w = msg.angular.z   # rad/s
        
        # Differential drive inverse kinematics
        v_left  = v - (w * self.L) / 2.0
        v_right = v + (w * self.L) / 2.0
        
        # Convert m/s to RPM: RPM = (v / (2π * R)) * 60
        rpm_left  = (v_left  / (2.0 * math.pi * self.R)) * 60.0
        rpm_right = (v_right / (2.0 * math.pi * self.R)) * 60.0
        
        # Clamp to max RPM for safety
        rpm_left  = max(-self.max_rpm, min(self.max_rpm, rpm_left))
        rpm_right = max(-self.max_rpm, min(self.max_rpm, rpm_right))
        
        command = f"CMD,VEL,{rpm_left:.1f},{rpm_right:.1f}\n"
        
        with self.serial_lock:
            if self.connected and self.ser is not None:
                try:
                    self.ser.write(command.encode('utf-8'))
                except Exception as e:
                    self.get_logger().error(f"Failed to write to serial: {e}")
                    self.connected = False
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None

    def connect_serial(self):
        """Try to establish connection with Arduino serial port"""
        with self.serial_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            self.connected = False
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
                self.connected = True
                self.get_logger().info(f"Connected to Arduino on {self.port} at {self.baudrate} baud.")
                return True
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to connect to serial port {self.port}: {e}")
                return False

    def serial_read_loop(self):
        """Read continuous ODOM data from Arduino with auto-reconnection"""
        import time
        error_count = 0
        MAX_ERRORS = 10
        while rclpy.ok():
            if not self.connected:
                self.get_logger().info("Attempting to reconnect to Arduino serial...")
                if self.connect_serial():
                    error_count = 0
                    time.sleep(0.5)
                else:
                    time.sleep(self.reconnect_delay)
                continue
                
            try:
                # Thread-safe read operation
                with self.serial_lock:
                    if self.ser is None:
                        self.connected = False
                        continue
                    line_bytes = self.ser.readline()
                
                if not line_bytes:
                    time.sleep(0.01)
                    continue
                line = line_bytes.decode('utf-8', errors='ignore').strip()
                
                # Format: ODOM,odomX,odomY,odomTheta,signedRpmKanan,signedRpmKiri
                if line.startswith("ODOM,"):
                    parts = line.split(',')
                    if len(parts) == 6:
                        # Signed RPM langsung dari Arduino — tidak perlu rekonstruksi tanda!
                        rpm_right = float(parts[4])
                        rpm_left  = float(parts[5])
                        self.process_odometry(rpm_left, rpm_right)
                        error_count = 0
                    elif len(parts) == 3:
                        # Fallback untuk format lama: ODOM,rpmKiri,rpmKanan
                        rpm_left  = float(parts[1])
                        rpm_right = float(parts[2])
                        self.process_odometry(rpm_left, rpm_right)
                        error_count = 0
                elif line.startswith("WIFI,") or line.startswith("wifi,"):
                    parts = line.split(',', 2)
                    if len(parts) == 3:
                        ssid = parts[1]
                        password = parts[2]
                        self.get_logger().info(f"Received WiFi connection request from ESP32: SSID={ssid}")
                        self.change_wifi(ssid, password)
            except Exception as e:
                error_count += 1
                self.get_logger().warn(f"Error reading from serial (attempt {error_count}/{MAX_ERRORS}): {e}")
                if error_count >= MAX_ERRORS:
                    self.get_logger().error(f"Max serial errors reached ({MAX_ERRORS}). Disconnecting and reconnecting...")
                    with self.serial_lock:
                        self.connected = False
                        if self.ser is not None:
                            try:
                                self.ser.close()
                            except Exception:
                                pass
                            self.ser = None
                time.sleep(1.0)

    def process_odometry(self, rpm_left, rpm_right):
        """Calculate x, y, theta from SIGNED wheel RPM and publish odom + TF"""
        current_time = self.get_clock().now()
        
        if self.first_odom:
            self.first_odom = False
            self.last_time = current_time
            return
            
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt > 1.0:
            self.get_logger().warning(f"Odom integration gap too large: {dt:.2f}s. Resetting timer.")
            return
        
        # Convert signed RPM to m/s
        v_left  = (rpm_left  / 60.0) * 2.0 * math.pi * self.R
        v_right = (rpm_right / 60.0) * 2.0 * math.pi * self.R
        
        # Differential drive forward kinematics
        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / self.L
        
        # Integrate to find position
        delta_x  = (v * math.cos(self.th)) * dt
        delta_y  = (v * math.sin(self.th)) * dt
        delta_th = w * dt
        
        self.x  += delta_x
        self.y  += delta_y
        self.th += delta_th
        
        # Quaternion from yaw
        q = self.euler_to_quaternion(0, 0, self.th)
        
        # 1. Publish TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
        # 2. Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        
        self.odom_pub.publish(odom)

    def change_wifi(self, ssid, password):
        """Automatically switch WiFi connection using nmcli in a separate thread"""
        def run_nmcli():
            self.get_logger().info(f"Connecting to WiFi SSID: '{ssid}'...")
            cmd = ["nmcli", "device", "wifi", "connect", ssid, "password", password]
            try:
                res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=30.0)
                if res.returncode == 0:
                    self.get_logger().info(f"Successfully connected to WiFi SSID: '{ssid}'")
                else:
                    self.get_logger().error(f"Failed to connect to WiFi SSID: '{ssid}'. Error: {res.stderr.strip()}")
            except subprocess.TimeoutExpired:
                self.get_logger().error(f"Timeout expired while connecting to WiFi SSID: '{ssid}'")
            except Exception as e:
                self.get_logger().error(f"Error executing nmcli command: {e}")

        # Run in background daemon thread to avoid blocking ROS execution or serial reader
        threading.Thread(target=run_nmcli, daemon=True).start()

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    bridge = ArduinoBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
