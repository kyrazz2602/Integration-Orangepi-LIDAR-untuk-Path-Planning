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
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from tf2_ros import TransformBroadcaster

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Parameters
        import os
        default_port = '/dev/arduino' if os.path.exists('/dev/arduino') else '/dev/ttyAS4'
        self.declare_parameter('port', default_port)
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.0325) # meters (radius roda)
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
        
        # Range Sensor Publishers
        self.range_pubs = {
            'depan': self.create_publisher(Range, '/ultrasonic/depan', 10),
            'kiri':  self.create_publisher(Range, '/ultrasonic/kiri', 10),
            'kanan': self.create_publisher(Range, '/ultrasonic/kanan', 10),
        }
        
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
        """Convert Twist to discrete movement commands for the new Arduino firmware"""
        v = msg.linear.x    # m/s
        w = msg.angular.z    # rad/s
        
        # Thresholds
        v_thresh = 0.03
        w_thresh = 0.08
        
        if v > v_thresh:
            cmd_str = "CMD,MAJU\n"
        elif v < -v_thresh:
            cmd_str = "CMD,MUNDUR\n"
        elif w > w_thresh:
            cmd_str = "CMD,KIRI\n"
        elif w < -w_thresh:
            cmd_str = "CMD,KANAN\n"
        else:
            cmd_str = "CMD,DIAM\n"
            
        with self.serial_lock:
            if self.connected and self.ser is not None:
                try:
                    self.ser.write(cmd_str.encode('utf-8'))
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
                
                # Format: ODOM,odomX,odomY,odomTheta,rpmKanan,rpmKiri
                if line.startswith("ODOM,"):
                    parts = line.split(',')
                    if len(parts) == 6:
                        try:
                            odom_x = float(parts[1])
                            odom_y = float(parts[2])
                            odom_theta = float(parts[3])
                            rpm_right = float(parts[4])
                            rpm_left  = float(parts[5])
                            self.process_odometry(odom_x, odom_y, odom_theta, rpm_left, rpm_right)
                            error_count = 0
                        except ValueError as e:
                            self.get_logger().error(f"Error parsing ODOM line: {e} from: {line}")
                elif line.startswith("SENSOR,"):
                    parts = line.split(',')
                    if len(parts) >= 4:
                        try:
                            # SENSOR,jarakDepan,jarakKiri,jarakKanan,irKiri,irTengah,irKanan
                            jarak_depan = float(parts[1])
                            jarak_kiri  = float(parts[2])
                            jarak_kanan = float(parts[3])
                            self.publish_range('depan', jarak_depan)
                            self.publish_range('kiri', jarak_kiri)
                            self.publish_range('kanan', jarak_kanan)
                            error_count = 0
                        except ValueError as e:
                            self.get_logger().error(f"Error parsing SENSOR line: {e} from: {line}")
                elif line.startswith("EVT:OBSTACLE"):
                    self.get_logger().warn(f"Hardware safety stop triggered: {line}")
                    error_count = 0
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
                time.sleep(0.05)

    def process_odometry(self, odom_x_cm, odom_y_cm, odom_theta, rpm_left, rpm_right):
        """Publish odom + TF directly from coordinates calculated by Arduino Mega"""
        current_time = self.get_clock().now()
        
        # Convert cm to meters
        self.x = odom_x_cm / 100.0
        self.y = odom_y_cm / 100.0
        self.th = odom_theta
        
        # Calculate linear and angular velocities from wheel RPM for status reporting
        v_left  = (rpm_left  / 60.0) * 2.0 * math.pi * self.R
        v_right = (rpm_right / 60.0) * 2.0 * math.pi * self.R
        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / self.L
        

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

    def publish_range(self, name: str, distance_cm: float):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'ultrasonic_{name}_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 degrees
        msg.min_range = 0.02
        msg.max_range = 4.0
        
        # 999.0 indicates out of range or no reading
        val = distance_cm / 100.0
        if val > 4.0 or val < 0.0:
            msg.range = float('inf')
        else:
            msg.range = val
            
        self.range_pubs[name].publish(msg)

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
