#!/usr/bin/env python3
"""
ROS 2 Arduino Serial Bridge
Handles:
1. Receiving /cmd_vel (Twist) and sending RPM commands to Arduino.
2. Reading Odometry (RPM) from Arduino and publishing /odom and TF.
"""

import math
import rclpy
from rclpy.node import Node
import serial
import threading
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.033) # meters (adjust according to your robot)
        self.declare_parameter('wheel_base', 0.20)    # meters (distance between wheels)
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value
        
        # State variables for odometry
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.first_odom = True
        
        # Setup Serial
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            raise SystemExit

        # Publishers & Subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Start read thread
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

    def cmd_vel_callback(self, msg: Twist):
        """Convert linear and angular velocities into left and right wheel RPM"""
        v = msg.linear.x
        w = msg.angular.z
        
        # Kinematics for Differential Drive
        v_right = v + (w * self.L / 2.0)
        v_left = v - (w * self.L / 2.0)
        
        # Convert m/s to RPM
        # v = (rpm / 60) * 2 * pi * R  => rpm = (v * 60) / (2 * pi * R)
        rpm_right = (v_right * 60.0) / (2.0 * math.pi * self.R)
        rpm_left = (v_left * 60.0) / (2.0 * math.pi * self.R)
        
        # Send to Arduino (Format: CMD,VEL,rpmKiri,rpmKanan)
        command = f"CMD,VEL,{rpm_left:.2f},{rpm_right:.2f}\n"
        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Failed to write to serial: {e}")

    def serial_read_loop(self):
        """Read continuous ODOM data from Arduino"""
        import time
        while rclpy.ok():
            try:
                line_bytes = self.ser.readline()
                if not line_bytes:
                    time.sleep(0.01)
                    continue
                line = line_bytes.decode('utf-8', errors='ignore').strip()
                
                # Format: ODOM,rpmKiri,rpmKanan
                if line.startswith("ODOM,"):
                    parts = line.split(',')
                    if len(parts) == 3:
                        rpm_left = float(parts[1])
                        rpm_right = float(parts[2])
                        self.process_odometry(rpm_left, rpm_right)
            except Exception as e:
                time.sleep(0.1)

    def process_odometry(self, rpm_left, rpm_right):
        """Calculate x, y, theta based on wheel RPM and publish"""
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
        
        # Convert RPM to m/s
        v_left = (rpm_left / 60.0) * 2.0 * math.pi * self.R
        v_right = (rpm_right / 60.0) * 2.0 * math.pi * self.R
        
        # Robot velocities
        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / self.L
        
        # Integrate to find position
        delta_x = (v * math.cos(self.th)) * dt
        delta_y = (v * math.sin(self.th)) * dt
        delta_th = w * dt
        
        self.x += delta_x
        self.y += delta_y
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
