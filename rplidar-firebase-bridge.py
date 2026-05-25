#!/usr/bin/env python3
"""
Robot to Firebase Bridge
Handles:
1. Publishing RPLidar scan data to Firebase
2. Listening to Firebase Commands for Manual Control and A* Path Planning
3. Publishing Robot Status back to Firebase
"""

import os
import sys
import math
import threading
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

try:
    import firebase_admin
    from firebase_admin import db
    from firebase_admin import credentials
except ImportError:
    print("ERROR: firebase-admin not installed. Run: pip install firebase-admin")
    sys.exit(1)

try:
    from dotenv import load_dotenv
    load_dotenv(os.path.expanduser('~/.env.firebase'))
except ImportError:
    pass

class RobotFirebaseBridge(Node):
    def __init__(self):
        super().__init__('robot_firebase_bridge')

        self.declare_parameter('firebase_db_url', 'https://airguard-b7ef4-default-rtdb.asia-southeast1.firebasedatabase.app')
        self.firebase_db_url = self.get_parameter('firebase_db_url').value

        # Initialize Firebase
        self.db_ref = None
        self.firebase_ready = self._init_firebase()

        if not self.firebase_ready:
            self.get_logger().error('Firebase initialization failed!')
            return

        # ROS 2 Publishers & Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Nav2 Action Client for A* Path Planning
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Firebase Listener
        self.command_ref = self.db_ref.child('Command')
        
        # Start Firebase Listener in a separate thread so it doesn't block ROS spin
        self.listener_thread = threading.Thread(target=self._start_firebase_listener, daemon=True)
        self.listener_thread.start()

        # State Variables
        self.frame_count = 0
        self.publish_interval = 20 # Publish LiDAR every 20 frames
        self.current_odom = None

        self.get_logger().info('=' * 60)
        self.get_logger().info('Robot Firebase Bridge Started')
        self.get_logger().info('Listening for Commands and Publishing Status/LiDAR')
        self.get_logger().info('=' * 60)

    def _init_firebase(self) -> bool:
        try:
            try:
                self.db_ref = db.reference()
                return True
            except ValueError:
                pass

            # Gunakan file JSON yang sudah ada di direktori yang sama dengan script
            script_dir = os.path.dirname(os.path.abspath(__file__))
            cred_path = os.path.join(script_dir, 'airguard-b7ef4-firebase-adminsdk-fbsvc-6361f49d51.json')
            
            if not Path(cred_path).exists():
                self.get_logger().error(f'Firebase credentials not found at: {cred_path}')
                return False

            db_url = os.getenv('FIREBASE_DB_URL') or self.firebase_db_url
            cred = credentials.Certificate(cred_path)
            firebase_admin.initialize_app(cred, {'databaseURL': db_url})

            self.db_ref = db.reference()
            self.get_logger().info('✓ Firebase connected successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Firebase init error: {e}')
            return False

    def _start_firebase_listener(self):
        """Listen to Firebase Realtime Database changes on the 'Command' node"""
        try:
            # Note: The firebase_admin SDK in Python doesn't support real-time .listen() as cleanly as JS.
            # Using standard reference listener.
            self.command_ref.listen(self._firebase_command_callback)
        except Exception as e:
            self.get_logger().error(f'Failed to start Firebase listener: {e}')

    def _firebase_command_callback(self, event):
        """Triggered when data in /Command changes"""
        self.get_logger().info(f'Received Firebase Command Update: {event.path} -> {event.data}')
        
        # If the entire Command object is updated or a specific field
        # For simplicity, we fetch the whole Command object
        command_data = self.command_ref.get()
        if not command_data:
            return

        # Handle Manual Movement (cmd_vel)
        if 'gerak' in command_data:
            gerak = command_data['gerak']
            twist = Twist()
            speed = 0.2 # m/s
            angular_speed = 0.5 # rad/s
            
            if gerak == 'MAJU':
                twist.linear.x = speed
            elif gerak == 'MUNDUR':
                twist.linear.x = -speed
            elif gerak == 'KIRI' or gerak == 'PUTAR_KIRI':
                twist.angular.z = angular_speed
            elif gerak == 'KANAN' or gerak == 'PUTAR_KANAN':
                twist.angular.z = -angular_speed
            elif gerak == 'DIAM':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f'Published Manual Twist: {gerak}')

        # Handle Autonomous Navigation (A* Path Planning)
        if 'goal_x' in command_data and 'goal_y' in command_data:
            goal_x = float(command_data['goal_x'])
            goal_y = float(command_data['goal_y'])
            self.send_nav_goal(goal_x, goal_y)
            
            # Clear goal from DB after reading so it doesn't loop
            self.command_ref.child('goal_x').delete()
            self.command_ref.child('goal_y').delete()

    def send_nav_goal(self, x, y):
        """Send goal to Nav2 Action Server"""
        self.get_logger().info(f'Sending Nav2 Goal: x={x}, y={y}')
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action Server not available!')
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0 # Facing forward
        
        self.nav_to_pose_client.send_goal_async(goal_msg)

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg
        # Publish status to Firebase every few seconds (optional: throttle this)
        # self.db_ref.child('Status').update({'rpmKanan': ..., 'rpmKiri': ...})

    def scan_callback(self, msg: LaserScan):
        if not self.firebase_ready: return
        self.frame_count += 1
        if self.frame_count % self.publish_interval != 0: return

        # Simplistic LiDAR processing to prevent huge payload
        min_distance = float('inf')
        for distance in msg.ranges:
            if math.isfinite(distance) and 0.01 < distance < 12.0:
                if distance < min_distance:
                    min_distance = distance
                    
        data = {
            'timestamp': datetime.now().isoformat(),
            'jarak_terdekat_cm': round(min_distance * 100, 2) if min_distance != float('inf') else 0
        }
        
        try:
            self.db_ref.child('LiDAR').child('latest').set(data)
        except Exception as e:
            self.get_logger().error(f'Firebase publish error: {e}')

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

if __name__ == '__main__':
    main()
