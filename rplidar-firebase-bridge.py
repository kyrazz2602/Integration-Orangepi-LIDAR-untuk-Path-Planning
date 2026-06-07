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
from action_msgs.msg import GoalStatus

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
        self.last_gerak = None
        self.last_odom_publish_time = self.get_clock().now()
        self.odom_publish_interval = 1.0 # seconds

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

            # Cari file JSON kredensial di beberapa lokasi
            cred_filename = 'airguard-b7ef4-firebase-adminsdk-fbsvc-6361f49d51.json'
            script_dir = os.path.dirname(os.path.abspath(__file__))
            
            paths_to_check = [
                os.path.join(script_dir, cred_filename), # Lokasi di install space / script dir
                os.path.join(os.getcwd(), cred_filename), # Lokasi di root workspace saat ini
                os.path.join(os.path.expanduser('~'), cred_filename), # Lokasi di home directory
            ]
            
            cred_path = None
            for path in paths_to_check:
                if Path(path).exists():
                    cred_path = path
                    break
                    
            if not cred_path:
                self.get_logger().error(f'Firebase credentials ({cred_filename}) not found in checked paths: {paths_to_check}')
                return False

            self.get_logger().info(f'Using Firebase credentials from: {cred_path}')
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
            if gerak != self.last_gerak:
                self.last_gerak = gerak
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
        else:
            self.last_gerak = None

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
            if self.firebase_ready:
                self.db_ref.child('Status').update({'navigation_status': 'SERVER_UNAVAILABLE'})
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0 # Facing forward
        
        if self.firebase_ready:
            self.db_ref.child('Status').update({'navigation_status': 'SENDING_GOAL'})
            
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            if self.firebase_ready:
                self.db_ref.child('Status').update({'navigation_status': 'REJECTED'})
            return

        self.get_logger().info('Goal accepted :)')
        if self.firebase_ready:
            self.db_ref.child('Status').update({'navigation_status': 'NAVIGATING'})
            
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            if self.firebase_ready:
                self.db_ref.child('Status').update({'navigation_status': 'SUCCEEDED'})
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info('Goal aborted!')
            if self.firebase_ready:
                self.db_ref.child('Status').update({'navigation_status': 'ABORTED'})
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal canceled!')
            if self.firebase_ready:
                self.db_ref.child('Status').update({'navigation_status': 'CANCELED'})
        else:
            self.get_logger().info(f'Goal finished with status code: {status}')
            if self.firebase_ready:
                self.db_ref.child('Status').update({'navigation_status': f'FINISHED_CODE_{status}'})

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
            
            if self.firebase_ready:
                try:
                    self.db_ref.child('Status').update({
                        'x': round(x, 2),
                        'y': round(y, 2),
                        'yaw': round(yaw, 2),
                        'linear_velocity': round(msg.twist.twist.linear.x, 2),
                        'angular_velocity': round(msg.twist.twist.angular.z, 2)
                    })
                except Exception as e:
                    self.get_logger().error(f'Firebase odom publish error: {e}')

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
