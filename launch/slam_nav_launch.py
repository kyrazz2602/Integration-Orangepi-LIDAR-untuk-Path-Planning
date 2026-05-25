import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths to package shares
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    slam_toolbox_dir = FindPackageShare('slam_toolbox')
    
    # 1. RPLidar Node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0', # Adjust if needed
            'serial_baudrate': 115200,
            'frame_id': 'laser_link',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen'
    )
    
    # 2. Static TF for Laser Link
    # Tells ROS where the laser is relative to the base of the robot.
    # Adjust the X, Y, Z coordinates (in meters) to match your physical robot.
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'laser_link'],
        output='screen'
    )

    # 3. Arduino Serial Bridge (Custom Node we created)
    # Note: If this node is not built as a ROS package, you might need to run it manually.
    # Assuming it's in the workspace or we run it manually for now.
    arduino_bridge_node = Node(
        package='rplidar_ros', # Assuming we add it to the package later
        executable='arduino_serial_bridge.py',
        name='arduino_bridge',
        parameters=[{
            'port': '/dev/ttyUSB1', # Arduino Port
            'baudrate': 115200,
            'wheel_radius': 0.033,
            'wheel_base': 0.20
        }],
        output='screen'
    )

    # 4. SLAM Toolbox (Online Sync)
    # Builds the map using /scan and /odom
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_toolbox_dir, 'launch', 'online_sync_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'false'
        }.items()
    )

    # 5. Nav2 Stack
    # Provides A* Path Planning and local obstacle avoidance
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            # Add parameters file here if you have a custom nav2_params.yaml
            # 'params_file': PathJoinSubstitution([my_pkg_dir, 'config', 'nav2_params.yaml'])
        }.items()
    )

    return LaunchDescription([
        rplidar_node,
        static_tf_node,
        # arduino_bridge_node, # Uncomment when packaged correctly, or run script manually: python3 arduino_serial_bridge.py
        slam_toolbox_launch,
        nav2_launch
    ])
