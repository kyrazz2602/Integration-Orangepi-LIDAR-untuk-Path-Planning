import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths to package shares
    my_pkg_dir = FindPackageShare('rplidar_ros')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    
    # Parameters File Path
    nav2_params_file = PathJoinSubstitution([my_pkg_dir, 'config', 'nav2_params.yaml'])
    
    # Declare Launch Arguments
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/rplidar' if os.path.exists('/dev/rplidar') else '/dev/ttyUSB0',
        description='Serial port for RPLidar (e.g. /dev/ttyUSB0 or /dev/ttyS1)'
    )
    
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/arduino' if os.path.exists('/dev/arduino') else '/dev/ttyAS4',
        description='Serial port for Arduino Mega (e.g. /dev/ttyUSB1 or /dev/ttyAS4)'
    )
    
    # 1. RPLidar Node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,
            'frame_id': 'laser_link',
            'inverted': False,
            'angle_compensate': True,
        }],
        respawn=True,
        output='screen'
    )
    
    # 2. Static TF for Laser Link
    # Transforms base_link -> laser_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'laser_link'],
        output='screen'
    )

    # 3. Arduino Serial Bridge (Hardware Odometry/Motor Control)
    arduino_bridge_node = Node(
        package='rplidar_ros',
        executable='arduino_serial_bridge.py',
        name='arduino_bridge',
        parameters=[{
            'port': LaunchConfiguration('arduino_port'),
            'baudrate': 115200,
            'wheel_radius': 0.033,
            'wheel_base': 0.20
        }],
        respawn=True,
        output='screen'
    )

    # 4. SLAM: Slam Toolbox (Async Mapping)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'config',
                'mapper_params_online_async.yaml'
            ]),
            {
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom'
            }
        ]
    )

    # 5. Nav2 Stack (AMCL Localization, Global Planner A*, Local Planner DWA)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file
        }.items()
    )

    # Wrap SLAM Toolbox with an 8-second delay
    delayed_slam_toolbox = TimerAction(
        period=8.0,
        actions=[slam_toolbox_node]
    )

    # Wrap Nav2 Stack with a 15-second delay
    delayed_nav2 = TimerAction(
        period=15.0,
        actions=[nav2_launch]
    )

    return LaunchDescription([
        lidar_port_arg,
        arduino_port_arg,
        rplidar_node,
        static_tf_node,
        arduino_bridge_node, # Uncomment if arduino is connected
        delayed_slam_toolbox,
        delayed_nav2
    ])

