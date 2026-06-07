import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
        default_value='/dev/arduino' if os.path.exists('/dev/arduino') else '/dev/ttyS4',
        description='Serial port for Arduino Mega (e.g. /dev/ttyUSB1 or /dev/ttyS4)'
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
        output='screen'
    )

    # 4. SLAM: GMapping
    # We use slam_gmapping package to create the Occupancy Grid Map.
    # Make sure ros-<distro>-slam-gmapping is installed or built in the workspace.
    slam_gmapping_node = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        parameters=[{
            'use_sim_time': False,
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'map_update_interval': 5.0,
            'maxUrange': 8.0,
            'sigma': 0.05,
            'kernelSize': 1,
            'lstep': 0.05,
            'astep': 0.05,
            'iterations': 5,
            'lsigma': 0.075,
            'ogain': 3.0,
            'lskip': 0,
            'minimumScore': 50.0,
            'srr': 0.1,
            'srt': 0.2,
            'str': 0.1,
            'stt': 0.2,
            'linearUpdate': 1.0,
            'angularUpdate': 0.5,
            'temporalUpdate': 3.0,
            'resampleThreshold': 0.5,
            'particles': 30,
            'xmin': -10.0,
            'ymin': -10.0,
            'xmax': 10.0,
            'ymax': 10.0,
            'delta': 0.05,
            'llsamplerange': 0.01,
            'llsamplestep': 0.01,
            'lasamplerange': 0.005,
            'lasamplestep': 0.005
        }],
        output='screen'
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

    return LaunchDescription([
        lidar_port_arg,
        arduino_port_arg,
        rplidar_node,
        static_tf_node,
        arduino_bridge_node, # Uncomment if arduino is connected
        slam_gmapping_node,
        nav2_launch
    ])
