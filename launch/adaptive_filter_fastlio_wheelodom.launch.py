from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    imu_arg   = DeclareLaunchArgument('imu',             default_value='/imu/data')
    lidar_arg = DeclareLaunchArgument('lidar_odom',      default_value='/Odometry')
    wheel_arg = DeclareLaunchArgument('wheel_odom',      default_value='/odom')
    out_arg   = DeclareLaunchArgument('filter_odom_out', default_value='/filter_odom')

    params_yaml = PathJoinSubstitution([
        FindPackageShare('adaptive_odom_filter'),
        'config', 'adaptive_filter_parameters.yaml'
    ])

    node = Node(
        package='adaptive_odom_filter',
        executable='adaptive_odom_filter_node',   # or EKFAdaptiveFilter if that’s your binary
        name='adaptive_odom_filter',
        output='screen',
        parameters=[
            params_yaml,
            {
                'enableVisual': False,   # important: don’t wait for cameras
                'enableImu': True,
                'enableWheel': True,
                'enableLidar': True,
                'use_sim_time': False,
                'freq': 50.0,
            }
        ],
        remappings=[
            ('/imu/data',    LaunchConfiguration('imu')),
            ('/lidar_odom',  LaunchConfiguration('lidar_odom')),
            ('/odom',        LaunchConfiguration('wheel_odom')),
            ('/filter_odom', LaunchConfiguration('filter_odom_out')),
        ],
        arguments=['--ros-args', '--log-level', 'adaptive_odom_filter:=debug'],
    )

    return LaunchDescription([imu_arg, lidar_arg, wheel_arg, out_arg, node])
