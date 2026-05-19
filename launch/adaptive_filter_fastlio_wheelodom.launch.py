from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    imu_arg   = DeclareLaunchArgument('imu',             default_value='/imu/data')
    lidar_arg = DeclareLaunchArgument('lidar_odom',      default_value='/Odometry')
    wheel_arg = DeclareLaunchArgument('wheel_odom',      default_value='/odom')
    out_arg   = DeclareLaunchArgument('filter_odom_out', default_value='/filter_odom')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_imu_arg = DeclareLaunchArgument('use_imu', default_value='false')
    use_wheel_arg = DeclareLaunchArgument('use_wheel', default_value='true')

    params_yaml = PathJoinSubstitution([
        FindPackageShare('adaptive_odom_filter'),
        'config', 'adaptive_filter_parameters.yaml'
    ])

    node = Node(
        package='adaptive_odom_filter',
        executable='adaptive_odom_filter_node',
        name='adaptive_odom_filter',
        output='screen',
        parameters=[
            params_yaml,
            {
                'enableVisual': False,   # important: don’t wait for cameras
                'enableImu': ParameterValue(LaunchConfiguration('use_imu'), value_type=bool),
                'enableWheel': ParameterValue(LaunchConfiguration('use_wheel'), value_type=bool),
                'enableLidar': True,
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'freq': 20.0,
                'imu_topic': LaunchConfiguration('imu'),
                'lidar_odom_topic': LaunchConfiguration('lidar_odom'),
                'wheel_odom_topic': LaunchConfiguration('wheel_odom'),
                'filter_odom_topic': LaunchConfiguration('filter_odom_out'),
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

    return LaunchDescription([
        imu_arg,
        lidar_arg,
        wheel_arg,
        out_arg,
        use_sim_time_arg,
        use_imu_arg,
        use_wheel_arg,
        node,
    ])
