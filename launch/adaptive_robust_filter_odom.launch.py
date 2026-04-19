from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_yaml = PathJoinSubstitution(
        [FindPackageShare('adaptive_odom_filter'), 'config', 'adaptive_robust_filter_parameters.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('filter', default_value='true'),
        DeclareLaunchArgument('imu', default_value='/imu/data'),
        DeclareLaunchArgument('lidar_slam_odom', default_value='/ekf_loam/laser_odom_with_cov'),
        DeclareLaunchArgument('lidar_features', default_value='/ekf_loam/features_cloud_info'),
        DeclareLaunchArgument('wheel_odom', default_value='/odom_with_cov'),
        DeclareLaunchArgument('filter_odom', default_value='/ekf_loam/filter_odom_to_init'),
        Node(
            condition=IfCondition(LaunchConfiguration('filter')),
            package='adaptive_odom_filter',
            executable='EKFRobustAdaptiveFilter',
            name='adaptive_robust_odom_filter',
            output='screen',
            parameters=[params_yaml],
            remappings=[
                ('/imu/data', LaunchConfiguration('imu')),
                ('/lidar_odom', LaunchConfiguration('lidar_slam_odom')),
                ('/features_cloud_info', LaunchConfiguration('lidar_features')),
                ('/odom', LaunchConfiguration('wheel_odom')),
                ('/filter_odom', LaunchConfiguration('filter_odom')),
            ],
        ),
    ])
