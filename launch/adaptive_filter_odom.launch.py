from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_yaml = PathJoinSubstitution(
        [FindPackageShare('adaptive_odom_filter'), 'config', 'adaptive_filter_parameters.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('filter', default_value='true'),
        DeclareLaunchArgument('test', default_value='false'),
        DeclareLaunchArgument('imu', default_value='/imu/data'),
        DeclareLaunchArgument('lidar_slam_odom', default_value='/ekf_loam/laser_odom_with_cov'),
        DeclareLaunchArgument('tracking_camera_odom', default_value='/camera/odom'),
        DeclareLaunchArgument('depth_camera_odom', default_value='/camera/odom'),
        DeclareLaunchArgument('wheel_odom', default_value='/odom_with_cov'),
        DeclareLaunchArgument('stereo_cam_left', default_value='/t265/fisheye2/image_raw'),
        DeclareLaunchArgument('stereo_cam_right', default_value='/t265/fisheye1/image_raw'),
        DeclareLaunchArgument('color_image', default_value='/d435i/color/image_raw'),
        DeclareLaunchArgument('filter_odom', default_value='/ekf_loam/filter_odom_to_init'),
        DeclareLaunchArgument('rtabmap_service', default_value='/rtabmap/reset_odom_to_pose'),
        Node(
            condition=IfCondition(LaunchConfiguration('test')),
            package='adaptive_odom_filter',
            executable='fake_odometry_path.py',
            name='fake_odometry_path',
            output='screen',
            parameters=[{
                'camera_rate_hz': 5.0,
                'wheel_rate_hz': 10.0,
                'lidar_rate_hz': 5.0,
                'imu_rate_hz': 10.0,
            }],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('filter')),
            package='adaptive_odom_filter',
            executable='adaptive_odom_filter_node',
            name='adaptive_odom_filter',
            output='screen',
            parameters=[params_yaml],
            remappings=[
                ('/imu/data', LaunchConfiguration('imu')),
                ('/lidar_odom', LaunchConfiguration('lidar_slam_odom')),
                ('/tracking_odom', LaunchConfiguration('tracking_camera_odom')),
                ('/depth_odom', LaunchConfiguration('depth_camera_odom')),
                ('/odom', LaunchConfiguration('wheel_odom')),
                ('/left_camera', LaunchConfiguration('stereo_cam_left')),
                ('/rigth_camera', LaunchConfiguration('stereo_cam_right')),
                ('/camera_color', LaunchConfiguration('color_image')),
                ('/filter_odom', LaunchConfiguration('filter_odom')),
                ('/rtabmap/reset_odom_to_pose', LaunchConfiguration('rtabmap_service')),
            ],
        ),
    ])
