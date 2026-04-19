#ifndef UTILITY_LIDAR_ODOMETRY_HPP_
#define UTILITY_LIDAR_ODOMETRY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#if __has_include(<cloud_msgs/msg/cloud_features.hpp>)
#include <cloud_msgs/msg/cloud_features.hpp>
#define ADAPTIVE_ODOM_FILTER_HAS_CLOUD_MSGS 1
#else
#define ADAPTIVE_ODOM_FILTER_HAS_CLOUD_MSGS 0
#endif

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

inline constexpr double PI = 3.14159265;
inline const double inf = std::numeric_limits<double>::infinity();

struct bias {
    double x {};
    double y {};
    double z {};
};

#endif
