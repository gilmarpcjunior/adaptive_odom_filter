#include "settings_adaptive_filter.h"
#include <adaptive_odom_filter/adaptive_robust_ekf.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <memory>
#include <string>

namespace {

builtin_interfaces::msg::Time to_stamp_from_sec(double sec) {
    builtin_interfaces::msg::Time stamp;
    if (sec < 0.0) {
        sec = 0.0;
    }

    const auto whole = static_cast<int32_t>(sec);
    const auto frac = sec - static_cast<double>(whole);
    stamp.sec = whole;
    stamp.nanosec = static_cast<uint32_t>(std::round(frac * 1.0e9));
    return stamp;
}

}  // namespace

class AdaptiveRobustOdomFilterNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_wheel_odometry_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_laser_odometry_;
#if ADAPTIVE_ODOM_FILTER_HAS_CLOUD_MSGS
    rclcpp::Subscription<cloud_msgs::msg::CloudFeatures>::SharedPtr sub_lidar_features_;
#endif

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_filtered_odometry_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_lidar_extrinsic_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_imu_extrinsic_;

    std_msgs::msg::Header header_i_;
    std_msgs::msg::Header header_w_;
    std_msgs::msg::Header header_l_;

    nav_msgs::msg::Odometry filtered_odometry_;
    geometry_msgs::msg::PoseStamped lidar_extrinsic_pose_;
    geometry_msgs::msg::PoseStamped imu_extrinsic_pose_;

    static constexpr int kNumStates = 57;
    static constexpr int kNumImu = 9;
    static constexpr int kNumWheel = 3;
    static constexpr int kNumLidar = 6;

    double imu_time_last_ {};
    double wheel_time_last_ {};
    double lidar_time_last_ {};
    double imu_time_current_ {};
    double wheel_time_current_ {};
    double lidar_time_current_ {};
    double imu_dt_ {0.005};
    double wheel_dt_ {0.05};
    double lidar_dt_ {0.1};

    bool imu_activated_ {false};
    bool wheel_activated_ {false};
    bool lidar_activated_ {false};

    AdaptiveRobustEKF filter_;

    Eigen::VectorXd imu_measure_ {Eigen::VectorXd::Zero(kNumImu)};
    Eigen::VectorXd wheel_measure_ {Eigen::VectorXd::Zero(kNumWheel)};
    Eigen::VectorXd lidar_measure_ {Eigen::VectorXd::Zero(kNumLidar)};
    Eigen::MatrixXd e_imu_ {Eigen::MatrixXd::Zero(kNumImu, kNumImu)};
    Eigen::MatrixXd e_wheel_ {Eigen::MatrixXd::Zero(kNumWheel, kNumWheel)};
    Eigen::MatrixXd e_lidar_ {Eigen::MatrixXd::Zero(kNumLidar, kNumLidar)};
    Eigen::VectorXd x_ {Eigen::VectorXd::Zero(kNumStates)};
    Eigen::MatrixXd p_ {Eigen::MatrixXd::Zero(kNumStates, kNumStates)};

    double lidar_corner_ {};
    double lidar_surf_ {};

    std::string imu_topic_ {"/imu/data"};
    std::string wheel_odom_topic_ {"/odom_with_cov"};
    std::string lidar_odom_topic_ {"/ekf_loam/laser_odom_with_cov"};
    std::string lidar_features_topic_ {"/features_cloud_info"};
    std::string filter_odom_topic_ {"/filter_odom"};
    std::string lidar_extrinsic_topic_ {"/lidar_extrinsic_pose"};
    std::string imu_extrinsic_topic_ {"/imu_extrinsic_pose"};
    std::string odom_frame_id_ {"chassis_init"};
    std::string base_frame_id_ {"ekf_odom_frame"};
    std::string lidar_frame_id_ {"lidar"};
    std::string imu_frame_id_ {"imu"};

public:
    bool enableFilter {};
    bool enableImu {};
    bool enableWheel {};
    bool enableLidar {};

    double freq {200.0};
    double alpha_lidar {0.98};

    float lidarG {};
    float wheelG {};
    float imuG {};

    float gamma_vx {};
    float gamma_omegaz {};
    float delta_vx {};
    float delta_omegaz {};

    int lidar_type_func {};
    int wheel_type_func {};
    std::string filterFreq {"l"};

    AdaptiveRobustOdomFilterNode() : rclcpp::Node("adaptive_robust_odom_filter") {}

    void load_parameters() {
        declare_parameter<bool>("enableFilter", false);
        declare_parameter<bool>("enableImu", false);
        declare_parameter<bool>("enableWheel", false);
        declare_parameter<bool>("enableLidar", false);
        declare_parameter<std::string>("filterFreq", "l");
        declare_parameter<double>("freq", 200.0);
        declare_parameter<double>("alpha_lidar", 0.98);
        declare_parameter<double>("lidarG", 10000000.0);
        declare_parameter<double>("wheelG", 10.0);
        declare_parameter<double>("imuG", 1.0);
        declare_parameter<double>("gamma_vx", 0.05);
        declare_parameter<double>("gamma_omegaz", 0.01);
        declare_parameter<double>("delta_vx", 0.0001);
        declare_parameter<double>("delta_omegaz", 0.00001);
        declare_parameter<int>("lidar_type_func", 2);
        declare_parameter<int>("wheel_type_func", 1);
        declare_parameter<std::string>("imu_topic", imu_topic_);
        declare_parameter<std::string>("wheel_odom_topic", wheel_odom_topic_);
        declare_parameter<std::string>("lidar_odom_topic", lidar_odom_topic_);
        declare_parameter<std::string>("lidar_features_topic", lidar_features_topic_);
        declare_parameter<std::string>("filter_odom_topic", filter_odom_topic_);
        declare_parameter<std::string>("lidar_extrinsic_topic", lidar_extrinsic_topic_);
        declare_parameter<std::string>("imu_extrinsic_topic", imu_extrinsic_topic_);
        declare_parameter<std::string>("odom_frame_id", odom_frame_id_);
        declare_parameter<std::string>("base_frame_id", base_frame_id_);
        declare_parameter<std::string>("lidar_frame_id", lidar_frame_id_);
        declare_parameter<std::string>("imu_frame_id", imu_frame_id_);

        enableFilter = get_parameter("enableFilter").as_bool();
        enableImu = get_parameter("enableImu").as_bool();
        enableWheel = get_parameter("enableWheel").as_bool();
        enableLidar = get_parameter("enableLidar").as_bool();
        filterFreq = get_parameter("filterFreq").as_string();
        freq = get_parameter("freq").as_double();
        alpha_lidar = get_parameter("alpha_lidar").as_double();
        lidarG = static_cast<float>(get_parameter("lidarG").as_double());
        wheelG = static_cast<float>(get_parameter("wheelG").as_double());
        imuG = static_cast<float>(get_parameter("imuG").as_double());
        gamma_vx = static_cast<float>(get_parameter("gamma_vx").as_double());
        gamma_omegaz = static_cast<float>(get_parameter("gamma_omegaz").as_double());
        delta_vx = static_cast<float>(get_parameter("delta_vx").as_double());
        delta_omegaz = static_cast<float>(get_parameter("delta_omegaz").as_double());
        lidar_type_func = get_parameter("lidar_type_func").as_int();
        wheel_type_func = get_parameter("wheel_type_func").as_int();
        imu_topic_ = get_parameter("imu_topic").as_string();
        wheel_odom_topic_ = get_parameter("wheel_odom_topic").as_string();
        lidar_odom_topic_ = get_parameter("lidar_odom_topic").as_string();
        lidar_features_topic_ = get_parameter("lidar_features_topic").as_string();
        filter_odom_topic_ = get_parameter("filter_odom_topic").as_string();
        lidar_extrinsic_topic_ = get_parameter("lidar_extrinsic_topic").as_string();
        imu_extrinsic_topic_ = get_parameter("imu_extrinsic_topic").as_string();
        odom_frame_id_ = get_parameter("odom_frame_id").as_string();
        base_frame_id_ = get_parameter("base_frame_id").as_string();
        lidar_frame_id_ = get_parameter("lidar_frame_id").as_string();
        imu_frame_id_ = get_parameter("imu_frame_id").as_string();
    }

    void filter_initialization() {
        filter_.enableImu = enableImu;
        filter_.enableWheel = enableWheel;
        filter_.enableLidar = enableLidar;
        filter_.lidar_type_func = lidar_type_func;
        filter_.wheel_type_func = wheel_type_func;
        filter_.freq = freq;
        filter_.wheelG = wheelG;
        filter_.imuG = imuG;
        filter_.lidarG = lidarG;
        filter_.alpha_lidar = alpha_lidar;
        filter_.gamma_vx = gamma_vx;
        filter_.gamma_omegaz = gamma_omegaz;
        filter_.delta_vx = delta_vx;
        filter_.delta_omegaz = delta_omegaz;
    }

    void ros_initialization() {
        using std::placeholders::_1;
        const auto sensor_qos = rclcpp::SensorDataQoS();

        sub_laser_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
            lidar_odom_topic_, sensor_qos,
            std::bind(&AdaptiveRobustOdomFilterNode::laserOdometryHandler, this, _1));
        sub_wheel_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
            wheel_odom_topic_, sensor_qos,
            std::bind(&AdaptiveRobustOdomFilterNode::wheelOdometryHandler, this, _1));
        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, sensor_qos,
            std::bind(&AdaptiveRobustOdomFilterNode::imuHandler, this, _1));
#if ADAPTIVE_ODOM_FILTER_HAS_CLOUD_MSGS
        sub_lidar_features_ = create_subscription<cloud_msgs::msg::CloudFeatures>(
            lidar_features_topic_, sensor_qos,
            std::bind(&AdaptiveRobustOdomFilterNode::laserFeaturesHandler, this, _1));
#endif

        pub_filtered_odometry_ = create_publisher<nav_msgs::msg::Odometry>(filter_odom_topic_, 5);
        pub_lidar_extrinsic_ =
            create_publisher<geometry_msgs::msg::PoseStamped>(lidar_extrinsic_topic_, 5);
        pub_imu_extrinsic_ =
            create_publisher<geometry_msgs::msg::PoseStamped>(imu_extrinsic_topic_, 5);
    }

    void filter_start() {
        RCLCPP_INFO(get_logger(), "Adaptive robust filter started.");
        filter_.start();
    }

    void filter_stop() {
        filter_.stop();
    }

    void imuHandler(const sensor_msgs::msg::Imu::ConstSharedPtr & imu_in) {
        const double time_local = rclcpp::Clock(RCL_STEADY_TIME).now().seconds();

        if (imu_activated_) {
            imu_time_last_ = imu_time_current_;
            imu_time_current_ = rclcpp::Time(imu_in->header.stamp).seconds();
        } else {
            imu_time_current_ = rclcpp::Time(imu_in->header.stamp).seconds();
            imu_time_last_ = imu_time_current_ + 0.001;
            imu_activated_ = true;
        }

        double roll {};
        double pitch {};
        double yaw {};
        const auto & orientation = imu_in->orientation;
        tf2::Matrix3x3(tf2::Quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);

        imu_measure_.block(0, 0, 3, 1) <<
            imu_in->angular_velocity.x, imu_in->angular_velocity.y, imu_in->angular_velocity.z;
        imu_measure_.block(3, 0, 3, 1) <<
            imu_in->linear_acceleration.x, imu_in->linear_acceleration.y, imu_in->linear_acceleration.z;
        imu_measure_.block(6, 0, 3, 1) << roll, pitch, yaw;

        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                e_imu_(row, col) = imu_in->angular_velocity_covariance[row * 3 + col];
                e_imu_(row + 3, col + 3) =
                    imu_in->linear_acceleration_covariance[row * 3 + col];
                e_imu_(row + 6, col + 6) = imu_in->orientation_covariance[row * 3 + col];
            }
        }
        e_imu_.block(6, 6, 3, 3) *= imuG;

        imu_dt_ = imu_time_current_ - imu_time_last_;
        header_i_ = imu_in->header;
        header_i_.stamp = to_stamp_from_sec(
            rclcpp::Clock(RCL_STEADY_TIME).now().seconds() - time_local + imu_time_current_);

        filter_.correction_imu_data(imu_measure_, e_imu_, imu_dt_);
    }

    void wheelOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr & wheel_odometry) {
        const double time_local = rclcpp::Clock(RCL_STEADY_TIME).now().seconds();

        if (wheel_activated_) {
            wheel_time_last_ = wheel_time_current_;
            wheel_time_current_ = rclcpp::Time(wheel_odometry->header.stamp).seconds();
        } else {
            wheel_time_current_ = rclcpp::Time(wheel_odometry->header.stamp).seconds();
            wheel_time_last_ = wheel_time_current_ + 0.01;
            wheel_activated_ = true;
        }

        wheel_measure_ << -1.0 * wheel_odometry->twist.twist.linear.x, 0.0,
            wheel_odometry->twist.twist.angular.z;
        e_wheel_.setZero();
        e_wheel_(0, 0) = 0.1;
        e_wheel_(1, 1) = 0.01;
        e_wheel_(2, 2) = 0.2;

        wheel_dt_ = wheel_time_current_ - wheel_time_last_;
        header_w_ = wheel_odometry->header;
        header_w_.stamp = to_stamp_from_sec(
            rclcpp::Clock(RCL_STEADY_TIME).now().seconds() - time_local + wheel_time_current_);

        filter_.correction_wheel_data(wheel_measure_, e_wheel_, wheel_dt_, imu_measure_(2));
    }

    void laserOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr & laser_odometry) {
        const double time_local = rclcpp::Clock(RCL_STEADY_TIME).now().seconds();

        if (lidar_activated_) {
            lidar_time_last_ = lidar_time_current_;
            lidar_time_current_ = rclcpp::Time(laser_odometry->header.stamp).seconds();
        } else {
            lidar_time_current_ = rclcpp::Time(laser_odometry->header.stamp).seconds();
            lidar_time_last_ = lidar_time_current_ + 0.01;
            lidar_activated_ = true;
        }

        double roll {};
        double pitch {};
        double yaw {};
        const auto & orientation = laser_odometry->pose.pose.orientation;
        tf2::Matrix3x3(tf2::Quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);

        lidar_measure_.block(0, 0, 3, 1) <<
            laser_odometry->pose.pose.position.x, laser_odometry->pose.pose.position.y,
            laser_odometry->pose.pose.position.z;
        lidar_measure_.block(3, 0, 3, 1) << roll, pitch, yaw;

        e_lidar_ = Eigen::MatrixXd::Constant(kNumLidar, kNumLidar, 0.9);
        lidar_dt_ = lidar_time_current_ - lidar_time_last_;
        header_l_ = laser_odometry->header;
        header_l_.stamp = to_stamp_from_sec(
            rclcpp::Clock(RCL_STEADY_TIME).now().seconds() - time_local + lidar_time_current_);

        filter_.correction_lidar_data(lidar_measure_, e_lidar_, lidar_dt_, lidar_corner_, lidar_surf_);
        filter_.get_state(x_, p_);
        publish_odom('l');
    }

#if ADAPTIVE_ODOM_FILTER_HAS_CLOUD_MSGS
    void laserFeaturesHandler(const cloud_msgs::msg::CloudFeatures::ConstSharedPtr & laser_features) {
        lidar_corner_ = laser_features->num_edge_points;
        lidar_surf_ = laser_features->num_plane_points;
    }
#endif

    void publish_odom(char model) {
        switch (model) {
            case 'i':
                filtered_odometry_.header = header_i_;
                break;
            case 'w':
                filtered_odometry_.header = header_w_;
                break;
            case 'l':
            default:
                filtered_odometry_.header = header_l_;
                break;
        }

        filtered_odometry_.header.frame_id = odom_frame_id_;
        filtered_odometry_.child_frame_id = base_frame_id_;

        tf2::Quaternion q;
        q.setRPY(x_(3), x_(4), x_(5));
        filtered_odometry_.pose.pose.orientation = tf2::toMsg(q);
        filtered_odometry_.pose.pose.position.x = x_(0);
        filtered_odometry_.pose.pose.position.y = x_(1);
        filtered_odometry_.pose.pose.position.z = x_(2);

        int k = 0;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                filtered_odometry_.pose.covariance[k++] = p_(i, j);
            }
        }

        filtered_odometry_.twist.twist.linear.x = x_(6);
        filtered_odometry_.twist.twist.linear.y = x_(7);
        filtered_odometry_.twist.twist.linear.z = x_(8);
        filtered_odometry_.twist.twist.angular.x = x_(9);
        filtered_odometry_.twist.twist.angular.y = x_(10);
        filtered_odometry_.twist.twist.angular.z = x_(11);

        k = 0;
        for (int i = 6; i < 12; ++i) {
            for (int j = 6; j < 12; ++j) {
                filtered_odometry_.twist.covariance[k++] = p_(i, j);
            }
        }

        pub_filtered_odometry_->publish(filtered_odometry_);
        publish_extrinsics(filtered_odometry_.header);
    }

    void publish_extrinsics(const std_msgs::msg::Header & header) {
        lidar_extrinsic_pose_.header = header;
        lidar_extrinsic_pose_.header.frame_id = lidar_frame_id_;
        lidar_extrinsic_pose_.pose.position.x = x_(45);
        lidar_extrinsic_pose_.pose.position.y = x_(46);
        lidar_extrinsic_pose_.pose.position.z = x_(47);
        tf2::Quaternion q_lidar;
        q_lidar.setRPY(x_(48), x_(49), x_(50));
        lidar_extrinsic_pose_.pose.orientation = tf2::toMsg(q_lidar);
        pub_lidar_extrinsic_->publish(lidar_extrinsic_pose_);

        imu_extrinsic_pose_.header = header;
        imu_extrinsic_pose_.header.frame_id = imu_frame_id_;
        imu_extrinsic_pose_.pose.position.x = x_(51);
        imu_extrinsic_pose_.pose.position.y = x_(52);
        imu_extrinsic_pose_.pose.position.z = x_(53);
        tf2::Quaternion q_imu;
        q_imu.setRPY(x_(54), x_(55), x_(56));
        imu_extrinsic_pose_.pose.orientation = tf2::toMsg(q_imu);
        pub_imu_extrinsic_->publish(imu_extrinsic_pose_);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AdaptiveRobustOdomFilterNode>();
    node->load_parameters();
    node->ros_initialization();
    node->filter_initialization();

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 7);

    if (node->enableFilter) {
        node->filter_start();
    } else {
        node->filter_stop();
        RCLCPP_INFO(node->get_logger(), "Adaptive robust filter stopped.");
    }

    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
