//=====================================================EKF-Fast-LIO2=====================================================================
//Institutions: Federal University of Minas Gerais (UFMG), Federal University of Ouro Preto (UFOP) and Instituto Tecnológico Vale (ITV)
//Description: This file is responsible for merging the wheel odometry with the IMU data and the Fast-LIO2 odometry.
//Milestones: 
//             Date: November 27, 2021
//             Description: Initial version of the code.
//             Members: Gilmar Pereira da Cruz Júnior and Adriano Resende
//             E-mails: gilmarpcruzjunior@gmail.com, adrianomcr18@gmail.com
//
//             Date: June 27, 2025
//             Description: Include the wheel odometry adaptive covariance and the Fast-LIO2 odometry input.
//             Members: Gilmar Pereira da Cruz Júnior and Gabriel Malaquias
//             E-mails: gilmarpcruzjunior@gmail.com, gmdeoliveira@ymail.com
//
//             Date: September 22, 2025
//             Description: New version of the code including visual odometry measurement.
//             Members: Gilmar Pereira da Cruz Júnior and Gabriel Malaquias
//             E-mails: gilmarpcruzjunior@gmail.com, gmdeoliveira@ymail.com
//=======================================================================================================================================

#include "settings_adaptive_filter.h"
// #include "ekf_adaptive_tools.h"
#include <adaptive_odom_filter/ekf_adaptive_tools.h>

// ------------------- ROS 2 / msgs / tf2 -------------------
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// rtabmap service (ROS 2)
#include <rtabmap_msgs/srv/reset_pose.hpp>

// ------------------- STL / Eigen -------------------
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <cstdint>

using namespace Eigen;
using namespace std;

class AdaptiveOdomFilterNode : public rclcpp::Node {

private:
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subWheelOdometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subVisualOdometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subVisualOdometryD;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subCamLeft;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subCamRight;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subCamRgb;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubFilteredOdometry;

    // Headers
    std_msgs::msg::Header headerI;
    std_msgs::msg::Header headerW;
    std_msgs::msg::Header headerL;
    std_msgs::msg::Header headerV;

    // Service client
    rclcpp::Client<rtabmap_msgs::srv::ResetPose>::SharedPtr srv_client_rgbd;

    // filtered odom
    nav_msgs::msg::Odometry filteredOdometry;

    // Times
    double imuTimeLast;
    double wheelTimeLast;
    double lidarTimeLast;
    double visualTimeLast;

    double imuTimeCurrent;
    double wheelTimeCurrent;
    double lidarTimeCurrent;
    double visualTimeCurrent;

    double imu_dt;
    double wheel_dt;
    double lidar_dt;
    double visual_dt;

    // boolean
    bool imuActivated;
    bool wheelActivated;
    bool lidarActivated;
    bool visualActivated;

    // adaptive covariance - visual odometry
    double averageIntensity1;
    double averageIntensity2;
    double averageIntensity;

    // filter constructor 
    AdaptiveOdomFilter filter;

    // Measure
    Eigen::VectorXd imuMeasure, wheelMeasure, lidarMeasure, visualMeasure;

    // Measure Covariance
    Eigen::MatrixXd E_imu, E_wheel, E_lidar, E_visual;

    // States and covariances
    Eigen::VectorXd X;
    Eigen::MatrixXd P;

    Eigen::Matrix3d R_init;
    rclcpp::Clock steady_clock_ {RCL_STEADY_TIME};

    std::string imu_topic_ {"/imu/data"};
    std::string wheel_odom_topic_ {"/odom_with_cov"};
    std::string lidar_odom_topic_ {"/ekf_loam/laser_odom_with_cov"};
    std::string tracking_odom_topic_ {"/tracking_odom"};
    std::string depth_odom_topic_ {"/depth_odom"};
    std::string left_camera_topic_ {"/left_camera"};
    std::string right_camera_topic_ {"/right_camera"};
    std::string color_camera_topic_ {"/camera_color"};
    std::string filter_odom_topic_ {"/filter_odom"};
    std::string rtabmap_service_topic_ {"/rtabmap/reset_odom_to_pose"};
    std::string odom_frame_id_ {"chassis_init"};
    std::string base_frame_id_ {"ekf_odom_frame"};

    double steady_now() const {
        return steady_clock_.now().seconds();
    }

    // --- tiny helpers ---
    static builtin_interfaces::msg::Time toStampFromSec(double sec) {
        builtin_interfaces::msg::Time t;
        if (sec < 0.0) sec = 0.0;
        const int32_t s = static_cast<int32_t>(sec);
        const double  frac = sec - static_cast<double>(s);
        const uint32_t ns = static_cast<uint32_t>(std::round(frac * 1e9));
        t.sec = s;
        t.nanosec = ns;
        return t;
    }


public:
    bool enableFilter;
    bool enableImu;
    bool enableWheel;
    bool enableLidar;
    bool enableVisual;

    double freq;

    double alpha_lidar;
    double alpha_visual;

    float lidarG;
    float visualG;
    float wheelGVx;
    float wheelGVy;
    float wheelGWz;
    float wheelOffset;
    float imuG;

    int experiment;

    float gamma_vx;
    float gamma_omegaz;
    float delta_vx;
    float delta_omegaz;

    double minIntensity; 
    double maxIntensity;

    int lidar_type_func;
    int visual_type_func;
    int wheel_type_func;

    int camera_type;

    std::string filterFreq;

    
    AdaptiveOdomFilterNode() : rclcpp::Node("adaptive_odom_filter")
    {
        initialization();
    }


    //------------------
    // Auxliar functions
    //------------------
    void initialization(){
        // times
        imuTimeLast = 0;
        lidarTimeLast = 0;
        visualTimeLast = 0;
        wheelTimeLast = 0;

        imuTimeCurrent = 0;
        lidarTimeCurrent = 0;
        visualTimeCurrent = 0;
        wheelTimeCurrent = 0;

        imu_dt = 0.005;
        wheel_dt = 0.05;
        lidar_dt = 0.1;
        visual_dt = 0.005;

        alpha_visual = 0.98;
        alpha_lidar = 0.98;

        // filter 
        freq = 200.0;

        // boolean
        imuActivated = false;
        lidarActivated = false;
        wheelActivated = false;
        visualActivated = false;

        enableFilter = false;
        enableImu = false;
        enableWheel = false;
        enableLidar = false;
        enableVisual = false;

        wheelGVx = 0;
        wheelGVy = 0;
        wheelGWz = 0;
        wheelOffset = 0;
        imuG = 0;
        experiment = 0;

        // adaptive covariance - visual odometry
        averageIntensity1 = 0;
        averageIntensity2 = 0;
        averageIntensity = 0;

        // measure
        imuMeasure.resize(9);
        wheelMeasure.resize(3);
        lidarMeasure.resize(6);
        visualMeasure.resize(6);

        imuMeasure = Eigen::VectorXd::Zero(9);
        wheelMeasure = Eigen::VectorXd::Zero(3);
        lidarMeasure = Eigen::VectorXd::Zero(6);
        visualMeasure = Eigen::VectorXd::Zero(6);

        E_imu.resize(9,9);
        E_wheel.resize(3,3);
        E_lidar.resize(6,6);
        E_visual.resize(6,6);

        E_imu = Eigen::MatrixXd::Zero(9,9);
        E_lidar = Eigen::MatrixXd::Zero(6,6);
        E_visual = Eigen::MatrixXd::Zero(6,6);
        E_wheel = Eigen::MatrixXd::Zero(3,3);

        X.resize(12);
        P.resize(12,12);
        X = Eigen::VectorXd::Zero(12);
        P = Eigen::MatrixXd::Zero(12,12);
        R_init = Eigen::Matrix3d::Identity();
    }

    void filter_initialization(){
        // setting the filter
        filter.enableImu = enableImu;
        filter.enableWheel = enableWheel;
        filter.enableLidar = enableLidar;
        filter.enableVisual = enableVisual;
        filter.lidar_type_func = lidar_type_func;
        filter.visual_type_func = visual_type_func;
        filter.wheel_type_func = wheel_type_func;

        filter.freq = freq;
        
        filter.wheelGVx = wheelGVx;
        filter.wheelGVy = wheelGVy;
        filter.wheelGWz = wheelGWz;
        filter.wheelOffset = wheelOffset;
        filter.imuG = imuG;
        filter.lidarG = lidarG;
        filter.visualG = visualG;

        filter.alpha_lidar = alpha_lidar;
        filter.alpha_visual = alpha_visual;

        filter.gamma_vx = gamma_vx;
        filter.gamma_omegaz = gamma_omegaz;
        filter.delta_vx = delta_vx;
        filter.delta_omegaz = delta_omegaz;

        filter.minIntensity = minIntensity;
        filter.maxIntensity = maxIntensity;
        filter.experiment = experiment;

        filter.lidar_type_func  = lidar_type_func; 
        filter.visual_type_func = visual_type_func;
        filter.wheel_type_func  = wheel_type_func; 
    }

    void ros_initialization(){
        using std::placeholders::_1;

        // Subscribers
        subLaserOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            lidar_odom_topic_, rclcpp::SensorDataQoS(), std::bind(&AdaptiveOdomFilterNode::laserOdometryHandler, this, _1));
        subWheelOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            wheel_odom_topic_, rclcpp::SensorDataQoS(), std::bind(&AdaptiveOdomFilterNode::wheelOdometryHandler, this, _1));
        subImu = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(), std::bind(&AdaptiveOdomFilterNode::imuHandler, this, _1));

        if (camera_type==1){
            subVisualOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
                tracking_odom_topic_, rclcpp::SensorDataQoS(), std::bind(&AdaptiveOdomFilterNode::visualOdometryHandler, this, _1));
            subCamLeft  = this->create_subscription<sensor_msgs::msg::Image>(
                left_camera_topic_, rclcpp::SensorDataQoS(), std::bind(&AdaptiveOdomFilterNode::camLeftHandler, this, _1));
            subCamRight = this->create_subscription<sensor_msgs::msg::Image>(
                right_camera_topic_, rclcpp::SensorDataQoS(), std::bind(&AdaptiveOdomFilterNode::camRightHandler, this, _1));
        }else if (camera_type==2){
            subVisualOdometryD = this->create_subscription<nav_msgs::msg::Odometry>(
                depth_odom_topic_, rclcpp::SensorDataQoS(), std::bind(&AdaptiveOdomFilterNode::visualOdometryDHandler, this, _1));
            subCamRgb = this->create_subscription<sensor_msgs::msg::Image>(
                color_camera_topic_, rclcpp::SensorDataQoS(), std::bind(&AdaptiveOdomFilterNode::camRgbHandler, this, _1));
        }
            
        // Publisher
        pubFilteredOdometry = this->create_publisher<nav_msgs::msg::Odometry> (filter_odom_topic_, 5);

        // Service client
        srv_client_rgbd = this->create_client<rtabmap_msgs::srv::ResetPose>(rtabmap_service_topic_);
    }

    //------------------
    // Filter functions
    //------------------
    void filter_start(){
        RCLCPP_INFO(this->get_logger(), "\033[1;32mAdaptive Filter:\033[0m Filter Started.");
        filter.start();
    }

    void filter_stop(){
        filter.stop();
    }

    //----------
    // callbacks
    //----------
    void imuHandler(const sensor_msgs::msg::Imu::ConstSharedPtr& imuIn){
        double timeL = steady_now();

        // time
        if (imuActivated){
            imuTimeLast = imuTimeCurrent;
            imuTimeCurrent = rclcpp::Time(imuIn->header.stamp).seconds();
        }else{
            imuTimeCurrent = rclcpp::Time(imuIn->header.stamp).seconds();
            imuTimeLast = imuTimeCurrent + 0.001;
            imuActivated = true;
        }       

        // roll, pitch and yaw 
        double roll, pitch, yaw;
        const auto& qmsg = imuIn->orientation;
        tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // measure
        imuMeasure.block(0,0,3,1) << imuIn->linear_acceleration.x, imuIn->linear_acceleration.y, imuIn->linear_acceleration.z;
        imuMeasure.block(3,0,3,1) << imuIn->angular_velocity.x, imuIn->angular_velocity.y, imuIn->angular_velocity.z; 
        imuMeasure.block(6,0,3,1) << roll, pitch, yaw;

        // covariance
        E_imu.block(0,0,3,3) << imuIn->linear_acceleration_covariance[0], imuIn->linear_acceleration_covariance[1], imuIn->linear_acceleration_covariance[2],
                                imuIn->linear_acceleration_covariance[3], imuIn->linear_acceleration_covariance[4], imuIn->linear_acceleration_covariance[5],
                                imuIn->linear_acceleration_covariance[6], imuIn->linear_acceleration_covariance[7], imuIn->linear_acceleration_covariance[8];
        E_imu.block(3,3,3,3) << imuIn->angular_velocity_covariance[0], imuIn->angular_velocity_covariance[1], imuIn->angular_velocity_covariance[2],
                                imuIn->angular_velocity_covariance[3], imuIn->angular_velocity_covariance[4], imuIn->angular_velocity_covariance[5],
                                imuIn->angular_velocity_covariance[6], imuIn->angular_velocity_covariance[7], imuIn->angular_velocity_covariance[8];
        E_imu.block(6,6,3,3) << imuIn->orientation_covariance[0], imuIn->orientation_covariance[1], imuIn->orientation_covariance[2],
                                imuIn->orientation_covariance[3], imuIn->orientation_covariance[4], imuIn->orientation_covariance[5],
                                imuIn->orientation_covariance[6], imuIn->orientation_covariance[7], imuIn->orientation_covariance[8];

        E_imu.block(6,6,3,3) = imuG*E_imu.block(6,6,3,3);

        // time
        imu_dt = imuTimeCurrent - imuTimeLast;
        imu_dt = 0.01;

        // header
        double timediff = steady_now() - timeL + imuTimeCurrent;
        headerI = imuIn->header;
        headerI.stamp = toStampFromSec(timediff);

        // correction stage aqui
        filter.correction_imu_data(imuMeasure, E_imu, imu_dt);
    }

    void wheelOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr& wheelOdometry){
        double timeL = steady_now();
    
        // time
        if (wheelActivated){
            wheelTimeLast    = wheelTimeCurrent;
            wheelTimeCurrent = rclcpp::Time(wheelOdometry->header.stamp).seconds();
        }else{
            wheelTimeCurrent = rclcpp::Time(wheelOdometry->header.stamp).seconds();
            wheelTimeLast    = wheelTimeCurrent;
            wheelActivated   = true;
            return;
        }
    
        wheel_dt = wheelTimeCurrent - wheelTimeLast;
    
        const double vx = -1.0 * wheelOdometry->twist.twist.linear.x;
        const double vy = 0.0;
        const double wz = wheelOdometry->twist.twist.angular.z;

        if (wheel_dt <= 1e-4)
            return;
    
        wheelMeasure << vx, vy, wz;
    
        E_wheel.setZero();
        E_wheel(0,0) = wheelGVx > 0.0f ? wheelGVx : 0.1;
        E_wheel(1,1) = wheelGVy > 0.0f ? wheelGVy : 0.01;
        E_wheel(2,2) = wheelGWz > 0.0f ? wheelGWz : 0.2;
    
        // header
        double timediff = steady_now() - timeL + wheelTimeCurrent;
        headerW = wheelOdometry->header;
        headerW.stamp = toStampFromSec(timediff);
    
        // correction
        filter.correction_wheel_data(wheelMeasure, E_wheel, wheel_dt, imuMeasure(5));
    }
    
    
    void laserOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr& laserOdometry){
        double timeL = steady_now();
    
        double stamp_sec = rclcpp::Time(laserOdometry->header.stamp).seconds();
    
        // time
        if (!lidarActivated) {
            lidarActivated   = true;
            lidarTimeLast    = stamp_sec;
            lidarTimeCurrent = stamp_sec;
            return;
        }
    
        lidarTimeLast    = lidarTimeCurrent;
        lidarTimeCurrent = stamp_sec;
        lidar_dt         = lidarTimeCurrent - lidarTimeLast;
    
        if (lidar_dt <= 1e-4)
            return;
    
        // rpy
        double roll, pitch, yaw;
        const auto& qmsg = laserOdometry->pose.pose.orientation;
        tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
        // measure
        lidarMeasure.block(0,0,3,1) << laserOdometry->pose.pose.position.x,
                                       laserOdometry->pose.pose.position.y,
                                       laserOdometry->pose.pose.position.z;
        lidarMeasure.block(3,0,3,1) << roll, pitch, yaw;
    
        // gate
        static bool have_last_lidar = false;
        static Eigen::Vector3d last_pos;
    
        Eigen::Vector3d curr_pos(
            laserOdometry->pose.pose.position.x,
            laserOdometry->pose.pose.position.y,
            laserOdometry->pose.pose.position.z);
    
        if (have_last_lidar) {
            double dx = (curr_pos - last_pos).norm();
            if (dx > 5.0) {
                last_pos = curr_pos;
                return;
            }
        }
        last_pos        = curr_pos;
        have_last_lidar = true;
    
        E_lidar = Eigen::MatrixXd::Constant(6, 6, 0.9);
    
        double corner = double(laserOdometry->twist.twist.linear.x);
        double surf   = double(laserOdometry->twist.twist.angular.x);
    
        // header
        double timediff = steady_now() - timeL + lidarTimeCurrent;
        headerL = laserOdometry->header;
        headerL.stamp = toStampFromSec(timediff);
    
        // correction
        filter.correction_lidar_data(lidarMeasure, E_lidar, lidar_dt, corner, surf);
        filter.get_state(X, P);
        publish_odom('l');
    }    
        
    void visualOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr& visualOdometry){
        if (camera_type == 1){ 
            double timeV = steady_now();

            if (visualActivated){
                visualTimeLast = visualTimeCurrent;
                visualTimeCurrent = rclcpp::Time(visualOdometry->header.stamp).seconds();
            }else{
                visualTimeCurrent = rclcpp::Time(visualOdometry->header.stamp).seconds();
                visualTimeLast = visualTimeCurrent + 0.01;
                visualActivated = true;
            }  
            
            // roll, pitch and yaw 
            double roll, pitch, yaw;
            const auto& qmsg = visualOdometry->pose.pose.orientation;
            tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            visualMeasure.block(0,0,3,1) << visualOdometry->pose.pose.position.x, visualOdometry->pose.pose.position.y, visualOdometry->pose.pose.position.z;
            visualMeasure.block(3,0,3,1) << roll, pitch, yaw;    

            // covariance (top-left 3x3)
            E_visual(0,0) = visualOdometry->pose.covariance[0];
            E_visual(0,1) = visualOdometry->pose.covariance[1];
            E_visual(0,2) = visualOdometry->pose.covariance[2];
            E_visual(1,0) = visualOdometry->pose.covariance[3];
            E_visual(1,1) = visualOdometry->pose.covariance[4];
            E_visual(1,2) = visualOdometry->pose.covariance[5];
            E_visual(2,0) = visualOdometry->pose.covariance[6];
            E_visual(2,1) = visualOdometry->pose.covariance[7];
            E_visual(2,2) = visualOdometry->pose.covariance[8];

            // time
            visual_dt = visualTimeCurrent - visualTimeLast;

            // header
            double timediff = steady_now() - timeV + visualTimeCurrent;
            headerV = visualOdometry->header;
            headerV.stamp = toStampFromSec(timediff);
            
            // compute average intensity
            averageIntensity = (averageIntensity1 + averageIntensity2)/2.0;

            //New measure
            filter.correction_visual_data(visualMeasure, E_visual, visual_dt, averageIntensity);
        }
    }

    void visualOdometryDHandler(const nav_msgs::msg::Odometry::ConstSharedPtr& visualOdometry){
        if (enableFilter && enableVisual && camera_type == 2){
            Eigen::MatrixXd E_visual(6,6);

            if (visualOdometry->pose.covariance[0] >= 9999.0 &&
                visualOdometry->pose.pose.position.x == 0 &&
                visualOdometry->pose.pose.position.y == 0 &&
                visualOdometry->pose.pose.position.z == 0)
            {
                // reset pose 
                auto req = std::make_shared<rtabmap_msgs::srv::ResetPose::Request>();
                req->x = visualOdometry->pose.pose.position.x;
                req->y = visualOdometry->pose.pose.position.y;
                req->z = visualOdometry->pose.pose.position.z;

                double roll, pitch, yaw;
                const auto& qmsg = visualOdometry->pose.pose.orientation;
                tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

                req->roll = roll;
                req->pitch = pitch;
                req->yaw = yaw;

                // call service (best-effort)
                if (!srv_client_rgbd->service_is_ready()) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 5000,
                        "ResetPose service is not ready; skipping request.");
                    return;
                }
                (void)srv_client_rgbd->async_send_request(req);
            }else{
                double timeV = steady_now();

                if (visualActivated){
                    visualTimeLast = visualTimeCurrent;
                    visualTimeCurrent = rclcpp::Time(visualOdometry->header.stamp).seconds();
                }else{
                    visualTimeCurrent = rclcpp::Time(visualOdometry->header.stamp).seconds();
                    visualTimeLast = visualTimeCurrent + 0.01;
                    visualActivated = true;
                }  
                
                // roll, pitch and yaw 
                double roll, pitch, yaw;
                const auto& qmsg = visualOdometry->pose.pose.orientation;
                tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

                visualMeasure.block(0,0,3,1) << visualOdometry->pose.pose.position.x, visualOdometry->pose.pose.position.y, visualOdometry->pose.pose.position.z;
                visualMeasure.block(3,0,3,1) << roll, pitch, yaw;    

                // covariance (top-left 3x3)
                E_visual(0,0) = visualOdometry->pose.covariance[0];
                E_visual(0,1) = visualOdometry->pose.covariance[1];
                E_visual(0,2) = visualOdometry->pose.covariance[2];
                E_visual(1,0) = visualOdometry->pose.covariance[3];
                E_visual(1,1) = visualOdometry->pose.covariance[4];
                E_visual(1,2) = visualOdometry->pose.covariance[5];
                E_visual(2,0) = visualOdometry->pose.covariance[6];
                E_visual(2,1) = visualOdometry->pose.covariance[7];
                E_visual(2,2) = visualOdometry->pose.covariance[8];

                // time
                visual_dt = visualTimeCurrent - visualTimeLast;

                // header
                double timediff = steady_now() - timeV + visualTimeCurrent;
                headerV = visualOdometry->header;
                headerV.stamp = toStampFromSec(timediff);
                
                //New measure
                filter.correction_visual_data(visualMeasure, E_visual, visual_dt, averageIntensity);
            }            
        }
    }

    void camLeftHandler(const sensor_msgs::msg::Image::ConstSharedPtr& camIn){
        int width = static_cast<int>(camIn->width);
        int height = static_cast<int>(camIn->height);

        int numPixels = width * height;
        double intensitySum = 0.0;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int pixelIndex = y * width + x;
                uint8_t intensity = camIn->data[pixelIndex];

                intensitySum += intensity;
            }
        }

        averageIntensity2 = intensitySum / numPixels;
    }

    void camRightHandler(const sensor_msgs::msg::Image::ConstSharedPtr& camIn){
        int width = static_cast<int>(camIn->width);
        int height = static_cast<int>(camIn->height);

        int numPixels = width * height;
        double intensitySum = 0.0;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int pixelIndex = y * width + x;
                uint8_t intensity = camIn->data[pixelIndex];

                intensitySum += intensity;
            }
        }

        averageIntensity1 = intensitySum / numPixels;
    }

    void camRgbHandler(const sensor_msgs::msg::Image::ConstSharedPtr& camIn){
        int width = static_cast<int>(camIn->width);
        int height = static_cast<int>(camIn->height);
        int numPixels = width * height;

        // Calculate pixel step size
        size_t pixel_step = camIn->step / camIn->width;

        // Define weights for RGB channels
        double redWeight = 0.2989;
        double greenWeight = 0.5870;
        double blueWeight = 0.1140;
        double intensitySum = 0.0;

        // Loop through image data and compute intensity
        for (size_t y = 0; y < camIn->height; ++y){
            for (size_t x = 0; x < camIn->width; ++x){
                size_t index = y * camIn->step + x * pixel_step;

                // Access RGB values
                uint8_t r = camIn->data[index];
                uint8_t g = camIn->data[index + 1];
                uint8_t b = camIn->data[index + 2];

                // Compute intensity using weighted average of RGB values
                intensitySum += redWeight * double(r) + greenWeight * double(g) + blueWeight * double(b);
            }
        }

        averageIntensity = intensitySum / numPixels;
    }

    //----------
    // publisher
    //----------
    void publish_odom(char model){
        switch(model){
                case 'i':
                    filteredOdometry.header = headerI;
                    break;
                case 'w':
                    filteredOdometry.header = headerW;
                    break;
                case 'l':
                    filteredOdometry.header = headerL;
                    break;
                case 'v':
                    filteredOdometry.header = headerV;
                    break;
            }
        
        filteredOdometry.header.frame_id = odom_frame_id_;
        filteredOdometry.child_frame_id = base_frame_id_;

        tf2::Quaternion q;
        q.setRPY (X(3), X(4), X(5));
        geometry_msgs::msg::Quaternion geoQuat = tf2::toMsg(q);

        // pose
        filteredOdometry.pose.pose.orientation = geoQuat;
        filteredOdometry.pose.pose.position.x = X(0); 
        filteredOdometry.pose.pose.position.y = X(1);
        filteredOdometry.pose.pose.position.z = X(2);

        // pose convariance
        int k = 0;
        for (int i = 0; i < 6; i++){
            for (int j = 0; j < 6; j++){
                filteredOdometry.pose.covariance[k] = P(i,j);
                k++;
            }
        }      

        // twist
        filteredOdometry.twist.twist.linear.x = X(6);
        filteredOdometry.twist.twist.linear.y = X(7);
        filteredOdometry.twist.twist.linear.z = X(8);
        filteredOdometry.twist.twist.angular.x = X(9);
        filteredOdometry.twist.twist.angular.y = X(10);
        filteredOdometry.twist.twist.angular.z = X(11);

        // twist convariance
        k = 0;
        for (int i = 6; i < 12; i++){
            for (int j = 6; j < 12; j++){
                filteredOdometry.twist.covariance[k] = P(i,j);
                k++;
            }
        } 

        pubFilteredOdometry->publish(filteredOdometry);
    }

public:
    // ----------------- parameter loading (minimal change) -----------------
    void load_parameters()
    {   
        // Declare with defaults (ROS 2 style). Use flat names.
        this->declare_parameter<bool>("enableFilter", false);
        this->declare_parameter<bool>("enableImu", false);
        this->declare_parameter<bool>("enableWheel", false);
        this->declare_parameter<bool>("enableLidar", false);
        this->declare_parameter<bool>("enableVisual", false);

        this->declare_parameter<std::string>("filterFreq", std::string("l"));
        this->declare_parameter<double>("freq", 200.0);

        this->declare_parameter<float>("wheelGVx", 0.001f);
        this->declare_parameter<float>("wheelGVy", 1.0f);
        this->declare_parameter<float>("wheelGWz", 40.0f);
        this->declare_parameter<float>("wheelOffset", 1.0e-5f);
        this->declare_parameter<float>("imuG", 0.1f);

        this->declare_parameter<double>("alpha_lidar", 0.98);
        this->declare_parameter<double>("alpha_visual", 0.98);

        this->declare_parameter<float>("lidarG", 1000.0f);
        this->declare_parameter<float>("visualG", 0.05f);

        this->declare_parameter<float>("gamma_vx", 0.05f);
        this->declare_parameter<float>("gamma_omegaz", 0.01f);
        this->declare_parameter<float>("delta_vx", 0.0001f);
        this->declare_parameter<float>("delta_omegaz", 0.00001f);

        this->declare_parameter<double>("minIntensity", 0.0);
        this->declare_parameter<double>("maxIntensity", 1.0);

        this->declare_parameter<int>("lidar_type_func", 2);
        this->declare_parameter<int>("visual_type_func", 2);
        this->declare_parameter<int>("wheel_type_func", 1);
        this->declare_parameter<int>("experiment", 0);

        this->declare_parameter<int>("camera_type", 0);
        this->declare_parameter<std::string>("imu_topic", imu_topic_);
        this->declare_parameter<std::string>("wheel_odom_topic", wheel_odom_topic_);
        this->declare_parameter<std::string>("lidar_odom_topic", lidar_odom_topic_);
        this->declare_parameter<std::string>("tracking_odom_topic", tracking_odom_topic_);
        this->declare_parameter<std::string>("depth_odom_topic", depth_odom_topic_);
        this->declare_parameter<std::string>("left_camera_topic", left_camera_topic_);
        this->declare_parameter<std::string>("right_camera_topic", right_camera_topic_);
        this->declare_parameter<std::string>("color_camera_topic", color_camera_topic_);
        this->declare_parameter<std::string>("filter_odom_topic", filter_odom_topic_);
        this->declare_parameter<std::string>("rtabmap_service_topic", rtabmap_service_topic_);
        this->declare_parameter<std::string>("odom_frame_id", odom_frame_id_);
        this->declare_parameter<std::string>("base_frame_id", base_frame_id_);
        
        // Get values
        enableFilter = this->get_parameter("enableFilter").as_bool();
        enableImu    = this->get_parameter("enableImu").as_bool();
        enableWheel  = this->get_parameter("enableWheel").as_bool();
        enableLidar  = this->get_parameter("enableLidar").as_bool();
        enableVisual = this->get_parameter("enableVisual").as_bool();

        filterFreq = this->get_parameter("filterFreq").as_string();
        freq       = this->get_parameter("freq").as_double();

        wheelGVx = this->get_parameter("wheelGVx").as_double();
        wheelGVy = this->get_parameter("wheelGVy").as_double();
        wheelGWz = this->get_parameter("wheelGWz").as_double();
        wheelOffset = this->get_parameter("wheelOffset").as_double();
        imuG = this->get_parameter("imuG").as_double();

        alpha_lidar  = this->get_parameter("alpha_lidar").as_double();
        alpha_visual = this->get_parameter("alpha_visual").as_double();

        lidarG  = this->get_parameter("lidarG").as_double();
        visualG = this->get_parameter("visualG").as_double();

        gamma_vx     = this->get_parameter("gamma_vx").as_double();
        gamma_omegaz = this->get_parameter("gamma_omegaz").as_double();
        delta_vx     = this->get_parameter("delta_vx").as_double();
        delta_omegaz = this->get_parameter("delta_omegaz").as_double();

        minIntensity = this->get_parameter("minIntensity").as_double();
        maxIntensity = this->get_parameter("maxIntensity").as_double();

        lidar_type_func  = this->get_parameter("lidar_type_func").as_int();
        visual_type_func = this->get_parameter("visual_type_func").as_int();
        wheel_type_func  = this->get_parameter("wheel_type_func").as_int();
        experiment = this->get_parameter("experiment").as_int();

        camera_type = this->get_parameter("camera_type").as_int();
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        wheel_odom_topic_ = this->get_parameter("wheel_odom_topic").as_string();
        lidar_odom_topic_ = this->get_parameter("lidar_odom_topic").as_string();
        tracking_odom_topic_ = this->get_parameter("tracking_odom_topic").as_string();
        depth_odom_topic_ = this->get_parameter("depth_odom_topic").as_string();
        left_camera_topic_ = this->get_parameter("left_camera_topic").as_string();
        right_camera_topic_ = this->get_parameter("right_camera_topic").as_string();
        color_camera_topic_ = this->get_parameter("color_camera_topic").as_string();
        filter_odom_topic_ = this->get_parameter("filter_odom_topic").as_string();
        rtabmap_service_topic_ = this->get_parameter("rtabmap_service_topic").as_string();
        odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    }
};


//-----------------------------
// Main 
//-----------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AdaptiveOdomFilterNode>();

    // Parameters init (ROS 2): loaded from YAML/CLI into the node
    try {
        RCLCPP_INFO(node->get_logger(),
            "\033[1;31mAdaptive Filter:\033[0m Loading parameters.");
        node->load_parameters();
    } catch (...) {
        RCLCPP_INFO(node->get_logger(),
            "\033[1;31mAdaptive Filter:\033[0m Exception occurred when importing parameters in Adaptive Filter Node.");
    }

    // ros initialization
    node->ros_initialization();

    // filter initialization
    node->filter_initialization();

    // Executor similar to AsyncSpinner(7)
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 7);

    if (node->enableFilter){
        RCLCPP_INFO(node->get_logger(), "\033[1;32mAdaptive Filter:\033[0m Started.");
        node->filter_start();
    }else{
        node->filter_stop();
        RCLCPP_INFO(node->get_logger(), "\033[1;32mAdaptive Filter: \033[0m Stopped.");
    }

    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
