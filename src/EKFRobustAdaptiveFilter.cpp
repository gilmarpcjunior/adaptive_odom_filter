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
#include <adaptive_odom_filter/adaptive_robust_ekf.h>
#include <rtabmap_msgs/ResetPose.h>

using namespace Eigen;
using namespace std;

//-----------------------------
// Global variables
//-----------------------------
std::mutex mtx; 

//-----------------------------
// Adaptive EKF class
//-----------------------------
class adaptive_odom_filter{

private:
    // ros node
    ros::NodeHandle nh;

    // Subscriber
    ros::Subscriber subImu;
    ros::Subscriber subWheelOdometry;
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subLiDARFeatures;

    // Publisher
    ros::Publisher pubFilteredOdometry;
    ros::Publisher pubLidarExtrinsic;
    ros::Publisher pubImuExtrinsic;

    // header
    std_msgs::Header headerI;
    std_msgs::Header headerW;
    std_msgs::Header headerL;
    std_msgs::Header headerV;

    // services
    ros::ServiceClient srv_client_rgbd;

    // TF 
    tf::StampedTransform filteredOdometryTrans;
    tf::TransformBroadcaster tfBroadcasterfiltered;

    // filtered odom
    nav_msgs::Odometry filteredOdometry;

    // number of state or measure vectors
    // x = [pos[3] ori[3] vel[3] acc[3] ang[3] vang[3] bias[12] biasL[6] biasW[3] biasI[6] pose_lidar_robo[6] pose_imu_robo[6]]
    int _N_STATES = 57;
    int _N_IMU = 6; 
    int _N_WHEEL = 3; 
    int _N_LIDAR = 6;

    // Times
    double imuTimeLast;
    double wheelTimeLast;
    double lidarTimeLast;

    double imuTimeCurrent;
    double wheelTimeCurrent;
    double lidarTimeCurrent;

    double imu_dt;
    double wheel_dt;
    double lidar_dt;

    // boolean
    bool imuActivated;
    bool wheelActivated;
    bool lidarActivated;

    // filter constructor 
    AdaptiveRobustEKF filter;

    // Measure
    Eigen::VectorXd imuMeasure, wheelMeasure, lidarMeasure;

    // Measure Covariance
    Eigen::MatrixXd E_imu, E_wheel, E_lidar;

    // States and covariances
    Eigen::VectorXd X;
    Eigen::MatrixXd P;

    // features
    double lidarCorner;
    double lidarSurf;

    // calibration pose topics
    geometry_msgs::PoseStamped lidarExtrinsicPose;
    geometry_msgs::PoseStamped imuExtrinsicPose;

public:
    bool enableFilter;
    bool enableImu;
    bool enableWheel;
    bool enableLidar;

    double freq;

    double alpha_lidar;

    float lidarG;
    float wheelG;
    float imuG;

    float gamma_vx;
    float gamma_omegaz;
    float delta_vx;
    float delta_omegaz;

    int lidar_type_func;
    int wheel_type_func;

    std::string filterFreq;

    adaptive_odom_filter():
        nh("~")
    {
        // Initialization
        initialization();
    }

    //------------------
    // Auxliar functions
    //------------------
    void initialization(){
        // times
        imuTimeLast = 0;
        lidarTimeLast = 0;
        wheelTimeLast = 0;

        imuTimeCurrent = 0;
        lidarTimeCurrent = 0;
        wheelTimeCurrent = 0;

        imu_dt = 0.005;
        wheel_dt = 0.05;
        lidar_dt = 0.1;

        alpha_lidar = 0.98;

        // filter 
        freq = 200.0;

        // boolean
        imuActivated = false;
        lidarActivated = false;
        wheelActivated = false;

        enableFilter = false;
        enableImu = false;
        enableWheel = false;
        enableLidar = false;

        wheelG = 0; // delete
        imuG = 0;

        // measure
        imuMeasure.resize(_N_IMU+3);
        wheelMeasure.resize(_N_WHEEL);
        lidarMeasure.resize(_N_LIDAR);

        imuMeasure = Eigen::VectorXd::Zero(_N_IMU+3);
        wheelMeasure = Eigen::VectorXd::Zero(_N_WHEEL);
        lidarMeasure = Eigen::VectorXd::Zero(_N_LIDAR);

        E_imu.resize(_N_IMU+3,_N_IMU+3);
        E_wheel.resize(_N_WHEEL,_N_WHEEL);
        E_lidar.resize(_N_LIDAR,_N_LIDAR);

        E_imu = Eigen::MatrixXd::Zero(_N_IMU+3,_N_IMU+3);
        E_lidar = Eigen::MatrixXd::Zero(_N_LIDAR,_N_LIDAR);
        E_wheel = Eigen::MatrixXd::Zero(_N_WHEEL,_N_WHEEL);

        X.resize(_N_STATES);
        P.resize(_N_STATES,_N_STATES);
        X = Eigen::VectorXd::Zero(_N_STATES);
        P = Eigen::MatrixXd::Zero(_N_STATES,_N_STATES);
    }

    void filter_initialization(){
        // setting the filter
        filter.enableImu = enableImu;
        filter.enableWheel = enableWheel;
        filter.enableLidar = enableLidar;
        filter.lidar_type_func = lidar_type_func;
        filter.wheel_type_func = wheel_type_func;

        filter.freq = freq;
        
        filter.wheelG = wheelG;
        filter.imuG = imuG;
        filter.lidarG = lidarG;

        filter.alpha_lidar = alpha_lidar;

        filter.gamma_vx = gamma_vx;
        filter.gamma_omegaz = gamma_omegaz;
        filter.delta_vx = delta_vx;
        filter.delta_omegaz = delta_omegaz;

        filter.lidar_type_func  = lidar_type_func; 
        filter.wheel_type_func  = wheel_type_func; 
    }

    void ros_initialization(){
        // Subscriber
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/lidar_odom", 5, &adaptive_odom_filter::laserOdometryHandler, this);   
        subWheelOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 5, &adaptive_odom_filter::wheelOdometryHandler, this);
        subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, &adaptive_odom_filter::imuHandler, this);
        subLiDARFeatures = nh.subscribe<cloud_msgs::cloud_features>("/features_cloud_info", 5, &adaptive_odom_filter::laserFeaturesHandler, this); 
                    
        // Publisher
        pubFilteredOdometry = nh.advertise<nav_msgs::Odometry> ("/filter_odom", 5);
        pubLidarExtrinsic = nh.advertise<geometry_msgs::PoseStamped>("/lidar_extrinsic_pose", 5);
        pubImuExtrinsic = nh.advertise<geometry_msgs::PoseStamped>("/imu_extrinsic_pose", 5);

    }

    //------------------
    // Filter functions
    //------------------
    void filter_start(){
        ROS_INFO("\033[1;32mAdaptive Filter:\033[0m Filter Started.");
        filter.start();
    }

    void filter_stop(){
        filter.stop();
    }

    //----------
    // callbacks
    //----------
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn){
        double timeL = ros::Time::now().toSec();

        // time
        if (imuActivated){
            imuTimeLast = imuTimeCurrent;
            imuTimeCurrent = imuIn->header.stamp.toSec();
        }else{
            imuTimeCurrent = imuIn->header.stamp.toSec();
            imuTimeLast = imuTimeCurrent + 0.001;
            imuActivated = true;
        }       

        // roll, pitch and yaw 
        double roll, pitch, yaw;
        geometry_msgs::Quaternion orientation = imuIn->orientation;
        tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);

        // measure
        imuMeasure.block(0,0,3,1) << imuIn->angular_velocity.x, imuIn->angular_velocity.y, imuIn->angular_velocity.z;
        imuMeasure.block(3,0,3,1) << imuIn->linear_acceleration.x, imuIn->linear_acceleration.y, imuIn->linear_acceleration.z; 
        imuMeasure.block(6,0,3,1) << roll, pitch, yaw;

        // covariance
        E_imu.block(0,0,3,3) << imuIn->angular_velocity_covariance[0], imuIn->angular_velocity_covariance[1], imuIn->angular_velocity_covariance[2],
                                imuIn->angular_velocity_covariance[3], imuIn->angular_velocity_covariance[4], imuIn->angular_velocity_covariance[5],
                                imuIn->angular_velocity_covariance[6], imuIn->angular_velocity_covariance[7], imuIn->angular_velocity_covariance[8];
        E_imu.block(3,3,3,3) << imuIn->linear_acceleration_covariance[0], imuIn->linear_acceleration_covariance[1], imuIn->linear_acceleration_covariance[2],
                                imuIn->linear_acceleration_covariance[3], imuIn->linear_acceleration_covariance[4], imuIn->linear_acceleration_covariance[5],
                                imuIn->linear_acceleration_covariance[6], imuIn->linear_acceleration_covariance[7], imuIn->linear_acceleration_covariance[8];
        E_imu.block(6,6,3,3) << imuIn->orientation_covariance[0], imuIn->orientation_covariance[1], imuIn->orientation_covariance[2],
                                imuIn->orientation_covariance[3], imuIn->orientation_covariance[4], imuIn->orientation_covariance[5],
                                imuIn->orientation_covariance[6], imuIn->orientation_covariance[7], imuIn->orientation_covariance[8];

        E_imu.block(6,6,3,3) = imuG*E_imu.block(6,6,3,3);

        // time
        imu_dt = imuTimeCurrent - imuTimeLast;
        // imu_dt = 0.01;

        // header
        double timediff = ros::Time::now().toSec() - timeL + imuTimeCurrent;
        headerI = imuIn->header;
        headerI.stamp = ros::Time().fromSec(timediff);

        // correction stage aqui
        filter.correction_imu_data(imuMeasure, E_imu, imu_dt);

    }

    void wheelOdometryHandler(const nav_msgs::Odometry::ConstPtr& wheelOdometry){
        double timeL = ros::Time::now().toSec();

        // time
        if (wheelActivated){
            wheelTimeLast = wheelTimeCurrent;
            wheelTimeCurrent = wheelOdometry->header.stamp.toSec();
        }else{
            wheelTimeCurrent = wheelOdometry->header.stamp.toSec();
            wheelTimeLast = wheelTimeCurrent + 0.01;
            wheelActivated = true;
        } 

        // measure
        wheelMeasure << -1.0*wheelOdometry->twist.twist.linear.x, 0.0, wheelOdometry->twist.twist.angular.z;

        // covariance adaptativa
        // E_wheel(0,0) = wheelG*wheelOdometry->twist.covariance[0];
        // E_wheel(1,1) = wheelG*wheelOdometry->twist.covariance[7];
        // E_wheel(2,2) = wheelG*wheelOdometry->twist.covariance[35];
        // covariancia fixa
        E_wheel(0,0) = 0.1;
        E_wheel(1,1) = 0.01;
        E_wheel(2,2) = 0.2;


        // time
        wheel_dt = wheelTimeCurrent - wheelTimeLast;

        // header
        double timediff = ros::Time::now().toSec() - timeL + wheelTimeCurrent;
        headerW = wheelOdometry->header;
        headerW.stamp = ros::Time().fromSec(timediff);

        // correction stage aqui
        filter.correction_wheel_data(wheelMeasure, E_wheel, wheel_dt, imuMeasure(2));
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry){
        double timeL = ros::Time::now().toSec();

        if (lidarActivated){
            lidarTimeLast = lidarTimeCurrent;
            lidarTimeCurrent = laserOdometry->header.stamp.toSec();
        }else{
            lidarTimeCurrent = laserOdometry->header.stamp.toSec();
            lidarTimeLast = lidarTimeCurrent + 0.01;
            lidarActivated = true;
        }  
        
        // roll, pitch and yaw 
        double roll, pitch, yaw;
        geometry_msgs::Quaternion orientation = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);

        // measure
        lidarMeasure.block(0,0,3,1) << laserOdometry->pose.pose.position.x, laserOdometry->pose.pose.position.y, laserOdometry->pose.pose.position.z;
        lidarMeasure.block(3,0,3,1) << roll, pitch, yaw;    

        // covariance
        int k = 0;
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                // covariancia adaptativa
                // E_lidar(i, j) = laserOdometry->pose.covariance[k];
                // covariancia fixa
                E_lidar(i, j) = 0.9;
                k++;
            }
        }

        // for adaptive covariance - commig with features cloud info
        // double corner = double(laserOdometry->twist.twist.linear.x);
        // double surf = double(laserOdometry->twist.twist.angular.x);

        // time
        lidar_dt = lidarTimeCurrent - lidarTimeLast;

        // header
        double timediff = ros::Time::now().toSec() - timeL + lidarTimeCurrent;
        headerL = laserOdometry->header;
        headerL.stamp = ros::Time().fromSec(timediff);
        
        // correction stage aqui
        filter.correction_lidar_data(lidarMeasure, E_lidar, lidar_dt, lidarCorner, lidarSurf); 

        // get state here
        filter.get_state(X, P);
        
        // publish
        publish_odom('l');
    }

    void laserFeaturesHandler(const cloud_msgs::cloud_features::ConstPtr& laserFeatures){
        lidarCorner = laserFeatures->num_edge_points;
        lidarSurf = laserFeatures->num_plane_points;
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
            }
        
        filteredOdometry.header.frame_id = "chassis_init";
        filteredOdometry.child_frame_id = "ekf_odom_frame";

        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw (X(3), X(4), X(5));

        // pose
        filteredOdometry.pose.pose.orientation.x = geoQuat.x;
        filteredOdometry.pose.pose.orientation.y = geoQuat.y;
        filteredOdometry.pose.pose.orientation.z = geoQuat.z;
        filteredOdometry.pose.pose.orientation.w = geoQuat.w;
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

        pubFilteredOdometry.publish(filteredOdometry);

        // calibrations
        publish_extrinsics(filteredOdometry.header);
    }

    void publish_extrinsics(const std_msgs::Header& header){
        // --- LIDAR EXTRINSIC ---
        lidarExtrinsicPose.header = header;
        lidarExtrinsicPose.header.frame_id = "lidar"; // Frame do robô estimado

        lidarExtrinsicPose.pose.position.x = X(45);
        lidarExtrinsicPose.pose.position.y = X(46);
        lidarExtrinsicPose.pose.position.z = X(47);

        geometry_msgs::Quaternion qL = tf::createQuaternionMsgFromRollPitchYaw(X(48), X(49), X(50));
        lidarExtrinsicPose.pose.orientation = qL;

        pubLidarExtrinsic.publish(lidarExtrinsicPose);

        // --- IMU EXTRINSIC ---
        imuExtrinsicPose.header = header;
        imuExtrinsicPose.header.frame_id = "imu";

        imuExtrinsicPose.pose.position.x = X(51);
        imuExtrinsicPose.pose.position.y = X(52);
        imuExtrinsicPose.pose.position.z = X(53);

        geometry_msgs::Quaternion qI = tf::createQuaternionMsgFromRollPitchYaw(X(54), X(55), X(56));
        imuExtrinsicPose.pose.orientation = qI;

        pubImuExtrinsic.publish(imuExtrinsicPose);
    }

};


//-----------------------------
// Main 
//-----------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "adaptive_odom_filter");

    adaptive_odom_filter AF;

    //Parameters init:    
    ros::NodeHandle nh_;
    try
    {
        nh_.param("/adaptive_filter/enableFilter", AF.enableFilter, false);
        nh_.param("/adaptive_filter/enableImu", AF.enableImu, false);
        nh_.param("/adaptive_filter/enableWheel", AF.enableWheel, false);
        nh_.param("/adaptive_filter/enableLidar", AF.enableLidar, false);

        nh_.param("/adaptive_filter/filterFreq", AF.filterFreq, std::string("l"));
        nh_.param("/adaptive_filter/freq", AF.freq, double(200.0));

        nh_.param("/adaptive_filter/alpha_lidar", AF.alpha_lidar, double(0.98));

        nh_.param("/adaptive_filter/lidarG", AF.lidarG, float(1000));
        nh_.param("/adaptive_filter/wheelG", AF.wheelG, float(0.05));
        nh_.param("/adaptive_filter/imuG", AF.imuG, float(0.1));

        nh_.param("/adaptive_filter/gamma_vx", AF.gamma_vx, float(0.05));
        nh_.param("/adaptive_filter/gamma_omegaz", AF.gamma_omegaz, float(0.01));
        nh_.param("/adaptive_filter/delta_vx", AF.delta_vx, float(0.0001));
        nh_.param("/adaptive_filter/delta_omegaz", AF.delta_omegaz, float(0.00001));

        nh_.param("/adaptive_filter/lidar_type_func", AF.lidar_type_func, int(2));
        nh_.param("/adaptive_filter/wheel_type_func", AF.wheel_type_func, int(1));
    }
    catch (int e)
    {
        ROS_INFO("\033[1;31mAdaptive Filter:\033[0m Exception occurred when importing parameters in Adaptive Filter Node. Exception Nr. %d", e);
    }

    // AsyncSpinner
    ros::AsyncSpinner spinner(7);
    spinner.start();

    // ros initialization
    AF.ros_initialization();

    // filter initialaization
    AF.filter_initialization();

    if (AF.enableFilter){
        ROS_INFO("\033[1;32mAdaptive Filter:\033[0m Started.");
        // runs
        AF.filter_start();
    }else{
        AF.filter_stop();
        ROS_INFO("\033[1;32mAdaptive Filter: \033[0m Stopped.");
    }
    
    ros::waitForShutdown();
    return 0;
}

