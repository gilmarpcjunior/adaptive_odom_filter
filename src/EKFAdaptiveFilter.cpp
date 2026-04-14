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
    ros::Subscriber subVisualOdometry;
    ros::Subscriber subVisualOdometryD;
    ros::Subscriber subCamLeft;
    ros::Subscriber subCamRight;
    ros::Subscriber subCamRgb;

    // Publisher
    ros::Publisher pubFilteredOdometry;

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

    // Imu orientation corretion -- erase this 
    Eigen::Matrix3d R_init;

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

        // wheelG = 0; // delete
        imuG = 0;

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

        // IMU orientation correction -- erase this
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

        // there are other parameters to set, i.e., the priori state with your covariance matrix
    }

    void ros_initialization(){
        // Subscriber
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/lidar_odom", 5, &adaptive_odom_filter::laserOdometryHandler, this);   
        subWheelOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 5, &adaptive_odom_filter::wheelOdometryHandler, this);
        subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, &adaptive_odom_filter::imuHandler, this);
        
        if (camera_type==1){
            subVisualOdometry = nh.subscribe<nav_msgs::Odometry>("/tracking_odom", 5, &adaptive_odom_filter::visualOdometryHandler, this);
            subCamLeft = nh.subscribe<sensor_msgs::Image>("/left_camera", 5, &adaptive_odom_filter::camLeftHandler, this);
            subCamRight = nh.subscribe<sensor_msgs::Image>("/rigth_camera", 5, &adaptive_odom_filter::camRightHandler, this);
        }else if (camera_type==2){
            subVisualOdometryD = nh.subscribe<nav_msgs::Odometry>("/depth_odom", 5, &adaptive_odom_filter::visualOdometryDHandler, this);
            subCamRgb = nh.subscribe<sensor_msgs::Image>("/camera_color", 5, &adaptive_odom_filter::camRgbHandler, this);
        }
            
        // Publisher
        pubFilteredOdometry = nh.advertise<nav_msgs::Odometry> ("/filter_odom", 5);

        // Services
        srv_client_rgbd = nh.serviceClient<rtabmap_msgs::ResetPose>("/rtabmap/reset_odom_to_pose");
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

        // roll, pitch and yaw 
        double roll, pitch, yaw;
        geometry_msgs::Quaternion orientation = imuIn->orientation;
        tf::Matrix3x3 m(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
        // m.getRPY(roll, pitch, yaw);
        
        // Para converter para Eigen::Matrix3d:
        Eigen::Matrix3d R_curr, R_new;
        R_curr << m[0][0], m[0][1], m[0][2],
                m[1][0], m[1][1], m[1][2],
                m[2][0], m[2][1], m[2][2];

        // first matrix -- erase this
        // time
        if (imuActivated){
            imuTimeLast = imuTimeCurrent;
            imuTimeCurrent = imuIn->header.stamp.toSec();
        }else{
            imuTimeCurrent = imuIn->header.stamp.toSec();
            imuTimeLast = imuTimeCurrent + 0.001;
            imuActivated = true;
            R_init = R_curr;
        } 

        R_new = R_init.transpose()*R_curr;
        // 2. Criar uma matriz tf::Matrix3x3 temporária
        tf::Matrix3x3 m_temp;

        // 3. Converter os dados do Eigen para o TF
        // Note que o Eigen usa R(linha, coluna)
        m_temp.setValue(R_new(0,0), R_new(0,1), R_new(0,2),
                        R_new(1,0), R_new(1,1), R_new(1,2),
                        R_new(2,0), R_new(2,1), R_new(2,2));

        m_temp.getRPY(roll, pitch, yaw);

        // time
        // if (imuActivated){
        //     imuTimeLast = imuTimeCurrent;
        //     imuTimeCurrent = imuIn->header.stamp.toSec();
        // }else{
        //     imuTimeCurrent = imuIn->header.stamp.toSec();
        //     imuTimeLast = imuTimeCurrent + 0.001;
        //     imuActivated = true;
        // }       

        // roll, pitch and yaw 
        // double roll, pitch, yaw;
        // geometry_msgs::Quaternion orientation = imuIn->orientation;
        // tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);

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
        wheelMeasure << -0.705*wheelOdometry->twist.twist.linear.x, wheelOdometry->twist.twist.linear.y, wheelOdometry->twist.twist.angular.z;

        // wheel covariance
        switch (experiment) {
            case 1:
                // equal
                E_wheel(0,0) = 0.02; 
                E_wheel(1,1) = 1e-4;
                E_wheel(2,2) = 0.05;
                break;
            case 2:
                // wheel > LiDAR
                E_wheel(0,0) = 0.02; 
                E_wheel(1,1) = 0.01;
                E_wheel(2,2) = 0.05;
                break;
            case 3:
                // wheel < LiDAR
                E_wheel(0,0) = 0.00001; 
                E_wheel(1,1) = 1e-4;
                E_wheel(2,2) = 0.005;
                break;
            default:
                // code to execute if no cases match
                E_wheel(0,0) = wheelGVx*wheelOdometry->twist.covariance[0] + wheelOffset; 
                E_wheel(1,1) = wheelGVy*wheelOdometry->twist.covariance[7];
                E_wheel(2,2) = wheelGWz*wheelOdometry->twist.covariance[35];
        }      

        // time
        wheel_dt = wheelTimeCurrent - wheelTimeLast;

        // header
        double timediff = ros::Time::now().toSec() - timeL + wheelTimeCurrent;
        headerW = wheelOdometry->header;
        headerW.stamp = ros::Time().fromSec(timediff);

        // correction stage aqui
        filter.correction_wheel_data(wheelMeasure, E_wheel, wheel_dt, imuMeasure(5));
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

        switch (experiment) {
            case 1:
                // equal
                E_lidar(0,0) = 0.02;  // vx
                E_lidar(1,1) = 1e-4;  // vy            
                E_lidar(2,2) = 0.02;
                E_lidar(3,3) = 1e-5;
                E_lidar(4,4) = 1e-5;
                E_lidar(5,5) = 0.05;  // wz
                break;
            case 2:
                // wheel > LiDAR
                E_lidar(0,0) = 0.00001;  // vx
                E_lidar(1,1) = 1e-4;  // vy            
                E_lidar(2,2) = 0.02;
                E_lidar(3,3) = 1e-5;
                E_lidar(4,4) = 1e-5;
                E_lidar(5,5) = 0.005;  // wz
                break;
            case 3:
                // wheel < LiDAR
                E_lidar(0,0) = 0.02;  // vx
                E_lidar(1,1) = 0.01;  // vy            
                E_lidar(2,2) = 0.02;
                E_lidar(3,3) = 1e-5;
                E_lidar(4,4) = 1e-5;
                E_lidar(5,5) = 0.05;  // wz
                break;
            default:
                // code to execute if no cases match
                E_lidar(0,0) = laserOdometry->twist.covariance[0];
                E_lidar(1,1) = laserOdometry->twist.covariance[7];
                E_lidar(2,2) = 40*laserOdometry->twist.covariance[14] + 30;
                E_lidar(3,3) = 40*laserOdometry->twist.covariance[21] + 30;
                E_lidar(4,4) = 40*laserOdometry->twist.covariance[28] + 30;
                E_lidar(5,5) = laserOdometry->twist.covariance[35];
        }

        // for adaptive covariance
        double corner = double(laserOdometry->twist.twist.linear.x);
        double surf = double(laserOdometry->twist.twist.angular.x);

        // time
        lidar_dt = lidarTimeCurrent - lidarTimeLast;

        // header
        double timediff = ros::Time::now().toSec() - timeL + lidarTimeCurrent;
        headerL = laserOdometry->header;
        headerL.stamp = ros::Time().fromSec(timediff);
        
        // correction stage aqui
        filter.correction_lidar_data(lidarMeasure, E_lidar, lidar_dt, corner, surf); // parei aqui. adicionar flag para publicação??

        // get state here
        filter.get_state(X, P);

        // publish
        publish_odom('l');
    }

    void visualOdometryHandler(const nav_msgs::Odometry::ConstPtr& visualOdometry){
        if (camera_type == 1){ 
            double timeV = ros::Time::now().toSec();

            if (visualActivated){
                visualTimeLast = visualTimeCurrent;
                visualTimeCurrent = visualOdometry->header.stamp.toSec();
            }else{
                visualTimeCurrent = visualOdometry->header.stamp.toSec();
                visualTimeLast = visualTimeCurrent + 0.01;
                visualActivated = true;
            }  
            
            // roll, pitch and yaw 
            double roll, pitch, yaw;
            geometry_msgs::Quaternion orientation = visualOdometry->pose.pose.orientation;
            tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);

            visualMeasure.block(0,0,3,1) << visualOdometry->pose.pose.position.x, visualOdometry->pose.pose.position.y, visualOdometry->pose.pose.position.z;
            visualMeasure.block(3,0,3,1) << roll, pitch, yaw;    

            // covariance
            E_visual(0,0) = visualOdometry->twist.covariance[0];
            E_visual(1,1) = visualOdometry->twist.covariance[7];
            E_visual(2,2) = visualOdometry->twist.covariance[14];
            E_visual(3,3) = visualOdometry->twist.covariance[21];
            E_visual(4,4) = visualOdometry->twist.covariance[28];
            E_visual(5,5) = visualOdometry->twist.covariance[35];

            // time
            visual_dt = visualTimeCurrent - visualTimeLast;

            // header
            double timediff = ros::Time::now().toSec() - timeV + visualTimeCurrent;
            headerV = visualOdometry->header;
            headerV.stamp = ros::Time().fromSec(timediff);
            
            // compute average intensity
            averageIntensity = (averageIntensity1 + averageIntensity2)/2;

            //New measure
            filter.correction_visual_data(visualMeasure, E_visual, visual_dt, averageIntensity);
        }
    }

    void visualOdometryDHandler(const nav_msgs::Odometry::ConstPtr& visualOdometry){
        if (enableFilter && enableVisual && camera_type == 2){
            Eigen::MatrixXd E_visual(6,6);

            if (visualOdometry->pose.covariance[0] >= 9999.0 && visualOdometry->pose.pose.position.x == 0 && visualOdometry->pose.pose.position.y == 0 && visualOdometry->pose.pose.position.z == 0){
                // reset pose 
                rtabmap_msgs::ResetPose poseRgb;
                poseRgb.request.x = visualOdometry->pose.pose.position.x;
                poseRgb.request.y = visualOdometry->pose.pose.position.y;
                poseRgb.request.z = visualOdometry->pose.pose.position.z;

                double roll, pitch, yaw;
                geometry_msgs::Quaternion orientation = visualOdometry->pose.pose.orientation;
                tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);

                poseRgb.request.roll = roll;
                poseRgb.request.pitch = pitch;
                poseRgb.request.yaw = yaw;

                // call service
                srv_client_rgbd.call(poseRgb);
            }else{
                double timeV = ros::Time::now().toSec();

                if (visualActivated){
                    visualTimeLast = visualTimeCurrent;
                    visualTimeCurrent = visualOdometry->header.stamp.toSec();
                }else{
                    visualTimeCurrent = visualOdometry->header.stamp.toSec();
                    visualTimeLast = visualTimeCurrent + 0.01;
                    visualActivated = true;
                }  
                
                // roll, pitch and yaw 
                double roll, pitch, yaw;
                geometry_msgs::Quaternion orientation = visualOdometry->pose.pose.orientation;
                tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);

                visualMeasure.block(0,0,3,1) << visualOdometry->pose.pose.position.x, visualOdometry->pose.pose.position.y, visualOdometry->pose.pose.position.z;
                visualMeasure.block(3,0,3,1) << roll, pitch, yaw;    

                // covariance
                E_visual(0,0) = visualOdometry->twist.covariance[0];
                E_visual(1,1) = visualOdometry->twist.covariance[7];
                E_visual(2,2) = visualOdometry->twist.covariance[14];
                E_visual(3,3) = visualOdometry->twist.covariance[21];
                E_visual(4,4) = visualOdometry->twist.covariance[28];
                E_visual(5,5) = visualOdometry->twist.covariance[35];

                // time
                visual_dt = visualTimeCurrent - visualTimeLast;
                // visual_dt = 0.05;

                // header
                double timediff = ros::Time::now().toSec() - timeV + visualTimeCurrent;
                headerV = visualOdometry->header;
                headerV.stamp = ros::Time().fromSec(timediff);
                
                //New measure
                filter.correction_visual_data(visualMeasure, E_visual, visual_dt, averageIntensity);
            }            
        }
    }

    void camLeftHandler(const sensor_msgs::ImageConstPtr& camIn){
        int width = camIn->width;
        int height = camIn->height;

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

    void camRightHandler(const sensor_msgs::ImageConstPtr& camIn){
        int width = camIn->width;
        int height = camIn->height;

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

    void camRgbHandler(const sensor_msgs::ImageConstPtr& camIn){
        int width = camIn->width;
        int height = camIn->height;
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
        nh_.param("/adaptive_filter/enableVisual", AF.enableVisual, false);

        nh_.param("/adaptive_filter/filterFreq", AF.filterFreq, std::string("l"));
        nh_.param("/adaptive_filter/freq", AF.freq, double(200.0));

        // nh_.param("/adaptive_filter/wheelG", AF.wheelG, float(0.05));
        nh_.param("/adaptive_filter/imuG", AF.imuG, float(0.1));

        nh_.param("/adaptive_filter/alpha_lidar", AF.alpha_lidar, double(0.98));
        nh_.param("/adaptive_filter/alpha_visual", AF.alpha_visual, double(0.98));

        nh_.param("/adaptive_filter/lidarG", AF.lidarG, float(1000));
        nh_.param("/adaptive_filter/wheelGVx", AF.wheelGVx, float(1.0));
        nh_.param("/adaptive_filter/wheelGVy", AF.wheelGVy, float(1.0));
        nh_.param("/adaptive_filter/wheelGWz", AF.wheelGWz, float(1.0));
        nh_.param("/adaptive_filter/wheelOffset", AF.wheelOffset, float(0.0));
        nh_.param("/adaptive_filter/visualG", AF.visualG, float(0.05));
        nh_.param("/adaptive_filter/imuG", AF.imuG, float(0.1));

        nh_.param("/adaptive_filter/experiment", AF.experiment, int(0));

        nh_.param("/adaptive_filter/gamma_vx", AF.gamma_vx, float(0.05));
        nh_.param("/adaptive_filter/gamma_omegaz", AF.gamma_omegaz, float(0.01));
        nh_.param("/adaptive_filter/delta_vx", AF.delta_vx, float(0.0001));
        nh_.param("/adaptive_filter/delta_omegaz", AF.delta_omegaz, float(0.00001));

        nh_.param("/adaptive_filter/minIntensity", AF.minIntensity, double(0.0));
        nh_.param("/adaptive_filter/maxIntensity", AF.maxIntensity, double(1.0));

        nh_.param("/adaptive_filter/lidar_type_func", AF.lidar_type_func, int(2));
        nh_.param("/adaptive_filter/visual_type_func", AF.visual_type_func, int(2));
        nh_.param("/adaptive_filter/wheel_type_func", AF.wheel_type_func, int(1));

        nh_.param("/adaptive_filter/camera_type", AF.camera_type, int(0));
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

