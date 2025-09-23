//=====================================================EKF-Fast-LIO2=====================================================================
//Institutions: Federal University of Minas Gerais (UFMG), Federal University of Ouro Preto (UFOP) and Instituto Tecnológico Vale (ITV)
//Description: This file is responsible for merging the wheel odometry with the IMU data and the Fast-LIO2 odometry.
//Milestones: 
//
//             Date: September 22, 2025
//             Description: New version of the code including visual odometry measurement.
//             Members: Gilmar Pereira da Cruz Júnior and Gabriel Malaquias
//             E-mails: gilmarpcruzjunior@gmail.com, gmdeoliveira@ymail.com
//=======================================================================================================================================

#include "settings_adaptive_filter.h"
#include <rtabmap_msgs/ResetPose.h>


using namespace Eigen;
using namespace std;

//-----------------------------
// Global variables
//-----------------------------


std::mutex mtx; 

//-----------------------------
// Adaptive Covariacne class
//-----------------------------
class AdaptiveCov{
private:
    // ros node
    ros::NodeHandle nh;

    // Subscriber
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subPointCloud;
    ros::Subscriber subVisualOdometry;
    ros::Subscriber subVisualOdometryD;
    ros::Subscriber subCamLeft;
    ros::Subscriber subCamRight;
    ros::Subscriber subCamRgb;
    ros::Subscriber subWheelOdometry;
    ros::Subscriber subImu;

    // Publisher
    ros::Publisher pubLiDAROdometry;
    ros::Publisher pubVisualOdometry;
    ros::Publisher pubWheelOdometry;

    // services
    ros::ServiceClient srv_client_rgbd; 

    // adaptive covariance - lidar odometry
    double nCorner, nSurf; 
    double Gx, Gy, Gz, Gphi, Gtheta, Gpsi;
    double Gvx, Gvy, Gvz, Gvphi, Gvtheta, Gvpsi;
    float l_min;

    // adaptive covariance - visual odometry
    double averageIntensity1;
    double averageIntensity2;
    double averageIntensity;
    double minIntensity; // only to teste
    double maxIntensity; // only to teste

    bool first_rgbd;

    // imu measure
    Eigen::VectorXd imuMeasure;

public:
    bool enableFilter;
    bool enableImu;
    bool enableWheel;
    bool enableLidar;
    bool enableVisual;

    float lidarG;
    float visualG;
    float wheelG;
    float imuG;

    float gamma_vx;
    float gamma_omegaz;
    float delta_vx;
    float delta_omegaz;

    int lidar_type_func;
    int visual_type_func;
    int wheel_type_func;

    int camera_type;

    std::string filterFreq;

    AdaptiveCov():
        nh("~")
    {
        // intiatialization
        initialization();

        // Subscriber
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/ekf_loam/laser_odom_to_initOut", 5, &AdaptiveCov::laserOdometryHandler, this);
        subVisualOdometry = nh.subscribe<nav_msgs::Odometry>("/t265/odom/sample", 5, &AdaptiveCov::visualOdometryHandler, this);
        subVisualOdometryD = nh.subscribe<nav_msgs::Odometry>("/rtabmap/odom", 5, &AdaptiveCov::visualOdometryDHandler, this);
        subWheelOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 5, &AdaptiveCov::wheelOdometryHandler, this);

        subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, &AdaptiveCov::imuHandler, this);
        
        // subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/os1_cloud_node/points", 5, &AdaptiveCov::ptCloudHandler, this);
        subCamLeft = nh.subscribe<sensor_msgs::Image>("/t265/fisheye2/image_raw", 5, &AdaptiveCov::camLeftHandler, this);
        subCamRight = nh.subscribe<sensor_msgs::Image>("/t265/fisheye1/image_raw", 5, &AdaptiveCov::camRightHandler, this);
        subCamRgb = nh.subscribe<sensor_msgs::Image>("/d435i/color/image_raw", 5, &AdaptiveCov::camRgbHandler, this);

        // Publisher
        pubLiDAROdometry = nh.advertise<nav_msgs::Odometry> ("/ekf_loam/laser_odom_to_init_cov", 5);
        pubVisualOdometry = nh.advertise<nav_msgs::Odometry> ("/visual/odom/sample_cov", 5); // trocar nome e adicionar subscriber do rtab map
        pubWheelOdometry = nh.advertise<nav_msgs::Odometry> ("/odom_cov", 5);

        // Services
        srv_client_rgbd = nh.serviceClient<rtabmap_msgs::ResetPose>("/rtabmap/reset_odom_to_pose");

    }

    //------------------
    // Auxliar functions
    //------------------
    void initialization(){
        // parameters
        enableFilter = false;
        enableImu = false;
        enableWheel = false;
        enableLidar = false;
        enableVisual = false;
        filterFreq = 'l';

        lidar_type_func = 0;
        visual_type_func = 0;
        wheel_type_func = 0;

        camera_type = 1;

        lidarG = 1.0;
        visualG = 1.0;
        wheelG = 1.0;
        imuG = 1.0;

        // adptive covariance constants - lidar odometry
        nCorner = 500.0; // 7000
        nSurf = 5000;    // 5400
        
        Gz = 0.0048;    // x [m]
        Gx = 0.0022;    // y [m]
        Gy = 0.0016;    // z [m]
        Gpsi = 0.0044;  // phi [rad]
        Gphi = 0.0052;  // theta [rad]
        Gtheta = 0.005; // psi [rad]

        Gvz = 0.001;    // x [m]
        Gvx = 0.001;    // y [m]
        Gvy = 0.001;    // z [m]
        Gvpsi = 0.001;  // phi [rad]
        Gvphi = 0.001;  // theta [rad]
        Gvtheta = 0.001; // psi [rad]

        l_min = 0.01;

        // adaptive covariance - visual odometry
        averageIntensity1 = 0;
        averageIntensity2 = 0;
        averageIntensity = 0;
        minIntensity = 15.0;
        maxIntensity = 170.0;

        first_rgbd = true;

        imuMeasure.resize(9);
        imuMeasure = Eigen::VectorXd::Zero(9);
    }

    MatrixXd adaptive_covariance(double fCorner, double fSurf){
        Eigen::MatrixXd Q(6,6);
        double cov_x, cov_y, cov_z, cov_phi, cov_psi, cov_theta;
        
        // heuristic
        switch(lidar_type_func){
            case 0:
                cov_x     = lidarG*((nCorner - min(fCorner,nCorner))/nCorner + l_min);
                cov_y     = lidarG*((nCorner - min(fCorner,nCorner))/nCorner + l_min);
                cov_psi   = lidarG*((nCorner - min(fCorner,nCorner))/nCorner + l_min);
                cov_z     = lidarG*((nSurf - min(fSurf,nSurf))/nSurf + l_min);
                cov_phi   = lidarG*((nSurf - min(fSurf,nSurf))/nSurf + l_min);
                cov_theta = lidarG*((nSurf - min(fSurf,nSurf))/nSurf + l_min);
                break;
            case 1:
                cov_x     = lidarG*exp(-lidarG*(nCorner - min(fCorner,nCorner))/nCorner) + l_min;
                cov_y     = lidarG*exp(-lidarG*(nCorner - min(fCorner,nCorner))/nCorner) + l_min;
                cov_psi   = lidarG*exp(-lidarG*(nCorner - min(fCorner,nCorner))/nCorner) + l_min;
                cov_z     = lidarG*exp(-lidarG*(nSurf - min(fSurf,nSurf))/nSurf) + l_min;
                cov_phi   = lidarG*exp(-lidarG*(nSurf - min(fSurf,nSurf))/nSurf) + l_min;
                cov_theta = lidarG*exp(-lidarG*(nSurf - min(fSurf,nSurf))/nSurf) + l_min;
                break;
        }
        
        Q = MatrixXd::Zero(6,6);
        Q(0,0) = Gx*cov_x;
        Q(1,1) = Gy*cov_y;
        Q(2,2) = Gz*cov_z;
        Q(3,3) = Gphi*cov_phi;
        Q(4,4) = Gtheta*cov_theta;
        Q(5,5) = Gpsi*cov_psi;

        return Q;
    }

    MatrixXd adaptive_visual_covariance(double IntensityIn){
        Eigen::MatrixXd Q(6,6);
        double cov, Intensity;

        Intensity = (IntensityIn - minIntensity)/(maxIntensity - minIntensity);
        
        // heuristic
        switch(visual_type_func){
            case 0:
                cov = visualG*((1.0 - min(Intensity,1.0))/1.0 + l_min);
                break;
            case 1:
                cov = visualG*exp(-visualG*Intensity) + l_min;
                break;
        }
        
        Q = MatrixXd::Zero(6,6);
        Q(0,0) = Gvx*cov;
        Q(1,1) = Gvy*cov;
        Q(2,2) = Gvz*cov;
        Q(3,3) = Gvphi*cov;
        Q(4,4) = Gvtheta*cov;
        Q(5,5) = Gvpsi*cov;

        return Q;
    }

    MatrixXd wheelOdometryAdaptiveCovariance(double omegaz_wheel_odom, double omegaz_imu){
        Eigen::MatrixXd E(2,2);

        E(0,0) = gamma_vx * abs(omegaz_wheel_odom - omegaz_imu) + delta_vx;
        E(1,1) = gamma_omegaz * abs(omegaz_wheel_odom - omegaz_imu) + delta_omegaz;

        return E;
    }     

    //----------
    // callbacks
    //----------
    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry){
        if (enableFilter && enableLidar){
            // odometry message
            nav_msgs::Odometry lidarOdometryOut;
            lidarOdometryOut.header = laserOdometry->header;
            lidarOdometryOut.pose = laserOdometry->pose;
            lidarOdometryOut.twist = laserOdometry->twist;

            // covariance
            double corner = double(laserOdometry->twist.twist.linear.x);
            double surf = double(laserOdometry->twist.twist.angular.x);
            if (lidar_type_func==2){
                lidarOdometryOut.pose.covariance = laserOdometry->pose.covariance;
                lidarOdometryOut.twist.covariance = laserOdometry->twist.covariance;
            }else{
                Eigen::MatrixXd E_lidar(6,6);
                E_lidar = adaptive_covariance(corner, surf);
                
                // pose convariance
                int k = 0;
                for (int i = 0; i < 6; i++){
                    for (int j = 0; j < 6; j++){
                        lidarOdometryOut.pose.covariance[k] = E_lidar(i,j);
                        k++;
                    }
                }
            }

            pubLiDAROdometry.publish(lidarOdometryOut);
        }
    }

    void visualOdometryHandler(const nav_msgs::Odometry::ConstPtr& visualOdometry){
        if (enableFilter && enableVisual && camera_type == 1){            
            // odometry message
            nav_msgs::Odometry visualOdometryOut;
            visualOdometryOut.header = visualOdometry->header;
            visualOdometryOut.pose = visualOdometry->pose;
            visualOdometryOut.twist = visualOdometry->twist;

            if (visual_type_func==2){
                visualOdometryOut.pose.covariance = visualOdometry->pose.covariance;
                visualOdometryOut.twist.covariance = visualOdometry->twist.covariance;
            }else{
                averageIntensity = (averageIntensity1 + averageIntensity2)/2;            
                // averageIntensity = (averageIntensity - minIntensity)/(maxIntensity - minIntensity);

                Eigen::MatrixXd E_visual(6,6);
                E_visual = adaptive_visual_covariance(averageIntensity);

                
                // pose convariance
                int k = 0;
                for (int i = 0; i < 6; i++){
                    for (int j = 0; j < 6; j++){
                        visualOdometryOut.pose.covariance[k] = E_visual(i,j);
                        k++;
                    }
                }
            }

            pubVisualOdometry.publish(visualOdometryOut);

        }
    }

    void visualOdometryDHandler(const nav_msgs::Odometry::ConstPtr& visualOdometry){
        nav_msgs::Odometry visualOdometryOut;

        if (enableFilter && enableVisual && camera_type == 2){
            Eigen::MatrixXd E_visual(6,6);

            if (visualOdometry->pose.covariance[0] >= 9999.0 && visualOdometry->pose.pose.position.x == 0 && visualOdometry->pose.pose.position.y == 0 && visualOdometry->pose.pose.position.z == 0){
                // reset pose 
                rtabmap_msgs::ResetPose poseRgb;
                poseRgb.request.x = visualOdometryOut.pose.pose.position.x;
                poseRgb.request.y = visualOdometryOut.pose.pose.position.y;
                poseRgb.request.z = visualOdometryOut.pose.pose.position.z;

                double roll, pitch, yaw;
                geometry_msgs::Quaternion orientation = visualOdometryOut.pose.pose.orientation;
                tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);

                poseRgb.request.roll = roll;
                poseRgb.request.pitch = pitch;
                poseRgb.request.yaw = yaw;

                // call service
                srv_client_rgbd.call(poseRgb);
            }else{
                // odometry message
                visualOdometryOut.header = visualOdometry->header;
                visualOdometryOut.pose = visualOdometry->pose;
                visualOdometryOut.twist = visualOdometry->twist;

                if (visual_type_func==2){
                    visualOdometryOut.pose.covariance = visualOdometry->pose.covariance;
                    visualOdometryOut.twist.covariance = visualOdometry->twist.covariance;
                }else{
                    E_visual = adaptive_visual_covariance(averageIntensity);
                
                    // pose convariance
                    int k = 0;
                    for (int i = 0; i < 6; i++){
                        for (int j = 0; j < 6; j++){
                            visualOdometryOut.pose.covariance[k] = E_visual(i,j);
                            k++;
                        }
                    }

                    if (first_rgbd){
                        first_rgbd = false;
                    }else{
                        pubVisualOdometry.publish(visualOdometryOut);
                    }
                }
            }            
        }
    }

    void wheelOdometryHandler(const nav_msgs::Odometry::ConstPtr& wheelOdometry){
        nav_msgs::Odometry wheelOdometryOut;
        if (enableFilter && enableWheel){
            // odometry message
            nav_msgs::Odometry lidarOdometryOut;
            wheelOdometryOut.header = wheelOdometry->header;
            wheelOdometryOut.pose = wheelOdometry->pose;
            wheelOdometryOut.twist = wheelOdometry->twist;

            // covariance
            if (lidar_type_func==2){
                wheelOdometryOut.pose.covariance = wheelOdometry->pose.covariance;
                wheelOdometryOut.twist.covariance = wheelOdometry->twist.covariance;
            }else{
                Eigen::MatrixXd E_wheel(2,2);
                E_wheel = wheelOdometryAdaptiveCovariance(wheelOdometry->twist.twist.angular.z, imuMeasure(5));
                
                // pose convariance
                int k = 0;
                for (int i = 0; i < 2; i++){
                    for (int j = 0; j < 2; j++){
                        wheelOdometryOut.pose.covariance[k] = E_wheel(i,j);
                        k++;
                    }
                }
            }

            pubWheelOdometry.publish(wheelOdometryOut);
        }
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn){
        // measure
        imuMeasure.block(0,0,3,1) << imuIn->linear_acceleration.x, imuIn->linear_acceleration.y, imuIn->linear_acceleration.z;
        imuMeasure.block(3,0,3,1) << imuIn->angular_velocity.x, imuIn->angular_velocity.y, imuIn->angular_velocity.z; 
        // imuMeasure.block(6,0,3,1) << roll, pitch, yaw;
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

        // std::cout << "Intensity = " << averageIntensity << std::endl;
    }

};


//-----------------------------
// Main 
//-----------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "adaptive_covariance");

    AdaptiveCov AC;

    //Parameters init:    
    ros::NodeHandle nh_;
    try
    {
        nh_.param("/ekf_loam/enableFilter", AC.enableFilter, false);
        nh_.param("/adaptive_filter/enableImu", AC.enableImu, false);
        nh_.param("/adaptive_filter/enableWheel", AC.enableWheel, false);
        nh_.param("/adaptive_filter/enableLidar", AC.enableLidar, false);
        nh_.param("/adaptive_filter/enableVisual", AC.enableVisual, false);
        nh_.param("/adaptive_filter/filterFreq", AC.filterFreq, std::string("l"));

        nh_.param("/adaptive_filter/lidarG", AC.lidarG, float(1000));
        nh_.param("/adaptive_filter/wheelG", AC.wheelG, float(0.05));
        nh_.param("/adaptive_filter/visualG", AC.visualG, float(0.05));
        nh_.param("/adaptive_filter/imuG", AC.imuG, float(0.1));

        nh_.param("/adaptive_filter/gamma_vx", AC.gamma_vx, float(0.05));
        nh_.param("/adaptive_filter/gamma_omegaz", AC.gamma_omegaz, float(0.01));
        nh_.param("/adaptive_filter/delta_vx", AC.delta_vx, float(0.0001));
        nh_.param("/adaptive_filter/delta_omegaz", AC.delta_omegaz, float(0.00001));

        nh_.param("/adaptive_filter/lidar_type_func", AC.lidar_type_func, int(2));
        nh_.param("/adaptive_filter/visual_type_func", AC.visual_type_func, int(2));
        nh_.param("/adaptive_filter/wheel_type_func", AC.wheel_type_func, int(1));

        nh_.param("/adaptive_filter/camera_type", AC.camera_type, int(0));
    }
    catch (int e)
    {
        ROS_INFO("\033[1;31mAdaptive Covariance:\033[0m Exception occurred when importing parameters in Adaptive Covariance Node. Exception Nr. %d", e);
    }
    
    if (AC.enableFilter){
        ROS_INFO("\033[1;32mAdaptive Covariance:\033[0m Started.");
        // runs
        // AC.run();
    }else{
        ROS_INFO("\033[1;32mAdaptive Covariance:\033[0m Stopped.");
    }
    
    ros::spin();
    return 0;
}

