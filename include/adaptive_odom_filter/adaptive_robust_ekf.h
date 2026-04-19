//=====================================================EKF-Fast-LIO2=====================================================================
//Institutions: Federal University of Minas Gerais (UFMG), Federal University of Ouro Preto (UFOP) and Instituto Tecnológico Vale (ITV)
//Description: This file is responsible for merging the wheel odometry with the IMU data and the Fast-LIO2 odometry.
//Milestones: 
//
//             Date: Febraury 24, 2026
//             Description: New version of the code.
//             Members: Gilmar Pereira da Cruz Júnior and Gabriel Malaquias
//             E-mails: gilmarpcruzjunior@gmail.com, gmdeoliveira@ymail.com
//=======================================================================================================================================

#include "settings_adaptive_filter.h"
#include <adaptive_odom_filter/alphaBetaFilter.h>

using namespace Eigen;
using namespace std;

//-----------------------------
// Adaptive EKF class
//-----------------------------
class AdaptiveRobustEKF{

private:
    // Measure
    Eigen::VectorXd _imuMeasure, _wheelMeasure, _lidarMeasure, _lidarMeasureL;

    // Measure Covariance
    Eigen::MatrixXd _E_imu, _E_wheel, _E_lidar, _E_lidarL, _E_pred;

    // States and covariances
    Eigen::VectorXd _X, _V;
    Eigen::MatrixXd _P, _PV;

    // pose and velocities
    Eigen::VectorXd _pose, _velocities;

    // Times
    double _imu_dt;
    double _wheel_dt;
    double _lidar_dt;

    double _eps;
    double _epsR;

    // number of state or measure vectors
    // x = [pos[3] ori[3] vel[3] acc[3] ang[3] biasL[6] biasW[3] biasI[6] pose_lidar_robo[6] pose_imu_robo[6]]
    int _N_STATES = 42; //42 or 57
    int _N_IMU = 6; 
    int _N_WHEEL = 3; 
    int _N_LIDAR = 6;

    // alphabetaFiltr
    alphaBetaFilter _velocityFilter;
    
    // boolean
    bool _imuActivated;
    bool _wheelActivated;
    bool _lidarActivated;
    bool _imuNew;
    bool _wheelNew;
    bool _lidarNew;
    bool _velComp;
    bool _firstLidar;
    bool _computing;

    // variáveis de controle
    bool _running = false;
    std::thread _thread;
    std::mutex _mutex;

    //------------------
    // Auxliar functions
    //------------------
    void allocateMemory();

    void initialization();

    void covariance_initialization();

    inline double angularResidual(double a);

    inline Eigen::Vector3d angularResidualVec(const Eigen::Vector3d& a);

    inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);

    inline Eigen::Matrix3d eulerMatrix(const Eigen::Vector3d& a);

    inline Eigen::Vector3d leverArmEffect(const Eigen::Vector3d& omega, const Eigen::Vector3d& p_offset);

    inline Eigen::Vector3d dRv_dphi(double phi, double theta, double psi, const Eigen::Vector3d& v);

    inline Eigen::Vector3d dRv_dtheta(double phi, double theta, double psi, const Eigen::Vector3d& v);

    inline Eigen::Vector3d dRv_dpsi(double phi, double theta, double psi, const Eigen::Vector3d& v);

    inline Eigen::Matrix3d eulerKinematicMatrix(double phi, double theta);

    inline Eigen::Vector3d dJw_dphi(double phi, double theta, const Eigen::Vector3d& w);

    inline Eigen::Vector3d dJw_dtheta(double phi, double theta, const Eigen::Vector3d& w);

    Eigen::Matrix3d dR_dphi(double phi, double theta, double psi);

    Eigen::Matrix3d dR_dtheta(double phi, double theta, double psi);
    
    Eigen::Matrix3d dR_dpsi(double phi, double theta, double psi);

    //-----------------
    // predict function
    //-----------------
    void prediction_stage(double dt);

    //-----------------
    // correction stage
    //-----------------
    void correction_wheel_stage(double dt);

    void correction_imu_stage(double dt);

    void correction_lidar_stage(double dt);

    void correction_lidar_stage_old(double dt);

    //---------
    // Models
    //---------
    VectorXd f_prediction_model(VectorXd x, double dt);

    VectorXd indirect_odometry_measurement(VectorXd u, VectorXd ul, double dt);

    VectorXd indirect_odometry_measurement_new(VectorXd u, VectorXd ul, double dt);

    //----------
    // Jacobians
    //----------
    MatrixXd jacobian_state(VectorXd x, double dt);

    MatrixXd jacobian_odometry_measurement(VectorXd u, VectorXd ul, double dt);

    MatrixXd jacobian_odometry_measurementL(VectorXd u, VectorXd ul, double dt);

    Eigen::MatrixXd analitycal_jacobin_state(const Eigen::VectorXd& x, double dt);

    Eigen::MatrixXd calculateJacobianIMU(const Eigen::VectorXd& x);

    Eigen::MatrixXd calculateJacobianLidar(const Eigen::VectorXd& x);

    //----------------
    // run
    //----------------
    void run();

    //---------------------------
    // covariances functions
    //---------------------------
    MatrixXd adaptive_covariance(double fCorner, double fSurf);

    MatrixXd wheelOdometryAdaptiveCovariance(double omegaz_wheel_odom, double omegaz_imu);


public:
    //------------------
    // Filter settings
    //------------------
    bool enableImu;
    bool enableWheel;
    bool enableLidar;
    double freq;

    float alpha_lidar;

    //------------------
    // Covariance settings
    //------------------
    float lidarG;
    float wheelG;
    float imuG;

    float gamma_vx;
    float gamma_omegaz;
    float delta_vx;
    float delta_omegaz;

    int lidar_type_func;
    int wheel_type_func;

    // adaptive covariance - lidar odometry
    double nCorner, nSurf; 
    double Gx, Gy, Gz, Gphi, Gtheta, Gpsi;
    double Gvx, Gvy, Gvz, Gvphi, Gvtheta, Gvpsi;
    float l_min;

    double minIntensity; 
    double maxIntensity; 

    //------------------
    // Constructor -  Destructor
    //------------------   
    AdaptiveRobustEKF();
    ~AdaptiveRobustEKF();
    
    //----------
    // Datas - ok
    //----------
    void correction_imu_data(VectorXd imu, MatrixXd E_imu, double dt);

    void correction_wheel_data(VectorXd wheel_odom, MatrixXd E_wheel, double dt, double omegaz_imu);

    void correction_lidar_data(VectorXd lidar_odom, MatrixXd E_lidar, double dt, double corner, double surf);

    //------------------
    // filter control
    // -----------------
    void get_state(VectorXd &X_state, MatrixXd &E_state);

    void set_initial_state(VectorXd X_state, MatrixXd E_state);

    void start();     // inicia a thread

    void stop();      // para a thread

};
