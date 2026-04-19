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

// #include "ekf_adaptive_tools.h"
#include <adaptive_odom_filter/adaptive_robust_ekf.h>
#include <adaptive_odom_filter/alphaBetaFilter.h>
#include <rclcpp/rclcpp.hpp>

using namespace Eigen;
using namespace std;

//------------------
// Constructor - Destructor
//------------------   
AdaptiveRobustEKF::AdaptiveRobustEKF(){
    // alocate memory
    allocateMemory();

    // initialization
    initialization();
    covariance_initialization();
}

AdaptiveRobustEKF::~AdaptiveRobustEKF() {
    stop();  
}

//**************************
// PRIVATE FUNCTIONS
//**************************
//------------------
// Auxliar functions
//------------------
void AdaptiveRobustEKF::allocateMemory(){
    _imuMeasure.resize(_N_IMU);
    _wheelMeasure.resize(_N_WHEEL);
    _lidarMeasure.resize(_N_LIDAR);
    _lidarMeasureL.resize(_N_LIDAR);

    _E_imu.resize(_N_IMU,_N_IMU);
    _E_wheel.resize(_N_WHEEL,_N_WHEEL);
    _E_lidar.resize(_N_LIDAR,_N_LIDAR);
    _E_lidarL.resize(_N_LIDAR,_N_LIDAR);
    _E_pred.resize(_N_STATES,_N_STATES);

    _X.resize(_N_STATES);
    _P.resize(_N_STATES,_N_STATES);

    _V.resize(_N_STATES);
    _PV.resize(_N_STATES,_N_STATES);
}

void AdaptiveRobustEKF::initialization(){
    // times and frequencies
    _imu_dt = 0.0005;
    _wheel_dt = 0.05;
    _lidar_dt = 0.1;

    _eps = 1e-9;
    _epsR = 1e-4;

    freq = 200;

    // indirect measurement
    alpha_lidar = 0.99;

    // boolean
    _imuActivated = false;
    _lidarActivated = false;
    _wheelActivated = false;
    _imuNew = false;
    _wheelNew = false;
    _lidarNew = false;
    _firstLidar = true;  

    enableImu = false;
    enableWheel = false;
    enableLidar = false;

    _computing = false;

    // matrices and vectors
    _imuMeasure = Eigen::VectorXd::Zero(_N_IMU);
    _wheelMeasure = Eigen::VectorXd::Zero(_N_WHEEL);
    _lidarMeasure = Eigen::VectorXd::Zero(_N_LIDAR);
    _lidarMeasureL = Eigen::VectorXd::Zero(_N_LIDAR);
    
    _E_imu = Eigen::MatrixXd::Zero(_N_IMU,_N_IMU);
    _E_lidar = Eigen::MatrixXd::Zero(_N_LIDAR,_N_LIDAR);
    _E_lidarL = Eigen::MatrixXd::Zero(_N_LIDAR,_N_LIDAR);
    _E_wheel = Eigen::MatrixXd::Zero(_N_WHEEL,_N_WHEEL);
    _E_pred = Eigen::MatrixXd::Identity(_N_STATES,_N_STATES)*1e-10;

    // state initial
    _X = Eigen::VectorXd::Zero(_N_STATES);
    _P = Eigen::MatrixXd::Identity(_N_STATES,_N_STATES) * 1e-10;
    _V = Eigen::VectorXd::Zero(_N_STATES);

    // 2. Pose (Posição 0-2 e Orientação 3-5)
    _P.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * 1e-8; 
    _P.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * 1e-8;

    // 3. Velocidades (Linear 6-8 e Angular 9-11)
    // Este é o "core" do seu filtro. Damos um pouco mais de incerteza inicial.
    _P.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * 1e-5; 
    _P.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * 1e-5;

    // 4. Aceleração Linear (12-14)
    _P.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * 1e-8;

    // initial covariance - bias and calibration block
    _P.block(15,15,15,15) = Eigen::MatrixXd::Identity(15,15) * 1e-10;
    _P.block(30,30,12,12) = Eigen::MatrixXd::Identity(12,12) * 1e-10;

    // vector initialization
    // x = [pos[0-2] ori[3-5] vel[6-8] ang[9-11] acc[12-14] biasL[15-20] biasW[21-23] biasI[24-29] pose_lidar_robo[30-35] pose_imu_robo[36-41]]
    // biasL[15-20] = vx, vy, vz, wx, wy, wz,
    _X.segment(15, 6).setConstant(1e-9);
    // biasW[21-23] = vx, vy, wz
    _X.segment(21, 3).setConstant(1e-9);
    // biasI[24-29] = wx, wy, wz, ax, ay, az
    _X.segment(24, 6).setConstant(1e-9);
    // pose_lidar_robo[30-35]
    _X[30] = 0.0;
    _X[31] = 0.0;
    _X[32] = 0.0;
    _X[33] = 0.0;
    _X[34] = 0.0;
    _X[35] = 0.0;
    // pose_imu_robo[36-41]] - simulation data
    _X[36] = 0.0;
    _X[37] = 0.0;
    _X[38] = 0.0085; // 0.0085
    _X[39] = 0.0;
    _X[40] = 0.0;
    _X[41] = 0.0;

    // out data
    _V = _X;
    _PV = _P;

    // alphaBetaFilter
    _velocityFilter.init(_N_LIDAR, 0.8, 0.2);
}

void AdaptiveRobustEKF::covariance_initialization(){
    // parameters
    lidar_type_func = 0;
    wheel_type_func = 0;

    lidarG = 1.0;
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
}

inline double AdaptiveRobustEKF::angularResidual(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

inline Eigen::Vector3d AdaptiveRobustEKF::angularResidualVec(const Eigen::Vector3d& a) {
    Eigen::Vector3d b;

    b(0) = angularResidual(a(0));
    b(1) = angularResidual(a(1));
    b(2) = angularResidual(a(2));

    return b;
}

inline Eigen::Matrix3d AdaptiveRobustEKF::skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return m;
}

inline Eigen::Matrix3d AdaptiveRobustEKF::eulerMatrix(const Eigen::Vector3d& a){
    Eigen::Matrix3d R, Rx, Ry, Rz;

    Rx = Eigen::AngleAxisd(a(0), Eigen::Vector3d::UnitX());
    Ry = Eigen::AngleAxisd(a(1), Eigen::Vector3d::UnitY());
    Rz = Eigen::AngleAxisd(a(2), Eigen::Vector3d::UnitZ());
    R = Rz*Ry*Rx;

    return R;
}

inline Eigen::Vector3d AdaptiveRobustEKF::leverArmEffect(const Eigen::Vector3d& omega, const Eigen::Vector3d& p_offset) {
    // Implementação direta via produto vetorial (mais rápida para vetores 3x1)
    return omega.cross(omega.cross(p_offset));
}

inline Eigen::Vector3d AdaptiveRobustEKF::dRv_dphi(double phi, double theta, double psi, const Eigen::Vector3d& v) {
    double s_ph = sin(phi), c_ph = cos(phi);
    double s_th = sin(theta), c_th = cos(theta);
    double s_ps = sin(psi), c_ps = cos(psi);

    Eigen::Matrix3d dR;
    dR << 0, (s_ps*s_ph + c_ps*s_th*c_ph), (s_ps*c_ph - c_ps*s_th*s_ph),
          0, (-c_ps*s_ph + s_ps*s_th*c_ph), (-c_ps*c_ph - s_ps*s_th*s_ph),
          0, (c_th*c_ph), (-c_th*s_ph);
    return dR * v;
}

inline Eigen::Vector3d AdaptiveRobustEKF::dRv_dtheta(double phi, double theta, double psi, const Eigen::Vector3d& v) {
    double s_ph = sin(phi), c_ph = cos(phi);
    double s_th = sin(theta), c_th = cos(theta);
    double s_ps = sin(psi), c_ps = cos(psi);

    Eigen::Matrix3d dR;
    dR << (-c_ps*s_th), (c_ps*c_th*s_ph), (c_ps*c_th*c_ph),
          (-s_ps*s_th), (s_ps*c_th*s_ph), (s_ps*c_th*c_ph),
          (-c_th), (-s_th*s_ph), (-s_th*c_ph);
    return dR * v;
}

inline Eigen::Vector3d AdaptiveRobustEKF::dRv_dpsi(double phi, double theta, double psi, const Eigen::Vector3d& v) {
    double s_ph = sin(phi), c_ph = cos(phi);
    double s_th = sin(theta), c_th = cos(theta);
    double s_ps = sin(psi), c_ps = cos(psi);

    Eigen::Matrix3d dR;
    dR << (-s_ps*c_th), (-s_ps*s_th*s_ph - c_ps*c_ph), (-s_ps*s_th*c_ph + c_ps*s_ph),
          (c_ps*c_th), (c_ps*s_th*s_ph - s_ps*c_ph), (c_ps*s_th*c_ph + s_ps*s_ph),
          0, 0, 0;
    return dR * v;
}

inline Eigen::Matrix3d AdaptiveRobustEKF::eulerKinematicMatrix(double phi, double theta) {
    double s_ph = sin(phi), c_ph = cos(phi);
    double t_th = tan(theta), sec_th = 1.0/cos(theta);

    Eigen::Matrix3d J;
    J << 1, s_ph*t_th, c_ph*t_th,
         0, c_ph, -s_ph,
         0, s_ph*sec_th, c_ph*sec_th;
    return J;
}

inline Eigen::Vector3d AdaptiveRobustEKF::dJw_dphi(double phi, double theta, const Eigen::Vector3d& w) {
    double s_ph = sin(phi), c_ph = cos(phi);
    double t_th = tan(theta), sec_th = 1.0/cos(theta);
    
    Eigen::Vector3d res;
    res(0) = (c_ph*t_th)*w(1) - (s_ph*t_th)*w(2);
    res(1) = (-s_ph)*w(1) - (c_ph)*w(2);
    res(2) = (c_ph*sec_th)*w(1) - (s_ph*sec_th)*w(2);
    return res;
}

inline Eigen::Vector3d AdaptiveRobustEKF::dJw_dtheta(double phi, double theta, const Eigen::Vector3d& w) {
    double s_ph = sin(phi), c_ph = cos(phi);
    double sec_th = 1.0/cos(theta);
    double sec2_th = sec_th * sec_th;
    
    Eigen::Vector3d res;
    res(0) = (s_ph*sec2_th)*w(1) + (c_ph*sec2_th)*w(2);
    res(1) = 0;
    res(2) = (s_ph*sec_th*tan(theta))*w(1) + (c_ph*sec_th*tan(theta))*w(2);
    return res;
}

inline Eigen::Matrix3d AdaptiveRobustEKF::dR_dphi(double phi, double theta, double psi) {
    double s_ph = sin(phi), c_ph = cos(phi);
    double s_th = sin(theta), c_th = cos(theta);
    double s_ps = sin(psi), c_ps = cos(psi);

    Eigen::Matrix3d dR;
    dR << 0, (s_ps*s_ph + c_ps*s_th*c_ph), (s_ps*c_ph - c_ps*s_th*s_ph),
          0, (-c_ps*s_ph + s_ps*s_th*c_ph), (-c_ps*c_ph - s_ps*s_th*s_ph),
          0, (c_th*c_ph), (-c_th*s_ph);
    return dR;
}

inline Eigen::Matrix3d AdaptiveRobustEKF::dR_dtheta(double phi, double theta, double psi) {
    double s_ph = sin(phi), c_ph = cos(phi);
    double s_th = sin(theta), c_th = cos(theta);
    double s_ps = sin(psi), c_ps = cos(psi);

    Eigen::Matrix3d dR;
    dR << (-c_ps*s_th), (c_ps*c_th*s_ph), (c_ps*c_th*c_ph),
          (-s_ps*s_th), (s_ps*c_th*s_ph), (s_ps*c_th*c_ph),
          (-c_th), (-s_th*s_ph), (-s_th*c_ph);
    return dR;
}

inline Eigen::Matrix3d AdaptiveRobustEKF::dR_dpsi(double phi, double theta, double psi) {
    double s_ph = sin(phi), c_ph = cos(phi);
    double s_th = sin(theta), c_th = cos(theta);
    double s_ps = sin(psi), c_ps = cos(psi);

    Eigen::Matrix3d dR;
    dR << (-s_ps*c_th), (-s_ps*s_th*s_ph - c_ps*c_ph), (-s_ps*s_th*c_ph + c_ps*s_ph),
          (c_ps*c_th), (c_ps*s_th*s_ph - s_ps*c_ph), (c_ps*s_th*c_ph + s_ps*s_ph),
          0, 0, 0;
    return dR;
}

//-----------------
// predict function
//-----------------
void AdaptiveRobustEKF::prediction_stage(double dt){
    Eigen::MatrixXd F(_N_STATES,_N_STATES);

    // jacobian's computation
    F = analitycal_jacobin_state(_X, dt);

    // Priori state and covariance estimated
    _X = f_prediction_model(_X, dt);

    // Priori covariance
    _P = F*_P*F.transpose() + _E_pred;
}

//-----------------
// correction stage
//-----------------
void AdaptiveRobustEKF::correction_wheel_stage(double dt){
    (void)dt;
    // x = [pos[0-2] ori[3-5] vel[6-8] ang[9-11] acc[12-14] biasL[15-20] biasW[21-23] biasI[24-29] pose_lidar_robo[30-35] pose_imu_robo[36-41]]
    Eigen::VectorXd Y(_N_WHEEL), hx(_N_WHEEL), KY(_N_STATES);
    Eigen::MatrixXd H(_N_WHEEL,_N_STATES), K(_N_STATES,_N_WHEEL), E(_N_WHEEL,_N_WHEEL), S(_N_WHEEL,_N_WHEEL);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(_N_STATES,_N_STATES);

    // -------------
    // measure model of wheel odometry (only foward linear velocity)
    // -------------
    hx(0) = _X(6);
    hx(1) = _X(7);
    hx(2) = _X(11);
    // hx + bias
    hx = hx + _X.block(21,0,3,1);

    // -------------
    // Wheel measurement + covariance
    // -------------
    // measurement
    Y = _wheelMeasure;
    // covariance matrices
    E << _E_wheel;

    // -------------
    // Jacobian of hx with respect to the states
    // -------------
    H = Eigen::MatrixXd::Zero(_N_WHEEL,_N_STATES);
    // Derivada em relação às velocidades do robô (estados 6, 7 e 11)
    H(0,6) = 1.0; 
    H(1,7) = 1.0;
    H(2,11) = 1.0;    
    // Derivada em relação ao bias das rodas (estados 21, 22, 23)
    H(0,21) = 1.0;
    H(1,22) = 1.0;
    H(2,23) = 1.0;   

    // -------------
    // Kalman's gain
    // -------------
    // 1. Inovação e Covariância nominal
    Eigen::VectorXd y = Y - hx;
    S = H * _P * H.transpose() + E;

    // 2. Robustez Estatística (Cálculo de Lambda)
    // Distância de Mahalanobis: epsilon = y' * S^-1 * y
    double epsilon = y.transpose() * S.completeOrthogonalDecomposition().pseudoInverse() * y;
    
    double gammak = 7.81; // Valor crítico Chi-quadrado (m=6, alpha=0.05). Para rodas (m=3), use 7.81.
    double lambdak = 1.0;

    if (epsilon > gammak) {
        lambdak = epsilon / gammak;
    }

    // 3. Escalonamento da Matriz de Medição
    Eigen::MatrixXd E_robust = lambdak * E;
    Eigen::MatrixXd S_robust = H * _P * H.transpose() + E_robust;
    Eigen::MatrixXd S_reg = S_robust + _eps * Eigen::MatrixXd::Identity(S_robust.rows(), S_robust.cols());
    
    K = _P * H.transpose() * S_reg.completeOrthogonalDecomposition().pseudoInverse();

    // -------------
    // _X = _X + K*(Y - hx);
    // -------------
    // Residuo correction (usando a inovação y já calculada)
    KY = K * y;

    // Robot Position
    _X.block(0,0,3,1) = _X.block(0,0,3,1) + KY.block(0,0,3,1);

    // Robot Orientation (Yaw, Pitch, Roll)
    _X.block(3,0,3,1) = angularResidualVec(_X.block(3,0,3,1) + KY.block(3,0,3,1));

    // Velocities + Bias + Relative LiDAR-Robot position 
    _X.block(6,0,27,1) = _X.block(6,0,27,1) + KY.block(6,0,27,1);

    // Relative LiDAR-Robot orientation (Online Calibration)
    _X.block(33,0,3,1) = angularResidualVec(_X.block(33,0,3,1) + KY.block(33,0,3,1));
    
    // Relative IMU-Robot position
    _X.block(36,0,3,1) = _X.block(36,0,3,1) + KY.block(36,0,3,1);

    // Relative IMU-Robot orientation (Online Calibration)
    _X.block(39,0,3,1) = angularResidualVec(_X.block(39,0,3,1) + KY.block(39,0,3,1));

    // -------------
    // _P = (I - KH)P(I - KH)' + KRK'
    // -------------
    // Para Joseph ser consistente, R deve ser a matriz de ruído escalonada usada no Ganho
    Eigen::MatrixXd IKH = I - K * H;

    _P = IKH * _P * IKH.transpose() + K * E_robust * K.transpose(); // forma de Joseph - Evita perda de simetria / negativo-definiteness
    _P = 0.5 * (_P + _P.transpose()); // force symmetrize (corrige erros numéricos)
}

void AdaptiveRobustEKF::correction_imu_stage(double dt){
    (void)dt;
    // x = [pos[0-2] ori[3-5] vel[6-8] ang[9-11] acc[12-14] biasL[15-20] biasW[21-23] biasI[24-29] pose_lidar_robo[30-35] pose_imu_robo[36-41]]
    Eigen::VectorXd Y(_N_IMU), hx(_N_IMU), KY(_N_STATES);
    Eigen::MatrixXd H(_N_IMU,_N_STATES), K(_N_STATES,_N_IMU), E(_N_IMU,_N_IMU), S(_N_IMU,_N_IMU),R(_N_IMU,_N_IMU);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(_N_STATES,_N_STATES);
    Eigen::Matrix3d R_i_r;
    Eigen::Vector3d p_i_r;

    // -------------
    // measure model
    // -------------
    // Rotation matrix
    R_i_r = eulerMatrix(_X.block(39,0,3,1));
    Eigen::Matrix3d R_r_i = R_i_r.transpose(); // Inversa: do Robô para o IMU
    // Translation vector
    p_i_r = _X.block(36,0,3,1);
    // mesure model
    Eigen::Vector3d g_world(0, 0, 9.81);
    Eigen::Matrix3d R_wr = eulerMatrix(_X.block(3,0,3,1)); // Robô no Mundo
    Eigen::Vector3d g_robot = R_wr.transpose() * g_world;
    hx.block(0,0,3,1) = R_r_i*_X.block(9,0,3,1) + _X.block(24,0,3,1);
    hx.block(3,0,3,1) = R_r_i*(_X.block(12,0,3,1) + leverArmEffect(_X.block(9,0,3,1), p_i_r) + g_robot) + _X.block(27,0,3,1);

    // -------------
    // IMU measurement + covariance
    // -------------
    Y = _imuMeasure.block(0,0,6,1);
    // covariance matrices
    E = _E_imu.block(0,0,6,6);

    // -------------
    // Jacobian of hx with respect to the states -- MUDAR TODAS
    // -------------
    H = calculateJacobianIMU(_X);

    // -------------
    // Kalman's gain
    // -------------
    // 1. Inovação e Covariância nominal
    Eigen::VectorXd y = Y - hx;
    S = H * _P * H.transpose() + E;

    // 2. Robustez Estatística (Cálculo de Lambda)
    // Distância de Mahalanobis: epsilon = y' * S^-1 * y
    double epsilon = y.transpose() * S.completeOrthogonalDecomposition().pseudoInverse() * y;
    
    double gammak = 12.59; // Valor crítico Chi-quadrado (m=6, alpha=0.05). Para rodas (m=3), use 7.81.
    double lambdak = 1.0;

    if (epsilon > gammak) {
        lambdak = epsilon / gammak;
    }

    // 3. Escalonamento da Matriz de Medição
    Eigen::MatrixXd E_robust = lambdak * E;
    Eigen::MatrixXd S_robust = H * _P * H.transpose() + E_robust;
    Eigen::MatrixXd S_reg = S_robust + _eps * Eigen::MatrixXd::Identity(S_robust.rows(), S_robust.cols());
    
    K = _P * H.transpose() * S_reg.completeOrthogonalDecomposition().pseudoInverse();

    // -------------
    // _X = _X + K*(Y - hx);
    // -------------
    // Residuo correction (usando a inovação y já calculada)
    KY = K * y;

    // Robot Position
    _X.block(0,0,3,1) = _X.block(0,0,3,1) + KY.block(0,0,3,1);

    // Robot Orientation (Yaw, Pitch, Roll)
    _X.block(3,0,3,1) = angularResidualVec(_X.block(3,0,3,1) + KY.block(3,0,3,1));

    // Velocities + Bias + Relative LiDAR-Robot position 
    _X.block(6,0,27,1) = _X.block(6,0,27,1) + KY.block(6,0,27,1);

    // Relative LiDAR-Robot orientation (Online Calibration)
    _X.block(33,0,3,1) = angularResidualVec(_X.block(33,0,3,1) + KY.block(33,0,3,1));
    
    // Relative IMU-Robot position
    _X.block(36,0,3,1) = _X.block(36,0,3,1) + KY.block(36,0,3,1);

    // Relative IMU-Robot orientation (Online Calibration)
    _X.block(39,0,3,1) = angularResidualVec(_X.block(39,0,3,1) + KY.block(39,0,3,1));

    // -------------
    // _P = (I - KH)P(I - KH)' + KRK'
    // -------------
    // Para Joseph ser consistente, R deve ser a matriz de ruído escalonada usada no Ganho
    Eigen::MatrixXd IKH = I - K * H;

    _P = IKH * _P * IKH.transpose() + K * E_robust * K.transpose(); // forma de Joseph - Evita perda de simetria / negativo-definiteness
    _P = 0.5 * (_P + _P.transpose()); // force symmetrize (corrige erros numéricos)
}

void AdaptiveRobustEKF::correction_lidar_stage(double dt){
    // x = [pos[0-2] ori[3-5] vel[6-8] ang[9-11] acc[12-14] biasL[15-20] biasW[21-23] biasI[24-29] pose_lidar_robo[30-35] pose_imu_robo[36-41]]
    Eigen::MatrixXd K(_N_STATES,_N_LIDAR), S(_N_LIDAR,_N_LIDAR), G(_N_LIDAR,_N_LIDAR), Gl(_N_LIDAR,_N_LIDAR), E(_N_LIDAR,_N_LIDAR), R(_N_LIDAR,_N_LIDAR);
    Eigen::VectorXd Y(_N_LIDAR), hx(_N_LIDAR), KY(_N_STATES);
    Eigen::MatrixXd H(_N_LIDAR,_N_STATES); 
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(_N_STATES,_N_STATES);
    Eigen::Matrix3d R_l_r;
    Eigen::Vector3d p_l_r;

    // -------------
    // measure model
    // -------------
    // rotation matrix
    R_l_r = eulerMatrix(_X.block(33,0,3,1));
    Eigen::Matrix3d R_r_l = R_l_r.transpose(); // Inversa: do Robô para o LiDAR
    // Translation vector
    p_l_r = _X.block(30,0,3,1);
    // measure model
    hx.block(0,0,3,1) = R_r_l*(_X.block(6,0,3,1) + skewSymmetric(_X.block(9,0,3,1))*p_l_r) + _X.block(15,0,3,1);
    hx.block(3,0,3,1) = R_r_l*_X.block(9,0,3,1) + _X.block(18,0,3,1);
    
    // -------------
    // lidar measurement
    // -------------
    if (_firstLidar){
        _lidarMeasureL = _lidarMeasure;
        _E_lidarL =_E_lidar;
        _firstLidar = false;
    }
    Y = indirect_odometry_measurement(_lidarMeasure, _lidarMeasureL, dt);
    // Error propagation
    G = jacobian_odometry_measurement(_lidarMeasure, _lidarMeasureL, dt);
    // E =  G*_E_lidar*G.transpose(); // the covariance matrix is aready about the relative pose
    E = _E_lidar; 
    
    // -------------
    // Jacobian of hx with respect to the states
    // -------------
    H = calculateJacobianLidar(_X);

    // -------------
    // Kalman's gain
    // -------------
    // 1. Inovação e Covariância nominal
    Eigen::VectorXd y = Y - hx;
    S = H * _P * H.transpose() + E;

    // 2. Robustez Estatística (Cálculo de Lambda)
    // Distância de Mahalanobis: epsilon = y' * S^-1 * y
    double epsilon = y.transpose() * S.completeOrthogonalDecomposition().pseudoInverse() * y;
    
    double gammak = 12.59; // Valor crítico Chi-quadrado (m=6, alpha=0.05). Para rodas (m=3), use 7.81.
    double lambdak = 1.0;

    if (epsilon > gammak) {
        lambdak = epsilon / gammak;
    }

    // 3. Escalonamento da Matriz de Medição
    Eigen::MatrixXd E_robust = lambdak * E;
    Eigen::MatrixXd S_robust = H * _P * H.transpose() + E_robust;
    Eigen::MatrixXd S_reg = S_robust + _eps * Eigen::MatrixXd::Identity(S_robust.rows(), S_robust.cols());
    
    K = _P * H.transpose() * S_reg.completeOrthogonalDecomposition().pseudoInverse();

    // -------------
    // _X = _X + K*(Y - hx);
    // -------------
    // Residuo correction (usando a inovação y já calculada)
    KY = K * y;

    // Robot Position
    _X.block(0,0,3,1) = _X.block(0,0,3,1) + KY.block(0,0,3,1);

    // Robot Orientation (Yaw, Pitch, Roll)
    _X.block(3,0,3,1) = angularResidualVec(_X.block(3,0,3,1) + KY.block(3,0,3,1));

    // Velocities + Bias + Relative LiDAR-Robot position 
    _X.block(6,0,27,1) = _X.block(6,0,27,1) + KY.block(6,0,27,1);

    // Relative LiDAR-Robot orientation (Online Calibration)
    _X.block(33,0,3,1) = angularResidualVec(_X.block(33,0,3,1) + KY.block(33,0,3,1));
    
    // Relative IMU-Robot position
    _X.block(36,0,3,1) = _X.block(36,0,3,1) + KY.block(36,0,3,1);

    // Relative IMU-Robot orientation (Online Calibration)
    _X.block(39,0,3,1) = angularResidualVec(_X.block(39,0,3,1) + KY.block(39,0,3,1));

    // -------------
    // _P = (I - KH)P(I - KH)' + KRK'
    // -------------
    // Para Joseph ser consistente, R deve ser a matriz de ruído escalonada usada no Ganho
    Eigen::MatrixXd IKH = I - K * H;

    _P = IKH * _P * IKH.transpose() + K * E_robust * K.transpose(); // forma de Joseph - Evita perda de simetria / negativo-definiteness
    _P = 0.5 * (_P + _P.transpose()); // force symmetrize (corrige erros numéricos)

    // -------------
    // last measurement
    // -------------
    _lidarMeasureL = _lidarMeasure;
    _E_lidarL = _E_lidar;
}

//---------
// Models
//---------
VectorXd AdaptiveRobustEKF::f_prediction_model(VectorXd x, double dt){ 
    // x = [pos[0-2] ori[3-5] vel[6-8] ang[9-11] acc[12-14] biasL[15-20] biasW[21-23] biasI[24-29] pose_lidar_robo[30-35] pose_imu_robo[36-41]]
    Eigen::Matrix3d R, Rx, Ry, Rz, J;
    Eigen::VectorXd xp(_N_STATES), Axdt(6), xNew(_N_STATES);
    Eigen::MatrixXd A(6,6);

    // Rotation matrix
    Rx = Eigen::AngleAxisd(x(3), Eigen::Vector3d::UnitX());
    Ry = Eigen::AngleAxisd(x(4), Eigen::Vector3d::UnitY());
    Rz = Eigen::AngleAxisd(x(5), Eigen::Vector3d::UnitZ());
    R = Rz*Ry*Rx;
    
    // Jacobian matrix
    J << 1.0, sin(x(3))*tan(x(4)), cos(x(3))*tan(x(4)),
         0.0, cos(x(3)), -sin(x(3)),
         0.0, sin(x(3))/cos(x(4)), cos(x(3))/cos(x(4));
    
    // model
    A = Eigen::MatrixXd::Identity(6,6);
    A.block(0,0,3,3) = R;
    A.block(3,3,3,3) = J;

    // x(2) = 0.0; // Força Posição Z no chãos
    x(7) = 0.0; // Força Posição vy no chãos
    x(8) = 0.0; // Força Posição vz no chãos
    x(9) = 0.0; // Força Velocidade wx zero
    x(10) = 0.0; // Força Velocidade wy zero

    // displacement
    //       (   velocities  )             
    Axdt = A*x.block(6,0,6,1)*dt;
    // position
    xp.block(0,0,3,1) = x.block(0,0,3,1) + Axdt.block(0,0,3,1);
    // orientation
    xp.block(3,0,3,1) = angularResidualVec(x.block(3,0,3,1) + Axdt.block(3,0,3,1));
    // linear velocity
    // v_k            = v_k-1            + (   aceleration  )
    xp.block(6,0,3,1) = x.block(6,0,3,1) + x.block(12,0,3,1)*dt;
    // the rest of states
    xp.block(9,0,33,1) = x.block(9,0,33,1);

    return xp;
}

VectorXd AdaptiveRobustEKF::indirect_odometry_measurement(VectorXd u, VectorXd ul, double dt){
    Eigen::Matrix3d R, J;
    Eigen::VectorXd up, u_diff; 
    Eigen::MatrixXd A; 

    up.resize(_N_LIDAR);
    u_diff.resize(_N_LIDAR);
    A.resize(_N_LIDAR,_N_LIDAR); 
    A = Eigen::MatrixXd::Zero(_N_LIDAR,_N_LIDAR);

    // Rotation matrix
    R = eulerMatrix(ul.block(3,0,3,1));
    
    // Jacobian matrix
    J << 1.0, sin(ul(3))*tan(ul(4)), cos(ul(3))*tan(ul(4)),
            0.0, cos(ul(3)), -sin(ul(3)),
            0.0, sin(ul(3))/cos(ul(4)), cos(ul(3))/cos(ul(4));
    
    u_diff.block(0,0,3,1) = (u.block(0,0,3,1) - ul.block(0,0,3,1));
    u_diff.block(3,0,3,1) = angularResidualVec(u.block(3,0,3,1) - ul.block(3,0,3,1));

    // model
    u_diff = alpha_lidar*u_diff;

    A.block(0,0,3,3) = R.transpose();
    A.block(3,3,3,3) = J.inverse();

    up = A*u_diff/dt;

    // velocity filter
    Eigen::VectorXd f_up = _velocityFilter.update(up, dt);

    return f_up;
}

//----------
// Jacobians
//----------
MatrixXd AdaptiveRobustEKF::jacobian_state(VectorXd x, double dt){
    Eigen::MatrixXd J(_N_STATES,_N_STATES);
    Eigen::VectorXd f0(_N_STATES), f1(_N_STATES), x_plus(_N_STATES);

    f0 = f_prediction_model(x, dt);

    double delta = 1e-4;
    for (int i = 0; i < _N_STATES; ++i){
        x_plus = x;
        x_plus(i) = x_plus(i) + delta;

        f1 = f_prediction_model(x_plus, dt);
        
        //change here - there are more angles state parts
        // x = [pos[3] ori[3] vel[3] ang[3] acc[3] vang[3] bias[12] biasL[6] biasW[3] biasI[6] pose_lidar_robo[6] pose_imu_robo[6]]
        J.block(0,i,_N_STATES,1) = (f1 - f0)/delta;
        // state orientation
        J.block(3,i,3,1) = angularResidualVec(f1.block(3,0,3,1) - f0.block(3,0,3,1))/delta;
        // lidar-robot orientation
        J.block(48,i,3,1) = angularResidualVec(f1.block(48,0,3,1) - f0.block(48,0,3,1))/delta;
        // imu-robot orientation
        J.block(54,i,3,1) = angularResidualVec(f1.block(54,0,3,1) - f0.block(54,0,3,1))/delta;
    }

    return J;
}

MatrixXd AdaptiveRobustEKF::jacobian_odometry_measurement(VectorXd u, VectorXd ul, double dt){
    Eigen::MatrixXd J;
    Eigen::VectorXd f0(_N_LIDAR), f1(_N_LIDAR), u_plus(_N_LIDAR);
    double delta;

    J.resize(_N_LIDAR,_N_LIDAR);
    f0.resize(_N_LIDAR);
    f1.resize(_N_LIDAR);
    u_plus.resize(_N_LIDAR);

    f0 = indirect_odometry_measurement(u, ul, dt);

    delta = 0.0000001;
    for (int i = 0; i < _N_LIDAR; ++i){
        u_plus = u;
        u_plus(i) = u_plus(i) + delta;

        f1 = indirect_odometry_measurement(u_plus, ul, dt);
    
        J.block(0,i,_N_LIDAR,1) = (f1 - f0)/delta; 
        J.block(3,i,3,1) = angularResidualVec(f1.block(3,0,3,1) - f0.block(3,0,3,1))/delta;
    }

    return J;
}

MatrixXd AdaptiveRobustEKF::jacobian_odometry_measurementL(VectorXd u, VectorXd ul, double dt){ 
    Eigen::MatrixXd J;
    Eigen::VectorXd f0(_N_LIDAR), f1(_N_LIDAR), ul_plus(_N_LIDAR);
    double delta;

    J.resize(_N_LIDAR,_N_LIDAR);
    f0.resize(_N_LIDAR);
    f1.resize(_N_LIDAR);
    ul_plus.resize(_N_LIDAR);

    f0 = indirect_odometry_measurement(u, ul, dt);

    delta = 0.0000001;
    for (int i = 0; i < _N_LIDAR; ++i){
        ul_plus = ul;
        ul_plus(i) = ul_plus(i) + delta;

        f1 = indirect_odometry_measurement(u, ul_plus, dt);
    
        J.block(0,i,_N_LIDAR,1) = (f1 - f0)/delta;
        J.block(3,i,3,1) = angularResidualVec(f1.block(3,0,3,1) - f0.block(3,0,3,1))/delta;
    }

    return J;
}

Eigen::MatrixXd AdaptiveRobustEKF::analitycal_jacobin_state(const Eigen::VectorXd& x, double dt) {
    // Inicializa como Identidade (42x42)
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(_N_STATES, _N_STATES);

    // Extração de estados dinâmicos
    double phi = x(3), theta = x(4), psi = x(5);
    Eigen::Vector3d v_robot = x.block<3,1>(6,0);
    Eigen::Vector3d w_robot = x.block<3,1>(9,0);

    // --- BLOCO POSIÇÃO (Linhas 0-2) ---
    // dp/d_euler (Dependência da rotação em relação aos ângulos)
    F.block<3,1>(0,3) = dRv_dphi(phi, theta, psi, v_robot) * dt;
    F.block<3,1>(0,4) = dRv_dtheta(phi, theta, psi, v_robot) * dt;
    F.block<3,1>(0,5) = dRv_dpsi(phi, theta, psi, v_robot) * dt;
    
    // dp/dv (Rotação aplicada à velocidade)
    Eigen::Matrix3d R = eulerMatrix(x.block<3,1>(3,0));
    F.block<3,3>(0,6) = R * dt;

    // --- BLOCO ORIENTAÇÃO (Linhas 3-5) ---
    // d_euler/d_euler (Sensibilidade da matriz cinemática J)
    F.block<3,1>(3,3) += dJw_dphi(phi, theta, w_robot) * dt;
    F.block<3,1>(3,4) += dJw_dtheta(phi, theta, w_robot) * dt;
    
    // d_euler/dw (Matriz cinemática de Euler)
    Eigen::Matrix3d J = eulerKinematicMatrix(phi, theta);
    F.block<3,3>(3,9) = J * dt;

    // --- BLOCO VELOCIDADE LINEAR (Linhas 6-8) ---
    // dv/da (Velocidade depende da aceleração)
    F.block<3,3>(6,12) = Eigen::Matrix3d::Identity() * dt;

    // --- BLOCO VELOCIDADE ANGULAR (Linhas 9-11) ---
    // No seu modelo xp(9:11) = x(9:11), portanto F(9:11, 9:11) já é Identidade.
    // Não há termos extras aqui pois não há aceleração angular no estado.

    // Os estados de bias (15-29) e extrínsecos (30-41) são constantes na predição,
    // mantendo apenas a Identidade na diagonal principal.

    return F;
}

Eigen::MatrixXd AdaptiveRobustEKF::calculateJacobianIMU(const Eigen::VectorXd& x) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, _N_STATES);

    // 1. Extração de Estados
    Eigen::Vector3d att_r = x.segment<3>(3);
    Eigen::Vector3d w_robot = x.segment<3>(9);
    Eigen::Vector3d a_robot = x.segment<3>(12);
    Eigen::Vector3d p_ir = x.segment<3>(36);
    Eigen::Vector3d att_ir = x.segment<3>(39);
    
    Eigen::Matrix3d R_ir = eulerMatrix(att_ir);
    Eigen::Matrix3d R_ri = R_ir.transpose();

    // --- GYRO (Linhas 0-2) ---
    H.block<3,3>(0,9) = R_ri; 
    H.block<3,3>(0,24) = Eigen::Matrix3d::Identity();

    // Derivada do Gyro em relação à atitude extrínseca (att_ir)
    // d(R_ir^T * w)/d_theta = (dR_ir/d_theta)^T * w
    H.block<3,1>(0,39) = dR_dphi(att_ir(0), att_ir(1), att_ir(2)).transpose() * w_robot;
    H.block<3,1>(0,40) = dR_dtheta(att_ir(0), att_ir(1), att_ir(2)).transpose() * w_robot;
    H.block<3,1>(0,41) = dR_dpsi(att_ir(0), att_ir(1), att_ir(2)).transpose() * w_robot;

    // --- ACCEL (Linhas 3-5) ---
    H.block<3,3>(3,12) = R_ri; 
    H.block<3,3>(3,27) = Eigen::Matrix3d::Identity();

    // 2. Derivada da Gravidade (em relação à atitude do robô att_r)
    // Como a medida é R_ri * (a + R_wr^T * g), a derivada em relação a att_r 
    // é R_ri * (dR_wr/d_theta)^T * g
    Eigen::Vector3d g_world(0, 0, 9.81);
    H.block<3,1>(3,3) = R_ri * (dR_dphi(att_r(0), att_r(1), att_r(2)).transpose() * g_world);
    H.block<3,1>(3,4) = R_ri * (dR_dtheta(att_r(0), att_r(1), att_r(2)).transpose() * g_world);
    H.block<3,1>(3,5) = R_ri * (dR_dpsi(att_r(0), att_r(1), att_r(2)).transpose() * g_world);

    // 3. Coriolis / Centrípeta (em relação a w_robot)
    H.block<3,3>(3,9) = R_ri * (skewSymmetric(w_robot) * (-skewSymmetric(p_ir)) - skewSymmetric(w_robot.cross(p_ir)));

    // 4. Extrínseco Posição (p_ir)
    H.block<3,3>(3,36) = R_ri * (skewSymmetric(w_robot) * skewSymmetric(w_robot));

    // 5. Extrínseco Atitude (att_ir)
    // d(R_ir^T * acc_body)/d_theta = (dR_ir/d_theta)^T * acc_body
    Eigen::Vector3d g_robot = eulerMatrix(att_r).transpose() * g_world;
    Eigen::Vector3d acc_body = a_robot + g_robot + w_robot.cross(w_robot.cross(p_ir));
    
    H.block<3,1>(3,39) = dR_dphi(att_ir(0), att_ir(1), att_ir(2)).transpose() * acc_body;
    H.block<3,1>(3,40) = dR_dtheta(att_ir(0), att_ir(1), att_ir(2)).transpose() * acc_body;
    H.block<3,1>(3,41) = dR_dpsi(att_ir(0), att_ir(1), att_ir(2)).transpose() * acc_body;

    return H;
}

Eigen::MatrixXd AdaptiveRobustEKF::calculateJacobianLidar(const Eigen::VectorXd& x) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 42); // 42 estados fixos

    // Extração com índices corrigidos
    Eigen::Vector3d v_r = x.segment<3>(6);
    Eigen::Vector3d w_r = x.segment<3>(9);
    Eigen::Vector3d p_lr = x.segment<3>(30);   // Posição LiDAR-Robô
    Eigen::Vector3d att_lr = x.segment<3>(33); // Orientação LiDAR-Robô
    
    Eigen::Matrix3d R_lr = eulerMatrix(att_lr);
    Eigen::Matrix3d R_rl = R_lr.transpose(); // Inversa: do Robô para o LiDAR

    // --- VELOCIDADE LINEAR (Linhas 0, 1, 2) ---
    // d_hv / d_vr
    H.block<3,3>(0,6) = R_rl;
    
    // d_hv / d_wr -> R_rl * (-skew(p_lr))
    H.block<3,3>(0,9) = R_rl * (-skewSymmetric(p_lr));
    
    // d_hv / d_biasL_linear (Índices 15, 16, 17)
    H.block<3,3>(0,15) = Eigen::Matrix3d::Identity();
    
    // d_hv / d_p_lr (Índices 30, 31, 32) -> R_rl * skew(w_r)
    H.block<3,3>(0,30) = R_rl * skewSymmetric(w_r);
    
    // d_hv / d_att_lr (Índices 33, 34, 35)
    // d(R^T * v) / d_angle = (dR / d_angle)^T * v
    Eigen::Vector3d v_total_body = v_r + w_r.cross(p_lr);
    H.block<3,1>(0,33) = dRv_dphi(att_lr(0), att_lr(1), att_lr(2), v_total_body).transpose();
    H.block<3,1>(0,34) = dRv_dtheta(att_lr(0), att_lr(1), att_lr(2), v_total_body).transpose();
    H.block<3,1>(0,35) = dRv_dpsi(att_lr(0), att_lr(1), att_lr(2), v_total_body).transpose();

    // --- VELOCIDADE ANGULAR (Linhas 3, 4, 5) ---
    // d_hw / d_wr
    H.block<3,3>(3,9) = R_rl;
    
    // d_hw / d_biasL_angular (Índices 18, 19, 20)
    H.block<3,3>(3,18) = Eigen::Matrix3d::Identity();
    
    // d_hw / d_att_lr (Índices 33, 34, 35)
    H.block<3,1>(3,33) = dRv_dphi(att_lr(0), att_lr(1), att_lr(2), w_r).transpose();
    H.block<3,1>(3,34) = dRv_dtheta(att_lr(0), att_lr(1), att_lr(2), w_r).transpose();
    H.block<3,1>(3,35) = dRv_dpsi(att_lr(0), att_lr(1), att_lr(2), w_r).transpose();

    return H;
}

//--------------
// run
//--------------
void AdaptiveRobustEKF::run(){
    // rate
    rclcpp::Rate r(freq);
    rclcpp::Clock clock(RCL_STEADY_TIME);

    double t_last = clock.now().seconds();
    double t_now;
    double dt_now;

    double imu_count, wheel_count, lidar_count;
    bool imu_fail_msg, wheel_fail_msg, lidar_fail_msg;
    imu_count = 0;
    wheel_count = 0;
    lidar_count = 0;
    imu_fail_msg = false;
    wheel_fail_msg = false;
    lidar_fail_msg = false;
    bool firstData = false;

    while (_running && rclcpp::ok())
    {
        // prediction stage
        t_now = clock.now().seconds();
        dt_now = t_now-t_last;
        t_last = t_now;

        if((imu_count >= _imu_dt*1.2)) {
            imu_fail_msg = true;
        }
        
        if (_imuNew){
            imu_fail_msg = false;
        }

        if((wheel_count >= _wheel_dt*1.2)) {
            wheel_fail_msg = true;
        }
        
        if (_wheelNew){
            wheel_fail_msg = false;
        }

        if((lidar_count >= _lidar_dt*1.2)) {
            lidar_fail_msg = true;
        }
        
        if (_lidarNew){
            lidar_fail_msg = false;
        }

        if ((_imuActivated || _wheelActivated || _lidarActivated) && !firstData){
            firstData = true;
        }

        // **************
        // tests de flags
        // **************
        if (((_imuActivated && enableImu) || (_wheelActivated && enableWheel) || (_lidarActivated && enableLidar)) &&
            (!((imu_fail_msg || !enableImu) && (wheel_fail_msg || !enableWheel) && (lidar_fail_msg || !enableLidar))) && 
                firstData){

            prediction_stage(dt_now);
        }
        else {
            // printf("Sensors failed, waiting for new data...\n");
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(_mutex);
            //Correction IMU
            if (enableImu && _imuActivated && _imuNew){
                correction_imu_stage(_imu_dt);
                _imuNew =  false;
                imu_fail_msg = false;
                imu_count = 0;
                // ROS_INFO("\033[1;32mAdaptive Filter:\033[0m Correction IMU.");
            }

            // Correction wheel
            if (enableWheel && _wheelActivated && _wheelNew){                
                correction_wheel_stage(_wheel_dt);
                _wheelNew =  false;
                wheel_fail_msg = false;
                wheel_count = 0;
                // ROS_INFO("\033[1;32mAdaptive Filter:\033[0m Correction Rodas.");
            }

            //Corection LiDAR
            if (enableLidar && _lidarActivated && _lidarNew){                
                correction_lidar_stage(_lidar_dt);
                _lidarNew =  false;
                lidar_fail_msg = false;
                lidar_count = 0;
                // ROS_INFO("\033[1;32mAdaptive Filter:\033[0m Correction LiDAR.");
            }

            // out data
            _V = _X;
            _PV = _P;
            // ROS_INFO("\033[1;32mAdaptive Filter:\033[0m Correction END.");
        }
        
        // counters increment
        imu_count += dt_now;
        wheel_count += dt_now;
        lidar_count += dt_now;

        // final marker 
        r.sleep();
    }
}

//---------------------------
// covariances functions
//---------------------------
MatrixXd AdaptiveRobustEKF::adaptive_covariance(double fCorner, double fSurf){
    Eigen::MatrixXd Q(6,6);
    double cov_x = l_min;
    double cov_y = l_min;
    double cov_z = l_min;
    double cov_phi = l_min;
    double cov_psi = l_min;
    double cov_theta = l_min;
    
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
            cov_z     = lidarG*exp(-lidarG*(nSurf -   min(fSurf,nSurf))/nSurf) + l_min;
            cov_phi   = lidarG*exp(-lidarG*(nSurf -   min(fSurf,nSurf))/nSurf) + l_min;
            cov_theta = lidarG*exp(-lidarG*(nSurf -   min(fSurf,nSurf))/nSurf) + l_min;
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

MatrixXd AdaptiveRobustEKF::wheelOdometryAdaptiveCovariance(double omegaz_wheel_odom, double omegaz_imu){
    Eigen::MatrixXd E(2,2);

    E(0,0) = gamma_vx * abs(omegaz_wheel_odom - omegaz_imu) + delta_vx;
    E(1,1) = gamma_omegaz * abs(omegaz_wheel_odom - omegaz_imu) + delta_omegaz;

    return E;
}     

//**************************
// PUBLIC FUNCTIONS
//**************************
//----------
// Datas
//----------
void AdaptiveRobustEKF::correction_imu_data(VectorXd imu, MatrixXd E_imu, double dt){
    std::lock_guard<std::mutex> lock(_mutex);
    // activation
    if (!_imuActivated){
        _imuActivated = true;
    }

    // update data
    _imuMeasure = imu;
    _E_imu.block(0,0,3,3) = E_imu.block(0,0,3,3);
    _E_imu.block(3,3,3,3) = 100*E_imu.block(0,0,3,3);
    _imu_dt = dt;

    //New measure
    _imuNew = true;
}

// conferir aqui se usar alguma nova formulacao para testes somente
void AdaptiveRobustEKF::correction_wheel_data(VectorXd wheel_odom, MatrixXd E_wheel, double dt, double omegaz_imu){
    // mutex
    std::lock_guard<std::mutex> lock(_mutex);

    // activation
    if (!_wheelActivated){
        _wheelActivated = true;
    }

    // update data
    _wheelMeasure = wheel_odom;

    if (wheel_type_func==1){
        _E_wheel = E_wheel;
    }else{
        double omegaz_wheel_odom = wheel_odom(2);
        _E_wheel = wheelOdometryAdaptiveCovariance(omegaz_wheel_odom, omegaz_imu);
    }

    _wheel_dt = dt;

    //New measure
    _wheelNew = true;
}

void AdaptiveRobustEKF::correction_lidar_data(VectorXd lidar_odom, MatrixXd E_lidar, double dt, double corner, double surf){
    std::lock_guard<std::mutex> lock(_mutex);
    // activation
    if (!_lidarActivated){
        _lidarActivated = true;
    }

    // update data
    _lidarMeasure = lidar_odom;
    if (lidar_type_func==2){
        _E_lidar = E_lidar;
    }else{
        _E_lidar = adaptive_covariance(corner, surf);                
    }

    _lidar_dt = dt;

    //New measure
    _lidarNew = true;
}

//------------------
// filter control - verfica aqui
// -----------------
void AdaptiveRobustEKF::get_state(VectorXd &X_state, MatrixXd &E_state) {
    std::lock_guard<std::mutex> lock(_mutex);
    // get the last state updated
    X_state = _X;
    E_state = _P;
}

void AdaptiveRobustEKF::set_initial_state(VectorXd X_state, MatrixXd E_state){
    std::lock_guard<std::mutex> lock(_mutex);
    // priori stare
    _X = X_state;
    _P = E_state;
}

void AdaptiveRobustEKF::start() {
    _running = true;
    _thread = std::thread(&AdaptiveRobustEKF::run, this);
}

void AdaptiveRobustEKF::stop() {
    _running = false;
    if (_thread.joinable()) {
        _thread.join();
    }
}
