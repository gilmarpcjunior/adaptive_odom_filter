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
    _E_pred = Eigen::MatrixXd::Identity(_N_STATES,_N_STATES)*1e-3;

    // state initial
    _X = Eigen::VectorXd::Zero(_N_STATES);
    _P = Eigen::MatrixXd::Identity(_N_STATES,_N_STATES) * 1e-6;
    _V = Eigen::VectorXd::Zero(_N_STATES);

    // initial covariance - bias and calibration block
    _P.block(18,18,39,39) = Eigen::MatrixXd::Identity(39,39) * 1e-2;
    _P.block(45,45,12,12) = Eigen::MatrixXd::Identity(12,12) * 1e-5;

    // vector initialization
    // x = [pos[0-2] ori[3-5] vel[6-8] ang[9-11] acc[12-14] vang[15-17] bias[18-29] biasL[30-35] biasW[36-38] biasI[39-44] pose_lidar_robo[45-50] pose_imu_robo[51-56]]
    // bias[18-29] = vx, vy, vz, wx, wy, wz, ax, ay, az, vwx, vwy, vwz
    _X.segment(18, 9).setConstant(1e-4);
    _X.segment(27, 3).setConstant(1e-2);
    // biasL[30-35] = vx, vy, vz, wx, wy, wz,
    _X.segment(30, 6).setConstant(1e-3);
    // biasW[36-38] = vx, vy, wz
    _X.segment(36, 3).setConstant(1e-4);
    // biasI[39-44] = wx, wy, wz, ax, ay, az
    _X.segment(39, 6).setConstant(1e-5);
    // pose_lidar_robo[45-50]
    _X[45] = 0.0;
    _X[46] = 0.0;
    _X[47] = 0.0;
    _X[48] = 0.0;
    _X[49] = 0.0;
    _X[50] = 0.0;
    // pose_imu_robo[51-56]] - simulation data
    _X[51] = 0.0;
    _X[52] = 0.0;
    _X[53] = 0.0085;
    _X[54] = 0.0;
    _X[55] = 0.0;
    _X[56] = 0.0;

    // out data
    _V = _X;
    _PV = _P;
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
    Eigen::VectorXd Y(_N_WHEEL), hx(_N_WHEEL), KY(_N_STATES);
    Eigen::MatrixXd H(_N_WHEEL,_N_STATES), K(_N_STATES,_N_WHEEL), E(_N_WHEEL,_N_WHEEL), S(_N_WHEEL,_N_WHEEL),R(_N_WHEEL,_N_WHEEL);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(_N_STATES,_N_STATES);

    // -------------
    // measure model of wheel odometry (only foward linear velocity)
    // -------------
    hx(0) = _X(6);
    hx(1) = _X(7);
    hx(2) = _X(11);
    // hx + bias
    hx = hx + _X.block(36,0,3,1);

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
    // Derivada em relação ao bias das rodas (estados 36, 37, 38)
    H(0,36) = 1.0;
    H(1,37) = 1.0;
    H(2,38) = 1.0;   

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
    _X.block(6,0,42,1) = _X.block(6,0,42,1) + KY.block(6,0,42,1);

    // Relative LiDAR-Robot orientation (Online Calibration)
    _X.block(48,0,3,1) = angularResidualVec(_X.block(48,0,3,1) + KY.block(48,0,3,1));
    
    // Relative IMU-Robot position
    _X.block(51,0,3,1) = _X.block(51,0,3,1) + KY.block(51,0,3,1);

    // Relative IMU-Robot orientation (Online Calibration)
    _X.block(54,0,3,1) = angularResidualVec(_X.block(54,0,3,1) + KY.block(54,0,3,1));

    // -------------
    // _P = (I - KH)P(I - KH)' + KRK'
    // -------------
    // Para Joseph ser consistente, R deve ser a matriz de ruído escalonada usada no Ganho
    Eigen::MatrixXd IKH = I - K * H;

    _P = IKH * _P * IKH.transpose() + K * E_robust * K.transpose(); // forma de Joseph - Evita perda de simetria / negativo-definiteness
    _P = 0.5 * (_P + _P.transpose()); // force symmetrize (corrige erros numéricos)
}

void AdaptiveRobustEKF::correction_imu_stage(double dt){
    Eigen::VectorXd Y(_N_IMU), hx(_N_IMU), KY(_N_STATES);
    Eigen::MatrixXd H(_N_IMU,_N_STATES), K(_N_STATES,_N_IMU), E(_N_IMU,_N_IMU), S(_N_IMU,_N_IMU),R(_N_IMU,_N_IMU);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(_N_STATES,_N_STATES);
    Eigen::Matrix3d R_i_r;
    Eigen::Vector3d p_i_r;

    // -------------
    // measure model
    // -------------
    // Rotation matrix
    R_i_r = eulerMatrix(_X.block(54,0,3,1));
    Eigen::Matrix3d R_r_i = R_i_r.transpose();
    // Translation vector
    p_i_r = _X.block(51,0,3,1);
    // mesure model
    hx.block(0,0,3,1) = R_r_i*_X.block(9,0,3,1) + _X.block(39,0,3,1);
    hx.block(3,0,3,1) = R_r_i*(_X.block(12,0,3,1) + skewSymmetric(_X.block(15,0,3,1) - _X.block(27,0,3,1))*p_i_r + leverArmEffect(_X.block(9,0,3,1), p_i_r)) + _X.block(42,0,3,1);

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
    _X.block(6,0,42,1) = _X.block(6,0,42,1) + KY.block(6,0,42,1);

    // Relative LiDAR-Robot orientation (Online Calibration)
    _X.block(48,0,3,1) = angularResidualVec(_X.block(48,0,3,1) + KY.block(48,0,3,1));
    
    // Relative IMU-Robot position
    _X.block(51,0,3,1) = _X.block(51,0,3,1) + KY.block(51,0,3,1);

    // Relative IMU-Robot orientation (Online Calibration)
    _X.block(54,0,3,1) = angularResidualVec(_X.block(54,0,3,1) + KY.block(54,0,3,1));

    // -------------
    // _P = (I - KH)P(I - KH)' + KRK'
    // -------------
    // Para Joseph ser consistente, R deve ser a matriz de ruído escalonada usada no Ganho
    Eigen::MatrixXd IKH = I - K * H;

    _P = IKH * _P * IKH.transpose() + K * E_robust * K.transpose(); // forma de Joseph - Evita perda de simetria / negativo-definiteness
    _P = 0.5 * (_P + _P.transpose()); // force symmetrize (corrige erros numéricos)
}

void AdaptiveRobustEKF::correction_lidar_stage(double dt){
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
    R_l_r = eulerMatrix(_X.block(48,0,3,1));
    Eigen::Matrix3d R_r_l = R_l_r.transpose(); // Inversa: do Robô para o LiDAR
    // Translation vector
    p_l_r = _X.block(45,0,3,1);
    // measure model
    hx.block(0,0,3,1) = R_r_l*(_X.block(6,0,3,1) + skewSymmetric(_X.block(9,0,3,1))*p_l_r) + _X.block(30,0,3,1);
    hx.block(3,0,3,1) = R_r_l*_X.block(9,0,3,1) + _X.block(33,0,3,1);

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
    _X.block(6,0,42,1) = _X.block(6,0,42,1) + KY.block(6,0,42,1);

    // Relative LiDAR-Robot orientation (Online Calibration)
    _X.block(48,0,3,1) = angularResidualVec(_X.block(48,0,3,1) + KY.block(48,0,3,1));
    
    // Relative IMU-Robot position
    _X.block(51,0,3,1) = _X.block(51,0,3,1) + KY.block(51,0,3,1);

    // Relative IMU-Robot orientation (Online Calibration)
    _X.block(54,0,3,1) = angularResidualVec(_X.block(54,0,3,1) + KY.block(54,0,3,1));

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
    // x = [pos[0-2] ori[3-5] vel[6-8] ang[9-11] acc[12-14] vang[15-17] bias[18-29] biasL[30-35] biasW[36-38] biasI[39-44]
    //      pose_lidar_robo[45-50] pose_imu_robo[51-56]]
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

    // displacement
    //       (   velocities  ) - (     bias       )                 
    Axdt = A*(x.block(6,0,6,1) - x.block(18,0,6,1))*dt;
    // position
    xp.block(0,0,3,1) = x.block(0,0,3,1) + Axdt.block(0,0,3,1);
    // orientation
    xp.block(3,0,3,1) = angularResidualVec(x.block(3,0,3,1) + Axdt.block(3,0,3,1));
    // linear velocity
    // v_k            = v_k-1            + (   aceleration    - bias_aceleration )
    xp.block(6,0,3,1) = x.block(6,0,3,1) + (x.block(12,0,3,1) - x.block(24,0,3,1))*dt;
    // angular velocity
    // w_k            = w_k-1            + (   ang accel       - bias_ang_accel )
    xp.block(9,0,3,1) = x.block(9,0,3,1) + (x.block(15,0,3,1) - x.block(27,0,3,1))*dt;
    // the rest of states
    xp.block(12,0,45,1) = x.block(12,0,45,1);

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

    return up;
}

//----------
// Jacobians
//----------
MatrixXd AdaptiveRobustEKF::jacobian_state(VectorXd x, double dt){
    Eigen::MatrixXd J(_N_STATES,_N_STATES);
    Eigen::VectorXd f0(_N_STATES), f1(_N_STATES), x_plus(_N_STATES);

    f0 = f_prediction_model(x, dt);

    double delta = 1e-4;
    for (size_t i = 0; i < _N_STATES; i++){
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
    for (size_t i = 0; i < _N_LIDAR; i++){
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
    for (size_t i = 0; i < _N_LIDAR; i++){
        ul_plus = ul;
        ul_plus(i) = ul_plus(i) + delta;

        f1 = indirect_odometry_measurement(u, ul_plus, dt);
    
        J.block(0,i,_N_LIDAR,1) = (f1 - f0)/delta;
        J.block(3,i,3,1) = angularResidualVec(f1.block(3,0,3,1) - f0.block(3,0,3,1))/delta;
    }

    return J;
}

Eigen::MatrixXd AdaptiveRobustEKF::analitycal_jacobin_state(const Eigen::VectorXd& x, double dt) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(_N_STATES, _N_STATES);

    // Estados atuais
    double phi = x(3), theta = x(4), psi = x(5);
    Eigen::Vector3d v_eff = x.block<3,1>(6,0) - x.block<3,1>(18,0); // v - b_v
    Eigen::Vector3d w_eff = x.block<3,1>(9,0) - x.block<3,1>(21,0); // w - b_w

    // --- BLOCO POSIÇÃO (0-2) ---
    // dp/d_euler
    F.block<3,1>(0,3) = dRv_dphi(phi, theta, psi, v_eff) * dt;
    F.block<3,1>(0,4) = dRv_dtheta(phi, theta, psi, v_eff) * dt;
    F.block<3,1>(0,5) = dRv_dpsi(phi, theta, psi, v_eff) * dt;
    // dp/dv e dp/db_v
    Eigen::Matrix3d R = eulerMatrix(x.block(3,0,3,1));
    F.block<3,3>(0,6) = R * dt;
    F.block<3,3>(0,18) = -R * dt;

    // --- BLOCO ORIENTAÇÃO (3-5) ---
    // d_euler/d_euler
    F.block<3,1>(3,3) += dJw_dphi(phi, theta, w_eff) * dt;
    F.block<3,1>(3,4) += dJw_dtheta(phi, theta, w_eff) * dt;
    // d_euler/dw e d_euler/db_w
    Eigen::Matrix3d J = eulerKinematicMatrix(phi, theta);
    F.block<3,3>(3,9) = J * dt;
    F.block<3,3>(3,21) = -J * dt;

    // --- BLOCO VELOCIDADES (6-11) ---
    // dv/da e dv/db_a
    F.block<3,3>(6,12) = Eigen::Matrix3d::Identity() * dt;
    F.block<3,3>(6,24) = -Eigen::Matrix3d::Identity() * dt;
    // dw/d_alpha e dw/db_alpha
    F.block<3,3>(9,15) = Eigen::Matrix3d::Identity() * dt;
    F.block<3,3>(9,27) = -Eigen::Matrix3d::Identity() * dt;

    // Os estados de bias de sensores (b_L, b_W, b_I) e calibração extrínseca (X_L^R, X_I^R)
    // permanecem constantes na predição (Identidade na diagonal), portanto não 
    // requerem termos adicionais fora da diagonal principal.

    return F;
}

Eigen::MatrixXd AdaptiveRobustEKF::calculateJacobianIMU(const Eigen::VectorXd& x) {
    // Inicializa a matriz com as dimensões parametrizadas
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(_N_IMU, _N_STATES);

    // Extração de parâmetros de calibração e estados dinâmicos
    Eigen::Vector3d w_eff = x.block<3,1>(9,0);   // omega_robot
    Eigen::Vector3d a_eff = x.block<3,1>(12,0);  // acc_robot
    Eigen::Vector3d wd_eff = x.block<3,1>(15,0) - x.block<3,1>(27,0); // alpha_robot - b_alpha
    
    Eigen::Vector3d p_ir = x.block<3,1>(51,0);   // Posição extrínseca IMU-Robô
    Eigen::Vector3d att_ir = x.block<3,1>(54,0); // Orientação extrínseca IMU-Robô
    
    Eigen::Matrix3d R_ir = eulerMatrix(att_ir);

    // --- BLOCO GYRO (Medidas 0, 1, 2) ---
    // d_gyro / d_omega_robot
    H.block<3,3>(0,9) = R_ir;
    // d_gyro / d_bias_gyro_imu (estados 39, 40, 41)
    H.block<3,3>(0,39) = Eigen::Matrix3d::Identity();
    // d_gyro / d_att_ir (Sensibilidade à orientação da calibração)
    H.block<3,1>(0,54) = dRv_dphi(att_ir(0), att_ir(1), att_ir(2), w_eff);
    H.block<3,1>(0,55) = dRv_dtheta(att_ir(0), att_ir(1), att_ir(2), w_eff);
    H.block<3,1>(0,56) = dRv_dpsi(att_ir(0), att_ir(1), att_ir(2), w_eff);

    // --- BLOCO ACCEL (Medidas 3, 4, 5) ---
    // d_acc / d_accel_robot
    H.block<3,3>(3,12) = R_ir;
    // d_acc / d_alpha_robot (Lever arm angular acceleration)
    H.block<3,3>(3,15) = R_ir * (-skewSymmetric(p_ir));
    // d_acc / d_omega_robot (Efeito centrípeto)
    H.block<3,3>(3,9) = R_ir * (skewSymmetric(w_eff) * (-skewSymmetric(p_ir)) - skewSymmetric(w_eff.cross(p_ir)));
    // d_acc / d_bias_acc_imu (estados 42, 43, 44)
    H.block<3,3>(3,42) = Eigen::Matrix3d::Identity();
    // d_acc / d_p_ir (Calibração de posição IMU-Robô)
    H.block<3,3>(3,51) = R_ir * (skewSymmetric(wd_eff) + skewSymmetric(w_eff) * skewSymmetric(w_eff));
    // d_acc / d_att_ir (Calibração de orientação IMU-Robô)
    Eigen::Vector3d acc_in_body = a_eff + wd_eff.cross(p_ir) + w_eff.cross(w_eff.cross(p_ir));
    H.block<3,1>(3,54) = dRv_dphi(att_ir(0), att_ir(1), att_ir(2), acc_in_body);
    H.block<3,1>(3,55) = dRv_dtheta(att_ir(0), att_ir(1), att_ir(2), acc_in_body);
    H.block<3,1>(3,56) = dRv_dpsi(att_ir(0), att_ir(1), att_ir(2), acc_in_body);

    return H;
}

Eigen::MatrixXd AdaptiveRobustEKF::calculateJacobianLidar(const Eigen::VectorXd& x) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(_N_LIDAR, _N_STATES);

    Eigen::Vector3d v_robot = x.block<3,1>(6,0);
    Eigen::Vector3d w_robot = x.block<3,1>(9,0);
    Eigen::Vector3d p_lr = x.block<3,1>(45,0);   // Posição extrínseca LiDAR-Robô
    Eigen::Vector3d att_lr = x.block<3,1>(48,0); // Orientação extrínseca LiDAR-Robô
    
    Eigen::Matrix3d R_lr = eulerMatrix(att_lr);

    // --- BLOCO VELOCIDADE LINEAR (Medidas 0, 1, 2) ---
    // d_v_lidar / d_v_robot
    H.block<3,3>(0,6) = R_lr;
    // d_v_lidar / d_w_robot (Braço de alavanca na velocidade)
    H.block<3,3>(0,9) = -R_lr * skewSymmetric(p_lr);
    // d_v_lidar / d_bias_lidar_linear (estados 30, 31, 32)
    H.block<3,3>(0,30) = Eigen::Matrix3d::Identity();
    // d_v_lidar / d_p_lr (Sensibilidade à posição da calibração)
    H.block<3,3>(0,45) = R_lr * skewSymmetric(w_robot);
    // d_v_lidar / d_att_lr (Sensibilidade à orientação da calibração)
    Eigen::Vector3d v_total_body = v_robot + w_robot.cross(p_lr);
    H.block<3,1>(0,48) = dRv_dphi(att_lr(0), att_lr(1), att_lr(2), v_total_body);
    H.block<3,1>(0,49) = dRv_dtheta(att_lr(0), att_lr(1), att_lr(2), v_total_body);
    H.block<3,1>(0,50) = dRv_dpsi(att_lr(0), att_lr(1), att_lr(2), v_total_body);

    // --- BLOCO VELOCIDADE ANGULAR (Medidas 3, 4, 5) ---
    // d_w_lidar / d_w_robot
    H.block<3,3>(3,9) = R_lr;
    // d_w_lidar / d_bias_lidar_angular (estados 33, 34, 35)
    H.block<3,3>(3,33) = Eigen::Matrix3d::Identity();
    // d_w_lidar / d_att_lr
    H.block<3,1>(3,48) = dRv_dphi(att_lr(0), att_lr(1), att_lr(2), w_robot);
    H.block<3,1>(3,49) = dRv_dtheta(att_lr(0), att_lr(1), att_lr(2), w_robot);
    H.block<3,1>(3,50) = dRv_dpsi(att_lr(0), att_lr(1), att_lr(2), w_robot);

    return H;
}

//--------------
// run
//--------------
void AdaptiveRobustEKF::run(){
    // rate
    ros::Rate r(freq);        

    double t_last = ros::Time::now().toSec();
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

    while (_running && ros::ok())
    {
        // prediction stage
        t_now = ros::Time::now().toSec();
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
        std::vector<bool> flags ={(_imuActivated && enableImu), (_wheelActivated && enableWheel), (_lidarActivated && enableLidar),
                                   (imu_fail_msg || !enableImu), (wheel_fail_msg || !enableWheel), (lidar_fail_msg || !enableLidar),
                                    firstData};
        // ROS_INFO("\033[1;32mAdaptive Filter:\033[0m Flags | Active: [IMU:%s, W:%s, L:%s] Fails: [I:%s, W:%s, L:%s] First: %s",
        //         flags[0] ? "ON" : "OFF", flags[1] ? "ON" : "OFF", flags[2] ? "ON" : "OFF",
        //         flags[3] ? "FAIL" : "OK", flags[4] ? "FAIL" : "OK", flags[5] ? "FAIL" : "OK", flags[6] ? "ON" : "OFF");

        // **************
        // test flag end
        // **************

        if (((_imuActivated && enableImu) || (_wheelActivated && enableWheel) || (_lidarActivated && enableLidar)) &&
            (!((imu_fail_msg || !enableImu) && (wheel_fail_msg || !enableWheel) && (lidar_fail_msg || !enableLidar))) && 
                firstData){

            // **************
            // tests - state
            // **************

            ROS_INFO_STREAM("\033[1;34m--- EKF STATE (X) ---\033[0m" << 
                        "\n  Pos   : " << _X.segment(0, 3).transpose() <<
                        "\n  Ori   : " << _X.segment(3, 3).transpose() <<
                        "\n  Vel   : " << _X.segment(6, 3).transpose() <<
                        "\n  AngV  : " << _X.segment(9, 3).transpose() <<
                        "\n  Acc   : " << _X.segment(12, 3).transpose() <<
                        "\n  VAng  : " << _X.segment(15, 3).transpose() <<
                        "\n  Bias  : " << _X.segment(18, 12).transpose() <<
                        "\n  BiasL : " << _X.segment(30, 6).transpose() <<
                        "\n  BiasW : " << _X.segment(36, 3).transpose() <<
                        "\n  BiasI : " << _X.segment(39, 6).transpose() <<
                        "\n  T_L/R : " << _X.segment(45, 6).transpose() <<
                        "\n  T_I/R : " << _X.segment(51, 6).transpose() <<
                        "\n\033[1;34m---------------------\033[0m");

            // **************
            // test state end
            // **************

            prediction_stage(dt_now);
            // ROS_INFO("\033[1;32mAdaptive Filter:\033[0m Prediction:");
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
    _E_imu = E_imu;
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
        double omegaz_wheel_odom = wheel_odom(1);
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