#include <adaptive_odom_filter/alphaBetaFilter.h>

alphaBetaFilter::alphaBetaFilter() 
    : _dim(0), _alpha(0.0), _beta(0.0), _is_configured(false), _is_init(false) {
    // Objeto nasce vazio
}

void alphaBetaFilter::init(int dim, double alpha, double beta) {
    _dim = dim;
    _alpha = alpha;
    _beta = beta;
    _x_hat = Eigen::VectorXd::Zero(dim);
    _v_hat = Eigen::VectorXd::Zero(dim);
    _is_configured = true;
    _is_init = false; // Reinicia o histórico
}

void alphaBetaFilter::reset() {
    _is_init = false;
}

Eigen::VectorXd alphaBetaFilter::update(const Eigen::VectorXd& z_meas, double dt) {
    // Se não foi configurado ou dt for inválido, retorna zeros
    if (!_is_configured || dt <= 0.0) return Eigen::VectorXd::Zero(_dim);

    if (!_is_init) {
        _x_hat = z_meas;
        _v_hat.setZero();
        _is_init = true;
        return _v_hat;
    }

    // Algoritmo Alpha-Beta
    Eigen::VectorXd x_pred = _x_hat + _v_hat * dt;
    Eigen::VectorXd residual = z_meas - x_pred;

    _x_hat = x_pred + _alpha * residual;
    _v_hat = _v_hat + (_beta / dt) * residual;

    return _x_hat;
}