#ifndef ALPHA_BETA_FILTER_H
#define ALPHA_BETA_FILTER_H

#include <Eigen/Dense>

class alphaBetaFilter {
public:
    // Construtor padrão (permite declarar no .h sem inicializar)
    alphaBetaFilter();

    /**
     * @brief Configura ou reinicia o filtro
     * @param dim Dimensão do vetor
     * @param alpha Ganho de estado
     * @param beta Ganho de taxa
     */
    void init(int dim, double alpha, double beta);

    /**
     * @brief Atualiza o filtro e retorna a taxa (velocidade) filtrada
     */
    Eigen::VectorXd update(const Eigen::VectorXd& z_meas, double dt);

    void reset();

private:
    int _dim;
    double _alpha;
    double _beta;
    bool _is_configured; // Garante que o init foi chamado
    bool _is_init;       // Flag para o primeiro dado recebido

    Eigen::VectorXd _x_hat; 
    Eigen::VectorXd _v_hat; 
};

#endif