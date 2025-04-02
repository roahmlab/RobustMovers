#include "controller_adaptive.hpp"

namespace robust_controller {

controller_adaptive::controller_adaptive(const std::shared_ptr<model>& modelPtr_in) {
    mbdPtr_ = std::make_shared<MultiBodyDynamicsYphi>(modelPtr_in);
    NB = modelPtr_in->NB;

    // Initialize control parameters
    // params.Kp = 20 * Eigen::VectorXd::Ones(NB); // PD gain
    params.Kd = 10 * Eigen::VectorXd::Ones(NB); // Derivative gain  now constants but can be esitiamte H * lambda
    params.Gamma = Eigen::MatrixXd::Identity(NB*10, NB*10) * 0.1; // Adaptation gain. PSD matrix
    params.Kr = 5 * Eigen::VectorXd::Ones(NB); // Reference gain
    params.dt = 0.001; // Time step
}

controller_adaptive::controller_adaptive(
    const std::shared_ptr<model>& modelPtr_in,
    const parameters& params_in) :
    params(params_in) {
    mbdPtr_ = std::make_shared<MultiBodyDynamicsYphi>(modelPtr_in);
    NB = modelPtr_in->NB;

    if (params.Kr.size() != NB ||
        params.Gamma.rows() != 10 * NB || params.Gamma.cols() != 10 * NB || !isPSD(params.Gamma)) {
        throw std::invalid_argument("controller_adaptive.cpp: Kr or Kd or Gamma size mismatch!");
    }
}

Eigen::VectorXd controller_adaptive::update(
    const Eigen::VectorXd& q,  
    const Eigen::VectorXd& q_d, 
    const Eigen::VectorXd& qd, 
    const Eigen::VectorXd& qd_d, 
    const Eigen::VectorXd& qd_dd
) {
    // Step 1: Compute tracking errors
    Eigen::VectorXd q_diff = controller_utils::clamp(qd - q);
    Eigen::VectorXd q_d_diff = qd_d - q_d;
    
    // Step 2: Compute sliding variable
    const VecX qa_d = qd_d + params.Kr.cwiseProduct(q_diff);
    const VecX qa_dd = qd_dd + params.Kr.cwiseProduct(q_d_diff);
    const VecX s = q_d_diff + params.Kr.cwiseProduct(q_diff);

    // Step 3: Feedforward compensation
    mbdPtr_->rnea(q, q_d, qa_d, qa_dd);
    Eigen::VectorXd tau_feedforward = mbdPtr_->tau;  // Feedforward torque

    // Step 4: PD feedback control
    Eigen::VectorXd tau_feedback = params.Kd.cwiseProduct(s);
    
    // Step 5: Adaptive parameter update
    Eigen::MatrixXd Y_end = mbdPtr_->Y.rightCols(10); // End parameters

    double adaptive_scaling = 1 / (1.0 + s.norm() * s.norm()); // Dynamic scaling
    Eigen::MatrixXd Gamma = adaptive_scaling * params.Gamma;
    mbdPtr_->modelPtr_->phi.tail(10) += Gamma.block(60, 60, 10, 10) * Y_end.transpose() * s * params.dt; // Update end parameters

    // step 6 calculate uncertainty torque
    Eigen::VectorXd tau_max_error(NB);
    tau_max_error = Eigen::VectorXd::Ones(NB) * 0.01;

    Eigen::VectorXd phi_uncertainty_max = 0.05 * mbdPtr_->modelPtr_->phi.head(10* (NB-1)).cwiseAbs();
    Eigen::VectorXd eta = Eigen::VectorXd::Ones(NB) * 0.01;

    Eigen::MatrixXd Y_fix = mbdPtr_->Y.leftCols(10*(NB-1)).cwiseAbs();

    Eigen::VectorXd k = Y_fix * phi_uncertainty_max + eta + tau_max_error;
    Eigen::VectorXd tau_uncertainty = k.cwiseProduct(s.cwiseSign());

    // Step 7: Compute total torque
    Eigen::VectorXd total_tau = tau_feedforward + tau_feedback + tau_uncertainty;
    
    return total_tau;
}

bool controller_adaptive::isPSD(const Eigen::MatrixXd& M) {
    if(M != M.transpose()){
        throw std::invalid_argument("Matrix is not symmetric");
        return false;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(M);
    if (es.eigenvalues().minCoeff() <= 0){
        throw std::invalid_argument("Matrix is not positive definite");
        return false;
    }
    return true;
}

}; // namespace robust_controller
