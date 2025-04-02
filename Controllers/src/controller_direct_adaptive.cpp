#include "controller_direct_adaptive.hpp"

namespace robust_controller {

controller_direct_adaptive::controller_direct_adaptive(const std::shared_ptr<model>& modelPtr_in) {
    mbdPtr_ = std::make_shared<MultiBodyDynamicsYphi>(modelPtr_in);
    NB = modelPtr_in->NB;

    // Initialize control parameters
    params.Kd = 10 * VecX::Ones(NB); // Derivative gain  now constants but can be esitiamte H * lambda
    params.Kr = 5 * VecX::Ones(NB); // Reference gain

    params.beta = 1.0; // Adaptive gain
    params.delta = 20.0; // Adaptive gain
    params.eta = 0.1; // Adaptive gain
    params.alpha = 0.4; // Forgetting factor

    params.dt = 0.001; // Time step

    Gamma = MatX::Identity(10, 10); // Adaptation gain. PSD matrix
}

controller_direct_adaptive::controller_direct_adaptive(
    const std::shared_ptr<model>& modelPtr_in,
    const parameters& params_in) :
    params(params_in) {
    mbdPtr_ = std::make_shared<MultiBodyDynamicsYphi>(modelPtr_in);
    NB = modelPtr_in->NB;

    if (params.Kr.size() != NB ||
        params.Kd.size() != NB) {
        throw std::invalid_argument("controller_direct_adaptive.cpp: Kr or Kd size mismatch!");
    }
}

Eigen::VectorXd controller_direct_adaptive::update(
    const VecX& q,  
    const VecX& q_d, 
    const VecX& qd, 
    const VecX& qd_d, 
    const VecX& qd_dd
) {
    // Step 1: Compute tracking errors
    VecX q_diff = controller_utils::clamp(qd - q);
    VecX q_d_diff = qd_d - q_d;
    
    // Step 2: Compute sliding variable
    const VecX qa_d = qd_d + params.Kr.cwiseProduct(q_diff);
    const VecX qa_dd = qd_dd + params.Kr.cwiseProduct(q_d_diff);
    const VecX s = q_d_diff + params.Kr.cwiseProduct(q_diff);

    // Step 3: Feedforward compensation
    mbdPtr_->rnea(q, q_d, qa_d, qa_dd);
    VecX tau_feedforward = mbdPtr_->tau;  // Feedforward torque

    // Step 4: PD feedback control
    VecX tau_feedback_1 = params.Kd.cwiseProduct(s);
    VecX tau_feedback_2 = params.beta * (params.delta + params.eta) * s.cwiseSign();

    // Step 5: Adaptive parameter update
    const MatX& Y_end = mbdPtr_->Y.rightCols(10);

    // MatX dGamma_dt = Gamma - (Gamma * Y_end * Y_end.transpose() * Gamma) / 
    //     (1.0 + params.alpha * (Y_end.transpose() * Gamma * Y_end).trace());
    // Gamma += dGamma_dt * params.dt;

    // Step 7: Compute total torque
    VecX total_tau = tau_feedforward + tau_feedback_1 + tau_feedback_2;

    return total_tau;
}

}; // namespace robust_controller
