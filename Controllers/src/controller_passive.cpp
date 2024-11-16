#include "controller_passive.hpp"

namespace robust_controller {

controller_passive::controller_passive(const std::shared_ptr<model>& modelPtr_in) {
    mbdPtr_ = std::make_shared<MultiBodyDynamicsPinocchio>(modelPtr_in);
    NB = modelPtr_in->NB;
    params.Kr = 5 * Eigen::VectorXd::Ones(NB);
    params.Kp = 20 * Eigen::VectorXd::Ones(NB);
}

controller_passive::controller_passive(
    const std::shared_ptr<model>& modelPtr_in,
    const parameters& params_in) :
    params(params_in) {
    mbdPtr_ = std::make_shared<MultiBodyDynamicsPinocchio>(modelPtr_in);
    NB = modelPtr_in->NB;
    if (params.Kr.size() != NB ||
        params.Kp.size() != NB) {
        throw std::invalid_argument("controller_passive.cpp: controller_passive(): Kr or Kp or Kd size mismatch!");
    }
}

Eigen::VectorXd controller_passive::update(
    const Eigen::VectorXd& q,  
    const Eigen::VectorXd& q_d, 
    const Eigen::VectorXd& qd, 
    const Eigen::VectorXd& qd_d, 
    const Eigen::VectorXd& qd_dd
) {
    // Step 1, calculate reference terms
    q_diff = controller_utils::clamp(qd - q);
    q_d_diff = qd_d - q_d;
    
    const VecX qa_d = qd_d + params.Kr.cwiseProduct(q_diff);
    const VecX qa_dd = qd_dd + params.Kr.cwiseProduct(q_d_diff);

    // Step 2, calculate nominal torque from passivity RNEA
    mbdPtr_->rnea(q, q_d, qa_d, qa_dd);

    // Step 3, calculate stabilizing torque
    const VecX r = q_d_diff + params.Kr.cwiseProduct(q_diff);
    const VecX v = params.Kp.cwiseProduct(r);

    return mbdPtr_->tau + v;
}

}; // namespace robust_controller