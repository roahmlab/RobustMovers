#include "controller_armour.hpp"

namespace robust_controller {

controller_armour::controller_armour(const std::shared_ptr<model>& modelPtr_in) {
    mbdPtr_ = std::make_shared<MultiBodyDynamicsYphi>(modelPtr_in);
    params.Kr = 5 * Eigen::VectorXd::Ones(NB);
}

controller_armour::controller_armour(
    const std::shared_ptr<model>& modelPtr_in,
    const parameters& params_in) :
    params(params_in) {
    mbdPtr_ = std::make_shared<MultiBodyDynamicsYphi>(modelPtr_in);
    NB = modelPtr_in->NB;
    if (params.Kr.size() != NB) {
        throw std::invalid_argument("controller_armour.cpp: controller_armour(): Kr size mismatch!");
    }
}

controller_armour::controller_armour(
    const std::shared_ptr<model>& modelPtr_in,
    const parameters& params_in, 
    const double phi_eps_in) :
    params(params_in) {
    mbdPtr_ = std::make_shared<MultiBodyDynamicsYphi>(modelPtr_in, phi_eps_in);
    NB = modelPtr_in->NB;

    if (params.Kr.size() != NB) {
        throw std::invalid_argument("controller_armour.cpp: controller_armour(): Kr size mismatch!");
    }
}

Eigen::VectorXd controller_armour::update(
    const VecX& q,  
    const VecX& q_d, 
    const VecX& qd, 
    const VecX& qd_d, 
    const VecX& qd_dd) {
    // Step 1, calculate reference terms
    const VecX q_diff = controller_utils::clamp(qd - q);
    const VecX q_d_diff = qd_d - q_d;
    
    const VecX qa_d = qd_d + params.Kr.cwiseProduct(q_diff);
    const VecX qa_dd = qd_dd + params.Kr.cwiseProduct(q_d_diff);
    const VecX r = q_d_diff + params.Kr.cwiseProduct(q_diff);

    // Step 2, calculate upper bound of Lyapunov function
    double r_norm = r.norm();
    double V_sup = 0;
        
    // Only compute robust input when r is large enough to avoid numerical issues
    if (r_norm > params.r_norm_threshold) {
        // Step 5a, calculate V = 0.5 * r' * M * r
        q_d_zero = VecX::Zero(q.size());
        mbdPtr_->rnea_interval(q, q_d_zero, q_d_zero, r, false);

        // dot product between r and M * r
        for (int i = 0; i < mbdPtr_->tau.size(); i++) {
            if (r(i) > 0) {
                V_sup += 0.5 * r(i) * mbdPtr_->tau_sup(i);
            }
            else {
                V_sup += 0.5 * r(i) * mbdPtr_->tau_inf(i);
            }
        }
    }
    // else {
    //     v is just zero vector then
    // }

    // Step 3, calculate nominal torque from passivity RNEA
    mbdPtr_->rnea(q, q_d, qa_d, qa_dd);

    // Step 4, calculate interval torque from passivity RNEA
    mbdPtr_->rnea_interval(q, q_d, qa_d, qa_dd);

    // sanity check for size since constrained system might be involved
    if (mbdPtr_->tau.size() != mbdPtr_->tau_sup.size() || 
        mbdPtr_->tau.size() != mbdPtr_->tau_inf.size()) {
        throw std::runtime_error("controller_armour.cpp: update(): Size mismatch in nominal torque and interval torque!");
    }

    // Interval check
    for (size_t i = 0; i < mbdPtr_->tau.size(); i++) {
        if (mbdPtr_->tau(i) > mbdPtr_->tau_sup(i) || mbdPtr_->tau(i) < mbdPtr_->tau_inf(i)) {
            std::cerr << "tau: "     << mbdPtr_->tau(i)     << std::endl;
            std::cerr << "tau_sup: " << mbdPtr_->tau_sup(i) << std::endl;
            std::cerr << "tau_inf: " << mbdPtr_->tau_inf(i) << std::endl;
            throw std::runtime_error("controller_armour.cpp: update(): Nominal torque output falls outside interval output!");
        }
    }

    // Step 5, calculate error bound
    VecX bound(mbdPtr_->tau.size());
    for (int i = 0; i < mbdPtr_->tau.size(); i++) {
        bound(i) = std::max(
            mbdPtr_->tau_sup(i) - mbdPtr_->tau(i), 
            mbdPtr_->tau(i)     - mbdPtr_->tau_inf(i));
    }

    // Step 6, calculate robust input
    v = VecX::Zero(mbdPtr_->tau.size());

    // Only compute robust input when r is large enough to avoid numerical issues
    if (r_norm > params.r_norm_threshold) {
        // Step 5b, compute robust input
        double h = params.V_max - V_sup;
        double lambda = std::max(
            0.0, 
            -params.alpha * h / r_norm + bound.norm());

        v = lambda * r / r_norm;
    }
    
    return  mbdPtr_->tau + v;
}

}; // namespace robust_controller_armour
