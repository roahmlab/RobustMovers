#include "dynamics_pinocchio.hpp"

namespace robust_controller {

void MultiBodyDynamicsPinocchio::rnea(
    const VecX& q, 
    const VecX& q_d, 
    const VecX& q_aux_d,
    const VecX& q_aux_dd,
    const bool add_gravity
) {
    if (q.size() != NB || 
        q_d.size() != NB || 
        q_aux_d.size() != NB || 
        q_aux_dd.size() != NB) {
        throw std::invalid_argument("Yphi_passive: input vector size mismatch!");
    }

    if (has_evaluated(q, q_d, q_aux_d, q_aux_dd)) {
        return;
    }

    auto& model_pinocchio = modelPtr_->model_pinocchio;
    auto& data_pinocchio = modelPtr_->data_pinocchio;

    const auto original_gravity = model_pinocchio.gravity;
    if (!add_gravity) {
        model_pinocchio.gravity.setZero();
    }

    // Since crba returns only the upper triangular part of M, we need to symmetrize it
    MatX M = pinocchio::crba(model_pinocchio, data_pinocchio, q);
    for (int i = 0; i < M.rows(); i++) {
        for (int j = i + 1; j < M.cols(); j++) {
            M(j, i) = M(i, j);
        }
    }

    pinocchio::computeCoriolisMatrix(model_pinocchio, data_pinocchio, q, q_d);
    const MatX C = data_pinocchio.C;

    pinocchio::computeGeneralizedGravity(model_pinocchio, data_pinocchio, q);
    const VecX g = data_pinocchio.g;

    tau = M * q_aux_dd + C * q_aux_d + g;

    model_pinocchio.gravity = original_gravity;
}

void MultiBodyDynamicsPinocchio::rnea_interval(
    const VecX& q, 
    const VecX& q_d, 
    const VecX& q_aux_d,
    const VecX& q_aux_dd,
    const bool add_gravity
) {
    throw std::runtime_error("rnea_interval not implemented for MultiBodyDynamicsPinocchio!");
}

void MultiBodyDynamicsPinocchio::Yphi_passive(
    const VecX& q, 
    const VecX& q_d, 
    const VecX& q_aux_d,
    const VecX& q_dd, 
    const bool add_gravity
) {
    throw std::runtime_error("Yphi_passive not implemented for MultiBodyDynamicsPinocchio!");
}

}; // namespace robust_controller