#include "operational_space_inverse_kinematics.hpp"

namespace robust_controller {

OperationalSpaceInverseKinematics::OperationalSpaceInverseKinematics(
    const int NB_in, 
    const int NO_in) :
    NB(NB_in),
    NO(NO_in) {
    c = VecX::Zero(NO);
    J = MatX::Zero(NO, NB);
    J_d = MatX::Zero(NO, NB);
}

OperationalSpaceInverseKinematics::OperationalSpaceInverseKinematics(
    const std::shared_ptr<model> modelPtr_in,
    const int NO_in) :
    modelPtr_(modelPtr_in),
    NO(NO_in) {
    NB = modelPtr_->NB;
    c = VecX::Zero(NO);
    J = MatX::Zero(NO, NB);
    J_d = MatX::Zero(NO, NB);
};

void OperationalSpaceInverseKinematics::fill_dependent(
    VecX& q_full, 
    VecX& q_d_full,
    const VecX& c_target,
    const VecX& c_d_target,
    bool q_already_filled) {
    if (q_full.size() != NB || 
        q_d_full.size() != NB) {
        throw std::invalid_argument("fill_dependent: input vector size mismatch!");
    }

    if (c_target.size() != NO || 
        c_d_target.size() != NO) {
        throw std::invalid_argument("fill_dependent: target vector size mismatch!");
    }

    // fill in dependent part of the position
    if (!q_already_filled) {
        fill_dependent(q_full, c_target);
    }

    // fill in dependent part of the velocity
    get_J(q_full);
    J_qr = Eigen::ColPivHouseholderQR<MatX>(J);

    if (J_qr.rank() < NO) {
        throw std::runtime_error("fill_dependent: Jacobian is not full rank!");
    }

    q_d_full = J_qr.solve(c_d_target);
}

void OperationalSpaceInverseKinematics::fill_dependent(
    VecX& q_full, 
    VecX& q_d_full, 
    VecX& q_dd_full,
    const VecX& c_target,
    const VecX& c_d_target,
    const VecX& c_dd_target,
    bool q_already_filled) {
    if (q_full.size() != NB || 
        q_d_full.size() != NB || 
        q_dd_full.size() != NB) {
        throw std::invalid_argument("fill_dependent: input vector size mismatch!");
    }

    if (c_target.size() != NO || 
        c_d_target.size() != NO || 
        c_dd_target.size() != NO) {
        throw std::invalid_argument("fill_dependent: target vector size mismatch!");
    }

    // fill in dependent part of the position
    if (!q_already_filled) {
        fill_dependent(q_full, c_target);
    }

    // fill in dependent part of the velocity
    get_J(q_full);
    J_qr = Eigen::ColPivHouseholderQR<MatX>(J);

    if (J_qr.rank() < NO) {
        throw std::runtime_error("fill_dependent: Jacobian is not full rank!");
    }

    q_d_full = J_qr.solve(c_d_target);

    // fill in dependent part of the acceleration
    get_J_d(q_full, q_d_full);
    q_dd_full = J_qr.solve(c_dd_target - J_d * q_d_full);
}

}; // namespace robust_controller
