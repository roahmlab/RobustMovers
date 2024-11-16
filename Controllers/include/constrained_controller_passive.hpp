#ifndef CONSTRAINED_CONTROLLER_PASSIVE_HPP
#define CONSTRAINED_CONTROLLER_PASSIVE_HPP

#include "constrained_controller.hpp"

namespace robust_controller {

/**
 * @brief Class representing a passivity-based constrained controller interface.
 * 
 * This class implements an interface of the passivity-based constrained controller for a multi-body dynamics system.
 */
class constrained_controller_passive : public constrained_controller {
public:
    using VecX = Eigen::VectorXd; //!< Type alias for Eigen dynamic vector.

    VecX q_act_diff; //!< Joint position difference.
    VecX q_act_d_diff; //!< Joint velocity difference.

    /**
     * @brief Struct representing controller parameters.
     */
    struct parameters {
        VecX Kr; //!< Feedback gain values.
        VecX Kp; //!< Proportional gain values.
    };

    parameters params; //!< Controller parameters.

    /**
     * @brief Default constructor.
     */
    constrained_controller_passive() = default;
    
    /**
     * @brief Destructor.
     */
    ~constrained_controller_passive() = default;

    /**
     * @brief Update the constrained_controller.
     * 
     * This function updates the constrained_controller state based on current joint states.
     * 
     * @param q Current joint positions.
     * @param q_d Current joint positions.
     * @param qd_act Target actuated joint positions.
     * @param qd_act_d Target actuated joint velocities.
     * @param qd_act_dd Target actuated joint accelerations.
     * @return VecX Control input.
     */
    virtual VecX update(
        const VecX& q,  
        const VecX& q_d, 
        const VecX& qd_act, 
        const VecX& qd_act_d, 
        const VecX& qd_act_dd
    ) override {
        // Step 1, calculate reference terms
        const VecX q_act = cmbdPtr_->get_actuated(q);
        const VecX q_act_d = cmbdPtr_->get_actuated(q_d);

        q_act_diff = controller_utils::clamp(qd_act - q_act);
        q_act_d_diff = qd_act_d - q_act_d;

        const VecX qa_d = qd_act_d + params.Kr.cwiseProduct(q_act_diff);
        const VecX qa_dd = qd_act_dd + params.Kr.cwiseProduct(q_act_d_diff);
        const VecX r = q_act_d_diff + params.Kr.cwiseProduct(q_act_diff);

        // Step 2, fill in dependent joint states
        VecX q_aux_d_full = VecX::Zero(NB);
        VecX q_aux_dd_full = VecX::Zero(NB);

        // the following are just placeholders
        // since for constrained system, the target values in operational space are just 0
        const VecX c_target = VecX::Zero(opIKPtr_->NO);
        const VecX c_d_target = VecX::Zero(opIKPtr_->NO);
        const VecX c_dd_target = VecX::Zero(opIKPtr_->NO);

        opIKPtr_->fill_independent(q_aux_d_full, qa_d);
        opIKPtr_->fill_independent(q_aux_dd_full, qa_dd);

        VecX q_copy = q; // fill_dependent requires a nonconst reference
        opIKPtr_->fill_dependent(
            q_copy, q_aux_d_full, q_aux_dd_full, 
            c_target, c_d_target, c_dd_target,
            true);

        // Step 3, calculate nominal control input
        cmbdPtr_->rnea(q, q_d, q_aux_d_full, q_aux_dd_full);

        // Step 4, calculate stabilizing torque
        const VecX r_act = q_act_d_diff + params.Kr.cwiseProduct(q_act_diff);
        const VecX v = params.Kp.cwiseProduct(r_act);

        return cmbdPtr_->tau_constrained + v;
    };
};

}; // namespace robust_controller

#endif // CONSTRAINED_CONTROLLER_PASSIVE_HPP