#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include "model.hpp"
#include "utils.hpp"

namespace robust_controller {

/**
 * @brief Class representing a pure abstract instance of multi-body dynamics.
 * 
 * This class represents the dynamics of a multi-body system, including
 * methods for computing the dynamics and updating relevant variables.
 */
class MultiBodyDynamics {
public:
    using VecX = Eigen::VectorXd; //!< Type alias for Eigen dynamic vector.
    using MatX = Eigen::MatrixXd; //!< Type alias for Eigen dynamic matrix.

    std::shared_ptr<model> modelPtr_; //!< Pointer to the robot model.

    int NB = 0; //!< Number of joints.

    VecX tau; //!< Joint torque vector.
    VecX tau_inf; //!< Lower bound of joint torques.
    VecX tau_sup; //!< Upper bound of joint torques.

    VecX q_copy; //!< Copy of joint position vector that just has been evaluated.
    VecX q_d_copy; //!< Copy of joint velocity vector that just has been evaluated.
    VecX q_aux_d_copy; //!< Copy of auxiliary joint velocity vector that just has been evaluated.
    VecX q_aux_dd_copy; //!< Copy of auxiliary joint acceleration vector that just has been evaluated.

    /**
     * @brief Default constructor.
     */
    MultiBodyDynamics() = default;

    /**
     * @brief Constructor with model pointer.
     * 
     * @param modelPtr_in Pointer to the robot model.
     */
    MultiBodyDynamics(const std::shared_ptr<model>& modelPtr_in) :
        modelPtr_(modelPtr_in) {
        NB = modelPtr_->NB;
        tau.resize(NB);
    }

    /**
     * @brief Destructor.
     */
    ~MultiBodyDynamics() = default;

    /**
     * @brief Return the actuated dimension of the system.
     * 
     * For system without constraints, the actual system dimension is the number of joints.
     * For systems with constraints, the actual system dimension is the number of actuated joints.
     */
    virtual int actual_system_dimension() const = 0;

    /**
     * @brief Check if the dynamics has been evaluated for the given state.
     * 
     * This function checks if the dynamics has been evaluated for the given state.
     * 
     * @param q Joint position vector.
     * @param q_d Joint velocity vector.
     * @param q_aux_d Auxiliary joint velocity vector.
     * @param q_aux_dd Auxiliary joint acceleration vector.
     */
    bool has_evaluated(
        const VecX& q, 
        const VecX& q_d, 
        const VecX& q_aux_d,
        const VecX& q_aux_dd
    ) {
        bool res = 
            dynamics_utils::ifTwoVectorEqual(q, q_copy) && 
            dynamics_utils::ifTwoVectorEqual(q_d, q_d_copy) && 
            dynamics_utils::ifTwoVectorEqual(q_aux_d, q_aux_d_copy) && 
            dynamics_utils::ifTwoVectorEqual(q_aux_dd, q_aux_dd_copy);
        
        if (!res) {
            q_copy = q;
            q_d_copy = q_d;
            q_aux_d_copy = q_aux_d;
            q_aux_dd_copy = q_aux_dd;
        }

        return res;
    };

    /**
     * @brief Compute inverse dynamics.
     * 
     * This function computes the Recursive Newton-Euler Algorithm (RNEA) and updates
     * joint torques tau.
     * 
     * @param q Joint position vector.
     * @param q_d Joint velocity vector.
     * @param q_aux_d Auxiliary joint velocity vector.
     * @param q_aux_dd Auxiliary joint acceleration vector.
     * @param add_gravity Flag indicating whether to add gravity effects (default: true).
     */
    virtual void rnea(
        const VecX& q, 
        const VecX& q_d, 
        const VecX& q_aux_d,
        const VecX& q_aux_dd,
        const bool add_gravity = true
    ) = 0;

    /**
     * @brief Compute inverse dynamics for an interval model.
     * 
     * This function computes the Recursive Newton-Euler Algorithm (RNEA) and updates
     * joint torques tau_inf and tau_sup based on a model 
     * where inertial parameters are defined over an interval.
     * 
     * @param q Joint position vector.
     * @param q_d Joint velocity vector.
     * @param q_aux_d Auxiliary joint velocity vector.
     * @param q_aux_dd Auxiliary joint acceleration vector.
     * @param add_gravity Flag indicating whether to add gravity effects (default: true).
     */
    virtual void rnea_interval(
        const VecX& q, 
        const VecX& q_d, 
        const VecX& q_aux_d,
        const VecX& q_aux_dd,
        const bool add_gravity = true
    ) = 0;
};

}; // namespace robust_controller

#endif // DYNAMICS_HPP
