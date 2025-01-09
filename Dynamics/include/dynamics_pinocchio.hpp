#ifndef DYNAMICS_PINOCCHIO_HPP
#define DYNAMICS_PINOCCHIO_HPP

#include "dynamics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"

namespace robust_controller {

/**
 * @brief Class representing multi-body dynamics.
 * 
 * This class represents the dynamics of a multi-body system, including
 * methods for computing the dynamics and updating relevant variables.
 */
class MultiBodyDynamicsPinocchio : public MultiBodyDynamics {
public:
    using VecX = Eigen::VectorXd; //!< Type alias for Eigen dynamic vector.
    using MatX = Eigen::MatrixXd; //!< Type alias for Eigen dynamic matrix.

    /**
     * @brief Default constructor.
     */
    MultiBodyDynamicsPinocchio() = default;

    /**
     * @brief Constructor with model pointer.
     * 
     * @param modelPtr_in Pointer to the robot model.
     */
    MultiBodyDynamicsPinocchio(const std::shared_ptr<model>& modelPtr_in) :
        MultiBodyDynamics(modelPtr_in) {}

    /**
     * @brief Destructor.
     */
    ~MultiBodyDynamicsPinocchio() = default;

    /**
     * @brief Return the actuated dimension of the system.
     * 
     * For system without constraints, the actual system dimension is the number of joints.
     */
    virtual int actual_system_dimension() const final override {
        return NB;
    }

    /**
     * @brief Compute RNEA and update dynamics matrices.
     * 
     * This function computes the Recursive Newton-Euler Algorithm (RNEA) and updates
     * the dynamics matrices Y, Yfull, and joint torques tau.
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
    ) final override;

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
    ) final override;

    virtual void Yphi_passive(
        const VecX& q, 
        const VecX& q_d, 
        const VecX& q_aux_d,
        const VecX& q_aux__dd, 
        const bool add_gravity = true
    ) override;

};

}; // namespace robust_controller

#endif // DYNAMICS_PINOCCHIO_HPP
