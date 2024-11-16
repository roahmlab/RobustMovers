#ifndef OPERATIONAL_SPACE_IK_HPP
#define OPERATIONAL_SPACE_IK_HPP

#include "dynamics.hpp"

namespace robust_controller {

/**
 * @brief Class for constrained multi-body dynamics.
 */
class OperationalSpaceInverseKinematics {
public:
    using VecX = Eigen::VectorXd; //!< Type alias for Eigen dynamic vector.
    using MatX = Eigen::MatrixXd; //!< Type alias for Eigen dynamic matrix.

    std::shared_ptr<model> modelPtr_; //!< Pointer to the robot model.

    int NB = 0; ///< Number of joints.
    int NO = 0; ///< Number of operational space dimensions.

    // VecX q_full_copy; ///< copy of the full joint position
    // VecX q_d_full_copy; ///< copy of the full joint velocity
    // VecX q_dd_full_copy; ///< copy of the full joint acceleration

    // constraints related variables
    VecX c; ///< Operational space target value
    MatX J; ///< Operational space Jacobian
    Eigen::ColPivHouseholderQR<MatX> J_qr; ///< QR decomposition of operational space Jacobian
    MatX J_d; ///< Operational space Jacobian time derivative

    /**
     * @brief Default constructor.
     */
    OperationalSpaceInverseKinematics() = default;

    /**
     * @brief Constructor with number of independent joints and operational space dimensions.
     * 
     * @param NB_in Number of independent joints.
     * @param NO_in Number of operational space dimensions.
     */
    OperationalSpaceInverseKinematics(
        const int NB_in, 
        const int NO_in);

    /**
     * @brief Constructor with number of independent joints and dynamics pointer.
     * 
     * @param modelPtr_in Pointer to the robot model.
     * @param NO_in Number of operational space dimensions.
     */
    OperationalSpaceInverseKinematics(
        const std::shared_ptr<model> modelPtr_in,
        const int NO_in);

    /**
     * @brief Destructor.
     */
    ~OperationalSpaceInverseKinematics() = default;

    /**
     * @brief Get the operational space target value.
     * 
     * This function computes the operational space target value.
     * 
     * @param q Joint position vector.
     */
    virtual void get_c(const VecX& q) = 0;

    /**
     * @brief Get the operational space Jacobian.
     *  
     * This function computes the operational space Jacobian.
     * 
     * @param q Joint position vector.
     */
    virtual void get_J(const VecX& q) = 0;

    /**
     * @brief Get the operational space Jacobian time derivative.
     *  
     * This function computes the operational space Jacobian time derivative.
     * 
     * @param q Joint position vector.
     * @param q_d Joint velocity vector.
     */
    virtual void get_J_d(
        const VecX& q, 
        const VecX& q_d
    ) = 0;

    /**
     * @brief Fill in the independent part of a vector.
     * 
     * This function fills in the independent part of a vector.
     * This vector could be joint positions, velocities, accelerations, etc.
     * 
     * @param q_full Full vector.
     * @param q_independent Independent part of vector.
     */
    virtual void fill_independent(
        VecX& q_full,
        const VecX& q_independent
    ) const = 0;

    /**
     * @brief Fill in the dependent part of the joint positions (inverse kinematics).
     * 
     * This function fills in the dependent part of the joint positions (inverse kinematics)
     * so that the full joint positions satisfy the target values in operational space.
     * It assumes the independent part is already filled.
     * 
     * @param q_full Full joint position vector.
     * @param c_target Target operational space value.
     */
    virtual void fill_dependent(
        VecX& q_full,
        const VecX& c_target
    ) = 0;

    /**
     * @brief Fill in the dependent part of the joint positions (inverse kinematics).
     * 
     * This function fills in the dependent part of the joint positions (inverse kinematics)
     * so that the full joint positions, velocities satisfy the target values in operational space.
     * It assumes the independent part is already filled.
     * 
     * @param q_full Full joint position vector.
     * @param q_d_full Full joint velocity vector.
     * @param c_target Target operational space value.
     * @param c_d_target Target operational space velocity.
     * @param q_already_filled Flag indicating whether the independent part is already filled.
     */
    virtual void fill_dependent(
        VecX& q_full, 
        VecX& q_d_full,
        const VecX& c_target,
        const VecX& c_d_target,
        bool q_already_filled = false);

    /**
     * @brief Fill in the dependent part of the joint positions (inverse kinematics).
     * 
     * This function fills in the dependent part of the joint positions (inverse kinematics)
     * so that the full joint positions, velocities, accelerations satisfy the target values in operational space.
     * It assumes the independent part is already filled.
     * 
     * @param q_full Full joint position vector.
     * @param q_d_full Full joint velocity vector.
     * @param q_dd_full Full joint acceleration vector.
     * @param c_target Target operational space value.
     * @param c_d_target Target operational space velocity.
     * @param c_dd_target Target operational space acceleration.
     * @param q_already_filled Flag indicating whether the independent part is already filled.
     */
    virtual void fill_dependent(
        VecX& q_full, 
        VecX& q_d_full,
        VecX& q_dd_full,
        const VecX& c_target,
        const VecX& c_d_target,
        const VecX& c_dd_target,
        bool q_already_filled = false);
};

}; // namespace robust_controller

#endif // OPERATIONAL_SPACE_IK_HPP
