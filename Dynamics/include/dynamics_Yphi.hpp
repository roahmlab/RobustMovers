#ifndef DYNAMICS_YPHI_HPP
#define DYNAMICS_YPHI_HPP

#include "dynamics.hpp"

namespace robust_controller {

/**
 * @brief Class representing multi-body dynamics.
 * 
 * This class represents the dynamics of a multi-body system, including
 * methods for computing the dynamics and updating relevant variables.
 */
class MultiBodyDynamicsYphi : public MultiBodyDynamics {
public:
    using VecX = Eigen::VectorXd; //!< Type alias for Eigen dynamic vector.
    using MatX = Eigen::MatrixXd; //!< Type alias for Eigen dynamic matrix.

    Mat6 Xj; //!< Spatial rotation matrix.
    std::vector<Vec6> S; //!< Unit screw vectors indicating joint types.
    std::vector<Mat6> Xup; //!< Spatial transform matrices.
    Vec6 vJ; //!< Joint velocity vector.
    Vec6 vJ_aux; //!< Auxiliary joint velocity vector.
    std::vector<Vec6> v; //!< Velocity vectors.
    std::vector<Vec6> v_aux; //!< Auxiliary velocity vectors.
    std::vector<Vec6> a; //!< Acceleration vectors.
    std::vector<Eigen::Matrix<double, 6, 10>> K; //!< Assembly matrices.
    MatX Yfull; //!< Full dynamics regressor matrix.
    // MatX Y; //!< Dynamics regressor matrix.
    VecX phi_inf; //!< Lower bound of joint positions.
    VecX phi_sup; //!< Upper bound of joint positions.

    /**
     * @brief Default constructor.
     */
    MultiBodyDynamicsYphi();

    /**
     * @brief Constructor with model pointer.
     * 
     * @param modelPtr_in Pointer to the robot model.
     */
    MultiBodyDynamicsYphi(const std::shared_ptr<model>& modelPtr_in);

    /**
     * @brief Constructor with model pointer and tolerance parameter.
     * 
     * @param modelPtr_in Pointer to the robot model.
     * @param phi_eps_in Model uncertainty for each inertial parameter.
     */
    MultiBodyDynamicsYphi(
        const std::shared_ptr<model>& modelPtr_in, 
        const double phi_eps_in);

    /**
     * @brief Destructor.
     */
    ~MultiBodyDynamicsYphi() = default;

    /**
     * @brief Return the actuated dimension of the system.
     * 
     * For system without constraints, the actual system dimension is the number of joints.
     */
    virtual int actual_system_dimension() const final override {
        return NB;
    }

    /**
     * @brief Update dynamics matrices.
     * 
     * This function updates the dynamics matrices Y and Yfull for passive dynamics.
     * 
     * @param q Joint position vector.
     * @param q_d Joint velocity vector.
     * @param q_aux_d Auxiliary joint velocity vector.
     * @param q_dd Auxiliary joint acceleration vector.
     * @param add_gravity Flag indicating whether to add gravity effects (default: true).
     */
    virtual void Yphi_passive(
        const VecX& q, 
        const VecX& q_d, 
        const VecX& q_aux_d,
        const VecX& q_aux_dd, 
        const bool add_gravity = true
    ) override;

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
    ) override;

    /**
     * @brief Compute RNEA with smarter interval evaluation and update dynamics matrices.
     * 
     * This function computes the Recursive Newton-Euler Algorithm (RNEA) using
     * interval arithmetic and updates the dynamics matrices Y, Yfull, joint torques
     * tau, and their respective lower and upper bounds.
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
    );
};

}; // namespace robust_controller

#endif // DYNAMICS_YPHI_HPP
