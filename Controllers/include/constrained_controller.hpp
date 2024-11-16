#ifndef CONSTRAINED_CONTROLLER_HPP
#define CONSTRAINED_CONTROLLER_HPP

#include "controller.hpp"
#include "constrained_dynamics.hpp"
#include "operational_space_inverse_kinematics.hpp"

namespace robust_controller {

/**
 * @brief Class representing a pure virtual constrained controller interface.
 * 
 * This class implements an interface of the constrained controller for a multi-body dynamics system.
 */
class constrained_controller {
public:
    using VecX = Eigen::VectorXd; //!< Type alias for Eigen dynamic vector.

    std::shared_ptr<ConstrainedMultiBodyDynamics> cmbdPtr_; //!< Pointer to constrained multi-body dynamics.
    std::shared_ptr<OperationalSpaceInverseKinematics> opIKPtr_; //!< Pointer to operational space inverse kinematics.

    int NB = 0; //!< Number of joints.
    int Nact = 0; //!< Number of actuated joints.

    /**
     * @brief Default constructor.
     */
    constrained_controller() = default;
    
    /**
     * @brief Destructor.
     */
    ~constrained_controller() = default;

    /**
     * @brief Update the constrained_controller.
     * 
     * This function updates the constrained_controller state based on current joint states.
     * 
     * @param q Current joint positions.
     * @param q_d Current joint positions.
     * @param qd Target joint positions.
     * @param qd_d Target joint velocities.
     * @param qd_dd Target joint accelerations.
     * @return VecX Control input.
     */
    virtual VecX update(
        const VecX& q,  
        const VecX& q_d, 
        const VecX& qd, 
        const VecX& qd_d, 
        const VecX& qd_dd
    ) = 0;
};

}; // namespace robust_controller

#endif // CONSTRAINED_CONTROLLER_HPP