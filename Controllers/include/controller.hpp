#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "utils.hpp"
#include "dynamics.hpp"

namespace robust_controller {

/**
 * @brief Class representing a pure virtual controller interface.
 * 
 * This class implements an interface of the controller for a multi-body dynamics system.
 */
class controller {
public:
    using VecX = Eigen::VectorXd; //!< Type alias for Eigen dynamic vector.

    std::shared_ptr<MultiBodyDynamics> mbdPtr_; //!< Pointer to multi-body dynamics.

    int NB = 0; //!< Number of actuated joints.

    /**
     * @brief Default constructor.
     */
    controller() = default;
    
    /**
     * @brief Destructor.
     */
    ~controller() = default;

    /**
     * @brief Update the controller.
     * 
     * This function updates the controller state based on current joint states.
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

#endif // CONTROLLER_HPP