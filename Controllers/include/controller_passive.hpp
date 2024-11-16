#ifndef CONTROLLER_PASSIVE_HPP
#define CONTROLLER_PASSIVE_HPP

#include "controller.hpp"
#include "dynamics_pinocchio.hpp"

namespace robust_controller {

/**
 * @brief Class representing a naive passivity-based controller interface.
 * 
 * This class implements an interface of the naive passivity-based controller for a multi-body dynamics system.
 */
class controller_passive : public controller {
public:
    using VecX = Eigen::VectorXd; //!< Type alias for Eigen dynamic vector.

    VecX q_diff; //!< Joint position difference.
    VecX q_d_diff; //!< Joint velocity difference.

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
    controller_passive() = default;

    /**
     * @brief Constructor with model pointer.
     * 
     * @param modelPtr_in Pointer to the robot model.
     */
    controller_passive(const std::shared_ptr<model>& modelPtr_in);

    /**
     * @brief Constructor with model pointer.
     * 
     * @param modelPtr_in Pointer to the robot model.
     * @param params_in Controller parameters.
     */
    controller_passive(
        const std::shared_ptr<model>& modelPtr_in,
        const parameters& params_in);

    /**
     * @brief Destructor.
     */
    ~controller_passive() = default;

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
    ) final override;
};

}; // namespace robust_controller

#endif // CONTROLLER_PASSIVE_HPP