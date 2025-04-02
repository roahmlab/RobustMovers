#ifndef CONTROLLER_ROBUST_HPP
#define CONTROLLER_ROBUST_HPP

#include "controller.hpp"
#include "dynamics_Yphi.hpp"

namespace robust_controller {

/**
 * @brief Class representing a robust controller.
 * 
 * This class implements a robust controller for a multi-body dynamics system.
 */
class controller_armour : public controller {
public:
    using VecX = Eigen::VectorXd; //!< Type alias for Eigen dynamic vector.

    /**
     * @brief Struct representing controller parameters.
     */
    struct parameters {
        VecX Kr; //!< Feedback gain values.
        double V_max = 1e-4; //!< Barrier value for the Lyapunov function.
        double alpha = 10; //!< Alpha parameter.
        double r_norm_threshold = 1e-6; //!< Threshold for the norm of the tracking error.
    };

    /**
     * @brief Struct representing controller parameters.
     */
    parameters params; //!< Controller parameters.

    VecX v; //!< Robust control input.

    VecX q_d_zero; //!< Placeholders for the zero vector.

    /**
     * @brief Default constructor.
     */
    controller_armour() = default;

    /**
     * @brief Constructor with model pointer.
     * 
     * @param modelPtr_in Pointer to the robot model.
     */
    controller_armour(const std::shared_ptr<model>& modelPtr_in);

    /**
     * @brief Constructor with model pointer and parameters.
     * 
     * @param modelPtr_in Pointer to the robot model.
     * @param params_in Controller parameters.
     */
    controller_armour(
        const std::shared_ptr<model>& modelPtr_in,
        const parameters& params_in
    );

    /**
     * @brief Constructor with model pointer, parameters, and tolerance parameter.
     * 
     * @param modelPtr_in Pointer to the robot model.
     * @param params_in Controller parameters.
     * @param phi_eps_in Model uncertainty for each inertial parameter.
     */
    controller_armour(
        const std::shared_ptr<model>& modelPtr_in,
        const parameters& params_in, 
        const double phi_eps_in
    );

    /**
     * @brief Destructor.
     */
    ~controller_armour() = default;

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

#endif // CONTROLLER_ROBUST_HPP
