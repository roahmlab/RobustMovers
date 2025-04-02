#ifndef CONTROLLER_ADAPTIVE_HPP
#define CONTROLLER_ADAPTIVE_HPP

#include "controller.hpp"
#include "dynamics_Yphi.hpp"

namespace robust_controller {

class controller_adaptive : public controller {
public:
    using VecX = Eigen::VectorXd;

    // Control parameters structure
    struct parameters {
        Eigen::VectorXd Kd;         // Derivative gain
        Eigen::MatrixXd Gamma;      // Adaptation gain (PSD matrix)
        Eigen::VectorXd Kr;         // Reference gain
        double dt;                  // Time step
    };

    parameters params; //Controller parameters.

    // Constructor
    controller_adaptive() = default;

    controller_adaptive(const std::shared_ptr<model>& modelPtr_in);
    controller_adaptive(const std::shared_ptr<model>& modelPtr_in, const parameters& params_in);

    // Destructor
    ~controller_adaptive() = default;


    // Update function to compute control torque
    virtual VecX update(
        const VecX& q,  
        const VecX& q_d, 
        const VecX& qd, 
        const VecX& qd_d, 
        const VecX& qd_dd
    ) final override;

    bool isPSD(const Eigen::MatrixXd& M);
};

} // namespace robust_controller

#endif // CONTROLLER_ADAPTIVE_HPP
