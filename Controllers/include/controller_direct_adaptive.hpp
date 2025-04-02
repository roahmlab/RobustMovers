#ifndef CONTROLLER_DIRECT_ADAPTIVE_HPP
#define CONTROLLER_DIRECT_ADAPTIVE_HPP

#include "controller.hpp"
#include "dynamics_Yphi.hpp"

namespace robust_controller {

/*
Yin, Xiuxing, and Li Pan. 
"Enhancing trajectory tracking accuracy for industrial robot with robust adaptive control." 
Robotics and Computer-Integrated Manufacturing 51 (2018): 97-102.
*/

class controller_direct_adaptive : public controller {
public:
    using VecX = Eigen::VectorXd;
    using MatX = Eigen::MatrixXd;

    // Control parameters structure
    struct parameters {
        VecX Kd;         // Derivative gain
        VecX Kr;         // Reference gain
        double beta;                // Adaptive gain    
        double delta;               // Adaptive gain
        double eta;                 // Adaptive gain 
        double alpha;               // Forgetting factor
        double dt;                  // Time step
    };

    parameters params; //Controller parameters.

    MatX Gamma; // Adaptation gain (PSD matrix)

    // Constructor
    controller_direct_adaptive() = default;

    controller_direct_adaptive(const std::shared_ptr<model>& modelPtr_in);
    controller_direct_adaptive(const std::shared_ptr<model>& modelPtr_in, const parameters& params_in);

    // Destructor
    ~controller_direct_adaptive() = default;

    // Update function to compute control torque
    virtual VecX update(
        const VecX& q,  
        const VecX& q_d, 
        const VecX& qd, 
        const VecX& qd_d, 
        const VecX& qd_dd
    ) final override;
};

} // namespace robust_controller

#endif // CONTROLLER_DIRECT_ADAPTIVE_HPP
