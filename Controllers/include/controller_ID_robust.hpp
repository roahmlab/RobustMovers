#ifndef CONTROLLER_ID_ROBUST_HPP
#define CONTROLLER_ID_ROBUST_HPP

#include "controller.hpp"
#include "dynamics_pinocchio.hpp"
#include "dynamics_Yphi.hpp"

namespace robust_controller {


class controller_ID_robust : public controller {
public:
    using VecX = Eigen::VectorXd; 

    struct parameters {
        Eigen::MatrixXd Kp;       
        Eigen::MatrixXd Kd;       
        double epsilon  = 1e-2;   
        double rho_init = 0.0;    
        double k_rho    = 10.0;   
        double dt       = 0.001; 
        double alpha    = 0.7; 
    };
    parameters params;


    controller_ID_robust(const std::shared_ptr<model>& modelPtr_in);

    controller_ID_robust(
        const std::shared_ptr<model>& modelPtr_in,
        const parameters& params_in
    );

    ~controller_ID_robust() = default;

    VecX update(
        const VecX& q,  
        const VecX& q_d, 
        const VecX& q_dd,
        const VecX& dq, 
        const VecX& dq_d, 
        const VecX& ddq_d
    ); 

    virtual VecX update(
        const VecX& q,  
        const VecX& q_d, 
        const VecX& qd, 
        const VecX& qd_d, 
        const VecX& qd_dd
    ) final override;

    VecX rnea(const VecX& q, 
              const VecX& q_d, 
              const VecX& a);

    void get_rho(const std::shared_ptr<model>& modelPtr_in);

private:
    double rho_ = 0.0;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd D;
    Eigen::MatrixXd H_tilde;

};

} // namespace robust_controller

#endif // CONTROLLER_ID_ROBUST_HPP
