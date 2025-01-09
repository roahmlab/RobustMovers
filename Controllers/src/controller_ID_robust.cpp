#include "controller_ID_robust.hpp"


namespace robust_controller {

controller_ID_robust::controller_ID_robust(const std::shared_ptr<model>& modelPtr_in)
{
    NB = modelPtr_in->NB; 
    mbdPtr_ = std::make_shared<MultiBodyDynamicsPinocchio>(modelPtr_in);

    // define default control parameters, not checked for validity
    double kp = 100.0;
    params.Kp       = kp * Eigen::MatrixXd::Identity(NB, NB);
    params.Kd       =  2 * std::sqrt(kp) * Eigen::MatrixXd::Identity(NB, NB);
    params.epsilon  = 1e-2;
    params.rho_init = 0.0;
    params.k_rho    = 1.0;
    params.dt       = 0.001;

    Q = 10 * Eigen::MatrixXd::Identity(2 * NB, 2 * NB);
    D = Eigen::MatrixXd::Zero(2 * NB, NB);
    D.block(NB, 0, NB, NB) = Eigen::MatrixXd::Identity(NB, NB);
    rho_ = 10;
}


controller_ID_robust::controller_ID_robust(
    const std::shared_ptr<model>& modelPtr_in,
    const parameters& params_in)
    : params(params_in)
{
    NB = modelPtr_in->NB;
    mbdPtr_ = std::make_shared<MultiBodyDynamicsPinocchio>(modelPtr_in);

    if (params.Kp.rows() != NB || params.Kp.cols() != NB ||
        params.Kd.rows() != NB || params.Kd.cols() != NB) {
        throw std::invalid_argument("controller_ID_robust.cpp: constructor(): Kp/Kd size mismatch!");
    }

    // rho_ = params.rho_init;
    Q = 10 * Eigen::MatrixXd::Identity(2 * NB, 2 * NB);  // Q ?
    D = Eigen::MatrixXd::Zero(2 * NB, NB);
    D.block(NB, 0, NB, NB) = Eigen::MatrixXd::Identity(NB, NB);
    H_tilde = Eigen::MatrixXd::Zero(2 * NB, 2* NB);
    H_tilde.block(0, NB, NB, NB) = Eigen::MatrixXd::Identity(NB, NB);
    H_tilde.block(NB, 0, NB, NB) = -params.Kp;
    H_tilde.block(NB, NB, NB, NB) = -params.Kd;

    get_rho(modelPtr_in);
}

Eigen::VectorXd controller_ID_robust::update(
    const VecX& q,  
    const VecX& q_d, 
    const VecX& q_dd,
    const VecX& dq, 
    const VecX& dq_d, 
    const VecX& dq_dd)
{
    // Step 1, calculate tracking errors
    VecX e  = controller_utils::clamp(dq - q);
    VecX ed = dq_d - q_d;
    VecX edd = dq_dd - q_dd;
    VecX xi(2 * NB);
    xi << e , ed;

    // step 2, calculate eta
    Eigen::MatrixXd eta = edd + params.Kd * ed + params.Kp * e;

    // Step 3, calculate robust term w
    VecX z = D.transpose() * Q * xi;
    double z_norm = z.norm();
    VecX w = VecX::Zero(NB); 
    
    Eigen::MatrixXd P = -(H_tilde.transpose() * Q + Q * H_tilde); // MOVE ABOVE
    double term1 = -(xi.transpose() * P * xi).value();

    Eigen::VectorXd rhs = eta - rho_ * (z / z_norm); 
    double term2 = 2.0 * (z.transpose() * rhs).value();

    double dV = term1 + term2;
    
    if (z_norm >= params.epsilon) {
        w = (rho_ / z_norm) * z;
        // Step 4, update rho
        if (dV > 0) {
            rho_ += params.k_rho * params.dt;
            std::cout << "upate rho: " << rho_ << std::endl;
        }
    } else {
        w = (rho_ / params.epsilon) * z;
    }

    // Step 5, calculate control input
    VecX a = dq_dd + params.Kp * e + params.Kd * ed + w;  
    VecX tau = rnea(q, q_d, a);
   
    return tau; 
}

Eigen::VectorXd controller_ID_robust::rnea(const VecX& q, 
    const VecX& q_d, 
    const VecX& a)
    {
    // nominal dynamics
    mbdPtr_->rnea(q, q_d, q_d, a);
    VecX tau =  mbdPtr_->tau;
    // motor dynamics
    for (int i = 0; i < NB; i++) {
        tau(i) +=  mbdPtr_->modelPtr_->damping[i] * q_d(i) +
                   mbdPtr_->modelPtr_->transmissionInertia[i] * a(i) +  mbdPtr_->modelPtr_->offset[i];
                  
        if (fabs(q_d(i)) > 1e-8) {
            if (q_d(i) > 0) {
                tau(i) +=  mbdPtr_->modelPtr_->friction[i];
            }
            else {
                tau(i) -=  mbdPtr_->modelPtr_->friction[i];
            }
        }
    }
    // if total tau bigger than the limits, throw an exception
    // if( tau.maxCoeff() > 100|| tau.minCoeff() < -100){
    //     throw std::runtime_error("controller_adaptive.cpp: update(): torque limits exceeded!");
    // }
    return tau;
}

Eigen::VectorXd controller_ID_robust::update(
    const VecX& q,  
    const VecX& q_d, 
    const VecX& q_dd, 
    const VecX& dq, 
    const VecX& dq_d
) {
    VecX ddq_d = VecX::Zero(q.size());  
    std::cout << "controller_ID_robust::update() not used in this version " << std::endl;
    return this->update(q, q_d, q_dd, dq, dq_d, ddq_d);
}

void controller_ID_robust::get_rho(const std::shared_ptr<model>& modelPtr_in) {
    double M_max = 10; // inertia limit
    double Q_max = 8;  // acceleration limit

    Eigen ::VectorXd xi_max_vec = Eigen::VectorXd::Ones(NB) * 0.01;  // tracking error limit (q, dq)
    double xi_max_norm = xi_max_vec.norm();

    Eigen::MatrixXd K(NB, 2 * NB);
    K << params.Kp, params.Kd;
    double K_norm = K.norm();

    MultiBodyDynamicsYphi mbd_temp = MultiBodyDynamicsYphi(modelPtr_in);
    Eigen::VectorXd q_rand = 2 * Eigen::VectorXd::Random(NB)*M_PI - Eigen::VectorXd::Ones(NB)*M_PI;
    Eigen::VectorXd q_d_rand = 4 * Eigen::VectorXd::Random(NB)*M_PI;
    Eigen::VectorXd q_dd = Eigen::VectorXd::Zero(NB);
    mbd_temp.rnea_interval(q_rand , q_d_rand , q_d_rand , q_dd, true); // gravity included ?
    double phi = (mbd_temp.tau_sup - mbd_temp.tau).norm();

    rho_ = 1/(1-params.alpha) * (params.alpha * Q_max +  params.alpha * K_norm* xi_max_norm + M_max * phi);
    std::cout << "rho: " << rho_ << std::endl;
    std::cout << "phi: " << phi << std::endl;
    std::cout << "xi_max_norm: " << xi_max_norm << std::endl;
    std::cout << "K_norm: " << K_norm << std::endl;

}
} // namespace robust_controller


