#include "kinova_controller_ID_robust_pybindwrapper.hpp"
#include <stdexcept>
#include <cmath>
#include <iostream>


namespace robust_controller {
namespace Kinova {

namespace nb = nanobind;

kinova_controller_ID_robust_pybindwrapper::kinova_controller_ID_robust_pybindwrapper(
    const std::string urdf_filename,
    const std::string config_filename,
    const double Kp,
    const double Kd,
    const double epsilon,
    const double rho_init,
    const double k_rho,
    const double dt,
    const double alpha)
    : urdf_filename_copy(urdf_filename)
{

    modelPtr_ = std::make_shared<model>(urdf_filename, config_filename);

    // Kp, Kd, epsilon, rho_init, k_rho, dt, alpha 
    if (Kp <= 0) {
        throw std::invalid_argument("...: Kp must be positive!");
    }
    params.Kp = Kp * Eigen::MatrixXd::Identity(modelPtr_->NB, modelPtr_->NB);

    if(Kd <= 0){
        throw std::invalid_argument("...: Kd must be positive!");
    }
    params.Kd = Kd * Eigen::MatrixXd::Identity(modelPtr_->NB, modelPtr_->NB);

    if(epsilon <= 0){
        throw std::invalid_argument("...: epsilon must be positive!");
    }
    if(epsilon >=0.5){
        throw std::invalid_argument("...: epsilon must be less than 0.5!");
    }
    params.epsilon = epsilon;

    if (k_rho <= 0) {
        throw std::invalid_argument("...: k_rho must be positive!");
    }
    params.k_rho = k_rho;

    if(dt <= 0){
        throw std::invalid_argument("...: dt must be positive!");
    }
    params.dt = dt;

    params.rho_init = rho_init;

    if (alpha <= 0) {
        throw std::invalid_argument("...: alpha must be positive!");
    }
    if (alpha >=1){
        throw std::invalid_argument("...: alpha must be less than 1!");
    }
    params.alpha = alpha;


    controllerPtr_ = std::make_shared<controller_ID_robust>(modelPtr_, params);

    q      = VecX::Zero(modelPtr_->NB);
    q_d    = VecX::Zero(modelPtr_->NB);
    q_dd   = VecX::Zero(modelPtr_->NB);
    qd     = VecX::Zero(modelPtr_->NB);
    qd_d   = VecX::Zero(modelPtr_->NB);
    qd_dd  = VecX::Zero(modelPtr_->NB);

    tau    = VecX::Zero(modelPtr_->NB);
}

void kinova_controller_ID_robust_pybindwrapper::reset_model_parameters(
    const std::string config_filename)
{
    modelPtr_ = std::make_shared<model>(urdf_filename_copy, config_filename);
    controllerPtr_ = std::make_shared<controller_ID_robust>(modelPtr_, params);
}

nb::ndarray<nb::numpy, const double> kinova_controller_ID_robust_pybindwrapper::update(
    const nb_1d_double& q_input,  
    const nb_1d_double& q_d_input, 
    const nb_1d_double& q_dd_input,
    const nb_1d_double& qd_input, 
    const nb_1d_double& qd_d_input, 
    const nb_1d_double& qd_dd_input)
{
    if (q_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("update(): q_input size mismatch!");
    }
    if (q_d_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("update(): q_d_input size mismatch!");
    }
    if (q_dd_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("update(): q_dd_input size mismatch!");
    }
    if (qd_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("update(): qd_input size mismatch!");
    }
    if (qd_d_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("update(): qd_d_input size mismatch!");
    }
    if (qd_dd_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("update(): qd_dd_input size mismatch!");
    }

    for (int i = 0; i < q_input.size(); i++)        q(i)     = q_input(i);
    for (int i = 0; i < q_d_input.size(); i++)      q_d(i)   = q_d_input(i);
    for (int i = 0; i < q_dd_input.size(); i++)     q_dd(i)  = q_dd_input(i);
    for (int i = 0; i < qd_input.size(); i++)       qd(i)    = qd_input(i);
    for (int i = 0; i < qd_d_input.size(); i++)     qd_d(i)  = qd_d_input(i);
    for (int i = 0; i < qd_dd_input.size(); i++)    qd_dd(i) = qd_dd_input(i);

    tau = controllerPtr_->update(q, q_d, q_dd, qd, qd_d, qd_dd);

    const size_t shape_ptr[] = { static_cast<size_t>(modelPtr_->NB) };
    auto control = nb::ndarray<nb::numpy, const double>(
        tau.data(),     
        1,               
        shape_ptr,        
        nb::handle()     
    );

    return control;
}

} // namespace Kinova
} // namespace robust_controller
