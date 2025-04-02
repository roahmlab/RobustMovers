#include "kinova_controller_direct_adaptive_pybindwrapper.hpp"

namespace robust_controller {
namespace Kinova {

kinova_controller_direct_adaptive_pybindwrapper::kinova_controller_direct_adaptive_pybindwrapper(
    const std::string& urdf_filename,
    const std::string& config_filename,
    const nb_1d_double& Kd,
    const nb_1d_double& Kr,
    const double beta,
    const double delta,
    const double eta,
    const double alpha,
    const double dt)
    : urdf_filename_copy(urdf_filename) {
    modelPtr_ = std::make_shared<model>(urdf_filename, config_filename);
    // kd
    if (Kd.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_direct_adaptive_pybindwrapper.cpp: kinova_controller_direct_adaptive_pybindwrapper(): Kd size mismatch!");
    } 
    params.Kd = Eigen::VectorXd::Zero(modelPtr_->NB);
    for (int i = 0; i < Kd.size(); i++) {
        if (Kd(i) <= 0) {
            throw std::invalid_argument("kinova_controller_direct_adaptive_pybindwrapper.cpp: kinova_controller_direct_adaptive_pybindwrapper(): Kd must be positive!");
        }
        params.Kd(i) = Kd(i);
    }
    
    // kr
    if (Kr.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_direct_adaptive_pybindwrapper.cpp: kinova_controller_direct_adaptive_pybindwrapper(): Kr size mismatch!");
    }
    params.Kr = Eigen::VectorXd::Zero(modelPtr_->NB);
    for (int i = 0; i < Kr.size(); i++) {
        if (Kr(i) <= 0) {
            throw std::invalid_argument("kinova_controller_direct_adaptive_pybindwrapper.cpp: kinova_controller_direct_adaptive_pybindwrapper(): Kr must be positive!");
        }
        params.Kr(i) = Kr(i);
    } 

    // adative gain
    params.beta = beta;
    params.delta = delta;
    params.eta = eta;
    params.alpha = alpha;
    params.dt = dt;

    controllerPtr_ = std::make_shared<controller_direct_adaptive>(modelPtr_, params);

    q = Eigen::VectorXd::Zero(modelPtr_->NB);
    q_d = Eigen::VectorXd::Zero(modelPtr_->NB);
    
    qd = Eigen::VectorXd::Zero(modelPtr_->NB);
    qd_d = Eigen::VectorXd::Zero(modelPtr_->NB);
    qd_dd = Eigen::VectorXd::Zero(modelPtr_->NB);

    tau = Eigen::VectorXd::Zero(modelPtr_->NB);
}

void kinova_controller_direct_adaptive_pybindwrapper::reset_model_parameters(const std::string& config_filename) {
    modelPtr_ = std::make_shared<model>(urdf_filename_copy, config_filename);
    controllerPtr_ = std::make_shared<controller_direct_adaptive>(modelPtr_,params);
}

nb::ndarray<nb::numpy, const double> kinova_controller_direct_adaptive_pybindwrapper::update(
    const nb_1d_double& q_input,  
    const nb_1d_double& q_d_input, 
    const nb_1d_double& qd_input, 
    const nb_1d_double& qd_d_input, 
    const nb_1d_double& qd_dd_input) {
    if (q_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_direct_adaptive_pybindwrapper: update(): q_input size mismatch!");
    }
    if (q_d_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_direct_adaptive_pybindwrapper: update(): q_d_input size mismatch!");
    }
    if (qd_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_direct_adaptive_pybindwrapper update(): qd_input size mismatch!");
    }
    if (qd_d_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_direct_adaptive_pybindwrapper: update(): qd_d_input size mismatch!");
    }
    if (qd_dd_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_direct_adaptive_pybindwrapper: update(): qd_dd_input size mismatch!");
    }

    q = Eigen::VectorXd::Zero(modelPtr_->NB);
    for (int i = 0; i < q_input.size(); i++) {
        q(i) = q_input(i);
    }

    q_d = Eigen::VectorXd::Zero(modelPtr_->NB);
    for (int i = 0; i < q_d_input.size(); i++) {
        q_d(i) = q_d_input(i);
    }

    qd = Eigen::VectorXd::Zero(modelPtr_->NB);
    for (int i = 0; i < qd_input.size(); i++) {
        qd(i) = qd_input(i);
    }

    qd_d = Eigen::VectorXd::Zero(modelPtr_->NB);
    for (int i = 0; i < qd_d_input.size(); i++) {
        qd_d(i) = qd_d_input(i);
    }

    qd_dd = Eigen::VectorXd::Zero(modelPtr_->NB);
    for (int i = 0; i < qd_dd_input.size(); i++) {
        qd_dd(i) = qd_dd_input(i);
    }

    tau = controllerPtr_->update(q, q_d, qd, qd_d, qd_dd);
    const size_t shape_ptr[] = {modelPtr_->NB};
    auto control = nb::ndarray<nb::numpy, const double>(
        tau.data(), 1, shape_ptr, nb::handle());
    return control;
}

nb::ndarray<nb::numpy, const double> kinova_controller_direct_adaptive_pybindwrapper::get_parameters() {
    Eigen::VectorXd phi = controllerPtr_->mbdPtr_ -> modelPtr_->phi;
    const size_t shape_ptr[] = {phi.size()};
    auto full_phi = nb::ndarray<nb::numpy, const double>(
        phi.data(), 1, shape_ptr, nb::handle());
    return full_phi;
}

} // namespace Kinova
} // namespace robust_controller
