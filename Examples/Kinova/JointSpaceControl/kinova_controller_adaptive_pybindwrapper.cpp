#include "kinova_controller_adaptive_pybindwrapper.hpp"

namespace robust_controller {
namespace Kinova {

kinova_controller_adaptive_pybindwrapper::kinova_controller_adaptive_pybindwrapper(const std::string& urdf_filename,
                                    const std::string& config_filename,
                                    const nb_1d_double& Kd,
                                    const nb_1d_double& Kr,
                                    const nb_1d_double& gamma, 
                                    const double dt)
    : urdf_filename_copy(urdf_filename) {
    modelPtr_ = std::make_shared<model>(urdf_filename, config_filename);
    // kd
    if (Kd.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): Kd size mismatch!");
    } 
    params.Kd = Eigen::VectorXd::Zero(modelPtr_->NB);
    for (int i = 0; i < Kd.size(); i++) {
        if (Kd(i) <= 0) {
            throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): Kd must be positive!");
        }
        params.Kd(i) = Kd(i);
    }
    
    // kr
    if (Kr.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): Kr size mismatch!");
    }
    params.Kr = Eigen::VectorXd::Zero(modelPtr_->NB);
    for (int i = 0; i < Kr.size(); i++) {
        if (Kr(i) <= 0) {
            throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): Kr must be positive!");
        }
        params.Kr(i) = Kr(i);
    } // shrink the constraints? 

    // adative gain
       // Initialize Gamma
    if (gamma.size() != 10) {
        throw std::invalid_argument("gamma size mismatch! Expected size: 10.");
    }
    Eigen::VectorXd gamma_vec(10);
    for (int i = 0; i < gamma.size(); i++) {
        gamma_vec(i) = gamma(i);
    }

    params.Gamma = Eigen::MatrixXd::Zero(modelPtr_->NB * gamma_vec.size(), modelPtr_->NB * gamma_vec.size());
    for (int joint = 0; joint < modelPtr_->NB; ++joint) {
        int start_idx = joint * gamma_vec.size(); // Starting index for each joint block
        params.Gamma.block(start_idx, start_idx, gamma_vec.size(), gamma_vec.size()) = gamma_vec.asDiagonal();
    }
    // params.Gamma = Eigen::MatrixXd::Identity(modelPtr_->NB*10, modelPtr_->NB*10) * gamma;
    // time to update parameters, slow rate
    params.dt = dt;

    controllerPtr_ = std::make_shared<controller_adaptive>(modelPtr_, params);

    q = Eigen::VectorXd::Zero(modelPtr_->NB);
    q_d = Eigen::VectorXd::Zero(modelPtr_->NB);
    
    qd = Eigen::VectorXd::Zero(modelPtr_->NB);
    qd_d = Eigen::VectorXd::Zero(modelPtr_->NB);
    qd_dd = Eigen::VectorXd::Zero(modelPtr_->NB);

    tau = Eigen::VectorXd::Zero(modelPtr_->NB);
}

void kinova_controller_adaptive_pybindwrapper::reset_model_parameters(const std::string& config_filename) {
    modelPtr_ = std::make_shared<model>(urdf_filename_copy, config_filename);
    controllerPtr_ = std::make_shared<controller_adaptive>(modelPtr_,params);
}

nb::ndarray<nb::numpy, const double> kinova_controller_adaptive_pybindwrapper::update(
    const nb_1d_double& q_input,  
    const nb_1d_double& q_d_input, 
    const nb_1d_double& qd_input, 
    const nb_1d_double& qd_d_input, 
    const nb_1d_double& qd_dd_input) {
    if (q_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_adaptive_pybindwrapper: update(): q_input size mismatch!");
    }
    if (q_d_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_adaptive_pybindwrapper: update(): q_d_input size mismatch!");
    }
    if (qd_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_adaptive_pybindwrapper update(): qd_input size mismatch!");
    }
    if (qd_d_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_adaptive_pybindwrapper: update(): qd_d_input size mismatch!");
    }
    if (qd_dd_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_adaptive_pybindwrapper: update(): qd_dd_input size mismatch!");
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

nb::ndarray<nb::numpy, const double> kinova_controller_adaptive_pybindwrapper::get_parameters() {
    Eigen::VectorXd phi = controllerPtr_->mbdPtr_ -> modelPtr_->phi;
    const size_t shape_ptr[] = {phi.size()};
    auto full_phi = nb::ndarray<nb::numpy, const double>(
        phi.data(), 1, shape_ptr, nb::handle());
    return full_phi;
}

} // namespace Kinova
} // namespace robust_controller
