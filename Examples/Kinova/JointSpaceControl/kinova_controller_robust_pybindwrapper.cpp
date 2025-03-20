#include "kinova_controller_robust_pybindwrapper.hpp"

namespace robust_controller {
namespace Kinova {

kinova_controller_robust_pybindwrapper::kinova_controller_robust_pybindwrapper(
    const std::string urdf_filename,
    const std::string config_filename,
    const nb_1d_double& Kr,
    const double V_max,
    const double alpha,
    const double r_norm_threshold) :
    urdf_filename_copy(urdf_filename) {
    modelPtr_ = std::make_shared<model>(
        urdf_filename, config_filename);
    
    if (Kr.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): Kr size mismatch!");
    }
    params.Kr = VecX::Zero(modelPtr_->NB);
    for (int i = 0; i < Kr.size(); i++) {
        if (Kr(i) <= 0) {
            throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): Kr must be positive!");
        }

        params.Kr(i) = Kr(i);
    }

    if (V_max <= 0) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): V_max must be positive!");
    }
    params.V_max = V_max;

    if (alpha <= 0) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): alpha must be positive!");
    }
    params.alpha = alpha;

    if (r_norm_threshold < 0) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): r_norm_threshold must be non-negative!");
    }
    params.r_norm_threshold = r_norm_threshold;

    controllerPtr_ = std::make_shared<controller_robust>(
        modelPtr_, params);

    q = VecX::Zero(modelPtr_->NB);
    q_d = VecX::Zero(modelPtr_->NB);
    
    qd = VecX::Zero(modelPtr_->NB);
    qd_d = VecX::Zero(modelPtr_->NB);
    qd_dd = VecX::Zero(modelPtr_->NB);

    tau = VecX::Zero(modelPtr_->NB);
}

void kinova_controller_robust_pybindwrapper::reset_model_parameters(const std::string config_filename) {
    modelPtr_ = std::make_shared<model>(
        urdf_filename_copy, config_filename);

    controllerPtr_ = std::make_shared<controller_robust>(
        modelPtr_, params);
}

void kinova_controller_robust_pybindwrapper::reset_controller_parameters(
    const nb_1d_double Kr,
    const double V_max,
    const double alpha,
    const double r_norm_threshold) {
    if (Kr.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): Kr size mismatch!");
    }
    params.Kr = VecX::Zero(modelPtr_->NB);
    for (int i = 0; i < Kr.size(); i++) {
        if (Kr(i) <= 0) {
            throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): Kr must be positive!");
        }

        params.Kr(i) = Kr(i);
    }

    if (V_max <= 0) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): V_max must be positive!");
    }
    params.V_max = V_max;

    if (alpha <= 0) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): alpha must be positive!");
    }
    params.alpha = alpha;

    if (r_norm_threshold < 0) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: kinova_controller_robust_pybindwrapper(): r_norm_threshold must be non-negative!");
    }
    params.r_norm_threshold = r_norm_threshold;

    controllerPtr_ = std::make_shared<controller_robust>(
        modelPtr_, params);
}

nb::ndarray<nb::numpy, const double> kinova_controller_robust_pybindwrapper::update(
    const nb_1d_double& q_input,  
    const nb_1d_double& q_d_input, 
    const nb_1d_double& qd_input, 
    const nb_1d_double& qd_d_input, 
    const nb_1d_double& qd_dd_input) {
    if (q_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: update(): q_input size mismatch!");
    }
    if (q_d_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: update(): q_d_input size mismatch!");
    }
    if (qd_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: update(): qd_input size mismatch!");
    }
    if (qd_d_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: update(): qd_d_input size mismatch!");
    }
    if (qd_dd_input.size() != modelPtr_->NB) {
        throw std::invalid_argument("kinova_controller_robust_pybindwrapper.cpp: update(): qd_dd_input size mismatch!");
    }

    q = VecX::Zero(modelPtr_->NB);
    for (int i = 0; i < q_input.size(); i++) {
        q(i) = q_input(i);
    }

    q_d = VecX::Zero(modelPtr_->NB);
    for (int i = 0; i < q_d_input.size(); i++) {
        q_d(i) = q_d_input(i);
    }

    qd = VecX::Zero(modelPtr_->NB);
    for (int i = 0; i < qd_input.size(); i++) {
        qd(i) = qd_input(i);
    }

    qd_d = VecX::Zero(modelPtr_->NB);
    for (int i = 0; i < qd_d_input.size(); i++) {
        qd_d(i) = qd_d_input(i);
    }

    qd_dd = VecX::Zero(modelPtr_->NB);
    for (int i = 0; i < qd_dd_input.size(); i++) {
        qd_dd(i) = qd_dd_input(i);
    }

    tau = controllerPtr_->update(q, q_d, qd, qd_d, qd_dd);

    for (int i = 0; i < modelPtr_->NB; i++) {
        if (tau(i) > TORQUE_LIMITS_UPPER[i] ||
            tau(i) < TORQUE_LIMITS_LOWER[i]) {
            // std::cerr << "motor: " << i << " tau: " << tau(i) << std::endl;
            // std::cerr << "limits: [" << TORQUE_LIMITS_LOWER[i] << ", " << TORQUE_LIMITS_UPPER[i] << "]" << std::endl; 
            // throw std::runtime_error("kinova_controller_robust_pybindwrapper.cpp: update(): torque limits exceeded!");

            // clamp tau
            if (tau(i) > TORQUE_LIMITS_UPPER[i]) {
                tau(i) = TORQUE_LIMITS_UPPER[i];
            }
            else if (tau(i) < TORQUE_LIMITS_LOWER[i]) {
                tau(i) = TORQUE_LIMITS_LOWER[i];
            }
        }
    }

    const size_t shape_ptr[] = {modelPtr_->NB};
    auto control = nb::ndarray<nb::numpy, const double>(
        tau.data(), 1, shape_ptr, nb::handle());

    return control;
}

}; // namespace Kinova
}; // namespace robust_controller
