#ifndef DYNAMICS_UTILS_HPP
#define DYNAMICS_UTILS_HPP

#include <Eigen/Dense>

namespace robust_controller {
namespace dynamics_utils {

inline bool ifTwoVectorEqual(
    const Eigen::VectorXd& a, 
    const Eigen::VectorXd& b, 
    double tol = 0) {
    if (a.size() != b.size()) {
        return false;
    }
    for (int i = 0; i < a.size(); i++) {
        if (fabs(a(i) - b(i)) > tol) {
            return false;
        }
    }
    return true;
}

inline double safeasin(
    const double x,
    const bool throw_exception = false) {
    if (x > 1.0) {
        if (throw_exception) {
            throw std::runtime_error("Input value is greater than 1.0");
        }
        return M_PI_2;
    } 
    else if (x < -1.0) {
        if (throw_exception) {
            throw std::runtime_error("Input value is less than -1.0");
        }
        return -M_PI_2;
    } 
    return std::asin(x);
}

inline double safeacos(
    const double x,
    const bool throw_exception = false) {
    if (x > 1.0) {
        if (throw_exception) {
            throw std::runtime_error("Input value is greater than 1.0");
        }
        return 0.0;
    } 
    else if (x < -1.0) {
        if (throw_exception) {
            throw std::runtime_error("Input value is less than -1.0");
        }
        return M_PI;
    } 
    return std::acos(x);
}

inline Eigen::Vector3d rot2rpy(const Eigen::Matrix3d& R) {
    Eigen::Vector3d rpy;
    rpy(0) = std::atan2(-R(1, 2), R(2, 2));
    rpy(1) = std::asin(R(0, 2));
    rpy(2) = std::atan2(-R(0, 1), R(0, 0));
    return rpy;
}

}; // namespace dynamics_utils
}; // namespace robust_controller

#endif // DYNAMICS_UTILS_HPP