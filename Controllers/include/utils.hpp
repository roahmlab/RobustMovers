#ifndef CONTROLLER_UTILS_HPP
#define CONTROLLER_UTILS_HPP

#include <Eigen/Dense>
#include <cmath>

namespace robust_controller {
namespace controller_utils {

/**
 * @brief Clamp a value to the range [-pi, pi].
 * 
 * @param inp Input value.
 * @return double Clamped value.
 */
inline double clamp(double inp) {
    double res = inp;
    while (res >= M_PI) res -= M_2_PI;
    while (res < -M_PI) res += M_2_PI;
    return res;
}

/**
 * @brief Clamp a vector to the range [-pi, pi].
 * 
 * @param inp Input vector.
 * @return Eigen::VectorXd Clamped vector.
 */
inline Eigen::VectorXd clamp(const Eigen::VectorXd& inp) {
    Eigen::VectorXd res = inp;
    for (int i = 0; i < res.size(); i++) {
        res(i) = clamp(res(i));
    }
    return res;
}

}; // namespace controller_utils
}; // namespace robust_controller

#endif // CONTROLLER_UTILS_HPP