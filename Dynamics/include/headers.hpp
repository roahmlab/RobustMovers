
#ifndef HEADERS_HPP
#define HEADERS_HPP

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstdio>
#include <memory>
#include <chrono>
#include <array>
#include <stdexcept>
#include <Eigen/Dense>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/data.hpp"

namespace robust_controller {

typedef Eigen::Matrix<double, 6, 6> Mat6;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 3, 3> Mat3;
typedef Eigen::Matrix<double, 3, 1> Vec3;

}; // namespace robust_controller

#endif // HEADERS_HPP
