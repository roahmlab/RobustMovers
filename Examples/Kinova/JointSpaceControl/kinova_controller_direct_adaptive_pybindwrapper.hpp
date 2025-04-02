#ifndef CONTROLLER_DIRECT_ADAPTIVE_PYBINDWRAPPER_HPP
#define CONTROLLER_DIRECT_ADAPTIVE_PYBINDWRAPPER_HPP

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include "controller_direct_adaptive.hpp"
#include "kinova_constants.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"

namespace robust_controller {
namespace Kinova {
namespace nb = nanobind;

class kinova_controller_direct_adaptive_pybindwrapper {
public:

    using Model = pinocchio::Model; //!< Type alias for Pinocchio model.
    using Vec3 = Eigen::Vector3d; //!< Type alias for Eigen dynamic 3D vector.
    using VecX = Eigen::VectorXd; //!< Type alias for Eigen dynamic vector.
    using MatX = Eigen::MatrixXd; //!< Type alias for Eigen dynamic matrix.

    using nb_1d_double = nb::ndarray<double, nb::ndim<1>, nb::c_contig, nb::device::cpu>; //!< Type alias for 1D double array.
    using nb_2d_double = nb::ndarray<double, nb::ndim<2>, nb::c_contig, nb::device::cpu>; //!< Type alias for 2D double array.

    // Class members
    std::string urdf_filename_copy; //!< URDF file path.
    std::shared_ptr<model> modelPtr_; //!< Pointer to the robot model.

    controller_direct_adaptive::parameters params; //!< Controller parameters.

    std::shared_ptr<controller_direct_adaptive> controllerPtr_; //!< Pointer to the controller.

    VecX q; //!< Joint position.
    VecX q_d; //!< Joint velocity.
    VecX qd; //!< Desired joint position.
    VecX qd_d; //!< Desired joint velocity.
    VecX qd_dd; //!< Desired joint acceleration.
    VecX tau; //!< Joint torque.

    // Constructor with fkd as element_wise kd * I
    kinova_controller_direct_adaptive_pybindwrapper(
        const std::string& urdf_filename,
        const std::string& config_filename,
        const nb_1d_double& Kd,
        const nb_1d_double& Kr,
        const double beta,
        const double delta,
        const double eta,
        const double alpha,
        const double dt);

    // Reset model parameters using a new configuration file
    void reset_model_parameters(const std::string& config_filename);

    // Update function to compute control torques
    nb::ndarray<nb::numpy, const double> update(
        const nb_1d_double& q_input,
        const nb_1d_double& q_d_input,
        const nb_1d_double& qd_input,
        const nb_1d_double& qd_d_input,
        const nb_1d_double& qd_dd_input);

    nb::ndarray<nb::numpy, const double> get_parameters();
};

} // namespace Kinova
} // namespace robust_controller

#endif // CONTROLLER_ADAPTIVE_PYBINDWRAPPER_HPP
