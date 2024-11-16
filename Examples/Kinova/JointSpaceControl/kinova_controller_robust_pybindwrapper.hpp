#ifndef KINOVA_CONTROLLER_ROBUST_PYBINDWRAPPER_H
#define KINOVA_CONTROLLER_ROBUST_PYBINDWRAPPER_H

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include "controller_robust.hpp"
#include "kinova_constants.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

namespace robust_controller {
namespace Kinova {

namespace nb = nanobind;

/**
 * @brief Class representing a Python wrapper for interfacing to Kinova controller.
 * 
 * This class implements a Python wrapper for interfacing to the Kinova controller.
 * 
 * @note This class is intended to be used with Python bindings.
 */
class kinova_controller_robust_pybindwrapper {
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

    controller_robust::parameters params; //!< Controller parameters.

    std::shared_ptr<controller_robust> controllerPtr_; //!< Pointer to the controller.

    VecX q; //!< Joint position.
    VecX q_d; //!< Joint velocity.
    VecX qd; //!< Desired joint position.
    VecX qd_d; //!< Desired joint velocity.
    VecX qd_dd; //!< Desired joint acceleration.
    VecX tau; //!< Joint torque.

    // Constructor
    /**
     * @brief Default constructor.
     */
    kinova_controller_robust_pybindwrapper() = default;

    /**
     * @brief Constructor with URDF filename, configuration filename, and controller parameters.
     */
    kinova_controller_robust_pybindwrapper(
        const std::string urdf_filename,
        const std::string config_filename,
        const nb_1d_double& Kr,
        const double V_max,
        const double alpha,
        const double r_norm_threshold);

    // Destructor
    /**
     * @brief Destructor.
     */
    ~kinova_controller_robust_pybindwrapper() = default;

    // Class methods
    /**
     * @brief Reset the robot model parameters (motor parameters and model uncertainty).
     */
    void reset_model_parameters(const std::string config_filename);

    /**
     * @brief Reset the controller parameters.
     */
    void reset_controller_parameters(
        const nb_1d_double Kr,
        const double V_max,
        const double alpha,
        const double r_norm_threshold);

    /**
     * @brief Compute the torque given current trajectory and desired trajectory.
     */
    nb::ndarray<nb::numpy, const double> update(
        const nb_1d_double& q_input,  
        const nb_1d_double& q_d_input, 
        const nb_1d_double& qd_input, 
        const nb_1d_double& qd_d_input, 
        const nb_1d_double& qd_dd_input);
};

}; // namespace Kinova
}; // namespace robust_controller

#endif // KINOVA_CONTROLLER_ROBUST_PYBINDWRAPPER_H
