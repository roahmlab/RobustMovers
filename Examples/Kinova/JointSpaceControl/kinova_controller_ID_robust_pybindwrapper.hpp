#ifndef KINOVA_CONTROLLER_ID_ROBUST_PYBINDWRAPPER_H
#define KINOVA_CONTROLLER_ID_ROBUST_PYBINDWRAPPER_H

#
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include "controller_ID_robust.hpp"        
#include "kinova_constants.hpp" 

#include "pinocchio/parsers/urdf.hpp"   
#include "pinocchio/algorithm/joint-configuration.hpp"

namespace robust_controller {
namespace Kinova {

namespace nb = nanobind;

class kinova_controller_ID_robust_pybindwrapper {
public:
    using VecX          = Eigen::VectorXd;                  
    using MatX          = Eigen::MatrixXd;                  
    using nb_1d_double = nb::ndarray<double, nb::ndim<1>, nb::c_contig, nb::device::cpu>; //!< 1D numpy array
    using nb_2d_double = nb::ndarray<double, nb::ndim<2>, nb::c_contig, nb::device::cpu>; //!< 2D numpy array

  
    kinova_controller_ID_robust_pybindwrapper() = default;

    kinova_controller_ID_robust_pybindwrapper(
        const std::string urdf_filename,
        const std::string config_filename,
        const double Kp,
        const double Kd,
        const double epsilon,
        const double rho_init,
        const double k_rho,
        const double dt,
        const double alpha);
   
    ~kinova_controller_ID_robust_pybindwrapper() = default;
 
    void reset_model_parameters(const std::string config_filename);

    void reset_controller_parameters(
        const nb_1d_double Kr,
        const double V_max,
        const double alpha,
        const double r_norm_threshold);


    nb::ndarray<nb::numpy, const double> update(
        const nb_1d_double& q_input,  
        const nb_1d_double& q_d_input, 
        const nb_1d_double& q_dd_input,
        const nb_1d_double& qd_input, 
        const nb_1d_double& qd_d_input, 
        const nb_1d_double& qd_dd_input);

    std::string urdf_filename_copy; 

    std::shared_ptr<model> modelPtr_;                      
    controller_ID_robust::parameters params;               
    std::shared_ptr<controller_ID_robust> controllerPtr_;     

    VecX q;    
    VecX q_d;  
    VecX q_dd;  

    VecX qd;   
    VecX qd_d;  
    VecX qd_dd; 

    VecX tau;   
};

} // namespace Kinova
} // namespace robust_controller

#endif // KINOVA_CONTROLLER_ROBUST_PYBINDWRAPPER_H
