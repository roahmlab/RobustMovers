#include <nanobind/nanobind.h>
#include "kinova_controller_adaptive_pybindwrapper.hpp"

namespace nb = nanobind;

using namespace robust_controller;
using namespace Kinova;

NB_MODULE(kinova_controller_adaptive_nanobind, m) {
    m.doc() = "nanobind kinova_controller_adaptive_nanobind plugin";

    nb::class_<kinova_controller_adaptive_pybindwrapper>(m, "kinova_controller_adaptive_pybindwrapper")
        .def(nb::init<
            const std::string, const std::string, 
            const kinova_controller_adaptive_pybindwrapper::nb_1d_double&, 
            const kinova_controller_adaptive_pybindwrapper::nb_1d_double&, 
            const kinova_controller_adaptive_pybindwrapper::nb_1d_double&, 
            const double
            >())
        .def("reset_model_parameters", &kinova_controller_adaptive_pybindwrapper::reset_model_parameters)
        .def("update", &kinova_controller_adaptive_pybindwrapper::update)
        .def( "get_parameters",&kinova_controller_adaptive_pybindwrapper::get_parameters);
}