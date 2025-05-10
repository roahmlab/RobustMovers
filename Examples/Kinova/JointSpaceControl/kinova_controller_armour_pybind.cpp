#include <nanobind/nanobind.h>

#include "kinova_controller_armour_pybindwrapper.hpp"

namespace nb = nanobind;

using namespace robust_controller;
using namespace Kinova;

NB_MODULE(kinova_controller_armour_nanobind, m) {
    m.doc() = "nanobind kinova_controller_armour_nanobind plugin";

    nb::class_<kinova_controller_armour_pybindwrapper>(m, "kinova_controller_armour_pybindwrapper")
        .def(nb::init<
            const std::string, const std::string, 
            const kinova_controller_armour_pybindwrapper::nb_1d_double&, 
            const double, const double, const double,
            const std::string
            >())
        .def("reset_model_parameters", &kinova_controller_armour_pybindwrapper::reset_model_parameters)
        .def("reset_controller_parameters", &kinova_controller_armour_pybindwrapper::reset_controller_parameters)
        .def("update", &kinova_controller_armour_pybindwrapper::update);
}
