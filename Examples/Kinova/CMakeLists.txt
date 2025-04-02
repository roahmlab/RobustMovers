# Add Python bindings
nanobind_add_module(kinova_controller_robust_nanobind 
    NB_SHARED LTO
    ${CMAKE_CURRENT_SOURCE_DIR}/JointSpaceControl/kinova_controller_robust_pybind.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/JointSpaceControl/kinova_controller_robust_pybindwrapper.cpp)
target_include_directories(kinova_controller_robust_nanobind PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/JointSpaceControl)
target_link_libraries(kinova_controller_robust_nanobind PUBLIC
    dynlib
    conlib
    ${BOOST_LIBRARIES} 
    pinocchio::pinocchio
    yaml-cpp
    ${PYTHON_LIBRARIES})
nanobind_compile_options(kinova_controller_robust_nanobind PUBLIC
    ${PINOCCHIO_FLAGS})
set_property(TARGET 
    kinova_controller_robust_nanobind PROPERTY POSITION_INDEPENDENT_CODE ON)