# Minimum required CMake version
cmake_minimum_required(VERSION 3.18)

# Project name
project(RobustMovers
    DESCRIPTION "RobustMovers: robust controllers for robotic manipulators and humanoids")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp")

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Number of threads that openmp will use
if(DEFINED NUM_THREADS)
  add_definitions(-DNUM_THREADS=${NUM_THREADS})
  message(STATUS "Number of threads used in openmp: ${NUM_THREADS}")
else()
    # Get the maximum number of threads using nproc
    execute_process(
        COMMAND nproc
        OUTPUT_VARIABLE MAX_THREADS
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    add_definitions(-DNUM_THREADS=${MAX_THREADS})
    message(STATUS "Maximize number of threads used in openmp: ${MAX_THREADS}")
endif()

# Find Python binding library
find_package(Python 3.10
  REQUIRED COMPONENTS Interpreter Development.Module
  OPTIONAL_COMPONENTS Development.SABIModule)
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

# Find GSL library
find_package(GSL REQUIRED)
include_directories(${GSL_INCLUDE_DIRS})
link_directories(${GSL_LIBRARY_DIRS})

# Find Eigen library
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find Pinocchio library
find_package(pinocchio REQUIRED)
include_directories(${PINOCCHIO_INCLUDE_DIRS})
link_directories(${PINOCCHIO_LIBRARY_DIRS})

# Find Boost library
find_package(Boost REQUIRED COMPONENTS 
    system
    filesystem
    serialization
)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Find YAML library
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
link_directories(${YAML_CPP_LIBRARY_DIR})

# Compile dynamics library
add_library(dynlib SHARED
    Dynamics/src/spatial.cpp
    Dynamics/src/model.cpp
    Dynamics/src/dynamics_Yphi.cpp
    Dynamics/src/dynamics_pinocchio.cpp
    Dynamics/src/operational_space_inverse_kinematics.cpp)

target_include_directories(dynlib PUBLIC
    Dynamics/include)

include_directories(Dynamics/include)

# Compile controllers library
add_library(conlib SHARED
    Controllers/src/controller_passive.cpp
    Controllers/src/controller_robust.cpp)

target_include_directories(conlib PUBLIC
    Controllers/include)

include_directories(Controllers/include)

# Compile examples
add_subdirectory(Examples/Kinova)

# Compile tests
add_executable(Dynamics_test
    Tests/DynamicsTest.cpp)
target_link_libraries(Dynamics_test PUBLIC
    dynlib
    ${BOOST_LIBRARIES} 
    pinocchio::pinocchio
    yaml-cpp)
target_compile_options(Dynamics_test PUBLIC
    ${PINOCCHIO_FLAGS})

add_executable(Regressor_test
    Tests/RegressorTest.cpp)
target_link_libraries(Regressor_test PUBLIC
    dynlib
    ${BOOST_LIBRARIES} 
    pinocchio::pinocchio
    yaml-cpp)
target_compile_options(Regressor_test PUBLIC
    ${PINOCCHIO_FLAGS})

add_executable(ReadRobotInfoTest
    Tests/ReadRobotInfoTest.cpp)
target_link_libraries(ReadRobotInfoTest PUBLIC
    dynlib
    conlib
    ${BOOST_LIBRARIES} 
    pinocchio::pinocchio
    yaml-cpp)
target_compile_options(ReadRobotInfoTest PUBLIC
    ${PINOCCHIO_FLAGS})

# Install
install(TARGETS 
    dynlib
    conlib
    LIBRARY DESTINATION lib)