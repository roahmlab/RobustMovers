# ARMOUR - RobustMovers

A C++ library implementing the robust controller for robotic manipulators introduced in [ARMOUR](https://roahmlab.github.io/armour/) (Section VII).

For more information on ARMOUR (Autonomous Robust Manipulation via Optimization with Uncertainty-aware Reachability), please refer to our [webpage](https://roahmlab.github.io/armour/) and the [paper](https://arxiv.org/abs/2301.13308).

## Features

- Multiple controller implementations:
  - **ARMOUR**
  - Passivity-based controllers
  - Robust interval-based controllers
  - PID controllers with gravity compensation
  - Adaptive controllers
- Python bindings for easy integration with Python-based projects
- Support for Kinova Gen3 robot arm with examples in Python
- Visualization using PyBullet

## Structure

```
RobustMovers/
├── Controllers/         # Controller implementations
├── Dynamics/            # Dynamics computation (RNEA)
├── Examples/            # Usage examples
│   └── Kinova/          # Examples for Kinova Gen3 robot
├── Robots/              # Robot models
│   └── kinova-gen3/     # Kinova Gen3 model and URDF
├── Tests/               # Test files
└── docker/              # Docker configuration
```

## Dependencies

- C++17 compiler (GCC or Clang)
- CMake (>= 3.18)
- Eigen3 (>= 3.3.7)
- Pinocchio (>= 3.1.0)
- Boost (system, filesystem, serialization)
- GSL
- yaml-cpp
- Python 3.10+ (for Python bindings)
- nanobind (for Python bindings)
- PyBullet (for visualization)

## Installation

### Using Docker (Strongly Recommended)

1. Setting up the docker:
    The repository includes a [Dockerfile](docker/Dockerfile) that sets up all required dependencies.

    In Visual Studio Code, simply click `ctrl+shift+P` and search "Dev Containers: Rebuild and Reopen Container".
    It will build the environment automatically for you from the [Dockerfile](docker/Dockerfile) so that you don't need to follow the steps below.

2. Build the project:
   ```bash
   mkdir build && cd build
   cmake ..
   make -j$(nproc)
   ```

### Manual Installation

1. Install the dependencies:
   ```bash
   # Ubuntu/Debian
   sudo apt-get install build-essential cmake libboost-all-dev libgsl-dev libeigen3-dev libyaml-cpp-dev
   
   # Install Pinocchio (follow instructions at https://github.com/stack-of-tasks/pinocchio)
   ```

2. Build the project:
   ```bash
   mkdir build && cd build
   cmake ..
   make -j$(nproc)
   ```

## Examples

You can find complete examples in the `Examples/` directory:

- `Examples/Kinova/JointSpaceControl/`: Joint space control examples for the Kinova Gen3 robot arm

## Authors

[Bohao Zhang](https://cfather.github.io/) (jimzhang@umich.edu): **Current maintainer**, Robust controller implementation in C++.

Jonathan Michaux (jmichaux@umich.edu): Robust controller theory.

Patrick D. Holmes (pdholmes@umich.edu): Robust controller theory.

## License

**ARMOUR** is released under a [GNU license](https://github.com/roahmlab/RobustMovers/blob/main/LICENSE). 
For a list of all code/library dependencies, please check dependency section. 
For a closed-source version of **ARMOUR** for commercial purpose, **please contact the authors**. 

An overview of the theoretical and implementation details has been published in arxiv. 
If you use **ARMOUR** in an academic work, please cite using the following BibTex entry:
```
@article{article,
author = {Michaux, Jonathan and Holmes, Patrick and Zhang, Bohao and Chen, Che and Wang, Baiyue and Sahgal, Shrey and Zhang, Tiancheng and Dey, Sidhartha and Kousik, Shreyas and Vasudevan, Ram},
year = {2023},
month = {01},
pages = {},
title = {Can't Touch This: Real-Time, Safe Motion Planning and Control for Manipulators Under Uncertainty}
doi={10.48550/arXiv.2301.13308}}
```