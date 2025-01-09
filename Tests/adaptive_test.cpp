#include <iostream>
#include <Eigen/Dense>
#include "controller_adaptive.hpp"
using namespace robust_controller;

int main() {
    try {
        // Step 1: Create a mock model with NB joints
        const std::string urdf_filename = "../Robots/kinova-gen3/kinova.urdf";
        std::shared_ptr<model> modelPtr = std::make_shared<model>(urdf_filename);

        // Step 2: Initialize controller parameters
        const int NB = 7;
        controller_adaptive::parameters params;
        params.Kd = 10 * Eigen::VectorXd::Ones(NB);
        params.Kr = 5 * Eigen::VectorXd::Ones(NB);
        params.Gamma = Eigen::MatrixXd::Identity(NB*10, NB*10) * 0.1;
        params.dt = 0.001;

        // Step 3: Create the controller
        robust_controller::controller_adaptive controller(modelPtr, params);

        // Step 4: Define test inputs (joint positions, velocities, and desired trajectories)
        Eigen::VectorXd q = Eigen::VectorXd::Zero(NB);       // Current joint positions
        Eigen::VectorXd q_d = Eigen::VectorXd::Zero(NB);     // Current joint velocities
        Eigen::VectorXd qd = Eigen::VectorXd::Ones(NB);      // Desired joint positions
        Eigen::VectorXd qd_d = Eigen::VectorXd::Ones(NB);    // Desired joint velocities
        Eigen::VectorXd qd_dd = Eigen::VectorXd::Zero(NB);   // Desired joint accelerations

        // Step 5: Call the update function
        Eigen::VectorXd tau = controller.update(q, q_d, qd, qd_d, qd_dd);

        // Step 6: Print the output torques
        std::cout << "Computed torques (tau):\n" << tau << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
