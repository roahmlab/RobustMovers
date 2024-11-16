#include "pinocchio/algorithm/regressor.hpp"
#include "dynamics_Yphi.hpp"
#include <iostream>

using namespace robust_controller;

int main() {
    const std::string urdf_filename = "../Robots/kinova-gen3/kinova.urdf";
    std::shared_ptr<model> modelPtr = std::make_shared<model>(urdf_filename);

    MultiBodyDynamicsYphi dynamicsYphi(modelPtr);

    std::srand(std::time(nullptr));
    Eigen::VectorXd q = Eigen::VectorXd::Random(modelPtr->NB);
    Eigen::VectorXd q_d = Eigen::VectorXd::Random(modelPtr->NB);
    Eigen::VectorXd q_dd = Eigen::VectorXd::Random(modelPtr->NB);

    std::cout << "q: " << q.transpose() << std::endl;
    std::cout << "q_d: " << q_d.transpose() << std::endl;
    std::cout << "q_dd: " << q_dd.transpose() << std::endl;

    dynamicsYphi.rnea(q, q_d, q_d, q_dd);
    pinocchio::computeJointTorqueRegressor(modelPtr->model_pinocchio, modelPtr->data_pinocchio, q, q_d, q_dd);

    // std::cout << dynamicsYphi.Y.transpose() << std::endl << std::endl;
    // std::cout << modelPtr->data_pinocchio.jointTorqueRegressor.transpose() << std::endl << std::endl;

    for (int i = 0; i < modelPtr->NB; i++) {
        const Eigen::MatrixXd& Yblock = 
            dynamicsYphi.Y.middleCols(10 * i, 10);
        const Eigen::MatrixXd& Yblock_pinocchio = 
            modelPtr->data_pinocchio.jointTorqueRegressor.middleCols(10 * i, 10);

        std::cout << "Link " << i + 1 << ":" << std::endl;

        // mass
        std::cout << "  mass cols:\n";
        std::cout << "    " << (Yblock.col(0) - Yblock_pinocchio.col(0)).transpose() << std::endl;

        // com
        std::cout << "  com cols:\n";
        for (int j = 0; j < 3; j++) {
            std::cout << "    " << (Yblock.col(1 + j) - Yblock_pinocchio.col(1 + j)).transpose() << std::endl;
        }

        // inertia
        std::cout << "  inertia cols:\n";
        for (int j = 0; j < 6; j++) {
            std::cout << "    " << (Yblock.col(4 + j) - Yblock_pinocchio.col(4 + j)).transpose() << std::endl;
        }
    }

    return 0;
}