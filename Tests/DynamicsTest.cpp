#include "dynamics_Yphi.hpp"
#include "dynamics_pinocchio.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"

using namespace robust_controller;

int main() {
    const std::string urdf_filename = "../Robots/kinova-gen3/kinova.urdf";
    std::shared_ptr<model> modelPtr = std::make_shared<model>(urdf_filename);

    MultiBodyDynamicsYphi dynamicsYphi(modelPtr);
    MultiBodyDynamicsPinocchio dynamicsPinocchio(modelPtr);

    std::srand(std::time(nullptr));
    Eigen::VectorXd q = Eigen::VectorXd::Random(modelPtr->NB);
    Eigen::VectorXd q_d = Eigen::VectorXd::Random(modelPtr->NB);
    Eigen::VectorXd q_aux_d = Eigen::VectorXd::Random(modelPtr->NB);
    Eigen::VectorXd q_dd = Eigen::VectorXd::Random(modelPtr->NB);

    dynamicsYphi.rnea(q, q_d, q_d, q_dd, true);
    dynamicsPinocchio.rnea(q, q_d, q_d, q_dd, true);

    std::cout << "Inverse dynamics test:" << std::endl;
    std::cout << (dynamicsYphi.tau - dynamicsPinocchio.tau).transpose() << std::endl;

    // Coriolis matrix may have different values due to different implementations.
    // So plugging in the same q_aux_d may still have different results between
    // the two implementations.
    // But it's fine as long as the Coriolis matrix passed the test.

    // Recover the Coriolis matrix from dynamicsYphi first.
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(modelPtr->NB, modelPtr->NB);
    for (int i = 0; i < modelPtr->NB; i++) {
        q_aux_d.setZero();
        q_aux_d(i) = 1;
        dynamicsYphi.rnea(q, q_d, q_aux_d, Eigen::VectorXd::Zero(modelPtr->NB), false);
        C.col(i) = dynamicsYphi.tau;
    }
    // pinocchio::computeCoriolisMatrix(modelPtr->model_pinocchio, modelPtr->data_pinocchio, q, q_d);
    // const auto& C = modelPtr->data_pinocchio.C;

    // Get d H / d t then
    modelPtr->model_pinocchio.gravity.setZero();
    Eigen::MatrixXd dHdt = Eigen::MatrixXd::Zero(modelPtr->NB, modelPtr->NB);
    Eigen::MatrixXd rnea_partial_dq(modelPtr->NB, modelPtr->NB);
    Eigen::MatrixXd rnea_partial_dv(modelPtr->NB, modelPtr->NB);
    Eigen::MatrixXd rnea_partial_da(modelPtr->NB, modelPtr->NB);
    for (int i = 0; i < modelPtr->NB; i++) { 
        q_dd.setZero();
        q_dd(i) = 1;
        pinocchio::computeRNEADerivatives(
            modelPtr->model_pinocchio, modelPtr->data_pinocchio,
            q, Eigen::VectorXd::Zero(modelPtr->NB), q_dd,
            rnea_partial_dq, rnea_partial_dv, rnea_partial_da);
            dHdt.col(i) = rnea_partial_dq * q_d;
    }

    std::cout << "Coriolis matrix test:" << std::endl;
    std::cout << q_d.transpose() * (dHdt - 2 * C) * q_d << std::endl;
    
    return 0;
}