#include "model.hpp"

using namespace robust_controller;

int main() {
    const std::string urdf_filename = "../Robots/kinova-gen3/kinova.urdf";
    const std::string config_filename = "../Examples/Kinova/kinova_model_parameters.yaml";

    std::shared_ptr<model> modelPtr_ = 
        std::make_shared<model>(
            urdf_filename, 
            config_filename);

    modelPtr_->print();

    return 0;
}