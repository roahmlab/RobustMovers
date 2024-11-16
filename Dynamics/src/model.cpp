#include "model.hpp"

namespace robust_controller {

int convertPinocchioJointType(const std::string& jtype) {
    if (jtype.find('R') != std::string::npos) {
        if (jtype.find('X') != std::string::npos) {
            return 1;
        }
        else if (jtype.find('Y') != std::string::npos) {
            return 2;
        }
        else if (jtype.find('Z') != std::string::npos) {
            return 3;
        }
        else if (jtype.find('U') != std::string::npos) {
            // This is specific to the digit robot
            // There are 4 joints that have "<axis xyz="0 0 -1"/>"
            // But they can not be identified by urdf parser so we manually set them to be of type -3
            std::cerr << "Warning: one joint is set to be of type -3!" << std::endl;
            std::cerr << "         We assume that this would only happen for Digit-v3, but not other robots!" << std::endl;
            return -3;
        }
        else {
            throw std::invalid_argument("convertPinocchioJointType: invalid joint type!");
        }
    }
    else if (jtype.find('P') != std::string::npos) {
        if (jtype.find('X') != std::string::npos) {
            return 4;
        }
        else if (jtype.find('Y') != std::string::npos) {
            return 5;
        }
        else if (jtype.find('Z') != std::string::npos) {
            return 6;
        }
        else {
            throw std::invalid_argument("convertPinocchioJointType: invalid joint type!");
        }
    }
    else {
        throw std::invalid_argument("convertPinocchioJointType: invalid joint type!");
    }
}

model::model(const std::string& urdf_filename,  
             const double phi_eps_input) {
    pinocchio::urdf::buildModel(urdf_filename, model_pinocchio);
    data_pinocchio = pinocchio::Data(model_pinocchio);

    NB = model_pinocchio.nv; // number of joints

    offset = VecX::Zero(NB);
    friction = VecX::Zero(NB);
    friction_eps = VecX::Zero(NB);
    damping = VecX::Zero(NB);
    damping_eps = VecX::Zero(NB);
    transmissionInertia = VecX::Zero(NB);
    transmissionInertia_eps = VecX::Zero(NB);

    jtype.resize(NB);
    Xtree.resize(NB);
    parent.resize(NB);
    phi = VecX::Zero(10 * NB);
    phi_eps = VecX::Constant(10 * NB, phi_eps_input);

    readRobotDynamicsFromPinocchio();
}

model::model(const std::string& urdf_filename,
             const VecX& friction_input,
             const VecX& friction_eps_input,
             const VecX& damping_input,
             const VecX& damping_eps_input,
             const VecX& transmissionInertia_input,
             const VecX& transmissionInertia_eps_input,
             const VecX& offset_input,
             const double phi_eps_input) :
    friction(friction_input),
    friction_eps(friction_eps_input),
    damping(damping_input),
    damping_eps(damping_eps_input),
    transmissionInertia(transmissionInertia_input),
    transmissionInertia_eps(transmissionInertia_eps_input),
    offset(offset_input) {
    pinocchio::urdf::buildModel(urdf_filename, model_pinocchio);
    data_pinocchio = pinocchio::Data(model_pinocchio);

    NB = model_pinocchio.nv; // number of joints

    if (friction_input.size() != NB) {
        throw std::invalid_argument("friction_input.size() != NB");
    }

    if (friction_eps_input.size() != NB) {
        throw std::invalid_argument("friction_eps_input.size() != NB");
    }

    if (damping_input.size() != NB) {
        throw std::invalid_argument("damping_input.size() != NB");
    }

    if (damping_eps_input.size() != NB) {
        throw std::invalid_argument("damping_eps_input.size() != NB");
    }

    if (transmissionInertia_input.size() != NB) {
        throw std::invalid_argument("transmissionInertia_input.size() != NB");
    }

    if (transmissionInertia_eps_input.size() != NB) {
        throw std::invalid_argument("transmissionInertia_eps_input.size() != NB");
    }

    if (offset_input.size() != NB) {
        throw std::invalid_argument("offset_input.size() != NB");
    }

    jtype.resize(NB);
    Xtree.resize(NB);
    parent.resize(NB);
    phi = VecX::Zero(10 * NB);
    phi_eps = VecX::Constant(10 * NB, phi_eps_input);

    readRobotDynamicsFromPinocchio();
}

model::model(const std::string& urdf_filename,
             const std::string& config_filename) {
    pinocchio::urdf::buildModel(urdf_filename, model_pinocchio);

    NB = model_pinocchio.nv; // number of joints

    offset = VecX::Zero(NB);
    friction = VecX::Zero(NB);
    friction_eps = VecX::Zero(NB);
    damping = VecX::Zero(NB);
    damping_eps = VecX::Zero(NB);
    transmissionInertia = VecX::Zero(NB);
    transmissionInertia_eps = VecX::Zero(NB);

    jtype.resize(NB);
    Xtree.resize(NB);
    parent.resize(NB);
    phi.resize(10 * NB);
    phi_eps = VecX::Zero(10 * NB);

    readRobotDynamicsFromPinocchio();

    YAML::Node config;
    
    try {
        config = YAML::LoadFile(config_filename);
    }
    catch (const std::exception& e) {
        throw std::runtime_error("Failed to load the YAML file." + std::string(e.what()));
    }

    const std::string& robot_name = config.begin()->second.as<std::string>();

    try {
        for (auto entry = std::next(config.begin()); entry != config.end(); entry++) {
            const std::string& joint_name = entry->first.as<std::string>();
            const auto& joint_properties = entry->second;

            int joint_id = model_pinocchio.getJointId(joint_name);
            if (joint_id == -1) {
                throw std::runtime_error("Joint " + joint_name + " not found in the URDF model.");
            }

            joint_id = joint_id - 1; // pinocchio joint id starts from 1

            if (joint_properties["motor_parameters"]) {
                const auto& motor_parameters = joint_properties["motor_parameters"];

                friction[joint_id] = motor_parameters["friction"].as<double>();
                damping[joint_id] = motor_parameters["damping"].as<double>();
                transmissionInertia[joint_id] = motor_parameters["transmissionInertia"].as<double>();
                offset[joint_id] = motor_parameters["offset"].as<double>();
            }
            else {
                throw std::runtime_error("Motor dynamics parameters not found for joint " + joint_name);
            }

            if (joint_properties["uncertainty"]) {
                const auto& uncertainty = joint_properties["uncertainty"];

                phi_eps(10 * joint_id + 0) = uncertainty["mass_eps"].as<double>();
                phi_eps.segment(10 * joint_id + 1, 3).setConstant(uncertainty["com_eps"].as<double>());
                phi_eps.segment(10 * joint_id + 4, 6).setConstant(uncertainty["inertia_eps"].as<double>());
                friction_eps[joint_id] = uncertainty["friction_eps"].as<double>();
                damping_eps[joint_id] = uncertainty["damping_eps"].as<double>();
                transmissionInertia_eps[joint_id] = uncertainty["transmissionInertia_eps"].as<double>();
            }
            else {
                throw std::runtime_error("Uncertainty parameters not found for joint " + joint_name);
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        throw std::runtime_error("Failed to parse information from the YAML file.");
    }
}

void model::readRobotDynamicsFromPinocchio() {
    for (int i = 0; i < NB; i++) {
        const int pinocchio_joint_id = i + 1; // the first joint in pinocchio is the root joint
        try {
            jtype[i] = convertPinocchioJointType(model_pinocchio.joints[pinocchio_joint_id].shortname());
        }
        catch (const std::invalid_argument& e) {
            std::cerr << e.what() << std::endl;
            std::cerr << "joint name: " << model_pinocchio.joints[pinocchio_joint_id].shortname() << std::endl;
            throw;
        }

        parent[i] = model_pinocchio.parents[pinocchio_joint_id] - 1;

        // plux in Roy Featherstone's code (transformation matrix from parent to child)
        Xtree[i] = plux(model_pinocchio.jointPlacements[pinocchio_joint_id].rotation().transpose(), 
                        model_pinocchio.jointPlacements[pinocchio_joint_id].translation());

        phi.segment(10 * i, 10) = model_pinocchio.inertias[pinocchio_joint_id].toDynamicParameters();
    }

    a_grav << model_pinocchio.gravity.angular(),
              model_pinocchio.gravity.linear();
}

void model::print() {
    std::cout << "Robot name: " << model_pinocchio.name << std::endl;
    std::cout << "NB: " << NB << std::endl;
    std::cout << "a_grav: " << a_grav.transpose() << std::endl;
    std::cout << "jtype: " << std::endl;
    for (int i = 0; i < NB; i++) {
        std::cout << jtype[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "parent: " << std::endl;
    for (int i = 0; i < NB; i++) {
        std::cout << parent[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Xtree: " << std::endl;
    for (int i = 0; i < NB; i++) {
        std::cout << Xtree[i] << std::endl;
    }

    std::cout << "phi: " << std::endl;
    for (int i = 0; i < NB; i++) {
        std::cout << phi.segment(10 * i, 10).transpose() << std::endl;
    }

    std::cout << "phi_eps: " << phi_eps.transpose() << std::endl;
    std::cout << "friction: " << friction.transpose() << std::endl;
    std::cout << "friction_eps: " << friction_eps.transpose() << std::endl;
    std::cout << "damping: " << damping.transpose() << std::endl;
    std::cout << "damping_eps: " << damping_eps.transpose() << std::endl;
    std::cout << "transmissionInertia: " << transmissionInertia.transpose() << std::endl;
    std::cout << "transmissionInertia_eps: " << transmissionInertia_eps.transpose() << std::endl;
    std::cout << "offset: " << offset.transpose() << std::endl;
}

} // namespace robust_controller
