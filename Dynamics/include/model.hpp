#ifndef MODEL_HPP
#define MODEL_HPP

#include <yaml-cpp/yaml.h>
#include "spatial.hpp"

namespace robust_controller {

/**
 * @brief Convert Pinocchio joint type string to integer.
 * 
 * This function converts a string representing a Pinocchio joint type to an integer.
 * 
 * @param jtype The string representing the Pinocchio joint type.
 * @return int The integer corresponding to the joint type.
 *         1 = revolute X axis 'Rx'
 *         2 = revolute Y axis 'Ry'
 *         3 = revolute Z axis 'Rz'
 *         -1 = reversed revolute X axis '-Rx'
 *         -2 = reversed revolute Y axis '-Ry'
 *         -3 = reversed revolute Z axis '-Rz'
 *         4 = prismatic X axis 'Px' 
 *         5 = prismatic Y axis 'Py'
 *         6 = prismatic Z axis 'Pz'
 *         -4 = reversed prismatic X axis '-Px'
 *         -5 = reversed prismatic Y axis '-Py'
 *         -6 = reversed prismatic Z axis '-Pz'
 *         0 = fixed joint (actually can not identified from a pinocchio model since it's already merged)
 */
int convertPinocchioJointType(const std::string& jtype);
 
/**
 * @brief Class representing a robot model.
 * 
 * This class represents a robot model, containing various attributes and methods
 * for describing the robot's structure and properties.
 */
class model {
public:
    using VecX = Eigen::VectorXd; //!< Type alias for Eigen dynamic vector.

    int NB = 0; //!< Number of joints.
    Vec6 a_grav; //!< Gravity acceleration vector.
    std::vector<int> jtype; //!< Vector of joint types.
    std::vector<int> parent; //!< Vector of parent joint indices.
    std::vector<Mat6> Xtree; //!< Vector of spatial transform matrices.
    VecX phi; //!< Vector of inertial parameters.
    VecX phi_eps; //!< Vector of uncertainty of inertial parameters.

    // motor dynamics parameters
    VecX friction; //!< Vector of friction parameters.
    VecX friction_eps; //!< Vector of uncertainty of friction parameters.
    VecX damping; //!< Vector of damping parameters.
    VecX damping_eps; //!< Vector of uncertainty of damping parameters.
    VecX transmissionInertia; //!< Vector of transmission inertia parameters.
    VecX transmissionInertia_eps; //!< Vector of uncertainty of transmission inertia parameters.    
    VecX offset; //!< Vector of offset parameters.

    pinocchio::Model model_pinocchio; //!< Pinocchio model.
    pinocchio::Data data_pinocchio; //!< Pinocchio data.

    /**
     * @brief Default constructor.
     */
    model() = default;

    /**
     * @brief Constructor to load model from URDF file.
     * 
     * This constructor initializes the robot model by loading it from a URDF file.
     * 
     * @param urdf_filename The filename of the URDF file.
     * @param phi_eps_input The model uncertainty for each inertial parameter.
     */
    model(
        const std::string& urdf_filename,
        const double phi_eps_input = 0.0);

    /**
     * @brief Constructor to load model from URDF file with additional parameters.
     * 
     * This constructor initializes the robot model by loading it from a URDF file,
     * with additional parameters for friction, damping, and transmission inertia.
     * 
     * @param urdf_filename The filename of the URDF file.
     * @param friction_input Vector of friction parameters.
     * @param friction_eps_input Vector of friction uncertainty parameters.
     * @param damping_input Vector of damping parameters.
     * @param damping_eps_input Vector of damping uncertainty parameters.
     * @param transmissionInertia_input Vector of transmission inertia parameters.
     * @param transmissionInertia_eps_input Vector of transmission inertia uncertainty parameters.
     * @param offset_input Vector of offset parameters.
     * @param phi_eps_input The model uncertainty for each inertial parameter.
     */
    model(
        const std::string& urdf_filename,
        const VecX& friction_input,
        const VecX& friction_eps_input,
        const VecX& damping_input,
        const VecX& damping_eps_input,
        const VecX& transmissionInertia_input,
        const VecX& transmissionInertia_eps_input,
        const VecX& offset_input,
        const double phi_eps_input = 0.0);

    /**
     * @brief Constructor.
     * 
     * This constructor initializes the robot model by loading it from a URDF file and a configuration file.
     * 
     * @param urdf_filename The filename of the URDF file.
     * @param config_filename The filename of the configuration file.
     */
    model(
        const std::string& urdf_filename,
        const std::string& config_filename);

    void readRobotDynamicsFromPinocchio();

    void print();

    /**
     * @brief Destructor.
     */
    virtual ~model() = default;
};

}; // namespace robust_controller

#endif // MODEL_HPP
