import pinocchio as pin
import numpy as np
from scipy.interpolate import interp1d
from scipy.integrate import solve_ivp
import scipy.io
import pybullet as p
import time
import matplotlib.pyplot as plt

from kinova_dynamics import set_position, integrate

import sys
sys.path.append('/workspaces/RobustMovers/build/lib') # be careful about the path
import kinova_controller_armour_nanobind as controller_armour

# Desired trajectory: a simple sinusoidal trajectory
def desired_trajectory(t):
    qd = np.sin(t) * np.ones(7)
    qd_d = np.cos(t) * np.ones(7)
    qd_dd = -np.sin(t) * np.ones(7)
    return qd, qd_d, qd_dd

# Kinova URDF model
model_path = "../../../Robots/kinova-gen3/kinova.urdf"
config_path = "../kinova_model_parameters.yaml"

# Ratio of uncertainties in the inertial parameters
mass_model_uncertainty = 0.05 # 5%
com_model_uncertainty = 0.05 # 5%
inertia_model_uncertainty = 0.05 # 5%

if __name__ == "__main__":
    # Load the URDF model
    model = pin.buildModelFromUrdf(model_path)
    
    # Add uncertainties to the model
    for i in range(model.nv):
        mass_uncertainty_range = (2 * mass_model_uncertainty * np.random.rand() - mass_model_uncertainty)
        model.inertias[i].mass *= (1 + mass_uncertainty_range)
        com_uncertainty_range = (2 * com_model_uncertainty * np.random.rand() - com_model_uncertainty)
        model.inertias[i].lever *= (1 + com_uncertainty_range)
        inertia_uncertainty_range = (2 * inertia_model_uncertainty * np.random.rand() - inertia_model_uncertainty)
        model.inertias[i].inertia *= (1 + inertia_uncertainty_range)
    
    # Controller parameters
    Kr = 10 * np.ones(model.nv)
    V_max = 1e-2
    alpha = 20
    r_norm_threshold = 1e-10
    controller = controller_armour.kinova_controller_armour_pybindwrapper(
        model_path, config_path, Kr, V_max, alpha, r_norm_threshold, "ignore"
    )
    
    # Set the initial condition (to be aligned with the beginning of the desired trajectory)
    q0, v0, _ = desired_trajectory(0)

    # Set the simulation time (4 seconds, 1000 steps for visualization)
    ts_sim = np.linspace(0, 4, 1000)

    # Integrate the dynamics using pinocchio
    pos, vel, tau = integrate(model, ts_sim, np.concatenate([q0, v0]), desired_trajectory, controller)
    
    # Recover the desired trajectory
    desired_pos = np.zeros((len(ts_sim), 7))
    desired_vel = np.zeros((len(ts_sim), 7))
    for i, t in enumerate(ts_sim):
        desired_pos[i], desired_vel[i], _ = desired_trajectory(t)
        
    tracking_error = pos - desired_pos
    tracking_error_vel = vel - desired_vel
    
    # Initialize the PyBullet simulation
    p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    robot = p.loadURDF(model_path, useFixedBase=True, basePosition=[0,0,0], baseOrientation=[0,0,0,1])
    num_joints = p.getNumJoints(robot)
    
    # Playback the simulation
    for q in pos:
        set_position(robot, q)
        time.sleep(0.01)
        
    input("Press Enter to continue...")
    p.disconnect()
    
    # Plot the tracking error
    plt.figure()
    for i in range(7):
        plt.subplot(2, 4, i+1)
        plt.plot(ts_sim, tracking_error[:,i], label=f"Joint {i}")
        plt.xlabel("Time [s]")
        plt.ylabel("Tracking error [rad]")
        plt.legend()
    plt.savefig("tracking_error.png")
    
    # Plot the control input
    plt.figure()
    for i in range(7):
        plt.subplot(2, 4, i+1)
        plt.plot(ts_sim, tau[:,i], label=f"Joint {i}")
        plt.xlabel("Time [s]")
        plt.ylabel("Control input [Nm]")
        plt.legend()
    plt.savefig("control_input.png")

