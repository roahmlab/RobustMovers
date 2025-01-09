import pinocchio as pin
import numpy as np
from scipy.interpolate import interp1d
from scipy.integrate import solve_ivp
import scipy.io
import pybullet as p
import time
import sys
sys.path.append("/workspaces/kinova_control_docker/src/RobustMovers/build/lib")
import kinova_controller_robust_nanobind as controller_robust
import matplotlib.pyplot as plt

def desired_trajectory(t):
    qd = np.sin(t) * np.ones(7)
    qd_d = np.cos(t) * np.ones(7)
    qd_dd = -np.sin(t) * np.ones(7)
    return qd, qd_d, qd_dd

def integrate(model, ts_sim, x0, controller, method='RK45'):
    nq = model.nq
    nv = model.nv
    data = model.createData()

    def dynamics(t, x):
        q = x[:nq]
        v = x[nq:]
        
        qd, qd_d, qd_dd = desired_trajectory(t)
        
        tau = controller.update(q, v, qd, qd_d, qd_dd)
        print(tau)
        
        a = pin.aba(
            model, data, q, v, tau
        )
        print(a)
        
        return np.concatenate([v, a])
    
    sol = solve_ivp(
        dynamics, 
        [ts_sim[0], ts_sim[-1]],
        x0, 
        method=method,
        t_eval=ts_sim
    )
    
    position = sol.y[:nq,:].T
    velocity = sol.y[nq:,:].T
    
    return position, velocity

# Kinova URDF model
model_path = "../../../Robots/kinova-gen3/kinova.urdf"
config_path = "../kinova_model_parameters.yaml"

mass_model_uncertainty = 0.05
com_model_uncertainty = 0.05
inertia_model_uncertainty = 0.05

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
    controller = controller_robust.kinova_controller_robust_pybindwrapper(
        model_path, config_path, Kr, V_max, alpha, r_norm_threshold
    )
    
    # Set the initial condition
    q0, v0, _ = desired_trajectory(0)

    # Set the simulation time
    ts_sim = np.linspace(0, 4, 1000)

    # Integrate the dynamics using pinocchio
    pos, vel = integrate(model, ts_sim, np.concatenate([q0, v0]), controller)
    
    # Recover the desired trajectory
    desired_pos = np.zeros((len(ts_sim), 7))
    desired_vel = np.zeros((len(ts_sim), 7))
    for i, t in enumerate(ts_sim):
        desired_pos[i], desired_vel[i], _ = desired_trajectory(t)
        
    tracking_error = pos - desired_pos
    tracking_error_vel = vel - desired_vel
    
    # Initialize the PyBullet simulation
    p.connect(p.GUI)
    # p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    robot = p.loadURDF(model_path, useFixedBase=True, basePosition=[0,0,0], baseOrientation=[0,0,0,1])
    num_joints = p.getNumJoints(robot)
    
    # Playback the simulation
    for q in pos:
        id = 0
        for i in range(num_joints):
            joint_info = p.getJointInfo(robot, i)
            joint_type = joint_info[2]
            if joint_type == p.JOINT_FIXED:
                p.resetJointState(robot, i, targetValue=0)
            else:
                p.resetJointState(robot, i, targetValue=q[id])
                id += 1
        time.sleep(0.01)
        
    input("Press Enter to continue...")
    p.disconnect()
    
    plt.figure()
    for i in range(7):
        plt.subplot(2, 4, i+1)
        plt.plot(ts_sim, tracking_error[:,i], label=f"Joint {i}")
        plt.xlabel("Time [s]")
        plt.ylabel("Tracking error [rad]")
        plt.legend()
    plt.savefig("tracking_error.png")
    
    

