import pinocchio as pin
import numpy as np
from scipy.interpolate import interp1d
from scipy.integrate import solve_ivp
import scipy.io
import pybullet as p
import time
import sys
sys.path.append("/workspaces/kinova_control_docker/src/RobustMovers/build/lib")
import kinova_controller_ID_robust_nanobind as controller_ID_robust
import matplotlib.pyplot as plt

def desired_trajectory(t):
    qd = np.sin(t) * np.ones(7)
    qd_d = np.cos(t) * np.ones(7)
    qd_dd = -np.sin(t) * np.ones(7)
    return qd, qd_d, qd_dd

def integrate(model, ts_sim, x0, controller, a_init, method='RK45'):
    nq = model.nq
    nv = model.nv
    data = model.createData()

    current_a = a_init
    tau_data = []

    def dynamics(t, x):
        nonlocal current_a
        q = x[:nq]
        v = x[nq:]
        
        qd, qd_d, qd_dd = desired_trajectory(t)
        
        tau = controller.update(q, v, current_a , qd, qd_d, qd_dd)
        tau_data.append((t, tau.copy()))  

        current_a = pin.aba(
            model, data, q, v, tau
        )
        
        return np.concatenate([v, current_a])
    
    sol = solve_ivp(
        dynamics, 
        [ts_sim[0], ts_sim[-1]],
        x0, 
        method=method,
        t_eval=ts_sim
    )
    
    position = sol.y[:nq,:].T
    velocity = sol.y[nq:,:].T
    
    return position, velocity, tau_data

# Kinova URDF model
# model_path = "../../../Robots/kinova-gen3/kinova.urdf"
model_path = "/workspaces/kinova_control_docker/src/RobustMovers/Robots/kinova-gen3/kinova.urdf"
# config_path = "../kinova_model_parameters.yaml"
config_path = "/workspaces/kinova_control_docker/src/RobustMovers/Examples/Kinova/kinova_model_parameters.yaml"

mass_model_uncertainty = 0.0
com_model_uncertainty = 0.0
inertia_model_uncertainty = 0.0

if __name__ == "__main__":
    # Load the URDF model
    model = pin.buildModelFromUrdf(model_path)

    # Add uncertainties to the model
    # wrong in the simulation.py the pincochio model is indexed is start from 1 
    for i in range(1, model.nv + 1):
        mass_uncertainty_range = (2 * mass_model_uncertainty * np.random.rand() - mass_model_uncertainty)
        model.inertias[i].mass *= (1 + mass_uncertainty_range)

        com_uncertainty_range = (2 * com_model_uncertainty * np.random.rand() - com_model_uncertainty)
        model.inertias[i].lever *= (1 + com_uncertainty_range)

        inertia_uncertainty_range = (2 * inertia_model_uncertainty * np.random.rand() - inertia_model_uncertainty)
        model.inertias[i].inertia *= (1 + inertia_uncertainty_range)

    # mass_uncertainty_range = (2 * mass_model_uncertainty * np.random.rand() - mass_model_uncertainty)
    # model.inertias[model.nv].mass *= (1 + mass_uncertainty_range)

    # com_uncertainty_range = (2 * com_model_uncertainty * np.random.rand() - com_model_uncertainty)
    # model.inertias[model.nv].lever *= (1 + com_uncertainty_range)

    # inertia_uncertainty_range = (2 * inertia_model_uncertainty * np.random.rand() - inertia_model_uncertainty)
    # model.inertias[model.nv].inertia *= (1 + inertia_uncertainty_range)

 
    
    # # Controller parameters
    Kp = 500.0
    Kd = 2 * np.sqrt(Kp)
    rho_init = 3
    k_rho = 0.1
    dt = 0.001
    epsilon = 0.49
    alpha = 0.7

  

    controller = controller_ID_robust.kinova_controller_ID_robust_pybindwrapper(
        model_path, config_path, Kp,Kd, epsilon, rho_init, k_rho,dt ,alpha
    )
       
    # Set the initial condition
    q0, v0, a_init = desired_trajectory(0)


    # Set the simulation time
    ts_sim = np.linspace(0, 1, 1000)

    # Integrate the dynamics using pinocchio
    pos, vel, tau_data = integrate(model, ts_sim, np.concatenate([q0, v0]), controller,a_init)
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
    
    # plt.figure()
    # for i in range(7):
    #     plt.subplot(2, 4, i+1)
    #     plt.plot(ts_sim, tracking_error[:,i], label=f"Joint {i}")
    #     plt.xlabel("Time [s]")
    #     plt.ylabel("Tracking error [rad]")
    #     plt.legend()
    # plt.savefig("adaptive_tracking_error.png")

    plt.figure(figsize=(12, 6))  # Adjust the figsize as needed (width, height)
    for i in range(7):
        plt.subplot(2, 4, i + 1)
        plt.plot(ts_sim, tracking_error[:, i], label=f"Joint {i}")
        plt.xlabel("Time [s]")
        plt.ylabel("Tracking error [rad]")
        plt.legend()
    plt.tight_layout()  # Adjust spacing between subplots
    plt.savefig("ID_robust_tracking_error.png", dpi=300)  
    # plt.show()

    times = [t for t, _ in tau_data]
    tau_values = np.array([tau for _, tau in tau_data])  

    plt.figure(figsize=(12, 6))  
    for i in range(tau_values.shape[1]):  
        plt.subplot(2, 4, i + 1)
        plt.plot(times, tau_values[:, i], label=f"Joint {i + 1}")
        plt.xlabel("Time [s]")
        plt.ylabel("Torque [Nm]")
        plt.legend()
        plt.grid(True)

    plt.tight_layout() 
    plt.savefig("ID_robust_tau.png", dpi=300)  
    plt.show()  


    
    
    

